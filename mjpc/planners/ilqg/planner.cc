// Copyright 2022 DeepMind Technologies Limited
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "mjpc/planners/ilqg/planner.h"

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdio>
#include <iostream>
#include <shared_mutex>

#include <mujoco/mujoco.h>
#include "mjpc/array_safety.h"
#include "mjpc/planners/ilqg/backward_pass.h"
#include "mjpc/planners/ilqg/policy.h"
#include "mjpc/planners/ilqg/settings.h"
#include "mjpc/planners/planner.h"
#include "mjpc/states/state.h"
#include "mjpc/task.h"
#include "mjpc/threadpool.h"
#include "mjpc/trajectory.h"
#include "mjpc/utilities.h"

namespace mjpc {
namespace mju = ::mujoco::util_mjpc;

// initialize data and settings
void iLQGPlanner::Initialize(mjModel* model, const Task& task) {
  // delete mjData instances since model might have changed.
  data_.clear();

  // allocate one mjData for nominal.
  ResizeMjData(model, 1);

  // model
  this->model = model;

  // task
  this->task = &task;

  // dimensions
  dim_state = model->nq + model->nv + model->na;  // state dimension
  dim_state_derivative =
      2 * model->nv + model->na;    // state derivative dimension
  dim_action = model->nu;           // action dimension
  dim_sensor = model->nsensordata;  // number of sensor values
  dim_max =
      mju_max(mju_max(mju_max(dim_state, dim_state_derivative), dim_action),
              model->nuser_sensor);
  num_rollouts_gui_ = GetNumberOrDefault(10, model, "ilqg_num_rollouts");
  settings.regularization_type = GetNumberOrDefault(
      settings.regularization_type, model, "ilqg_regularization_type");
}

// allocate memory
void iLQGPlanner::Allocate() {
  // state
  state.resize(model->nq + model->nv + model->na);
  mocap.resize(7 * model->nmocap);
  userdata.resize(model->nuserdata);

  // Keypoint thresholds
  jerk_thresholds.resize(model->nv);
  velocity_change_thresholds.resize(model->nv);


  // candidate trajectories
  for (int i = 0; i < kMaxTrajectory; i++) {
    trajectory[i].Initialize(dim_state, dim_action, task->num_residual,
                             task->num_trace, kMaxTrajectoryHorizon);
    trajectory[i].Allocate(kMaxTrajectoryHorizon);
  }

  // model derivatives
  model_derivative.Allocate(dim_state_derivative, dim_action, dim_sensor,
                            kMaxTrajectoryHorizon);

  // costs derivatives
  cost_derivative.Allocate(dim_state_derivative, dim_action, task->num_residual,
                           kMaxTrajectoryHorizon, dim_max);

  // backward pass
  backward_pass.Allocate(dim_state_derivative, dim_action,
                         kMaxTrajectoryHorizon);

  // policy
  policy.Allocate(model, *task, kMaxTrajectoryHorizon);
  previous_policy.Allocate(model, *task, kMaxTrajectoryHorizon);
  for (int i = 0; i < kMaxTrajectory; i++) {
    candidate_policy[i].Allocate(model, *task, kMaxTrajectoryHorizon);
  }

  // ----- boxQP ----- //
  boxqp.Allocate(dim_action);
}

// reset memory to zeros
void iLQGPlanner::Reset(int horizon, const double* initial_repeated_action) {
  // state
  std::fill(state.begin(), state.end(), 0.0);
  std::fill(mocap.begin(), mocap.end(), 0.0);
  std::fill(userdata.begin(), userdata.end(), 0.0);
  time = 0.0;

  // model derivatives
  model_derivative.Reset(dim_state_derivative, dim_action, dim_sensor, horizon);

  // cost derivatives
  cost_derivative.Reset(dim_state_derivative, dim_action, task->num_residual,
                        horizon);

  // backward pass
  backward_pass.Reset(dim_state_derivative, dim_action, horizon);

  // policy
  policy.Reset(horizon, initial_repeated_action);
  previous_policy.Reset(horizon, initial_repeated_action);
  for (int i = 0; i < kMaxTrajectory; i++) {
    candidate_policy[i].Reset(horizon, initial_repeated_action);
  }

  // candidate trajectories
  for (int i = 0; i < kMaxTrajectory; i++) {
    trajectory[i].Reset(horizon, initial_repeated_action);
  }

  // values
  action_step = 0.0;
  feedback_scaling = 0.0;
  improvement = 0.0;
  expected = 0.0;
  surprise = 0.0;
}

// set state
void iLQGPlanner::SetState(const State& state) {
  state.CopyTo(this->state.data(), this->mocap.data(), this->userdata.data(),
               &this->time);
}

void iLQGPlanner::UpdateNumTrajectoriesFromGUI() {
  num_trajectory_ = mju_min(num_rollouts_gui_, kMaxTrajectory);
}

// optimize nominal policy using iLQG
void iLQGPlanner::OptimizePolicy(int horizon, ThreadPool& pool) {
  // freeze the GUI value once per optimization step.
  UpdateNumTrajectoriesFromGUI();
  // get nominal trajectory
  this->NominalTrajectory(horizon, pool);

  // iteration
  this->Iteration(horizon, pool);
}

// compute trajectory using nominal policy
void iLQGPlanner::NominalTrajectory(int horizon, ThreadPool& pool) {
  if (num_trajectory_ == 0) {
    return;
  }
  // resize data for rollouts
  ResizeMjData(model, pool.NumThreads());

  // step sizes (log scaling)
  LogScale(linesearch_steps, 1.0, settings.min_linesearch_step,
           num_trajectory_ - 1);
  linesearch_steps[num_trajectory_ - 1] = 0.0;

  // ----- nominal rollout ----- //
  // start timer
  auto nominal_start = std::chrono::steady_clock::now();

  // no one else should be writing, but we lock just in case:
  {
    const std::shared_lock<std::shared_mutex> lock(mtx_);
    for (int i = 0; i < num_trajectory_; i++) {
      candidate_policy[i].CopyFrom(policy, horizon);
      candidate_policy[i].representation = policy.representation;
    }
  }

  // feedback rollouts (parallel)
  this->FeedbackRollouts(horizon, pool);

  // evaluate rollouts
  int best_nominal = this->BestRollout();

  // check for all rollout failures
  if (best_nominal == -1) {
    // The policy's traj's first state is not the correct initial state, but
    // what alternative do we have, if all rollouts from the current initial
    // state failed?
    {
      const std::shared_lock<std::shared_mutex> lock(mtx_);
      candidate_policy[0].trajectory = policy.trajectory;
    }

    // set feedback scaling
    feedback_scaling = 0.0;
  } else {
    // update nominal with winner
    candidate_policy[0].trajectory = trajectory[best_nominal];

    // set feedback scaling
    feedback_scaling = linesearch_steps[best_nominal];
  }

  // end timer
  nominal_compute_time = GetDuration(nominal_start);
}

// set action from policy
void iLQGPlanner::ActionFromPolicy(double* action, const double* state,
                                   double time, bool use_previous) {
  const std::shared_lock<std::shared_mutex> lock(mtx_);
  if (use_previous) {
    previous_policy.Action(action, state, time);
  } else {
    policy.Action(action, state, time);
  }
}

// return trajectory with best total return
const Trajectory* iLQGPlanner::BestTrajectory() {
  const std::shared_lock<std::shared_mutex> lock(mtx_);
  return &policy.trajectory;
}

// visualize planner-specific traces in GUI
void iLQGPlanner::Traces(mjvScene* scn) {}

// planner-specific GUI elements
void iLQGPlanner::GUI(mjUI& ui) {
  mjuiDef defiLQG[] = {
      {mjITEM_SLIDERINT, "Rollouts", 2, &num_rollouts_gui_, "0 1"},
      // {mjITEM_RADIO, "Action Lmt.", 2, &settings.action_limits, "Off\nOn"},
      {mjITEM_SELECT, "Policy Interp.", 2, &policy.representation,
       "Zero\nLinear\nCubic"},
      {mjITEM_SELECT, "Reg. Type", 2, &settings.regularization_type,
       "Control\nFeedback\nValue\nNone"},
      {mjITEM_CHECKINT, "Terminal Print", 2, &settings.verbose, ""},
      {mjITEM_END}};

  // set number of trajectory slider limits
  mju::sprintf_arr(defiLQG[0].other, "%i %i", 1, kMaxTrajectory);

  // add iLQG planner
  mjui_add(&ui, defiLQG);

  // TODO(DMackRus) - better choice for this, max degrees of freedom??
  mjuiDef defKeypoints[kMaxCostTerms + 6];

  defKeypoints[0] = {mjITEM_SEPARATOR, "Keypoints", 1};
  defKeypoints[1] = {mjITEM_CHECKINT, "keypoints", 2, &use_keypoints, ""};
  defKeypoints[2] = {mjITEM_SELECT, "Keypoint method", 2, &active_keypoint_method,
                    "Set Interval\nAdaptive Jerk\nVelocity Change"},
  defKeypoints[3] = {mjITEM_SLIDERINT, "min. interval", 2, &min_n, "1 16"};
  defKeypoints[4] = {mjITEM_SLIDERINT, "max. interval", 2, &max_n, "1 16"};

  for(int i = 0; i < model->nv; i++){
    defKeypoints[5 + i] = {mjITEM_SLIDERNUM, "Jerk threshold",
                         1, DataAt(jerk_thresholds, i), "0 1"};

    // Set names of sliders for jerk thresholds
    const char* prefix = "Jerk: ";
    int len1 = strlen(prefix);
    int len2 = strlen(model->names + model->name_jntadr[i]);

    char* result = new char[len1 + len2 + 1];

    // Copy the first string into the result buffer
    strcpy(result, prefix);

    // Concatenate the second string to the result buffer
    strcat(result, model->names + model->name_jntadr[i]);

    mju::strcpy_arr(defKeypoints[5 + i].name,
                      result);

    // limits
//    double* s = model_->sensor_user + i * model_->nuser_sensor;
//    mju::sprintf_arr(defNormWeight[i].other, "%f %f", s[2], s[3]);

  }

  defKeypoints[6 + model->nv] = {mjITEM_END};

  mjui_add(&ui, defKeypoints);

}

// planner-specific plots
void iLQGPlanner::Plots(mjvFigure* fig_planner, mjvFigure* fig_timer,
                        int planner_shift, int timer_shift, int planning,
                        int* shift) {
  // bounds
  double planner_bounds[2] = {-6, 6};

  // ----- planner ----- //

  // regularization
  mjpc::PlotUpdateData(fig_planner, planner_bounds,
                       fig_planner->linedata[0 + planner_shift][0] + 1,
                       mju_log10(mju_max(backward_pass.regularization, 1.0e-6)),
                       100, 0 + planner_shift, 0, 1, -100);

  // action step size
  mjpc::PlotUpdateData(fig_planner, planner_bounds,
                       fig_planner->linedata[1 + planner_shift][0] + 1,
                       mju_log10(mju_max(action_step, 1.0e-6)), 100,
                       1 + planner_shift, 0, 1, -100);

  // feedback scaling
  mjpc::PlotUpdateData(fig_planner, planner_bounds,
                       fig_planner->linedata[2 + planner_shift][0] + 1,
                       mju_log10(mju_max(feedback_scaling, 1.0e-6)), 100,
                       2 + planner_shift, 0, 1, -100);

  // improvement
  // mjpc::PlotUpdateData(
  //     fig_planner, planner_bounds, fig_planner->linedata[3 +
  //     planner_shift][0] + 1, mju_log10(mju_max(improvement, 1.0e-6)), 100, 3
  //     + planner_shift, 0, 1, -100);

  // // expected
  // mjpc::PlotUpdateData(
  //     fig_planner, planner_bounds, fig_planner->linedata[4 +
  //     planner_shift][0] + 1, mju_log10(mju_max(expected, 1.0e-6)), 100, 4 +
  //     planner_shift, 0, 1, -100);

  // // surprise
  // mjpc::PlotUpdateData(
  //     fig_planner, planner_bounds, fig_planner->linedata[5 +
  //     planner_shift][0] + 1, mju_log10(mju_max(surprise, 1.0e-6)), 100, 5 +
  //     planner_shift, 0, 1, -100);

  // legend
  mju::strcpy_arr(fig_planner->linename[0 + planner_shift], "Regularization");
  mju::strcpy_arr(fig_planner->linename[1 + planner_shift], "Action Step");
  mju::strcpy_arr(fig_planner->linename[2 + planner_shift], "Feedback Scaling");
  // mju::strcpy_arr(fig_planner->linename[3 + planner_shift], "Improvement");
  // mju::strcpy_arr(fig_planner->linename[4 + planner_shift], "Expected");
  // mju::strcpy_arr(fig_planner->linename[5 + planner_shift], "Surprise");

  // ranges
  fig_planner->range[1][0] = planner_bounds[0];
  fig_planner->range[1][1] = planner_bounds[1];

  // ----- timer ----- //
  double timer_bounds[2] = {0, 1};

  // update plots
  PlotUpdateData(fig_timer, timer_bounds,
                 fig_timer->linedata[0 + timer_shift][0] + 1,
                 1.0e-3 * nominal_compute_time * planning, 100, 0 + timer_shift,
                 0, 1, -100);

  PlotUpdateData(fig_timer, timer_bounds,
                 fig_timer->linedata[1 + timer_shift][0] + 1,
                 1.0e-3 * model_derivative_compute_time * planning, 100,
                 1 + timer_shift, 0, 1, -100);

  PlotUpdateData(fig_timer, timer_bounds,
                 fig_timer->linedata[2 + timer_shift][0] + 1,
                 1.0e-3 * cost_derivative_compute_time * planning, 100,
                 2 + timer_shift, 0, 1, -100);

  PlotUpdateData(fig_timer, timer_bounds,
                 fig_timer->linedata[3 + timer_shift][0] + 1,
                 1.0e-3 * backward_pass_compute_time * planning, 100,
                 3 + timer_shift, 0, 1, -100);

  PlotUpdateData(fig_timer, timer_bounds,
                 fig_timer->linedata[4 + timer_shift][0] + 1,
                 1.0e-3 * rollouts_compute_time * planning, 100,
                 4 + timer_shift, 0, 1, -100);

  PlotUpdateData(fig_timer, timer_bounds,
                 fig_timer->linedata[5 + timer_shift][0] + 1,
                 1.0e-3 * policy_update_compute_time * planning, 100,
                 5 + timer_shift, 0, 1, -100);

  // legend
  mju::strcpy_arr(fig_timer->linename[0 + timer_shift], "Nominal");
  mju::strcpy_arr(fig_timer->linename[1 + timer_shift], "Model Deriv.");
  mju::strcpy_arr(fig_timer->linename[2 + timer_shift], "Cost Deriv.");
  mju::strcpy_arr(fig_timer->linename[3 + timer_shift], "Backward Pass");
  mju::strcpy_arr(fig_timer->linename[4 + timer_shift], "Rollouts");
  mju::strcpy_arr(fig_timer->linename[5 + timer_shift], "Policy Update");

  fig_timer->range[0][0] = -100;
  fig_timer->range[0][1] = 0;
  fig_timer->range[1][0] = 0.0;
  fig_timer->range[1][1] = timer_bounds[1];

  // planner shift
  shift[0] += 3;

  // timer shift
  shift[1] += 6;
}

// single iLQG iteration
void iLQGPlanner::Iteration(int horizon, ThreadPool& pool) {
  // set previous best cost
  double previous_return = candidate_policy[0].trajectory.total_return;

  // ----- setup ----- //
  // resize data for rollouts
  ResizeMjData(model, pool.NumThreads());

  // step sizes (log scaling)
  LogScale(linesearch_steps, 1.0, settings.min_linesearch_step,
           num_trajectory_ - 1);
  linesearch_steps[num_trajectory_ - 1] = 0.0;

  // ----- model derivatives ----- //
  // start timer
  auto model_derivative_start = std::chrono::steady_clock::now();

  if(use_keypoints){
      keypoint_method active_method;

      active_method.min_N = min_n;
      active_method.max_N = max_n;

      active_method.method = active_keypoint_method;

      if(active_keypoint_method == ADAPTIVE_JERK){

          // Set jerk thresholds from GUI
          for(int i = 0; i < model->nv; i++){
              active_method.jerk_thresholds.push_back(jerk_thresholds[i]);
          }

      }
      else if(active_keypoint_method == VELOCITY_CHANGE){
          // Set velocity change thresholds from GUI
          for(int i = 0; i < model->nv; i++){
              // TODO: Add velocity change thresholds to GUI
              active_method.velocity_change_thresholds.push_back(jerk_thresholds[i]);
          }
      }

      // Generate keypoints
      std::vector<std::vector<int>> keypoints = key_point_generator.GenerateKeyPoints(active_method,
                                                                                      horizon,
                                                                                      candidate_policy[0].trajectory.states.data(),
                                                                                      dim_state);

      // Compute derivatives
      // time this
      auto time_fd_start = std::chrono::steady_clock::now();
      model_derivative.ComputeKeypoints(
              model, data_, candidate_policy[0].trajectory.states.data(),
              candidate_policy[0].trajectory.actions.data(),
              candidate_policy[0].trajectory.times.data(), dim_state,
              dim_state_derivative, dim_action, dim_sensor, horizon,
              settings.fd_tolerance, settings.fd_mode, pool, keypoints);

        // end timer
        time_fd = GetDuration(time_fd_start);

      // Interpolate derivatives
      // time this
        auto time_start_inter = std::chrono::steady_clock::now();
      key_point_generator.InterpolateDerivatives(keypoints,
                                                 model_derivative.AT,
                                                 model_derivative.BT,
                                                 model_derivative.CT,
                                                 model_derivative.DT,
                                                 dim_state, dim_action, dim_sensor, horizon);

        // end timer
        key_point_generator.time_interpolate = GetDuration(time_start_inter);


      //time this
      auto time_start = std::chrono::steady_clock::now();

//      for(int t = 0; t < horizon; t++){
//          if(t != horizon - 1){
//              mju_transpose(DataAt(model_derivative.AT, t * (dim_state_derivative * dim_state_derivative)),
//                            DataAt(model_derivative.A, t * (dim_state_derivative * dim_state_derivative)),
//                            dim_state_derivative, dim_state_derivative);
//
//              mju_transpose(DataAt(model_derivative.BT, t * (dim_state_derivative * dim_action)),
//                            DataAt(model_derivative.B, t * (dim_state_derivative * dim_action)),
//                            dim_action, dim_state_derivative);
//
//              mju_transpose(DataAt(model_derivative.DT, t * (dim_sensor * dim_action)),
//                            DataAt(model_derivative.D, t * (dim_sensor * dim_action)),
//                            dim_action, dim_sensor);
//          }
//
//          mju_transpose(DataAt(model_derivative.CT, t * (dim_sensor * dim_state_derivative)),
//                        DataAt(model_derivative.C, t * (dim_sensor * dim_state_derivative)),
//                        dim_state_derivative, dim_sensor);
//      }

      for(int t = 0; t < horizon; t++){
          if(t != horizon - 1){
              mju_transpose(DataAt(model_derivative.A, t * (dim_state_derivative * dim_state_derivative)),
                            DataAt(model_derivative.AT, t * (dim_state_derivative * dim_state_derivative)),
                            dim_state_derivative, dim_state_derivative);

              mju_transpose(DataAt(model_derivative.B, t * (dim_state_derivative * dim_action)),
                            DataAt(model_derivative.BT, t * (dim_state_derivative * dim_action)),
                            dim_action, dim_state_derivative);

              mju_transpose(DataAt(model_derivative.D, t * (dim_sensor * dim_action)),
                            DataAt(model_derivative.DT, t * (dim_sensor * dim_action)),
                            dim_action, dim_sensor);
          }

          mju_transpose(DataAt(model_derivative.C, t * (dim_sensor * dim_state_derivative)),
                        DataAt(model_derivative.CT, t * (dim_sensor * dim_state_derivative)),
                        dim_state_derivative, dim_sensor);
      }

      // Copy matrices back into A, B, C and D
//      mju_copy(model_derivative.A.data(), model_derivative.AT.data(), model_derivative.A.size());
//      mju_copy(model_derivative.B.data(), model_derivative.BT.data(), model_derivative.B.size());
//      mju_copy(model_derivative.C.data(), model_derivative.CT.data(), model_derivative.C.size());
//      mju_copy(model_derivative.D.data(), model_derivative.DT.data(), model_derivative.D.size());

        // end timer
        time_transpose = GetDuration(time_start);

  }
  else{
      // Compute dynamics derivatives
      model_derivative.Compute(
              model, data_, candidate_policy[0].trajectory.states.data(),
              candidate_policy[0].trajectory.actions.data(),
              candidate_policy[0].trajectory.times.data(), dim_state,
              dim_state_derivative, dim_action, dim_sensor, horizon,
              settings.fd_tolerance, settings.fd_mode, pool);
  }

  // stop timer
  double model_derivative_time = GetDuration(model_derivative_start);

  // ----- cost derivatives ----- //
  // start timer
  auto cost_derivative_start = std::chrono::steady_clock::now();

  // cost derivatives
  cost_derivative.Compute(
      candidate_policy[0].trajectory.residual.data(), model_derivative.C.data(),
      model_derivative.D.data(), dim_state_derivative, dim_action, dim_max,
      dim_sensor, task->num_residual, task->dim_norm_residual.data(),
      task->num_term, task->weight.data(), task->norm.data(),
      task->norm_parameter.data(), task->num_norm_parameter.data(), task->risk,
      horizon, pool);

  // end timer
  double cost_derivative_time = GetDuration(cost_derivative_start);

  // ----- backward pass ----- //
  // start timer
  auto backward_pass_start = std::chrono::steady_clock::now();

  // initialize backward pass
  int regularization_iteration = 0;
  int backward_pass_status = 0;
  int t;
  while (regularization_iteration < settings.max_regularization_iterations &&
         backward_pass_status == 0) {
    // reset cost-to-go approximation difference
    mju_zero(backward_pass.dV, 2);

    // terminal time step cost-to-go
    mju_copy(DataAt(backward_pass.Vx, (horizon - 1) * dim_state_derivative),
             DataAt(cost_derivative.cx, (horizon - 1) * dim_state_derivative),
             dim_state_derivative);
    mju_copy(DataAt(backward_pass.Vxx, (horizon - 1) * dim_state_derivative *
                                           dim_state_derivative),
             DataAt(cost_derivative.cxx, (horizon - 1) * dim_state_derivative *
                                             dim_state_derivative),
             dim_state_derivative * dim_state_derivative);

    // backward recursion
    for (t = horizon - 2; t >= 0; t--) {
      int status = backward_pass.RiccatiStep(
          dim_state_derivative, dim_action, backward_pass.regularization,
          DataAt(backward_pass.Vx, (t + 1) * dim_state_derivative),
          DataAt(backward_pass.Vxx,
                 (t + 1) * dim_state_derivative * dim_state_derivative),
          DataAt(model_derivative.A,
                 t * dim_state_derivative * dim_state_derivative),
          DataAt(model_derivative.B, t * dim_state_derivative * dim_action),
          DataAt(cost_derivative.cx, t * dim_state_derivative),
          DataAt(cost_derivative.cu, t * dim_action),
          DataAt(cost_derivative.cxx,
                 t * dim_state_derivative * dim_state_derivative),
          DataAt(cost_derivative.cxu, t * dim_state_derivative * dim_action),
          DataAt(cost_derivative.cuu, t * dim_action * dim_action),
          DataAt(backward_pass.Vx, t * dim_state_derivative),
          DataAt(backward_pass.Vxx,
                 t * dim_state_derivative * dim_state_derivative),
          DataAt(candidate_policy[0].action_improvement, t * dim_action),
          DataAt(candidate_policy[0].feedback_gain,
                 t * dim_action * dim_state_derivative),
          backward_pass.dV, DataAt(backward_pass.Qx, t * dim_state_derivative),
          DataAt(backward_pass.Qu, t * dim_action),
          DataAt(backward_pass.Qxx,
                 t * dim_state_derivative * dim_state_derivative),
          DataAt(backward_pass.Qxu, t * dim_state_derivative * dim_action),
          DataAt(backward_pass.Quu, t * dim_action * dim_action),
          backward_pass.Q_scratch.data(), boxqp,
          DataAt(candidate_policy[0].trajectory.actions, t * dim_action),
          model->actuator_ctrlrange, settings.regularization_type,
          settings.action_limits);

      // failure
      if (!status) {
        // information
        if (settings.verbose) {
          printf("Backward Pass Failure (%i / %i)\n", regularization_iteration,
                 settings.max_regularization_iterations);
          printf("  time index: %i\n", t);  // Note
          printf("  simulation time: %f\n", time);
          printf("  regularization: %f\n", backward_pass.regularization);
          printf("  regularization factor: %f\n",
                 backward_pass.regularization_factor);
        }
        break;
      }

      // complete
      if (t == 0) {
        // set feedback gains and improvement at final time step
        mju_copy(DataAt(candidate_policy[0].feedback_gain,
                        (horizon - 1) * dim_action * dim_state_derivative),
                 DataAt(candidate_policy[0].feedback_gain,
                        (horizon - 2) * dim_action * dim_state_derivative),
                 dim_action * dim_state_derivative);
        mju_copy(DataAt(candidate_policy[0].action_improvement,
                        (horizon - 1) * dim_action),
                 DataAt(candidate_policy[0].action_improvement,
                        (horizon - 2) * dim_action),
                 dim_action);

        // backward pass status -> success
        backward_pass_status = 1;
        break;
      }
    }

    // increase regularization
    if (backward_pass.regularization <= settings.max_regularization &&
        backward_pass_status == 0) {
      backward_pass.ScaleRegularization(backward_pass.regularization_factor,
                                        settings.min_regularization,
                                        settings.max_regularization);
      regularization_iteration += 1;
    }
  }

  // end timer
  double backward_pass_time = GetDuration(backward_pass_start);

  // terminate early if backward pass failure
  if (backward_pass_status == 0) {
    // set timers
    model_derivative_compute_time = model_derivative_time;
    cost_derivative_compute_time = cost_derivative_time;
    rollouts_compute_time = 0.0;
    backward_pass_compute_time = backward_pass_time;
    policy_update_compute_time = 0.0;
    return;
  }

  // ----- rollout policy ----- //
  auto rollouts_start = std::chrono::steady_clock::now();

  // copy policy
  for (int j = 1; j < num_trajectory_; j++) {
    candidate_policy[j].CopyFrom(candidate_policy[0], horizon);
    candidate_policy[j].representation = candidate_policy[0].representation;
  }

  // feedback rollouts (parallel)
  this->ActionRollouts(horizon, pool);

  // ----- evaluate rollouts ----- //

  // get best rollout
  int best_rollout = this->BestRollout();
  if (best_rollout == -1) {
    return;
  } else {
    winner = best_rollout;
  }

  // update nominal with winner
  candidate_policy[0].trajectory = trajectory[winner];

  // improvement
  action_step = linesearch_steps[winner];
  expected = -1.0 * action_step *
                 (backward_pass.dV[0] + action_step * backward_pass.dV[1]) +
             1.0e-16;
  improvement = previous_return - trajectory[winner].total_return;
  surprise = mju_min(mju_max(0, improvement / expected), 2);

  // update regularization
  backward_pass.UpdateRegularization(settings.min_regularization,
                                     settings.max_regularization, surprise,
                                     action_step);

  if (settings.verbose) {
    std::cout << "iLQG Information\n" << '\n';
    std::cout << "  best return: " << trajectory[winner].total_return << '\n';
    std::cout << "  previous return: " << previous_return << '\n';
    std::cout << "  nominal return: " << policy.trajectory.total_return << '\n';
    std::cout << "  linesearch step size: " << action_step << '\n';
    std::cout << "  improvement: " << improvement << '\n';
    std::cout << "  regularization: " << backward_pass.regularization << '\n';
    std::cout << "  regularization factor: "
              << backward_pass.regularization_factor << '\n';
    std::cout << "  dV: " << expected << '\n';
    std::cout << "  dV[0]: " << backward_pass.dV[0] << '\n';
    std::cout << "  dV[1]: " << backward_pass.dV[1] << '\n';
    std::cout << std::endl;
  }

  // stop timer
  double rollouts_time = GetDuration(rollouts_start);

  // ----- policy update ----- //
  // start timer
  auto policy_update_start = std::chrono::steady_clock::now();
  {
    const std::shared_lock<std::shared_mutex> lock(mtx_);
    // improvement
    previous_policy = policy;
    policy.CopyFrom(candidate_policy[winner], horizon);

    // feedback scaling
    policy.feedback_scaling = 1.0;
  }

  // stop timer
  double policy_update_time = GetDuration(policy_update_start);

    print_counter++;
    if(print_counter == 20){
        print_counter = 0;

        std::cout << "percent derivs: " << key_point_generator.percent_of_derivs << "\n";
        std::cout << "time fd: " << time_fd << "\n";
        std::cout << "time model derivs: " << model_derivative_time << "\n";
        std::cout << "time transpose: " << time_transpose << "\n";
        std::cout << "time itnerpolate: " << key_point_generator.time_interpolate << "\n";

    }



//    std::cout << "time cost derivs: " << cost_derivative_time << "\n";
//    std::cout << "time rollouts: " << rollouts_time << "\n";
//    std::cout << "time backward pass: " << backward_pass_time << "\n";
//    std::cout << "time policy update: " << policy_update_time << "\n";

  // set timers
  model_derivative_compute_time = model_derivative_time;
  cost_derivative_compute_time = cost_derivative_time;
  rollouts_compute_time = rollouts_time;
  backward_pass_compute_time = backward_pass_time;
  policy_update_compute_time = policy_update_time;
}

// compute candidate trajectories
void iLQGPlanner::ActionRollouts(int horizon, ThreadPool& pool) {
  int count_before = pool.GetCount();
  for (int i = 0; i < num_trajectory_; i++) {
    pool.Schedule([&data = data_, &trajectory = trajectory,
                   &candidate_policy = candidate_policy,
                   &linesearch_steps = linesearch_steps, &model = this->model,
                   &task = this->task, &state = this->state, &time = this->time,
                   &mocap = this->mocap, horizon, &userdata = this->userdata,
                   i]() {
      // scale improvement
      mju_addScl(candidate_policy[i].trajectory.actions.data(),
                 candidate_policy[i].trajectory.actions.data(),
                 candidate_policy[i].action_improvement.data(),
                 linesearch_steps[i], model->nu * horizon);

      // policy
      auto feedback_policy = [&candidate_policy = candidate_policy, model, i](
                                 double* action, const double* state,
                                 int index) {
        // dimensions
        int dim_state = model->nq + model->nv + model->na;
        int dim_state_derivative = 2 * model->nv + model->na;
        int dim_action = model->nu;

        // set improved action
        mju_copy(
            action,
            DataAt(candidate_policy[i].trajectory.actions, index * dim_action),
            dim_action);

        // ----- feedback ----- //

        // difference between current state and nominal state
        StateDiff(
            model, candidate_policy[i].state_scratch.data(),
            DataAt(candidate_policy[i].trajectory.states, index * dim_state),
            state, 1.0);

        // compute feedback term
        mju_mulMatVec(candidate_policy[i].action_scratch.data(),
                      DataAt(candidate_policy[i].feedback_gain,
                             index * dim_action * dim_state_derivative),
                      candidate_policy[i].state_scratch.data(), dim_action,
                      dim_state_derivative);

        // add feedback
        mju_addTo(action, candidate_policy[i].action_scratch.data(),
                  dim_action);

        // clamp controls
        Clamp(action, model->actuator_ctrlrange, dim_action);
      };

      // policy rollout (discrete time)
      trajectory[i].RolloutDiscrete(
          feedback_policy, task, model, data[ThreadPool::WorkerId()].get(),
          state.data(), time, mocap.data(), userdata.data(), horizon);
    });
  }
  pool.WaitCount(count_before + num_trajectory_);

  pool.ResetCount();
}

// compute candidate trajectories searching over feedback scaling
void iLQGPlanner::FeedbackRollouts(int horizon, ThreadPool& pool) {
  int count_before = pool.GetCount();
  for (int i = 0; i < num_trajectory_; i++) {
    pool.Schedule([&data = data_, &trajectory = trajectory,
                   &candidate_policy = candidate_policy,
                   &linesearch_steps = linesearch_steps, &model = this->model,
                   &task = this->task, &state = this->state, &time = this->time,
                   &mocap = this->mocap, horizon, &userdata = this->userdata,
                   &settings = this->settings, i]() {
      // feedback scaling
      candidate_policy[i].feedback_scaling = linesearch_steps[i];

      // policy
      auto feedback_policy =
          [&candidate_policy = candidate_policy[i], &settings = settings](
              double* action, const double* state, double time) {
            candidate_policy.Action(
                action, settings.nominal_feedback_scaling ? state : NULL, time);
          };

      // policy rollout
      trajectory[i].Rollout(feedback_policy, task, model,
                            data[ThreadPool::WorkerId()].get(), state.data(),
                            time, mocap.data(), userdata.data(), horizon);
    });
  }
  pool.WaitCount(count_before + num_trajectory_);

  pool.ResetCount();
}

// return index of trajectory with best rollout
int iLQGPlanner::BestRollout() {
  double best_return = 0;
  int best_rollout = -1;

  for (int j = num_trajectory_ - 1; j >= 0; j--) {
    if (trajectory[j].failure) continue;
    double rollout_return = trajectory[j].total_return;
    if (best_rollout == -1 || rollout_return < best_return) {
      best_return = rollout_return;
      best_rollout = j;
    }
  }
  return best_rollout;
}

}  // namespace mjpc
