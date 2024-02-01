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

#ifndef MJPC_PLANNERS_MODEL_DERIVATIVES_H_
#define MJPC_PLANNERS_MODEL_DERIVATIVES_H_

#include <mujoco/mujoco.h>

#include <cstdlib>
#include <vector>

#include "mjpc/threadpool.h"
#include "mjpc/utilities.h"

#define TM_START mjtNum _tm = (mjcb_time ? mjcb_time() : 0);
#define TM_END(i) {d->timer[i].duration += ((mjcb_time ? mjcb_time() : 0) - _tm); d->timer[i].number++;}

namespace mjpc {

// data and methods for model derivatives
class ModelDerivatives {
 public:
  // constructor
  ModelDerivatives() = default;

  // destructor
  ~ModelDerivatives() = default;

  // allocate memory
  void Allocate(int dim_state_derivative, int dim_action, int dim_sensor,
                int T);

  // reset memory to zeros
  void Reset(int dim_state_derivative, int dim_action, int dim_sensor, int T);

  // compute derivatives at all time steps
  void Compute(const mjModel* m, const std::vector<UniqueMjData>& data,
               const double* x, const double* u, const double* h, int dim_state,
               int dim_state_derivative, int dim_action, int dim_sensor, int T,
               double tol, int mode, ThreadPool& pool);

  void ComputeSkips(const mjModel* m, const std::vector<UniqueMjData>& data,
                    const double* x, const double* u, const double* h, int dim_state,
                    int dim_state_derivative, int dim_action, int dim_sensor, int T,
                    double tol, int mode, ThreadPool& pool, int skip);

  //TODO (DMackRus) - test this function rigorously
  void ComputeKeypoints(const mjModel* m, const std::vector<UniqueMjData>& data,
                        const double* x, const double* u, const double* h, int dim_state,
                        int dim_state_derivative, int dim_action, int dim_sensor, int T,
                        double tol, int mode, ThreadPool& pool, std::vector<std::vector<int>> keypoints);

  // Compute only specific columns of the A, B, C, and D matrices.
  void mjd_Transition_FD_keypoints(const mjModel_ *m, mjData_ *d, mjtNum eps, mjtByte flg_centered,
                                   mjtNum *A_, mjtNum *B_, mjtNum *C_, mjtNum *D_,
                                   std::vector<int> columns);

  void mjd_step_FD_keypoints(const mjModel* m, mjData* d, mjtNum eps, mjtByte flg_centered,
                             mjtNum* DyDq, mjtNum* DyDv, mjtNum* DyDa, mjtNum* DyDu,
                             mjtNum* DsDq, mjtNum* DsDv, mjtNum* DsDa, mjtNum* DsDu,
                             std::vector<int> columns);


  // Jacobians
  std::vector<double> A;  // model Jacobians wrt state
                          //   (T * dim_state_derivative * dim_state_derivative)
  std::vector<double> B;  // model Jacobians wrt action
                          //   (T * dim_state_derivative * dim_action)
  std::vector<double> C;  // output Jacobians wrt state
                          //   (T * dim_sensor * dim_state_derivative)
  std::vector<double> D;  // output Jacobians wrt action
                          //   (T * dim_sensor * dim_action)

private:
    // Keypoint indices
    std::vector<int> evaluate_;
    std::vector<int> interpolate_;

    // Finite differencing utilities
    void getState(const mjModel* m, const mjData* d, mjtNum* state, mjtNum* sensordata);
    void diff(mjtNum* dx, const mjtNum* x1, const mjtNum* x2, mjtNum h, int n);
    void stateDiff(const mjModel* m, mjtNum* ds, const mjtNum* s1, const mjtNum* s2, mjtNum h);
    void clampedDiff(mjtNum* dx, const mjtNum* x, const mjtNum* x_plus, const mjtNum* x_minus,
                            mjtNum h, int nx);
    int inRange(const mjtNum x1, const mjtNum x2, const mjtNum* range);
    void mj_stepSkip(const mjModel* m, mjData* d, int skipstage, int skipsensor);
    void clampedStateDiff(const mjModel* m, mjtNum* ds, const mjtNum* s, const mjtNum* s_plus,
                          const mjtNum* s_minus, mjtNum h);


};

}  // namespace mjpc

#endif  // MJPC_PLANNERS_MODEL_DERIVATIVES_H_
