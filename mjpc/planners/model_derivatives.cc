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

#include "mjpc/planners/model_derivatives.h"

#include <algorithm>

#include <mujoco/mujoco.h>

namespace mjpc {

// allocate memory
void ModelDerivatives::Allocate(int dim_state_derivative, int dim_action,
                                int dim_sensor, int T) {
  A.resize(dim_state_derivative * dim_state_derivative * T);
  B.resize(dim_state_derivative * dim_action * T);
  C.resize(dim_sensor * dim_state_derivative * T);
  D.resize(dim_sensor * dim_action * T);

    AT.resize(dim_state_derivative * dim_state_derivative * T);
    BT.resize(dim_state_derivative * dim_action * T);
    CT.resize(dim_sensor * dim_state_derivative * T);
    DT.resize(dim_sensor * dim_action * T);

}

// reset memory to zeros
void ModelDerivatives::Reset(int dim_state_derivative, int dim_action,
                             int dim_sensor, int T) {
  std::fill(A.begin(),
            A.begin() + T * dim_state_derivative * dim_state_derivative, 0.0);
  std::fill(B.begin(), B.begin() + T * dim_state_derivative * dim_action, 0.0);
  std::fill(C.begin(), C.begin() + T * dim_sensor * dim_state_derivative, 0.0);
  std::fill(D.begin(), D.begin() + T * dim_sensor * dim_action, 0.0);
}

// compute derivatives at all time steps
void ModelDerivatives::Compute(const mjModel* m,
                               const std::vector<UniqueMjData>& data,
                               const double* x, const double* u,
                               const double* h, int dim_state,
                               int dim_state_derivative, int dim_action,
                               int dim_sensor, int T, double tol, int mode,
                               ThreadPool& pool) {
  {
    int count_before = pool.GetCount();
    for (int t = 0; t < T; t++) {
      pool.Schedule([&m, &data, &A = A, &B = B, &C = C, &D = D, &x, &u, &h,
                     dim_state, dim_state_derivative, dim_action, dim_sensor,
                     tol, mode, t, T]() {
        mjData* d = data[ThreadPool::WorkerId()].get();
        // set state
        SetState(m, d, x + t * dim_state);
        d->time = h[t];

        // set action
        mju_copy(d->ctrl, u + t * dim_action, dim_action);

        // Jacobians
        if (t == T - 1) {
          // Jacobians
          mjd_transitionFD(m, d, tol, mode, nullptr, nullptr,
                           DataAt(C, t * (dim_sensor * dim_state_derivative)),
                           nullptr);
        } else {
          // derivatives
          mjd_transitionFD(
              m, d, tol, mode,
              DataAt(A, t * (dim_state_derivative * dim_state_derivative)),
              DataAt(B, t * (dim_state_derivative * dim_action)),
              DataAt(C, t * (dim_sensor * dim_state_derivative)),
              DataAt(D, t * (dim_sensor * dim_action)));
        }
      });
    }
    pool.WaitCount(count_before + T);
  }
  pool.ResetCount();
}

void ModelDerivatives::ComputeKeypoints(const mjModel* m, const std::vector<UniqueMjData>& data,
                       const double* x, const double* u, const double* h, int dim_state,
                       int dim_state_derivative, int dim_action, int dim_sensor, int T,
                       double tol, int mode, ThreadPool& pool, std::vector<std::vector<int>> keypoints){

//        for (int t = 0; t < T; t++) {
//
//            // If no keypoints for this dof, skip computation
//            if(keypoints[t].size() == 0){
//                continue;
//            }
//
//            mjData *d = data[ThreadPool::WorkerId()].get();
//            // set state
//            SetState(m, d, x + t * dim_state);
//            d->time = h[t];
//
//            // set action
//            mju_copy(d->ctrl, u + t * dim_action, dim_action);
//
//            // Columns at current time index to compute derivatives for
//            int *columns = new int[keypoints[t].size()];
//            int num_columns = keypoints[t].size();
//            for(int i = 0; i < num_columns; i++){
//                columns[i] = keypoints[t][i];
//            }
//
//            // Jacobians
//            if (t == T - 1) {
//                // Jacobians
//                mjd_transitionFD_columns(m, d, tol, mode, nullptr, nullptr,
//                                         DataAt(C, t * (dim_sensor * dim_state_derivative)),
//                                         nullptr, columns, num_columns);
//            } else {
//                // derivatives
//                mjd_transitionFD_columns(m, d, tol, mode,
//                                            DataAt(A, t * (dim_state_derivative * dim_state_derivative)),
//                                            DataAt(B, t * (dim_state_derivative * dim_action)),
//                                            DataAt(C, t * (dim_sensor * dim_state_derivative)),
//                                            DataAt(D, t * (dim_sensor * dim_action)),
//                                            columns, num_columns);
//
//            }
//
//            delete[] columns;
//    }

    int count_before = pool.GetCount();
    int threads_for_keypoints = 0;
    for (int t = 0; t < T; t++) {

        // If no keypoints for this dof, skip computation
        if(keypoints[t].size() == 0){
            continue;
        }
        threads_for_keypoints++;

        pool.Schedule([&m, &data, &A = A, &B = B, &C = C, &D = D, &x, &u, &h,
                              dim_state, dim_state_derivative, dim_action, dim_sensor,
                              tol, mode, t, T, keypoints]() {

            mjData *d = data[ThreadPool::WorkerId()].get();
            // set state
            SetState(m, d, x + t * dim_state);
            d->time = h[t];

            // set action
            mju_copy(d->ctrl, u + t * dim_action, dim_action);

            // Columns at current time index to compute derivatives for
            int *columns = new int[keypoints[t].size()];
            int num_columns = keypoints[t].size();
            for(int i = 0; i < num_columns; i++){
                columns[i] = keypoints[t][i];
            }

            // Jacobians
            if (t == T - 1) {
                // Jacobians
                mjd_transitionFD_columns(m, d, tol, mode, nullptr, nullptr,
                                         DataAt(C, t * (dim_sensor * dim_state_derivative)),
                                         nullptr, columns, num_columns);
            } else {
                // derivatives
                mjd_transitionFD_columns(m, d, tol, mode,
                                            DataAt(A, t * (dim_state_derivative * dim_state_derivative)),
                                            DataAt(B, t * (dim_state_derivative * dim_action)),
                                            DataAt(C, t * (dim_sensor * dim_state_derivative)),
                                            DataAt(D, t * (dim_sensor * dim_action)),
                                            columns, num_columns);

            }

            delete[] columns;
        });
    }

    pool.WaitCount(count_before + threads_for_keypoints);
    pool.ResetCount();
}

}  // namespace mjpc
