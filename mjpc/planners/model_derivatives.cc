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

void ModelDerivatives::ComputeSkips(const mjModel *m, const std::vector<UniqueMjData> &data, const double *x,
                                    const double *u, const double *h, int dim_state, int dim_state_derivative,
                                    int dim_action, int dim_sensor, int T, double tol, int mode, ThreadPool &pool,
                                    int skip) {
    {
        evaluate_.clear();
        interpolate_.clear();

        int s = skip + 1;
        evaluate_.push_back(0);
        for (int t = s; t < T - s; t += s) {
            evaluate_.push_back(t);
        }
        evaluate_.push_back(T - 2);
        evaluate_.push_back(T - 1);

        // interpolate indices
        for (int t = 0; t < T; t++) {
            if (std::find(evaluate_.begin(), evaluate_.end(), t) == evaluate_.end()) {
                interpolate_.push_back(t);
            }
        }

        int count_before = pool.GetCount();
        for (int t : evaluate_) {

            // If no keypoints for this dof, skip computation
//            if(keypoints.size() == 0){
//                continue;
//            }
            pool.Schedule([&m, &data, &A = A, &B = B, &C = C, &D = D, &x, &u, &h,
                                  dim_state, dim_state_derivative, dim_action, dim_sensor,
                                  tol, mode, t, T]() {
                mjData *d = data[ThreadPool::WorkerId()].get();
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

        pool.WaitCount(count_before + evaluate_.size());
        pool.ResetCount();

        // interpolate derivatives
//        count_before = pool.GetCount();
//        for (int t: interpolate_) {
//            pool.Schedule([&A = A, &B = B, &C = C, &D = D, &evaluate_ = this->evaluate_,
//                                  dim_state_derivative, dim_action, dim_sensor, t]() {
//                // find interval
//                int bounds[2];
//                FindInterval(bounds, evaluate_, t, evaluate_.size());
//                int e0 = evaluate_[bounds[0]];
//                int e1 = evaluate_[bounds[1]];
//
//                // normalized input
//                double tt = double(t - e0) / double(e1 - e0);
//                if (bounds[0] == bounds[1]) {
//                    tt = 0.0;
//                }
//
//                // A
//                int nA = dim_state_derivative * dim_state_derivative;
//                double *Ai = DataAt(A, t * nA);
//                double *AL = DataAt(A, e0 * nA);
//                double *AU = DataAt(A, e1 * nA);
//
//                mju_scl(Ai, AL, 1.0 - tt, nA);
//                mju_addToScl(Ai, AU, tt, nA);
//
//                // B
//                int nB = dim_state_derivative * dim_action;
//                double *Bi = DataAt(B, t * nB);
//                double *BL = DataAt(B, e0 * nB);
//                double *BU = DataAt(B, e1 * nB);
//
//                mju_scl(Bi, BL, 1.0 - tt, nB);
//                mju_addToScl(Bi, BU, tt, nB);
//
//                // C
//                int nC = dim_sensor * dim_state_derivative;
//                double *Ci = DataAt(C, t * nC);
//                double *CL = DataAt(C, e0 * nC);
//                double *CU = DataAt(C, e1 * nC);
//
//                mju_scl(Ci, CL, 1.0 - tt, nC);
//                mju_addToScl(Ci, CU, tt, nC);
//
//                // D
//                int nD = dim_sensor * dim_action;
//                double *Di = DataAt(D, t * nD);
//                double *DL = DataAt(D, e0 * nD);
//                double *DU = DataAt(D, e1 * nD);
//
//                mju_scl(Di, DL, 1.0 - tt, nD);
//                mju_addToScl(Di, DU, tt, nD);
//            });
//        }
//
//        pool.WaitCount(count_before + interpolate_.size());
//        pool.ResetCount();

        for (int t: interpolate_) {
                // find interval
                int bounds[2];
                FindInterval(bounds, evaluate_, t, evaluate_.size());
                int e0 = evaluate_[bounds[0]];
                int e1 = evaluate_[bounds[1]];

                // normalized input
                double tt = double(t - e0) / double(e1 - e0);
                if (bounds[0] == bounds[1]) {
                    tt = 0.0;
                }

                // A
                int nA = dim_state_derivative * dim_state_derivative;
                double *Ai = DataAt(A, t * nA);
                double *AL = DataAt(A, e0 * nA);
                double *AU = DataAt(A, e1 * nA);

                mju_scl(Ai, AL, 1.0 - tt, nA);
                mju_addToScl(Ai, AU, tt, nA);

                // B
                int nB = dim_state_derivative * dim_action;
                double *Bi = DataAt(B, t * nB);
                double *BL = DataAt(B, e0 * nB);
                double *BU = DataAt(B, e1 * nB);

                mju_scl(Bi, BL, 1.0 - tt, nB);
                mju_addToScl(Bi, BU, tt, nB);

                // C
                int nC = dim_sensor * dim_state_derivative;
                double *Ci = DataAt(C, t * nC);
                double *CL = DataAt(C, e0 * nC);
                double *CU = DataAt(C, e1 * nC);

                mju_scl(Ci, CL, 1.0 - tt, nC);
                mju_addToScl(Ci, CU, tt, nC);

                // D
                int nD = dim_sensor * dim_action;
                double *Di = DataAt(D, t * nD);
                double *DL = DataAt(D, e0 * nD);
                double *DU = DataAt(D, e1 * nD);

                mju_scl(Di, DL, 1.0 - tt, nD);
                mju_addToScl(Di, DU, tt, nD);
        }

    }
}

void ModelDerivatives::ComputeKeypoints(const mjModel* m, const std::vector<UniqueMjData>& data,
                       const double* x, const double* u, const double* h, int dim_state,
                       int dim_state_derivative, int dim_action, int dim_sensor, int T,
                       double tol, int mode, ThreadPool& pool, std::vector<std::vector<int>> keypoints){

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

    pool.WaitCount(count_before + threads_for_keypoints);
    pool.ResetCount();

}

//void ModelDerivatives::mjd_Transition_FD_keypoints(const mjModel_ *m, mjData_ *d, mjtNum eps, mjtByte flg_centered,
//                                 mjtNum *A_, mjtNum *B_, mjtNum *C_, mjtNum *D_,
//                                 std::vector<int> columns){
//
//    int nv = m->nv, na = m->na, nu = m->nu, ns = m->nsensordata;
//    int ndx = 2*nv+na;  // row length of state Jacobians
//
//    // stepFD() offset pointers, initialised to NULL
//    mjtNum *DyDq, *DyDv, *DyDa, *DsDq, *DsDv, *DsDa;
//    DyDq = DyDv = DyDa = DsDq = DsDv = DsDa = NULL;
//
//    mj_markStack(d);
//
//    mjtNum *AT = A_ ? mj_stackAllocNum(d, ndx*ndx) : NULL;  // state-transition matrix   (transposed)
//    mjtNum *BT = B_ ? mj_stackAllocNum(d, nu*ndx) : NULL;   // control-transition matrix (transposed)
//    mjtNum *CT = C_ ? mj_stackAllocNum(d, ndx*ns) : NULL;   // state-observation matrix   (transposed)
//    mjtNum *DT = D_ ? mj_stackAllocNum(d, nu*ns) : NULL;    // control-observation matrix (transposed)
//
//    // set offset pointers
//    if (A_) {
//        DyDq = AT;
//        DyDv = AT+ndx*nv;
//        DyDa = AT+ndx*2*nv;
//    }
//
//    if (C_) {
//        DsDq = CT;
//        DsDv = CT + ns*nv;
//        DsDa = CT + ns*2*nv;
//    }
//
//    // get Jacobians
//    mjd_step_FD_keypoints(m, d, eps, flg_centered, DyDq, DyDv, DyDa, BT, DsDq, DsDv, DsDa, DT, columns);
//
//    // transpose
//    if (A_) mju_transpose(A_, AT, ndx, ndx);
//    if (B_) mju_transpose(B_, BT, nu, ndx);
//    if (C_) mju_transpose(C_, CT, ndx, ns);
//    if (D_) mju_transpose(D_, DT, nu, ns);
//
//    mj_freeStack(d);
//}

//void ModelDerivatives::mjd_step_FD_keypoints(const mjModel* m, mjData* d, mjtNum eps, mjtByte flg_centered,
//                           mjtNum* DyDq, mjtNum* DyDv, mjtNum* DyDa, mjtNum* DyDu,
//                           mjtNum* DsDq, mjtNum* DsDv, mjtNum* DsDa, mjtNum* DsDu,
//                           std::vector<int> columns){
//    int nq = m->nq, nv = m->nv, na = m->na, nu = m->nu, ns = m->nsensordata;
//    int ndx = 2*nv+na;  // row length of Dy Jacobians
//    mj_markStack(d);
//
//    // state to restore after finite differencing
//    unsigned int restore_spec = mjSTATE_FULLPHYSICS | mjSTATE_CTRL;
//    restore_spec |= mjDISABLED(mjDSBL_WARMSTART) ? 0 : mjSTATE_WARMSTART;
//
//    mjtNum *fullstate  = mj_stackAllocNum(d, mj_stateSize(m, restore_spec));
//    mjtNum *state      = mj_stackAllocNum(d, nq+nv+na);  // current state
//    mjtNum *next       = mj_stackAllocNum(d, nq+nv+na);  // next state
//    mjtNum *next_plus  = mj_stackAllocNum(d, nq+nv+na);  // forward-nudged next state
//    mjtNum *next_minus = mj_stackAllocNum(d, nq+nv+na);  // backward-nudged next state
//
//    // sensors
//    int skipsensor = !DsDq && !DsDv && !DsDa && !DsDu;
//    mjtNum *sensor       = skipsensor ? NULL : mj_stackAllocNum(d, ns);  // sensor values
//    mjtNum *sensor_plus  = skipsensor ? NULL : mj_stackAllocNum(d, ns);  // forward-nudged sensors
//    mjtNum *sensor_minus = skipsensor ? NULL : mj_stackAllocNum(d, ns);  // backward-nudged sensors
//
//    // controls
//    mjtNum *ctrl = mj_stackAllocNum(d, nu);
//
//    // save current inputs
//    mj_getState(m, d, fullstate, restore_spec);
//    mju_copy(ctrl, d->ctrl, nu);
//    getState(m, d, state, NULL);
//
//    // step input
//    mj_stepSkip(m, d, mjSTAGE_NONE, skipsensor);
//
//    // save output
//    getState(m, d, next, sensor);
//
//    // restore input
//    mj_setState(m, d, fullstate, restore_spec);
//
//    // finite-difference controls: skip=mjSTAGE_VEL, handle ctrl at range limits
//    if (DyDu || DsDu) {
//        for (int i=0; i < nu; i++) {
//            int limited = m->actuator_ctrllimited[i];
//            // nudge forward, if possible given ctrlrange
//            int nudge_fwd = !limited || inRange(ctrl[i], ctrl[i]+eps, m->actuator_ctrlrange+2*i);
//            if (nudge_fwd) {
//                // nudge forward
//                d->ctrl[i] += eps;
//
//                // step, get nudged output
//                mj_stepSkip(m, d, mjSTAGE_VEL, skipsensor);
//                getState(m, d, next_plus, sensor_plus);
//
//                // reset
//                mj_setState(m, d, fullstate, restore_spec);
//            }
//
//            // nudge backward, if possible given ctrlrange
//            int nudge_back = (flg_centered || !nudge_fwd) &&
//                             (!limited || inRange(ctrl[i]-eps, ctrl[i], m->actuator_ctrlrange+2*i));
//            if (nudge_back) {
//                // nudge backward
//                d->ctrl[i] -= eps;
//
//                // step, get nudged output
//                mj_stepSkip(m, d, mjSTAGE_VEL, skipsensor);
//                getState(m, d, next_minus, sensor_minus);
//
//                // reset
//                mj_setState(m, d, fullstate, restore_spec);
//            }
//
//            // difference states
//            if (DyDu) {
//                clampedStateDiff(m, DyDu+i*ndx, next, nudge_fwd ? next_plus : NULL,
//                                 nudge_back ? next_minus : NULL, eps);
//            }
//
//            // difference sensors
//            if (DsDu) {
//                clampedDiff(DsDu+i*ns, sensor, nudge_fwd ? sensor_plus : NULL,
//                            nudge_back ? sensor_minus : NULL, eps, ns);
//            }
//        }
//    }
//
//    // finite-difference activations: skip=mjSTAGE_VEL
//    if (DyDa || DsDa) {
//        for (int i=0; i < na; i++) {
//            // nudge forward
//            d->act[i] += eps;
//
//            // step, get nudged output
//            mj_stepSkip(m, d, mjSTAGE_VEL, skipsensor);
//            getState(m, d, next_plus, sensor_plus);
//
//            // reset
//            mj_setState(m, d, fullstate, restore_spec);
//
//            // nudge backward
//            if (flg_centered) {
//                // nudge backward
//                d->act[i] -= eps;
//
//                // step, get nudged output
//                mj_stepSkip(m, d, mjSTAGE_VEL, skipsensor);
//                getState(m, d, next_minus, sensor_minus);
//
//                // reset
//                mj_setState(m, d, fullstate, restore_spec);
//            }
//
//            // difference states
//            if (DyDa) {
//                if (!flg_centered) {
//                    stateDiff(m, DyDa+i*ndx, next, next_plus, eps);
//                } else {
//                    stateDiff(m, DyDa+i*ndx, next_minus, next_plus, 2*eps);
//                }
//            }
//
//            // difference sensors
//            if (DsDa) {
//                if (!flg_centered) {
//                    diff(DsDa+i*ns, sensor, sensor_plus, eps, ns);
//                } else {
//                    diff(DsDa+i*ns, sensor_minus, sensor_plus, 2*eps, ns);
//                }
//            }
//        }
//    }
//
//
//    // finite-difference velocities: skip=mjSTAGE_POS
//    if (DyDv || DsDv) {
//        for (int i=0; i < nv; i++) {
//            // nudge forward
//            d->qvel[i] += eps;
//
//            // step, get nudged output
//            mj_stepSkip(m, d, mjSTAGE_POS, skipsensor);
//            getState(m, d, next_plus, sensor_plus);
//
//            // reset
//            mj_setState(m, d, fullstate, restore_spec);
//
//            // nudge backward
//            if (flg_centered) {
//                // nudge
//                d->qvel[i] -= eps;
//
//                // step, get nudged output
//                mj_stepSkip(m, d, mjSTAGE_POS, skipsensor);
//                getState(m, d, next_minus, sensor_minus);
//
//                // reset
//                mj_setState(m, d, fullstate, restore_spec);
//            }
//
//            // difference states
//            if (DyDv) {
//                if (!flg_centered) {
//                    stateDiff(m, DyDv+i*ndx, next, next_plus, eps);
//                } else {
//                    stateDiff(m, DyDv+i*ndx, next_minus, next_plus, 2*eps);
//                }
//            }
//
//            // difference sensors
//            if (DsDv) {
//                if (!flg_centered) {
//                    diff(DsDv+i*ns, sensor, sensor_plus, eps, ns);
//                } else {
//                    diff(DsDv+i*ns, sensor_minus, sensor_plus, 2*eps, ns);
//                }
//            }
//        }
//    }
//
//    // finite-difference positions: skip=mjSTAGE_NONE
//    if (DyDq || DsDq) {
//        mjtNum *dpos  = mj_stackAllocNum(d, nv);  // allocate position perturbation
//        for (int i=0; i < nv; i++) {
//            // nudge forward
//            mju_zero(dpos, nv);
//            dpos[i] = 1;
//            mj_integratePos(m, d->qpos, dpos, eps);
//
//            // step, get nudged output
//            mj_stepSkip(m, d, mjSTAGE_NONE, skipsensor);
//            getState(m, d, next_plus, sensor_plus);
//
//            // reset
//            mj_setState(m, d, fullstate, restore_spec);
//
//            // nudge backward
//            if (flg_centered) {
//                // nudge backward
//                mju_zero(dpos, nv);
//                dpos[i] = 1;
//                mj_integratePos(m, d->qpos, dpos, -eps);
//
//                // step, get nudged output
//                mj_stepSkip(m, d, mjSTAGE_NONE, skipsensor);
//                getState(m, d, next_minus, sensor_minus);
//
//                // reset
//                mj_setState(m, d, fullstate, restore_spec);
//            }
//
//            // difference states
//            if (DyDq) {
//                if (!flg_centered) {
//                    stateDiff(m, DyDq+i*ndx, next, next_plus, eps);
//                } else {
//                    stateDiff(m, DyDq+i*ndx, next_minus, next_plus, 2*eps);
//                }
//            }
//
//            // difference sensors
//            if (DsDq) {
//                if (!flg_centered) {
//                    diff(DsDq+i*ns, sensor, sensor_plus, eps, ns);
//                } else {
//                    diff(DsDq+i*ns, sensor_minus, sensor_plus, 2*eps, ns);
//                }
//            }
//        }
//    }
//
//    mj_freeStack(d);
//
//}
//
//
//// ------ finite difference utility functions -------- //
//void ModelDerivatives::getState(const mjModel* m, const mjData* d, mjtNum* state, mjtNum* sensordata) {
//    mj_getState(m, d, state, mjSTATE_PHYSICS);
//    if (sensordata) {
//        mju_copy(sensordata, d->sensordata, m->nsensordata);
//    }
//}
//
//// Had to delete restrict keyword from here... dont know what it does.
//// dx = (x2 - x1) / h
//void ModelDerivatives::diff(mjtNum*  dx, const mjtNum* x1, const mjtNum* x2, mjtNum h, int n) {
//    mjtNum inv_h = 1/h;
//    for (int i=0; i < n; i++) {
//        dx[i] = inv_h * (x2[i] - x1[i]);
//    }
//}
//
//
//// finite-difference two state vectors ds = (s2 - s1) / h
//void ModelDerivatives::stateDiff(const mjModel* m, mjtNum* ds, const mjtNum* s1, const mjtNum* s2, mjtNum h) {
//    int nq = m->nq, nv = m->nv, na = m->na;
//
//    if (nq == nv) {
//        diff(ds, s1, s2, h, nq+nv+na);
//    } else {
//        mj_differentiatePos(m, ds, h, s1, s2);
//        diff(ds+nv, s1+nq, s2+nq, h, nv+na);
//    }
//}
//
//
//
//// finite-difference two vectors, forward, backward or centered
//void ModelDerivatives::clampedDiff(mjtNum* dx, const mjtNum* x, const mjtNum* x_plus, const mjtNum* x_minus,
//                        mjtNum h, int nx) {
//    if (x_plus && !x_minus) {
//        // forward differencing
//        diff(dx, x, x_plus, h, nx);
//    } else if (!x_plus && x_minus) {
//        // backward differencing
//        diff(dx, x_minus, x, h, nx);
//    } else if (x_plus && x_minus) {
//        // centered differencing
//        diff(dx, x_plus, x_minus, 2*h, nx);
//    } else {
//        // differencing failed, write zeros
//        mju_zero(dx, nx);
//    }
//}
//
//void ModelDerivatives::clampedStateDiff(const mjModel* m, mjtNum* ds, const mjtNum* s, const mjtNum* s_plus,
//                             const mjtNum* s_minus, mjtNum h) {
//    if (s_plus && !s_minus) {
//        // forward differencing
//        stateDiff(m, ds, s, s_plus, h);
//    } else if (!s_plus && s_minus) {
//        // backward differencing
//        stateDiff(m, ds, s_minus, s, h);
//    } else if (s_plus && s_minus) {
//        // centered differencing
//        stateDiff(m, ds, s_minus, s_plus, 2*h);
//    } else {
//        // differencing failed, write zeros
//        mju_zero(ds, 2*m->nv + m->na);
//    }
//}
//
//// check if two numbers are inside a given range
//int ModelDerivatives::inRange(const mjtNum x1, const mjtNum x2, const mjtNum* range) {
//    return x1 >= range[0] && x1 <= range[1] &&
//           x2 >= range[0] && x2 <= range[1];
//}
//
//
//
//// advance simulation using control callback, skipstage is mjtStage
//void ModelDerivatives::mj_stepSkip(const mjModel* m, mjData* d, int skipstage, int skipsensor) {
//    TM_START;
//
//    // common to all integrators
//    mj_checkPos(m, d);
//    mj_checkVel(m, d);
//    mj_forwardSkip(m, d, skipstage, skipsensor);
//    mj_checkAcc(m, d);
//
//    // compare forward and inverse solutions if enabled
//    if (mjENABLED(mjENBL_FWDINV)) {
//        mj_compareFwdInv(m, d);
//    }
//
//    // use selected integrator
//    switch ((mjtIntegrator) m->opt.integrator) {
//        case mjINT_EULER:
//            mj_EulerSkip(m, d, skipstage >= mjSTAGE_POS);
//            break;
//
//        case mjINT_RK4:
//            // ignore skipstage
//            mj_RungeKutta(m, d, 4);
//            break;
//
////        case mjINT_IMPLICIT:
////        case mjINT_IMPLICITFAST:
////            mj_implicitSkip(m, d, skipstage >= mjSTAGE_VEL);
////            break;
//
////        default:
////            mjERROR("invalid integrator");
//    }
//
//    TM_END(mjTIMER_STEP);
//}


}  // namespace mjpc
