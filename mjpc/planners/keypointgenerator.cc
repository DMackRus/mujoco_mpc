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

#include "mjpc/planners/keypointgenerator.h"

namespace mjpc {

std::vector<std::vector<int>> KeypointGenerator::GenerateKeyPoints(keypoint_method keypoint_method,
                                                                   int T, const double* x, int dim_state){

    // Example how keypoint indices are stored, first dimension of vector is time
    // along the trajectory. at each index, is another vector for the degree of
    // freedom index that we want to compute dynamics gradients for. I.E, a list
    // at any time index can be empty. But we need all dofs, at t = 0, and t=T.
    //
    // t = 0, t = 5, ... t = T
    //   0,     0          0
    //   1      2          1
    //   2                 2

    // Indices at which to compute dynamics derivatives via finite-differencing
    std::vector<std::vector<int>> keypoints;

    // TODO DMackRus - I dont think this is always be true, sometimes there is a mismatch between qvel and qpos.
    int dof = dim_state / 2;

    // size of x = (dim_state * T)?

    std::vector<int> row_time_index;

    // Enforce all dofs in keypoint list at t = 0
    for(int i = 0; i < dof; i++){
        row_time_index.push_back(i);
    }
    keypoints.push_back(row_time_index);

    // For the trivial case of horizon == 2, we only need to evaluate the start and end points
    if(T == 2){
        keypoints.push_back(row_time_index);
        return keypoints;
    }

    if(keypoint_method.method == SET_INTERVAL){
        for(int i = 1; i < T; i++){

            if(i % keypoint_method.min_N == 0){
                std::vector<int> one_row;
                for(int j = 0; j < dof; j++){
                    one_row.push_back(j);
                }
                keypoints.push_back(one_row);
            }
            else{
                std::vector<int> one_row;
                keypoints.push_back(one_row);
            }
        }
    }
    else if(keypoint_method.method == ADAPTIVE_JERK){
        const double *jerk_profile = GenerateJerkProfile(T, x, dim_state);

        keypoints = GenerateKeypoints_AdaptiveJerk(keypoint_method, T, dim_state, jerk_profile);
    }
    else if(keypoint_method.method == VELOCITY_CHANGE){
        keypoints = GenerateKeypoints_VelocityChange(keypoint_method, T, dim_state, x);
    }

//    else if(keypoint_method.name == "iterative_error"){
//        computed_keypoints.clear();
//        physics_simulator->initModelForFiniteDifferencing();
//        keypoints = GenerateKeyPointsIteratively(horizon, keypoint_method, trajectory_states, A, B);
//        physics_simulator->resetModelAfterFiniteDifferencing();
//
//    }

    else{
        std::cout << "ERROR: keyPointsMethod not recognised \n";
    }

    // Enforce that last two time-steps have all dofs in keypoint list
    keypoints[T - 2] = keypoints[0];
    keypoints[T - 1] = keypoints[0];

    // temporary compute percentage of derivs computed.
    int total_derivs = 0;
    for(int i = 0; i < T; i++){
        total_derivs += keypoints[i].size();
    }

    percent_of_derivs = (double(total_derivs) / (double(T) * double(dof))) * 100;

//     Print out the key points
//    for(int i = 0; i < keypoints.size(); i++){
//        std::cout << "timestep " << i << ": ";
//        for(int j = 0; j < keypoints[i].size(); j++){
//            std::cout << keypoints[i][j] << " ";
//        }
//        std::cout << "\n";
//    }

    return keypoints;
}

//TODO(DMackRus) - Make this line shorter?
std::vector<std::vector<int>> KeypointGenerator::GenerateKeypoints_AdaptiveJerk(keypoint_method keypoint_method,
                                                                 int T, int dim_state,
                                                                 const double* jerk_profile){
    std::vector<std::vector<int>> keypoints;

    // TODO(DMackRus) - this could be wrong sometimes, if qpos != qvel
    int dof = dim_state / 2;

    for(int t = 0; t < T; t++){
        keypoints.push_back(std::vector<int>());
    }

    // Enforce all dofs in keypoint list at t = 0
    for(int i = 0; i < dof; i++){
        keypoints[0].push_back(i);
    }

    int *last_indices = new int[dof];

    for(int i = 0; i < dof; i++){
        last_indices[i] = 0;
    }

    // Loop over the trajectory
    for(int j = 0; j < dof; j++){
        for(int t = 1; t < T - 2; t++){

            if((t - last_indices[j]) >= keypoint_method.min_N) {
                // Check if the jerk is above the threshold
                if (jerk_profile[(t * dof) + j] > keypoint_method.jerk_thresholds[j]) {
                    keypoints[t].push_back(j);
                    last_indices[j] = t;
                }
            }

            if((t - last_indices[j]) >= keypoint_method.max_N){
                keypoints[t].push_back(j);
                last_indices[j] = t;
            }
        }
    }

    return keypoints;
}

std::vector<std::vector<int>> KeypointGenerator::GenerateKeypoints_VelocityChange(keypoint_method keypoint_method,
                                                               int T, int dim_state,
                                                               const double* x){
    std::vector<std::vector<int>> keypoints;

    // TODO(DMackRus) - this could be wrong sometimes, if qpos != qvel
    int dof = dim_state / 2;

    for(int t = 0; t < T; t++){
        keypoints.push_back(std::vector<int>());
    }

    // Enforce all dofs in keypoint list at t = 0
    for(int i = 0; i < dof; i++){
        keypoints[0].push_back(i);
    }

    // Keeps track of interval from last keypoint for this dof
    std::vector<int> last_keypoint_counter = std::vector<int>(dof, 0);
    std::vector<double> last_vel_value = std::vector<double>(dof, 0);
    std::vector<double> last_vel_direction = std::vector<double>(dof, 0);

    for(int i = 0; i < dof; i++){
        last_vel_value[i] = x[i + dof];
    }

    // Loop over the dofs
    for(int i = 0; i < dof; i++){
        // Loop over the horizon - skip 0, T-1 and T-2.
        for(int t = 1; t < T - 2; t++){

            last_keypoint_counter[i]++;
            // t at i
            double current_vel_direction = x[t * dim_state + dof + i] - x[(t - 1) * dim_state + dof + i];
            double current_vel_change_since_last_keypoint = x[t * dim_state + dof + i] - last_vel_value[i];

            // If the vel change is above the required threshold
            if(last_keypoint_counter[i] >= keypoint_method.min_N){
                if(abs(current_vel_change_since_last_keypoint) > keypoint_method.velocity_change_thresholds[i]){
                    keypoints[t].push_back(i);
                    last_vel_value[i] = x[t * dim_state + dof + i];
                    last_keypoint_counter[i] = 0;
                    continue;
                }
            }

            // If the interval is greater than min_N
            if(last_keypoint_counter[i] >= keypoint_method.min_N){
                // If the direction of the velocity has changed
                if(current_vel_direction * last_vel_direction[i] < 0){
                    keypoints[t].push_back(i);
                    last_vel_value[i] = x[t * dim_state + dof + i];
                    last_keypoint_counter[i] = 0;
                    continue;
                }
            }
            else{
                last_vel_direction[i] = current_vel_direction;
            }

            // If interval is greater than max_N
            if(last_keypoint_counter[i] >= keypoint_method.max_N){
                keypoints[t].push_back(i);
                last_vel_value[i] = x[t * dim_state + dof + i];
                last_keypoint_counter[i] = 0;
                continue;
            }
        }
    }

    return keypoints;
}

void KeypointGenerator::InterpolateDerivatives(std::vector<std::vector<int>> keypoints,
                                               std::vector<double> &A,
                                               std::vector<double> &B,
                                               std::vector<double> &C,
                                               std::vector<double> &D,
                                               int dim_state, int dim_action, int dim_sensor, int T){


    // Offset variables in time dimension for A, B, C, D matrices
    int nA = dim_state * dim_state;
    int nB = dim_state * dim_action;
    int nC = dim_sensor * dim_state;
    int nD = dim_sensor * dim_action;

    // TODO(DMackRus) - not certain if this is always the case that qvel = qpos
    // offset for velocity elements
    int dof = dim_state / 2;

    // Assign last indices array to all zeros. All columns
    // HAVE to be computed at t = 0.
    int *last_indices = new int[keypoints[0].size()];

    for(int i = 0; i < keypoints[0].size(); i++){
        last_indices[i] = 0;
    }

    // Loop through trajectory horizon and perform interpolation
    for(int t = 1; t < T; t++) {

        // Skip this time index if all derivatives computed via F.D
        if (keypoints[t].size() == 0) {
            continue;
        }

        // Loop through all the columns at this time index
        for (int col : keypoints[t]) {

            for(int i = last_indices[col] + 1; i < t; i++){
//                std::cout << "interpolating dof " << col << " at time " << i << " from " << last_indices[col] << " to " << t << "\n";

                double tt = double(i - last_indices[col]) / double(t - last_indices[col]);

                // ---------------------------- A matrices -------------------------------------

                // Position and velocity column that we are interpolating
                double *A_pi = DataAt(A, (nA * i) + (col * dim_state));
                double *A_vi = DataAt(A, (nA * i) + (col * dim_state) + (dim_state * dof));

                // Position and velocity column that we are interpolating from
                double *A_pL = DataAt(A, (last_indices[col] * nA) + (col * dim_state));
                double *A_vL = DataAt(A, (last_indices[col] * nA) + (col * dim_state) + (dim_state * dof));

                // Position and velocity column that we are interpolating to
                double *A_pU = DataAt(A, (t * nA) + (col * dim_state));
                double *A_vU = DataAt(A, (t * nA) + (col * dim_state) + (dim_state * dof));

                // Perform the interpolation
                mju_scl(A_pi, A_pL, 1.0 - tt, dim_state);
                mju_addToScl(A_pi, A_pU, tt, dim_state);

                mju_scl(A_vi, A_vL, 1.0 - tt, dim_state);
                mju_addToScl(A_vi, A_vU, tt, dim_state);

                // ---------------------------- B matrices -------------------------------------

                // Perform a check that the column is within the range of the B matrix
                if(col < dim_action){

                        // Position and velocity column that we are interpolating
                        double *B_i = DataAt(B, (i * nB) + (col * dim_state));

                        // Position and velocity column that we are interpolating from
                        double *B_L = DataAt(B, (last_indices[col] * nB) + (col * dim_state));

                        // Position and velocity column that we are interpolating to
                        double *B_U = DataAt(B, (t * nB) + (col * dim_state));

                        // Perform the interpolation
                        mju_scl(B_i, B_L, 1.0 - tt, dim_state);
                        mju_addToScl(B_i, B_U, tt, dim_state);
                }

                // ---------------------------- C matrices -------------------------------------

                // Position and velocity column that we are interpolating
                double *C_pi = DataAt(C, (i * nC) + (col * dim_sensor));
                double *C_vi = DataAt(C, (i * nC) + (col * dim_sensor) + (dim_sensor * dof));

                // Position and velocity column that we are interpolating from
                double *C_pL = DataAt(C, (last_indices[col] * nC) + (col * dim_sensor));
                double *C_vL = DataAt(C, (last_indices[col] * nC) + (col * dim_sensor) + (dim_sensor * dof));

                // Position and velocity column that we are interpolating to
                double *C_pU = DataAt(C, (t * nC) + (col * dim_sensor));
                double *C_vU = DataAt(C, (t * nC) + (col * dim_sensor) + (dim_sensor * dof));

                // Perform the interpolation
                mju_scl(C_pi, C_pL, 1.0 - tt, dim_sensor);
                mju_addToScl(C_pi, C_pU, tt, dim_sensor);

                mju_scl(C_vi, C_vL, 1.0 - tt, dim_sensor);
                mju_addToScl(C_vi, C_vU, tt, dim_sensor);

                // ---------------------------- D matrices -------------------------------------

                // Perform a check that the column is within the range of the D matrix
                if(col < dim_action){

                    // Position and velocity column that we are interpolating
                    double *D_i = DataAt(D, (i * nD) + (col * dim_sensor));

                    // Position and velocity column that we are interpolating from
                    double *D_L = DataAt(D, (last_indices[col] * nD) + (col * dim_sensor));

                    // Position and velocity column that we are interpolating to
                    double *D_U = DataAt(D, (t * nD) + (col * dim_sensor));

                    // Perform the interpolation
                    mju_scl(D_i, D_L, 1.0 - tt, dim_sensor);
                    mju_addToScl(D_i, D_U, tt, dim_sensor);
                }
            }
            // Update last time index for this column
            last_indices[col] = t;
        }
    }
}

// TODO(DMackRus) - consider whether 0 and 1 should be empty or last 2 indices
// TODO(DMackRus) - could this be done with mju_ functions?
double* KeypointGenerator::GenerateJerkProfile(int T, const double* x, int dim_state){
    double* jerk_profile = new double[(T - 2) * (dim_state/2)];

    for(int t = 2; t < T; t++){
        for(int i = 0; i < dim_state/2; i++){
            double qdd = (x[(t * dim_state) + i] - 2 * x[((t - 1) * dim_state) + i] + x[((t - 2) * dim_state) + i]);
            jerk_profile[((t - 2) * (dim_state/2)) + i] = qdd;
        }
    }

    return jerk_profile;
}

}  // namespace mjpc
