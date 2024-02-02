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

std::vector<std::vector<int>> KeyPointGenerator::GenerateKeyPoints(keypoint_method keypoint_method,
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

    if(keypoint_method.name == "set_interval"){
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
//    else if(keypoint_method.name == "Adaptive_Jerk"){
//        double *jerk_profile = GenerateJerkProfile(T, trajectory_states);
//        keypoints = GenerateKeyPointsAdaptive(horizon, jerk_profile, keypoint_method);
//    }
//    else if(keypoint_method.name == "iterative_error"){
//        computed_keypoints.clear();
//        physics_simulator->initModelForFiniteDifferencing();
//        keypoints = GenerateKeyPointsIteratively(horizon, keypoint_method, trajectory_states, A, B);
//        physics_simulator->resetModelAfterFiniteDifferencing();
//
//    }
//    else if(keypoint_method.name == "Velocity_Change"){
//        std::vector<MatrixXd> velocity_profile = GenerateVelocityProfile(horizon, trajectory_states);
//        keypoints = GenerateKeyPointsVelocityChange(horizon, velocity_profile, keypoint_method);
//    }
    else{
        std::cout << "ERROR: keyPointsMethod not recognised \n";
    }

    // Enforce that last time step is evaluated for all dofs, otherwise nothing to interpolate to
    keypoints.back().clear();
    for(int i = 0; i < dof; i++){
        keypoints.back().push_back(i);
    }

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

void KeyPointGenerator::InterpolateDerivatives(std::vector<std::vector<int>> keypoints,
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

    // This method works for full matrix interpolation and fast
//    int last_index = 0;
//    for(int t = 1; t < T; t++){
//
//        if(keypoints[t].size() == 0){
//            continue;
//        }
//
//        for(int i = last_index + 1; i < t; i++){
//
//            // proportion between 0 and 1 between last_index and t
//            double tt = double(i - last_index) / double(t - last_index);
//
//            // A
//            double *Ai = DataAt(A, i * nA);
//            double *AL = DataAt(A, last_index * nA);
//            double *AU = DataAt(A, t * nA);
//
//            mju_scl(Ai, AL, 1.0 - tt, nA);
//            mju_addToScl(Ai, AU, tt, nA);
//
//            // B
//            double *Bi = DataAt(B, i * nB);
//            double *BL = DataAt(B, last_index * nB);
//            double *BU = DataAt(B, t * nB);
//
//            mju_scl(Bi, BL, 1.0 - tt, nB);
//            mju_addToScl(Bi, BU, tt, nB);
//
//            // C
//            double *Ci = DataAt(C, i * nC);
//            double *CL = DataAt(C, last_index * nC);
//            double *CU = DataAt(C, t * nC);
//
//            mju_scl(Ci, CL, 1.0 - tt, nC);
//            mju_addToScl(Ci, CU, tt, nC);
//
//            // D
//            double *Di = DataAt(D, i * nD);
//            double *DL = DataAt(D, last_index * nD);
//            double *DU = DataAt(D, t * nD);
//
//            mju_scl(Di, DL, 1.0 - tt, nD);
//            mju_addToScl(Di, DU, tt, nD);
//
//        }
//
//        last_index = t;
//    }

    // TODO(DMackRus) - not certain if this is always the case that qvel = qpos
    // offset for velocity elements

    int dof = dim_state / 2;

    // Assign last indices array to all zeros. All columns
    // HAVE to be computed at t = 0.
    int *last_indices = new int[keypoints[0].size()];

    for(int i = 0; i < keypoints[0].size(); i++){
        last_indices[i] = 0;
    }

    // Columnwise interpolation using mju_scl and mju_addToScl
    // Loop through trajectory horizon and perform interpolation
    for(int t = 1; t < T; t++) {

        // Skip this time index if all derivatives computed via F.D
        if (keypoints[t].size() == 0) {
            continue;
        }

        // Loop through all the columns at this time index
        for (int col : keypoints[t]) {

            for(int i = last_indices[col] + 1; i < t; i++){

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
                // TODO(DMackRus) <= or < ?
                if(col <= dim_action){

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
            last_indices[col] = t;
        }
    }

    // Lets try print the matrices and see if we can see null values??
//    for(int t = 0; t < T; t++){
//        std::cout << "timestep: " << t << "\n";
//        for(int i = 0; i < dim_state; i++){
//            for(int j = 0; j < dim_state; j++){
//                std::cout << A[t * nA + i * dim_state + j] << " ";
//            }
//            std::cout << "\n";
//        }
//    }

    // B matrices
//    for(int t = 0; t < T; t++){
//        std::cout << "timestep: " << t << "\n";
//        for(int i = 0; i < dim_state; i++){
//            for(int j = 0; j < dim_action; j++){
//                std::cout << B[t * nB + i * dim_action + j] << " ";
//            }
//            std::cout << "\n";
//        }
//    }

    // C matrices
//    for(int t = 0; t < T; t++){
//        std::cout << "timestep: " << t << "\n";
//        for(int i = 0; i < dim_sensor; i++){
//            for(int j = 0; j < dim_state; j++){
//                std::cout << C[t * nC + i * dim_state + j] << " ";
//            }
//            std::cout << "\n";
//        }
//    }




    // Trying to do the interpolation myself - seems slow atm, and also doesnt work
    // Loop through trajectory horizon and perform interpolation
//    for(int t = 1; t < T; t++) {
//
//        // Skip this time index if derivatives already computed
//        if (keypoints[t].size() == 0) {
//            continue;
//        }
//
//        // Loop through all the columns at this time index
//        for (int col: keypoints[t]) {
//
//            // A matrices - dim_state is how many rows in A matrix
//            for (int i = 0; i < dim_state; i++) {
//
//                //Start and end values for interpolation
//                mjtNum start_val_pos = A[nA * last_indices[col] + (col * dim_state) + i];
//                mjtNum end_val_pos = A[nA * t + (col * dim_state) + i];
//
//                // Amount to add per time step between start and end
//                mjtNum pos_add = (end_val_pos - start_val_pos) / (t - last_indices[col]);
//
//                // Do the same for velocity elements in the A matrix
//                mjtNum start_val_vel = A[(nA * last_indices[col]) + (col * dim_state) + (dim_state * dof) + i];
//                mjtNum end_val_vel = A[(nA * t) + (col * dim_state) + (dim_state * dof) + i];
//
//                // Amount to add per time step between start and end
//                mjtNum vel_add = (end_val_vel - start_val_vel) / (t - last_indices[col]);
//
//                // Loop through from previous keypoint to now, interpolating all values
//                // in the columns for that dof
//                for (int j = last_indices[col]; j < t; j++) {
//                    A[nA * j + (col * dim_state) + i] = start_val_pos + ((j - last_indices[col]) * pos_add);
//                    A[nA * j + (col * dim_state) + (dim_state * dof) + i] =
//                            start_val_vel + ((j - last_indices[col]) * vel_add);
//                }
//            }
//
//            // B matrices - dim_state is how many rows in the B matrix
//            for (int i = 0; i < dim_state; i++) {
//
//                // Perform a check that the column is within the range of the B matrix
//                // i.e not all dofs are actuated
//                if (col > dim_action) {
//                    continue;
//                }
//
//                // Start and end values for interpolation
//                mjtNum start_val_ctrl = B[(nB * last_indices[col]) + (col * dim_state) + i];
//                mjtNum end_val_ctrl = B[nB * t + (col * dim_state) + i];
//
//                // Amount to add per timestep between start and end
//                mjtNum ctrl_add = (end_val_ctrl - start_val_ctrl) / (t - last_indices[col]);
//
//                for (int j = last_indices[col]; j < t; j++) {
//                    B[nB * j + (col * dim_state) + i] = start_val_ctrl + ((j - last_indices[col]) * ctrl_add);
//                }
//            }
//
//            // C matrices - dim_sensor is how many rows in the C matrix
//            for (int i = 0; i < dim_sensor; i++) {
//
//                // Start and end values for interpolation
//                mjtNum start_val_pos = C[nC * last_indices[col] + (col * dim_sensor) + i];
//                mjtNum end_val_pos = C[nC * t + (col * dim_sensor) + i];
//
//                // Amount to add per timestep between start and end
//                mjtNum pos_add = (end_val_pos - start_val_pos) / (t - last_indices[col]);
//
//                mjtNum start_val_vel = C[nC * last_indices[col] + (col * dim_sensor) + (dim_sensor * dof) + i];
//                mjtNum end_val_vel = C[nC * t + (col * dim_sensor) + (dim_sensor * dof) + i];
//
//                // Amount to add per timestep between start and end
//                mjtNum vel_add = (end_val_vel - start_val_vel) / (t - last_indices[col]);
//
//                for (int j = last_indices[col]; j < t; j++) {
//                    C[nC * j + (col * dim_sensor) + i] = start_val_pos + ((j - last_indices[col]) * pos_add);
//                    C[nC * j + (col * dim_sensor) + (dim_sensor * dof) + i] =
//                            start_val_vel + ((j - last_indices[col]) * vel_add);
//                }
//            }
//
//            // D matrices - dim_sensor is how many rows in the D matrix
//            for (int i = 0; i < dim_sensor; i++) {
//
//                // Perform a check that the column is within the range of the D matrix
//                // i.e not all dofs are actuated
//                if (col > dim_action) {
//                    continue;
//                }
//
//                // Start and end values for interpolation
//                mjtNum start_val_ctrl = D[nD * last_indices[col] + (col * dim_sensor) + i];
//                mjtNum end_val_ctrl = D[nD * t + (col * dim_sensor) + i];
//
//                // Amount to add per timestep between start and end
//                mjtNum ctrl_add = (end_val_ctrl - start_val_ctrl) / (t - last_indices[col]);
//
//                for (int j = last_indices[col]; j < t; j++) {
//                    D[nD * j + (col * dim_sensor) + i] = start_val_ctrl + ((j - last_indices[col]) * ctrl_add);
//                }
//            }
//        }
//    }







}

//void InterpolateDerivatives(std::vector<std::vector<int>> keypoints,
//                            std::vector<double> &A,
//                            std::vector<double> &B,
//                            std::vector<double> &C,
//                            std::vector<double> &D,
//                            int dim_state, int dim_action, int T){
//
//    //TODO - DMackRus, this could be wrong in situations when qvel != qpos
//    int num_dof = dim_state / 2;
//
//    // Create an array to track start_indices of current interpolation per dof
//    int *start_indices = new int[num_dof];
//    for(int i = 0; i < num_dof; i++){
//        start_indices[i] = 0;
//    }
//
//    // Loop through all the time indices - can skip the first
//    // index as we preload the first index as the start index for all dofs.
//    for(int t = 1; t < T; t++){
//        // Loop through all the dofs
//        for(int dof = 0; dof < num_dof; dof++){
//            // Check if this time index contains any key-points for any dof.
//            std::vector<int> columns = keypoints[t];
//
//            // If there are no keypoints, continue onto next run of the loop
//            if(columns.size() == 0){
//                continue;
//            }
//
//            for(int j = 0; j < columns.size(); j++){
//
//                // If there is a match, interpolate between the start index and the current index
//                // For the given columns
//                if(dof == columns[j]){
//
//
//                    // A, B, C, D matrices are column
//
//                    // Starting index of top left of A matrix for last keypoint.
////                    int A_index_time = (start_indices[dof] * dim_state * dim_state);
//
//                    // Starting index of top left of B matrix for last keypoint.
////                    int B_index_time = (start_indices[dof] * dim_state * dim_action);
//
//
////                    int start_val_pos = A[A_index_time + (dof * dim_state)];
////                    int start_val_vel = A[A_index_time + (dim_state * (dim_state / 2)) + (dof * dim_state)];
////                    int start_val_ctrl = B[B_index_time + (dof * dim_action)];
//
//                    // Loop through from previous keypoint to now, interpolating all values
//                    // in the columns for that dof.
//                    for(int k = start_indices[dof]; k < t; k++){
//
//                    }
//
//
////                    cout << "dof: " << i << " end index: " << t << " start index: " << startIndices[i] << "\n";
////                    MatrixXd startACol1 = A[startIndices[i]].block(dof, i, dof, 1);
////                    MatrixXd endACol1 = A[t].block(dof, i, dof, 1);
////                    MatrixXd addACol1 = (endACol1 - startACol1) / (t - startIndices[i]);
////
////                    // Same again for column 2 which is dof + i
////                    MatrixXd startACol2 = A[startIndices[i]].block(dof, i + dof, dof, 1);
////                    MatrixXd endACol2 = A[t].block(dof, i + dof, dof, 1);
////                    MatrixXd addACol2 = (endACol2 - startACol2) / (t - startIndices[i]);
////
////                    if(i < num_ctrl){
////                        startB = B[startIndices[i]].block(dof, i, dof, 1);
////                        endB = B[t].block(dof, i, dof, 1);
////                        addB = (endB - startB) / (t - startIndices[i]);
////                    }
////
////                    for(int k = startIndices[i]; k < t; k++){
////                        A[k].block(dof, i, dof, 1) = startACol1 + ((k - startIndices[i]) * addACol1);
////
////                        A[k].block(dof, i + dof, dof, 1) = startACol2 + ((k - startIndices[i]) * addACol2);
////
////                        if(i < num_ctrl){
////                            B[k].block(dof, i, dof, 1) = startB + ((k - startIndices[i]) * addB);
////                        }
////                    }
//                    start_indices[dof] = t;
//                }
//            }
//        }
//    }
//}

double* GenerateJerkProfile(int T, const double* x, int dim_state){
    double* jerk_profile = new double[T * (dim_state/2)];

    return jerk_profile;
}

}  // namespace mjpc
