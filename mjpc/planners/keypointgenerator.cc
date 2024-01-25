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

    if(keypoint_method.name == "Set_Interval"){
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
    for(int i = 0; i < keypoints.size(); i++){
        std::cout << "timestep " << i << ": ";
        for(int j = 0; j < keypoints[i].size(); j++){
            std::cout << keypoints[i][j] << " ";
        }
        std::cout << "\n";
    }

    return keypoints;
}

double* GenerateJerkProfile(int T, const double* x, int dim_state){
    double* jerk_profile = new double[T * (dim_state/2)];

    return jerk_profile;
}
