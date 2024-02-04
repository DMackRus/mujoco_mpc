/*
================================================================================
    File: keypointgenerator.h
    Author: David Russell (DMackRus)
    Date: January 25th, 2024
    Description:
        High level idea - Computing dynamics gradients via finite-differencing
        is computationally expensive. But gradient based methods enjoy faster
        convergence than sampling based planners. To speed up this computation,
        we can compute the dynamics gradients only at KEY-POINTS and interpolate
        approximations to the dynamics derivatives in-between via cheap interpolation.

        The ideas present in this class come from (https://github.com/DMackRus/TrajOptKP).

        This class provides methods for computing key-points PER degree of
        freedom for a model at which to compute dynamics gradients. This enables further
        speed up due to each column of the dynamics gradients requiring costly evaluations
        of the model dynamics.

        Implemented methods are:
        Set_Interval: (min_n) - Simply spaces out key-point at set intervals from one
        another.
        Adaptive_Jerk: (min_n, max_n, jerk_thresholds) - Places key-points at intervals
        bigger than min_n and smaller than max_n. This depends on the jerk of that
        degree of freedom in the trajectory. When jerk is high, we place more key-points.
        Velocity_Change: (min_n, max_n, velocity_change_thresholds) - Places key-points
        at intervals bigger than min_n and smaller than max_n. This depends on the
        velocity profile of the degree of freedom. When the velocity changes noticeably
        from the last key-point, we place a new key-point. Key-points are also placed
        at turning points, i.e when acceleration = 0.
================================================================================
*/

#ifndef MJPC_PLANNERS_KEYPOINTGENERATOR_H
#define MJPC_PLANNERS_KEYPOINTGENERATOR_H

#include <mujoco/mujoco.h>
#include <vector>
#include <chrono>
#include <string>
#include <iostream>

#include "mjpc/utilities.h"

namespace mjpc {

enum keypoint_method_names{
    SET_INTERVAL = 0,
    ADAPTIVE_JERK = 1,
    VELOCITY_CHANGE = 2
};

struct keypoint_method{
    std::string name;
    int min_N;
    int max_N;
    std::vector<double> jerk_thresholds;
    std::vector<double> accell_thresholds;
    std::vector<double> velocity_change_thresholds;
};

class KeypointGenerator{
public:
    // Constructor
    KeypointGenerator() = default;

    // Destructor
    ~KeypointGenerator() = default;

    /**
     * Generates a set of key-points per degree of freedom over a trajectory depending on the
     * key-point method specified. Usually takes into account the trajectory_states of the system.
     *
     * @param  keypoint_method The method and parameters used to generate key-points.
     * @param  T Optimisation horizon.
     * @param  x A sequence of state vectors over the trajectory.
     * @param  u A sequence of control vectors over the trajectory.
     *
     * @return std::vector<std::vector<int>> A set of key-points (integer indices over the trajectory) per
     * degree of freedom.
     */
    std::vector<std::vector<int>> GenerateKeyPoints(keypoint_method keypoint_method,
                                                    int T, const double* x, int dim_state);

    /**
     * Interpolates the state transition matrices (A, B, C, D) between key-points
     * via linear interpolation approximate a full set. Before this function is
     * called, the key-points must be generated and the dynamics derivatives
     * computed at each key-point via finite-differencing,
     *
     * @param keypoints - A 2D vector of keypoints. The first dimesnsion is
     *                    time along the trajectory. At each index, the integers
     *                    specify what degree of freedom to compute dynamics gradients
     *                    for.
     * @param A - Dynamics gradients w.r.t state vector (dim_state_derivative * dim_state_derivative * T)
     * @param B - Dynamics gradients w.r.t action vector (dim_state_derivative * dim_action * T)
     * @param C - Sensor gradients w.r.t state vector (dim_sensor * dim_state_derivative * T)
     * @param D - Sensor gradients w.r.t action vector (dim_sensor * dim_action * T)
     * @param dim_state - Dimension of the state vector
     * @param dim_action - Dimension of the action vector
     * @param dim_sensor - Dimension of the sensor vector
     * @param T - Optimisation horizon
     */
    void InterpolateDerivatives(std::vector<std::vector<int>> keypoints,
                                std::vector<double> &A,
                                std::vector<double> &B,
                                std::vector<double> &C,
                                std::vector<double> &D,
                                int dim_state, int dim_action, int dim_sensor, int T);

private:

    std::vector<std::vector<int>> GenerateKeypoints_AdaptiveJerk(keypoint_method keypoint_method,
                                                                 int T, int dim_state,
                                                                 double* jerk_profile);

    double* GenerateJerkProfile(int T, const double* x, int dim_state);




};

}  // namespace mjpc

#endif //MJPC_PLANNERS_KEYPOINTGENERATOR_H