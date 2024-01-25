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

class KeyPointGenerator{
    
};



#endif //MJPC_PLANNERS_KEYPOINTGENERATOR_H
