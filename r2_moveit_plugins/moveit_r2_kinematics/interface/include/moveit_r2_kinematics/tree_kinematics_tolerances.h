/* Author: Ryan Luna */

#ifndef MOVEIT_R2_TREE_KINEMATICS_TOLERANCES_
#define MOVEIT_R2_TREE_KINEMATICS_TOLERANCES_

namespace moveit_r2_kinematics
{
    /// \brief Critical (highest) priority task linear tolerance
    const double CRITICAL_PRIO_LINEAR_TOL = 1e-4;
    /// \brief Critical (highest) priority task angular tolerance (radians)
    const double CRITICAL_PRIO_ANGULAR_TOL = 1e-3;

    /// \brief High priority task linear tolerance
    const double HIGH_PRIO_LINEAR_TOL = 1e-1;
    /// \brief High priority task angular tolerance (radians)
    const double HIGH_PRIO_ANGULAR_TOL = 1e-1;

    /// \brief Medium priority task linear tolerance
    const double MEDIUM_PRIO_LINEAR_TOL = 1.0;
    /// \brief Medium priority task angular tolerance (radians)
    const double MEDIUM_PRIO_ANGULAR_TOL = 0.5;

    /// \brief Low priority task linear tolerance
    const double LOW_PRIO_LINEAR_TOL = 3.0;
    /// \brief Low priority task angular tolerance (radians)
    const double LOW_PRIO_ANGULAR_TOL = 1.0;

    /// \brief The maximum allowed joint velocity
    const double MAX_JOINT_VELOCITY = 10.0;

    /// \brief The maximum allowed twist
    const double MAX_TWIST = 0.5;
}

#endif