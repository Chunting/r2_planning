/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2014, Rice University
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Rice University nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Ryan Luna */

#ifndef MOVEIT_OMPL_INTERFACE_GEOMETRIC_FIXED_POSE_PLANNING_CONTEXT_
#define MOVEIT_OMPL_INTERFACE_GEOMETRIC_FIXED_POSE_PLANNING_CONTEXT_

#include "moveit/ompl_interface/geometric_planning_context.h"
#include "moveit_ompl_components/CartesianSpaceInterpolator.h"
#include <boost/thread/mutex.hpp>


namespace ompl_interface
{

/// \brief Definition of a geometric planning context that respects a SINGLE pose constraint
///
/// This context operates by planning in joint space using the typical methods, but augments
/// the interpolation process to ensure an arbitrary base link remains fixed during planning.
/// Useful for humanoid type robots whose kinematic base may change during execution.
class GeometricFixedPosePlanningContext : public GeometricPlanningContext
{
public:
    GeometricFixedPosePlanningContext();

    virtual ~GeometricFixedPosePlanningContext();

    virtual std::string getDescription();

    virtual void initialize(const std::string& ros_namespace, const PlanningContextSpecification& spec);

protected:
    /// \brief Allocate the StateSpace for the given specification.  This will initialize the
    /// \e mbss_ member.
    virtual void allocateStateSpace(const ModelBasedStateSpaceSpecification& state_space_spec);

    // Distance a to b using cartesian joint coordinates instead of joint angles
    double cartesianDistance(const ompl::base::State *a, const ompl::base::State *b) const;

    /// \brief Interpolate between from and to using the given parameters.
    bool interpolate(const ompl::base::State *from, const ompl::base::State *to,
                     const double t, ompl::base::State *state, bool cartesianInterpolator);

    // In the given state, force link to achieve desired_pose (in the world frame) by adjusting
    // the base_joint.  base_joint MUST be a floating joint.
    void shiftStateByError(robot_state::RobotStatePtr state, const robot_state::JointModel* base_joint,
                           const std::string& link, const Eigen::Affine3d& desired_pose) const;

    /// \brief Return true if we are using the Cartesian distance minimizing interpolator
    bool useCartesianInterpolator(const std::string& planner) const;

    /// \brief Thread-safe scratch space for the context.
    boost::mutex work_state_lock_;
    robot_state::RobotStatePtr work_state_1_;
    robot_state::RobotStatePtr work_state_2_;
    robot_state::RobotStatePtr work_state_3_;

    /// \brief Name of the link whose pose is fixed during planning
    std::string fixed_link_;

    CartesianSpaceInterpolator* interpolator_;

};

}

#endif