/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2015, Rice University
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

#ifndef MOVEIT_OMPL_INTERFACE_R2_GEOMETRIC_PLANNING_CONTEXT_
#define MOVEIT_OMPL_INTERFACE_R2_GEOMETRIC_PLANNING_CONTEXT_

#include "moveit_ompl_pose_constraint_interface/GeometricPoseConstraintPlanningContext.h"

namespace ompl_interface
{

/// \brief Definition of a geometric planning context that respects a single pose constraint
class R2GeometricPlanningContext : public GeometricPoseConstraintPlanningContext
{
public:
    R2GeometricPlanningContext();

    virtual ~R2GeometricPlanningContext();

    virtual std::string getDescription();

    virtual void initialize(const std::string& ros_namespace, const PlanningContextSpecification& spec);

protected:
    /// \brief Allocate the StateSpace for the given specification.  This will initialize the
    /// \e mbss_ member.
    virtual void allocateStateSpace(const ModelBasedStateSpaceSpecification& state_space_spec);

    /// \brief Simplify the solution path (in simple setup).  Use no more than max_time seconds.
    virtual double simplifySolution(double max_time);

    kinematics::KinematicsBasePtr left_leg_kinematics_;
    kinematics::KinematicsBasePtr right_leg_kinematics_;
    kinematics::KinematicsBasePtr legs_kinematics_;
};

}

#endif