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

/* Author: Stephen Butler */

#ifndef MOVEIT_R2_CONSTRAINT_SAMPLER_R2_JOINT_CONSTRAINT_SAMPLER
#define MOVEIT_R2_CONSTRAINT_SAMPLER_R2_JOINT_CONSTRAINT_SAMPLER

#include <moveit/constraint_samplers/constraint_sampler_allocator.h>
#include <moveit/constraint_samplers/default_constraint_samplers.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_state/robot_state.h>

namespace moveit_r2_constraints
{

class MoveItR2JointConstraintSamplerAllocator : public constraint_samplers::ConstraintSamplerAllocator
{
public:
    MoveItR2JointConstraintSamplerAllocator();
    virtual ~MoveItR2JointConstraintSamplerAllocator();

    virtual constraint_samplers::ConstraintSamplerPtr alloc(const planning_scene::PlanningSceneConstPtr &scene, const std::string &group_name, const moveit_msgs::Constraints &constr);
    virtual bool canService(const planning_scene::PlanningSceneConstPtr &scene, const std::string &group_name, const moveit_msgs::Constraints &constr) const;
};

// A constraint sampler that returns a complete robot state.  This sampler is configured by specifying all joint positions.
class R2JointConstraintSampler : public constraint_samplers::ConstraintSampler
{
public:
    R2JointConstraintSampler(const planning_scene::PlanningSceneConstPtr &scene, const std::string &group_name);

    virtual ~R2JointConstraintSampler();

    /// \brief Configure this constraint sampler using a set of joint constraints.  Any additional constraints
    /// specified are presumed to be satisfied by the joint constraints.
    virtual bool configure(const moveit_msgs::Constraints &constr);


    /// \brief Returns the robot state as specified by the joint constraints
    /// reference_state and max_attempts are not used
    virtual bool sample(robot_state::RobotState &state,
                        const robot_state::RobotState &/*reference_state*/,
                        unsigned int /*max_attempts*/);

    /// \brief Returns the robot state as specified by the joint constraints
    /// max_attempts is not used
    virtual bool project(robot_state::RobotState &state,
                         unsigned int /*max_attempts*/);

    /// \brief Return the name of this constraint sampler
    virtual const std::string& getName() const;

protected:
    robot_state::RobotState *complete_robot_state_;
};

}

#endif