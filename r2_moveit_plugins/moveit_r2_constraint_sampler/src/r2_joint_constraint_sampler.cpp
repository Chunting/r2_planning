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

#include <ros/ros.h>
#include "moveit_r2_constraints/r2_joint_constraint_sampler.h"
#include <moveit/robot_state/robot_state.h>
#include <class_loader/class_loader.h>

moveit_r2_constraints::MoveItR2JointConstraintSamplerAllocator::MoveItR2JointConstraintSamplerAllocator() : constraint_samplers::ConstraintSamplerAllocator()
{
}

moveit_r2_constraints::MoveItR2JointConstraintSamplerAllocator::~MoveItR2JointConstraintSamplerAllocator()
{
}

constraint_samplers::ConstraintSamplerPtr moveit_r2_constraints::MoveItR2JointConstraintSamplerAllocator::alloc(const planning_scene::PlanningSceneConstPtr &scene,
                                                                                                                const std::string &group_name,
                                                                                                                const moveit_msgs::Constraints &constr)
{
    constraint_samplers::ConstraintSamplerPtr cs(new R2JointConstraintSampler(scene, group_name));
    cs->configure(constr);
    return cs;
}

bool moveit_r2_constraints::MoveItR2JointConstraintSamplerAllocator::canService(const planning_scene::PlanningSceneConstPtr &scene,
                                                                                const std::string &group_name,
                                                                                const moveit_msgs::Constraints &constr) const
{
    // If there are any joint constraints, this constraint sampler will be used
    if (constr.joint_constraints.size() < 1)
    {
        // ROS_ERROR("MoveItR2JointConstraintSamplerAllocator: The number of joint constraints must equal the number of joints in the model.  Only complete joint specifications are acceptable for the current joint constraint sampler.");
        return false;
    }

    return true;
}

moveit_r2_constraints::R2JointConstraintSampler::R2JointConstraintSampler(const planning_scene::PlanningSceneConstPtr &scene, const std::string &group_name)
    : constraint_samplers::ConstraintSampler(scene, group_name)
{
	complete_robot_state_ = new robot_state::RobotState(scene_->getRobotModel());
}

moveit_r2_constraints::R2JointConstraintSampler::~R2JointConstraintSampler()
{
    delete complete_robot_state_;
}

bool moveit_r2_constraints::R2JointConstraintSampler::configure(const moveit_msgs::Constraints &constr)
{
    clear();
    // It is assumed that the joint constraints already respect any additional constraints.
    if (constr.joint_constraints.size() != scene_->getRobotModel()->getVariableCount())
    {
        ROS_ERROR("The number of joint constraints specified does not equal the number of joints in the robot.");
        return false;
    }

    ROS_INFO("Configuring R2JointConstraintSampler for complete robot state");

    // Initializing a complete state for the robot that has all joints specified
    for(int i = 0; i < constr.joint_constraints.size(); ++i)
        complete_robot_state_->setVariablePosition(constr.joint_constraints[i].joint_name, constr.joint_constraints[i].position);
    complete_robot_state_->update();
    return true;
}

bool moveit_r2_constraints::R2JointConstraintSampler::sample(robot_state::RobotState &state,
                                                             const robot_state::RobotState &/*reference_state*/,
                                                             unsigned int /*max_attempts*/)
{
    state = *complete_robot_state_;
    return true;
}

bool moveit_r2_constraints::R2JointConstraintSampler::project(robot_state::RobotState &state,
                                                              unsigned int /*max_attempts*/)
{
    state = *complete_robot_state_;
    return true;
}

const std::string& moveit_r2_constraints::R2JointConstraintSampler::getName() const
{
    static const std::string SAMPLER_NAME = "R2JointConstraintSampler";
    return SAMPLER_NAME;
}

CLASS_LOADER_REGISTER_CLASS(moveit_r2_constraints::MoveItR2JointConstraintSamplerAllocator, constraint_samplers::ConstraintSamplerAllocator)