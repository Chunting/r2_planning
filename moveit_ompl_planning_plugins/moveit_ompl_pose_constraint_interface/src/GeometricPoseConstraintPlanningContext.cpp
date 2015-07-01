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

#include "moveit_ompl_interpolation_interface/GeometricPoseConstraintPlanningContext.h"
#include "moveit/ompl_interface/parameterization/joint_space/joint_model_state_space.h"

#include <pluginlib/class_loader.h>

using namespace ompl_interface;

template<typename T>
static ompl::base::PlannerPtr allocatePlanner(const ompl::base::SpaceInformationPtr &si,
                                              const std::string &new_name, const std::map<std::string, std::string>& params)
{
    ompl::base::PlannerPtr planner(new T(si));
    if (!new_name.empty())
        planner->setName(new_name);
    planner->params().setParams(params, true);
    return planner;
}

GeometricPoseConstraintPlanningContext::GeometricPoseConstraintPlanningContext() : GeometricPlanningContext()
{
}

GeometricPoseConstraintPlanningContext::~GeometricPoseConstraintPlanningContext()
{
}

std::string GeometricPoseConstraintPlanningContext::getDescription()
{
    return "OMPL Geometric Pose Constraint Planning";
}

void GeometricPoseConstraintPlanningContext::initialize(const std::string& ros_namespace, const PlanningContextSpecification& spec)
{
    GeometricPlanningContext::initialize(ros_namespace, spec);

    if (!path_constraints_)
    {
        ROS_ERROR("GeometricPoseConstraintPlanningContext: No path constraints specified");
        initialized_ = false;
        return;
    }

    const std::vector<moveit_msgs::PositionConstraint>& pos_constraints = path_constraints_->getPositionConstraints();
    const std::vector<moveit_msgs::OrientationConstraint>& orn_constraints = path_constraints_->getOrientationConstraints();

    if (pos_constraints.size() != 1 || orn_constraints.size() != 1)
        ROS_WARN("Expected exactly one position and one orientation constraint");

    work_state_1_.reset(new robot_state::RobotState(mbss_->getRobotModel()));
    work_state_1_->setToDefaultValues(); // VERY IMPORTANT
    work_state_1_->update();
    work_state_2_.reset(new robot_state::RobotState(mbss_->getRobotModel()));
    work_state_2_->setToDefaultValues(); // VERY IMPORTANT
    work_state_2_->update();
    work_state_3_.reset(new robot_state::RobotState(mbss_->getRobotModel()));
    work_state_3_->setToDefaultValues(); // VERY IMPORTANT
    work_state_3_->update();

    // Setup interpolation function that also adjusts for exactly one pose constraint
    mbss_->setInterpolationFunction(boost::bind(&GeometricPoseConstraintPlanningContext::interpolate, this, _1, _2, _3, _4));
}

bool GeometricPoseConstraintPlanningContext::interpolate(const ompl::base::State *from, const ompl::base::State *to,
                                                         const double t, ompl::base::State *state)
{
    boost::mutex::scoped_lock(work_state_lock_);

    const robot_model::JointModelGroup* group = mbss_->getJointModelGroup();
    work_state_1_->setJointGroupPositions(group, from->as<ModelBasedStateSpace::StateType>()->values);
    work_state_1_->update();
    work_state_2_->setJointGroupPositions(group, to->as<ModelBasedStateSpace::StateType>()->values);
    work_state_2_->update();

    // Interpolate, then store the result in work_state_3
    group->interpolate(from->as<ModelBasedStateSpace::StateType>()->values, to->as<ModelBasedStateSpace::StateType>()->values,
                       t, state->as<ModelBasedStateSpace::StateType>()->values);

    work_state_3_->setJointGroupPositions(group, state->as<ModelBasedStateSpace::StateType>()->values);
    work_state_3_->update();

    // fix work_state_3_ to satisfy to the path constraints
    if (path_constraints_)
    {
        const std::vector<moveit_msgs::PositionConstraint>& pos_constraints = path_constraints_->getPositionConstraints();
        const std::vector<moveit_msgs::OrientationConstraint>& orn_constraints = path_constraints_->getOrientationConstraints();
        assert(pos_constraints.size() == 1);
        assert(orn_constraints.size() == 1);
        assert(pos_constraints[0].link_name == orn_constraints[0].link_name);

        const std::string& link_name = pos_constraints[0].link_name;

        // One of the input states must satisfy the path constraints - typically the 'from' state.
        // Depending on the planner, however, we may interpolate 'backward', so we have to figure out
        // at runtime which state is the satisfying one.
        if (path_constraints_->decide(*(work_state_2_.get())).satisfied)
            shiftStateByError(work_state_3_, work_state_3_->getRobotModel()->getRootJoint(),
                              link_name, work_state_2_->getGlobalLinkTransform(link_name));
        else
        {
            assert(path_constraints_->decide(*(work_state_1_.get())).satisfied);
            shiftStateByError(work_state_3_, work_state_3_->getRobotModel()->getRootJoint(),
                              link_name, work_state_1_->getGlobalLinkTransform(link_name));
        }
    }

    mbss_->copyToOMPLState(state, *(work_state_3_.get()));

    // Do NOT EVER return false from this function.
    // A false return will trigger the "default" interpolation scheme, which will violate all the constraints
    return true;
}

void GeometricPoseConstraintPlanningContext::shiftStateByError(robot_state::RobotStatePtr state, const robot_state::JointModel* base_joint,
                                                              const std::string& link, const Eigen::Affine3d& desired_pose) const
{
    if (base_joint->getType() != robot_state::JointModel::FLOATING)
    {
        ROS_ERROR("%s: Expected root joint '%s' to be floating.  This joint is %s",
                  __FUNCTION__, base_joint->getName().c_str(), base_joint->getTypeName().c_str());
        return;
    }

    Eigen::Affine3d actual_pose = state->getGlobalLinkTransform(link);

    // This is the error in the pose
    Eigen::Affine3d diff(desired_pose * actual_pose.inverse());

    // Apply the difference to the (unconstrained) base frame
    // VERY important that diff is multiplied on left side
    Eigen::Affine3d new_base_frame = diff * state->getGlobalLinkTransform(base_joint->getChildLinkModel()->getName());
    Eigen::Quaterniond q(new_base_frame.rotation());

    // Variable order is hard-coded for efficiency
    // This assumes a fixed variable order that "could" change in a future version of MoveIt
    std::vector<double> new_variables(7, 0.0);
    new_variables[0] = new_base_frame.translation()(0); // base_joint_model->getName() + "/trans_x"
    new_variables[1] = new_base_frame.translation()(1); // base_joint_model->getName() + "/trans_y"
    new_variables[2] = new_base_frame.translation()(2); // base_joint_model->getName() + "/trans_z"
    new_variables[3] = q.x();                           // base_joint_model->getName() + "/rot_x"
    new_variables[4] = q.y();                           // base_joint_model->getName() + "/rot_y"
    new_variables[5] = q.z();                           // base_joint_model->getName() + "/rot_z"
    new_variables[6] = q.w();                           // base_joint_model->getName() + "/rot_w"

    // Setting new base position
    state->setJointPositions(base_joint, new_variables);
    state->update();
}

void GeometricPoseConstraintPlanningContext::allocateStateSpace(const ModelBasedStateSpaceSpecification& state_space_spec)
{
    JointModelStateSpacePtr state_space_(new JointModelStateSpace(state_space_spec));
    mbss_ = boost::static_pointer_cast<ModelBasedStateSpace>(state_space_);
}

CLASS_LOADER_REGISTER_CLASS(ompl_interface::GeometricPoseConstraintPlanningContext, ompl_interface::OMPLPlanningContext);