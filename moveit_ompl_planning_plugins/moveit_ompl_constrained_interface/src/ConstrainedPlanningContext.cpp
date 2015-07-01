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

#include "moveit_ompl_constrained_interface/ConstrainedPlanningContext.h"
#include "moveit_ompl_constrained_interface/OMPLConstraint.h"
#include "moveit/ompl_interface/parameterization/joint_space/joint_model_state_space.h"
#include "moveit/ompl_interface/detail/constrained_goal_sampler.h"
#include "moveit/ompl_interface/detail/goal_union.h"
#include <pluginlib/class_loader.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/Constraints.h>

// Add planner includes here
#include <ompl/geometric/planners/rrt/CBiRRT2.h>


namespace og = ompl::geometric;

using namespace moveit_ompl_constrained_interface;


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

ConstrainedPlanningContext::ConstrainedPlanningContext() : ompl_interface::GeometricPlanningContext()
{
    // Clear the geometric planner allocators and create our own.
    planner_allocators_.clear();
    registerPlannerAllocator("geometric::CBiRRT2", boost::bind(&allocatePlanner<og::CBiRRT2>, _1, _2, _3));

    // Do NOT interpolate or try to simplify constrained plans
    // This will violate the constraints
    interpolate_ = simplify_ = false;
}

ConstrainedPlanningContext::~ConstrainedPlanningContext()
{
}

std::string ConstrainedPlanningContext::getDescription()
{
    return "OMPL Constrained Planning";
}

void ConstrainedPlanningContext::clear()
{
    ompl_interface::GeometricPlanningContext::clear();
    path_constraints_->clear();
}

void ConstrainedPlanningContext::initialize(const std::string& ros_namespace, const ompl_interface::PlanningContextSpecification& spec)
{
    spec_ = spec;

    // Erase the type and plugin fields from the configuration items
    std::map<std::string, std::string>::iterator it = spec_.config.find("type");
    if (it != spec_.config.end())
    {
        planner_id_ = it->second;
        spec_.config.erase(it);
    }
    else
        ROS_WARN("No planner type specified.  Using default planner configuration");

    it = spec_.config.find("plugin");
    if (it != spec_.config.end())
        spec_.config.erase(it);

    OMPLPlanningContext::initialize(ros_namespace, spec_);
    constraint_sampler_manager_ = spec_.constraint_sampler_mgr;

    if (!complete_initial_robot_state_)
        complete_initial_robot_state_ = new robot_state::RobotState(spec_.model);

    ROS_INFO("Initializing ConstrainedPlanningContext for '%s'", spec_.planner.c_str());

    // State space
    ompl_interface::ModelBasedStateSpaceSpecification state_space_spec(spec_.model, spec_.group);
    ompl_interface::JointModelStateSpacePtr state_space_(new ompl_interface::JointModelStateSpace(state_space_spec));
    mbss_ = boost::static_pointer_cast<ompl_interface::ModelBasedStateSpace>(state_space_);

    simple_setup_.reset(new ompl::geometric::ConstrainedSimpleSetup(state_space_));

    // Initializing path constraints
    // There must be some constraints for this context to be valid
    if (request_.path_constraints.position_constraints.size() == 0 && request_.path_constraints.orientation_constraints.size() == 0 &&
        request_.path_constraints.joint_constraints.size() == 0    && request_.path_constraints.visibility_constraints.size() == 0)
    {
        ROS_ERROR("%s: No path constraints specified.  Cannot initialize planning context.", getDescription().c_str());
        initialized_ = false;
        return;
    }

    path_constraints_msg_ = request_.path_constraints;
    path_constraints_.reset(new kinematic_constraints::KinematicConstraintSet(getRobotModel()));
    path_constraints_->add(request_.path_constraints, getPlanningScene()->getTransforms());

    // Configure constraint sampler
    constraint_samplers::ConstraintSamplerPtr cs;
    cs = constraint_sampler_manager_->selectSampler(getPlanningScene(), getGroupName(), path_constraints_->getAllConstraints());
    if (cs)
    {
        ompl::base::ConstraintInformationPtr ci(new ompl::base::ConstraintInformation());
        ompl::base::ConstraintPtr constraint(new OMPLKinematicConstraints(mbss_, path_constraints_, cs));
        ci->addConstraint(constraint);
        boost::static_pointer_cast<ompl::geometric::ConstrainedSimpleSetup>(simple_setup_)->getConstrainedSpaceInformation()->setConstraintInformation(ci);
    }
    else
        ROS_ERROR("%s: Failed to configure constraint sampler", getDescription().c_str());

    // Projection evaluator
    it = spec_.config.find("projection_evaluator");
    if (it != spec_.config.end())
    {
        setProjectionEvaluator(boost::trim_copy(it->second));
        spec_.config.erase(it);
    }

    // Planner
    ompl::base::PlannerPtr planner = configurePlanner(planner_id_, spec_.config);
    simple_setup_->setPlanner(planner);

    // Mark the context as initialized
    initialized_ = true;
}

ompl::base::PlannerPtr ConstrainedPlanningContext::configurePlanner(const std::string& planner_name, const std::map<std::string, std::string>& params)
{
    std::map<std::string, PlannerAllocator>::const_iterator it = planner_allocators_.find(planner_name);
    // Allocating planner using planner allocator
    if (it != planner_allocators_.end())
        return it->second(simple_setup_->getSpaceInformation(), "", params);

    // No planner configured by this name
    ROS_WARN("No planner allocator found with name '%s'", planner_name.c_str());
    for(std::map<std::string, PlannerAllocator>::const_iterator it = planner_allocators_.begin(); it != planner_allocators_.end(); ++it)
        ROS_WARN("  %s", it->first.c_str());
    return ompl::base::PlannerPtr();

}

CLASS_LOADER_REGISTER_CLASS(moveit_ompl_constrained_interface::ConstrainedPlanningContext, ompl_interface::OMPLPlanningContext);