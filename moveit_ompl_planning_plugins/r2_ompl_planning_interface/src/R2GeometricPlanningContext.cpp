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

#include "r2_ompl_planning_interface/R2GeometricPlanningContext.h"
#include "r2_ompl_planning_interface/R2XXLPositionDecomposition.h"
#include "moveit_ompl_components/CartesianDistancePathSimplifier.h"

#include <pluginlib/class_loader.h>

#include <ompl/geometric/planners/hilo/XXL.h>
#include <ompl/geometric/planners/rlrt/RLRT.h>
#include <ompl/geometric/planners/rlrt/BiRLRT.h>

using namespace ompl_interface;

template<typename T>
static ompl::base::PlannerPtr allocateXXL(const ompl::base::SpaceInformationPtr &si,
                                          const std::string &new_name, const std::map<std::string, std::string>& params,
                                          ModelBasedStateSpacePtr mbss,
                                          kinematics::KinematicsBasePtr group_kinematics,
                                          kinematic_constraints::KinematicConstraintSetPtr constraints,
                                          const moveit_msgs::WorkspaceParameters& workspace,
                                          int slices)
{
    // Totally guessing the workspace bounds.
    // TODO: Need a better way to do this.  There is a workspace description in the motion_plan_request.
    // ompl::base::RealVectorBounds xyz(3);
    // xyz.low[0] = -3;
    // xyz.low[1] = -2.0; // large, since the foot can enter the stow closet thing // -1.1;
    // xyz.low[2] = -1.1;
    // xyz.high[0] = 1.1; // halfway ish down the module
    // xyz.high[1] = 1.1;
    // xyz.high[2] = 1.1;

    ompl::base::RealVectorBounds xyz(3);
    xyz.low[0] = workspace.min_corner.x;
    xyz.low[1] = workspace.min_corner.y; // large, since the foot can enter the stow closet thing // -1.1;
    xyz.low[2] = workspace.min_corner.z;
    xyz.high[0] = workspace.max_corner.x; // halfway ish down the module
    xyz.high[1] = workspace.max_corner.y;
    xyz.high[2] = workspace.max_corner.z;

    std::vector<int> xyzSlices(3, slices); // 3D, 'slices' cuts in each dimension
    bool diagonalEdges = true;

    ompl::geometric::XXLDecompositionPtr decomp(new R2XXLPositionDecomposition(xyz, xyzSlices, diagonalEdges, mbss, group_kinematics, constraints, si));

    ompl::base::PlannerPtr planner(new T(si, decomp));
    if (!new_name.empty())
        planner->setName(new_name);
    planner->params().setParams(params, true);
    return planner;
}

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


R2GeometricPlanningContext::R2GeometricPlanningContext() : GeometricFixedPosePlanningContext()
{
    GeometricPlanningContext::registerPlannerAllocator("geometric::XXL", boost::bind(&allocateXXL<ompl::geometric::XXL>, _1, _2, _3, mbss_, kinematics::KinematicsBasePtr(), kinematic_constraints::KinematicConstraintSetPtr(), request_.workspace_parameters, 4));
    GeometricPlanningContext::registerPlannerAllocator("geometric::XXL1", boost::bind(&allocateXXL<ompl::geometric::XXL>, _1, _2, _3, mbss_, kinematics::KinematicsBasePtr(), kinematic_constraints::KinematicConstraintSetPtr(), request_.workspace_parameters, 1));
    GeometricPlanningContext::registerPlannerAllocator("geometric::RLRT", boost::bind(&allocatePlanner<ompl::geometric::RLRT>, _1, _2, _3));
    GeometricPlanningContext::registerPlannerAllocator("geometric::BiRLRT", boost::bind(&allocatePlanner<ompl::geometric::BiRLRT>, _1, _2, _3));
}

R2GeometricPlanningContext::~R2GeometricPlanningContext()
{
}

std::string R2GeometricPlanningContext::getDescription()
{
    return "R2 Geometric Planning";
}

void R2GeometricPlanningContext::initialize(const std::string& ros_namespace, const PlanningContextSpecification& spec)
{
    if (spec.group != "legs")
        ROS_WARN("R2GeometricPlanningContext is meant for the legs group.  Received request for group %s", spec.group.c_str());

    // Extract IK solver.  Very important this is done prior to the call to R2GeometricPlanningContext::initialize
    const robot_model::JointModelGroup *legs = spec.model->getJointModelGroup("legs");
    if (!legs)
    {
        ROS_ERROR("R2GeometricPlanningContext: Could not find group configuration for 'legs'");
        return;
    }

    std::pair<robot_model::JointModelGroup::KinematicsSolver, robot_model::JointModelGroup::KinematicsSolverMap> slv = legs->getGroupKinematics();
    if (!slv.first)
        ROS_ERROR("Did not find kinematics solver for group legs");
    else
        legs_kinematics_ = slv.first.solver_instance_;

    GeometricFixedPosePlanningContext::initialize(ros_namespace, spec);

    // Configure the Cartesian space interpolator to do "normal" interpolation
    // for the following revolute joints:
    if (interpolator_)
    {
        interpolator_->ignoreJoint("r2/left_leg/joint2");
        interpolator_->ignoreJoint("r2/left_leg/joint4");
        interpolator_->ignoreJoint("r2/left_leg/joint6");
        interpolator_->ignoreJoint("r2/right_leg/joint2");
        interpolator_->ignoreJoint("r2/right_leg/joint4");
        interpolator_->ignoreJoint("r2/right_leg/joint6");
    }
}

void R2GeometricPlanningContext::allocateStateSpace(const ModelBasedStateSpaceSpecification& state_space_spec)
{
    // Call base class.  We are not actually overriding the state space part of this function
    GeometricFixedPosePlanningContext::allocateStateSpace(state_space_spec);

    // THIS IS ABSOLUTELY HORRIBLE.  MOVE THIS OUT OF HERE.
    // Do this (again) here to avoid seg fault because state space is not allocated in constructor and bind latches the parameter at execution time
    // Must also do it in the constructor, otherwise the context will not be found.
    // Must do it RIGHT HERE because the planner is allocated in the base class initialize call.
    GeometricPlanningContext::registerPlannerAllocator("geometric::XXL", boost::bind(&allocateXXL<ompl::geometric::XXL>, _1, _2, _3, mbss_, legs_kinematics_, path_constraints_, request_.workspace_parameters, 4));
    GeometricPlanningContext::registerPlannerAllocator("geometric::XXL1", boost::bind(&allocateXXL<ompl::geometric::XXL>, _1, _2, _3, mbss_, legs_kinematics_, path_constraints_, request_.workspace_parameters, 1));
}

// Simplify using the Cartesian distance minimizing interpolator
double R2GeometricPlanningContext::simplifySolution(double max_time)
{
    // bypass the fancy simplifier for now...
    return GeometricPlanningContext::simplifySolution(max_time);

    /////
    // Fancy simplifier that uses cartesian interpolator...

    ompl::time::point start = ompl::time::now();

    ompl::base::PlannerTerminationCondition ptc = ompl::base::timedPlannerTerminationCondition(max_time);

    bool cartesianInterpolator = useCartesianInterpolator(spec_.planner);

    CartesianDistancePathSimplifier simplifier(simple_setup_->getSpaceInformation(), interpolator_);
    simplifier.setPathConstraints(path_constraints_);

    ompl::geometric::PathGeometric simplifiedPath(simple_setup_->getSpaceInformation());
    ompl::geometric::PathGeometric &pg = simple_setup_->getSolutionPath();

    // The maximum length of a single segment in the solution path
    //double max_segment_length = (spec_.max_waypoint_distance > 0.0 ? spec_.max_waypoint_distance : simple_setup_->getStateSpace()->getMaximumExtent() / 100.0);
    // Computing the total number of waypoints we want in the solution path
    //unsigned int waypoint_count = std::max((unsigned int)floor(0.5 + pg.length() / max_segment_length), spec_.min_waypoint_count);

    // very important to set the interpolator label (1 for cartesian space interpolator, 0 for normal joint space interpolator)
    simplifier.simplify(ptc, pg, simplifiedPath, /*waypoint_count*/0 , cartesianInterpolator ? 1 : 0);

    pg = simplifiedPath;

    return ompl::time::seconds(ompl::time::now() - start);

    // // interpolate first, so that there is something to simplify
    // ompl::geometric::PathGeometric &pg = simple_setup_->getSolutionPath();
    // double max_segment_length = (spec_.max_waypoint_distance > 0.0 ? spec_.max_waypoint_distance : simple_setup_->getStateSpace()->getMaximumExtent() / 100.0);
    // unsigned int waypoint_count = std::max((unsigned int)floor(0.5 + pg.length() / max_segment_length), spec_.min_waypoint_count);
    // interpolateSolution(pg, waypoint_count);

    // ROS_INFO("Simplifying path using Cartesian interpolation scheme");
    // mbss_->setInterpolationFunction(boost::bind(&R2GeometricPlanningContext::interpolate, this, _1, _2, _3, _4, true));
    // // must do this for the fancy interpolator
    // simple_setup_->getSpaceInformation()->setMotionValidator(ompl::base::MotionValidatorPtr(new LinearMotionValidator(simple_setup_->getSpaceInformation())));

    //return GeometricPlanningContext::simplifySolution(max_time);
}

CLASS_LOADER_REGISTER_CLASS(ompl_interface::R2GeometricPlanningContext, ompl_interface::OMPLPlanningContext);