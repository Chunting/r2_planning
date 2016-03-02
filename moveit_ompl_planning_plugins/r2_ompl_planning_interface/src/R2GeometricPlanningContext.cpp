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
#include "moveit_ompl_components/TorsoDeclinationOptimizationObjective.h"

#include <pluginlib/class_loader.h>

#include <ompl/geometric/planners/hilo/XXL.h>
#include <ompl/geometric/planners/rlrt/RLRT.h>
#include <ompl/geometric/planners/rlrt/BiRLRT.h>

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

// Allocate the XXL planner that uses a 3D grid decomposition of the robot's workspace.
// Slices is the number of cells in the grid along each dimension; splits^3 total grid cells.
ompl::base::PlannerPtr R2GeometricPlanningContext::allocateXXL(const ompl::base::SpaceInformationPtr &si, const std::string &new_name,
                                                               const std::map<std::string, std::string>& params, int slices)
{
    const moveit_msgs::WorkspaceParameters& workspace = request_.workspace_parameters;

    ompl::base::RealVectorBounds xyz(3);
    xyz.low[0] = workspace.min_corner.x;
    xyz.low[1] = workspace.min_corner.y;
    xyz.low[2] = workspace.min_corner.z;
    xyz.high[0] = workspace.max_corner.x;
    xyz.high[1] = workspace.max_corner.y;
    xyz.high[2] = workspace.max_corner.z;

    std::vector<int> xyzSlices(3, slices); // 3D decomposition, 'slices' cuts in each dimension
    bool diagonalEdges = true;

    // Figure out which leg is 'fixed'
    const std::vector<moveit_msgs::PositionConstraint>& pos_constraints = path_constraints_->getPositionConstraints();
    const std::vector<moveit_msgs::OrientationConstraint>& orn_constraints = path_constraints_->getOrientationConstraints();

    // Complete hack to figure out what link is fixed.  Assuming there is only one link with both position and orientation constrained
    std::string fixed_link_name = "";
    for(size_t i = 0; i < pos_constraints.size(); ++i)
    {
        for(size_t j = 0; j < orn_constraints.size(); ++j)
            if (pos_constraints[i].link_name == orn_constraints[j].link_name)
                fixed_link_name = pos_constraints[i].link_name;
    }

    if (fixed_link_name == "")
    {
        ROS_ERROR("%s: Could not determine which link is the base", __FUNCTION__);
        throw;
    }

    // The list of links to project, in topological order
    std::vector<std::string> projectedLinks;

    // A set of "free" DoFs after planning for each link in the hierarchy
    // The first entry is always empty, since these DoFs are never 'free'
    std::vector<std::vector<std::string> >freeDoFs;

    projectedLinks.push_back("r2/robot_world"); // world is always first
    freeDoFs.push_back(std::vector<std::string>()); // this is always empty

    if (fixed_link_name.find("left") != std::string::npos) // left leg is fixed, so project the right foot
    {
        projectedLinks.push_back("r2/right_leg_foot");

        std::vector<std::string> dofs(7);
        dofs[0] = "r2/right_leg/joint0";
        dofs[1] = "r2/right_leg/joint1";
        dofs[2] = "r2/right_leg/joint2";
        dofs[3] = "r2/right_leg/joint3";
        dofs[4] = "r2/right_leg/joint4";
        dofs[5] = "r2/right_leg/joint5";
        dofs[6] = "r2/right_leg/joint6";
        freeDoFs.push_back(dofs);
    }
    else // right leg is fixed, so project the left foot
    {
        projectedLinks.push_back("r2/left_leg_foot");

        std::vector<std::string> dofs(7);
        dofs[0] = "r2/left_leg/joint0";
        dofs[1] = "r2/left_leg/joint1";
        dofs[2] = "r2/left_leg/joint2";
        dofs[3] = "r2/left_leg/joint3";
        dofs[4] = "r2/left_leg/joint4";
        dofs[5] = "r2/left_leg/joint5";
        dofs[6] = "r2/left_leg/joint6";
        freeDoFs.push_back(dofs);
    }

    // if we are planning for arms and legs, add in those links.  Order does not matter
    if (spec_.group == "legs_right_arm" || spec_.group == "legs_arms")
    {
        projectedLinks.push_back("r2/right_palm");

        std::vector<std::string> dofs(7);
        dofs[0] = "r2/right_arm/joint0";
        dofs[1] = "r2/right_arm/joint1";
        dofs[2] = "r2/right_arm/joint2";
        dofs[3] = "r2/right_arm/joint3";
        dofs[4] = "r2/right_arm/joint4";
        dofs[5] = "r2/right_arm/wrist/pitch";
        dofs[6] = "r2/right_arm/wrist/yaw";
        freeDoFs.push_back(dofs);
    }
    else if (spec_.group == "legs_left_arm" || spec_.group == "legs_arms")
    {
        projectedLinks.push_back("r2/left_palm");

        std::vector<std::string> dofs(7);
        dofs[0] = "r2/left_arm/joint0";
        dofs[1] = "r2/left_arm/joint1";
        dofs[2] = "r2/left_arm/joint2";
        dofs[3] = "r2/left_arm/joint3";
        dofs[4] = "r2/left_arm/joint4";
        dofs[5] = "r2/left_arm/wrist/pitch";
        dofs[6] = "r2/left_arm/wrist/yaw";
        freeDoFs.push_back(dofs);
    }

    //ompl::geometric::XXLDecompositionPtr decomp(new R2XXLPositionDecomposition(xyz, xyzSlices, diagonalEdges, mbss_, legs_kinematics_, path_constraints_, si));
    ompl::geometric::XXLDecompositionPtr decomp(new R2XXLPositionDecomposition(xyz, xyzSlices, diagonalEdges, mbss_, legs_kinematics_, projectedLinks, freeDoFs, fixed_link_name, si));

    ompl::base::PlannerPtr planner(new ompl::geometric::XXL(si, decomp));
    if (!new_name.empty())
        planner->setName(new_name);
    planner->params().setParams(params, true);
    return planner;
}

R2GeometricPlanningContext::R2GeometricPlanningContext() : GeometricFixedPosePlanningContext()
{
    GeometricPlanningContext::registerPlannerAllocator("geometric::XXL", boost::bind(&R2GeometricPlanningContext::allocateXXL, this, _1, _2, _3, 4)); // that 4 must look really confusing
    GeometricPlanningContext::registerPlannerAllocator("geometric::XXL1", boost::bind(&R2GeometricPlanningContext::allocateXXL, this, _1, _2, _3, 1));
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
    //if (spec.group != "legs")
    //    ROS_WARN("R2GeometricPlanningContext is meant for the legs group.  Received request for group %s", spec.group.c_str());
    if (spec.group.find("legs") != 0) // group should be prefixed with 'legs'
        ROS_WARN("R2GeometricPlanningContext is meant for the legs.  Received request for group %s", spec.group.c_str());

    // Extract IK solver.  Very important this is done prior to the call to R2GeometricPlanningContext::initialize
    const robot_model::JointModelGroup *group = spec.model->getJointModelGroup(spec.group);
    if (!group)
    {
        ROS_ERROR("R2GeometricPlanningContext: Could not find group configuration for '%s'", spec.group.c_str());
        return;
    }

    ROS_WARN("Initializing R2GeometricPlanningContext for group %s", spec.group.c_str());

    std::pair<robot_model::JointModelGroup::KinematicsSolver, robot_model::JointModelGroup::KinematicsSolverMap> slv = group->getGroupKinematics();
    if (!slv.first)
        ROS_ERROR("Did not find kinematics solver for group %s", spec.group.c_str());
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

    // total hack
    if (spec.planner == "TRRT")
    {
        // for TRRT, penalize torso deviation
        ROS_WARN("Using Torso Declination optimization objective");

        ompl::base::OptimizationObjectivePtr objective(new ompl_interface::TorsoDeclinationOptimizationObjective(this, "r2/head"));
        simple_setup_->setOptimizationObjective(objective);

        // Don't simplify.  This may make the path worse
        simplify_ = false;
    }
}

// Simplify using the Cartesian distance minimizing interpolator
// Probably not good to use this.  Interpolator strongly encourages
// self collisions.
/*double R2GeometricPlanningContext::simplifySolution(double max_time)
{
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
    int waypoint_count = 0;
    simplifier.simplify(ptc, pg, simplifiedPath, waypoint_count , cartesianInterpolator ? 1 : 0);

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
}*/

// setGoalConstraints will update the goal_constraints in the case where joint constraints are specified.
// For each moveit_msgs::Constraints with joint constraints, we will construct a complete robot state from the
// initial robot state where only those joints which have values in the moveit_msgs::Constraints will be updated.
// From this state we will construct a new moveit_msgs::Constraints in which every joint for the robot is specified.
bool R2GeometricPlanningContext::setGoalConstraints(const std::vector<moveit_msgs::Constraints> &goal_constraints, moveit_msgs::MoveItErrorCodes *error)
{
    // std::vector<moveit_msgs::Constraints> joint_goal_constraints = goal_constraints;
    std::vector<moveit_msgs::Constraints> joint_goal_constraints;
    for (std::vector<moveit_msgs::Constraints>::const_iterator goal_constraint = goal_constraints.begin();
        goal_constraint != goal_constraints.end();
        ++goal_constraint)
    {
        // Check if joint constraints are set
        // If a joint constraint is set it is assumed that the complete joint state of the robot is the goal.
        // If some joints are not specified in the constraint then we set these to the initial robot state for planning.
        if (goal_constraint->joint_constraints.size() > 0 &&
            goal_constraint->joint_constraints.size() < complete_initial_robot_state_->getVariableCount())
        {
            robot_state::RobotState goal_state(*complete_initial_robot_state_);
            // Each joint specified in the constraint will be updated in the goal_state.
            for(std::vector<moveit_msgs::JointConstraint>::const_iterator joint = goal_constraint->joint_constraints.begin();
                joint != goal_constraint->joint_constraints.end();
                ++ joint)
            {
                goal_state.setVariablePosition(joint->joint_name, joint->position);
            }
            goal_state.update();
            const std::vector<std::string> &nm = goal_state.getVariableNames();
            const double *positions = goal_state.getVariablePositions();
            moveit_msgs::Constraints joint_goal;
            // From the created goal state re-create the joint constraint based on each joint position in the robot.
            for(std::size_t i = 0; i < nm.size(); ++i)
            {
                moveit_msgs::JointConstraint joint_constraint;
                joint_constraint.joint_name = nm[i];
                joint_constraint.position = positions[i];
                // The tolerances and weight are not used but set with dummy values
                joint_constraint.tolerance_above = 0.0001;
                joint_constraint.tolerance_below = 0.0001;
                joint_constraint.weight = 1.0;
                joint_goal.joint_constraints.push_back(joint_constraint);
            }
            joint_goal_constraints.push_back(joint_goal);
        }
        else
        {
            // If no joint limits were specified then copy the goal constraint
            joint_goal_constraints.push_back(*goal_constraint);
        }
    }
    // Call the base class setGoalConstraints with our updated constraints.
    return GeometricPlanningContext::setGoalConstraints(joint_goal_constraints, error);
}

CLASS_LOADER_REGISTER_CLASS(ompl_interface::R2GeometricPlanningContext, ompl_interface::OMPLPlanningContext);