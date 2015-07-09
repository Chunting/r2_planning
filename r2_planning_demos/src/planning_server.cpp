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

#include <ros/ros.h>
#include <eigen_conversions/eigen_msg.h>
#include <moveit/move_group/capability_names.h>
#include <moveit/robot_state/conversions.h>
#include <r2_planning_interface/R2Interface.h>
#include <moveit_r2_kinematics/tree_kinematics_tolerances.h>

#include <moveit_msgs/GetMotionPlan.h>
#include <r2_planning_msgs/R2MotionPlanRequest.h>
#include <r2_planning_msgs/R2SimplePlanRequest.h>

static const char* const R2_GLOBAL_FRAME_NAME = "virtual_world";
static const char* const R2_PLANNING_SERVICE_NAME = "r2_motion_plan";
static const char* const R2_SIMPLE_PLANNING_SERVICE_NAME = "r2_simple_motion_plan";

class R2PlanningServer
{
public:
    R2PlanningServer(const std::string& global_frame) : global_frame_(global_frame)
    {
    }

    void start()
    {
        ROS_INFO("Starting R2 planning server...");

        // wait for move group to come online.  Timeout after 1 minute
        bool good = ros::service::waitForService(move_group::PLANNER_SERVICE_NAME, ros::Duration(60));
        if (!good)
            ROS_ERROR("R2 planning server: Timeout waiting for move_group to come online.  Motion planning may be unavailable");

        ros::ServiceServer service = nh.advertiseService(R2_PLANNING_SERVICE_NAME, &R2PlanningServer::planningService, this);
        ros::ServiceServer simpleservice = nh.advertiseService(R2_SIMPLE_PLANNING_SERVICE_NAME, &R2PlanningServer::simplePlanningService, this);
        ROS_INFO("R2 planning server online.  Services: %s %s", R2_PLANNING_SERVICE_NAME, R2_SIMPLE_PLANNING_SERVICE_NAME);

        // spin for-ev-er.  FOR-EV-ER.  FOR...EV...ER.
        ros::spin();
    }

protected:
    void setStartState(moveit_msgs::RobotState& state, const moveit_msgs::RobotState& req_state)
    {
        // Request provided a joint state
        if (req_state.joint_state.name.size() > 0 && req_state.joint_state.name.size() == req_state.joint_state.position.size())
            state = req_state;
        else
        {
            ROS_DEBUG("Setting initial state as current robot state");
            moveit::core::robotStateToRobotStateMsg(interface.getCurrentRobotState(), state);
        }
    }

    // Create a PositionConstraint message that constrains the given link to
    // the desired pose within the given tolerance.
    moveit_msgs::PositionConstraint createPositionConstraint(const std::string& link_name,
                                                             const geometry_msgs::PoseStamped& pose,
                                                             double tol = moveit_r2_kinematics::CRITICAL_PRIO_LINEAR_TOL)
    {
        moveit_msgs::PositionConstraint pos_constraint;

        // Setting position constraint for link_name
        pos_constraint.link_name = link_name;
        pos_constraint.target_point_offset.x = 0.0;
        pos_constraint.target_point_offset.y = 0.0;
        pos_constraint.target_point_offset.z = 0.0;
        pos_constraint.weight = 1.0;

        shape_msgs::SolidPrimitive box;
        box.type = shape_msgs::SolidPrimitive::BOX;
        box.dimensions.resize(3, tol);  // box with side length = tol
        pos_constraint.constraint_region.primitives.push_back(box);

        // Place the sphere at the link position
        pos_constraint.constraint_region.primitive_poses.push_back(pose.pose);
        pos_constraint.header.frame_id = pose.header.frame_id;

        return pos_constraint;
    }

    // Create an OrientationConstraint message that constrains the given link to the
    // orientation in pose_msg within the given tolerance on each axis.
    moveit_msgs::OrientationConstraint createOrientationConstraint(const std::string& link_name,
                                                                   const geometry_msgs::PoseStamped& pose_msg,
                                                                   double tol = moveit_r2_kinematics::CRITICAL_PRIO_ANGULAR_TOL)
    {
        moveit_msgs::OrientationConstraint or_constraint;

        // Create an orientation constraint for link_name
        or_constraint.link_name = link_name;
        or_constraint.orientation = pose_msg.pose.orientation;
        or_constraint.header.frame_id = pose_msg.header.frame_id;
        or_constraint.weight = 1.0;

        or_constraint.absolute_x_axis_tolerance = tol;
        or_constraint.absolute_y_axis_tolerance = tol;
        or_constraint.absolute_z_axis_tolerance = tol;
        return or_constraint;
    }

    // Create an OrientationConstraint message that restricts the orientation
    // of r2/robot_world to yaw only.
    moveit_msgs::OrientationConstraint createTorsoUpConstraint()
    {
        geometry_msgs::Quaternion q;
        q.x = 0.0;
        q.y = 0.0;
        q.z = 0.0;
        q.w = 1.0;

        // Constrain torso upright
        moveit_msgs::OrientationConstraint torso_orn_constraint;
        torso_orn_constraint.header.frame_id = global_frame_;
        torso_orn_constraint.orientation = q;
        torso_orn_constraint.link_name = "r2/robot_world";
        torso_orn_constraint.absolute_x_axis_tolerance = moveit_r2_kinematics::CRITICAL_PRIO_ANGULAR_TOL;
        torso_orn_constraint.absolute_y_axis_tolerance = moveit_r2_kinematics::CRITICAL_PRIO_ANGULAR_TOL;
        torso_orn_constraint.absolute_z_axis_tolerance = 6.283185;  // yaw is unconstrained (2pi)
        torso_orn_constraint.weight = 1.0;

        return torso_orn_constraint;
    }

    void setGoalConstraints(const std::string& goal_frame, const geometry_msgs::PoseStamped& goal_pose,
                            const std::string& secondary_frame, const std::vector<double>& secondary_xyzrpy,
                            std::vector<moveit_msgs::Constraints>& goal_constraints)
    {
        goal_constraints.resize(1);
        goal_constraints[0].position_constraints.push_back(createPositionConstraint(goal_frame, goal_pose));
        goal_constraints[0].orientation_constraints.push_back(createOrientationConstraint(goal_frame, goal_pose));

        if(secondary_frame.size())
        {
            if (secondary_xyzrpy.size() != 6)
                ROS_ERROR("Secondary frame xyzrpy not specified correctly");
            else
            {
                // position
                if (secondary_xyzrpy[0] > -998 || secondary_xyzrpy[1] > -998 || secondary_xyzrpy[2] > -998)
                {
                    // Position is constrained relative to pose (in global frame):
                    Eigen::Affine3d pose = Eigen::Affine3d::Identity();
                    if (secondary_xyzrpy[0] > -998) pose.translation()(0) = secondary_xyzrpy[0];
                    if (secondary_xyzrpy[1] > -998) pose.translation()(1) = secondary_xyzrpy[1];
                    if (secondary_xyzrpy[2] > -998) pose.translation()(2) = secondary_xyzrpy[2];

                    geometry_msgs::Pose pose_msg;
                    tf::poseEigenToMsg(pose, pose_msg);

                    moveit_msgs::PositionConstraint secondary_position;

                    // Setting position constraint
                    secondary_position.link_name = secondary_frame;
                    secondary_position.target_point_offset.x = 0.0;
                    secondary_position.target_point_offset.y = 0.0;
                    secondary_position.target_point_offset.z = 0.0;
                    secondary_position.weight = 1.0;

                    shape_msgs::SolidPrimitive box;
                    box.type = shape_msgs::SolidPrimitive::BOX;
                    box.dimensions.push_back(secondary_xyzrpy[0] > -998 ? moveit_r2_kinematics::HIGH_PRIO_LINEAR_TOL : 10);  // BOX_X
                    box.dimensions.push_back(secondary_xyzrpy[1] > -998 ? moveit_r2_kinematics::HIGH_PRIO_LINEAR_TOL : 10);  // BOX_Y
                    box.dimensions.push_back(secondary_xyzrpy[2] > -998 ? moveit_r2_kinematics::HIGH_PRIO_LINEAR_TOL : 10);  // BOX_Z
                    secondary_position.constraint_region.primitives.push_back(box);

                    secondary_position.constraint_region.primitive_poses.push_back(pose_msg);
                    secondary_position.header.frame_id = goal_pose.header.frame_id;

                    goal_constraints[0].position_constraints.push_back(secondary_position);
                    ROS_INFO("Added secondary position constraint");
                }

                // orientation
                if (secondary_xyzrpy[3] > -998 || secondary_xyzrpy[4] > -998 || secondary_xyzrpy[5] > -998)
                {
                    // Orientation is constrained relative to pose
                    Eigen::Affine3d pose = Eigen::Affine3d::Identity();

                    if (secondary_xyzrpy[3] > -998) pose *= Eigen::AngleAxisd(secondary_xyzrpy[3], Eigen::Vector3d::UnitX());
                    if (secondary_xyzrpy[4] > -998) pose *= Eigen::AngleAxisd(secondary_xyzrpy[4], Eigen::Vector3d::UnitY());
                    if (secondary_xyzrpy[5] > -998) pose *= Eigen::AngleAxisd(secondary_xyzrpy[5], Eigen::Vector3d::UnitZ());

                    geometry_msgs::Pose pose_msg;
                    tf::poseEigenToMsg(pose, pose_msg);

                    moveit_msgs::OrientationConstraint secondary_orientation;
                    secondary_orientation.link_name = secondary_frame;
                    secondary_orientation.orientation = pose_msg.orientation;
                    secondary_orientation.header.frame_id = goal_pose.header.frame_id;
                    secondary_orientation.weight = 1.0;

                    // set tolerances on orientation dimensions, 2pi if unconstrained
                    secondary_orientation.absolute_x_axis_tolerance = (secondary_xyzrpy[3] > -998 ? moveit_r2_kinematics::CRITICAL_PRIO_ANGULAR_TOL : 6.283185);
                    secondary_orientation.absolute_y_axis_tolerance = (secondary_xyzrpy[4] > -998 ? moveit_r2_kinematics::CRITICAL_PRIO_ANGULAR_TOL : 6.283185);
                    secondary_orientation.absolute_z_axis_tolerance = (secondary_xyzrpy[5] > -998 ? moveit_r2_kinematics::CRITICAL_PRIO_ANGULAR_TOL : 6.283185);

                    goal_constraints[0].orientation_constraints.push_back(secondary_orientation);
                    ROS_INFO("Added secondary orientation constraint");
                }
            }
        }
    }

    void setBaseFrame(const std::string& base_frame, const moveit_msgs::RobotState& start_state,
                      moveit_msgs::Constraints& path_constraints)
    {
        moveit::core::RobotStatePtr state = interface.allocRobotState();
        moveit::core::robotStateMsgToRobotState(start_state, *state);

        geometry_msgs::PoseStamped base_pose;
        base_pose.header.frame_id = global_frame_;
        Eigen::Affine3d pose = state->getGlobalLinkTransform(base_frame);
        tf::poseEigenToMsg(pose, base_pose.pose);

        path_constraints.position_constraints.push_back(createPositionConstraint(base_frame, base_pose));
        path_constraints.orientation_constraints.push_back(createOrientationConstraint(base_frame, base_pose));
    }

    std::string setPlanner(const std::string& group, const moveit_msgs::Constraints& path_constraints)
    {
        // Currently, only CBiRRT2 is capable of handling more than one pose constraint
        if (path_constraints.position_constraints.size() > 1 || path_constraints.orientation_constraints.size() > 1)
            return "CBiRRT2";

        // At most one pose constraint.  We can handle this.
        if (group != "legs")  // Moving one limb only (probably)
            return "RRTConnectkConfigDefault";
        else
            return "RRT";
    }

    bool simplePlanningService(r2_planning_msgs::R2SimplePlanRequest::Request& req,
                               r2_planning_msgs::R2SimplePlanRequest::Response& resp)
    {
        if (!ros::service::exists(move_group::PLANNER_SERVICE_NAME, false))
        {
            ROS_ERROR("Move_group services not online.  R2 planning service cannot continue.");
            return false;
        }

        ROS_INFO("R2 planning server received new simple plan request");

        // Open up a client to the planner service
        ros::ServiceClient plan_client = nh.serviceClient<moveit_msgs::GetMotionPlan>(move_group::PLANNER_SERVICE_NAME);

        // translating request into MoveIt request
        moveit_msgs::GetMotionPlan::Request  plan_req;
        moveit_msgs::GetMotionPlan::Response plan_resp;

        plan_req.motion_plan_request.group_name = req.group_name;

        setGoalConstraints(req.goal_frame,
                           req.goal_pose,
                           req.secondary_frame,
                           req.secondary_xyz_rpy,
                           plan_req.motion_plan_request.goal_constraints);

        setStartState(plan_req.motion_plan_request.start_state, req.start_state);

        if (req.base_frame.size() > 0)
            setBaseFrame(req.base_frame, plan_req.motion_plan_request.start_state, plan_req.motion_plan_request.path_constraints);

        if (req.torso_up)
            plan_req.motion_plan_request.path_constraints.orientation_constraints.push_back(createTorsoUpConstraint());

        if (req.planner_id.size() > 0)
            plan_req.motion_plan_request.planner_id = req.planner_id;
        else
            plan_req.motion_plan_request.planner_id = setPlanner(req.group_name, plan_req.motion_plan_request.path_constraints);

        if (req.num_planning_attempts > 0)
            plan_req.motion_plan_request.num_planning_attempts = req.num_planning_attempts;
        else
            plan_req.motion_plan_request.num_planning_attempts = 1;

        if (req.allowed_planning_time > 0)
            plan_req.motion_plan_request.allowed_planning_time = req.allowed_planning_time;
        else
            plan_req.motion_plan_request.allowed_planning_time = 10.0;

        // Update the allowed collision matrix
        std::vector<std::string> bodies;
        for(size_t i = 0; i < req.collision_updates.size(); ++i)
            interface.enableCollisionChecking(req.collision_updates[i].bodies, req.collision_updates[i].allow_collision, true);

        // Motion planning
        if (plan_client.call(plan_req, plan_resp))
        {
            resp.plan_response.trajectory_start = plan_resp.motion_plan_response.trajectory_start;
            resp.plan_response.group_name = plan_resp.motion_plan_response.group_name;
            resp.plan_response.visualization_trajectory = plan_resp.motion_plan_response.trajectory;

            // Remove the body pose information from the robot trajectory.  This is not an actuable joint.
            resp.plan_response.robot_trajectory = resp.plan_response.visualization_trajectory;
            resp.plan_response.robot_trajectory.multi_dof_joint_trajectory.joint_names.clear();
            resp.plan_response.robot_trajectory.multi_dof_joint_trajectory.points.clear();

            resp.plan_response.planning_time = plan_resp.motion_plan_response.planning_time;
            resp.plan_response.error_code = plan_resp.motion_plan_response.error_code;

            ROS_INFO("R2 planning server: motion plan successful");
            return true;
        }
        else
        {
            resp.plan_response.error_code = plan_resp.motion_plan_response.error_code;

            ROS_WARN("R2 planning server: motion plan failed");
            return false;
        }
    }

    bool planningService(r2_planning_msgs::R2MotionPlanRequest::Request& req,
                         r2_planning_msgs::R2MotionPlanRequest::Response& resp)
    {
        if (!ros::service::exists(move_group::PLANNER_SERVICE_NAME, false))
        {
            ROS_ERROR("Move_group services not online.  R2 planning service cannot continue.");
            return false;
        }

        ROS_INFO("R2 planning server received new plan request");

        // Open up a client to the planner service
        ros::ServiceClient plan_client = nh.serviceClient<moveit_msgs::GetMotionPlan>(move_group::PLANNER_SERVICE_NAME);

        // translating request into MoveIt request
        moveit_msgs::GetMotionPlan::Request  plan_req;
        moveit_msgs::GetMotionPlan::Response plan_resp;

        plan_req.motion_plan_request.start_state = req.plan_request.start_state;
        plan_req.motion_plan_request.goal_constraints = req.plan_request.goal_constraints;
        plan_req.motion_plan_request.path_constraints = req.plan_request.path_constraints;
        plan_req.motion_plan_request.planner_id = req.plan_request.planner_id;
        plan_req.motion_plan_request.group_name = req.plan_request.group_name;
        plan_req.motion_plan_request.num_planning_attempts = req.plan_request.num_planning_attempts;
        plan_req.motion_plan_request.allowed_planning_time = req.plan_request.allowed_planning_time;

        // Update the allowed collision matrix
        std::vector<std::string> bodies;
        for(size_t i = 0; i < req.plan_request.collision_updates.size(); ++i)
            interface.enableCollisionChecking(req.plan_request.collision_updates[i].bodies, req.plan_request.collision_updates[i].allow_collision, true);

        // Motion planning
        if (plan_client.call(plan_req, plan_resp))
        {
            resp.plan_response.trajectory_start = plan_resp.motion_plan_response.trajectory_start;
            resp.plan_response.group_name = plan_resp.motion_plan_response.group_name;
            resp.plan_response.visualization_trajectory = plan_resp.motion_plan_response.trajectory;

            // Remove the body pose information from the robot trajectory.  This is not an actuable joint.
            resp.plan_response.robot_trajectory = resp.plan_response.visualization_trajectory;
            resp.plan_response.robot_trajectory.multi_dof_joint_trajectory.joint_names.clear();
            resp.plan_response.robot_trajectory.multi_dof_joint_trajectory.points.clear();

            resp.plan_response.planning_time = plan_resp.motion_plan_response.planning_time;
            resp.plan_response.error_code = plan_resp.motion_plan_response.error_code;

            ROS_INFO("R2 planning server: motion plan successful");
            return true;
        }
        else
        {
            resp.plan_response.error_code = plan_resp.motion_plan_response.error_code;

            ROS_WARN("R2 planning server: motion plan failed");
            return false;
        }
    }

protected:
    // Global reference frame.  Assumed to be fixed with respect to the robot.
    std::string global_frame_;
    R2Interface interface;
    ros::NodeHandle nh;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "r2_planning_server");

    R2PlanningServer server(R2_GLOBAL_FRAME_NAME);
    sleep(1);
    server.start();  // this is a blocking call
}