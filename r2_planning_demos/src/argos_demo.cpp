// Navigation demo for R2 in ARGOS
// This demo relies on MoveGroup functionality for kinematics and planning
// Author: Ryan Luna
// May 2015

#include <vector>
#include <string>
#include <ros/ros.h>
#include <ros/serialization.h>
#include <Eigen/Dense>

#include <geometric_shapes/shapes.h>
#include <geometric_shapes/shape_operations.h>
#include <geometry_msgs/Pose.h>
#include <eigen_conversions/eigen_msg.h>

#include <moveit/kinematic_constraints/utils.h>
#include <moveit/robot_state/conversions.h>

#include "r2_planning_demos/argos_demo.h"
#include "moveit_r2_kinematics/tree_kinematics_tolerances.h"

static const std::string LEFT_GRIPPER_LINK0     = "r2/left_leg/gripper/jaw_left";
static const std::string LEFT_GRIPPER_LINK1     = "r2/left_leg/gripper/jaw_right";
static const std::string RIGHT_GRIPPER_LINK0    = "r2/right_leg/gripper/jaw_left";
static const std::string RIGHT_GRIPPER_LINK1    = "r2/right_leg/gripper/jaw_right";
static const std::string LEFT_FOOT_LINK         = "r2/left_leg_foot";
static const std::string RIGHT_FOOT_LINK        = "r2/right_leg_foot";

// Add the ARGOS gimbal arms to R2
void attachGimbalArms(R2Interface& interface, const Eigen::Affine3d& transform)
{
    // Add gimbal arms (box) to scene
    moveit_msgs::CollisionObject gimbal_arms;
    gimbal_arms.operation = moveit_msgs::CollisionObject::ADD;
    gimbal_arms.id = "Gimbal_Arms";
    gimbal_arms.header.frame_id = "virtual_world";

    shape_msgs::SolidPrimitive box_msg;
    box_msg.type = shape_msgs::SolidPrimitive::BOX;
    box_msg.dimensions.push_back(0.15); // X
    box_msg.dimensions.push_back(1.6); // Y
    box_msg.dimensions.push_back(0.4); // Z

    // Center of the box
    // Same as starting position for robot in argos
    geometry_msgs::Pose box_pose;
    tf::poseEigenToMsg(transform, box_pose);
    box_pose.position.z += 0.2;

    gimbal_arms.primitives.push_back(box_msg);
    gimbal_arms.primitive_poses.push_back(box_pose);

    // Add the gimbal arms box to the planning scene
    // Don't do this.  Just attach a new object to the robot directly
    //interface.addObstacleToPlanningScene(gimbal_arms);

    // Attach the gimbal arms to the robot
    moveit_msgs::AttachedCollisionObject attached_msg;
    // attach the handrail to the body cover
    attached_msg.link_name = "r2/body_cover";
    // allow a whole bunch of other links in the arms to collide with the gimbal arms
    attached_msg.touch_links.push_back("r2/body_cover");
    attached_msg.touch_links.push_back("r2/left_shoulder_roll");
    attached_msg.touch_links.push_back("r2/right_shoulder_roll");
    attached_msg.touch_links.push_back("r2/left_shoulder_pitch");
    attached_msg.touch_links.push_back("r2/right_shoulder_pitch");
    attached_msg.touch_links.push_back("r2/left_upper_arm");
    attached_msg.touch_links.push_back("r2/right_upper_arm");
    attached_msg.touch_links.push_back("r2/left_elbow");
    attached_msg.touch_links.push_back("r2/right_elbow");
    attached_msg.touch_links.push_back("r2/left_lower_arm");
    attached_msg.touch_links.push_back("r2/right_lower_arm");

    attached_msg.touch_links.push_back("r2/pelvis");
    attached_msg.touch_links.push_back("r2/waist_center");
    attached_msg.object = gimbal_arms;

    interface.attachObjectToRobot(attached_msg);
}

// Constrain the roll and pitch angles of the torso
// Default tolerance is HIGH rather than CRITICAL since ARGOS cannot perfectly constrain the
// roll and pitch of the body to zero.
void createARGOSTorsoConstraints(moveit_msgs::Constraints& constraints, double tol = moveit_r2_kinematics::HIGH_PRIO_ANGULAR_TOL)
{
    // Orientation is constrained relative to pose (global frame):
    Eigen::Affine3d I = Eigen::Affine3d::Identity();
    geometry_msgs::Pose pose_msg;
    tf::poseEigenToMsg(I , pose_msg);

    moveit_msgs::OrientationConstraint or_constraint;
    or_constraint.link_name = "r2/robot_world";
    or_constraint.orientation = pose_msg.orientation;
    or_constraint.header.frame_id = "virtual_world";
    or_constraint.weight = 1.0;

    // No roll or pitch allowed.  Yaw is unconstrained
    or_constraint.absolute_x_axis_tolerance = tol;
    or_constraint.absolute_y_axis_tolerance = tol;
    or_constraint.absolute_z_axis_tolerance = 6.283185; // 2*PI

    constraints.orientation_constraints.push_back(or_constraint);
}

// Constrain linkName to be at the given pose (position and orientation)
void createBaseLinkConstraints(moveit_msgs::Constraints& constraints, const std::string& linkName, geometry_msgs::Pose& pose,
                               double linearTol = moveit_r2_kinematics::CRITICAL_PRIO_LINEAR_TOL,
                               double angularTol = moveit_r2_kinematics::CRITICAL_PRIO_ANGULAR_TOL)
{
    moveit_msgs::PositionConstraint pos_constraint;
    std::string frame_id = "virtual_world";

    // Setting position constraint
    pos_constraint.link_name = linkName;
    pos_constraint.target_point_offset.x = 0.0;
    pos_constraint.target_point_offset.y = 0.0;
    pos_constraint.target_point_offset.z = 0.0;
    pos_constraint.weight = 1.0;

    shape_msgs::SolidPrimitive box;
    box.type = shape_msgs::SolidPrimitive::BOX;
    box.dimensions.push_back(linearTol);  // BOX_X
    box.dimensions.push_back(linearTol);  // BOX_Y
    box.dimensions.push_back(linearTol);  // BOX_Z
    pos_constraint.constraint_region.primitives.push_back(box);

    pos_constraint.constraint_region.primitive_poses.push_back(pose);
    pos_constraint.header.frame_id = frame_id;
    constraints.position_constraints.push_back(pos_constraint);

    moveit_msgs::OrientationConstraint or_constraint;

    // Create an orientation constraint for link_name
    or_constraint.link_name = linkName;
    or_constraint.orientation = pose.orientation;
    or_constraint.header.frame_id = frame_id;
    or_constraint.weight = 1.0;

    or_constraint.absolute_x_axis_tolerance = angularTol;
    or_constraint.absolute_y_axis_tolerance = angularTol;
    or_constraint.absolute_z_axis_tolerance = angularTol;

    constraints.orientation_constraints.push_back(or_constraint);
}

struct Step
{
    std::string base;
    std::string link;
    geometry_msgs::PoseStamped pose;

    bool torsoYaw;
    double yaw;

    bool torsoX;
    double x;

    bool torsoY;
    double y;

    bool torsoZ;
    double z;
};

// Create a position and orientation constraint for the moving link in the step
void createGoalConstraints(const Step& step,
                           std::vector<moveit_msgs::Constraints>& constraints,
                           double linearTol = moveit_r2_kinematics::CRITICAL_PRIO_LINEAR_TOL,
                           double angularTol = moveit_r2_kinematics::CRITICAL_PRIO_ANGULAR_TOL)
{
    moveit_msgs::Constraints goal;

    moveit_msgs::PositionConstraint position;
    position.link_name = step.link;
    position.target_point_offset.x = 0.0;
    position.target_point_offset.y = 0.0;
    position.target_point_offset.z = 0.0;
    position.weight = 1.0;

    shape_msgs::SolidPrimitive box;
    box.type = shape_msgs::SolidPrimitive::BOX;
    box.dimensions.push_back(linearTol);  // BOX_X
    box.dimensions.push_back(linearTol);  // BOX_Y
    box.dimensions.push_back(linearTol);  // BOX_Z
    position.constraint_region.primitives.push_back(box);

    position.constraint_region.primitive_poses.push_back(step.pose.pose);
    position.header.frame_id = step.pose.header.frame_id;
    goal.position_constraints.push_back(position);

    moveit_msgs::OrientationConstraint orientation;
    orientation.link_name = step.link;
    orientation.orientation = step.pose.pose.orientation;
    orientation.header.frame_id = step.pose.header.frame_id;
    orientation.weight = 1.0;

    orientation.absolute_x_axis_tolerance = angularTol;
    orientation.absolute_y_axis_tolerance = angularTol;
    orientation.absolute_z_axis_tolerance = angularTol;
    goal.orientation_constraints.push_back(orientation);

    // If a desired torso yaw is requested, also setup this constraint
    if (step.torsoYaw)
    {
        // Orientation is constrained relative to pose (global frame):
        Eigen::Affine3d pose = Eigen::Affine3d::Identity();
        pose *= Eigen::AngleAxisd(step.yaw, Eigen::Vector3d::UnitZ());
        geometry_msgs::Pose pose_msg;
        tf::poseEigenToMsg(pose, pose_msg);

        moveit_msgs::OrientationConstraint torso_constraint;
        torso_constraint.link_name = "r2/robot_world";
        torso_constraint.orientation = pose_msg.orientation;
        torso_constraint.header.frame_id = "virtual_world";
        torso_constraint.weight = 1.0;

        // roll and pitch are not constrained
        torso_constraint.absolute_x_axis_tolerance = 6.283185;
        torso_constraint.absolute_y_axis_tolerance = 6.283185;
        torso_constraint.absolute_z_axis_tolerance = angularTol;

        goal.orientation_constraints.push_back(torso_constraint);
    }

    // Desired x, y and/or z value
    if (step.torsoX || step.torsoY || step.torsoZ)
    {
        // Position is constrained relative to pose (in global frame):
        Eigen::Affine3d pose = Eigen::Affine3d::Identity();
        if (step.torsoX) pose.translation()(0) = step.x;
        if (step.torsoY) pose.translation()(1) = step.y;
        if (step.torsoZ) pose.translation()(2) = step.z;
        geometry_msgs::Pose pose_msg;
        tf::poseEigenToMsg(pose, pose_msg);

        moveit_msgs::PositionConstraint torso_position;
        std::string frame_id = "virtual_world";

        // Setting position constraint
        torso_position.link_name = "r2/robot_world";
        torso_position.target_point_offset.x = 0.0;
        torso_position.target_point_offset.y = 0.0;
        torso_position.target_point_offset.z = 0.0;
        torso_position.weight = 1.0;

        shape_msgs::SolidPrimitive box;
        box.type = shape_msgs::SolidPrimitive::BOX;
        box.dimensions.push_back(step.torsoX ? moveit_r2_kinematics::HIGH_PRIO_LINEAR_TOL : 10);  // BOX_X
        box.dimensions.push_back(step.torsoY ? moveit_r2_kinematics::HIGH_PRIO_LINEAR_TOL : 10);  // BOX_Y
        box.dimensions.push_back(step.torsoZ ? moveit_r2_kinematics::HIGH_PRIO_LINEAR_TOL : 10);  // BOX_Z
        torso_position.constraint_region.primitives.push_back(box);

        torso_position.constraint_region.primitive_poses.push_back(pose_msg);
        torso_position.header.frame_id = frame_id;

        goal.position_constraints.push_back(torso_position);
    }

    constraints.push_back(goal);
}

void letGripperTouchObject(R2Interface& interface, bool left, const std::string& object, bool allow)
{
    interface.enableCollisionChecking(left ? LEFT_GRIPPER_LINK0 : RIGHT_GRIPPER_LINK0, object, allow, true);
    interface.enableCollisionChecking(left ? LEFT_GRIPPER_LINK1 : RIGHT_GRIPPER_LINK1, object, allow, true);
    interface.enableCollisionChecking(left ? LEFT_FOOT_LINK : RIGHT_FOOT_LINK, object, allow, true); // true, means wait for ack
}

void writeTrajectory(const moveit_msgs::RobotTrajectory& trajectory, const std::string& filename)
{
    uint32_t size = ros::serialization::serializationLength(trajectory);
    ROS_INFO("Serializing trajectory of length %u", size);
    boost::shared_array<uint8_t> buffer(new uint8_t[size]);

    ros::serialization::OStream stream(buffer.get(), size);
    ros::serialization::serialize(stream, trajectory);

    std::ofstream fout(filename.c_str(), std::ios::out | std::ios::binary);
    if (!fout)
    {
        ROS_ERROR("Failed to open '%s' for writing", filename.c_str());
        return;
    }

    fout.write((char*) buffer.get(), size);
    fout.close();
}

bool loadTrajectory(moveit_msgs::RobotTrajectory& trajectory, const std::string& filename)
{
    std::ifstream fin;
    fin.open(filename.c_str(), std::ios::in | std::ios::binary);

    if (!fin)
    {
        ROS_ERROR("Failed to open '%s'", filename.c_str());
        return false;
    }

    fin.seekg(0, std::ios::end);
    std::streampos end = fin.tellg();
    fin.seekg(0, std::ios::beg);
    std::streampos begin = fin.tellg();

    uint32_t size = end - begin;
    ROS_INFO("Deserializing trajectory of length %u", size);
    boost::shared_array<uint8_t> buffer(new uint8_t[size]);
    fin.read((char*) buffer.get(), size);
    fin.close();

    ros::serialization::IStream stream(buffer.get(), size);
    ros::serialization::deserialize(stream, trajectory);

    return true;
}

class MyRobotTrajectory : public robot_trajectory::RobotTrajectory
{
public:
    MyRobotTrajectory(const robot_model::RobotModelConstPtr &kmodel, const std::string &group) : robot_trajectory::RobotTrajectory(kmodel, group)
    {
    }

    bool recoverTrajectory(const robot_state::RobotState& current_state, moveit_msgs::RobotTrajectory& trajectory)
    {
        // find closest state in the trajectory
        if (getWayPointCount() == 0)
        {
            ROS_ERROR("There are no waypoints in the trajectory to recover from");
            return false;
        }

        size_t closestIdx = 0;
        double dist = current_state.distance(getFirstWayPoint());

        for(size_t i = 1; i < getWayPointCount(); ++i)
        {
            double d = current_state.distance(getWayPoint(i));
            if (d < dist)
            {
                closestIdx = i;
                dist = d;
            }
        }

        ROS_INFO("Recovering trajectory.  Closest point is %lu of %lu, a distance of %f from current state", closestIdx+1, getWayPointCount(), dist);
        getRobotTrajectoryMsg(trajectory);
        if (closestIdx > 0)
        {
            trajectory.joint_trajectory.points.erase(trajectory.joint_trajectory.points.begin(), trajectory.joint_trajectory.points.begin() + closestIdx);
            if (trajectory.multi_dof_joint_trajectory.points.size() > closestIdx)
                trajectory.multi_dof_joint_trajectory.points.erase(trajectory.multi_dof_joint_trajectory.points.begin(), trajectory.multi_dof_joint_trajectory.points.begin() + closestIdx);
        }

        return true;
    }
};

int executeTrajectory(R2Interface& interface, moveit_msgs::RobotTrajectory& traj, bool execute)
{
    if (execute)
    {
        moveit_msgs::RobotTrajectory trajectory(traj);

        // Remove the body pose information from the trajectory.  This is not an actuable joint.
        trajectory.multi_dof_joint_trajectory.joint_names.clear();
        trajectory.multi_dof_joint_trajectory.points.clear();

        MyRobotTrajectory trajectoryRecovery(interface.getCurrentRobotState().getRobotModel(), "legs");
        trajectoryRecovery.setRobotTrajectoryMsg(interface.getCurrentRobotState(), traj);

        int val = interface.executeTrajectory(trajectory, true);  // true = block until the controller says execution is finished (or something bad happens)
        if (val == moveit_msgs::MoveItErrorCodes::PREEMPTED)
        {
            std::cout << "Preempt detected.  Attempt recovery? [y/n]" << std::endl;
            char ch;
            std::cin >> ch;
            if (ch == 'y' || ch == 'Y')
            {
                if (trajectoryRecovery.recoverTrajectory(interface.getCurrentRobotState(), trajectory))
                    traj = trajectory;
                else
                    ROS_ERROR("Failed to recover trajectory");
            }
            else
                ROS_WARN("NOT recovering from execution preempt");

            std::cout << "Execute trajectory returning " << val << std::endl;
        }
    }
    else
    {
        interface.simulateTrajectory(traj, false);  // false = don't simulate body moving.  The localizer should do this
        return moveit_msgs::MoveItErrorCodes::SUCCESS;
    }
}

bool takeStep(R2Interface& interface, const Step& step, bool execute, moveit_msgs::RobotTrajectory& trajectory)
{
    std::cout << "Planning for " << step.link << " with " << step.base << " as base" << std::endl;

    // Allocate a copy of the current robot state
    robot_state::RobotStatePtr currentState = interface.allocRobotState();
    *(currentState.get()) = interface.getCurrentRobotState();

    if (!currentState->hasAttachedBody("Gimbal_Arms"))
        ROS_WARN("Gimbal arms not attached to robot?");
    else
        ROS_WARN("Gimbal arms are attached");

    // start state
    moveit_msgs::RobotState start_state;
    moveit::core::robotStateToRobotStateMsg(*(currentState.get()), start_state);

    // Setup constraints along path
    moveit_msgs::Constraints path_constraints;

    // torso orientation constraints
    createARGOSTorsoConstraints(path_constraints);

    // base link pose constraint
    geometry_msgs::Pose base_pose;
    tf::poseEigenToMsg(currentState->getGlobalLinkTransform(step.base), base_pose);
    createBaseLinkConstraints(path_constraints, step.base, base_pose);

    // goal pose constraints
    std::vector<moveit_msgs::Constraints> goal_constraints;
    createGoalConstraints(step, goal_constraints);

    // Allow collisions on the fixed foot and the floor
    letGripperTouchObject(interface, step.base == "r2/left_leg/gripper/tip", "ISS_Deck", true);
    // Disallow collisions between moving link and floor
    letGripperTouchObject(interface, step.base != "r2/left_leg/gripper/tip", "ISS_Deck", false);
    if (step.base == "r2/left_leg/gripper/tip")
    {
        ROS_INFO("Disabling collisions with left foot and floor");
        ROS_INFO("Enabling collisions with right foot and floor");
    }
    else
    {
        ROS_INFO("Disabling collisions with right foot and floor");
        ROS_INFO("Enabling collisions with left foot and floor");
    }

    // Planning and execution
    bool executed = false;
    if (interface.plan(start_state, path_constraints, goal_constraints, "legs", 30.0, "CBiRRT2", trajectory))
    {
        std::cout << "Execute trajectory? [y/n] ";
        char ch;
        std::cin >> ch;
        if (ch == 'y')
        {

            std::cout << "Did you change the localizer to use " << step.base << " as base? [y/n] ";
            std::cin >> ch;
            if (ch == 'y')
            {
                executeTrajectory(interface, trajectory, execute);
                executed = true;
            }
        }
    }
    else
        ROS_WARN("Planning failed");

    return executed;
}

bool step1(R2Interface& interface, bool execute, moveit_msgs::RobotTrajectory& trajectory)
{
    Step step;
    step.base = "r2/right_leg/gripper/tip";
    step.link = "r2/left_leg/gripper/tip";
    step.pose.pose.position.x = -2.16;
    step.pose.pose.position.y =  0.45;
    step.pose.pose.position.z = -0.85;
    step.pose.pose.orientation.x = 0;
    step.pose.pose.orientation.y = -1;
    step.pose.pose.orientation.z = 0;
    step.pose.pose.orientation.w = 0;
    step.pose.header.frame_id = "virtual_world";

    step.torsoYaw = true;
    step.yaw = 0.0;

    step.torsoX = false;

    step.torsoY = false;
    //step.y = -0.15;

    step.torsoZ = false;

    return takeStep(interface, step, execute, trajectory);
}

bool step2(R2Interface& interface, bool execute, moveit_msgs::RobotTrajectory& trajectory)
{
    Step step;
    step.base = "r2/left_leg/gripper/tip";
    step.link = "r2/right_leg/gripper/tip";
    step.pose.pose.position.x = -1.60;
    step.pose.pose.position.y = -0.85337;
    step.pose.pose.position.z = -0.85;
    step.pose.pose.orientation.x = 0.70710678118;
    step.pose.pose.orientation.y = -0.70710678118;
    step.pose.pose.orientation.z = 0;
    step.pose.pose.orientation.w = 0;
    step.pose.header.frame_id = "virtual_world";

    step.torsoYaw = true;
    step.yaw = 1.57079;

    step.torsoX = false;
    step.torsoY = false;
    step.torsoZ = false;
    //step.z = -0.25;

    return takeStep(interface, step, execute, trajectory);
}

bool step3(R2Interface& interface, bool execute, moveit_msgs::RobotTrajectory& trajectory)
{
    Step step;
    step.base = "r2/right_leg/gripper/tip";
    step.link = "r2/left_leg/gripper/tip";
    step.pose.pose.position.x = -1.09;
    //step.pose.pose.position.y =  0.45;
    step.pose.pose.position.y =  0.05;
    step.pose.pose.position.z = -0.85;
    step.pose.pose.orientation.x = 0;
    step.pose.pose.orientation.y = -1;
    step.pose.pose.orientation.z = 0;
    step.pose.pose.orientation.w = 0;
    step.pose.header.frame_id = "virtual_world";

    step.torsoYaw = false;
    step.torsoX = false;
    step.torsoY = false;
    step.torsoZ = false;

    // step.torsoYaw = true;
    // step.yaw = 0.0;

    // step.torsoX = true;
    // step.x = -1.35;

    // step.torsoY = true;
    // step.y = -0.15;

    // step.torsoZ = true;
    // step.z = -0.172348;

    return takeStep(interface, step, execute, trajectory);
}

bool step4(R2Interface& interface, bool execute, moveit_msgs::RobotTrajectory& trajectory)
{
    Step step;
    step.base = "r2/left_leg/gripper/tip";
    step.link = "r2/right_leg/gripper/tip";
    step.pose.pose.position.x = -0.07;
    //step.pose.pose.position.y = 0.1;
    step.pose.pose.position.y = -0.1;
    step.pose.pose.position.z = -0.85;
    step.pose.pose.orientation.x = 0;
    step.pose.pose.orientation.y = 1;
    step.pose.pose.orientation.z = 0;
    step.pose.pose.orientation.w = 0;
    step.pose.header.frame_id = "virtual_world";

    step.torsoYaw = true;
    step.yaw = 1.57079;

    step.torsoX = true;
    step.x = -0.58;

    step.torsoY = true;
    step.y = 0.10;

    step.torsoZ = true;
    step.z = -0.25;

    return takeStep(interface, step, execute, trajectory);
}

bool step5(R2Interface& interface, bool execute, moveit_msgs::RobotTrajectory& trajectory)
{
    Step step;
    step.base = "r2/left_leg/gripper/tip";
    step.link = "r2/right_leg/gripper/tip";
    step.pose.pose.position.x = -1.40;
    step.pose.pose.position.y = -0.85337;
    step.pose.pose.position.z = -0.85;
    step.pose.pose.orientation.x = 0.70710678118;
    step.pose.pose.orientation.y = -0.70710678118;
    step.pose.pose.orientation.z = 0;
    step.pose.pose.orientation.w = 0;
    step.pose.header.frame_id = "virtual_world";

    // step.torsoYaw = false;
    // step.torsoX = false;
    // step.torsoY = false;
    // step.torsoZ = false;

    step.torsoYaw = true;
    step.yaw = 0.777803;

    step.torsoX = true;
    step.x = -1.35202;

    step.torsoY = true;
    step.y = -0.202012;

    step.torsoZ = true;
    step.z = -0.246897;

    return takeStep(interface, step, execute, trajectory);
}

bool step6(R2Interface& interface, bool execute, moveit_msgs::RobotTrajectory& trajectory)
{
    Step step;
    step.base = "r2/right_leg/gripper/tip";
    step.link = "r2/left_leg/gripper/tip";
    step.pose.pose.position.x = -2.16;
    step.pose.pose.position.y =  0.25;
    step.pose.pose.position.z = -0.85;
    step.pose.pose.orientation.x = 0;
    step.pose.pose.orientation.y = -1;
    step.pose.pose.orientation.z = 0;
    step.pose.pose.orientation.w = 0;
    step.pose.header.frame_id = "virtual_world";

    step.torsoYaw = true;
    step.yaw = 1.57079;

    step.torsoX = false;

    step.torsoY = false;
    //step.y = -0.05;

    step.torsoZ = false;
    //step.z = -0.30;

    return takeStep(interface, step, execute, trajectory);
}

bool step7(R2Interface& interface, bool execute, moveit_msgs::RobotTrajectory& trajectory)
{
    Step step;
    step.base = "r2/left_leg/gripper/tip";
    step.link = "r2/right_leg/gripper/tip";
    step.pose.pose.position.x = -3.0;
    step.pose.pose.position.y = -0.85;
    step.pose.pose.position.z = -0.85;
    step.pose.pose.orientation.x = 0.70710678118;
    step.pose.pose.orientation.y = -0.70710678118;
    step.pose.pose.orientation.z = 0;
    step.pose.pose.orientation.w = 0;
    step.pose.header.frame_id = "virtual_world";

    step.torsoYaw = true;
    step.yaw = 0.0;

    step.torsoX = false;
    //step.x = -2.87;

    step.torsoY = false;
    //step.y = -0.15;

    step.torsoZ = false;
    //step.z = -0.40;

    return takeStep(interface, step, execute, trajectory);
}

bool stepTurnaround(R2Interface& interface, bool execute, moveit_msgs::RobotTrajectory& trajectory)
{
    Step step;
    step.base = "r2/left_leg/gripper/tip";
    step.link = "r2/right_leg/gripper/tip";
    step.pose.pose.position.x = -2.16;
    step.pose.pose.position.y = -0.30;
    step.pose.pose.position.z = -0.85;
    step.pose.pose.orientation.x = 0.70710678118;
    step.pose.pose.orientation.y = -0.70710678118;
    step.pose.pose.orientation.z = 0;
    step.pose.pose.orientation.w = 0;
    step.pose.header.frame_id = "virtual_world";

    step.torsoYaw = true;
    step.yaw = -1.57079;

    step.torsoX = false;
    step.torsoY = false;
    step.torsoZ = false;
    step.z = -0.25;

    return takeStep(interface, step, execute, trajectory);
}

bool stepCustom(R2Interface& interface, bool execute, moveit_msgs::RobotTrajectory& trajectory)
{
    Step step;
    std::cout << "Left leg as base? [y/n] ";
    char ch;
    std::cin >> ch;
    if (ch == 'y' || ch == 'Y')
    {
        step.base = "r2/left_leg/gripper/tip";
        step.link = "r2/right_leg/gripper/tip";
    }
    else
    {
        step.base = "r2/right_leg/gripper/tip";
        step.link = "r2/left_leg/gripper/tip";
    }

    // orientation of the deck handrails wrt the link
    step.pose.pose.orientation.x = 0;
    step.pose.pose.orientation.y = -1;
    step.pose.pose.orientation.z = 0;
    step.pose.pose.orientation.w = 0;

    std::cout << "Current (global) position of " << step.link << ": " << interface.getCurrentRobotState().getGlobalLinkTransform(step.link).translation().transpose() << std::endl;
    std::cout << "Enter desired position (X Y Z) in global coordinates" << std::endl;
    std::cin >> step.pose.pose.position.x >> step.pose.pose.position.y >> step.pose.pose.position.z;

    step.pose.header.frame_id = "virtual_world";

    std::cout << "Moving " << step.link << " to " << step.pose.pose << std::endl;
    std::cout << "Continue? [y/n] ";
    std::cin >> ch;

    bool success = false;
    while (ch == 'y' || ch == 'Y')
    {
        success = takeStep(interface, step, execute, trajectory);

        if (!success)
        {
            std::cout << "Try again? [y/n] ";
            std::cin >> ch;
        }
        else
            ch = 'R'; // it's a good letter
    }

    return success;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "argos_demo");
    ros::AsyncSpinner spinner(0);  // use # threads = num cores
    spinner.start();

    ros::NodeHandle nh("argos_demo");
    bool execute = false;
    if (nh.hasParam("execute"))
        nh.getParam("execute", execute);

    if (execute)
        ROS_WARN("You are executing trajectories FOR REALS");
    else
        ROS_WARN("Will only simulate trajectories.  No execution");

    R2Interface r2Interface("robot_description");
    sleep(1);
    loadApproximateISSModelPlanningScene(r2Interface);  // argos_demo.h
    attachGimbalArms(r2Interface, r2Interface.getCurrentRobotState().getGlobalLinkTransform("r2/robot_world"));
    sleep(1);

    r2Interface.clearTrajectory();

    int ch = -1;
    bool previousSuccess = false;
    int maxNumSteps = 7;
    moveit_msgs::RobotTrajectory trajectory;
    do
    {
        if (trajectory.joint_trajectory.joint_names.size() > 0)
            r2Interface.viewTrajectory(trajectory);

        if (ch == -1 || !previousSuccess || ch >= maxNumSteps)
        {
            std::cout << "ARGOS Demo" << std::endl;
            std::cout << "Which step are we taking? " << std::endl;
            std::cout << " (1) Step 1 [Left leg moving]" << std::endl;
            std::cout << " (2) Step 2 [Right leg moving]" << std::endl;
            std::cout << " (3) Step 3 [Left leg moving]" << std::endl;
            std::cout << " (4) Step 4 [Right leg moving]" << std::endl;
            std::cout << " (5) Step 5 [Right leg moving]" << std::endl;
            std::cout << " (6) Step 6 [Left leg moving]" << std::endl;
            std::cout << " (7) Step 7 [Right leg moving]" << std::endl;
            //std::cout << " (8) Custom step" << std::endl;
            std::cout << " (8) Step Turnaround [Right leg moving]" << std::endl;
            std::cout << " (9) Save trajectory" << std::endl;
            std::cout << " (10) Load trajectory" << std::endl;
            std::cout << " (11) Execute trajectory" << std::endl;
            std::cout << " (0) EXIT" << std::endl;
            std::cin >> ch;
        }
        else if (previousSuccess && ch < maxNumSteps)
        {
            std::cout << "Take step " << ch+1 << "? [y/n] ";
            char choice;
            std::cin >> choice;
            if (choice == 'y')
                ch++;
            else
                ch = -1;
        }

        switch(ch)
        {
            case 0:
                break;
            case 1:
                previousSuccess = step1(r2Interface, execute, trajectory);
                break;
            case 2:
                previousSuccess = step2(r2Interface, execute, trajectory);
                break;
            case 3:
                previousSuccess = step3(r2Interface, execute, trajectory);
                break;
            case 4:
                previousSuccess = step4(r2Interface, execute, trajectory);
                break;
            case 5:
                previousSuccess = step5(r2Interface, execute, trajectory);
                break;
            case 6:
                previousSuccess = step6(r2Interface, execute, trajectory);
                break;
            case 7:
                previousSuccess = step7(r2Interface, execute, trajectory);
                break;
            case 8:
                previousSuccess = false;
                //stepCustom(r2Interface, execute, trajectory);
                stepTurnaround(r2Interface, execute, trajectory);
                ch = -1;
                break;

            case 9:
            {
                std::cout << "Enter filename to save to: ";
                std::string filename;
                std::cin >> filename;
                writeTrajectory(trajectory, filename);

                previousSuccess = false;
                ch = -1;
                break;
            }

            case 10:
            {
                std::cout << "Enter filename to load trajectory from: ";
                std::string filename;
                std::cin >> filename;
                if (loadTrajectory(trajectory, filename))
                    r2Interface.viewTrajectory(trajectory);

                previousSuccess = false;
                ch = -1;
                break;
            }

            case 11:
            {
                std::cout << "Is the base link set properly in the localizer? [y/n] ";
                char answer;
                std::cin >> answer;

                if (answer == 'y' || answer == 'Y')
                    executeTrajectory(r2Interface, trajectory, execute);

                previousSuccess = false;
                ch = -1;
                break;
            }

            default:
                previousSuccess = false;
                ch = -1;
                break;
        }
    } while (ros::ok() && ch != 0);
}