// Navigation demo for R2 in the US Lab of the ISS
// Author: Ryan Luna
// May 2015

#include <ros/ros.h>
#include <moveit_msgs/WorkspaceParameters.h>
#include <eigen_conversions/eigen_msg.h>
#include <moveit/robot_state/conversions.h>

#include "r2_planning_interface/R2Interface.h"
#include "r2_planning_interface/ISSWorld.h"
#include <moveit_r2_kinematics/tree_kinematics_tolerances.h>

#include <moveit/move_group/capability_names.h>
#include <moveit_msgs/GetMotionPlan.h>

static const std::string LEFT_LEG_END_LINK      = "r2/left_leg/gripper/tip"; //"r2/left_ankle_roll";
static const std::string RIGHT_LEG_END_LINK     = "r2/right_leg/gripper/tip"; //"r2/right_ankle_roll";
static const std::string LEFT_GRIPPER_LINK0     = "r2/left_leg/gripper/jaw_left";
static const std::string LEFT_GRIPPER_LINK1     = "r2/left_leg/gripper/jaw_right";
static const std::string RIGHT_GRIPPER_LINK0    = "r2/right_leg/gripper/jaw_left";
static const std::string RIGHT_GRIPPER_LINK1    = "r2/right_leg/gripper/jaw_right";
static const std::string LEFT_FOOT_LINK         = "r2/left_leg_foot";
static const std::string RIGHT_FOOT_LINK        = "r2/right_leg_foot";

// Create a trajectory that opens/closes the left/right gripper
void setGripper(moveit_msgs::RobotTrajectory& trajectory, bool right, bool open)
{
    trajectory.joint_trajectory.joint_names.push_back(right ? "r2/right_leg/gripper/jawLeft" : "r2/left_leg/gripper/jawLeft");
    trajectory.joint_trajectory.joint_names.push_back(right ? "r2/right_leg/gripper/jawRight" : "r2/left_leg/gripper/jawRight");

    // This is the joint angle of the gripper
    double closed_position = 0.0;
    double open_position = 0.75;

    trajectory_msgs::JointTrajectoryPoint closed_pt;
    closed_pt.positions.push_back(closed_position);
    closed_pt.positions.push_back(closed_position);
    closed_pt.time_from_start = ros::Duration(open ? 0.0 : 1.0);

    trajectory_msgs::JointTrajectoryPoint open_pt;
    open_pt.positions.push_back(open_position);
    open_pt.positions.push_back(open_position);
    open_pt.time_from_start = ros::Duration(open ? 1.0 : 0.0);

    trajectory.joint_trajectory.points.push_back(open ? closed_pt : open_pt);
    trajectory.joint_trajectory.points.push_back(open ? open_pt : closed_pt);
}

/// Open and close the grippers of the legs to match the grasping actions
/// in the given set of trajectories.  It is assumed that the legs alternate
/// between open an close.  leftLegFixed=true indicates that the left leg
/// gripper should be locked during the first trajectory.
void openAndCloseGrippers(std::vector<moveit_msgs::RobotTrajectory>& trajectories, bool leftLegFixed)
{
    // Open and close the appropriate grippers between each trajectory
    std::vector<moveit_msgs::RobotTrajectory> newTrajectories;
    for(size_t i = 0; i < trajectories.size(); ++i)
    {
        // open the gripper for the leg that is moving during this trajectory
        moveit_msgs::RobotTrajectory openGripperTrajectory;
        setGripper(openGripperTrajectory, leftLegFixed, true);
        newTrajectories.push_back(openGripperTrajectory);

        newTrajectories.push_back(trajectories[i]);

        // close the gripper for the leg that moved in trajectories[i]
        moveit_msgs::RobotTrajectory closeGripperTrajectory;
        setGripper(closeGripperTrajectory, leftLegFixed, false);
        newTrajectories.push_back(closeGripperTrajectory);

        leftLegFixed = !leftLegFixed;
    }

    trajectories.swap(newTrajectories);
}

/// Create a PositionConstraint message that constrains the given link to
/// the desired pose within the given tolerance.
moveit_msgs::PositionConstraint createPositionConstraint(const std::string& link_name,
                                                         const geometry_msgs::Pose& pose_msg,
                                                         double tol)
{
    moveit_msgs::PositionConstraint pos_constraint;

    // This pose is with respect to the kinematic root
    std::string frame_id = "virtual_world";

    // Setting position constraint for link_name
    pos_constraint.link_name = link_name;
    pos_constraint.target_point_offset.x = 0.0;
    pos_constraint.target_point_offset.y = 0.0;
    pos_constraint.target_point_offset.z = 0.0;
    pos_constraint.weight = 10.0;

    shape_msgs::SolidPrimitive box;
    box.type = shape_msgs::SolidPrimitive::BOX;
    box.dimensions.push_back(tol);  // BOX_X
    box.dimensions.push_back(tol);  // BOX_Y
    box.dimensions.push_back(tol);  // BOX_Z
    pos_constraint.constraint_region.primitives.push_back(box);

    // Place the sphere at the link position
    pos_constraint.constraint_region.primitive_poses.push_back(pose_msg);
    pos_constraint.header.frame_id = frame_id;

    return pos_constraint;
}

/// Create a PositionConstraint message that constrains the given link to its
/// current position in state within the given tolerance.
moveit_msgs::PositionConstraint createPositionConstraint(const std::string& link_name,
                                                         const robot_state::RobotState& state,
                                                         double tol)
{
    if (!state.getRobotModel()->hasLinkModel(link_name))
        throw std::runtime_error("Link does not exist");

    // Getting link pose from state
    geometry_msgs::Pose pose_msg;
    Eigen::Affine3d pose = state.getGlobalLinkTransform(link_name);
    tf::poseEigenToMsg(pose, pose_msg);

    return createPositionConstraint(link_name, pose_msg, tol);
}

/// Create an OrientationConstraint message that constrains the given link to the
/// orientation in pose_msg within the given tolerance on each axis.
moveit_msgs::OrientationConstraint createOrientationConstraint(const std::string& link_name,
                                                               geometry_msgs::Pose pose_msg,
                                                               double tol)
{
    moveit_msgs::OrientationConstraint or_constraint;

    // This pose is with respect to the kinematic root
    std::string frame_id = "virtual_world";

    // Create an orientation constraint for link_name
    or_constraint.link_name = link_name;
    or_constraint.orientation = pose_msg.orientation;
    or_constraint.header.frame_id = frame_id;
    or_constraint.weight = 10.0;

    or_constraint.absolute_x_axis_tolerance = tol;
    or_constraint.absolute_y_axis_tolerance = tol;
    or_constraint.absolute_z_axis_tolerance = tol;
    return or_constraint;
}

/// Create an OrientationConstraint message that constrains the given link to its
/// current orientation in state within the given tolerance.
moveit_msgs::OrientationConstraint createOrientationConstraint(const std::string& link_name,
                                                               const robot_state::RobotState& state,
                                                               double tol)
{
    if (!state.getRobotModel()->hasLinkModel(link_name))
        throw std::runtime_error("Link does not exist");

    // Getting link pose from state
    geometry_msgs::Pose pose_msg;
    Eigen::Affine3d pose = state.getGlobalLinkTransform(link_name);
    tf::poseEigenToMsg(pose, pose_msg);

    return createOrientationConstraint(link_name, pose_msg, tol);
}

// Initialize the planning scene from the world representation
void createPlanningScene(R2Interface& interface, const ISSWorld& world)
{
    // Add ISS to scene
    moveit_msgs::CollisionObject iss_msg;
    world.getISSCollisionObjectMessage(iss_msg);
    interface.addObstacleToPlanningScene(iss_msg);

    // Add all handrails to scene
    for(unsigned int i = 0; i < world.numHandrails(); ++i)
    {
        moveit_msgs::CollisionObject obj_msg;
        world.getHandrailCollisionObjectMessage(i, obj_msg);
        interface.addObstacleToPlanningScene(obj_msg);
    }
}

// Return the name of the handrail that intersects with the given pose.
std::string getHandrailAtPose(const ISSWorld& world, const Eigen::Affine3d& pose)
{
    unsigned int handrail_idx = world.findHandrail(pose.translation());
    if (handrail_idx < world.numHandrails())
        return world.getHandrail(handrail_idx)->name;

    return "";  // no handrail at pose
}

// Let the entire foot collide/not collide with the given handrail
void letGripperTouchObject(R2Interface& interface, bool left, const std::string& handrail, bool allow)
{
    interface.enableCollisionChecking(left ? LEFT_GRIPPER_LINK0 : RIGHT_GRIPPER_LINK0, handrail, allow, true);
    interface.enableCollisionChecking(left ? LEFT_GRIPPER_LINK1 : RIGHT_GRIPPER_LINK1, handrail, allow, true);
    interface.enableCollisionChecking(left ? LEFT_FOOT_LINK : RIGHT_FOOT_LINK, handrail, allow, true); // true, means wait for ack
}

// Compute the desired orientation for the end effector given the handrail group
void getEndEffectorOrientation(const std::string& handrail_group, std::vector<double>& rpy)
{
    // "Normal" grasp orientation for R2 on a handrail at the origin
    Eigen::Affine3d pose = Eigen::Affine3d::Identity();
    pose *= (Eigen::AngleAxisd(3.141592653, Eigen::Vector3d::UnitX()) *
             Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY()) *
             Eigen::AngleAxisd(1.57079, Eigen::Vector3d::UnitZ()));

    Eigen::Affine3d rotation = Eigen::Affine3d::Identity();

    if (handrail_group == "Handrail_starboard")
    {
        rotation *= (Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX()) *
                     Eigen::AngleAxisd(-1.57079, Eigen::Vector3d::UnitY()) *
                     Eigen::AngleAxisd(-1.57079, Eigen::Vector3d::UnitZ()));
    }
    else if (handrail_group == "Handrail_port")
    {
        rotation *= (Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX()) *
                     Eigen::AngleAxisd(1.57079, Eigen::Vector3d::UnitY()) *
                     Eigen::AngleAxisd(-1.57079, Eigen::Vector3d::UnitZ()));
    }
    else if (handrail_group == "Handrail_deck")
    {
        rotation *= (Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX()) *
                     Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY()) *
                     Eigen::AngleAxisd(-1.57079, Eigen::Vector3d::UnitZ()));
    }
    else if (handrail_group == "Handrail_overhead")
    {
        rotation *= (Eigen::AngleAxisd(-3.141592653, Eigen::Vector3d::UnitX()) *
                     Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY()) *
                     Eigen::AngleAxisd(-1.57079, Eigen::Vector3d::UnitZ()));
    }
    else
        ROS_WARN("Unknown handrail group '%s'", handrail_group.c_str());

    pose = pose * rotation;

    Eigen::Vector3d euler = pose.rotation().eulerAngles(0, 1, 2);
    rpy.resize(3);
    rpy[0] = euler(0);
    rpy[1] = euler(1);
    rpy[2] = euler(2);
}

// update the allowed collision matrix to allow/disallow collisions with handrails
void adjustAllowedHandrailCollisions(R2Interface& interface, robot_state::RobotStatePtr currentState, const ISSWorld& world,
                                     bool leftLegFixed, const std::string& handrail)
{
    std::string touchingHandrail = getHandrailAtPose(world, currentState->getGlobalLinkTransform(leftLegFixed ? LEFT_GRIPPER_LINK0 : RIGHT_GRIPPER_LINK0));
    if (touchingHandrail == "")
        touchingHandrail = getHandrailAtPose(world, currentState->getGlobalLinkTransform(leftLegFixed ? LEFT_GRIPPER_LINK1 : RIGHT_GRIPPER_LINK1));

    // Disable collisions with the destination handrail
    if (touchingHandrail == "")
        ROS_ERROR("No handrail collision detected.  Expected %s to be touching a handrail.", leftLegFixed ? "left leg" : "right leg");
    else
    {
        letGripperTouchObject(interface, leftLegFixed, touchingHandrail, true);      // make the start state valid
        letGripperTouchObject(interface, !leftLegFixed, touchingHandrail, true);      // make the start state valid
        letGripperTouchObject(interface, leftLegFixed ? false : true, handrail, true); // make the goal state valid

        std::cout << (leftLegFixed ? "Right " : "Left ") << "leg is allowed to collide with " << handrail << std::endl;

        // TODO: Should turn collisions back on for previous handrails

        // Make sure the gripper is open for the moving leg and closed for fixed leg
        currentState->setVariablePosition((leftLegFixed ? "r2/right_leg/gripper/jawLeft" : "r2/left_leg/gripper/jawLeft"), 0.75);
        currentState->setVariablePosition((leftLegFixed ? "r2/right_leg/gripper/jawRight" : "r2/left_leg/gripper/jawRight"), 0.75);

        currentState->setVariablePosition((leftLegFixed ? "r2/left_leg/gripper/jawLeft" : "r2/right_leg/gripper/jawLeft"), 0.0);
        currentState->setVariablePosition((leftLegFixed ? "r2/left_leg/gripper/jawRight" : "r2/right_leg/gripper/jawRight"), 0.0);
        currentState->update();

        std::cout << (leftLegFixed ? "Left " : "Right ") << "leg is holding " << touchingHandrail << std::endl;
    }
}

// Return the desired goal pose (in global frame) to reach the given handrail
Eigen::Affine3d getGoalPose(const ISSWorld& world, const std::string& handrail)
{
    const ISSWorld::Handrail* hr = world.getHandrail(handrail);
    Eigen::Affine3d goal_pose;
    goal_pose.translation() = world.getHandrailMidpoint(hr);

    // Since the end of the leg group is NOT the foot, we offset by roughly the height of the foot
    /*if (hr->group == "Handrail_starboard")
    {
        goal_pose.translation()(1) -= 0.27;
    }
    else if (hr->group == "Handrail_port")
    {
        goal_pose.translation()(1) += 0.27;
    }
    else if (hr->group == "Handrail_deck")
    {
        goal_pose.translation()(2) += 0.27;
    }
    else if (hr->group == "Handrail_overhead")
    {
        goal_pose.translation()(2) -= 0.27;
    }
    else
        ROS_WARN("Unknown handrail group '%s'", hr->group.c_str());*/

    // Retrieve the proper foot orientation for the goal handrail
    std::vector<double> rpy;
    //handrails->getHandrailOrientation(hr, rpy);
    getEndEffectorOrientation(hr->group, rpy);

    goal_pose = Eigen::Translation3d(goal_pose.translation()) *
               (Eigen::AngleAxisd(rpy[0], Eigen::Vector3d::UnitX()) *
                Eigen::AngleAxisd(rpy[1], Eigen::Vector3d::UnitY()) *
                Eigen::AngleAxisd(rpy[2], Eigen::Vector3d::UnitZ()));

    return goal_pose;
}

moveit_msgs::OrientationConstraint createTorsoUpConstraint()
{
    geometry_msgs::Quaternion q;
    q.x = 0.0;
    q.y = 0.0;
    q.z = 0.0;
    q.w = 1.0;

    // Constrain torso upright
    moveit_msgs::OrientationConstraint torso_orn_constraint;
    torso_orn_constraint.header.frame_id = "virtual_world";
    torso_orn_constraint.orientation = q;
    torso_orn_constraint.link_name = "r2/robot_world";
    torso_orn_constraint.absolute_x_axis_tolerance = moveit_r2_kinematics::HIGH_PRIO_ANGULAR_TOL;
    torso_orn_constraint.absolute_y_axis_tolerance = moveit_r2_kinematics::HIGH_PRIO_ANGULAR_TOL;
    torso_orn_constraint.absolute_z_axis_tolerance = 6.283185;
    torso_orn_constraint.weight = 1.0;

    return torso_orn_constraint;
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

// Create a position and orientation constraint for the moving link in the step
void createGoalConstraints(//const Step& step,
                           const std::string& link, const geometry_msgs::PoseStamped& pose,
                           std::vector<moveit_msgs::Constraints>& constraints,
                           double linearTol = moveit_r2_kinematics::CRITICAL_PRIO_LINEAR_TOL,
                           double angularTol = moveit_r2_kinematics::CRITICAL_PRIO_ANGULAR_TOL)
{
    moveit_msgs::Constraints goal;

    moveit_msgs::PositionConstraint position;
    position.link_name = link;
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

    position.constraint_region.primitive_poses.push_back(pose.pose);
    position.header.frame_id = pose.header.frame_id;
    goal.position_constraints.push_back(position);

    moveit_msgs::OrientationConstraint orientation;
    orientation.link_name = link;
    orientation.orientation = pose.pose.orientation;
    orientation.header.frame_id = pose.header.frame_id;
    orientation.weight = 1.0;

    orientation.absolute_x_axis_tolerance = angularTol;
    orientation.absolute_y_axis_tolerance = angularTol;
    orientation.absolute_z_axis_tolerance = angularTol;
    goal.orientation_constraints.push_back(orientation);

    // If a desired torso yaw is requested, also setup this constraint
    // if (step.torsoYaw)
    // {
    //     // Orientation is constrained relative to pose (global frame):
    //     Eigen::Affine3d pose = Eigen::Affine3d::Identity();
    //     pose *= Eigen::AngleAxisd(step.yaw, Eigen::Vector3d::UnitZ());
    //     geometry_msgs::Pose pose_msg;
    //     tf::poseEigenToMsg(pose, pose_msg);

    //     moveit_msgs::OrientationConstraint torso_constraint;
    //     torso_constraint.link_name = "r2/robot_world";
    //     torso_constraint.orientation = pose_msg.orientation;
    //     torso_constraint.header.frame_id = "virtual_world";
    //     torso_constraint.weight = 1.0;

    //     // roll and pitch are not constrained
    //     torso_constraint.absolute_x_axis_tolerance = 6.283185;
    //     torso_constraint.absolute_y_axis_tolerance = 6.283185;
    //     torso_constraint.absolute_z_axis_tolerance = angularTol;

    //     goal.orientation_constraints.push_back(torso_constraint);
    // }

    // // Desired x, y and/or z value
    // if (step.torsoX || step.torsoY || step.torsoZ)
    // {
    //     // Position is constrained relative to pose (in global frame):
    //     Eigen::Affine3d pose = Eigen::Affine3d::Identity();
    //     if (step.torsoX) pose.translation()(0) = step.x;
    //     if (step.torsoY) pose.translation()(1) = step.y;
    //     if (step.torsoZ) pose.translation()(2) = step.z;
    //     geometry_msgs::Pose pose_msg;
    //     tf::poseEigenToMsg(pose, pose_msg);

    //     moveit_msgs::PositionConstraint torso_position;
    //     std::string frame_id = "virtual_world";

    //     // Setting position constraint
    //     torso_position.link_name = "r2/robot_world";
    //     torso_position.target_point_offset.x = 0.0;
    //     torso_position.target_point_offset.y = 0.0;
    //     torso_position.target_point_offset.z = 0.0;
    //     torso_position.weight = 1.0;

    //     shape_msgs::SolidPrimitive box;
    //     box.type = shape_msgs::SolidPrimitive::BOX;
    //     box.dimensions.push_back(step.torsoX ? moveit_r2_kinematics::HIGH_PRIO_LINEAR_TOL : 10);  // BOX_X
    //     box.dimensions.push_back(step.torsoY ? moveit_r2_kinematics::HIGH_PRIO_LINEAR_TOL : 10);  // BOX_Y
    //     box.dimensions.push_back(step.torsoZ ? moveit_r2_kinematics::HIGH_PRIO_LINEAR_TOL : 10);  // BOX_Z
    //     torso_position.constraint_region.primitives.push_back(box);

    //     torso_position.constraint_region.primitive_poses.push_back(pose_msg);
    //     torso_position.header.frame_id = frame_id;

    //     goal.position_constraints.push_back(torso_position);
    // }

    constraints.push_back(goal);
}

bool handrailClimbing(R2Interface& interface, const ISSWorld& world)
{
    // Creating a list of handrails to climb across
    bool leftLegFixed = false;
    std::vector<std::string> handrailPlacements;
    handrailPlacements.push_back("Handrail_deck_1");   // left
    handrailPlacements.push_back("Handrail_port_0");   // right
    handrailPlacements.push_back("Handrail_stbd_2");   // left

    // Allocating two states - the "current" state and the next state (the goal state)
    robot_state::RobotStatePtr currentState = interface.allocRobotState();
    *(currentState.get()) = interface.getCurrentRobotState();
    currentState->update();

    // This is where we will store the trajectories as they are computed
    std::vector<moveit_msgs::RobotTrajectory> trajectories(handrailPlacements.size());

    bool success = true;
    bool leftLegFixedFirst = leftLegFixed;

    for(size_t i = 0; i < handrailPlacements.size(); ++i)
    {
        // Fix collisions with handrails
        adjustAllowedHandrailCollisions(interface, currentState, world, leftLegFixed, handrailPlacements[i]);

        // Figure out what is moving where
        std::string fixedLink, movingLink;
        fixedLink = leftLegFixed ? LEFT_LEG_END_LINK : RIGHT_LEG_END_LINK;
        movingLink = leftLegFixed ? RIGHT_LEG_END_LINK : LEFT_LEG_END_LINK;
        std::cout << "Fixing link " << fixedLink << "  and moving link " << movingLink << std::endl;

        Eigen::Affine3d goal_pose = getGoalPose(world, handrailPlacements[i]);
        geometry_msgs::PoseStamped pose_msg;
        tf::poseEigenToMsg(goal_pose, pose_msg.pose);
        pose_msg.header.frame_id = "virtual_world";

        // fix the fixed leg for the whole path
        // moveit_msgs::Constraints path_constraints;
        // path_constraints.position_constraints.push_back(createPositionConstraint(fixedLink, *(currentState.get()), moveit_r2_kinematics::CRITICAL_PRIO_LINEAR_TOL));
        // path_constraints.orientation_constraints.push_back(createOrientationConstraint(fixedLink, *(currentState.get()), moveit_r2_kinematics::CRITICAL_PRIO_ANGULAR_TOL));

        moveit_msgs::Constraints path_constraints;

        // base link pose constraint
        geometry_msgs::Pose base_pose;
        tf::poseEigenToMsg(currentState->getGlobalLinkTransform(fixedLink), base_pose);
        createBaseLinkConstraints(path_constraints, fixedLink, base_pose);
        // body pose constraint
        //createARGOSTorsoConstraints(path_constraints); // whole path torso-up

        // goal pose constraints
        std::vector<moveit_msgs::Constraints> goal_constraints;
        createGoalConstraints(movingLink, pose_msg, goal_constraints);
        createARGOSTorsoConstraints(goal_constraints[0]);  // torso up at goal

        // // Setup goal pose constraint for moving foot
        // moveit_msgs::PositionConstraint goal_pos_constraint;
        // goal_pos_constraint.header.frame_id = "virtual_world";
        // goal_pos_constraint.link_name = movingLink;
        // goal_pos_constraint.target_point_offset.x = 0;
        // goal_pos_constraint.target_point_offset.y = 0;
        // goal_pos_constraint.target_point_offset.z = 0;

        // // Define a bounding box - the goal region
        // shape_msgs::SolidPrimitive box;
        // box.type = shape_msgs::SolidPrimitive::BOX;

        // // Allow for wiggle room along the handrail
        // // TODO: There is a bug here, so no wiggle room allowed.
        // // Likely a problem with tolerances and the wiggle room.  Is the frame correct??
        // const ISSWorld::Handrail* hr = world.getHandrail(handrailPlacements[i]);
        // if (hr->group == "Handrail_deck" || hr->group == "Handrail_overhead")
        // {
        //     box.dimensions.push_back(moveit_r2_kinematics::CRITICAL_PRIO_LINEAR_TOL);   // BOX_X
        //     box.dimensions.push_back(moveit_r2_kinematics::CRITICAL_PRIO_LINEAR_TOL);   // BOX_Y
        //     //box.dimensions.push_back(0.4);                                              // BOX_Y - half width of handrail
        //     box.dimensions.push_back(moveit_r2_kinematics::CRITICAL_PRIO_LINEAR_TOL);   // BOX_Z
        // }
        // else if (hr->group == "Handrail_port" || hr->group == "Handrail_starboard")
        // {
        //     box.dimensions.push_back(moveit_r2_kinematics::CRITICAL_PRIO_LINEAR_TOL);   // BOX_X
        //     box.dimensions.push_back(moveit_r2_kinematics::CRITICAL_PRIO_LINEAR_TOL);   // BOX_Y
        //     //box.dimensions.push_back(0.4);                                              // BOX_Z - half width of handrail
        //     box.dimensions.push_back(moveit_r2_kinematics::CRITICAL_PRIO_LINEAR_TOL);    // BOX_Z
        // }

        // goal_pos_constraint.constraint_region.primitives.push_back(box);
        // goal_pos_constraint.weight = 1.0;

        // // Center the bounding box at the goal pose location
        // goal_pos_constraint.constraint_region.primitive_poses.push_back(pose_msg.pose);

        // // Create orientation constraint (upright over handrail)
        // moveit_msgs::OrientationConstraint goal_orn_constraint;
        // goal_orn_constraint.header.frame_id = pose_msg.header.frame_id;
        // goal_orn_constraint.orientation = pose_msg.pose.orientation;
        // goal_orn_constraint.link_name = movingLink;
        // goal_orn_constraint.absolute_x_axis_tolerance = moveit_r2_kinematics::CRITICAL_PRIO_ANGULAR_TOL;
        // goal_orn_constraint.absolute_y_axis_tolerance = moveit_r2_kinematics::CRITICAL_PRIO_ANGULAR_TOL;
        // goal_orn_constraint.absolute_z_axis_tolerance = moveit_r2_kinematics::CRITICAL_PRIO_ANGULAR_TOL;
        // goal_orn_constraint.weight = 1.0;

        // // Amalgamate all goal constraints together
        // std::vector<moveit_msgs::Constraints> goal_constraints(1);
        // goal_constraints.back().position_constraints.push_back(goal_pos_constraint);
        // goal_constraints.back().orientation_constraints.push_back(goal_orn_constraint);
        // //goal_constraints.back().orientation_constraints.push_back(createTorsoUpConstraint());

        moveit_msgs::RobotState start_state;
        moveit::core::robotStateToRobotStateMsg(*(currentState.get()), start_state);

        unsigned int tries = 4;
        success = false;

        // Try a few times before giving up.  Sometimes we get a crappy goal state
        for(size_t attempt = 0; attempt < tries && !success; ++attempt)
        {
            if (interface.plan(start_state, path_constraints, goal_constraints, "legs",
                               20.0,
                               //"HiLo",       // good-ish.  Can probably make better
                               //"HiLo_AG",    // seems to be the winner - compromise between path wackiness and speed.
                               //"BiHiLo",        // better, still not great
                               "XXL",         // seems to work well-ish.  I saw a backflip.
                               //"XXL_AG",        // seems to work well-ish.  No backflips, but second step is difficult for interpolator.
                               //"RRTConnect", // high failure rate, even for "easy" problems.  When it works, it's super fast.  Wacky paths
                               //"RRTConnect_AG", // High failure rate and bad paths.  Probably safe to exclude from any benchmarking
                               //"RRT_AG", // lots of fails, but less (?) than bidirectional RRT
                               //"RRT",       // much better success rate than RRT_AG and RRTConnect, but paths look cray cray
                               //"RRTstar",     // Reasonable success rate, but still wacky looking paths
                               //"CBiRRT2",  // slow, but paths look good-ish.  Probably because it is leveraging the IK solver joint seed to minimize joint distance
                               //"RRT_LC",  // super experimental
                               //"XXL_LC",  // it don't get more experimental than this
                               //"KPIECE",
                               trajectories[i]))
            {
                ROS_WARN("Step %lu (to %s) is glorious success", i, handrailPlacements[i].c_str());
                success = true;
            }
            else
                ROS_ERROR("Attempt %lu to '%s' brings shame upon you", attempt, handrailPlacements[i].c_str());
        }

        if (!success)
        {
            ROS_ERROR("Too many planning failures.  Giving up");
            break;
        }

        // Extracting the next "start" state from the end of the most recently computed trajectory
        std::map<std::string, double> newValues;
        const trajectory_msgs::JointTrajectory& nextStateMsg = trajectories[i].joint_trajectory;
        for(size_t j = 0; j < nextStateMsg.joint_names.size(); ++j)
            newValues[nextStateMsg.joint_names[j]] = nextStateMsg.points.back().positions[j];

        const trajectory_msgs::MultiDOFJointTrajectory& multiDOFTraj = trajectories[i].multi_dof_joint_trajectory;
        for(size_t j = 0; j < multiDOFTraj.joint_names.size(); ++j)
        {
            const geometry_msgs::Transform& tf = multiDOFTraj.points.back().transforms[j];
            newValues[multiDOFTraj.joint_names[j] + std::string("/trans_x")] = tf.translation.x;
            newValues[multiDOFTraj.joint_names[j] + std::string("/trans_y")] = tf.translation.y;
            newValues[multiDOFTraj.joint_names[j] + std::string("/trans_z")] = tf.translation.z;

            newValues[multiDOFTraj.joint_names[j] + std::string("/rot_x")] = tf.rotation.x;
            newValues[multiDOFTraj.joint_names[j] + std::string("/rot_y")] = tf.rotation.y;
            newValues[multiDOFTraj.joint_names[j] + std::string("/rot_z")] = tf.rotation.z;
            newValues[multiDOFTraj.joint_names[j] + std::string("/rot_w")] = tf.rotation.w;
        }

        currentState->setVariablePositions(newValues);
        currentState->update();

        // Assuming that the legs alternate being fixed
        leftLegFixed = !leftLegFixed;
    }

    if (success)
    {
        openAndCloseGrippers(trajectories, leftLegFixedFirst);
        interface.viewTrajectory(trajectories);
        ros::spinOnce();
        sleep(1);
    }

    return success;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "iss_navigation");
    ros::AsyncSpinner spinner(0); // use # threads = num cores
    spinner.start();

    // Create instance of R2Interface for all your planning needs
    R2Interface r2Interface;
    sleep(1);

    // Load the planning representation of the handrails
    ISSWorld world;
    createPlanningScene(r2Interface, world);
    sleep(1);  // wait until everything is synced

    handrailClimbing(r2Interface, world);
}

/// Create a PlanningScene message that approximates the detailed ISS model.
/// Note: The handrails object is only used to infer the size of the
/// bounding box that will approximate the exterior of the ISS.
/*void loadApproximateISSModelPlanningScene(R2Interface& interface, const std::string& frame="virtual_world")
{
    double minX = -3.0;
    double maxX = 3.0;

    double minY = -1.1;
    double maxY = 1.1;

    double minZ = -1.1;
    double maxZ = 1.1;

    // TOP
    {
        moveit_msgs::CollisionObject top_msg;
        top_msg.operation = moveit_msgs::CollisionObject::ADD;
        top_msg.id = "ISS_overhead";
        top_msg.header.frame_id = frame;

        shape_msgs::SolidPrimitive box_msg;
        box_msg.type = shape_msgs::SolidPrimitive::BOX;
        box_msg.dimensions.push_back(maxX - minX); // X
        box_msg.dimensions.push_back(maxY - minY); // Y
        box_msg.dimensions.push_back(0.0); // Z

        // Center of the box
        geometry_msgs::Pose box_pose;
        box_pose.position.x = (fabs(maxX) - fabs(minX))/2.0;
        box_pose.position.y = (fabs(maxY) - fabs(minY))/2.0;
        box_pose.position.z = maxZ;
        box_pose.orientation.x = 0;
        box_pose.orientation.y = 0;
        box_pose.orientation.z = 0;
        box_pose.orientation.w = 1;

        top_msg.primitives.push_back(box_msg);
        top_msg.primitive_poses.push_back(box_pose);
        interface.addObstacleToPlanningScene(top_msg);
    }

    // Bottom
    {
        moveit_msgs::CollisionObject bottom_msg;
        bottom_msg.operation = moveit_msgs::CollisionObject::ADD;
        bottom_msg.id = "ISS_Deck";
        bottom_msg.header.frame_id = frame;

        shape_msgs::SolidPrimitive box_msg;
        box_msg.type = shape_msgs::SolidPrimitive::BOX;
        box_msg.dimensions.push_back(maxX - minX); // X
        box_msg.dimensions.push_back(maxY - minY); // Y
        box_msg.dimensions.push_back(0.0); // Z

        // Center of the box
        geometry_msgs::Pose box_pose;
        box_pose.position.x = (fabs(maxX) - fabs(minX))/2.0;
        box_pose.position.y = (fabs(maxY) - fabs(minY))/2.0;
        box_pose.position.z = minZ;
        box_pose.orientation.x = 0;
        box_pose.orientation.y = 0;
        box_pose.orientation.z = 0;
        box_pose.orientation.w = 1;

        bottom_msg.primitives.push_back(box_msg);
        bottom_msg.primitive_poses.push_back(box_pose);
        interface.addObstacleToPlanningScene(bottom_msg);
    }

    // Starboard (+side of Y axis)
    {
        moveit_msgs::CollisionObject port_msg;
        port_msg.operation = moveit_msgs::CollisionObject::ADD;
        port_msg.id = "ISS_starboard";
        port_msg.header.frame_id = frame;

        shape_msgs::SolidPrimitive box_msg;
        box_msg.type = shape_msgs::SolidPrimitive::BOX;
        box_msg.dimensions.push_back(maxX - minX); // X
        box_msg.dimensions.push_back(0); // Y
        box_msg.dimensions.push_back(maxZ - minZ); // Z

        // Center of the box
        geometry_msgs::Pose box_pose;
        box_pose.position.x = (fabs(maxX) - fabs(minX))/2.0;
        box_pose.position.y = maxY;
        box_pose.position.z = (fabs(maxZ) - fabs(minZ))/2.0;
        box_pose.orientation.x = 0;
        box_pose.orientation.y = 0;
        box_pose.orientation.z = 0;
        box_pose.orientation.w = 1;

        port_msg.primitives.push_back(box_msg);
        port_msg.primitive_poses.push_back(box_pose);
        interface.addObstacleToPlanningScene(port_msg);
    }

    // Port (-side of Y axis)
    // This one gets weird because of R2's closet
    {
        moveit_msgs::CollisionObject port_msg;
        port_msg.operation = moveit_msgs::CollisionObject::ADD;
        port_msg.id = "ISS_port";
        port_msg.header.frame_id = frame;

        shape_msgs::SolidPrimitive box_msg;
        box_msg.type = shape_msgs::SolidPrimitive::BOX;
        box_msg.dimensions.push_back(maxX - minX - 0.80); // X
        box_msg.dimensions.push_back(0); // Y
        box_msg.dimensions.push_back(maxZ - minZ); // Z

        // Center of the box
        geometry_msgs::Pose box_pose;
        box_pose.position.x = (fabs(maxX) - fabs(minX) + 0.80)/2.0;
        box_pose.position.y = minY;
        box_pose.position.z = (fabs(maxZ) - fabs(minZ))/2.0;
        box_pose.orientation.x = 0;
        box_pose.orientation.y = 0;
        box_pose.orientation.z = 0;
        box_pose.orientation.w = 1;

        port_msg.primitives.push_back(box_msg);
        port_msg.primitive_poses.push_back(box_pose);
        interface.addObstacleToPlanningScene(port_msg);

        // CLOSET
        // side 1
        moveit_msgs::CollisionObject closet_side;
        closet_side.operation = moveit_msgs::CollisionObject::ADD;
        closet_side.id = "ISS_port_closet1";
        closet_side.header.frame_id = frame;

        //shape_msgs::SolidPrimitive box_msg;
        //box_msg.type = shape_msgs::SolidPrimitive::BOX;
        box_msg.dimensions[0] = 0.0;          // X
        box_msg.dimensions[1] = 1.0;          // Y
        box_msg.dimensions[2] = maxZ - minZ - 0.3;  // Z

        // Center of the box
        //geometry_msgs::Pose box_pose;
        box_pose.position.x = (minX + 0.80);
        box_pose.position.y = minY - 0.5;
        box_pose.position.z = (fabs(maxZ) - fabs(minZ))/2.0;
        box_pose.orientation.x = 0;
        box_pose.orientation.y = 0;
        box_pose.orientation.z = 0;
        box_pose.orientation.w = 1;

        closet_side.primitives.push_back(box_msg);
        closet_side.primitive_poses.push_back(box_pose);
        interface.addObstacleToPlanningScene(closet_side);

        // side 2
        closet_side.primitives.clear();
        closet_side.primitive_poses.clear();

        closet_side.id = "ISS_port_closet2";
        box_msg.dimensions[0] = 0.0;                // X size
        box_msg.dimensions[1] = 1.0;                // Y size
        box_msg.dimensions[2] = maxZ - minZ - 0.3;  // Z size
        box_pose.position.x = (minX - 0.20);        // position x

        closet_side.primitives.push_back(box_msg);
        closet_side.primitive_poses.push_back(box_pose);
        interface.addObstacleToPlanningScene(closet_side);

        // rear
        closet_side.primitives.clear();
        closet_side.primitive_poses.clear();

        closet_side.id = "ISS_port_closet3";
        box_msg.dimensions[0] = 1.0;                // X size
        box_msg.dimensions[1] = 0.0;                // Y size
        box_msg.dimensions[2] = maxZ - minZ - 0.3;  // Z size
        box_pose.position.x = (minX + 0.30);        // position x
        box_pose.position.y = minY - 1.0;

        closet_side.primitives.push_back(box_msg);
        closet_side.primitive_poses.push_back(box_pose);
        interface.addObstacleToPlanningScene(closet_side);

        // bottom
        closet_side.primitives.clear();
        closet_side.primitive_poses.clear();

        closet_side.id = "ISS_port_closet4";
        box_msg.dimensions[0] = 1.0;                // X size
        box_msg.dimensions[1] = 1.0;                // Y size
        box_msg.dimensions[2] = 0.0;                // Z size
        box_pose.position.x = (minX + 0.30);        // position x
        box_pose.position.y = minY - 0.5;
        box_pose.position.z = minZ + 0.15;

        closet_side.primitives.push_back(box_msg);
        closet_side.primitive_poses.push_back(box_pose);
        interface.addObstacleToPlanningScene(closet_side);

        // top
        closet_side.primitives.clear();
        closet_side.primitive_poses.clear();

        closet_side.id = "ISS_port_closet5";
        box_pose.position.z = maxZ - 0.15;

        closet_side.primitives.push_back(box_msg);
        closet_side.primitive_poses.push_back(box_pose);
        interface.addObstacleToPlanningScene(closet_side);

    }

    // Initializing robot workspace inside the fun little box we just created
    // moveit_msgs::WorkspaceParameters workspace;
    // workspace.min_corner.x = minX;
    // workspace.min_corner.y = minY;
    // workspace.min_corner.z = minZ;
    // workspace.max_corner.x = maxX;
    // workspace.max_corner.y = maxY;
    // workspace.max_corner.z = maxZ;

    // interface.setWorkspace(workspace);
}*/