// Create the planning scene database to run benchmarks on
// Author: Ryan Luna
// December 2015

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
static const std::string LEFT_ARM_END_LINK      = "r2/left_palm";
static const std::string RIGHT_ARM_END_LINK     = "r2/right_palm";
static const std::string LEFT_GRIPPER_LINK0     = "r2/left_leg/gripper/jaw_left";
static const std::string LEFT_GRIPPER_LINK1     = "r2/left_leg/gripper/jaw_right";
static const std::string RIGHT_GRIPPER_LINK0    = "r2/right_leg/gripper/jaw_left";
static const std::string RIGHT_GRIPPER_LINK1    = "r2/right_leg/gripper/jaw_right";
static const std::string LEFT_FOOT_LINK         = "r2/left_leg_foot";
static const std::string RIGHT_FOOT_LINK        = "r2/right_leg_foot";

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

void addExtraObstacle(R2Interface& interface)
{
    moveit_msgs::CollisionObject obj_msg;

    obj_msg.operation = moveit_msgs::CollisionObject::ADD;
    obj_msg.id = "badzone";
    obj_msg.header.frame_id = "world";
    obj_msg.header.stamp = ros::Time::now();

    shape_msgs::SolidPrimitive box;
    box.type = shape_msgs::SolidPrimitive::BOX;
    box.dimensions.push_back(0.3);   // BOX_X
    box.dimensions.push_back(0.6);  // BOX_Y
    box.dimensions.push_back(0.7);   // BOX_Z

    obj_msg.primitives.push_back(box);

    geometry_msgs::Pose box_pose;
    box_pose.position.x = -1.2;
    box_pose.position.y = -0.7;
    box_pose.position.z = -0.7;
    box_pose.orientation.x = box_pose.orientation.y = box_pose.orientation.z = 0.0;
    box_pose.orientation.w = 1.0;

    obj_msg.primitive_poses.push_back(box_pose);
    interface.addObstacleToPlanningScene(obj_msg);
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
        std::cout << (leftLegFixed ? "Left " : "Right ") << "leg is holding " << touchingHandrail << std::endl;

        letGripperTouchObject(interface, leftLegFixed, touchingHandrail, true);      // make the start state valid
        letGripperTouchObject(interface, !leftLegFixed, touchingHandrail, true);      // make the start state valid
        letGripperTouchObject(interface, leftLegFixed ? false : true, handrail, true); // make the goal state valid

        std::cout << (leftLegFixed ? "Right " : "Left ") << "leg is allowed to collide with " << handrail << std::endl;

        // see if other gripper (moving link) is already holding a handrail
        touchingHandrail = getHandrailAtPose(world, currentState->getGlobalLinkTransform(!leftLegFixed ? LEFT_GRIPPER_LINK0 : RIGHT_GRIPPER_LINK0));
        if (touchingHandrail != "")
        {
            std::cout << (!leftLegFixed ? "Left " : "Right ") << "leg is holding " << touchingHandrail << std::endl;
            letGripperTouchObject(interface, !leftLegFixed, touchingHandrail, true);      // make the start state valid
        }

        // TODO: Should turn collisions back on for previous handrails

        // Make sure the gripper is open for the moving leg and closed for fixed leg
        currentState->setVariablePosition((leftLegFixed ? "r2/right_leg/gripper/jawLeft" : "r2/left_leg/gripper/jawLeft"), 0.75);
        currentState->setVariablePosition((leftLegFixed ? "r2/right_leg/gripper/jawRight" : "r2/left_leg/gripper/jawRight"), 0.75);

        currentState->setVariablePosition((leftLegFixed ? "r2/left_leg/gripper/jawLeft" : "r2/right_leg/gripper/jawLeft"), 0.0);
        currentState->setVariablePosition((leftLegFixed ? "r2/left_leg/gripper/jawRight" : "r2/right_leg/gripper/jawRight"), 0.0);
        currentState->update();
    }
}

void adjustAllowedHandrailCollisionsArm(R2Interface& interface, robot_state::RobotStatePtr currentState, const ISSWorld& world, bool left_arm, const std::string& handrail)
{
    // All the links that compose the left hand
    std::vector<std::string> left_links;
    left_links.push_back("r2/left_palm");
    left_links.push_back("r2/left_thumb_base");
    left_links.push_back("r2/left_thumb_proximal");
    left_links.push_back("r2/left_thumb_medial_prime");
    left_links.push_back("r2/left_thumb_medial");
    left_links.push_back("r2/left_thumb_distal");
    left_links.push_back("r2/left_thumb_tip");
    left_links.push_back("r2/left_index_base");
    left_links.push_back("r2/left_index_yaw");
    left_links.push_back("r2/left_index_proximal");
    left_links.push_back("r2/left_index_medial");
    left_links.push_back("r2/left_middle_base");
    left_links.push_back("r2/left_middle_yaw");
    left_links.push_back("r2/left_middle_proximal");
    left_links.push_back("r2/left_middle_medial");
    //left_links.push_back("r2/left_middle_distal");
    //left_links.push_back("r2/left_middle_tip");
    left_links.push_back("r2/left_ring_proximal");
    //left_links.push_back("r2/left_ring_medial");
    //left_links.push_back("r2/left_ring_distal");
    //left_links.push_back("r2/left_ring_tip");
    left_links.push_back("r2/left_little_proximal");
    //left_links.push_back("r2/left_little_medial");
    //left_links.push_back("r2/left_little_distal");
    //left_links.push_back("r2/left_little_tip");

    // All the links that compose the right hand
    std::vector<std::string> right_links;
    right_links.push_back("r2/right_palm");
    right_links.push_back("r2/right_thumb_base");
    right_links.push_back("r2/right_thumb_proximal");
    right_links.push_back("r2/right_thumb_medial_prime");
    right_links.push_back("r2/right_thumb_medial");
    right_links.push_back("r2/right_thumb_distal");
    right_links.push_back("r2/right_thumb_tip");
    right_links.push_back("r2/right_index_base");
    right_links.push_back("r2/right_index_yaw");
    right_links.push_back("r2/right_index_proximal");
    right_links.push_back("r2/right_index_medial");
    right_links.push_back("r2/right_middle_base");
    right_links.push_back("r2/right_middle_yaw");
    right_links.push_back("r2/right_middle_proximal");
    right_links.push_back("r2/right_middle_medial");
    //right_links.push_back("r2/right_middle_distal");
    //right_links.push_back("r2/right_middle_tip");
    right_links.push_back("r2/right_ring_proximal");
    //right_links.push_back("r2/right_ring_medial");
    //right_links.push_back("r2/right_ring_distal");
    //right_links.push_back("r2/right_ring_tip");
    right_links.push_back("r2/right_little_proximal");
    //right_links.push_back("r2/right_little_medial");
    //right_links.push_back("r2/right_little_distal");
    //right_links.push_back("r2/right_little_tip");

    std::vector<std::string> handrail_links;
    handrail_links.push_back(handrail);

    interface.enableCollisionChecking(left_arm ? left_links : right_links, handrail_links, true, true);
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
    or_constraint.header.frame_id = "world";
    or_constraint.weight = 1.0;

    // No roll or pitch allowed.  Yaw is unconstrained
    or_constraint.absolute_x_axis_tolerance = tol;
    or_constraint.absolute_y_axis_tolerance = tol;
    or_constraint.absolute_z_axis_tolerance = 6.283185; // 2*PI

    constraints.orientation_constraints.push_back(or_constraint);
}

// Return the desired goal pose (in global frame) to reach the given handrail
Eigen::Affine3d getGoalPose(const ISSWorld& world, const std::string& handrail)
{
    const ISSWorld::Handrail* hr = world.getHandrail(handrail);
    Eigen::Affine3d goal_pose;
    goal_pose.translation() = world.getHandrailMidpoint(hr);

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

Eigen::Affine3d getGoalPoseArm(const ISSWorld& world, const std::string& handrail)
{
    const ISSWorld::Handrail* hr = world.getHandrail(handrail);
    Eigen::Affine3d goal_pose;
    goal_pose.translation() = world.getHandrailMidpoint(hr);

    // Retrieve the proper foot orientation for the goal handrail
    std::vector<double> rpy(3, 0.0);
    rpy[0] = 1.57079;
    rpy[1] = -1.57079;

    //handrails->getHandrailOrientation(hr, rpy);
    //getEndEffectorOrientation(hr->group, rpy);

    goal_pose = Eigen::Translation3d(goal_pose.translation()) *
               (Eigen::AngleAxisd(rpy[0], Eigen::Vector3d::UnitX()) *
                Eigen::AngleAxisd(rpy[1], Eigen::Vector3d::UnitY()) *
                Eigen::AngleAxisd(rpy[2], Eigen::Vector3d::UnitZ()));

    return goal_pose;
}

// Constrain linkName to be at the given pose (position and orientation)
void createBaseLinkConstraints(moveit_msgs::Constraints& constraints, const std::string& linkName, geometry_msgs::Pose& pose,
                               double linearTol = moveit_r2_kinematics::CRITICAL_PRIO_LINEAR_TOL,
                               double angularTol = moveit_r2_kinematics::CRITICAL_PRIO_ANGULAR_TOL)
{
    moveit_msgs::PositionConstraint pos_constraint;
    std::string frame_id = "world";

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

    constraints.push_back(goal);
}

void addGoalConstraints(const std::string& link, const geometry_msgs::PoseStamped& pose,
                        moveit_msgs::Constraints& goal_constraints,
                        double linearTol = moveit_r2_kinematics::CRITICAL_PRIO_LINEAR_TOL,
                        double angularTol = moveit_r2_kinematics::CRITICAL_PRIO_ANGULAR_TOL)
{
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
    goal_constraints.position_constraints.push_back(position);

    moveit_msgs::OrientationConstraint orientation;
    orientation.link_name = link;
    orientation.orientation = pose.pose.orientation;
    orientation.header.frame_id = pose.header.frame_id;
    orientation.weight = 1.0;

    orientation.absolute_x_axis_tolerance = angularTol;
    orientation.absolute_y_axis_tolerance = angularTol;
    orientation.absolute_z_axis_tolerance = angularTol;
    goal_constraints.orientation_constraints.push_back(orientation);
}

bool planToHandrail(R2Interface& interface, const ISSWorld& world, bool leftLegFixed, const std::string& handrail,
                    const moveit_msgs::RobotState& startStateMsg, moveit_msgs::RobotState& endState, bool updateHandrailCollisions,
                    int count)
{
    // Allocating two states - the "current" state and the next state (the goal state)
    robot_state::RobotStatePtr currentState = interface.allocRobotState();
    moveit::core::robotStateMsgToRobotState(startStateMsg, *(currentState.get()));
    currentState->update();

    //*(currentState.get()) = interface.getCurrentRobotState();
    //currentState->update();

    // moveit_msgs::RobotState start_state;
    // moveit::core::robotStateToRobotStateMsg(*(currentState.get()), start_state);

    // Fix collisions with handrails
    if (updateHandrailCollisions)
        adjustAllowedHandrailCollisions(interface, currentState, world, leftLegFixed, handrail);

    // Figure out what is moving where
    std::string fixedLink, movingLink;
    fixedLink = leftLegFixed ? LEFT_LEG_END_LINK : RIGHT_LEG_END_LINK;
    movingLink = leftLegFixed ? RIGHT_LEG_END_LINK : LEFT_LEG_END_LINK;
    std::cout << "Fixing link " << fixedLink << "  and moving link " << movingLink << std::endl;

    // The pose of the handrail we are trying to reach
    Eigen::Affine3d goal_pose = getGoalPose(world, handrail);
    geometry_msgs::PoseStamped pose_msg;
    tf::poseEigenToMsg(goal_pose, pose_msg.pose);
    pose_msg.header.frame_id = "world";

    moveit_msgs::Constraints path_constraints;

    // base link pose constraint
    geometry_msgs::Pose base_pose;
    tf::poseEigenToMsg(currentState->getGlobalLinkTransform(fixedLink), base_pose);
    createBaseLinkConstraints(path_constraints, fixedLink, base_pose);

    // goal pose constraints
    std::vector<moveit_msgs::Constraints> goal_constraints;
    createGoalConstraints(movingLink, pose_msg, goal_constraints);
    createARGOSTorsoConstraints(goal_constraints[0]);  // torso up at goal

    unsigned int tries = 4;
    bool success = false;
    moveit_msgs::RobotTrajectory trajectory;

    std::string group_name = "legs";
    std::string planner_name = "RRT";
    double max_time = 20.0; // seconds

    // Try a few times before giving up.  Sometimes we get a crappy goal state
    for(size_t attempt = 0; attempt < tries && !success; ++attempt)
    {
        if (interface.plan(startStateMsg, path_constraints, goal_constraints, group_name,
                           max_time,
                           //"HiLo",       // good-ish.  Can probably make better
                           //"HiLo_AG",    // seems to be the winner - compromise between path wackiness and speed.
                           //"BiHiLo",        // better, still not great
                           //"XXL",         // seems to work well-ish.  I saw a backflip.
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
                           planner_name,
                           trajectory))
        {
            ROS_WARN("Planning to %s is glorious success", handrail.c_str());
            success = true;
        }
        else
            ROS_ERROR("Planning attempt %lu to %s brings shame upon you", attempt, handrail.c_str());
    }

    bool stored = success;
    if (success)
    {
        // Extracting the next "start" state from the end of the most recently computed trajectory
        std::map<std::string, double> newValues;
        const trajectory_msgs::JointTrajectory& nextStateMsg = trajectory.joint_trajectory;
        for(size_t j = 0; j < nextStateMsg.joint_names.size(); ++j)
            newValues[nextStateMsg.joint_names[j]] = nextStateMsg.points.back().positions[j];

        const trajectory_msgs::MultiDOFJointTrajectory& multiDOFTraj = trajectory.multi_dof_joint_trajectory;
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
        moveit::core::robotStateToRobotStateMsg(*(currentState.get()), endState);


        char ch;
        std::cout << "Save request to db? [y/n] ";
        std::cin >> ch;

        if (ch == 'y' || ch == 'Y')
        {
            std::stringstream str;
            str << handrail << "_" << count;
            interface.storeRobotState(str.str(), endState);

            moveit_msgs::MotionPlanRequest request;
            interface.createMotionPlanRequest(startStateMsg, path_constraints, goal_constraints,
                                              group_name, max_time, planner_name, tries, request);

            str.str("");
            str << "step_" << handrail << "_" << count;
            interface.storeMotionPlanRequest(request, str.str(), "ISS");

            count++;
            stored = true;
        }
    }

    return stored;
}

bool planToHandrailWithLegsRArm(R2Interface& interface, const ISSWorld& world, bool leftLegFixed, const std::string& leg_handrail, const std::string& arm_handrail,
                                const moveit_msgs::RobotState& startStateMsg, moveit_msgs::RobotState& endState, bool updateHandrailCollisions,
                                int count)
{
    // Allocating two states - the "current" state and the next state (the goal state)
    robot_state::RobotStatePtr currentState = interface.allocRobotState();
    moveit::core::robotStateMsgToRobotState(startStateMsg, *(currentState.get()));
    currentState->update();

    // Fix collisions with handrails
    if (updateHandrailCollisions)
    {
        adjustAllowedHandrailCollisions(interface, currentState, world, leftLegFixed, leg_handrail);
        adjustAllowedHandrailCollisionsArm(interface, currentState, world, false, arm_handrail);
    }

    // Figure out what is moving where
    std::string fixedLink, movingLink;
    fixedLink = leftLegFixed ? LEFT_LEG_END_LINK : RIGHT_LEG_END_LINK;
    movingLink = leftLegFixed ? RIGHT_LEG_END_LINK : LEFT_LEG_END_LINK;
    std::cout << "Fixing link " << fixedLink << "  and moving link " << movingLink << std::endl;

    // Setting up constraints for entire path
    moveit_msgs::Constraints path_constraints;

    // base link pose constraint for whole path
    geometry_msgs::Pose base_pose;
    tf::poseEigenToMsg(currentState->getGlobalLinkTransform(fixedLink), base_pose);
    createBaseLinkConstraints(path_constraints, fixedLink, base_pose);


    // Setting up goal constraints
    std::vector<moveit_msgs::Constraints> goal_constraints(1);

    // goal pose for leg
    Eigen::Affine3d goal_pose_leg = getGoalPose(world, leg_handrail);
    geometry_msgs::PoseStamped leg_pose_msg;
    tf::poseEigenToMsg(goal_pose_leg, leg_pose_msg.pose);
    leg_pose_msg.header.frame_id = "world";
    addGoalConstraints(movingLink, leg_pose_msg, goal_constraints[0]);

    // goal pose for arm - position only
    Eigen::Affine3d goal_pose_arm = getGoalPoseArm(world, arm_handrail);
    geometry_msgs::PoseStamped arm_pose_msg;
    tf::poseEigenToMsg(goal_pose_arm, arm_pose_msg.pose);
    arm_pose_msg.header.frame_id = "world";
    //addGoalConstraints(RIGHT_ARM_END_LINK, arm_pose_msg, goal_constraints[0]);  // position and orientation

    // arm position
    moveit_msgs::PositionConstraint position;
    position.link_name = RIGHT_ARM_END_LINK;
    position.target_point_offset.x = 0.0;
    position.target_point_offset.y = 0.0;
    position.target_point_offset.z = 0.0;
    position.weight = 1.0;

    shape_msgs::SolidPrimitive box;
    box.type = shape_msgs::SolidPrimitive::BOX;
    box.dimensions.push_back(moveit_r2_kinematics::HIGH_PRIO_LINEAR_TOL);  // BOX_X
    box.dimensions.push_back(moveit_r2_kinematics::HIGH_PRIO_LINEAR_TOL);  // BOX_Y
    box.dimensions.push_back(moveit_r2_kinematics::HIGH_PRIO_LINEAR_TOL);  // BOX_Z
    position.constraint_region.primitives.push_back(box);

    position.constraint_region.primitive_poses.push_back(arm_pose_msg.pose);
    position.header.frame_id = arm_pose_msg.header.frame_id;
    goal_constraints[0].position_constraints.push_back(position);

    // arm orientation
    Eigen::Affine3d armFrame = Eigen::Affine3d::Identity();
    armFrame *= (Eigen::AngleAxisd(1.96901, Eigen::Vector3d::UnitX()) *
                 Eigen::AngleAxisd(-2.97015, Eigen::Vector3d::UnitY()) *
                 Eigen::AngleAxisd(3.13885, Eigen::Vector3d::UnitZ()));
    geometry_msgs::Pose armOrientationMsg;
    tf::poseEigenToMsg(armFrame, armOrientationMsg);

    moveit_msgs::OrientationConstraint orientation;
    orientation.link_name = RIGHT_ARM_END_LINK;
    orientation.orientation = armOrientationMsg.orientation;
    orientation.header.frame_id = "world";
    orientation.weight = 1.0;

    orientation.absolute_x_axis_tolerance = moveit_r2_kinematics::HIGH_PRIO_ANGULAR_TOL;
    orientation.absolute_y_axis_tolerance = moveit_r2_kinematics::HIGH_PRIO_ANGULAR_TOL;
    orientation.absolute_z_axis_tolerance = moveit_r2_kinematics::HIGH_PRIO_ANGULAR_TOL;
    goal_constraints[0].orientation_constraints.push_back(orientation);

    // Torso up at goal
    createARGOSTorsoConstraints(goal_constraints[0]);  // torso up at goal

    // Lets plan
    unsigned int tries = 2;
    bool success = false;
    moveit_msgs::RobotTrajectory trajectory;

    std::string group_name = "legs_right_arm";
    std::string planner_name = "XXL";
    double max_time = 30.0; // seconds

    // Try a few times before giving up.  Sometimes we get a crappy goal state
    for(size_t attempt = 0; attempt < tries && !success; ++attempt)
    {
        if (interface.plan(startStateMsg, path_constraints, goal_constraints, group_name,
                           max_time, planner_name, trajectory))
        {
            ROS_WARN("Planning to %s is glorious success", leg_handrail.c_str());
            success = true;
        }
        else
            ROS_ERROR("Planning attempt %lu to %s brings shame upon you", attempt, leg_handrail.c_str());
    }

    bool stored = success;
    if (success)
    {
        // Extracting the next "start" state from the end of the most recently computed trajectory
        std::map<std::string, double> newValues;
        const trajectory_msgs::JointTrajectory& nextStateMsg = trajectory.joint_trajectory;
        for(size_t j = 0; j < nextStateMsg.joint_names.size(); ++j)
            newValues[nextStateMsg.joint_names[j]] = nextStateMsg.points.back().positions[j];

        const trajectory_msgs::MultiDOFJointTrajectory& multiDOFTraj = trajectory.multi_dof_joint_trajectory;
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
        moveit::core::robotStateToRobotStateMsg(*(currentState.get()), endState);


        char ch;
        std::cout << "Save request to db? [y/n] ";
        std::cin >> ch;

        if (ch == 'y' || ch == 'Y')
        {
            std::stringstream str;
            str << leg_handrail << "_r_" << arm_handrail << "_" << count;
            interface.storeRobotState(str.str(), endState);

            moveit_msgs::MotionPlanRequest request;
            interface.createMotionPlanRequest(startStateMsg, path_constraints, goal_constraints,
                                              group_name, max_time, planner_name, tries, request);

            str.str("");
            str << "step_" << leg_handrail << "_r_" << arm_handrail << "_" << count;
            interface.storeMotionPlanRequest(request, str.str(), "ISS");

            count++;
            stored = true;
        }
    }

    return stored;
}

void makeDB(R2Interface& interface, ISSWorld& world)
{
    // Store the ISS and handrail geometry under 'iss'
    interface.storeCurrentPlanningScene("ISS");

    // Initial states
    // Unstow
    moveit_msgs::RobotState currentStateMsg;
    moveit::core::robotStateToRobotStateMsg(interface.getCurrentRobotState(), currentStateMsg);
    interface.storeRobotState("unstow", currentStateMsg);

    //if (!interface.loadRobotState("Handrail_port_0_3", currentStateMsg))
    //    throw;

    bool leftLegFixed = false;
    std::vector<std::string> handrailPlacements;
    handrailPlacements.push_back("Handrail_deck_1");   // left
    handrailPlacements.push_back("Handrail_port_0");   // right
    handrailPlacements.push_back("Handrail_stbd_2");   // left

    for(size_t i = 0; i < handrailPlacements.size(); ++i)
    {
        int count = 0;
        bool done = false;
        char ch;
        int attempt = 0;
        while (!done)
        {
            moveit_msgs::RobotState resultStateMsg;
            if (planToHandrail(interface, world, leftLegFixed, handrailPlacements[i], currentStateMsg, resultStateMsg, attempt++ < 2, count))
            {
                count++;
            }

            std::cout << "Done with this handrail? [y/n] ";
            std::cin >> ch;

            if (ch == 'y' || ch == 'Y')
            {
                done = true;
                currentStateMsg = resultStateMsg;
            }
        }

        leftLegFixed = !leftLegFixed;
    }
}

void makeDBLegsRArm(R2Interface& interface, ISSWorld& world)
{
    // Store the ISS and handrail geometry
    //interface.storeCurrentPlanningScene("ISS_arms");

    // Initial states: unstow
    robot_state::RobotStatePtr currentState = interface.allocRobotState();
    *currentState = interface.getCurrentRobotState();
    currentState->update();

    moveit_msgs::RobotState currentStateMsg;
    moveit::core::robotStateMsgToRobotState(currentStateMsg, *(currentState.get()));

    //interface.storeRobotState("unstow", currentStateMsg);

    // Plan to first handrail, but also move arm somewhere
    bool leftLegFixed = false;
    std::string handrail = "Handrail_deck_1";
    std::string fixedLink, movingLink;
    fixedLink = leftLegFixed ? LEFT_LEG_END_LINK : RIGHT_LEG_END_LINK;
    movingLink = leftLegFixed ? RIGHT_LEG_END_LINK : LEFT_LEG_END_LINK;
    std::cout << "Fixing link " << fixedLink << "  and moving link " << movingLink << std::endl;

    // Fix collisions with handrails
    adjustAllowedHandrailCollisions(interface, currentState, world, leftLegFixed, handrail);


    // Setting up constraints for entire path
    moveit_msgs::Constraints path_constraints;

    // base link pose constraint for whole path
    geometry_msgs::Pose base_pose;
    tf::poseEigenToMsg(currentState->getGlobalLinkTransform(fixedLink), base_pose);
    createBaseLinkConstraints(path_constraints, fixedLink, base_pose);


    // Setting up goal constraints
    std::vector<moveit_msgs::Constraints> goal_constraints(1);

    // goal pose for leg
    Eigen::Affine3d goal_pose_leg = getGoalPose(world, handrail);
    geometry_msgs::PoseStamped leg_pose_msg;
    tf::poseEigenToMsg(goal_pose_leg, leg_pose_msg.pose);
    leg_pose_msg.header.frame_id = "world";
    addGoalConstraints(movingLink, leg_pose_msg, goal_constraints[0]);

    // goal pose for hand
    // Position
    Eigen::Affine3d goal_pose_hand = currentState->getGlobalLinkTransform(RIGHT_ARM_END_LINK);
    geometry_msgs::PoseStamped hand_pose_msg;
    tf::poseEigenToMsg(goal_pose_hand, hand_pose_msg.pose);
    hand_pose_msg.pose.position.x = -1.8;
    hand_pose_msg.pose.position.y = -0.55;
    hand_pose_msg.pose.position.z = 0.0;
    hand_pose_msg.header.frame_id = "world";

    moveit_msgs::PositionConstraint position;
    position.link_name = RIGHT_ARM_END_LINK;
    position.target_point_offset.x = 0.0;
    position.target_point_offset.y = 0.0;
    position.target_point_offset.z = 0.0;
    position.weight = 1.0;

    shape_msgs::SolidPrimitive box;
    box.type = shape_msgs::SolidPrimitive::BOX;
    box.dimensions.push_back(moveit_r2_kinematics::HIGH_PRIO_LINEAR_TOL);  // BOX_X
    box.dimensions.push_back(moveit_r2_kinematics::HIGH_PRIO_LINEAR_TOL);  // BOX_Y
    box.dimensions.push_back(moveit_r2_kinematics::HIGH_PRIO_LINEAR_TOL);  // BOX_Z
    position.constraint_region.primitives.push_back(box);

    position.constraint_region.primitive_poses.push_back(hand_pose_msg.pose);
    position.header.frame_id = hand_pose_msg.header.frame_id;
    goal_constraints[0].position_constraints.push_back(position);

    // Torso up at goal
    //createARGOSTorsoConstraints(goal_constraints[0]);  // torso up at goal

    // Same torso orientation at goal
    Eigen::Affine3d frame = currentState->getGlobalLinkTransform("r2/robot_world");
    geometry_msgs::Pose torso_pose_msg;
    tf::poseEigenToMsg(frame, torso_pose_msg);
    moveit_msgs::OrientationConstraint or_constraint;
    or_constraint.link_name = "r2/robot_world";
    or_constraint.orientation = torso_pose_msg.orientation;
    or_constraint.header.frame_id = "world";
    or_constraint.weight = 1.0;
    or_constraint.absolute_x_axis_tolerance = moveit_r2_kinematics::CRITICAL_PRIO_ANGULAR_TOL;
    or_constraint.absolute_y_axis_tolerance = moveit_r2_kinematics::CRITICAL_PRIO_ANGULAR_TOL;
    or_constraint.absolute_z_axis_tolerance = moveit_r2_kinematics::CRITICAL_PRIO_ANGULAR_TOL; // 6.283185; // 2*PI
    goal_constraints[0].orientation_constraints.push_back(or_constraint);

    // Lets plan
    unsigned int tries = 2;
    bool success = false;
    moveit_msgs::RobotTrajectory trajectory;

    std::string group_name = "legs_right_arm";
    std::string planner_name = "XXL";//"RRT";
    double max_time = 30.0; // seconds

    int count = 0;
    bool done = false;

    // Try a few times before giving up.  Sometimes we get a crappy goal state
    //for(size_t attempt = 0; attempt < tries && !done; ++attempt)
    while (!done)
    {
        moveit_msgs::RobotState endState;

        if (interface.plan(currentStateMsg, path_constraints, goal_constraints, group_name,
                           max_time, planner_name, trajectory))
        {
            ROS_WARN("Planning is glorious success");

            char ch;
            std::cout << "Save request to db? [y/n] ";
            std::cin >> ch;
            if (ch == 'y' || ch == 'Y')
            {
                // Extracting the next "start" state from the end of the most recently computed trajectory
                std::map<std::string, double> newValues;
                const trajectory_msgs::JointTrajectory& nextStateMsg = trajectory.joint_trajectory;
                for(size_t j = 0; j < nextStateMsg.joint_names.size(); ++j)
                    newValues[nextStateMsg.joint_names[j]] = nextStateMsg.points.back().positions[j];

                const trajectory_msgs::MultiDOFJointTrajectory& multiDOFTraj = trajectory.multi_dof_joint_trajectory;
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

                robot_state::RobotStatePtr xstate = interface.allocRobotState();
                *xstate = interface.getCurrentRobotState();
                xstate->setVariablePositions(newValues);
                xstate->update();

                const Eigen::Affine3d& frame = xstate->getGlobalLinkTransform(RIGHT_ARM_END_LINK);
                Eigen::Vector3d hand_rpy = frame.rotation().eulerAngles(0,1,2);
                std::cout << "Hand rpy: " << hand_rpy(0) << " " << hand_rpy(1) << " " << hand_rpy(2) << std::endl;

                // copy xstate into msg
                moveit::core::robotStateToRobotStateMsg(*xstate, endState);

                // Store both the end state and the query
                std::stringstream str;
                str << handrail << "_r_arm_" << count;
                interface.storeRobotState(str.str(), endState);

                moveit_msgs::MotionPlanRequest request;
                interface.createMotionPlanRequest(currentStateMsg, path_constraints, goal_constraints,
                                                  group_name, max_time, planner_name, tries, request);

                str.str("");
                str << "step_" << handrail << "_r_arm_" << count;
                interface.storeMotionPlanRequest(request, str.str(), "ISS_arms_obs2");

                count++;
            }

        }
        else
            ROS_ERROR("Planning attempt brings shame upon you");

        char ch;
        std::cout << "Done with this request? [y/n] ";
        std::cin >> ch;
        if (ch == 'y' || ch == 'Y')
            done = true;
    }
}

void makeDBLegsRArm2(R2Interface& interface, ISSWorld& world)
{
    // Initial state: right leg on stow handrail, left leg on handrail_deck_1, right arm tucked upward
    moveit_msgs::RobotState currentStateMsg;
    //interface.loadRobotState("Handrail_deck_1_0", currentStateMsg);
    interface.loadRobotState("Handrail_deck_1_r_arm_0", currentStateMsg);

    robot_state::RobotStatePtr currentState = interface.allocRobotState();
    moveit::core::robotStateMsgToRobotState(currentStateMsg, *currentState);
    currentState->update();

    // Plan to first handrail, but also move arm somewhere
    bool leftLegFixed = true;
    std::string leg_handrail = "Handrail_deck_2";
    std::string fixedLink, movingLink;
    fixedLink = leftLegFixed ? LEFT_LEG_END_LINK : RIGHT_LEG_END_LINK;
    movingLink = leftLegFixed ? RIGHT_LEG_END_LINK : LEFT_LEG_END_LINK;
    std::cout << "Fixing link " << fixedLink << "  and moving link " << movingLink << std::endl;

    bool left_arm = false; // moving right arm
    std::string arm_handrail = "Handrail_stbd_1";

    // Fix collisions with handrails
    adjustAllowedHandrailCollisions(interface, currentState, world, leftLegFixed, leg_handrail);
    adjustAllowedHandrailCollisionsArm(interface, currentState, world, left_arm, arm_handrail);


    // Setting up constraints for entire path
    moveit_msgs::Constraints path_constraints;

    // base link pose constraint for whole path
    geometry_msgs::Pose base_pose;
    tf::poseEigenToMsg(currentState->getGlobalLinkTransform(fixedLink), base_pose);
    createBaseLinkConstraints(path_constraints, fixedLink, base_pose);


    // Setting up goal constraints
    std::vector<moveit_msgs::Constraints> goal_constraints(1);

    // goal pose for free leg
    Eigen::Affine3d goal_pose_leg = getGoalPose(world, leg_handrail);
    geometry_msgs::PoseStamped leg_pose_msg;
    tf::poseEigenToMsg(goal_pose_leg, leg_pose_msg.pose);
    leg_pose_msg.header.frame_id = "world";

    // create goal constraints for leg pose
    moveit_msgs::PositionConstraint leg_position;
    leg_position.link_name = movingLink;
    leg_position.target_point_offset.x = 0.0;
    leg_position.target_point_offset.y = 0.0;
    leg_position.target_point_offset.z = 0.0;
    leg_position.weight = 1.0;

    shape_msgs::SolidPrimitive box;
    box.type = shape_msgs::SolidPrimitive::BOX;
    box.dimensions.push_back(moveit_r2_kinematics::CRITICAL_PRIO_LINEAR_TOL);  // BOX_X
    box.dimensions.push_back(moveit_r2_kinematics::CRITICAL_PRIO_LINEAR_TOL);  // BOX_Y
    box.dimensions.push_back(moveit_r2_kinematics::CRITICAL_PRIO_LINEAR_TOL);  // BOX_Z
    leg_position.constraint_region.primitives.push_back(box);

    leg_position.constraint_region.primitive_poses.push_back(leg_pose_msg.pose);
    leg_position.header.frame_id = "world";
    goal_constraints[0].position_constraints.push_back(leg_position);

    moveit_msgs::OrientationConstraint leg_orientation;
    leg_orientation.link_name = movingLink;
    leg_orientation.orientation = leg_pose_msg.pose.orientation;
    leg_orientation.header.frame_id = "world";
    leg_orientation.weight = 1.0;

    leg_orientation.absolute_x_axis_tolerance = moveit_r2_kinematics::CRITICAL_PRIO_ANGULAR_TOL;
    leg_orientation.absolute_y_axis_tolerance = moveit_r2_kinematics::CRITICAL_PRIO_ANGULAR_TOL;
    leg_orientation.absolute_z_axis_tolerance = moveit_r2_kinematics::CRITICAL_PRIO_ANGULAR_TOL;
    goal_constraints[0].orientation_constraints.push_back(leg_orientation);

    // goal position for arm
    {
    Eigen::Affine3d goal_pose_hand = Eigen::Affine3d::Identity();
    const ISSWorld::Handrail* hr = world.getHandrail(arm_handrail);
    goal_pose_hand.translation() = hr->pose.translation();
    currentState->getGlobalLinkTransform(RIGHT_ARM_END_LINK);
    geometry_msgs::Pose hand_pose;
    tf::poseEigenToMsg(goal_pose_hand, hand_pose);
    hand_pose.position.y -= 0.15;
    hand_pose.position.z -=  0.15;

    // position constraint for hand
    moveit_msgs::PositionConstraint arm_position;
    arm_position.link_name = RIGHT_ARM_END_LINK;
    arm_position.target_point_offset.x = 0.0;
    arm_position.target_point_offset.y = 0.0;
    arm_position.target_point_offset.z = 0.0;
    arm_position.weight = 1.0;

    shape_msgs::SolidPrimitive box;
    box.type = shape_msgs::SolidPrimitive::BOX;
    box.dimensions.push_back(moveit_r2_kinematics::HIGH_PRIO_LINEAR_TOL);  // BOX_X
    box.dimensions.push_back(moveit_r2_kinematics::HIGH_PRIO_LINEAR_TOL);  // BOX_Y
    box.dimensions.push_back(moveit_r2_kinematics::HIGH_PRIO_LINEAR_TOL);  // BOX_Z
    arm_position.constraint_region.primitives.push_back(box);

    arm_position.constraint_region.primitive_poses.push_back(hand_pose);
    arm_position.header.frame_id = "world";
    goal_constraints[0].position_constraints.push_back(arm_position);
    }

    // torso up at goal with specific yaw
    {
    geometry_msgs::Pose pose_msg;
    pose_msg.orientation.x = 0;
    pose_msg.orientation.y = 0;
    pose_msg.orientation.z = 0.70710678118;
    pose_msg.orientation.w = 0.70710678118;

    moveit_msgs::OrientationConstraint or_constraint;
    or_constraint.link_name = "r2/robot_world";
    or_constraint.orientation = pose_msg.orientation;
    or_constraint.header.frame_id = "world";
    or_constraint.weight = 1.0;

    or_constraint.absolute_x_axis_tolerance = moveit_r2_kinematics::CRITICAL_PRIO_ANGULAR_TOL;
    or_constraint.absolute_y_axis_tolerance = moveit_r2_kinematics::CRITICAL_PRIO_ANGULAR_TOL;
    or_constraint.absolute_z_axis_tolerance = moveit_r2_kinematics::CRITICAL_PRIO_ANGULAR_TOL;

    goal_constraints[0].orientation_constraints.push_back(or_constraint);
    }


    // Torso up at goal
    //createARGOSTorsoConstraints(goal_constraints[0]);  // torso up at goal

    // Lets plan
    unsigned int tries = 2;
    bool success = false;
    moveit_msgs::RobotTrajectory trajectory;

    std::string group_name = "legs_right_arm";
    std::string planner_name = "XXL";  //"RRT";
    double max_time = 90.0; // seconds

    int count = 0;
    bool done = false;

    // Try a few times before giving up.  Sometimes we get a crappy goal state
    //for(size_t attempt = 0; attempt < tries && !done; ++attempt)
    while (!done)
    {
        moveit_msgs::RobotState endState;

        if (interface.plan(currentStateMsg, path_constraints, goal_constraints, group_name,
                           max_time, planner_name, trajectory))
        {
            ROS_WARN("Planning is glorious success");

            // Extracting the next "start" state from the end of the most recently computed trajectory
            std::map<std::string, double> newValues;
            const trajectory_msgs::JointTrajectory& nextStateMsg = trajectory.joint_trajectory;
            for(size_t j = 0; j < nextStateMsg.joint_names.size(); ++j)
                newValues[nextStateMsg.joint_names[j]] = nextStateMsg.points.back().positions[j];

            const trajectory_msgs::MultiDOFJointTrajectory& multiDOFTraj = trajectory.multi_dof_joint_trajectory;
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

            robot_state::RobotStatePtr xstate = interface.allocRobotState();
            *xstate = interface.getCurrentRobotState();
            xstate->setVariablePositions(newValues);
            xstate->update();

            const Eigen::Affine3d& frame = xstate->getGlobalLinkTransform(RIGHT_ARM_END_LINK);
            Eigen::Vector3d hand_rpy = frame.rotation().eulerAngles(0,1,2);
            std::cout << "Hand rpy: " << hand_rpy(0) << " " << hand_rpy(1) << " " << hand_rpy(2) << std::endl;

            char ch;
            std::cout << "Save request to db? [y/n] ";
            std::cin >> ch;
            if (ch == 'y' || ch == 'Y')
            {
                // copy xstate into msg
                moveit::core::robotStateToRobotStateMsg(*xstate, endState);

                // Store both the end state and the query
                std::stringstream str;
                str << leg_handrail << "_r_arm_" << arm_handrail << "_" << count;
                interface.storeRobotState(str.str(), endState);

                moveit_msgs::MotionPlanRequest request;
                interface.createMotionPlanRequest(currentStateMsg, path_constraints, goal_constraints,
                                                  group_name, max_time, planner_name, tries, request);

                str.str("");
                //str << "step_" << handrail << "_r_arm_" << count;
                str << "step_ " << leg_handrail << "_r_arm_" << arm_handrail << "_" << count;
                interface.storeMotionPlanRequest(request, str.str(), "ISS_arms_obs2");

                count++;
            }

        }
        else
            ROS_ERROR("Planning attempt brings shame upon you");

        char ch;
        std::cout << "Done with this request? [y/n] ";
        std::cin >> ch;
        if (ch == 'y' || ch == 'Y')
            done = true;
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "make_db");
    ros::AsyncSpinner spinner(0); // use # threads = num cores
    spinner.start();

    // Create instance of R2Interface for all your planning needs
    R2Interface r2Interface("legs_right_arm");
    sleep(1);

    // Load the planning representation of the handrails
    ISSWorld world;
    createPlanningScene(r2Interface, world);
    addExtraObstacle(r2Interface);
    sleep(4);  // wait until everything is synced

    //makeDB(r2Interface, world);  // for legs only
    //makeDBLegsRArm(r2Interface, world); // legs_right_arm
    //makeDBLegsRArm2(r2Interface, world); // legs_right_arm


    // add unstow state to warehouse
    robot_state::RobotStatePtr currentState = r2Interface.allocRobotState();
    *currentState = r2Interface.getCurrentRobotState();
    currentState->update();

    moveit_msgs::RobotState currentStateMsg;
    moveit::core::robotStateMsgToRobotState(currentStateMsg, *(currentState.get()));


    moveit_msgs::WorkspaceParameters workspace;
    workspace.min_corner.x = -3.0;
    workspace.min_corner.y = -2.0;
    workspace.min_corner.z = -1.1;
    workspace.max_corner.x = 1.1;
    workspace.max_corner.y = 1.5;
    workspace.max_corner.z = 1.5;
    r2Interface.setWorkspace(workspace);


    //r2Interface.storeRobotState("UNNnstow", currentStateMsg);

    // Fancy new one with an obstacle for the arm and right leg
    //r2Interface.storeCurrentPlanningScene("ISS_arms_obs2");
    sleep(1);
    //makeDBLegsRArm(r2Interface, world); // legs_right_arm
    makeDBLegsRArm2(r2Interface, world); // legs_right_arm

    sleep(1);  // wait until everything is synced
}