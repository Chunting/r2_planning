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
    or_constraint.header.frame_id = "virtual_world";
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

    constraints.push_back(goal);
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
    pose_msg.header.frame_id = "virtual_world";

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

int main(int argc, char** argv)
{
    ros::init(argc, argv, "make_db");
    ros::AsyncSpinner spinner(0); // use # threads = num cores
    spinner.start();

    // Create instance of R2Interface for all your planning needs
    R2Interface r2Interface;
    sleep(1);

    // Load the planning representation of the handrails
    ISSWorld world;
    createPlanningScene(r2Interface, world);
    sleep(4);  // wait until everything is synced

    makeDB(r2Interface, world);
    sleep(1);  // wait until everything is synced
}