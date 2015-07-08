// R2Interface provides common planning functionalities for R2
// Author: Ryan Luna
// May 2015

#ifdef USE_BOOST_FILESYSTEM_V2
#define BOOST_FILESYSTEM_VERSION 2
#endif

#include "r2_planning_interface/R2Interface.h"
#include "r2_planning_interface/R2Poses.h"

#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>

#include <moveit/move_group/capability_names.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

#include <moveit/warehouse/planning_scene_storage.h>
#include <moveit/warehouse/state_storage.h>
#include <moveit/warehouse/constraints_storage.h>

#include <moveit_msgs/GetPositionIK.h>
#include <moveit_msgs/GetMotionPlan.h>
#include <moveit_msgs/ExecuteKnownTrajectory.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_r2_kinematics/tree_kinematics_tolerances.h>

static const char* const JOINT_STATES_TOPIC            = "r2_joint_states";                         // publish new joint values here
static const char* const COLLISION_OBJECT_TOPIC        = "collision_object";                        // publish obstacles here
static const char* const PLANNING_SCENE_UPDATE_TOPIC   = "/move_group/monitored_planning_scene";    // listen for planning scene updates here
static const char* const PLANNING_SCENE_PUBLISH_TOPIC  = "planning_scene";                          // publish changes to planning scene here
static const char* const ATTACHED_OBJECT_TOPIC         = "attached_collision_object";               // publish here when we attach/detach from a handrail
static const char* const RVIZ_DISPLAY_TRAJECTORY_TOPIC = "/move_group/display_planned_path";        // publish trajectories to visualize here

// Explicitly NOT loading kinematics solvers.  Takes a long time, and we don't use them here.  Use move_group for kinematics
R2Interface::R2Interface(const std::string& robot_description) : psm_(new planning_scene_monitor::PlanningSceneMonitor(robot_model_loader::RobotModelLoaderPtr(new robot_model_loader::RobotModelLoader(robot_description, false)))), fakeLocalize_(false), gazebo_(false)
{
    jointPublisher_ = nh_.advertise<sensor_msgs::JointState>(JOINT_STATES_TOPIC, 0);
    obstaclePublisher_ = nh_.advertise<moveit_msgs::CollisionObject>(COLLISION_OBJECT_TOPIC, 0);
    scenePublisher_ = nh_.advertise<moveit_msgs::PlanningScene>(PLANNING_SCENE_PUBLISH_TOPIC, 0);
    attachPublisher_ = nh_.advertise<moveit_msgs::AttachedCollisionObject>(ATTACHED_OBJECT_TOPIC, 0);
    vizPublisher_ = nh_.advertise<moveit_msgs::DisplayTrajectory>(RVIZ_DISPLAY_TRAJECTORY_TOPIC, 0);

    psm_->addUpdateCallback(boost::bind(&R2Interface::planningSceneUpdate, this, _1));
    psm_->startSceneMonitor(PLANNING_SCENE_UPDATE_TOPIC);
    planningSceneDirty_ = true;

    workspace_.min_corner.x = workspace_.min_corner.y = workspace_.min_corner.z = 0;
    workspace_.max_corner.x = workspace_.max_corner.y = workspace_.max_corner.z = 0;

    virtualJointName_ = "virtual_joint";
    worldFrame_ = Eigen::Affine3d::Identity();

    // HACK.  TODO: Fix this
    std::vector<std::string> tip_frames;
    tip_frames.push_back("r2/right_ankle_roll");
    tip_frames.push_back("r2/left_ankle_roll");
    std::string group_name("legs");
    std::string base_frame("virtual_world");
    treeKinematics_ = new moveit_r2_kinematics::R2TreeKinematicsInterface();
    if(!treeKinematics_->initialize(robot_description, group_name, base_frame, tip_frames))
        ROS_ERROR("%s: Failed to initialize tree kinematics interface.  TreeIK will not work", "R2Interface");
}

R2Interface::~R2Interface()
{
    if (fakeLocalize_)
    {
        fakeLocalize_ = false;
        localizationThread_.join();
    }

    delete treeKinematics_;
}

bool R2Interface::storeCurrentPlanningScene(const std::string& name) const
{
    moveit_msgs::PlanningScene scene;
    getPlanningScene()->getPlanningSceneMsg(scene);
    scene.name = name;
    return storePlanningScene(scene);
}

bool R2Interface::storePlanningScene(const moveit_msgs::PlanningScene& scene) const
{
    try
    {
        moveit_warehouse::PlanningSceneStorage pss;

        if (pss.hasPlanningScene(scene.name))
            ROS_WARN("Overwriting existing scene '%s' in warehouse", scene.name.c_str());
        pss.addPlanningScene(scene);
    }
    catch(std::runtime_error)
    {
        ROS_ERROR("R2Interface: Unable to store planning scene");
        return false;
    }

    return true;
}

bool R2Interface::loadPlanningScene(const std::string& name, moveit_msgs::PlanningScene& scene) const
{
    try
    {
        moveit_warehouse::PlanningSceneStorage pss;

        if (!pss.hasPlanningScene(name))
        {
            ROS_ERROR("No scene '%s' in warehouse", name.c_str());
            return false;
        }

        moveit_warehouse::PlanningSceneWithMetadata pswm;
        bool ok = pss.getPlanningScene(pswm, name);
        if (ok)
            scene = static_cast<moveit_msgs::PlanningScene>(*pswm);
        return ok;
    }
    catch(std::runtime_error)
    {
        ROS_ERROR("R2Interface: Unable to get planning scene");
    }

    return false;
}

bool R2Interface::storeCurrentRobotState(const std::string& name) const
{
    moveit_msgs::RobotState state;
    moveit::core::robotStateToRobotStateMsg(getCurrentRobotState(), state);

    return storeRobotState(name, state);
}

bool R2Interface::storeRobotState(const std::string& name, const moveit_msgs::RobotState& state) const
{
    try
    {
        moveit_warehouse::RobotStateStorage rss;
        if (rss.hasRobotState(name, "R2"))
            ROS_WARN("Overwriting existing state '%s' in warehouse", name.c_str());

        rss.addRobotState(state, name, "R2");
    }
    catch(std::runtime_error)
    {
        ROS_ERROR("R2Interface: Unable to store robot state");
        return false;
    }

    return true;
}

bool R2Interface::loadRobotState(const std::string& name, moveit_msgs::RobotState& state) const
{
    try
    {
        moveit_warehouse::RobotStateStorage rss;
        if (!rss.hasRobotState(name, "R2"))
        {
            ROS_ERROR("No state '%s' in warehouse", name.c_str());
            return false;
        }

        moveit_warehouse::RobotStateWithMetadata rswm;
        bool ok = rss.getRobotState(rswm, name, "R2");

        if (ok)
            state = static_cast<moveit_msgs::RobotState>(*rswm);
        return ok;
    }
    catch(std::runtime_error)
    {
        ROS_ERROR("R2Interface: Unable to load robot state");
    }

    return false;
}

bool R2Interface::storeMotionPlanRequest(const moveit_msgs::MotionPlanRequest& request, const std::string& requestName, const std::string& planningSceneName) const
{
    try
    {
        moveit_warehouse::PlanningSceneStorage pss;

        if (!pss.hasPlanningScene(planningSceneName))
            ROS_WARN("No scene named '%s' in warehouse", planningSceneName.c_str());
        else if (pss.hasPlanningQuery(planningSceneName, requestName))
            ROS_WARN("Overwriting existing query '%s' for scene '%s'", requestName.c_str(), planningSceneName.c_str());

        pss.addPlanningQuery(request, planningSceneName, requestName);
    }
    catch(std::runtime_error)
    {
        ROS_ERROR("R2Interface: Unable to store planning request");
        return false;
    }

    return true;
}

bool R2Interface::loadMotionPlanRequest(moveit_msgs::MotionPlanRequest& request, const std::string& requestName, const std::string& planningSceneName) const
{
    try
    {
        moveit_warehouse::PlanningSceneStorage pss;
        if (!pss.hasPlanningQuery(planningSceneName, requestName))
        {
            ROS_ERROR("No query '%s' exists with scene '%s'", requestName.c_str(), planningSceneName.c_str());
            return false;
        }

        moveit_warehouse::MotionPlanRequestWithMetadata mprwm;
        bool ok = pss.getPlanningQuery(mprwm, planningSceneName, requestName);
        if (ok)
            request = static_cast<moveit_msgs::MotionPlanRequest>(*mprwm);
        return ok;
    }
    catch(std::runtime_error)
    {
        ROS_ERROR("R2Interface: Unable to store planning request");
    }

    return false;
}

bool R2Interface::storeConstraints(const std::string& name, const moveit_msgs::Constraints& constraints) const
{
    try
    {
        moveit_warehouse::ConstraintsStorage cs;

        if (cs.hasConstraints(name))
            ROS_WARN("Overwriting existing constraints with name '%s'", name.c_str());

        cs.addConstraints(constraints, "R2");
    }
    catch(std::runtime_error)
    {
        ROS_ERROR("R2Interface: Unable to store constraints");
        return false;
    }

    return true;
}

bool R2Interface::loadConstraints(const std::string& name, moveit_msgs::Constraints& constraints) const
{
    try
    {
        moveit_warehouse::ConstraintsStorage cs;

        if (!cs.hasConstraints(name))
        {
            ROS_ERROR("No constraints found with name '%s'", name.c_str());
            return false;
        }

        moveit_warehouse::ConstraintsWithMetadata cwmd;
        bool ok = cs.getConstraints(cwmd, name, "R2");
        if (ok)
            constraints = static_cast<moveit_msgs::Constraints>(*cwmd);
        return ok;
    }
    catch(std::runtime_error)
    {
        ROS_ERROR("R2Interface: Unable to store constraints");
    }

    return false;
}

void R2Interface::clearTrajectory() const
{
    const robot_state::RobotState& current_state = getCurrentRobotState();

    // Clear the trajectory by sending an "empty" trajectory
    // Need to put one point in the trajectory.
    moveit_msgs::RobotTrajectory trajectory;
    trajectory.joint_trajectory.joint_names.push_back("r2/left_leg/joint0");
    trajectory_msgs::JointTrajectoryPoint pt;
    pt.positions.push_back(current_state.getVariablePosition("r2/left_leg/joint0"));
    trajectory.joint_trajectory.points.push_back(pt);

    moveit_msgs::DisplayTrajectory disp_trajectory;
    disp_trajectory.model_id = "r2";
    disp_trajectory.trajectory.push_back(trajectory);

    moveit_msgs::RobotState current_state_msg;
    robot_state::robotStateToRobotStateMsg(current_state, current_state_msg);
    disp_trajectory.trajectory_start = current_state_msg;

    vizPublisher_.publish(disp_trajectory);
}

void R2Interface::viewTrajectory(const moveit_msgs::RobotTrajectory& trajectory)
{
    viewTrajectory(getCurrentRobotState(), trajectory);
}

void R2Interface::viewTrajectory(const std::vector<moveit_msgs::RobotTrajectory>& trajectories)
{
    viewTrajectory(getCurrentRobotState(), trajectories);
}

void R2Interface::viewTrajectory(const robot_state::RobotState& initial_state, const moveit_msgs::RobotTrajectory& trajectory)
{
    moveit_msgs::DisplayTrajectory disp_trajectory;
    disp_trajectory.model_id = "r2";
    disp_trajectory.trajectory.push_back(trajectory);

    moveit_msgs::RobotState current_state_msg;
    robot_state::robotStateToRobotStateMsg(initial_state, current_state_msg);
    disp_trajectory.trajectory_start = current_state_msg;

    vizPublisher_.publish(disp_trajectory);
}

void R2Interface::viewTrajectory(const robot_state::RobotState& initial_state, const std::vector<moveit_msgs::RobotTrajectory>& trajectories)
{
    moveit_msgs::DisplayTrajectory disp_trajectory;
    disp_trajectory.model_id = "r2";
    for (size_t i = 0; i < trajectories.size(); ++i)
        disp_trajectory.trajectory.push_back(trajectories[i]);

    moveit_msgs::RobotState current_state_msg;
    robot_state::robotStateToRobotStateMsg(initial_state, current_state_msg);
    disp_trajectory.trajectory_start = current_state_msg;

    vizPublisher_.publish(disp_trajectory);
}

void R2Interface::planningSceneUpdate(planning_scene_monitor::PlanningSceneMonitor::SceneUpdateType type)
{
    //std::cout << "Planning scene updated " << type << std::endl;
    planningSceneDirty_ = false;
}

const robot_state::RobotState& R2Interface::getCurrentRobotState(void) const
{
    // DO NOT DO THIS!  Race condition on planning scene.  Things will crash eventually.
    //return psm_->getPlanningScene()->getCurrentState();

    // MUCH better.  Uses LockedPlanningSceneRO
    if (planningSceneDirty_)
        ROS_WARN("Planning scene is dirty.  Current state may be incorrect.");
    return getPlanningScene()->getCurrentState();
}

robot_state::RobotStatePtr R2Interface::allocRobotState(void) const
{
    // TODO: Cache a RobotModel instead of using planning scene?  Model should not change...
    robot_state::RobotStatePtr state(new robot_state::RobotState(getPlanningScene()->getRobotModel()));
    return state;
}

planning_scene_monitor::LockedPlanningSceneRO R2Interface::getPlanningScene() const
{
    planning_scene_monitor::LockedPlanningSceneRO psro(psm_);
    return psro;
}

void R2Interface::setPlanningScene(moveit_msgs::PlanningScene& scene)
{
    planningSceneDirty_ = true;
    scenePublisher_.publish(scene);
}

void R2Interface::addObstacleToPlanningScene(const moveit_msgs::CollisionObject& obstacle)
{
    planningSceneDirty_ = true;
    obstaclePublisher_.publish(obstacle);
}

void R2Interface::enableCollisionChecking(const std::string& link1, const std::string& link2, bool allow, bool waitForAck)
{
    collision_detection::AllowedCollisionMatrix acm(getPlanningScene()->getAllowedCollisionMatrix());
    acm.setEntry(link1, link2, allow);

    moveit_msgs::PlanningScene scene_msg;
    getPlanningScene()->getPlanningSceneMsg(scene_msg);
    acm.getMessage(scene_msg.allowed_collision_matrix);
    scene_msg.is_diff = true;

    planningSceneDirty_ = true;
    scenePublisher_.publish(scene_msg);

    if (waitForAck)
    {
        double waitTime = 10000; // us
        double maxWait = 2000000; // us (2 sec)
        double wait = 0.0;
        do
        {
            ros::spinOnce();  // mas importante
            usleep(waitTime);
            wait += waitTime;

            if (!planningSceneDirty_)  // make sure update included our matrix change
            {
                collision_detection::AllowedCollisionMatrix acm(getPlanningScene()->getAllowedCollisionMatrix());
                collision_detection::AllowedCollision::Type collisionType = allow ? collision_detection::AllowedCollision::ALWAYS : collision_detection::AllowedCollision::NEVER;
                if (!acm.getEntry(link1, link2, collisionType))
                    planningSceneDirty_ = true;
            }
        }
        while (planningSceneDirty_ && wait < maxWait);  // don't actually wait forever

        if (planningSceneDirty_)
            ROS_WARN("%s: ACK timeout exceeded for %s and %s", __FUNCTION__, link1.c_str(), link2.c_str());
    }
}

void R2Interface::enableCollisionChecking(const std::vector<std::string> bodies, bool allow, bool waitForAck)
{
    if (bodies.size() < 2)
        return;

    collision_detection::AllowedCollisionMatrix acm(getPlanningScene()->getAllowedCollisionMatrix());
    for(size_t i = 0; i < bodies.size(); ++i)
        for(size_t j = i+1; j < bodies.size(); ++j)
            acm.setEntry(bodies[i], bodies[j], allow);

    moveit_msgs::PlanningScene scene_msg;
    getPlanningScene()->getPlanningSceneMsg(scene_msg);
    acm.getMessage(scene_msg.allowed_collision_matrix);
    scene_msg.is_diff = true;

    planningSceneDirty_ = true;
    scenePublisher_.publish(scene_msg);

    if (waitForAck)
    {
        double waitTime = 10000; // us
        double maxWait = 2000000; // us (2 sec)
        double wait = 0.0;
        while (planningSceneDirty_ && wait < maxWait)  // don't actually wait forever
        {
            ros::spinOnce(); // mas importante
            usleep(waitTime);
            wait += waitTime;

            if (!planningSceneDirty_)  // make sure update included our matrix change
            {
                collision_detection::AllowedCollisionMatrix acm(getPlanningScene()->getAllowedCollisionMatrix());
                collision_detection::AllowedCollision::Type collisionType = allow ? collision_detection::AllowedCollision::ALWAYS : collision_detection::AllowedCollision::NEVER;
                for(size_t i = 0; i < bodies.size(); ++i)
                    for(size_t j = i+1; j < bodies.size(); ++j)
                        if (!acm.getEntry(bodies[i], bodies[j], collisionType))
                            planningSceneDirty_ = true;
            }
        }

        if (planningSceneDirty_)
            ROS_WARN("%s: ACK timeout exceeded", __FUNCTION__);
    }
}

void R2Interface::attachObjectToRobot(const moveit_msgs::AttachedCollisionObject& attached_obj)
{
    attachPublisher_.publish(attached_obj);
}

bool R2Interface::chainIK(const std::string& group, const std::string& link,
                          const geometry_msgs::PoseStamped& pose, robot_state::RobotStatePtr result)
{
    return chainIK(group, link, pose, getCurrentRobotState(), result);
}

// Compute an inverse kinematics solution for the given link and pose in the given group.
// Initialize the IK solver with the given seed values. If a solution is found, store the
// joint values in result.
bool R2Interface::chainIK(const std::string& group, const std::string& link, const geometry_msgs::PoseStamped& pose,
                          const robot_state::RobotState& seed_state, robot_state::RobotStatePtr result)
{
    // Make sure IK service is online
    if (!ros::service::exists(move_group::IK_SERVICE_NAME, 0))
    {
        ROS_ERROR("MoveGroup IK service %s is not advertised.", move_group::IK_SERVICE_NAME.c_str());
        return false;
    }

    // Open up a client to the kinematics service
    ros::ServiceClient kinematics_client = nh_.serviceClient<moveit_msgs::GetPositionIK>(move_group::IK_SERVICE_NAME);

    // Create the IK query
    moveit_msgs::GetPositionIK::Request  ik_req;
    moveit_msgs::GetPositionIK::Response ik_resp;

    // IK for what group?
    ik_req.ik_request.group_name = group;

    // Seeding start state
    sensor_msgs::JointState joint_state_msg;
    moveit::core::robotStateToJointStateMsg(seed_state, joint_state_msg);
    ik_req.ik_request.robot_state.joint_state = joint_state_msg;

    // We absolutely need a collision free IK solution
    ik_req.ik_request.avoid_collisions = true;
    // Try a few times...  VERY IMPORTANT to try more than once
    ik_req.ik_request.attempts = 3;

    ik_req.ik_request.pose_stamped_vector.push_back(pose);
    ik_req.ik_request.ik_link_names.push_back(link);

    // One IK, please
    if(kinematics_client.call(ik_req, ik_resp))
    {
        if (ik_resp.error_code.val != 1)
        {
            if (ik_resp.error_code.val == -31)
                ROS_WARN("No IK solution found.");
            else
                ROS_ERROR("Error when computing IK.  MoveIt! error code %d", ik_resp.error_code.val);
            return false;
        }
    }
    else
    {
        ROS_ERROR("Unknown error when invoking kinematics service");
        return false;
    }

    // Set the goal state from the IK service response
    moveit::core::jointStateToRobotState(ik_resp.solution.joint_state, *(result.get()));
    result->update();  // update frames

    return true;
}

bool R2Interface::treeIK(const std::string& group, const std::string& fixed_link, const std::string& link,
                         const geometry_msgs::PoseStamped& pose, robot_state::RobotStatePtr result)
{
    std::vector<std::string> links;
    links.push_back(link);

    std::vector<geometry_msgs::PoseStamped> poses;
    poses.push_back(pose);

    std::vector<int> priority(6, KdlTreeIk::CRITICAL);
    std::vector<std::vector<int> >priorities;
    priorities.push_back(priority);

    return treeIK(group, fixed_link, links, poses, priorities, getCurrentRobotState(), result);
}

bool R2Interface::treeIK(const std::string& group, const std::string& fixed_link, const std::string& link, const geometry_msgs::PoseStamped& pose,
                         const robot_state::RobotState& seed_state, robot_state::RobotStatePtr result)
{
    std::vector<std::string> links;
    links.push_back(link);

    std::vector<geometry_msgs::PoseStamped> poses;
    poses.push_back(pose);

    std::vector<int> priority(6, KdlTreeIk::CRITICAL);
    std::vector<std::vector<int> >priorities;
    priorities.push_back(priority);

    return treeIK(group, fixed_link, links, poses, priorities, seed_state, result);
}

bool R2Interface::treeIK(const std::string& group, const std::string& fixed_link, const std::string& link,
                         const geometry_msgs::PoseStamped& pose, const std::vector<int>& posePriorities,
                         const robot_state::RobotState& seed_state, robot_state::RobotStatePtr result)
{
    std::vector<std::string> links;
    links.push_back(link);

    std::vector<geometry_msgs::PoseStamped> poses;
    poses.push_back(pose);

    std::vector<std::vector<int> >priorities;
    priorities.push_back(posePriorities);

    return treeIK(group, fixed_link, links, poses, priorities, seed_state, result);
}

bool R2Interface::treeIK(const std::string& group, const std::string& fixed_link, const std::vector<std::string>& links,
                         const std::vector<geometry_msgs::PoseStamped>& poses, const std::vector<std::vector<int> >& posePriorities,
                         const robot_state::RobotState& seed_state, robot_state::RobotStatePtr result)
{
    if (links.size() < 1)
    {
        ROS_ERROR("%s: Must specify at least one node to compute IK for", __FUNCTION__);
        return false;
    }
    if (links.size() != poses.size())
    {
        ROS_ERROR("%s: # Links must equal # poses", __FUNCTION__);
        return false;
    }
    if (links.size() != posePriorities.size())
    {
        ROS_ERROR("%s: # Links must equal # priority vectors", __FUNCTION__);
        return false;
    }

    // Setup kinematics request
    moveit_r2_kinematics::TreeIkRequest request;
    moveit_r2_kinematics::TreeIkResponse response;

    // Setting fixed link
    request.addFixedLink(fixed_link);
    Eigen::Affine3d fixed_pose = seed_state.getGlobalLinkTransform(fixed_link);

    // Setting desired link poses
    for (size_t i = 0; i < links.size(); ++i)
        request.addLinkPose(links[i], poses[i].pose, posePriorities[i]);

    // Total hack to keep torso upright-ish
    //std::vector<int> torsoPriorities(6, KdlTreeIk::LOW);
    //torsoPriorities[0] = torsoPriorities[1] = torsoPriorities[2] = KdlTreeIk::IGNORE; // translation of torso does not matter
    //torsoPriorities[5] = KdlTreeIk::IGNORE; // Ignore yaw as well.  Only constrain roll and pitch to stay the same-ish
    //geometry_msgs::PoseStamped torsoPoseMsg;
    //Eigen::Affine3d torsoPose = seed_state.getGlobalLinkTransform("r2/robot_world");
    //tf::poseEigenToMsg(torsoPose, torsoPoseMsg.pose);
    //request.addLinkPose("r2/robot_world", torsoPoseMsg.pose, torsoPriorities);

    // Getting world pose
    Eigen::Affine3d world_pose = seed_state.getGlobalLinkTransform(seed_state.getJointModel(virtualJointName_)->getChildLinkModel()->getName());
    request.setWorldState(world_pose);

    std::vector<double> joint_values;
    const std::vector<std::string>& all_joints = treeKinematics_->getAllJointNames();
    for (size_t i = 0; i < all_joints.size(); ++i)
        joint_values.push_back(seed_state.getVariablePosition(all_joints[i]));

    request.setJointValues(joint_values);

    if (treeKinematics_->getPositionIk(request, response))
    {
        // Fill out joint_state
        const std::vector<double>& new_values = response.getJointValues();
        std::map<std::string, double> new_values_map;
        const std::vector<std::string>& joint_order = treeKinematics_->getJointNames();
        for (size_t j = 0; j < joint_order.size(); ++j)
            new_values_map[joint_order[j]] = new_values[j];

        // Add the world.   Hard-coded for now.  TODO: Make this less awful
        // tf::Quaternion doesn't yield correct values... :(  Using Eigen::Quaternion
        const Eigen::Affine3d& new_world_pose = response.getWorldState();
        Eigen::Quaternion<double> q(new_world_pose.rotation());

        new_values_map[virtualJointName_ + std::string("/trans_x")] = new_world_pose.translation()[0];
        new_values_map[virtualJointName_ + std::string("/trans_y")] = new_world_pose.translation()[1];
        new_values_map[virtualJointName_ + std::string("/trans_z")] = new_world_pose.translation()[2];
        new_values_map[virtualJointName_ + std::string("/rot_x")] = q.x();
        new_values_map[virtualJointName_ + std::string("/rot_y")] = q.y();
        new_values_map[virtualJointName_ + std::string("/rot_z")] = q.z();
        new_values_map[virtualJointName_ + std::string("/rot_w")] = q.w();

        result->setVariablePositions(new_values_map);
        result->update();  // update frames
        return true;
    }

    return false;
}

void R2Interface::setWorkspace(const moveit_msgs::WorkspaceParameters& workspace)
{
    workspace_ = workspace;
}

const moveit_msgs::WorkspaceParameters& R2Interface::getWorkspace() const
{
    return workspace_;
}

bool R2Interface::plan(const moveit_msgs::RobotState& start_state, const geometry_msgs::PoseStamped& goal_pose,
                       const std::string& goal_link, const std::string& group_name,
                       double max_time, const std::string& planner_name, moveit_msgs::RobotTrajectory& trajectory_msg,
                       unsigned int tries)
{
    moveit_msgs::Constraints no_constraints;
    return plan(start_state, goal_pose, goal_link, group_name, no_constraints, max_time, planner_name, trajectory_msg, tries);
}

bool R2Interface::plan(const moveit_msgs::RobotState& start_state, const geometry_msgs::PoseStamped& goal_pose, const std::string& goal_link, const std::string& group_name,
                       const moveit_msgs::Constraints& path_constraints, double max_time, const std::string& planner_name,
                       moveit_msgs::RobotTrajectory& trajectory_msg, unsigned int tries)
{
    /*// Make sure planning service is online
    if (!ros::service::exists(move_group::PLANNER_SERVICE_NAME, 0))
    {
        ROS_ERROR("Planner service %s is not advertised.", move_group::PLANNER_SERVICE_NAME.c_str());
        return false;
    }
    // Open up a client to the planner service
    ros::ServiceClient plan_client = nh_.serviceClient<moveit_msgs::GetMotionPlan>(move_group::PLANNER_SERVICE_NAME);

    // Create the motion plan query
    moveit_msgs::GetMotionPlan::Request  plan_req;
    moveit_msgs::GetMotionPlan::Response plan_resp;

    // Generic planning information
    plan_req.motion_plan_request.planner_id = planner_name;
    plan_req.motion_plan_request.group_name = group_name;
    plan_req.motion_plan_request.num_planning_attempts = tries;
    plan_req.motion_plan_request.allowed_planning_time = max_time;

    // Workspace bounds
    plan_req.motion_plan_request.workspace_parameters = workspace_;

    // Setting start state
    plan_req.motion_plan_request.start_state = start_state;

    // Setting goal state - this is a series of constraints on the system for the final state
    // Constructing position constraint
    moveit_msgs::PositionConstraint goal_pos_constraint;
    goal_pos_constraint.header.frame_id = goal_pose.header.frame_id;
    goal_pos_constraint.link_name = goal_link;
    goal_pos_constraint.target_point_offset.x = 0;
    goal_pos_constraint.target_point_offset.y = 0;
    goal_pos_constraint.target_point_offset.z = 0;

    // Define a very small bounding box
    shape_msgs::SolidPrimitive box;
    box.type = shape_msgs::SolidPrimitive::BOX;
    box.dimensions.push_back(moveit_r2_kinematics::CRITICAL_PRIO_LINEAR_TOL);  // BOX_X
    box.dimensions.push_back(moveit_r2_kinematics::CRITICAL_PRIO_LINEAR_TOL);  // BOX_Y
    box.dimensions.push_back(moveit_r2_kinematics::CRITICAL_PRIO_LINEAR_TOL);  // BOX_Z
    goal_pos_constraint.constraint_region.primitives.push_back(box);
    goal_pos_constraint.weight = 1.0;

    // Center the bounding volume at the pose location
    goal_pos_constraint.constraint_region.primitive_poses.push_back(goal_pose.pose);

    // Create orientation constraint
    moveit_msgs::OrientationConstraint goal_orn_constraint;
    goal_orn_constraint.header.frame_id = goal_pose.header.frame_id;
    goal_orn_constraint.orientation = goal_pose.pose.orientation;
    goal_orn_constraint.link_name = goal_link;
    goal_orn_constraint.absolute_x_axis_tolerance = moveit_r2_kinematics::CRITICAL_PRIO_ANGULAR_TOL;
    goal_orn_constraint.absolute_y_axis_tolerance = moveit_r2_kinematics::CRITICAL_PRIO_ANGULAR_TOL;
    goal_orn_constraint.absolute_z_axis_tolerance = moveit_r2_kinematics::CRITICAL_PRIO_ANGULAR_TOL;
    goal_orn_constraint.weight = 1.0;

    // Bring constraints together
    moveit_msgs::Constraints goal_constraint_msg;
    goal_constraint_msg.position_constraints.push_back(goal_pos_constraint);
    goal_constraint_msg.orientation_constraints.push_back(goal_orn_constraint);

    // Setting all goal constraints
    plan_req.motion_plan_request.goal_constraints.push_back(goal_constraint_msg);

    // Setting path constraints - constraints that must be respected at all times
    plan_req.motion_plan_request.path_constraints = path_constraints;

    // One motion plan, pretty please
    if(plan_client.call(plan_req, plan_resp))
    {
        if (plan_resp.motion_plan_response.error_code.val != 1)
        {
            ROS_ERROR("Failed to compute motion plan.  Error code %d", plan_resp.motion_plan_response.error_code.val);
            return false;
        }
        else
        {
            trajectory_msg = plan_resp.motion_plan_response.trajectory;
            ROS_INFO("Planning successful.  Time: %f secs.  Trajectory has %lu waypoints", plan_resp.motion_plan_response.planning_time, trajectory_msg.joint_trajectory.points.size());
            //lastPlanningTime_ = plan_resp.motion_plan_response.planning_time;
            return true;
        }
    }

    ROS_ERROR("Unknown error when invoking planning service call.");
    return false;*/

    moveit_msgs::PositionConstraint goal_pos_constraint;
    goal_pos_constraint.header.frame_id = goal_pose.header.frame_id;
    goal_pos_constraint.link_name = goal_link;
    goal_pos_constraint.target_point_offset.x = 0;
    goal_pos_constraint.target_point_offset.y = 0;
    goal_pos_constraint.target_point_offset.z = 0;

    // Define a very small bounding box
    shape_msgs::SolidPrimitive box;
    box.type = shape_msgs::SolidPrimitive::BOX;
    box.dimensions.push_back(moveit_r2_kinematics::CRITICAL_PRIO_LINEAR_TOL);  // BOX_X
    box.dimensions.push_back(moveit_r2_kinematics::CRITICAL_PRIO_LINEAR_TOL);  // BOX_Y
    box.dimensions.push_back(moveit_r2_kinematics::CRITICAL_PRIO_LINEAR_TOL);  // BOX_Z
    goal_pos_constraint.constraint_region.primitives.push_back(box);
    goal_pos_constraint.weight = 1.0;

    // Center the bounding volume at the pose location
    goal_pos_constraint.constraint_region.primitive_poses.push_back(goal_pose.pose);

    // Create orientation constraint
    moveit_msgs::OrientationConstraint goal_orn_constraint;
    goal_orn_constraint.header.frame_id = goal_pose.header.frame_id;
    goal_orn_constraint.orientation = goal_pose.pose.orientation;
    goal_orn_constraint.link_name = goal_link;
    goal_orn_constraint.absolute_x_axis_tolerance = moveit_r2_kinematics::CRITICAL_PRIO_ANGULAR_TOL;
    goal_orn_constraint.absolute_y_axis_tolerance = moveit_r2_kinematics::CRITICAL_PRIO_ANGULAR_TOL;
    goal_orn_constraint.absolute_z_axis_tolerance = moveit_r2_kinematics::CRITICAL_PRIO_ANGULAR_TOL;
    goal_orn_constraint.weight = 1.0;

    std::vector<moveit_msgs::Constraints> goal_constraints(1);
    goal_constraints.back().position_constraints.push_back(goal_pos_constraint);
    goal_constraints.back().orientation_constraints.push_back(goal_orn_constraint);

    return plan(start_state, path_constraints, goal_constraints, group_name, max_time, planner_name, trajectory_msg, tries);
}

bool R2Interface::plan(const moveit_msgs::RobotState& start_state, const moveit_msgs::Constraints& path_constraints,
                       const std::vector<moveit_msgs::Constraints>& goal_constraints, const std::string& group_name, double max_time,
                       const std::string& planner_name, moveit_msgs::RobotTrajectory& trajectory_msg, unsigned int tries)
{
    // Make sure planning service is online
    if (!ros::service::exists(move_group::PLANNER_SERVICE_NAME, 0))
    {
        ROS_ERROR("Planner service %s is not advertised.", move_group::PLANNER_SERVICE_NAME.c_str());
        return false;
    }
    // Open up a client to the planner service
    ros::ServiceClient plan_client = nh_.serviceClient<moveit_msgs::GetMotionPlan>(move_group::PLANNER_SERVICE_NAME);

    // Create the motion plan query
    moveit_msgs::GetMotionPlan::Request  plan_req;
    moveit_msgs::GetMotionPlan::Response plan_resp;

    // Generic planning information
    plan_req.motion_plan_request.planner_id = planner_name;
    plan_req.motion_plan_request.group_name = group_name;
    plan_req.motion_plan_request.num_planning_attempts = tries;
    plan_req.motion_plan_request.allowed_planning_time = max_time;

    // Workspace bounds
    plan_req.motion_plan_request.workspace_parameters = workspace_;

    // Setting start state
    plan_req.motion_plan_request.start_state = start_state;

    // Setting goal constraints
    plan_req.motion_plan_request.goal_constraints = goal_constraints;

    // Setting path constraints - constraints that must be respected at all times
    plan_req.motion_plan_request.path_constraints = path_constraints;

    // One motion plan, pretty please
    if(plan_client.call(plan_req, plan_resp))
    {
        if (plan_resp.motion_plan_response.error_code.val != 1)
        {
            ROS_ERROR("Failed to compute motion plan.  Error code %d", plan_resp.motion_plan_response.error_code.val);
            return false;
        }
        else
        {
            trajectory_msg = plan_resp.motion_plan_response.trajectory;
            ROS_INFO("Planning successful.  Time: %f secs.  Trajectory has %lu waypoints", plan_resp.motion_plan_response.planning_time, trajectory_msg.joint_trajectory.points.size());
            //lastPlanningTime_ = plan_resp.motion_plan_response.planning_time;
            return true;
        }
    }

    ROS_ERROR("Unknown error when invoking planning service call.");
    return false;
}

int R2Interface::executeTrajectory(const moveit_msgs::RobotTrajectory& trajectory, bool block)
{
    // Make sure execution service is online
    if (!ros::service::exists(move_group::EXECUTE_SERVICE_NAME, false))
    {
        ROS_ERROR("Trajectory execution service (%s) is not advertised", move_group::EXECUTE_SERVICE_NAME.c_str());
        return false;
    }

    ros::ServiceClient traj_executor = nh_.serviceClient<moveit_msgs::ExecuteKnownTrajectory>(move_group::EXECUTE_SERVICE_NAME);
    moveit_msgs::ExecuteKnownTrajectory::Request req;
    moveit_msgs::ExecuteKnownTrajectory::Response resp;

    req.trajectory = trajectory;
    req.wait_for_execution = block;

    // Possible error codes in response:
    // SUCCESS
    // PREEMPTED
    // TIMED_OUT
    // CONTROL_FAILED

    if (!traj_executor.call(req, resp))
        ROS_WARN("Execution service call failed");

    return resp.error_code.val;
}

void R2Interface::setDefaultJointPositions()
{
    // Default world pose
    issLegsWorldTransform(worldFrame_);

    // Reading in default joint positions
    std::map<std::string, double> jointVals;
    defaultPose(jointVals);
    readyArmPose(jointVals);
    //issLegsPose(jointVals);
    issLegsUnstowPose(jointVals);

    // Set the pose
    setJointPositions(jointVals);
}

void R2Interface::setJointPositions(const std::map<std::string, double>& jointVals)
{
    sensor_msgs::JointState joint_state_msg;
    joint_state_msg.header.stamp = ros::Time::now();
    for (std::map<std::string, double>::const_iterator it = jointVals.begin(); it != jointVals.end(); ++it)
    {
        joint_state_msg.name.push_back(it->first);
        joint_state_msg.position.push_back(it->second);
    }

    // Update TF with the new joint values
    jointPublisher_.publish(joint_state_msg);
}

void R2Interface::setJointPositions(const robot_state::RobotState& state, bool updateWorldFrame)
{
    // Create joint_state message with new joint states
    sensor_msgs::JointState joint_state_msg;
    joint_state_msg.name = state.getVariableNames();
    joint_state_msg.position.resize(joint_state_msg.name.size());
    memcpy(&joint_state_msg.position[0], state.getVariablePositions(), joint_state_msg.position.size()*sizeof(double));

    // Update TF with the new joint values
    jointPublisher_.publish(joint_state_msg);

    // Update the world transform
    if (updateWorldFrame)
        worldFrame_ = state.getGlobalLinkTransform("world");
}

const Eigen::Affine3d& R2Interface::getWorldFrame() const
{
    return worldFrame_;
}

void R2Interface::setWorldTransform(const Eigen::Affine3d& transform)
{
    worldFrame_ = transform;
}

void R2Interface::fakeLocalization(bool enable, unsigned int rate)
{
    if (fakeLocalize_ && !enable) // localization is running... turn it off
    {
        ROS_INFO("Broadcasting the virtual joint transform");
        fakeLocalize_ = enable;
        localizationThread_.join();
    }
    else if (!fakeLocalize_ && enable) // localization is not running... turn it on
    {

        fakeLocalize_ = enable;
        localizationThread_ = boost::thread(boost::bind(&R2Interface::publishVirtualJointFrame, this, rate));
    }
}
void R2Interface::gazeboLocalization(bool enable)
{
    //worldPoseSubscriber_ = nh_.subscribe<gazebo_msgs::LinkStates>("/gazebo/link_states", 20, boost::bind(&R2Interface::worldPoseUpdate, this, _1));
    worldPoseSubscriber_ = nh_.subscribe<nav_msgs::Odometry>("/r2/world_ref/pose_state", 20, boost::bind(&R2Interface::worldPoseUpdate, this, _1));
    gazebo_ = true;
    fakeLocalization(enable);
}

// void R2Interface::worldPoseUpdate(const gazebo_msgs::LinkStates::ConstPtr& msg)
// {
//     const std::string linkName = "r2::r2/world_ref";
//     for(size_t i = 0; i < msg->name.size(); ++i)
//     {
//         if (msg->name[i] == linkName)
//         {
//             tf::poseMsgToEigen(msg->pose[i], worldFrame_);
//             break;
//         }
//     }
// }

void R2Interface::worldPoseUpdate(const nav_msgs::Odometry::ConstPtr& msg)
{
    // const std::string linkName = "r2::r2/world_ref";
    // for(size_t i = 0; i < msg->name.size(); ++i)
    // {
    //     if (msg->name[i] == linkName)
    //     {
    //         tf::poseMsgToEigen(msg->pose[i], worldFrame_);
    //         break;
    //     }
    // }

    tf::poseMsgToEigen(msg->pose.pose, worldFrame_);
    //std::cout << "World pose: " << msg->pose.pose.position << std::endl;
}


void R2Interface::publishVirtualJointFrame(unsigned int rate)
{
    static tf::TransformBroadcaster tfBroadcaster;
    ROS_INFO("%s: Started broadcasting the virtual joint transform", "R2Interface");

    ros::Rate timeSlice(rate);

    while (ros::ok() && fakeLocalize_)
    {
        tf::Transform transform;
        tf::poseEigenToTF(worldFrame_, transform);

        if (gazebo_)
            tfBroadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "virtual_world", "reference_frame"));
        else
            tfBroadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "virtual_world", "world"));

        timeSlice.sleep();
    }

    ROS_INFO("%s: Stopped broadcasting the virtual joint transform", "R2Interface");
}

void R2Interface::simulateTrajectory(const std::vector<moveit_msgs::RobotTrajectory>& trajectories, bool updateWorldFrame)
{
    for(size_t i = 0; i < trajectories.size(); ++i)
        simulateTrajectory(trajectories[i], updateWorldFrame);
}

void R2Interface::simulateTrajectory(const moveit_msgs::RobotTrajectory& trajectory, bool updateWorldFrame)
{
    unsigned int rate = 5; // Rate at which waypoints are published, in Hz
    unsigned int delay = (1.0/rate)*1000000; // microseconds
    robot_state::RobotState state(getPlanningScene()->getCurrentState());

    std::size_t state_count = std::max(trajectory.joint_trajectory.points.size(),
                                       updateWorldFrame ? trajectory.multi_dof_joint_trajectory.points.size() : 0);

    for (std::size_t i = 0 ; i < state_count ; ++i)
    {
        if (trajectory.joint_trajectory.points.size() > i)
            state.setVariablePositions(trajectory.joint_trajectory.joint_names, trajectory.joint_trajectory.points[i].positions);
        if (updateWorldFrame && trajectory.multi_dof_joint_trajectory.points.size() > i)
        {
            for (std::size_t j = 0 ; j < trajectory.multi_dof_joint_trajectory.joint_names.size() ; ++j)
            {
                Eigen::Affine3d t;
                tf::transformMsgToEigen(trajectory.multi_dof_joint_trajectory.points[i].transforms[j], t);
                state.setJointPositions(trajectory.multi_dof_joint_trajectory.joint_names[j], t);
                worldFrame_ = t;
            }
        }

        setJointPositions(state, updateWorldFrame);
        usleep(delay);
    }
}