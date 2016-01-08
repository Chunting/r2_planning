// R2Interface provides common planning functionalities for R2
// Author: Ryan Luna
// May 2015

#ifndef R2_INTERFACE_H_
#define R2_INTERFACE_H_

#include <boost/thread.hpp>
#include <cassert>
#include <Eigen/Dense>

#include <ros/ros.h>
#include <moveit_r2_kinematics/r2_tree_kinematics_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/WorkspaceParameters.h>
#include <moveit_msgs/MotionPlanRequest.h>
#include <geometry_msgs/PoseStamped.h>
//#include <gazebo_msgs/LinkStates.h>
#include <nav_msgs/Odometry.h>

class R2Interface
{
public:
    R2Interface(const std::string& group = "legs", const std::string& robot_description = "robot_description");
    ~R2Interface();

    // Enable/disable fake robot localization.  This will turn on/off a TF broadcaster
    // that broadcasts the current frame of the virtual joint at the given rate in Hz.
    void fakeLocalization(bool enable, unsigned int rate = 50);

    // Enable/disable localization from Gazebo.  This will turn on/off a subscriber that
    // listens to world pose information from the Gazebo simulator.  Note, this will
    // also enable/disable fakeLocalization.
    void gazeboLocalization(bool enable);

    ///// WAREHOUSE /////
    // Store the current planning scene to the warehouse, under the given name
    bool storeCurrentPlanningScene(const std::string& name) const;
    // Store the planning scene message to the warehouse.  scene.name will be used as the name of the scene
    bool storePlanningScene(const moveit_msgs::PlanningScene& scene) const;
    // Load the planning scene with the given name from the warehouse.  Store the scene in scene
    bool loadPlanningScene(const std::string& name, moveit_msgs::PlanningScene& scene) const;

    // Store the current robot state to the warehouse, under the given name
    bool storeCurrentRobotState(const std::string& name) const;
    // Store the robot state to the warehouse, under the given name
    bool storeRobotState(const std::string& name, const moveit_msgs::RobotState& state) const;
    // Load the robot state with the given name from the warehouse.  Store the state in state
    bool loadRobotState(const std::string& name, moveit_msgs::RobotState& state) const;

    // Store the given motion planning request in the warehouse.  The request will be given requestName and
    // associated with a stored planning scene under the name planningSceneName
    bool storeMotionPlanRequest(const moveit_msgs::MotionPlanRequest& request, const std::string& requestName, const std::string& planningSceneName) const;
    // Load the motion planning request requestName associated with scene planningSceneName from the warehouse.
    // Store the motion plan request in request
    bool loadMotionPlanRequest(moveit_msgs::MotionPlanRequest& request, const std::string& requestName, const std::string& planningSceneName) const;

    // Store the constraint sets in the warehouse, under the given name
    bool storeConstraints(const std::string& name, const moveit_msgs::Constraints& constraints) const;
    // Load the named constraints from the warehouse.  Store the constraints in constraints
    bool loadConstraints(const std::string& name, moveit_msgs::Constraints& constraints) const;

    ///// VISUALIZATION (RViz) /////

    // Clear any trajectory being displayed in RViz
    void clearTrajectory() const;
    // View the given trajectory in RViz.  Assumes trajectory starts at current robot state
    void viewTrajectory(const moveit_msgs::RobotTrajectory& trajectory);
    // View the set of trajectories in RViz.  Assumes trajectory starts at current robot state
    void viewTrajectory(const std::vector<moveit_msgs::RobotTrajectory>& trajectories);
    // View the given trajectory in RViz, starting at initial_state
    void viewTrajectory(const robot_state::RobotState& initial_state, const moveit_msgs::RobotTrajectory& trajectory);
    // View the set of trajectories in RViz, starting at initial_state
    void viewTrajectory(const robot_state::RobotState& initial_state, const std::vector<moveit_msgs::RobotTrajectory>& trajectories);


    ///// ROBOT STATE /////
    // Return the current world frame of the robot.
    const Eigen::Affine3d& getWorldFrame() const;
    // Manually set the virtual joint transform (be careful with this)
    void setWorldTransform(const Eigen::Affine3d& transform);

    // Set joint positions to default values
    void setDefaultJointPositions();
    // Set the joint positions to the values given in the jointVals map
    void setJointPositions(const std::map<std::string, double>& jointVals);
    void setJointPositions(const robot_state::RobotState& state, bool updateWorldFrame=true);

    // Get the current robot state from the planning state
    const robot_state::RobotState& getCurrentRobotState(void) const;
    // Get a shiny new robot state
    robot_state::RobotStatePtr allocRobotState(void) const;

    ///// PLANNING SCENE /////
    // Return a read-only copy of the current planning scene
    planning_scene_monitor::LockedPlanningSceneRO getPlanningScene() const;

    // Set the planning scene from the given message
    void setPlanningScene(moveit_msgs::PlanningScene& scene);

    // Add an obstacle to the scene
    void addObstacleToPlanningScene(const moveit_msgs::CollisionObject& obstacle);

    // Enable/disable collisions between link1 and link2.
    void enableCollisionChecking(const std::string& link1, const std::string& link2, bool allow, bool waitForAck=false);
    // Enable/disable collisions between all pairs of bodies
    void enableCollisionChecking(const std::vector<std::string> bodies, bool allow, bool waitForAck=false);
    // Enable/disable collisions between all pairs of bodies in the two sets
    void enableCollisionChecking(const std::vector<std::string> set1, const std::vector<std::string> set2,
                                 bool allow, bool waitForAck=false);

    // Attach/detach an object from the robot
    void attachObjectToRobot(const moveit_msgs::AttachedCollisionObject& attached_obj);

    ///// KINEMATICS /////
    // Compute an inverse kinematics solution for the given link and pose in the (kinematic chain) group.
    // If a solution is found, store the joint values in result.
    bool chainIK(const std::string& group, const std::string& link, const geometry_msgs::PoseStamped& pose,
                 robot_state::RobotStatePtr result);
    // Compute an inverse kinematics solution for the given link and pose in the (kinematic chain) group.
    // Initialize the IK solver with the given seed values. If a solution is found, store the
    // joint values in result.
    bool chainIK(const std::string& group, const std::string& link, const geometry_msgs::PoseStamped& pose,
                 const robot_state::RobotState& seed_state,
                 robot_state::RobotStatePtr result);

    // Hold fixed_link constant and move link to the given pose.  Solution is stored in result
    // The seed state is the current state.  The pose is considered mission critical.
    bool treeIK(const std::string& group, const std::string& fixed_link, const std::string& link,
                const geometry_msgs::PoseStamped& pose, robot_state::RobotStatePtr result);

    // Hold fixed_link constant and move link to the given pose starting from the seed state.
    // IK solution is stored in result.  The pose is considered mission critical.
    bool treeIK(const std::string& group, const std::string& fixed_link, const std::string& link, const geometry_msgs::PoseStamped& pose,
                const robot_state::RobotState& seed_state, robot_state::RobotStatePtr result);

    // Hold fixed_link constant and move link to the given pose starting from the seed state.
    // IK solution is stored in result.  Tolerance on each element of the desired pose is given
    // in the posePriorities vector.
    bool treeIK(const std::string& group, const std::string& fixed_link, const std::string& link,
                const geometry_msgs::PoseStamped& pose, const std::vector<int>& posePriorities,
                const robot_state::RobotState& seed_state, robot_state::RobotStatePtr result);

    // Hold fixed_link constant and move the links to the given poses starting from the seed state.
    // IK solution is stored in result.  Tolerance on each element of the desired poses is given
    // in the posePriorities vector.
    bool treeIK(const std::string& group, const std::string& fixed_link, const std::vector<std::string>& links,
                const std::vector<geometry_msgs::PoseStamped>& poses, const std::vector<std::vector<int> >& posePriorities,
                const robot_state::RobotState& seed_state, robot_state::RobotStatePtr result);

    ///// MOTION PLANNING /////
    // Set the workspace of the robot.  Used for planar/floating joints to constrain sampling space.
    void setWorkspace(const moveit_msgs::WorkspaceParameters& workspace);
    // Retrieve the workspace of the robot
    const moveit_msgs::WorkspaceParameters& getWorkspace() const;

    // Create a motion plan request message for the given query input
    void createMotionPlanRequest(const moveit_msgs::RobotState& start_state, const moveit_msgs::Constraints& path_constraints,
                                 const std::vector<moveit_msgs::Constraints>& goal_constraints, const std::string& group_name, double max_time,
                                 const std::string& planner_name, unsigned int tries, moveit_msgs::MotionPlanRequest& request) const;

    // Compute a plan to move the goal_link to the goal_pose_offset (in the goal_link frame) from the given
    // start state
    bool plan(const moveit_msgs::RobotState& start_state, const geometry_msgs::PoseStamped& goal_pose,
              const std::string& goal_link, const std::string& group_name,
              double max_time, const std::string& planner_name, moveit_msgs::RobotTrajectory& trajectory_msg, unsigned int tries = 1);

    // Compute a plan to move the goal_link to the goal_pose_offset (in the goal_link frame) from the given
    // start state
    bool plan(const moveit_msgs::RobotState& start_state, const geometry_msgs::PoseStamped& goal_pose, const std::string& goal_link, const std::string& group_name,
              const moveit_msgs::Constraints& path_constraints, double max_time, const std::string& planner_name,
              moveit_msgs::RobotTrajectory& trajectory_msg, unsigned int tries = 1);

    // Compute a plan starting from start_state that satisfies goal_constraints at the end of the path and all of the path_constraints
    // for the entire duration of the motion.
    bool plan(const moveit_msgs::RobotState& start_state, const moveit_msgs::Constraints& path_constraints,
              const std::vector<moveit_msgs::Constraints>& goal_constraints, const std::string& group_name, double max_time,
              const std::string& planner_name, moveit_msgs::RobotTrajectory& trajectory_msg, unsigned int tries = 1);

    ///// TRAJECTORY EXECUTION /////
    // Execute the given trajectory.  If block is true, this call will block execution until execution has completed.
    // Note: Return type is a member of the moveit_msgs::MoveItErrorCodes enumeration
    int executeTrajectory(const moveit_msgs::RobotTrajectory& trajectory, bool block=false);

    // Simulate execution of a trajectory.  Useful for debugging in RViz
    void simulateTrajectory(const std::vector<moveit_msgs::RobotTrajectory>& trajectories, bool updateWorldFrame=true);
    void simulateTrajectory(const moveit_msgs::RobotTrajectory& trajectory, bool updateWorldFrame=true);


protected:
    void planningSceneUpdate(planning_scene_monitor::PlanningSceneMonitor::SceneUpdateType type);

private:
    void publishVirtualJointFrame(unsigned int rate);
    //void worldPoseUpdate(const gazebo_msgs::LinkStates::ConstPtr& msg);
    void worldPoseUpdate(const nav_msgs::Odometry::ConstPtr& msg);

    ros::NodeHandle nh_;

    planning_scene_monitor::PlanningSceneMonitorPtr psm_;

    // The group configured for inverse kinematics.
    std::string group_name_;

    // True if broadcasting worldFrame_ to tf
    bool fakeLocalize_;
    // True if localizing gazebo frame.  Gazebo inserts its own reference frame
    bool gazebo_;
    // The thread that broadcasts worldFrame_ to tf
    boost::thread localizationThread_;
    // The current coordinate frame for the virtual joint
    Eigen::Affine3d worldFrame_;

    ros::Publisher jointPublisher_;     // publisher for joint positions
    ros::Publisher obstaclePublisher_;  // publisher for obstacles
    ros::Publisher scenePublisher_;     // publisher for planning scene updates
    ros::Publisher attachPublisher_;    // publisher for attaching objects to the robot
    ros::Publisher vizPublisher_;       // publisher for viewing paths (RViz)

    ros::Subscriber worldPoseSubscriber_;

    moveit_msgs::WorkspaceParameters workspace_;
    std::string virtualJointName_;

    bool planningSceneDirty_;

    moveit_r2_kinematics::R2TreeKinematicsInterface* treeKinematics_;
};

#endif