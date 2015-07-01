/* Author: Ryan Luna */

#ifndef R2_TREE_KINEMATICS_INTERFACE_
#define R2_TREE_KINEMATICS_INTERFACE_

#include <vector>
#include <map>
#include <string>

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <boost/thread/mutex.hpp>
#include <Eigen/Dense>

// NASA R2
#include <nasa_robodyn_controllers_core/MobileTreeIk.h> // note: old kinematics
//#include "moveit_r2_kinematics/R2TreeIk.h"  // fake wrapper around KdlTreeIk for mobile tree.  Works ok-ish.  Using this sacrifices quality for speed
#include <nasa_robodyn_controllers_core/KdlTreeFk.h>

// MoveIt
//#include <moveit/rdf_loader/rdf_loader.h>
#include <moveit/robot_model/robot_model.h>

namespace moveit_r2_kinematics
{
    /// \brief Custom structure for IK requests on a kinematic tree.
    class TreeIkRequest
    {
    public:
        /// \brief Set the given link as fixed in the IK request.  This link will serve as a base for the IK solver.
        void addFixedLink(const std::string& link_name);
        /// \brief Add an IK task to move link_name to the given pose (in global coordinates) with the given priority
        void addLinkPose(const std::string& link_name, const geometry_msgs::Pose& pose, const int priority=KdlTreeIk::CRITICAL);
        /// \brief Add an IK task to move link_name to the given pose (in global coordinates) with the given priority
        void addLinkPose(const std::string& link_name, const geometry_msgs::Pose& pose, const std::vector<int>& priority);
        /// \brief Set the pose of the robot in the world.  This pose corresponds to the "mobile joints" of the IK solver.
        void setWorldState(const Eigen::Affine3d& pose);
        /// \brief Set the current (seed) values for the robot.
        void setJointValues(const std::vector<double>& values);
        /// \brief Clear this request
        void clear() { fixed_links_.clear(); moving_links_.clear(); poses_.clear(); priorities_.clear(); initial_joints_.clear(); world_state_rpy_.clear(); }

        const std::vector<std::string>& getFixedLinks() const;
        const std::vector<std::string>& getMovingLinks() const;
        const std::vector<geometry_msgs::Pose>& getMovingLinkPoses() const;
        const std::vector<double>& getJointValues() const;
        const geometry_msgs::Pose& getWorldState() const;
        const std::vector<double>& getWorldStateRPY() const;
        const std::vector<KdlTreeIk::NodePriority>& getPriorities() const;

    protected:
        std::vector<std::string> fixed_links_;
        std::vector<std::string> moving_links_;
        std::vector<geometry_msgs::Pose> poses_;
        std::vector<KdlTreeIk::NodePriority> priorities_;

        std::vector<double> initial_joints_;
        std::vector<double> world_state_rpy_;
        geometry_msgs::Pose world_state_;
    };

    /// \brief Custom structure for an IK response
    class TreeIkResponse
    {
    public:
        /// \brief True if the IK request was successful.
        bool successful() const;
        /// \brief Return the new pose of the robot in the world.  This pose corresponds to the "mobile joints" of the IK solver.
        const Eigen::Affine3d& getWorldState() const;
        /// \brief Return the new joint values for the robot
        const std::vector<double>& getJointValues() const;

        /// \brief Mark the IK request as a failure
        void setFailure();
        /// \brief Set the world state and joint values after a successful IK request
        void setValues(const Eigen::Affine3d& world, const std::vector<double>& joints);

    protected:
        bool success_;
        Eigen::Affine3d world_pose_;
        std::vector<double> joint_values_;
    };

    // Interface for FK and IK on a kinematic tree in MoveIt
    class R2TreeKinematicsInterface
    {
    public:
        R2TreeKinematicsInterface();
        ~R2TreeKinematicsInterface();

        // Initialize the kinematics interface
        bool initialize(const std::string& robot_description,
                        const std::string& group_name,
                        const std::string& base_frame,
                        const std::vector<std::string>& tip_frames);

        // Tree FK request.  Fills out the (empty) frames for each link specified in frames, given joint_angles for the group
        // TODO: Make API not depend on KDL?
        bool getPositionFK(const std::vector<double> &joint_angles,
                           std::map<std::string, KDL::Frame>& frames) const;

        // Tree IK request.  Returns true on successful IK response.
        bool getPositionIk(const TreeIkRequest& request, TreeIkResponse& response) const;

        // Return the order of ALL actuable joints in the entire system, not just the group
        const std::vector<std::string>& getAllJointNames() const;

        // Return true if the root link of the group is mobile.
        bool hasMobileBase() const;

        // Return the number of DoFs for the mobile base (zero if base is not mobile)
        unsigned int mobileBaseVariableCount() const;

        // Return the number of DoFs for the configured group.  Include those from mobile base.
        unsigned int groupVariableCount() const;

        // Return the robot model
        robot_model::RobotModelPtr getRobotModel() const;

        // Return the bijection between group variable indices and their counterparts in the full robot state
        // This bijeciton depends on group_name
        const std::vector<int>& getGroupToRobotStateBijection() const;

        // Return the bijection between ik joint variable indices and their counterparts in the full robot state
        // This bijection depends on tip_frames
        const std::vector<int>& getIKJointsToRobotStateBijection() const;

        // Return the total number of variables in the robot (the whole robot, not just the group)
        unsigned int allVariableCount() const;

        // Return the default position for ALL joints (the whole robot, not just the group)
        const KDL::JntArray& getAllDefaultJointPositions() const;

        //// KinematicsBase stuff

        // Return the list of joints configured in this group
        const std::vector<std::string>& getJointNames() const
        {
            return group_joints_;
        }

        // Return the list of link configured in this group
        const std::vector<std::string>& getLinkNames() const
        {
            return group_links_;
        }

    protected:
        /// \brief A model of the entire robot
        robot_model::RobotModelPtr robot_model_;

        /// \brief True if the robot is mobile (the robot is not fixed in place)
        bool mobile_base_;

        /// \brief The IK solver
        MobileTreeIk* ik_;
        //R2TreeIk* ik_;
        /// \brief The FK solver
        KdlTreeFk* fk_;

        /// \brief Mutex locking access to the ik_ member
        mutable boost::mutex ik_mutex_;
        /// \brief Mutex locking access to the fk_ member
        mutable boost::mutex fk_mutex_;

        /// \brief The default joint position for ALL joints in R2
        KDL::JntArray default_joint_positions_;

        /// \brief The number of actuable DOFs in R2
        unsigned int total_dofs_;
        /// \brief The number of passive DOFs in the mobile base joint
        unsigned int mobile_base_variable_count_;
        /// \brief The number of variables in the IK joint group (actuable and passive)
        unsigned int group_variable_count_;

        /// \brief A mapping of joints in the group to their index in jointNames_, default_joint_positions_, etc..
        std::map<std::string, unsigned int> group_joint_index_map_;

        /// \brief The set of joints to perform IK for.
        std::vector<std::string> group_joints_;
        /// \brief The set of links to perform IK for.
        std::vector<std::string> group_links_;
        /// \brief A list of every joint in the system
        std::vector<std::string> joint_names_;

        /// \brief A mapping of group joint variables to their indices in the full robot state
        std::vector<int> group_joint_to_robot_state_bijection_;
        /// \brief A mapping of joint variables in the IK representation to their indices in the full robot state
        std::vector<int> ik_joints_to_robot_joints_bijection_;

    };
}

#endif