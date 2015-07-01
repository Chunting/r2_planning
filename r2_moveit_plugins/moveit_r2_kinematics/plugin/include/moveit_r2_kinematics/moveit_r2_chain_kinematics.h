/* Author: Ryan Luna */

#ifndef MOVEIT_R2_CHAIN_KINEMATICS_PLUGIN_
#define MOVEIT_R2_CHAIN_KINEMATICS_PLUGIN_

// ROS
#include <ros/ros.h>

// MoveIt!
#include <moveit/kinematics_base/kinematics_base.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

// NASA R2
#include <nasa_robodyn_controllers_core/KdlChainIk.h>
#include <nasa_robodyn_controllers_core/KdlTreeFk.h>

#include <boost/thread/mutex.hpp>

namespace moveit_r2_kinematics
{
  /// \brief Custom structure for IK requests on a kinematic chain
    class MoveItR2ChainKinematicsPlugin : public kinematics::KinematicsBase
    {
    public:
        MoveItR2ChainKinematicsPlugin();
        virtual ~MoveItR2ChainKinematicsPlugin ();

        /// @group KinematicsBase interface
        /// @{

        /// @brief Given a desired pose of the end-effector, compute the joint angles to reach it
        /// @param ik_pose the desired pose of the link
        /// @param ik_seed_state an initial guess solution for the inverse kinematics
        /// @param solution the solution vector
        /// @param error_code an error code that encodes the reason for failure or success
        /// @param lock_redundant_joints if setRedundantJoints() was previously called, keep the values of the joints marked as redundant the same as in the seed
        /// @return True if a valid solution was found, false otherwise
        virtual bool getPositionIK(const geometry_msgs::Pose &ik_pose,
                                   const std::vector<double> &ik_seed_state,
                                   std::vector<double> &solution,
                                   moveit_msgs::MoveItErrorCodes &error_code,
                                   const kinematics::KinematicsQueryOptions &options = kinematics::KinematicsQueryOptions()) const;

        /// @brief Given a desired pose of the end-effector, search for the joint angles required to reach it.
        /// This particular method is intended for "searching" for a solutions by stepping through the redundancy
        /// (or other numerical routines).
        /// @param ik_pose the desired pose of the link
        /// @param ik_seed_state an initial guess solution for the inverse kinematics
        /// @param timeout The amount of time (in seconds) available to the solver
        /// @param solution the solution vector
        /// @param error_code an error code that encodes the reason for failure or success
        /// @param lock_redundant_joints if setRedundantJoints() was previously called, keep the values of the joints marked as redundant the same as in the seed
        /// @return True if a valid solution was found, false otherwise
        virtual bool searchPositionIK(const geometry_msgs::Pose &ik_pose,
                                      const std::vector<double> &ik_seed_state,
                                      double timeout,
                                      std::vector<double> &solution,
                                      moveit_msgs::MoveItErrorCodes &error_code,
                                      const kinematics::KinematicsQueryOptions &options = kinematics::KinematicsQueryOptions()) const;

        /// @brief Given a desired pose of the end-effector, search for the joint angles required to reach it.
        /// This particular method is intended for "searching" for a solutions by stepping through the redundancy
        /// (or other numerical routines).
        /// @param ik_pose the desired pose of the link
        /// @param ik_seed_state an initial guess solution for the inverse kinematics
        /// @param timeout The amount of time (in seconds) available to the solver
        /// @param consistency_limits the distance that any joint in the solution can be from the corresponding joints in the current seed state
        /// @param solution the solution vector
        /// @param error_code an error code that encodes the reason for failure or success
        /// @param lock_redundant_joints if setRedundantJoints() was previously called, keep the values of the joints marked as redundant the same as in the seed
        /// @return True if a valid solution was found, false otherwise
        virtual bool searchPositionIK(const geometry_msgs::Pose &ik_pose,
                                      const std::vector<double> &ik_seed_state,
                                      double timeout,
                                      const std::vector<double> &consistency_limits,
                                      std::vector<double> &solution,
                                      moveit_msgs::MoveItErrorCodes &error_code,
                                      const kinematics::KinematicsQueryOptions &options = kinematics::KinematicsQueryOptions()) const;

        /// @brief Given a desired pose of the end-effector, search for the joint angles required to reach it.
        /// This particular method is intended for "searching" for a solutions by stepping through the redundancy
        /// (or other numerical routines).
        /// @param ik_pose the desired pose of the link
        /// @param ik_seed_state an initial guess solution for the inverse kinematics
        /// @param timeout The amount of time (in seconds) available to the solver
        /// @param solution the solution vector
        /// @param desired_pose_callback A callback function for the desired link pose - could be used, e.g. to check for collisions for the end-effector
        /// @param solution_callback A callback solution for the IK solution
        /// @param error_code an error code that encodes the reason for failure or success
        /// @param lock_redundant_joints if setRedundantJoints() was previously called, keep the values of the joints marked as redundant the same as in the seed
        /// @return True if a valid solution was found, false otherwise
        virtual bool searchPositionIK(const geometry_msgs::Pose &ik_pose,
                                      const std::vector<double> &ik_seed_state,
                                      double timeout,
                                      std::vector<double> &solution,
                                      const IKCallbackFn &solution_callback,
                                      moveit_msgs::MoveItErrorCodes &error_code,
                                      const kinematics::KinematicsQueryOptions &options = kinematics::KinematicsQueryOptions()) const;

        /// @brief Given a desired pose of the end-effector, search for the joint angles required to reach it.
        /// This particular method is intended for "searching" for a solutions by stepping through the redundancy
        /// (or other numerical routines).
        /// @param ik_pose the desired pose of the link
        /// @param ik_seed_state an initial guess solution for the inverse kinematics
        /// @param timeout The amount of time (in seconds) available to the solver
        /// @param consistency_limits the distance that any joint in the solution can be from the corresponding joints in the current seed state
        /// @param solution the solution vector
        /// @param desired_pose_callback A callback function for the desired link pose - could be used, e.g. to check for collisions for the end-effector
        /// @param solution_callback A callback solution for the IK solution
        /// @param error_code an error code that encodes the reason for failure or success
        /// @param lock_redundant_joints if setRedundantJoints() was previously called, keep the values of the joints marked as redundant the same as in the seed
        /// @return True if a valid solution was found, false otherwise
        virtual bool searchPositionIK(const geometry_msgs::Pose &ik_pose,
                                      const std::vector<double> &ik_seed_state,
                                      double timeout,
                                      const std::vector<double> &consistency_limits,
                                      std::vector<double> &solution,
                                      const IKCallbackFn &solution_callback,
                                      moveit_msgs::MoveItErrorCodes &error_code,
                                      const kinematics::KinematicsQueryOptions &options) const;

        /// @brief Given a set of joint angles and a set of links, compute their pose
        /// @param link_names A set of links for which FK needs to be computed
        /// @param joint_angles The state for which FK is being computed
        /// @param poses The resultant set of poses (in the frame returned by getBaseFrame())
        /// @return True if a valid solution was found, false otherwise
        virtual bool getPositionFK(const std::vector<std::string> &link_names,
                                   const std::vector<double> &joint_angles,
                                   std::vector<geometry_msgs::Pose> &poses) const;

        /// @brief  Initialization function for the kinematics
        /// @param robot_description This parameter can be used as an identifier for the robot kinematics is computed for; For example, rhe name of the ROS parameter that contains the robot description;
        /// @param group_name The group for which this solver is being configured
        /// @param base_frame The base frame in which all input poses are expected.
        /// This may (or may not) be the root frame of the chain that the solver operates on
        /// @param tip_frame The tip of the chain
        /// @param search_discretization The discretization of the search when the solver steps through the redundancy
        /// @return True if initialization was successful, false otherwise
        virtual bool initialize(const std::string& robot_description,
                                const std::string& group_name,
                                const std::string& base_frame,
                                const std::string& tip_frame,
                                double search_discretization);

        /// @brief Return all the joint names in the order they are used internally
        virtual const std::vector<std::string>& getJointNames() const;

        /// @brief Return all the link names in the order they are represented internally
        virtual const std::vector<std::string>& getLinkNames() const;

        /// @brief Redundant joints are disallowed in this kinematics solver
        virtual bool setRedundantJoints(const std::vector<unsigned int> &redundant_joint_indices) { return false; }

        /// @}

        // Return the order of ALL actuable joints in the entire system, not just the group
        virtual const std::vector<std::string>& getAllJointNames() const;


    protected:
        /// \brief A model of the entire robot
        robot_model::RobotModelPtr robot_model_;

        /// \brief The IK solver
        KdlChainIk* ik_;
        /// \brief The FK solver
        KdlTreeFk* fk_;

        /// \brief The default joint position for ALL joints in R2
        KDL::JntArray default_joint_positions_;

        /// \brief The number of joint positions in this kinematic chain
        unsigned int num_dofs_;

        /// \brief A mapping of joints in the group to their index in jointNames_, default_joint_positions_, etc..
        std::map<std::string, unsigned int> group_joint_index_map_;

        /// \brief The set of joints to perform IK for.
        std::vector<std::string> group_joints_;
        /// \brief The set of links to perform IK for.
        std::vector<std::string> group_links_;
        /// \brief A list of every joint in the system
        std::vector<std::string> joint_names_;
  };
}

#endif
