/* Author: Ryan Luna */

#ifndef MOVEIT_R2_TREE_KINEMATICS_PLUGIN_
#define MOVEIT_R2_TREE_KINEMATICS_PLUGIN_

// ROS
#include <ros/ros.h>

// MoveIt!
#include <moveit/kinematics_base/kinematics_base.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

// NASA R2
#include <nasa_robodyn_controllers_core/MobileTreeIk.h> // note: old kinematics
//#include "moveit_r2_kinematics/R2TreeIk.h"  // fake wrapper around KdlTreeIk for mobile tree.  Works ok-ish.  Using this sacrifices quality for speed
#include <nasa_robodyn_controllers_core/KdlTreeFk.h>

//#include <boost/thread/mutex.hpp>
#include "moveit_r2_kinematics/r2_tree_kinematics_interface.h"

namespace moveit_r2_kinematics
{
    /// \brief Custom kinematics routines for R2.  NOTE: Currently this implementation is only be functional on the legs.  See implementation of initialize(...).
    class MoveItR2TreeKinematicsPlugin : public kinematics::KinematicsBase
    {
    public:
        MoveItR2TreeKinematicsPlugin();
        virtual ~MoveItR2TreeKinematicsPlugin ();

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

        /// @brief Given a set of desired poses for a planning group with multiple end-effectors, search for the joint angles
        ///        required to reach them. This is useful for e.g. biped robots that need to perform whole-body IK.
        ///        Not necessary for most robots that have kinematic chains.
        ///        This particular method is intended for "searching" for a solutions by stepping through the redundancy
        ///        (or other numerical routines).
        /// @param ik_poses the desired pose of each tip link
        /// @param ik_seed_state an initial guess solution for the inverse kinematics
        /// @param timeout The amount of time (in seconds) available to the solver
        /// @param consistency_limits the distance that any joint in the solution can be from the corresponding joints in the current seed state
        /// @param solution the solution vector
        /// @param solution_callback A callback solution for the IK solution
        /// @param error_code an error code that encodes the reason for failure or success
        /// @param options container for other IK options
        /// @param context_state (optional) the context in which this request
        ///        is being made.  The position values corresponding to
        ///        joints in the current group may not match those in
        ///        ik_seed_state.  The values in ik_seed_state are the ones
        ///        to use.  This is passed just to provide the \em other
        ///        joint values, in case they are needed for context, like
        ///        with an IK solver that computes a balanced result for a
        ///        biped.
        /// @return True if a valid solution was found, false otherwise
        virtual bool searchPositionIK(const std::vector<geometry_msgs::Pose> &ik_poses,
                                const std::vector<double> &ik_seed_state,
                                double timeout,
                                const std::vector<double> &consistency_limits,
                                std::vector<double> &solution,
                                const IKCallbackFn &solution_callback,
                                moveit_msgs::MoveItErrorCodes &error_code,
                                const kinematics::KinematicsQueryOptions &options = kinematics::KinematicsQueryOptions(),
                                const moveit::core::RobotState* context_state = NULL) const;

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

        /// @brief Initialization function for the kinematics, for use with non-chain IK solvers
        /// @param robot_description This parameter can be used as an identifier for the robot kinematics is computed for;
        /// For example, rhe name of the ROS parameter that contains the robot description;
        /// @param group_name The group for which this solver is being configured
        /// @param base_frame The base frame in which all input poses are expected.
        /// This may (or may not) be the root frame of the chain that the solver operates on
        /// @param tip_frames A vector of tips of the kinematic tree
        /// @param search_discretization The discretization of the search when the solver steps through the redundancy
        /// @return True if initialization was successful, false otherwise
        virtual bool initialize(const std::string& robot_description,
                                const std::string& group_name,
                                const std::string& base_frame,
                                const std::vector<std::string>& tip_frames,
                                double search_discretization);

        /// @brief Return all the joint names in the order they are used internally
        virtual const std::vector<std::string>& getJointNames() const;

        /// @brief Return all the link names in the order they are represented internally
        virtual const std::vector<std::string>& getLinkNames() const;

        /**
        * \brief Check if this solver supports a given JointModelGroup.
        *
        * Override this function to check if your kinematics solver
        * implementation supports the given group.
        *
        * The default implementation just returns jmg->isChain(), since
        * solvers written before this function was added all supported only
        * chain groups.
        *
        * \param jmg the planning group being proposed to be solved by this IK solver
        * \param error_text_out If this pointer is non-null and the group is
        *          not supported, this is filled with a description of why it's not
        *          supported.
        * \return True if the group is supported, false if not.
        */
        virtual const bool supportsGroup(const moveit::core::JointModelGroup *jmg,
                                         std::string* error_text_out = NULL) const;

        /// @brief Redundant joints are disallowed in this kinematics solver
        virtual bool setRedundantJoints(const std::vector<unsigned int> &redundant_joint_indices) { return false; }

        /// @}

        R2TreeKinematicsInterface* getTreeKinematicsInterface()
        {
            return interface_;
        }

        const R2TreeKinematicsInterface* getTreeKinematicsInterface() const
        {
            return interface_;
        }

    protected:
        R2TreeKinematicsInterface* interface_;
  };
}

#endif
