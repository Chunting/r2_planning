/* Author: Ryan Luna */

#include "moveit_r2_kinematics/moveit_r2_tree_kinematics.h"

#include <eigen_conversions/eigen_msg.h>
#include <class_loader/class_loader.h>
#include <kdl_conversions/kdl_msg.h>
#include <tf/transform_datatypes.h>

//register MoveItR2TreeKinematicsPlugin as a KinematicsBase implementation
CLASS_LOADER_REGISTER_CLASS(moveit_r2_kinematics::MoveItR2TreeKinematicsPlugin, kinematics::KinematicsBase)

namespace moveit_r2_kinematics
{

MoveItR2TreeKinematicsPlugin::MoveItR2TreeKinematicsPlugin () : interface_(NULL)
{
}

MoveItR2TreeKinematicsPlugin::~MoveItR2TreeKinematicsPlugin ()
{
    if (interface_)
        delete interface_;
}

// Total hack for RViz
bool MoveItR2TreeKinematicsPlugin::getPositionIK(const geometry_msgs::Pose &ik_pose,
                                                 const std::vector<double> &ik_seed_state,
                                                 std::vector<double> &solution,
                                                 moveit_msgs::MoveItErrorCodes &error_code,
                                                 const kinematics::KinematicsQueryOptions &options) const
{
    ROS_ERROR("GetPositionIK for one pose is NOT implemented in this solver");
    return false;
}

bool MoveItR2TreeKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose &ik_pose,
                                                    const std::vector<double> &ik_seed_state,
                                                    double timeout,
                                                    std::vector<double> &solution,
                                                    moveit_msgs::MoveItErrorCodes &error_code,
                                                    const kinematics::KinematicsQueryOptions &options) const
{
    return getPositionIK(ik_pose, ik_seed_state, solution, error_code, options);
}

bool MoveItR2TreeKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose &ik_pose,
                                                    const std::vector<double> &ik_seed_state,
                                                    double timeout,
                                                    const std::vector<double> &consistency_limits,
                                                    std::vector<double> &solution,
                                                    moveit_msgs::MoveItErrorCodes &error_code,
                                                    const kinematics::KinematicsQueryOptions &options) const
{
    return getPositionIK(ik_pose, ik_seed_state, solution, error_code, options);
}

bool MoveItR2TreeKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose &ik_pose,
                                                    const std::vector<double> &ik_seed_state,
                                                    double timeout,
                                                    std::vector<double> &solution,
                                                    const IKCallbackFn &solution_callback,
                                                    moveit_msgs::MoveItErrorCodes &error_code,
                                                    const kinematics::KinematicsQueryOptions &options) const
{
    return getPositionIK(ik_pose, ik_seed_state, solution, error_code, options);
}

bool MoveItR2TreeKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose &ik_pose,
                                                    const std::vector<double> &ik_seed_state,
                                                    double timeout,
                                                    const std::vector<double> &consistency_limits,
                                                    std::vector<double> &solution,
                                                    const IKCallbackFn &solution_callback,
                                                    moveit_msgs::MoveItErrorCodes &error_code,
                                                    const kinematics::KinematicsQueryOptions &options) const
{
    return getPositionIK(ik_pose, ik_seed_state, solution, error_code, options);
}

#define F_EQ(a, b) fabs(a - b) < 1e-4

static bool equalPoses(const geometry_msgs::Pose& pose1, const geometry_msgs::Pose& pose2)
{
    // position
    if (!F_EQ(pose1.position.x, pose2.position.x) ||
        !F_EQ(pose1.position.y, pose2.position.y) ||
        !F_EQ(pose1.position.z, pose2.position.z))
        return false;

    // orientation
    if (!F_EQ(pose1.orientation.x, pose2.orientation.x) ||
        !F_EQ(pose1.orientation.y, pose2.orientation.y) ||
        !F_EQ(pose1.orientation.z, pose2.orientation.z) ||
        !F_EQ(pose1.orientation.w, pose2.orientation.w))
        return false;

    return true;
}

bool MoveItR2TreeKinematicsPlugin::searchPositionIK(const std::vector<geometry_msgs::Pose> &ik_poses,
                                                    const std::vector<double> &ik_seed_state,
                                                    double timeout,
                                                    const std::vector<double> &consistency_limits,
                                                    std::vector<double> &solution,
                                                    const IKCallbackFn &solution_callback,
                                                    moveit_msgs::MoveItErrorCodes &error_code,
                                                    const kinematics::KinematicsQueryOptions &options,
                                                    const moveit::core::RobotState* context_state) const
{
    if (ik_seed_state.size() != interface_->groupVariableCount())
    {
        ROS_ERROR("IK seed state size (%lu) does not equal the number of group variables (%u)", ik_seed_state.size(), interface_->groupVariableCount());
        return false;
    }

    if (interface_->mobileBaseVariableCount() != 7)
    {
        ROS_ERROR("This IK method is meant for a robot with a floating base");
        return false;
    }

    // Creating the seed state
    robot_state::RobotStatePtr state(new robot_state::RobotState(interface_->getRobotModel()));
    const std::vector<int>& group_state_bijection = interface_->getGroupToRobotStateBijection();
    double* joint_angles = state->getVariablePositions();
    for(size_t i = 0; i < ik_seed_state.size(); ++i)
        joint_angles[group_state_bijection[i]] = ik_seed_state[i];
    state->update();  // forward kinematics

    TreeIkRequest request;
    TreeIkResponse response;

    // Hopefully one of the tips is at the same pose as the seed state
    std::string fixed_link("");
    std::string moving_link("");
    for(size_t i = 0; i < tip_frames_.size(); ++i)
    {
        // NOTE: The ik_poses are given in the coordinate frame of base_frame_.  Should probably transform the
        // pose below into global frame in case the global frame is NOT the base_frame_.
        const Eigen::Affine3d& pose = state->getGlobalLinkTransform(tip_frames_[i]);
        geometry_msgs::Pose pose_msg;
        tf::poseEigenToMsg(pose, pose_msg);

        if (equalPoses(pose_msg, ik_poses[i]))
            request.addFixedLink(tip_frames_[i]);
        else
            request.addLinkPose(tip_frames_[i], ik_poses[i]);
    }

    // Not sure which link to set as the fixed base
    if (request.getFixedLinks().size() == 0)
    {
        ROS_ERROR("TreeKinematics: Fixed base link not found");
        return false;
    }
    if (request.getMovingLinks().size() == 0)
    {
        ROS_ERROR("TreeKinematics: No IK tasks");
        return false;
    }

    // Setting joint seed values for the IK request
    const std::vector<std::string>& all_joints = interface_->getAllJointNames();
    std::vector<double> joint_seed(interface_->allVariableCount());
    const std::vector<int>& ik_state_bijection = interface_->getIKJointsToRobotStateBijection();
    for(size_t i = 0; i < all_joints.size(); ++i)
        joint_seed[i] = joint_angles[ik_state_bijection[i]];
    request.setJointValues(joint_seed);

    // Posing the robot in space.  By convention, the floating joint is the last seven DOFs in the seed state.
    geometry_msgs::Pose world_pose_msg;
    world_pose_msg.position.x = ik_seed_state[interface_->groupVariableCount() - 7];
    world_pose_msg.position.y = ik_seed_state[interface_->groupVariableCount() - 6];
    world_pose_msg.position.z = ik_seed_state[interface_->groupVariableCount() - 5];
    world_pose_msg.orientation.x = ik_seed_state[interface_->groupVariableCount() - 4];
    world_pose_msg.orientation.y = ik_seed_state[interface_->groupVariableCount() - 3];
    world_pose_msg.orientation.z = ik_seed_state[interface_->groupVariableCount() - 2];
    world_pose_msg.orientation.w = ik_seed_state[interface_->groupVariableCount() - 1];

    Eigen::Affine3d world_pose;
    tf::poseMsgToEigen(world_pose_msg, world_pose);
    request.setWorldState(world_pose);

    // IK request
    if (interface_->getPositionIk(request, response))
    {
        solution.assign(response.getJointValues().begin(), response.getJointValues().end());

        // Add the world pose.  A bit of a hard-code here - the order of the joint
        // variables is assumed to be x,y,z, qx, qy, qz, qw.
        const Eigen::Affine3d& world_pose = response.getWorldState();

        solution.push_back(world_pose.translation()[0]);    // x
        solution.push_back(world_pose.translation()[1]);    // y
        solution.push_back(world_pose.translation()[2]);    // z

        Eigen::Quaternion<double> q(world_pose.rotation());
        solution.push_back(q.x());
        solution.push_back(q.y());
        solution.push_back(q.z());
        solution.push_back(q.w());

        return true;
    }
    return false;
}

bool MoveItR2TreeKinematicsPlugin::getPositionFK(const std::vector<std::string> &link_names,
                                                 const std::vector<double> &joint_angles,
                                                 std::vector<geometry_msgs::Pose> &poses) const
{
    if (joint_angles.size() != interface_->groupVariableCount())
    {
        ROS_ERROR("Joint angles vector must have size %u.  Received vector with size %lu", interface_->groupVariableCount(), joint_angles.size());
        return false;
    }

    // Constructing map with names of frames we want
    std::map<std::string, KDL::Frame> frames;
    for(size_t i = 0; i < link_names.size(); ++i)
        frames[link_names[i]] = KDL::Frame();

    if (interface_->getPositionFK(joint_angles, frames))
    {
        poses.resize(link_names.size());

        // Extracting pose information
        for (size_t i = 0; i < poses.size(); ++i)
        {
            const std::string& link_name = link_names[i];
            tf::poseKDLToMsg(frames[link_name], poses[i]);

            Eigen::Affine3d pose;
            tf::poseMsgToEigen(poses[i], pose);

            // Need to transform all frames into world frame (world frame is conveniently located in joint_angles)
            // TODO: We need to transform all frames into base_frame_, which could be different from the world frame
            if (interface_->mobileBaseVariableCount() > 0)
            {
                if (interface_->mobileBaseVariableCount() != 7)
                    ROS_WARN("This FK is meant for a robot with a floating base.  Not setting mobile base values");
                else
                {
                    unsigned int idx = joint_angles.size() - interface_->mobileBaseVariableCount();

                    // A bit of a hard-code here - the order of the
                    // variables is assumed to be x,y,z, qx, qy, qz, qw
                    Eigen::Affine3d root_frame = Eigen::Affine3d::Identity();
                    root_frame.translation()(0) = joint_angles[idx++];
                    root_frame.translation()(1) = joint_angles[idx++];
                    root_frame.translation()(2) = joint_angles[idx++];

                    Eigen::Quaterniond q(joint_angles[idx+3], joint_angles[idx], joint_angles[idx+1], joint_angles[idx+2]);
                    root_frame = root_frame * Eigen::Affine3d(q.toRotationMatrix());

                    // One transformation please...
                    pose = root_frame * pose;
                    tf::poseEigenToMsg(pose, poses[i]);
                }
            }
        }
        return true;
    }
    return false;
}

bool MoveItR2TreeKinematicsPlugin::initialize(const std::string& robot_description,
                                              const std::string& group_name,
                                              const std::string& base_frame,
                                              const std::string& tip_frame,
                                              double search_discretization)
{
    std::vector<std::string> tip_frames;
    tip_frames.push_back(tip_frame);
    return initialize(robot_description, group_name, base_frame, tip_frames, search_discretization);
}

bool MoveItR2TreeKinematicsPlugin::initialize(const std::string& robot_description,
                                              const std::string& group_name,
                                              const std::string& base_frame,
                                              const std::vector<std::string>& tip_frames,
                                              double search_discretization)
{
    setValues(robot_description, group_name, base_frame, tip_frames, search_discretization);

    ROS_INFO("Initializing MoveItR2TreeKinematics for \"%s\"", group_name_.c_str());
    ROS_INFO("  Base frame: %s", base_frame_.c_str());
    ROS_INFO("  %lu Tip frames:", tip_frames_.size());
    for(size_t i = 0; i < tip_frames_.size(); ++i)
        ROS_INFO("  [%lu]: %s", i, tip_frames_[i].c_str());

    if (interface_)
        delete interface_;
    interface_ = new R2TreeKinematicsInterface();
    return interface_->initialize(robot_description_, group_name_, base_frame_, tip_frames_);
}

const std::vector<std::string>& MoveItR2TreeKinematicsPlugin::getJointNames() const
{
    return interface_->getJointNames();
}

const std::vector<std::string>& MoveItR2TreeKinematicsPlugin::getLinkNames() const
{
    return interface_->getLinkNames();
}

const bool MoveItR2TreeKinematicsPlugin::supportsGroup(const moveit::core::JointModelGroup *jmg,
                                                       std::string* error_text_out) const
{
    // We can do kinematics for virtually anything.
    return true;
}

} // namespace moveit_r2_kinematics
