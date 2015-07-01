/* Author: Ryan Luna */

#include "moveit_r2_kinematics/moveit_r2_chain_kinematics.h"

// URDF, SRDF
#include <urdf_model/model.h>
#include <srdfdom/model.h>
#include <moveit/rdf_loader/rdf_loader.h>

// Conversion from KDL pose to ROS msg
#include <kdl_conversions/kdl_msg.h>
#include <tf/transform_datatypes.h>

// For plugin library
#include <class_loader/class_loader.h>

// NASA R2
#include "nasa_robodyn_controllers_core/KdlTreeIk.h"

//register MoveItR2ChainKinematicsPlugin  as a KinematicsBase implementation
CLASS_LOADER_REGISTER_CLASS(moveit_r2_kinematics::MoveItR2ChainKinematicsPlugin, kinematics::KinematicsBase)

namespace moveit_r2_kinematics
{

//// Kinematics Plugin ////
MoveItR2ChainKinematicsPlugin::MoveItR2ChainKinematicsPlugin() : ik_(NULL), fk_(NULL)
{
}

MoveItR2ChainKinematicsPlugin::~MoveItR2ChainKinematicsPlugin()
{
    if (ik_)
        delete ik_;
    if (fk_)
        delete fk_;
}

bool MoveItR2ChainKinematicsPlugin::getPositionIK(const geometry_msgs::Pose &ik_pose,
                                                  const std::vector<double> &ik_seed_state,
                                                  std::vector<double> &solution,
                                                  moveit_msgs::MoveItErrorCodes &error_code,
                                                  const kinematics::KinematicsQueryOptions &options) const
{
    // convert geometry_msgs::Pose into KDL frame
    KDL::Frame kdl_frame;
    tf::poseMsgToKDL(ik_pose, kdl_frame);

    // R2 API requires a vector of poses and links
    std::vector<KDL::Frame> kdl_frames;
    kdl_frames.push_back(kdl_frame);

    // Update given ik_seed
    KDL::JntArray joints_in = default_joint_positions_;
    for(size_t i = 0; i < group_joints_.size(); i++)
    {
        unsigned int idx = group_joint_index_map_.find(group_joints_[i])->second;
        joints_in(idx) = ik_seed_state[i];
    }

    // Computing IK
    std::vector<KDL::JntArray> jointsOut;
    try
    {
        ik_->getJointPositions(joints_in, kdl_frames, jointsOut);
    }
    catch (std::runtime_error e)
    {
        // R2 code also prints this warning
        //ROS_WARN("IK Failed - %s", e.what());
        return false;
    }

    // Construct solution vector
    solution.resize(num_dofs_);
    KDL::JntArray final_joint_positions = jointsOut[0];
    for (unsigned int i = 0; i < group_joints_.size(); ++i)
    {
        unsigned int idx = group_joint_index_map_.find(group_joints_[i])->second;
        solution[i] = final_joint_positions(idx);
    }

    return true;
}

bool MoveItR2ChainKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose &ik_pose,
                                                     const std::vector<double> &ik_seed_state,
                                                     double timeout,
                                                     std::vector<double> &solution,
                                                     moveit_msgs::MoveItErrorCodes &error_code,
                                                     const kinematics::KinematicsQueryOptions &options) const
{
    return getPositionIK(ik_pose, ik_seed_state, solution, error_code, options);
}

bool MoveItR2ChainKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose &ik_pose,
                                                     const std::vector<double> &ik_seed_state,
                                                     double timeout,
                                                     const std::vector<double> &consistency_limits,
                                                     std::vector<double> &solution,
                                                     moveit_msgs::MoveItErrorCodes &error_code,
                                                     const kinematics::KinematicsQueryOptions &options) const
{
    return getPositionIK(ik_pose, ik_seed_state, solution, error_code, options);
}

bool MoveItR2ChainKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose &ik_pose,
                                                     const std::vector<double> &ik_seed_state,
                                                     double timeout,
                                                     std::vector<double> &solution,
                                                     const IKCallbackFn &solution_callback,
                                                     moveit_msgs::MoveItErrorCodes &error_code,
                                                     const kinematics::KinematicsQueryOptions &options) const
{
    return getPositionIK(ik_pose, ik_seed_state, solution, error_code, options);
}

bool MoveItR2ChainKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose &ik_pose,
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

bool MoveItR2ChainKinematicsPlugin::getPositionFK(const std::vector<std::string> &link_names,
                                                  const std::vector<double> &joint_angles,
                                                  std::vector<geometry_msgs::Pose> &poses) const
{
    poses.resize(link_names.size());
    if(joint_angles.size() != group_joints_.size())
    {
        ROS_ERROR("Joint angles vector must have size: %lu",group_joints_.size());
        return false;
    }

    // Set the input joint values to the defaults
    // Change the joint values to those specified in link_names and joint_angles
    KDL::JntArray joints_in = default_joint_positions_;
    for(size_t i = 0; i < poses.size(); i++)
    {
        unsigned int idx = group_joint_index_map_.find(link_names[i])->second;
        joints_in(idx) = joint_angles[i];
    }

    // Constructing empty map with names of frames we want
    std::map<std::string, KDL::Frame> frames;
    for (size_t i = 0; i < poses.size(); ++i)
    {
        // We're given links, but the FK requires joint names.  Get parent joint for each link
        const std::string& joint_name = robot_model_->getLinkModel(link_names[i])->getParentJointModel()->getName();
        frames[joint_name] = KDL::Frame();
    }

    // Requesting all frames
    bool valid = fk_ != NULL;
    try
    {
        if (fk_)
        {
            fk_->getPoses(joints_in, frames);
            // Extracting pose information
            for (size_t i = 0; i < poses.size(); ++i)
            {
                const std::string& joint_name = robot_model_->getLinkModel(link_names[i])->getParentJointModel()->getName();
                tf::poseKDLToMsg(frames[joint_name], poses[i]);
            }
        }
        else
        {
            ROS_WARN("Forward Kinematics is not initialized");
        }
    }
    catch (std::runtime_error e)
    {
        ROS_WARN("Exception caught during FK computation: %s", e.what());
        valid = false;
    }

    return valid;
}

bool MoveItR2ChainKinematicsPlugin::initialize(const std::string& robot_description,
                                               const std::string& group_name,
                                               const std::string& base_frame,
                                               const std::string& tip_frame,
                                               double search_discretization)
{
    // Consciously NOT using the provided setValues function in the base
    // This method removes the leading slash from base_frame and tip_frame, which wreaks havoc on the current R2 naming conventions for joints
    /*robot_description_ = robot_description;
    group_name_ = group_name;
    base_frame_ = base_frame;
    tip_frame_ = tip_frame;

    if (base_frame_[0] != '/')
        base_frame_ = "/" + base_frame_;
    if (tip_frame_[0] != '/')
        tip_frame_ = "/" + tip_frame_;
    tip_frames_.push_back(tip_frame_);

    search_discretization_ = search_discretization;
    // DO NOT USE THIS FUNCTION-----> setValues(robot_description, group_name, base_frame, tip_frame, search_discretization); <---- DO NOT USE THIS*/
    setValues(robot_description, group_name, base_frame, tip_frame, search_discretization);

    ROS_INFO("Initializing MoveItR2ChainKinematics for group \"%s\" with base frame \"%s\" and tip frame \"%s\"", group_name_.c_str(), base_frame_.c_str(), tip_frame_.c_str());

    // Loading URDF and SRDF
    rdf_loader::RDFLoader rdf_loader(robot_description_);
    const boost::shared_ptr<srdf::Model> &srdf = rdf_loader.getSRDF();
    const boost::shared_ptr<urdf::ModelInterface>& urdf_model = rdf_loader.getURDF();

    if (!urdf_model)
    {
        ROS_ERROR("URDF must be loaded for R2 kinematics solver to work.");
        return false;
    }
    if (!srdf)
    {
        ROS_ERROR("SRDF must be loaded for R2 kinematics solver to work.");
        return false;
    }

    // Load the robot kinematics and semantics (e.g., group information)
    robot_model_.reset(new robot_model::RobotModel(urdf_model, srdf));

    // Extract the joint group
    if(!robot_model_->hasJointModelGroup(group_name_))
    {
        ROS_ERROR("Kinematic model does not contain group \"%s\"", group_name_.c_str());
        return false;
    }

    // Getting kinematic model for the joint group in question
    robot_model::JointModelGroup* jmg = robot_model_->getJointModelGroup(group_name_);

    KdlTreeIk tree_ik;
    tree_ik.loadFromParam(robot_description_);

    KDL::Chain chain;
    if(!tree_ik.getTree().getChain(base_frame_, tip_frame_, chain))
    {
        ROS_ERROR("Failed to initialize kinematic chain with base %s and root %s", base_frame_.c_str(), tip_frame_.c_str());
        return false;
    }

    ik_ = new KdlChainIk(chain);
    fk_ = new KdlTreeFk();

    // Initializing Ik and FK from URDF parameter
    fk_->loadFromParam(robot_description_);
    ik_->getJointNames(joint_names_);

    // Setting joint limits from URDF
    //std::map<std::string, double> min_joint_limits, max_joint_limits;

    // Getting default joint values
    std::map<std::string, double> default_joints;
    robot_model_->getVariableDefaultPositions(default_joints);

    // The total number of DOF
    num_dofs_ = jmg->getVariableCount();
    default_joint_positions_.resize(num_dofs_);

    ROS_INFO("Initializing KdlChainIK for \"%s\" with %u DOFs", group_name_.c_str(), num_dofs_);

    // The order of joint_names_ comes from in house kinematics, and is important to maintain.
    for (size_t i = 0; i < joint_names_.size(); ++i)
    {
        //const moveit::core::VariableBounds& bounds = robot_model_->getVariableBounds(joint_names_[i]);
        //std::pair<double, double> limits(bounds.min_position_, bounds.max_position_);

        //ROS_INFO("    [%lu] - %s with range [%1.4f, %1.4f]", i, joint_names_[i].c_str(), limits.first, limits.second);
        //ROS_INFO("            Attached to link: %s", robot_model_->getJointModel(joint_names_[i])->getChildLinkModel()->getName().c_str());

        // Inserting into joint limit maps
        //min_joint_limits[joint_names_[i]] = limits.first;
        //max_joint_limits[joint_names_[i]] = limits.second;

        // This joint is in the group
        if (jmg->hasJointModel(joint_names_[i]))
        {
            group_joint_index_map_[joint_names_[i]] = i;
            group_joints_.push_back(joint_names_[i]);
            group_links_.push_back(robot_model_->getJointModel(joint_names_[i])->getChildLinkModel()->getName());
        }

        // Save default joint value
        default_joint_positions_(i) = default_joints[joint_names_[i]];
    }

    return true;
}

const std::vector<std::string>& MoveItR2ChainKinematicsPlugin::getJointNames() const
{
    return group_joints_;
}

const std::vector<std::string>& MoveItR2ChainKinematicsPlugin::getLinkNames() const
{
    return group_links_;
}

const std::vector<std::string>& MoveItR2ChainKinematicsPlugin::getAllJointNames() const
{
    return joint_names_;
}

} // namespace moveit_r2_kinematics
