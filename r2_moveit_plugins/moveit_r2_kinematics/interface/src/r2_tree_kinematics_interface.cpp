#include "moveit_r2_kinematics/r2_tree_kinematics_interface.h"
#include "moveit_r2_kinematics/tree_kinematics_tolerances.h"

#include <moveit/rdf_loader/rdf_loader.h>

#include <kdl_conversions/kdl_msg.h>
#include <tf/transform_datatypes.h>

namespace moveit_r2_kinematics
{

//// R2TreeKinematicsInterface ////
R2TreeKinematicsInterface::R2TreeKinematicsInterface() : ik_(NULL), fk_(NULL)
{
}

R2TreeKinematicsInterface::~R2TreeKinematicsInterface()
{
    if (ik_)
        delete ik_;
    if (fk_)
        delete fk_;
}

bool R2TreeKinematicsInterface::initialize(const std::string& robot_description,
                                           const std::string& group_name,
                                           const std::string& base_frame,
                                           const std::vector<std::string>& tip_frames)
{
    // Loading URDF and SRDF
    rdf_loader::RDFLoader rdf_loader(robot_description);
    const boost::shared_ptr<srdf::Model> &srdf = rdf_loader.getSRDF();
    const boost::shared_ptr<urdf::ModelInterface>& urdf = rdf_loader.getURDF();

    if (!urdf)
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
    robot_model_.reset(new robot_model::RobotModel(urdf, srdf));

    // Extract the joint group
    if(!robot_model_->hasJointModelGroup(group_name))
    {
        ROS_ERROR("Kinematic model does not contain group \"%s\"", group_name.c_str());
        return false;
    }

    // Getting kinematic model for the joint group in question
    robot_model::JointModelGroup* jmg = robot_model_->getJointModelGroup(group_name);
    group_variable_count_ = jmg->getVariableCount();

    ik_ = new MobileTreeIk(); // this is an old version of MobileTreeIk.  New stuff is really picky about velocities
    //ik_ = new R2TreeIk();
    fk_ = new KdlTreeFk();

    // Attempt to initialize kinematics solvers with a simplified URDF, for efficiency
    ros::NodeHandle nh;
    bool initialized_kinematics = false;
    bool use_simplified_urdf = false;
    if (nh.hasParam(group_name + "/simplified_robot_description"))
    {
        try
        {
            // Initializing Ik and FK from URDF parameter
            fk_->loadFromParam(group_name + "/simplified_robot_description");
            ik_->loadFromParam(group_name + "/simplified_robot_description"); // 2.5x speedup on legs IK when excluding the upper body
            ROS_INFO("TreeKinematics: Loaded simplified URDF for group '%s'", group_name.c_str());
            initialized_kinematics = true;
            use_simplified_urdf = true;
        }
        catch (std::runtime_error) {} // if the simplified_robot_description doesn't exist, a runtime error will be thrown
    }

    // No simplified URDF found, or there was an error loading simplified URDF
    if (!initialized_kinematics)
    {
        try
        {
            fk_->loadFromParam(robot_description);
            ik_->loadFromParam(robot_description);
        }
        catch (std::runtime_error)
        {
            return false;
        }
    }

    ik_->getJointNames(joint_names_);  // reading in ALL joint names from IK representation

    const std::vector<const robot_model::JointModel*>& joint_roots = jmg->getJointRoots();
    mobile_base_ = false;
    mobile_base_variable_count_ = 0;
    std::string base_joint_ = "";  // TODO: This should probably be equal to base_frame that is passed in.
    for (size_t i = 0; i < joint_roots.size(); ++i)
    {
        if (joint_roots[i]->getType() == robot_model::JointModel::FLOATING)
        {
            base_joint_ = joint_roots[i]->getName();
            mobile_base_ = true;
            mobile_base_variable_count_ = joint_roots[i]->getVariableCount();
        }
    }

    if (!mobile_base_)
        ROS_WARN("TreeKinematics: Expected a floating root joint");

    // Getting default joint values
    std::map<std::string, double> default_joints;
    std::map<std::string, double> joint_inertias;
    robot_model_->getVariableDefaultPositions(default_joints);

    // The total number of DOF, minus floating root
    if (use_simplified_urdf)
        total_dofs_ = joint_names_.size();
    else
        total_dofs_ = default_joints.size() - mobile_base_variable_count_;
    default_joint_positions_.resize(total_dofs_);

    //ROS_INFO("Initializing MobileTreeIK for \"%s\" with %u DOFs", group_name_.c_str(), total_dofs_);

    std::vector<double> min_joint_limits;
    std::vector<double> max_joint_limits;
    std::vector<double> joint_vel_limits;
    // Extracting all joint limits from URDF and joints in the group
    // TODO: Read joint limits from MoveIt yaml config instead?
    const KDL::Tree& kdl_tree = ik_->getTree();
    for (size_t i = 0; i < joint_names_.size(); ++i)
    {
        const moveit::core::VariableBounds& bounds = robot_model_->getVariableBounds(joint_names_[i]);
        std::pair<double, double> limits(bounds.min_position_, bounds.max_position_);

        // Need to limit the roll joints even further.  Robodyn does this, but these values are not in the URDF yet.
        if (joint_names_[i] == "r2/left_leg/joint2" || joint_names_[i] == "r2/left_leg/joint4" || joint_names_[i] == "r2/left_leg/joint6" ||
            joint_names_[i] == "r2/right_leg/joint2" || joint_names_[i] == "r2/right_leg/joint4" || joint_names_[i] == "r2/right_leg/joint6")
        {
            limits.first = -2.96705973; // -170 degrees
            limits.second = 2.96705973; // 170 degrees
        }

        // Inserting into joint limit maps
        min_joint_limits.push_back(limits.first);
        max_joint_limits.push_back(limits.second);
        joint_vel_limits.push_back(MAX_JOINT_VELOCITY);

        // Getting mass for each link
        std::string child_link = robot_model_->getJointModel(joint_names_[i])->getChildLinkModel()->getName();
        double mass = kdl_tree.getSegment(child_link)->second.segment.getInertia().getMass();
        joint_inertias[joint_names_[i]] = mass;

        // Save default joint value
        default_joint_positions_(i) = default_joints[joint_names_[i]];

        // Constructing bijection between joint value array for internal ik and joint value array for MoveIt robot state.
        const robot_model::JointModel* joint = robot_model_->getJointModel(joint_names_[i]);
        const std::vector<std::string>& var_names = joint->getVariableNames();
        for(size_t j = 0; j < var_names.size(); ++j)
            ik_joints_to_robot_joints_bijection_.push_back(robot_model_->getVariableIndex(var_names[j]));

        // This joint is in the group
        if (jmg->hasJointModel(joint_names_[i]))
        {
            group_joint_index_map_[joint_names_[i]] = i;
            group_joints_.push_back(joint_names_[i]);
            group_links_.push_back(robot_model_->getJointModel(joint_names_[i])->getChildLinkModel()->getName());

            // Constructing bijection between joint value array for the group and joint value array for MoveIt robot state.
            for(size_t j = 0; j < var_names.size(); ++j)
                group_joint_to_robot_state_bijection_.push_back(robot_model_->getVariableIndex(var_names[j]));
        }

        //ROS_INFO(" [%lu] - %s with range [%1.4f, %1.4f] and mass %f", i, joint_names_[i].c_str(), limits.first, limits.second, mass);
        //ROS_INFO("            Attached to link: %s", robot_model_->getJointModel(joint_names_[i])->getChildLinkModel()->getName().c_str());
    }

    // Adding the floating (virtual) joint to the group joints list since it is passively actuated
    // By design, this joint and link is always the last entry in the group_joints and group_links lists.
    if (mobile_base_)
    {
        const robot_model::JointModel* joint = robot_model_->getJointModel(base_joint_);

        group_joints_.push_back(base_joint_);
        group_links_.push_back(joint->getChildLinkModel()->getName());

        const std::vector<std::string>& var_names = joint->getVariableNames();
        for(size_t j = 0; j < var_names.size(); ++j)
            group_joint_to_robot_state_bijection_.push_back(robot_model_->getVariableIndex(var_names[j]));
    }

    if(group_joints_.size() != group_links_.size())
    {
        ROS_ERROR("TreeKinematics: The number of joints is not equal to the number of links!");
        return false;
    }
    if(group_joint_to_robot_state_bijection_.size() != group_variable_count_)
    {
        ROS_ERROR("TreeKinematics: Bijection size mismatch");
        return false;
    }

    // Initializing position limiter (joint limits) for MobileTreeIk
    boost::shared_ptr<JointNamePositionLimiter> position_limiter(new JointNamePositionLimiter());
    position_limiter->setJointPositionLimiter(boost::shared_ptr<JointPositionLimiter>(new JointPositionLimiter()));
    //position_limiter->setLimits(joint_names_, min_joint_limits, max_joint_limits, joint_vel_limits);
    position_limiter->setLimits(joint_names_, min_joint_limits, max_joint_limits);

    // Set IK joint limits and inertias
    ik_->setPositionLimiter(position_limiter);
    ik_->setJointInertias(joint_inertias);

    // Construct tolerance map for different priorities
    std::map<int, std::pair<double, double> > priority_tol; // linear and angular tolerances for each priority level
    priority_tol[KdlTreeIk::CRITICAL]   = std::make_pair<double, double>(CRITICAL_PRIO_LINEAR_TOL,  CRITICAL_PRIO_ANGULAR_TOL);
    priority_tol[KdlTreeIk::HIGH]       = std::make_pair<double, double>(HIGH_PRIO_LINEAR_TOL,      HIGH_PRIO_ANGULAR_TOL);
    priority_tol[KdlTreeIk::MEDIUM]     = std::make_pair<double, double>(MEDIUM_PRIO_LINEAR_TOL,    MEDIUM_PRIO_ANGULAR_TOL);
    priority_tol[KdlTreeIk::LOW]        = std::make_pair<double, double>(LOW_PRIO_LINEAR_TOL,       LOW_PRIO_ANGULAR_TOL);
    ik_->setPriorityTol(priority_tol);
    ik_->setMaxJointVel(MAX_JOINT_VELOCITY); // remove when kinematics works
    //ik_->setMaxTwist(MAX_TWIST);
    //ik_->setMBar(1e-12);

    return true;
}

bool R2TreeKinematicsInterface::getPositionFK(const std::vector<double> &joint_angles,
                                              std::map<std::string, KDL::Frame>& frames) const
{
    // Set the input joint values to the defaults
    // Change the joint values to those specified in link_names and joint_angles
    KDL::JntArray joints_in = default_joint_positions_;

    // Seeding input state
    // TODO: This is really slow.  Make faster with bijection.
    for(size_t i = 0; i < group_joints_.size(); ++i)
    {
        std::map<std::string, unsigned int>::const_iterator it;
        it = group_joint_index_map_.find(group_joints_[i]);

        if (it != group_joint_index_map_.end())
            joints_in(it->second) = joint_angles[i];
    }

    // Requesting frames
    bool valid = true;
    fk_mutex_.lock();
    try
    {
        fk_->getPoses(joints_in, frames);
    }
    catch (std::runtime_error e)
    {
        ROS_WARN("Exception caught during FK computation: %s", e.what());
        valid = false;
    }
    fk_mutex_.unlock();

    return valid;
}

bool R2TreeKinematicsInterface::getPositionIk(const TreeIkRequest& request, TreeIkResponse& response) const
{
    // convert geometry_msgs::Pose into KDL frames
    std::vector<KDL::Frame> frames;
    const std::vector<geometry_msgs::Pose>& poses = request.getMovingLinkPoses();
    for (size_t i = 0; i < poses.size(); ++i)
    {
        KDL::Frame frame;
        tf::poseMsgToKDL(poses[i], frame);
        frames.push_back(frame);
    }

    const std::vector<double>& seed_joints = request.getJointValues();
    if (seed_joints.size() != default_joint_positions_.rows())
    {
        ROS_ERROR("# joints in seed (%lu) not equal to the number of DOF (%u) in system", seed_joints.size(), default_joint_positions_.rows());
        response.setFailure();
        return false;
    }

    if (request.getFixedLinks().size() == 0)
    {
        ROS_ERROR("No fixed base links specified");
        response.setFailure();
        return false;
    }

    KDL::JntArray joints_in = default_joint_positions_;
    // Update given ik_seed
    for (size_t i = 0; i < seed_joints.size(); ++i)
        joints_in(i) = seed_joints[i];

    KDL::JntArray joints_out;
    joints_out.resize(total_dofs_);

    std::vector<double> world_joints; // store the resulting world DOFs here

    // NOTE: For some horrible reason, boost::mutex::scoped_lock does NOT actually acquire ik_mutex_.
    // MobileTreeIK is NOT threadsafe!
    ik_mutex_.lock();

    try
    {
        // Set fixed bases
        ik_->setBases(request.getFixedLinks());

        // Update world state
        ik_->setMobileJoints(request.getWorldStateRPY());

        // Performing IK
        ik_->getJointPositions(joints_in, request.getMovingLinks(), frames, joints_out, request.getPriorities());
    }
    catch (std::runtime_error e) // MobileTreeIk throws an exception when IK fails.
    {
        //ROS_ERROR("IK Failed - %s", e.what());
        response.setFailure();
        ik_mutex_.unlock();
        return false;
    }

    // fill out response
    ik_->getMobileJoints(world_joints);

    // We no longer need the lock
    ik_mutex_.unlock();

    // Getting the values of the passive DOFs (the world pose)
    Eigen::Affine3d world_pose;
    world_pose.translation()[0] = world_joints[0];
    world_pose.translation()[1] = world_joints[1];
    world_pose.translation()[2] = world_joints[2];

    world_pose = Eigen::Translation3d(world_pose.translation()) *
                (Eigen::AngleAxisd(world_joints[3], Eigen::Vector3d::UnitX()) *
                 Eigen::AngleAxisd(world_joints[4], Eigen::Vector3d::UnitY()) *
                 Eigen::AngleAxisd(world_joints[5], Eigen::Vector3d::UnitZ()));

    // Construct solution vector of joint positions
    std::vector<double> solution;
    for (unsigned int i = 0; i < group_joints_.size() - (mobile_base_ ? 1 : 0); ++i) // -1 excludes the floating multi-dof root joint.  This joint is ALWAYS added last
    {
        unsigned int idx = group_joint_index_map_.find(group_joints_[i])->second;
        solution.push_back(joints_out(idx));
    }

    response.setValues(world_pose, solution);
    return true;
}

const std::vector<std::string>& R2TreeKinematicsInterface::getAllJointNames() const
{
    return joint_names_;
}

bool R2TreeKinematicsInterface::hasMobileBase() const
{
    return mobile_base_ > 0;
}

unsigned int R2TreeKinematicsInterface::mobileBaseVariableCount() const
{
    return mobile_base_variable_count_;
}

unsigned int R2TreeKinematicsInterface::groupVariableCount() const
{
    return group_variable_count_;
}

robot_model::RobotModelPtr R2TreeKinematicsInterface::getRobotModel() const
{
    return robot_model_;
}

const std::vector<int>& R2TreeKinematicsInterface::getGroupToRobotStateBijection() const
{
    return group_joint_to_robot_state_bijection_;
}

const std::vector<int>& R2TreeKinematicsInterface::getIKJointsToRobotStateBijection() const
{
    return ik_joints_to_robot_joints_bijection_;
}

unsigned int R2TreeKinematicsInterface::allVariableCount() const
{
    return total_dofs_;
}

const KDL::JntArray& R2TreeKinematicsInterface::getAllDefaultJointPositions() const
{
    return default_joint_positions_;
}

//// TreeIkRequest ////
void TreeIkRequest::addFixedLink(const std::string& link_name)                     { fixed_links_.push_back(link_name); }
void TreeIkRequest::setJointValues(const std::vector<double>& values)              { initial_joints_ = values; }
const std::vector<std::string>& TreeIkRequest::getFixedLinks()               const { return fixed_links_; }
const std::vector<std::string>& TreeIkRequest::getMovingLinks()              const { return moving_links_; }
const std::vector<geometry_msgs::Pose>& TreeIkRequest::getMovingLinkPoses()  const { return poses_; }
const std::vector<double>& TreeIkRequest::getJointValues()                   const { return initial_joints_; }
const geometry_msgs::Pose& TreeIkRequest::getWorldState()                    const { return world_state_; }
const std::vector<double>& TreeIkRequest::getWorldStateRPY()                 const { return world_state_rpy_; }
const std::vector<KdlTreeIk::NodePriority>& TreeIkRequest::getPriorities()   const { return priorities_; }

void TreeIkRequest::addLinkPose(const std::string& link_name, const geometry_msgs::Pose& pose, const int priority)
{
    moving_links_.push_back(link_name);
    poses_.push_back(pose);
    KdlTreeIk::NodePriority priority_vec;
    for (size_t i = 0; i < 6; ++i) // Set each DOF to the given priority
        priority_vec[i] = priority;
    priorities_.push_back(priority_vec);
}

void TreeIkRequest::addLinkPose(const std::string& link_name, const geometry_msgs::Pose& pose, const std::vector<int>& priority)
{
    if (priority.size() != 6)
    {
        ROS_ERROR("Expected priority vector of size 6, but received vector with size %lu.  Not adding link pose", priority.size());
        return;
    }

    moving_links_.push_back(link_name);
    poses_.push_back(pose);
    KdlTreeIk::NodePriority priority_vec;
    for (size_t i = 0; i < priority.size(); ++i) // Set each DOF to the given priority
        priority_vec[i] = priority[i];
    priorities_.push_back(priority_vec);
}

void TreeIkRequest::setWorldState(const Eigen::Affine3d& pose)
{
    Eigen::Vector3d rpy = pose.rotation().eulerAngles(0,1,2);
    Eigen::Vector3d trans = pose.translation();

    world_state_rpy_.resize(6);
    world_state_rpy_[0] = trans(0);
    world_state_rpy_[1] = trans(1);
    world_state_rpy_[2] = trans(2);
    world_state_rpy_[3] = rpy(0);
    world_state_rpy_[4] = rpy(1);
    world_state_rpy_[5] = rpy(2);
}

//// TreeIkResponse ////
bool TreeIkResponse::successful()                           const { return success_; }
const Eigen::Affine3d& TreeIkResponse::getWorldState()      const { return world_pose_; }
const std::vector<double>& TreeIkResponse::getJointValues() const { return joint_values_; }
void TreeIkResponse::setFailure()                                 { success_ = false; }
void TreeIkResponse::setValues(const Eigen::Affine3d& world, const std::vector<double>& joints)
{
    success_ = true;
    world_pose_ = world;
    joint_values_ = joints;
}


}