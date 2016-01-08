/* Author: Ryan Luna */

#include <ros/ros.h>
#include "moveit_r2_constraints/r2_constraint_sampler.h"
#include "moveit_r2_kinematics/moveit_r2_tree_kinematics.h"
#include "moveit_r2_kinematics/tree_kinematics_tolerances.h"
#include <nasa_robodyn_controllers_core/KdlTreeIk.h> // for priority values

// For plugin library
#include <class_loader/class_loader.h>
#include <eigen_conversions/eigen_msg.h>


/// TOTAL HACK FOR NOW
static const unsigned int numImportantLinks = 3;
static const std::string leftFootLinks[] = {"r2/left_ankle_roll", "r2/left_leg/gripper/jaw_left", "r2/left_leg/gripper/jaw_right"};
static const std::string rightFootLinks[] = {"r2/right_ankle_roll", "r2/right_leg/gripper/jaw_left", "r2/right_leg/gripper/jaw_right"};

moveit_r2_constraints::MoveItR2ConstraintSamplerAllocator::MoveItR2ConstraintSamplerAllocator() : constraint_samplers::ConstraintSamplerAllocator()
{
}

moveit_r2_constraints::MoveItR2ConstraintSamplerAllocator::~MoveItR2ConstraintSamplerAllocator()
{
}

constraint_samplers::ConstraintSamplerPtr moveit_r2_constraints::MoveItR2ConstraintSamplerAllocator::alloc(const planning_scene::PlanningSceneConstPtr &scene,
                                                               const std::string &group_name,
                                                               const moveit_msgs::Constraints &constr)
{
    constraint_samplers::ConstraintSamplerPtr cs(new R2KinematicConstraintSampler(scene, group_name));
    cs->configure(constr);
    return cs;
}

bool moveit_r2_constraints::MoveItR2ConstraintSamplerAllocator::canService(const planning_scene::PlanningSceneConstPtr &scene,
                                                                           const std::string &group_name,
                                                                           const moveit_msgs::Constraints &constr) const
{
    // Any group that starts with 'legs' is good.
    // TODO: Probably should not discriminate based on group name.
    std::size_t idx = group_name.find("legs");
    if (idx != 0)
    //if (group_name != "legs")
    {
        //ROS_ERROR("MoveItR2ConstraintSamplerAllocator: Group '%s' is not allowed", group_name.c_str());
        ROS_ERROR("MoveItR2ConstraintSamplerAllocator cannot service group '%s'", group_name.c_str());
        return false;
    }

    // ONLY POSITION AND ORIENTATION CONSTRAINTS
    if (constr.joint_constraints.size() > 0 || constr.visibility_constraints.size() > 0)
    {
        //ROS_ERROR("MoveItR2ConstraintSamplerAllocator: Can only service position and orientation constraints");
        return false;
    }

    // This is a heavy duty constraint sampler, capable of IK to multiple poses
    // For a single, fixed pose, let the simpler pose sampler handle it.
    if (constr.position_constraints.size() > 1 || constr.orientation_constraints.size() > 1)
        return true;

    return false;
}

moveit_r2_constraints::R2KinematicConstraintSampler::R2KinematicConstraintSampler(const planning_scene::PlanningSceneConstPtr &scene, const std::string &group_name)
    : constraint_samplers::ConstraintSampler(scene, group_name)
{
}


static int getLinearPriorityValue(double value)
{
    int priority;

    if (value < moveit_r2_kinematics::HIGH_PRIO_LINEAR_TOL)
        priority = KdlTreeIk::CRITICAL;
    else if (value < moveit_r2_kinematics::MEDIUM_PRIO_LINEAR_TOL)
        priority = KdlTreeIk::HIGH;
    else if (value < moveit_r2_kinematics::LOW_PRIO_LINEAR_TOL)
        priority = KdlTreeIk::MEDIUM;
    else if (value < 2*moveit_r2_kinematics::LOW_PRIO_LINEAR_TOL)
        priority = KdlTreeIk::LOW;
    else
        priority = KdlTreeIk::IGNORE;

    return priority;
}

static int getAngularPriorityValue(double value)
{
    int priority;

    if (value < moveit_r2_kinematics::HIGH_PRIO_ANGULAR_TOL)
        priority = KdlTreeIk::CRITICAL;
    else if (value < moveit_r2_kinematics::MEDIUM_PRIO_ANGULAR_TOL)
        priority = KdlTreeIk::HIGH;
    else if (value < moveit_r2_kinematics::LOW_PRIO_ANGULAR_TOL)
        priority = KdlTreeIk::MEDIUM;
    else if (value < 2*moveit_r2_kinematics::LOW_PRIO_ANGULAR_TOL)
        priority = KdlTreeIk::LOW;
    else
        priority = KdlTreeIk::IGNORE;

    return priority;
}

static void extractConstraintPriorities(boost::shared_ptr<kinematic_constraints::PositionConstraint> pc,
                                        boost::shared_ptr<kinematic_constraints::OrientationConstraint> oc,
                                        std::vector<int>& priorities)
{
    priorities.resize(6);
    if (!pc)
        priorities[0] = priorities[1] = priorities[2] = KdlTreeIk::IGNORE;
    else
    {
        const std::vector<bodies::BodyPtr>& constraintRegions = pc->getConstraintRegions();
        double xrange = 0.0;
        double yrange = 0.0;
        double zrange = 0.0;

        for(size_t i = 0; i < constraintRegions.size(); ++i)
        {
            switch(constraintRegions[i]->getType())
            {
                case shapes::SPHERE:
                {
                    double radius = constraintRegions[i]->getDimensions()[0];
                    xrange = std::max(xrange, radius);
                    yrange = std::max(yrange, radius);
                    zrange = std::max(zrange, radius);
                    break;
                }

                case shapes::BOX:
                {
                    std::vector<double> dimensions = constraintRegions[i]->getDimensions();
                    xrange = std::max(xrange, dimensions[0]);
                    yrange = std::max(yrange, dimensions[1]);
                    zrange = std::max(zrange, dimensions[2]);
                    break;
                }

                case shapes::CYLINDER:
                {
                    std::vector<double> dimensions = constraintRegions[i]->getDimensions();
                    xrange = std::max(xrange, dimensions[0]); // radius
                    yrange = std::max(yrange, dimensions[0]); // radius
                    zrange = std::max(zrange, dimensions[1]); // height
                    break;
                }

                default:
                {
                    ROS_ERROR("Unknown constraint shape!  Not constraining position");
                    xrange = yrange = zrange = std::numeric_limits<double>::max();
                    break;
                }
            }
        }

        priorities[0] = getLinearPriorityValue(xrange);
        priorities[1] = getLinearPriorityValue(yrange);
        priorities[2] = getLinearPriorityValue(zrange);
    }

    if (!oc)
        priorities[3] = priorities[4] = priorities[5] = KdlTreeIk::IGNORE;
    else
    {
        double Rrange = oc->getXAxisTolerance();
        double Prange = oc->getYAxisTolerance();
        double Yrange = oc->getZAxisTolerance();

        priorities[3] = getAngularPriorityValue(Rrange);
        priorities[4] = getAngularPriorityValue(Prange);
        priorities[5] = getAngularPriorityValue(Yrange);
    }
}

static double getPriorityTolerance(int priority, bool linear)
{
    if (priority == KdlTreeIk::CRITICAL)
        return linear ? moveit_r2_kinematics::CRITICAL_PRIO_LINEAR_TOL : moveit_r2_kinematics::CRITICAL_PRIO_ANGULAR_TOL;
    else if (priority == KdlTreeIk::HIGH)
        return linear ? moveit_r2_kinematics::HIGH_PRIO_LINEAR_TOL : moveit_r2_kinematics::HIGH_PRIO_ANGULAR_TOL;
    else if (priority == KdlTreeIk::MEDIUM)
        return linear ? moveit_r2_kinematics::MEDIUM_PRIO_LINEAR_TOL : moveit_r2_kinematics::MEDIUM_PRIO_ANGULAR_TOL;
    else if (priority == KdlTreeIk::LOW)
        return linear ? moveit_r2_kinematics::LOW_PRIO_LINEAR_TOL : moveit_r2_kinematics::LOW_PRIO_ANGULAR_TOL;
    else if (priority == KdlTreeIk::IGNORE)
        return 0.0;

    ROS_ERROR("Unknown priority value %d", priority);
    return 0.0;
}

bool moveit_r2_constraints::R2KinematicConstraintSampler::configure(const moveit_msgs::Constraints &constr)
{
    clear();
    if (!loadIKSolver())
        return false;


    // There must be at least one position or one orientation constraint
    if (constr.position_constraints.size() == 0 && constr.orientation_constraints.size() == 0)
    {
        logError("No position or orientation constraint to configure");
        return false;
    }

    numFullyConstrainedLinks_ = 0;
    fullyConstrainedLinks_.clear();

    // Check all of the links to configure.  Make sure they are in the model, and
    // are only constrained once by the same kind of constraint
    for (std::size_t p = 0 ; p < constr.position_constraints.size() ; ++p)
    {
        bool configured = false;

        if (!validLink(constr.position_constraints[p].link_name))
        {
            ROS_ERROR("IK cannot be performed for link '%s'", constr.position_constraints[p].link_name.c_str());
            return false;
        }

        // Make sure this link has only one position constraint
        for (std::size_t q = p + 1 ; q < constr.position_constraints.size() ; ++q)
            if (constr.position_constraints[p].link_name == constr.position_constraints[q].link_name)
            {
                ROS_ERROR("Link '%s' has more than one position constraint defined", constr.position_constraints[p].link_name.c_str());
                return false;
            }

        // See if there is an orientation constraint for the same link
        for (std::size_t o = 0 ; o < constr.orientation_constraints.size() ; ++o)
        {
            if (constr.position_constraints[p].link_name == constr.orientation_constraints[o].link_name)
            {
                // Make sure this link has only one orientation constraint
                for (std::size_t q = o + 1 ; q < constr.orientation_constraints.size() ; ++q)
                    if (constr.orientation_constraints[o].link_name == constr.orientation_constraints[q].link_name)
                    {
                        ROS_ERROR("Link '%s' has more than one orientation constraint defined", constr.orientation_constraints[o].link_name.c_str());
                        return false;
                    }

                // Configure the sampling pose for the position and orientation constraint
                boost::shared_ptr<kinematic_constraints::PositionConstraint> pc(new kinematic_constraints::PositionConstraint(scene_->getRobotModel()));
                boost::shared_ptr<kinematic_constraints::OrientationConstraint> oc(new kinematic_constraints::OrientationConstraint(scene_->getRobotModel()));
                if (pc->configure(constr.position_constraints[p], scene_->getTransforms()) && oc->configure(constr.orientation_constraints[o], scene_->getTransforms()))
                {
                    constraint_samplers::IKSamplingPose poseConstraint(pc, oc);
                    if (pc->mobileReferenceFrame())
                        frame_depends_.push_back(pc->getReferenceFrame());
                    if (oc->mobileReferenceFrame())
                        frame_depends_.push_back(oc->getReferenceFrame());

                    sampling_poses_.push_back(poseConstraint);

                    std::vector<int> priorities;
                    extractConstraintPriorities(pc, oc, priorities);
                    pose_priorities_.push_back(priorities);

                    configured = true;

                    numFullyConstrainedLinks_++;
                    fullyConstrainedLinks_.push_back(true);
                }
                else
                {
                    ROS_ERROR("Failed to configure position and orientation constraint for '%s'", constr.position_constraints[p].link_name.c_str());
                    return false;
                }
            }
        }

        // Just a position constraint
        if (!configured)
        {
            // Configure the sampling pose for the position constraint
            boost::shared_ptr<kinematic_constraints::PositionConstraint> pc(new kinematic_constraints::PositionConstraint(scene_->getRobotModel()));
            if (pc->configure(constr.position_constraints[p], scene_->getTransforms()))
            {
                constraint_samplers::IKSamplingPose posConstraint(pc);
                if (pc->mobileReferenceFrame())
                    frame_depends_.push_back(pc->getReferenceFrame());

                sampling_poses_.push_back(posConstraint);

                std::vector<int> priorities;
                extractConstraintPriorities(pc, boost::shared_ptr<kinematic_constraints::OrientationConstraint>(), priorities);
                pose_priorities_.push_back(priorities);

                configured = true;
                fullyConstrainedLinks_.push_back(false);
            }
            else
            {
                ROS_ERROR("Failed to configure position constraint for '%s'", constr.position_constraints[p].link_name.c_str());
                return false;
            }
        }
    }

    // Now, need to check for singular orientation constraints
    for (std::size_t o = 0 ; o < constr.orientation_constraints.size() ; ++o)
    {
        bool hasPosition = false;
        // See if there is NO position constraint for this link
        for (std::size_t p = 0 ; p < constr.position_constraints.size() && !hasPosition; ++p)
            hasPosition = (constr.position_constraints[p].link_name == constr.orientation_constraints[o].link_name);

        if (!hasPosition)
        {
            if (!validLink(constr.orientation_constraints[o].link_name))
            {
                ROS_ERROR("IK cannot be performed for link '%s'", constr.orientation_constraints[o].link_name.c_str());
                return false;
            }

            // Make sure this link is constrained just once
            for (std::size_t q = o + 1 ; q < constr.orientation_constraints.size() ; ++q)
                if (constr.orientation_constraints[o].link_name == constr.orientation_constraints[q].link_name)
                {
                    ROS_ERROR("Link '%s' has more than one orientation constraint defined", constr.orientation_constraints[o].link_name.c_str());
                    return false;
                }

            // Configure the sampling pose for the orientation constraint
            boost::shared_ptr<kinematic_constraints::OrientationConstraint> oc(new kinematic_constraints::OrientationConstraint(scene_->getRobotModel()));
            if (oc->configure(constr.orientation_constraints[o], scene_->getTransforms()))
            {
                constraint_samplers::IKSamplingPose ornConstraint(oc);
                if (oc->mobileReferenceFrame())
                    frame_depends_.push_back(oc->getReferenceFrame());

                sampling_poses_.push_back(ornConstraint);
                fullyConstrainedLinks_.push_back(false);  // there cannot be a position constraint

                std::vector<int> priorities;
                extractConstraintPriorities(boost::shared_ptr<kinematic_constraints::PositionConstraint>(), oc, priorities);
                pose_priorities_.push_back(priorities);
            }
            else
            {
                ROS_ERROR("Failed to configure orientation constraint for '%s'", constr.orientation_constraints[o].link_name.c_str());
                return false;
            }

        }
    }

    is_valid_ = true;

    // Caching all constrained link names for convenience
    // TODO: See about combining this loop with one of the the previous loops
    for(size_t i = 0; i < sampling_poses_.size(); ++i)
    {
        std::string link_name = (sampling_poses_[i].position_constraint_ ?
                                 sampling_poses_[i].position_constraint_->getLinkModel()->getName() :
                                 sampling_poses_[i].orientation_constraint_->getLinkModel()->getName());

        link_names_.push_back(link_name);
    }

    // if more than two links are fully constrained, assume all tips are
    all_tips_constrained_ = numFullyConstrainedLinks_ >= 2;
    ROS_INFO("There are %u fully constrained and %lu partially constrained links", numFullyConstrainedLinks_, link_names_.size() - numFullyConstrainedLinks_);
    for(size_t i = 0; i < link_names_.size(); ++i)
        ROS_INFO("Configured constraint(s) for %s.  Fully constrained? %s", link_names_[i].c_str(), fullyConstrainedLinks_[i] ? "YES":"NO");

    return true;
}

bool moveit_r2_constraints::R2KinematicConstraintSampler::sample(robot_state::RobotState &state,
                                                                 const robot_state::RobotState &reference_state,
                                                                 unsigned int max_attempts)
{
    state.update();

    for (unsigned int a = 0 ; a < max_attempts ; ++a)
    {
        std::vector<geometry_msgs::Pose> poses;
        if (!samplePoses(poses, state, max_attempts))
        {
            ROS_ERROR("Failed to sample satisfying poses");
            return false;
        }

        // If all tips are constrained, we need to handle this case explicitly
        if (all_tips_constrained_)
        {
            if (callIKAllTips(poses, state, reference_state, a > 0))
                return true;
        }
        else
        {
            if (sampleIK(poses, state))
                return true;
        }
    }

    return false;
}

bool moveit_r2_constraints::R2KinematicConstraintSampler::project(robot_state::RobotState &state,
                                                                  unsigned int max_attempts)
{
    // When all tips are constrained, there is no obvious fixed base
    // Just sample a pose
    if (all_tips_constrained_)
        return sample(state, state, max_attempts);

    state.update();

    for (unsigned int a = 0 ; a < max_attempts ; ++a)
    {
        std::vector<geometry_msgs::Pose> poses;
        if (!samplePoses(poses, state, max_attempts))
            return false;

        if (projectIK(poses, state, state))
          return true;
    }

    return false;
}

bool moveit_r2_constraints::R2KinematicConstraintSampler::callIKAllTips(const std::vector<geometry_msgs::Pose>& poses,
                                                                        robot_state::RobotState &state,
                                                                        const robot_state::RobotState &reference_state,
                                                                        bool randomizeWorld) const
{
    const std::string root_joint = "virtual_joint";  // hack

    assert(poses.size() == link_names_.size());

    const moveit_r2_kinematics::R2TreeKinematicsInterface* r2kinematics;
    r2kinematics = static_cast<const moveit_r2_kinematics::MoveItR2TreeKinematicsPlugin*>(kb_.get())->getTreeKinematicsInterface();

    // Move one fully constrained link to its destination
    moveit_r2_kinematics::TreeIkRequest request;
    moveit_r2_kinematics::TreeIkResponse response;

    Eigen::Affine3d base_pose; // debug
    size_t baseIndex = 0; //debug

    bool foundOne = false;
    size_t movingIndex = 0;
    for(size_t i = 0; i < link_names_.size(); ++i)
    {
        if (fullyConstrainedLinks_[i])
        {
            if (!foundOne)
            {
                foundOne = true;
                movingIndex = i;
                request.addLinkPose(link_names_[i], poses[i], pose_priorities_[i]);  // this link is allowed to move
            }
            else
            {
                base_pose = state.getGlobalLinkTransform(link_names_[i]);
                baseIndex = i;

                request.addFixedLink(link_names_[i]);  // not allowed to move, but something needs to be the base
                break;
            }
        }
    }

    // Getting world pose.  Sample position in unit cube, with identity orientation
    Eigen::Affine3d world_pose = reference_state.getGlobalLinkTransform("r2/world_ref");
    if (randomizeWorld)
    {
        world_pose.translation()(0) += 2.0 * (random_number_generator_.uniform01() - 0.5); // random between [-1,1]
        world_pose.translation()(1) += 2.0 * (random_number_generator_.uniform01() - 0.5); // random between [-1,1]
        world_pose.translation()(2) += 2.0 * (random_number_generator_.uniform01() - 0.5); // random between [-1,1]
    }
    request.setWorldState(world_pose);

    // seeding a joint state.  TODO: This is probably slow.  Make faster.
    std::vector<double> joint_values;
    const std::vector<std::string>& all_joints = r2kinematics->getAllJointNames();
    for (size_t i = 0; i < all_joints.size(); ++i)
        joint_values.push_back(reference_state.getVariablePosition(all_joints[i]));
    request.setJointValues(joint_values);

    if (r2kinematics->getPositionIk(request, response))
    {
        // Fill out joint_state
        const std::vector<double>& new_values = response.getJointValues();
        std::map<std::string, double> new_values_map;
        const std::vector<std::string>& joint_order = r2kinematics->getJointNames();
        for (size_t j = 0; j < joint_order.size(); ++j)
            new_values_map[joint_order[j]] = new_values[j];

        // Add the world.   Hard-coded for now.  TODO: Make this less awful
        const Eigen::Affine3d& new_world_pose = response.getWorldState();
        Eigen::Quaternion<double> q(new_world_pose.rotation());

        new_values_map[root_joint + std::string("/trans_x")] = new_world_pose.translation()[0];
        new_values_map[root_joint + std::string("/trans_y")] = new_world_pose.translation()[1];
        new_values_map[root_joint + std::string("/trans_z")] = new_world_pose.translation()[2];
        new_values_map[root_joint + std::string("/rot_x")] = q.x();
        new_values_map[root_joint + std::string("/rot_y")] = q.y();
        new_values_map[root_joint + std::string("/rot_z")] = q.z();
        new_values_map[root_joint + std::string("/rot_w")] = q.w();

        state.setVariablePositions(new_values_map);
        state.update();
    }
    else
    {
        return false;
    }

    // One link is in position.  Now move the rest
    request.clear();
    for(size_t i = 0; i < link_names_.size(); ++i)
    {
        if (i == movingIndex)
            request.addFixedLink(link_names_[i]);
        else
            request.addLinkPose(link_names_[i], poses[i], pose_priorities_[i]);
    }

    // Set world pose as the one we ended up with in the previous call
    world_pose = state.getGlobalLinkTransform("r2/world_ref"); //state.getJointModel(root_joint)->getChildLinkModel()->getName());
    request.setWorldState(world_pose);

    // seeding a joint state
    joint_values.clear();
    // TODO: This is probably slow.  Make faster.
    for (size_t i = 0; i < all_joints.size(); ++i)
        joint_values.push_back(state.getVariablePosition(all_joints[i]));

    request.setJointValues(joint_values);

    if (r2kinematics->getPositionIk(request, response))
    {
        // Fill out joint_state
        const std::vector<double>& new_values = response.getJointValues();
        std::map<std::string, double> new_values_map;
        const std::vector<std::string>& joint_order = r2kinematics->getJointNames();
        for (size_t j = 0; j < joint_order.size(); ++j)
            new_values_map[joint_order[j]] = new_values[j];

        // Add the world.   Hard-coded for now.  TODO: Make this less awful
        // tf::Quaternion doesn't yield correct values... :(  Using Eigen::Quaternion
        const Eigen::Affine3d& new_world_pose = response.getWorldState();
        Eigen::Quaternion<double> q(new_world_pose.rotation());

        new_values_map[root_joint + std::string("/trans_x")] = new_world_pose.translation()[0];
        new_values_map[root_joint + std::string("/trans_y")] = new_world_pose.translation()[1];
        new_values_map[root_joint + std::string("/trans_z")] = new_world_pose.translation()[2];
        new_values_map[root_joint + std::string("/rot_x")] = q.x();
        new_values_map[root_joint + std::string("/rot_y")] = q.y();
        new_values_map[root_joint + std::string("/rot_z")] = q.z();
        new_values_map[root_joint + std::string("/rot_w")] = q.w();

        state.setVariablePositions(new_values_map);
        state.update();
        return true;
    }

    return false;
}

bool moveit_r2_constraints::R2KinematicConstraintSampler::sampleIK(const std::vector<geometry_msgs::Pose>& poses,
                                                                   robot_state::RobotState& state) const
{
    // Generating a random world pose
    // This is just for sampling variability.  IK will inevitably change this.
    Eigen::Affine3d world_pose = Eigen::Affine3d::Identity();
    world_pose.translation()(0) = 2.0 * (random_number_generator_.uniform01() - 0.5); // random between [-1,1]
    world_pose.translation()(1) = 2.0 * (random_number_generator_.uniform01() - 0.5); // random between [-1,1]
    world_pose.translation()(2) = 2.0 * (random_number_generator_.uniform01() - 0.5); // random between [-1,1]

    // sample a rotation matrix
    double angle_x = 2.0 * (random_number_generator_.uniform01() - 0.5) * (3.14159);
    double angle_y = 2.0 * (random_number_generator_.uniform01() - 0.5) * (3.14159);
    double angle_z = 2.0 * (random_number_generator_.uniform01() - 0.5) * (3.14159);
    Eigen::Affine3d rot(Eigen::AngleAxisd(angle_x, Eigen::Vector3d::UnitX())
                      * Eigen::AngleAxisd(angle_y, Eigen::Vector3d::UnitY())
                      * Eigen::AngleAxisd(angle_z, Eigen::Vector3d::UnitZ()));
    world_pose = world_pose * rot;


    // Sampling a random joint state
    const std::vector<unsigned int>& bijection = jmg_->getKinematicsSolverJointBijection();
    std::vector<double> random(bijection.size());
    jmg_->getVariableRandomPositions(random_number_generator_, random);
    state.setJointGroupPositions(jmg_, random);

    return callIK(poses, state, state, world_pose);
}

bool moveit_r2_constraints::R2KinematicConstraintSampler::projectIK(const std::vector<geometry_msgs::Pose>& poses,
                                                                    robot_state::RobotState& state,
                                                                    const robot_state::RobotState &reference_state) const
{
    Eigen::Affine3d world_pose = state.getGlobalLinkTransform(state.getJointModel("virtual_joint")->getChildLinkModel()->getName());
    return callIK(poses, state, reference_state, world_pose);
}

bool moveit_r2_constraints::R2KinematicConstraintSampler::callIK(const std::vector<geometry_msgs::Pose>& poses,
                                                                 robot_state::RobotState &state,
                                                                 const robot_state::RobotState &reference_state,
                                                                 const Eigen::Affine3d& world_pose) const
{
    moveit_r2_kinematics::TreeIkRequest request;
    moveit_r2_kinematics::TreeIkResponse response;

    // Figure out the base frame
    // Totally guessing that only one of the gripper tips is constrained
    std::vector<std::string> base_guesses;
    base_guesses.push_back("r2/left_leg/gripper/tip");
    base_guesses.push_back("r2/right_leg/gripper/tip");
    std::vector<bool> base_guesses_constrained(false, base_guesses.size());

    bool all_constrained = true;
    for(size_t i = 0; i < base_guesses.size(); ++i)
    {
        for(size_t j = 0; j < link_names_.size(); ++j)
        {
            if (base_guesses[i] == link_names_[j])
            {
                base_guesses_constrained[i] = true;
                break;
            }
        }

        all_constrained &= base_guesses_constrained[i];
    }

    if (all_constrained)
    {
        ROS_ERROR("R2KinematicConstraintSampler::callIK - No base link guess is unconstrained");
        return false;
    }

    for(size_t i = 0; i < base_guesses.size(); ++i)
    {
        if (!base_guesses_constrained[i])
            request.addFixedLink(base_guesses[i]);
    }

    // Setting all other tasks
    for(size_t i = 0; i < poses.size(); ++i)
        request.addLinkPose(link_names_[i], poses[i], pose_priorities_[i]);

    assert(request.getFixedLinks().size() > 0);
    assert(request.getMovingLinks().size() > 0);

    request.setWorldState(world_pose);

    // seeding a joint state
    std::vector<double> joint_values;

    const moveit_r2_kinematics::R2TreeKinematicsInterface* r2kinematics;
    r2kinematics = static_cast<const moveit_r2_kinematics::MoveItR2TreeKinematicsPlugin*>(kb_.get())->getTreeKinematicsInterface();

    const std::vector<std::string>& all_joints = r2kinematics->getAllJointNames();
    for (size_t i = 0; i < all_joints.size(); ++i)
        joint_values.push_back(reference_state.getVariablePosition(all_joints[i]));

    request.setJointValues(joint_values);

    if (r2kinematics->getPositionIk(request, response))
    {
        // Fill out joint_state
        const std::vector<double>& new_values = response.getJointValues();
        std::map<std::string, double> new_values_map;
        const std::vector<std::string>& joint_order = r2kinematics->getJointNames();
        for (size_t j = 0; j < joint_order.size(); ++j)
            new_values_map[joint_order[j]] = new_values[j];

        // Add the world.   Hard-coded for now.  TODO: Make this less awful
        const Eigen::Affine3d& new_world_pose = response.getWorldState();
        Eigen::Quaternion<double> q(new_world_pose.rotation());

        new_values_map["virtual_joint" + std::string("/trans_x")] = new_world_pose.translation()[0];
        new_values_map["virtual_joint" + std::string("/trans_y")] = new_world_pose.translation()[1];
        new_values_map["virtual_joint" + std::string("/trans_z")] = new_world_pose.translation()[2];
        new_values_map["virtual_joint" + std::string("/rot_x")] = q.x();
        new_values_map["virtual_joint" + std::string("/rot_y")] = q.y();
        new_values_map["virtual_joint" + std::string("/rot_z")] = q.z();
        new_values_map["virtual_joint" + std::string("/rot_w")] = q.w();

        state.setVariablePositions(new_values_map);
        state.update();
        return true;
    }
    return false;
}

const std::string& moveit_r2_constraints::R2KinematicConstraintSampler::getName() const
{
    static const std::string SAMPLER_NAME = "R2KinematicConstraintSampler";
    return SAMPLER_NAME;
}

void moveit_r2_constraints::R2KinematicConstraintSampler::clear()
{
    ConstraintSampler::clear();
    kb_.reset();
    is_valid_ = false;
    //tip_pose_bijection_.clear();
    link_names_.clear();
    all_tips_constrained_ = false;
}

bool moveit_r2_constraints::R2KinematicConstraintSampler::loadIKSolver()
{
    kb_ = jmg_->getSolverInstance();
    if (!kb_)
    {
        logError("No IK solver to load");
        return false;
    }

    ik_timeout_ = jmg_->getDefaultIKTimeout();

    // check if we need to transform the request into the coordinate frame expected by IK
    ik_frame_ = kb_->getBaseFrame();
    transform_ik_ = !robot_state::Transforms::sameFrame(ik_frame_, jmg_->getParentModel().getModelFrame());
    if (!ik_frame_.empty() && ik_frame_[0] == '/')
        ik_frame_.erase(ik_frame_.begin());

    if (transform_ik_ && !jmg_->getParentModel().hasLinkModel(ik_frame_))
    {
        logError("The IK solver expects requests in frame '%s' but this frame is not known to the sampler. Ignoring transformation (IK may fail)", ik_frame_.c_str());
        transform_ik_ = false;
    }

    return true;
}

bool moveit_r2_constraints::R2KinematicConstraintSampler::samplePose(const constraint_samplers::IKSamplingPose& sampling_pose, const std::vector<int>& priorities,
                                                                     geometry_msgs::Pose& pose, const robot_state::RobotState &ks, unsigned int max_attempts) const
{
    Eigen::Vector3d pos;
    Eigen::Quaterniond quat;

    // Position constraint
    // TODO: This logic is flawed.  We may sample points near the edge of the bounding volume and the kinematics
    // solver may find a solution outside the bounding volume due to the numerical tolerances of the underlying IK implementation.
    // To correct this, we need to pad the point we sample so that the point we sample + IK solver tolerance can never leave the
    // bounding volume.  Something similar is done below for orientation constraints, where the issue is much more prevalent.
    if (sampling_pose.position_constraint_)
    {
        const std::vector<bodies::BodyPtr> &b = sampling_pose.position_constraint_->getConstraintRegions();
        if (!b.empty())
        {
            bool found = false;
            std::size_t k = random_number_generator_.uniformInteger(0, b.size() - 1);
            for (std::size_t i = 0 ; i < b.size() ; ++i)
                if (b[(i+k) % b.size()]->samplePointInside(random_number_generator_, max_attempts, pos))
                {
                    found = true;
                    break;
                }
            if (!found)
            {
                logError("Unable to sample a point inside the constraint region");
                return false;
            }
        }
        else
        {
            logError("Unable to sample a point inside the constraint region. Constraint region is empty when it should not be.");
            return false;
        }

        // if this constraint is with respect a mobile frame, we need to convert this rotation to the root frame of the model
        if (sampling_pose.position_constraint_->mobileReferenceFrame())
            pos = ks.getFrameTransform(sampling_pose.position_constraint_->getReferenceFrame()) * pos;
    }
    else
    {
        // do FK for rand state
        robot_state::RobotState tempState(ks);
        tempState.setToRandomPositions(jmg_);
        pos = tempState.getGlobalLinkTransform(sampling_pose.orientation_constraint_->getLinkModel()).translation();
    }

    // Orientation constraint
    if (sampling_pose.orientation_constraint_)
    {
        // sample a rotation matrix within the allowed bounds
        // Note: subtracting the priority tolerance since the IK solver can only guarantee solutions within a tolerance.
        // Otherwise, we may unintentionally exceed the configured contraint tolerance by sampling near the tolerance boundary.
        double angle_x = 2.0 * (random_number_generator_.uniform01() - 0.50) * (sampling_pose.orientation_constraint_->getXAxisTolerance() - getPriorityTolerance(priorities[3], false));
        double angle_y = 2.0 * (random_number_generator_.uniform01() - 0.50) * (sampling_pose.orientation_constraint_->getYAxisTolerance() - getPriorityTolerance(priorities[4], false));
        double angle_z = 2.0 * (random_number_generator_.uniform01() - 0.50) * (sampling_pose.orientation_constraint_->getZAxisTolerance() - getPriorityTolerance(priorities[5], false));
        Eigen::Affine3d diff(Eigen::AngleAxisd(angle_x, Eigen::Vector3d::UnitX())
                           * Eigen::AngleAxisd(angle_y, Eigen::Vector3d::UnitY())
                           * Eigen::AngleAxisd(angle_z, Eigen::Vector3d::UnitZ()));
        Eigen::Affine3d reqr(sampling_pose.orientation_constraint_->getDesiredRotationMatrix() * diff.rotation());
        quat = Eigen::Quaterniond(reqr.rotation());

        // if this constraint is with respect a mobile frame, we need to convert this rotation to the root frame of the model
        if (sampling_pose.orientation_constraint_->mobileReferenceFrame())
        {
            const Eigen::Affine3d &t = ks.getFrameTransform(sampling_pose.orientation_constraint_->getReferenceFrame());
            Eigen::Affine3d rt(t.rotation() * quat.toRotationMatrix());
            quat = Eigen::Quaterniond(rt.rotation());
        }
    }
    else
    {
        // sample a random orientation
        double q[4];
        random_number_generator_.quaternion(q);
        quat = Eigen::Quaterniond(q[3], q[0], q[1], q[2]);
    }

    // if there is an offset, we need to undo the induced rotation in the sampled transform origin (point)
    if (sampling_pose.position_constraint_ && sampling_pose.position_constraint_->hasLinkOffset())
        // the rotation matrix that corresponds to the desired orientation
        pos = pos - quat.toRotationMatrix() * sampling_pose.position_constraint_->getLinkOffset();

    if (transform_ik_)
    {
        // we need to convert this transform to the frame expected by the IK solver
        // both the planning frame and the frame for the IK are assumed to be robot links
        Eigen::Affine3d ikq(Eigen::Translation3d(pos) * quat.toRotationMatrix());
        ikq = ks.getFrameTransform(ik_frame_).inverse() * ikq;
        pos = ikq.translation();
        quat = Eigen::Quaterniond(ikq.rotation());
    }

    pose.position.x = pos(0);
    pose.position.y = pos(1);
    pose.position.z = pos(2);
    pose.orientation.x = quat.x();
    pose.orientation.y = quat.y();
    pose.orientation.z = quat.z();
    pose.orientation.w = quat.w();
    return true;
}

bool moveit_r2_constraints::R2KinematicConstraintSampler::samplePoses(/*Eigen::Vector3d &pos, Eigen::Quaterniond &quat,*/ std::vector<geometry_msgs::Pose>& poses,
                                        const robot_state::RobotState &ks, unsigned int max_attempts) const
{
    // Sampling poses in constraint regions
    poses.resize(sampling_poses_.size());
    for (size_t i = 0; i < sampling_poses_.size(); ++i)
    {
        if (!samplePose(sampling_poses_[i], pose_priorities_[i], poses[i], ks, max_attempts))
            return false;
    }

    return true;
}

bool moveit_r2_constraints::R2KinematicConstraintSampler::validLink(const std::string& link_name) const
{
    // The R2TreeKinematics class can perform IK for any link.
    // TODO: Make sure the link exists and is in the configured group
    return true;
}

CLASS_LOADER_REGISTER_CLASS(moveit_r2_constraints::MoveItR2ConstraintSamplerAllocator, constraint_samplers::ConstraintSamplerAllocator)