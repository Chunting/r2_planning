/* Author: Ryan Luna */

#include <ros/ros.h>
#include "moveit_r2_constraints/r2_constraint_sampler.h"
#include "moveit_r2_kinematics/moveit_r2_tree_kinematics.h"
#include "moveit_r2_kinematics/tree_kinematics_tolerances.h"
#include <nasa_robodyn_controllers_core/KdlTreeIk.h> // for priority values

// For plugin library
#include <class_loader/class_loader.h>
#include <eigen_conversions/eigen_msg.h>


/// A mapping of the IK solver priority levels to/from the tolerances specified by the
/// user when specifying the constraints
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

///////////////////////////////////////////////////////////////////////////////////
////                   MoveItR2ConstraintSamplerAllocator                      ////
///////////////////////////////////////////////////////////////////////////////////

moveit_r2_constraints::MoveItR2ConstraintSamplerAllocator::MoveItR2ConstraintSamplerAllocator()
    : constraint_samplers::ConstraintSamplerAllocator()
{
}

moveit_r2_constraints::MoveItR2ConstraintSamplerAllocator::~MoveItR2ConstraintSamplerAllocator()
{
}

constraint_samplers::ConstraintSamplerPtr
moveit_r2_constraints::MoveItR2ConstraintSamplerAllocator::alloc(const planning_scene::PlanningSceneConstPtr &scene,
                                                                 const std::string &group_name,
                                                                 const moveit_msgs::Constraints &constr)
{
    constraint_samplers::ConstraintSamplerPtr cs(new R2KinematicConstraintSampler(scene, group_name));
    cs->configure(constr);
    return cs;
}

bool
moveit_r2_constraints::MoveItR2ConstraintSamplerAllocator::canService(const planning_scene::PlanningSceneConstPtr &scene,
                                                                      const std::string &group_name,
                                                                      const moveit_msgs::Constraints &constr) const
{
    // Any group that starts with 'legs' is good.
    // TODO: Probably should not discriminate based on group name.
    std::size_t idx = group_name.find("legs");
    if (idx != 0)
    {
        ROS_ERROR("MoveItR2ConstraintSamplerAllocator cannot service group '%s'", group_name.c_str());
        return false;
    }

    // ONLY POSITION AND ORIENTATION CONSTRAINTS
    if (constr.joint_constraints.size() > 0 || constr.visibility_constraints.size() > 0)
    {
        //ROS_ERROR("MoveItR2ConstraintSamplerAllocator: Can only service position and orientation constraints");
        return false;
    }

    // At least one position or orientation constraint
    if (constr.position_constraints.size() == 0 && constr.orientation_constraints.size() == 0)
    {
        //ROS_ERROR("MoveItR2ConstraintSamplerAllocator: Can only service position and orientation constraints");
        return false;
    }

    // sgood.
    return true;
}


///////////////////////////////////////////////////////////////////////////////////
////                       MoveItR2ConstraintSampler                           ////
///////////////////////////////////////////////////////////////////////////////////

moveit_r2_constraints::R2KinematicConstraintSampler::R2KinematicConstraintSampler(const planning_scene::PlanningSceneConstPtr &scene,
                                                                                  const std::string &group_name)
    : constraint_samplers::ConstraintSampler(scene, group_name)
{
}

bool moveit_r2_constraints::R2KinematicConstraintSampler::configure(const moveit_msgs::Constraints &constr)
{
    clear();
    if (!loadIKSolver())
        return false;

    // There must be at least one position or one orientation constraint
    if (constr.position_constraints.size() == 0 && constr.orientation_constraints.size() == 0)
    {
        ROS_ERROR("R2KinematicConstraintSampler: No position or orientation constraint to configure");
        return false;
    }

    // Check all of the links to configure.  Make sure they are in the model, and
    // are only constrained once by the same kind of constraint
    bool valid_configuration = true;
    for (std::size_t p = 0; p < constr.position_constraints.size(); ++p)
    {
        bool configured = false;
        bool skip = false;

        if (!validLink(constr.position_constraints[p].link_name))
        {
            ROS_ERROR("R2KinematicConstraintSampler: IK cannot be performed for link '%s'", constr.position_constraints[p].link_name.c_str());
            valid_configuration = false;
            continue;  // skip this constraint
        }

        // Make sure this link has only one position constraint
        for (std::size_t q = p + 1 ; q < constr.position_constraints.size() ; ++q)
            if (constr.position_constraints[p].link_name == constr.position_constraints[q].link_name)
            {
                ROS_WARN("R2KinematicConstraintSampler: Link '%s' has more than one position constraint defined", constr.position_constraints[p].link_name.c_str());
                // don't mark all constraints as invalid.  Just ignore subsequent constraints
                skip = true;
                break;
            }

        if (skip) continue;  // go to next constraint

        // See if there is an orientation constraint for the same link
        for (std::size_t o = 0; o < constr.orientation_constraints.size(); ++o)
        {
            if (constr.position_constraints[p].link_name == constr.orientation_constraints[o].link_name)
            {
                // Make sure this link has only one orientation constraint
                for (std::size_t q = o + 1; q < constr.orientation_constraints.size(); ++q)
                    if (constr.orientation_constraints[o].link_name == constr.orientation_constraints[q].link_name)
                    {
                        ROS_WARN("R2KinematicConstraintSampler: Link '%s' has more than one orientation constraint defined", constr.orientation_constraints[o].link_name.c_str());
                        // don't mark all constraints as invalid.  Just ignore subsequent constraints for this link
                        skip = true;
                        break;
                    }

                if (skip) break;

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

                    link_names_.push_back(constr.position_constraints[p].link_name);
                    sampling_poses_.push_back(poseConstraint);

                    std::vector<int> priorities;
                    extractConstraintPriorities(pc, oc, priorities);
                    pose_priorities_.push_back(priorities);

                    configured = true;
                    fullyConstrainedLinks_.push_back((int) link_names_.size()-1);
                }
                else
                {
                    ROS_ERROR("R2KinematicConstraintSampler: Failed to configure position and orientation constraint for '%s'", constr.position_constraints[p].link_name.c_str());
                    valid_configuration = false;
                }
            }
        }

        if (skip) continue;  // go to next constraint

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

                link_names_.push_back(constr.position_constraints[p].link_name);
                sampling_poses_.push_back(posConstraint);

                std::vector<int> priorities;
                extractConstraintPriorities(pc, boost::shared_ptr<kinematic_constraints::OrientationConstraint>(), priorities);
                pose_priorities_.push_back(priorities);

                configured = true;
            }
            else
            {
                ROS_ERROR("R2KinematicConstraintSampler: Failed to configure position constraint for '%s'", constr.position_constraints[p].link_name.c_str());
                valid_configuration = false;
            }
        }
    }

    // Now, need to check for singular orientation constraints
    for (std::size_t o = 0 ; o < constr.orientation_constraints.size() ; ++o)
    {
        bool skip = false;
        bool hasPosition = false;
        // See if there is NO position constraint for this link
        for (std::size_t p = 0 ; p < constr.position_constraints.size() && !hasPosition; ++p)
            hasPosition = (constr.position_constraints[p].link_name == constr.orientation_constraints[o].link_name);

        if (!hasPosition)
        {
            if (!validLink(constr.orientation_constraints[o].link_name))
            {
                ROS_ERROR("R2KinematicConstraintSampler: IK cannot be performed for link '%s'", constr.orientation_constraints[o].link_name.c_str());
                valid_configuration = false;
                continue;
            }

            // Make sure this link is constrained just once
            for (std::size_t q = o + 1 ; q < constr.orientation_constraints.size() ; ++q)
                if (constr.orientation_constraints[o].link_name == constr.orientation_constraints[q].link_name)
                {
                    ROS_WARN("R2KinematicConstraintSampler: Link '%s' has more than one orientation constraint defined", constr.orientation_constraints[o].link_name.c_str());
                    skip = true;
                }

            if (skip) continue;

            // Configure the sampling pose for the orientation constraint
            boost::shared_ptr<kinematic_constraints::OrientationConstraint> oc(new kinematic_constraints::OrientationConstraint(scene_->getRobotModel()));
            if (oc->configure(constr.orientation_constraints[o], scene_->getTransforms()))
            {
                constraint_samplers::IKSamplingPose ornConstraint(oc);
                if (oc->mobileReferenceFrame())
                    frame_depends_.push_back(oc->getReferenceFrame());

                link_names_.push_back(constr.orientation_constraints[o].link_name);
                sampling_poses_.push_back(ornConstraint);

                std::vector<int> priorities;
                extractConstraintPriorities(boost::shared_ptr<kinematic_constraints::PositionConstraint>(), oc, priorities);
                pose_priorities_.push_back(priorities);
            }
            else
            {
                ROS_ERROR("R2KinematicConstraintSampler: Failed to configure orientation constraint for '%s'", constr.orientation_constraints[o].link_name.c_str());
                valid_configuration = false;
            }

        }
    }

    if (fullyConstrainedLinks_.size() == 0)
        ROS_WARN("R2KinematicConstraintSampler: There are no fully constrained links.  Base frame is ambiguous");

    ROS_INFO("There are %lu fully constrained and %lu partially constrained links", fullyConstrainedLinks_.size(), link_names_.size() - fullyConstrainedLinks_.size());
    size_t fcIdx = 0;
    for(size_t i = 0; i < link_names_.size(); ++i)
    {
        bool fc = fullyConstrainedLinks_[fcIdx] = i;
        if (fc)
            fcIdx++;
        ROS_INFO("Configured constraint(s) for %s.  Fully constrained? %s", link_names_[i].c_str(), fc ? "YES" : "NO");
    }

    return valid_configuration;
}

bool moveit_r2_constraints::R2KinematicConstraintSampler::sample(robot_state::RobotState &state,
                                                                 const robot_state::RobotState &/*reference_state*/,
                                                                 unsigned int max_attempts)
{
    for (unsigned int a = 0 ; a < max_attempts ; ++a)
    {
        state.setToRandomPositions(jmg_, random_number_generator_);  // important, since we are sampling randomly
        state.update();

        std::vector<geometry_msgs::Pose> poses;
        if (!samplePoses(poses, state, max_attempts))
        {
            ROS_ERROR("R2KinematicConstraintSampler: Failed to sample satisfying poses");
            continue;
        }

        if (callIK(state, state, poses))
            return true;
    }

    return false;
}

bool moveit_r2_constraints::R2KinematicConstraintSampler::callIK(robot_state::RobotState &state,
                                                                 const robot_state::RobotState& ref_state,
                                                                 const std::vector<geometry_msgs::Pose>& poses) const
{
    if (fullyConstrainedLinks_.size() == 0)
    {
        ROS_ERROR("R2KinematicConstraintSampler: No base frame specified");
        return false;
    }

    // Extracting pointer to R2-IK solver
    const moveit_r2_kinematics::R2TreeKinematicsInterface* r2kinematics;
    r2kinematics = static_cast<const moveit_r2_kinematics::MoveItR2TreeKinematicsPlugin*>(kb_.get())->getTreeKinematicsInterface();

    // Step 1: Pick one fully constrained link and update the body pose in the seed
    // to place the link at exactly the correct position and orientation
    // Pick the base link at random
    size_t placed_idx = random_number_generator_.uniformInteger(0, fullyConstrainedLinks_.size()-1);

    std::string base_link = link_names_[placed_idx];
    Eigen::Affine3d base_pose;
    tf::poseMsgToEigen(poses[placed_idx], base_pose);

    // Update the body pose to put base_link at poses[placed_idx]
    shiftStateByError(state, state.getRobotModel()->getRootJoint(), link_names_[placed_idx], base_pose);

    if (link_names_.size() == 1) // done.  We fixed the only constraint
        return true;

    // Step 2: Use IK solver to achieve the remaining poses
    moveit_r2_kinematics::TreeIkRequest request;
    moveit_r2_kinematics::TreeIkResponse response;

    request.addFixedLink(base_link); // base frame
    request.setWorldState(state.getGlobalLinkTransform("r2/robot_world"));  // body pose

    // std::cout << "R2ConstraintSampler - Calling IK Solver" << std::endl;
    // std::cout << "  [Base frame]: " << base_link << std::endl;

    // Desired poses for remaining constrained links
    for(size_t i = 0; i < poses.size(); ++i)
    {
        if (i == placed_idx) continue;  // this is the base frame
        request.addLinkPose(link_names_[i], poses[i], pose_priorities_[i]);

        // std::cout << "  [" << link_names_[i] << "] moving to (" << poses[i].position.x << " " << poses[i].position.y << " " << poses[i].position.z << ") with priorities ";
        // for(size_t j = 0; j < pose_priorities_[i].size(); ++j)
        //     std::cout << pose_priorities_[i][j] << " ";
        // std::cout << std::endl;
    }

    // seeding a joint state
    std::vector<double> joint_values;
    const std::vector<std::string>& all_joints = r2kinematics->getAllJointNames();
    for (size_t i = 0; i < all_joints.size(); ++i)
        joint_values.push_back(ref_state.getVariablePosition(all_joints[i]));
    request.setJointValues(joint_values);

    // Calling IK solver
    if (r2kinematics->getPositionIk(request, response))
    {
        //std::cout << "IK WIN" << std::endl << std::endl;

        // Fill out joint_state
        const std::vector<double>& new_values = response.getJointValues();
        std::map<std::string, double> new_values_map;
        const std::vector<std::string>& joint_order = r2kinematics->getJointNames();
        for (size_t j = 0; j < joint_order.size(); ++j)
            new_values_map[joint_order[j]] = new_values[j];


        // Update body pose (virtual joint)
        const Eigen::Affine3d& new_world_pose = response.getWorldState();
        Eigen::Quaternion<double> q(new_world_pose.rotation());

        const std::string root_joint = "virtual_joint";  // hack

        new_values_map[root_joint + std::string("/trans_x")] = new_world_pose.translation()[0];
        new_values_map[root_joint + std::string("/trans_y")] = new_world_pose.translation()[1];
        new_values_map[root_joint + std::string("/trans_z")] = new_world_pose.translation()[2];
        new_values_map[root_joint + std::string("/rot_x")] = q.x();
        new_values_map[root_joint + std::string("/rot_y")] = q.y();
        new_values_map[root_joint + std::string("/rot_z")] = q.z();
        new_values_map[root_joint + std::string("/rot_w")] = q.w();

        // Set all variables and return
        state.setVariablePositions(new_values_map);
        state.update();
        return true;
    }

    //std::cout << "IK FAILED" << std::endl << std::endl;
    return false;
}

// Update the given state to place base_joint whose child is link at the desired pose
// by moving the body pose.
void moveit_r2_constraints::R2KinematicConstraintSampler::shiftStateByError(robot_state::RobotState& state, const robot_state::JointModel* base_joint,
                                                                            const std::string& link, const Eigen::Affine3d& desired_pose) const
{
    if (base_joint->getType() != robot_state::JointModel::FLOATING)
    {
        ROS_ERROR("%s: Expected root joint '%s' to be floating.  This joint is %s",
                  __FUNCTION__, base_joint->getName().c_str(), base_joint->getTypeName().c_str());
        return;
    }

    Eigen::Affine3d actual_pose = state.getGlobalLinkTransform(link);

    // This is the error in the pose
    Eigen::Affine3d diff(desired_pose * actual_pose.inverse());

    // Apply the difference to the (unconstrained) base frame
    // VERY important that diff is multiplied on left side
    Eigen::Affine3d new_base_frame = diff * state.getGlobalLinkTransform(base_joint->getChildLinkModel()->getName());
    Eigen::Quaterniond q(new_base_frame.rotation());

    // Variable order is hard-coded for efficiency
    // This assumes a fixed variable order that "could" change in a future version of MoveIt
    std::vector<double> new_variables(7, 0.0);
    new_variables[0] = new_base_frame.translation()(0); // base_joint_model->getName() + "/trans_x"
    new_variables[1] = new_base_frame.translation()(1); // base_joint_model->getName() + "/trans_y"
    new_variables[2] = new_base_frame.translation()(2); // base_joint_model->getName() + "/trans_z"
    new_variables[3] = q.x();                           // base_joint_model->getName() + "/rot_x"
    new_variables[4] = q.y();                           // base_joint_model->getName() + "/rot_y"
    new_variables[5] = q.z();                           // base_joint_model->getName() + "/rot_z"
    new_variables[6] = q.w();                           // base_joint_model->getName() + "/rot_w"

    // Setting new base position
    state.setJointPositions(base_joint, new_variables);
    state.update();
}

bool moveit_r2_constraints::R2KinematicConstraintSampler::project(robot_state::RobotState &state,
                                                                  unsigned int max_attempts)
{
    state.update();

    for (unsigned int a = 0 ; a < max_attempts ; ++a)
    {
        std::vector<geometry_msgs::Pose> poses;
        if (!projectPoses(poses, state, max_attempts))
        {
            ROS_ERROR("R2KinematicConstraintSampler: Failed to project satisfying poses");
            continue;
        }

        if (callIK(state, state, poses))
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

    sampling_poses_.clear();
    link_names_.clear();
    pose_priorities_.clear();

    numFullyConstrainedLinks_ = 0;
    fullyConstrainedLinks_.clear();
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

bool
moveit_r2_constraints::R2KinematicConstraintSampler::projectPose(const constraint_samplers::IKSamplingPose& sampling_pose,
                                                                 const std::vector<int>& priorities,
                                                                 geometry_msgs::Pose& pose,
                                                                 const robot_state::RobotState &ks,
                                                                 unsigned int max_attempts) const
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
        // Use wherever the link is now (the position is unconstrained, so IK will do the right thing)
        pos = ks.getGlobalLinkTransform(sampling_pose.orientation_constraint_->getLinkModel()).translation();
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
        // Use the current link orientation.  (the orientation is unconstrained, so IK will do the right thing)
        quat = Eigen::Quaterniond(ks.getGlobalLinkTransform(sampling_pose.orientation_constraint_->getLinkModel()).rotation());
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

bool
moveit_r2_constraints::R2KinematicConstraintSampler::samplePoses(std::vector<geometry_msgs::Pose>& poses,
                                                                 const robot_state::RobotState &ks,
                                                                 unsigned int max_attempts) const
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

bool
moveit_r2_constraints::R2KinematicConstraintSampler::projectPoses(std::vector<geometry_msgs::Pose>& poses,
                                                                  const robot_state::RobotState &ks,
                                                                  unsigned int max_attempts) const
{
    // Sampling poses in constraint regions
    poses.resize(sampling_poses_.size());
    for (size_t i = 0; i < sampling_poses_.size(); ++i)
    {
        if (!projectPose(sampling_poses_[i], pose_priorities_[i], poses[i], ks, max_attempts))
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