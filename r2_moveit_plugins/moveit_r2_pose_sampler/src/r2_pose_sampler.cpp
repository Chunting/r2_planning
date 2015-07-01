/* Author: Ryan Luna */

#include <ros/ros.h>
#include "moveit_r2_constraints/r2_pose_sampler.h"
#include "moveit_r2_kinematics/tree_kinematics_tolerances.h"

// For plugin library
#include <class_loader/class_loader.h>
#include <eigen_conversions/eigen_msg.h>

moveit_r2_constraints::MoveItR2PoseSamplerAllocator::MoveItR2PoseSamplerAllocator() : constraint_samplers::ConstraintSamplerAllocator()
{
}

moveit_r2_constraints::MoveItR2PoseSamplerAllocator::~MoveItR2PoseSamplerAllocator()
{
}

constraint_samplers::ConstraintSamplerPtr moveit_r2_constraints::MoveItR2PoseSamplerAllocator::alloc(const planning_scene::PlanningSceneConstPtr &scene,
                                                               const std::string &group_name,
                                                               const moveit_msgs::Constraints &constr)
{
    constraint_samplers::ConstraintSamplerPtr cs(new R2PoseSampler(scene, group_name));
    cs->configure(constr);
    return cs;
}

bool moveit_r2_constraints::MoveItR2PoseSamplerAllocator::canService(const planning_scene::PlanningSceneConstPtr &scene,
                                                                     const std::string &group_name,
                                                                     const moveit_msgs::Constraints &constr) const
{
    if (group_name != "legs") // ONLY LEGS!
    {
        //ROS_ERROR("Group '%s' is not allowed", group_name.c_str());
        return false;
    }

    // ONLY POSITION AND ORIENTATION CONSTRAINTS
    if (constr.joint_constraints.size() > 0 || constr.visibility_constraints.size() > 0)
    {
        //ROS_ERROR("Can only service position and orientation constraints");
        return false;
    }

    // Only service position and orientation constraint on the same link
    if (constr.position_constraints.size() == 1 && constr.orientation_constraints.size() == 1)
    {
        if (constr.position_constraints[0].link_name == constr.orientation_constraints[0].link_name)
            return true;
    }

    return false;
}

moveit_r2_constraints::R2PoseSampler::R2PoseSampler(const planning_scene::PlanningSceneConstPtr &scene, const std::string &group_name)
    : constraint_samplers::ConstraintSampler(scene, group_name)
{
}

bool moveit_r2_constraints::R2PoseSampler::configure(const moveit_msgs::Constraints &constr)
{
    clear();

    // There must be at least one position or one orientation constraint
    if (constr.position_constraints.size() == 0 && constr.orientation_constraints.size() == 0)
    {
        logError("No position or orientation constraint to configure");
        return false;
    }

    if (constr.position_constraints.size() > 1 || constr.orientation_constraints.size() > 1)
    {
        logError("Expected at most one position and one orientation constraint");
        return false;
    }
    else // there is at most one position constraint and at most one orientation constraint
    {
        if (constr.position_constraints.size() == 1 && constr.orientation_constraints.size() == 1)
        {
            if (validLink(constr.position_constraints[0].link_name))
            {
                if (constr.position_constraints[0].link_name != constr.orientation_constraints[0].link_name)
                {
                    logError("Position and orientation constraint do not constrain the same link");
                    return false;
                }
            }
            else return false;
        }
    }

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
                    configured = true;
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
                configured = true;
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
            }
            else
            {
                ROS_ERROR("Failed to configure orientation constraint for '%s'", constr.orientation_constraints[o].link_name.c_str());
                return false;
            }

        }
    }

    if (sampling_poses_.size() != 1)
    {
        ROS_ERROR("Expected only one sampling pose to configure");
        return false;
    }

    is_valid_ = true;

    // Caching all constrained link names for convenience
    for(size_t i = 0; i < sampling_poses_.size(); ++i)
    {
        std::string link_name = (sampling_poses_[i].position_constraint_ ?
                                 sampling_poses_[i].position_constraint_->getLinkModel()->getName() :
                                 sampling_poses_[i].orientation_constraint_->getLinkModel()->getName());

        link_names_.push_back(link_name);
    }

    return true;
}

bool moveit_r2_constraints::R2PoseSampler::sample(robot_state::RobotState &state,
                                                  const robot_state::RobotState &reference_state,
                                                  unsigned int max_attempts)
{
    state.setToRandomPositions(jmg_);
    state.update(true);
    return project(state, max_attempts);
}

bool moveit_r2_constraints::R2PoseSampler::project(robot_state::RobotState &state,
                                                   unsigned int max_attempts)
{
    geometry_msgs::Pose pose;
    samplePose(sampling_poses_[0], pose, state, max_attempts);
    Eigen::Affine3d pose_eigen;
    tf::poseMsgToEigen(pose, pose_eigen);

    shiftStateByError(state, state.getRobotModel()->getRootJoint(), link_names_[0], pose_eigen);
    return true;
}

const std::string& moveit_r2_constraints::R2PoseSampler::getName() const
{
    static const std::string SAMPLER_NAME = "R2PoseSampler";
    return SAMPLER_NAME;
}

void moveit_r2_constraints::R2PoseSampler::clear()
{
    ConstraintSampler::clear();
    is_valid_ = false;
    link_names_.clear();
}

void moveit_r2_constraints::R2PoseSampler::shiftStateByError(robot_state::RobotState& state, const robot_state::JointModel* base_joint,
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
    //state.update(true);
}

bool moveit_r2_constraints::R2PoseSampler::samplePose(const constraint_samplers::IKSamplingPose& sampling_pose, geometry_msgs::Pose& pose,
                                                      const robot_state::RobotState &ks, unsigned int max_attempts) const
{
    Eigen::Vector3d pos;
    Eigen::Quaterniond quat;

    std::vector<int> priorities(6, KdlTreeIk::CRITICAL);

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
        double angle_x = 2.0 * (random_number_generator_.uniform01() - 0.50) * (sampling_pose.orientation_constraint_->getXAxisTolerance() - moveit_r2_kinematics::CRITICAL_PRIO_ANGULAR_TOL);
        double angle_y = 2.0 * (random_number_generator_.uniform01() - 0.50) * (sampling_pose.orientation_constraint_->getYAxisTolerance() - moveit_r2_kinematics::CRITICAL_PRIO_ANGULAR_TOL);
        double angle_z = 2.0 * (random_number_generator_.uniform01() - 0.50) * (sampling_pose.orientation_constraint_->getZAxisTolerance() - moveit_r2_kinematics::CRITICAL_PRIO_ANGULAR_TOL);
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

    /*if (transform_ik_)
    {
        // we need to convert this transform to the frame expected by the IK solver
        // both the planning frame and the frame for the IK are assumed to be robot links
        Eigen::Affine3d ikq(Eigen::Translation3d(pos) * quat.toRotationMatrix());
        ikq = ks.getFrameTransform(ik_frame_).inverse() * ikq;
        pos = ikq.translation();
        quat = Eigen::Quaterniond(ikq.rotation());
    }*/

    pose.position.x = pos(0);
    pose.position.y = pos(1);
    pose.position.z = pos(2);
    pose.orientation.x = quat.x();
    pose.orientation.y = quat.y();
    pose.orientation.z = quat.z();
    pose.orientation.w = quat.w();
    return true;
}

bool moveit_r2_constraints::R2PoseSampler::validLink(const std::string& link_name) const
{
    // The R2TreeKinematics class can perform IK for any link.
    // TODO: Make sure the link exists
    return true;
}

CLASS_LOADER_REGISTER_CLASS(moveit_r2_constraints::MoveItR2PoseSamplerAllocator, constraint_samplers::ConstraintSamplerAllocator)