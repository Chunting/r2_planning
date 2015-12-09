#include "moveit_ompl_components/CartesianSpaceInterpolator.h"
#include <ros/ros.h>

namespace ompl_interface
{

CartesianSpaceInterpolator::CartesianSpaceInterpolator(const robot_model::JointModelGroup* group)
{
    group_ = group;

    const std::vector<const robot_model::JointModel*>& joints = group_->getJointModels();
    ignore_.resize(joints.size(), false);

    // ignore all non-revolute joints
    for(size_t i = 0; i < joints.size(); ++i)
        if (joints[i]->getType() != moveit::core::JointModel::REVOLUTE)
            ignore_[i] = true;
}

void CartesianSpaceInterpolator::ignoreJoint(const std::string& joint_name)
{
    const std::vector<const robot_model::JointModel*>& joints = group_->getJointModels();
    for(size_t i = 0; i < joints.size(); ++i)
    {
        if(joints[i]->getName() == joint_name)
        {
            ignore_[i] = true;
            break;
        }
    }
}

void CartesianSpaceInterpolator::interpolate(const robot_state::RobotStateConstPtr& a, const robot_state::RobotStateConstPtr& b,
                                             double t, robot_state::RobotStatePtr result) const
{
    interpolate(a.get(), b.get(), t, result.get());
}

void CartesianSpaceInterpolator::interpolate(const robot_state::RobotState* a, const robot_state::RobotState* b,
                                             double t, robot_state::RobotState* result) const
{
    const std::vector<const robot_model::JointModel*>& joints = group_->getJointModels();

    // Initialize result with initial values
    std::vector<double> joint_vals;
    a->copyJointGroupPositions(group_, joint_vals);
    result->setJointGroupPositions(group_, joint_vals);
    result->update();

    // This is the reference point on the link that we will track (in local frame)
    Eigen::Vector3d pt(1,0,0);

    // The current starting index into the joint value array
    size_t joint_index = 0;

    // We need to iterate the joints in order from the root
    // TODO: Is this the order of joints that we want?
    // How does joint order affect the interpolator?
    for (size_t i = 0; i < joints.size(); ++i)
    {
        if (ignore_[i])
            a->interpolate(*b, t, *result, joints[i]);
        else
        {
            // The coordinate frame of this joint in the initial state
            const Eigen::Affine3d& ja = a->getGlobalLinkTransform(joints[i]->getChildLinkModel());
            // The coordinate frame of this joint in the final state
            const Eigen::Affine3d& jb = b->getGlobalLinkTransform(joints[i]->getChildLinkModel());

            // The reference point interpolated linearly through space
            Eigen::Vector3d interp = (1.0-t)*(ja * pt) + t*(jb * pt);

            // This value is with respect to the CURRENT coordinate frame, not globally.
            // In other words, val is an angle offset with respect to the current angle.
            double val = revoluteJointInterpolation(static_cast<const robot_model::RevoluteJointModel*>(joints[i]),
                                                    result->getGlobalLinkTransform(joints[i]->getChildLinkModel()), interp);
            val += joint_vals[joint_index];

            // make sure angle position is within limits
            joints[i]->enforcePositionBounds(&val);

            // Update the joint angle
            result->setJointPositions(joints[i]->getName(), &val);
        }

        joint_index += joints[i]->getVariableCount();
    }
}

// frame is the local coordinate frame of the joint
// point is the desired location of the end of the joint in base frame
double CartesianSpaceInterpolator::revoluteJointInterpolation(const robot_model::RevoluteJointModel* joint,
                                                              const Eigen::Affine3d& frame,
                                                              const Eigen::Vector3d& point) const
{
    // (Unit) Normal vector to the plane of rotation
    Eigen::Vector3d normal = frame.rotation() * joint->getAxis();

    // Origin of the plane of rotation
    Eigen::Vector3d q = frame.translation();

    // v is the vector from q to the point we are projecting
    Eigen::Vector3d v = point - q;
    // proj is point projected onto the plane (in global frame)
    Eigen::Vector3d proj = point - (((v).dot(normal)) * normal);

    // Direction vector that we want to move toward
    Eigen::Vector3d local_proj = proj - q;

    // Unit vectors of coordinate frame.
    // It is assumed that Z is the axis of rotation
    Eigen::Vector3d xaxis = frame.rotation() * Eigen::Vector3d(1, 0, 0);
    Eigen::Vector3d yaxis = frame.rotation() * Eigen::Vector3d(0, 1, 0);

    // X and Y components of the direction vector
    double x = local_proj.dot(xaxis);
    double y = local_proj.dot(yaxis);
    double angle = atan2(y,x);

    return angle;
}

} // namespace ompl_interface