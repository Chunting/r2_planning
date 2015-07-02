#include "moveit_ompl_components/CartesianDistanceOptimizationObjective.h"

namespace ob = ompl::base;

ompl_interface::CartesianDistanceOptimizationObjective::CartesianDistanceOptimizationObjective(const OMPLPlanningContext* context) : OptimizationObjective(context->getOMPLSpaceInformation())
{
    description_ = "Cartesian distance";
    context_ = context;
    work_state_1_.reset(new robot_state::RobotState(context->getOMPLStateSpace()->getRobotModel()));
    work_state_2_.reset(new robot_state::RobotState(context->getOMPLStateSpace()->getRobotModel()));
}

ob::Cost ompl_interface::CartesianDistanceOptimizationObjective::stateCost(const ob::State *s) const
{
    return ompl::base::OptimizationObjective::identityCost();
}

// Compute the physical distance between the joints in the group (e.g., workspace distance)
ob::Cost ompl_interface::CartesianDistanceOptimizationObjective::cartesianJointDistance(const robot_state::RobotState& s1,
                                                                                        const robot_state::RobotState& s2,
                                                                                        const std::string& group_name) const
{
    const robot_model::JointModelGroup* group = s1.getJointModelGroup(group_name);
    double dist = 0.0;

    const std::vector<const robot_model::JointModel*>& joints = group->getJointModels();
    for (size_t i = 0; i < joints.size(); ++i)
    {
        /*std::string end_effector;
        if (i < joints.size()-1)
            end_effector = joints[i+1]->getChildLinkModel()->getName().c_str();
        else
        {
            const std::vector<const robot_model::JointModel*> & joint_children = joints[i]->getChildLinkModel()->getChildJointModels();
            end_effector = joint_children[0]->getChildLinkModel()->getName().c_str();
        }

        const Eigen::Affine3d& frame_a = s1.getGlobalLinkTransform(joints[i]->getChildLinkModel());
        const Eigen::Affine3d& frame_b = s2.getGlobalLinkTransform(joints[i]->getChildLinkModel());

        const Eigen::Vector3d& point_a = s1.getLinkModel(end_effector)->getJointOriginTransform().translation();
        const Eigen::Vector3d& point_b = s2.getLinkModel(end_effector)->getJointOriginTransform().translation();

        Eigen::Vector3d pa = frame_a * point_a;
        Eigen::Vector3d pb = frame_b * point_b;
        dist += (pb - pa).norm();*/

        const Eigen::Affine3d& frame_a = s1.getGlobalLinkTransform(joints[i]->getChildLinkModel());
        const Eigen::Affine3d& frame_b = s2.getGlobalLinkTransform(joints[i]->getChildLinkModel());
        dist += (frame_b.translation() - frame_a.translation()).norm();
    }
    return ob::Cost(dist);
}

ob::Cost ompl_interface::CartesianDistanceOptimizationObjective::motionCost(const ob::State *s1, const ob::State *s2) const
{
    // TODO: Lock this function for thread safety
    context_->getOMPLStateSpace()->copyToRobotState(*(work_state_1_.get()), s1);
    context_->getOMPLStateSpace()->copyToRobotState(*(work_state_2_.get()), s2);

    return cartesianJointDistance(*(work_state_1_.get()), *(work_state_2_.get()), context_->getGroupName());
}

ob::Cost ompl_interface::CartesianDistanceOptimizationObjective::motionCostHeuristic(const ob::State *s1, const ob::State *s2) const
{
    return motionCost(s1, s2);
}