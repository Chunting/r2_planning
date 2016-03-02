#include "moveit_ompl_components/TorsoDeclinationOptimizationObjective.h"

namespace ob = ompl::base;

ompl_interface::TorsoDeclinationOptimizationObjective::TorsoDeclinationOptimizationObjective(const OMPLPlanningContext* context, const std::string& link_name) : OptimizationObjective(context->getOMPLSpaceInformation()), context_(context), link_name_(link_name)
{
    description_ = "Torso declination";
    work_state_.reset(new robot_state::RobotState(context_->getOMPLStateSpace()->getRobotModel()));
}

ompl_interface::TorsoDeclinationOptimizationObjective::~TorsoDeclinationOptimizationObjective()
{
}

ob::Cost ompl_interface::TorsoDeclinationOptimizationObjective::stateCost(const ob::State *s) const
{
    Eigen::Vector3d axis(0,0,1); // the fixed axis to measure angular deviation from.  Should be unit vec.
    Eigen::Vector3d localPt(0,1,0); // point in the frame of 'link_name_' that deviates.  Should be unit vec.

    // Lock this function for thread safety of work_state_
    boost::mutex::scoped_lock(work_state_lock_);

    context_->getOMPLStateSpace()->copyToRobotState(*work_state_, s);
    work_state_->update();

    const Eigen::Affine3d& frame = work_state_->getGlobalLinkTransform(link_name_);  // FK
    Eigen::Vector3d vec = frame.rotation() * localPt;
    double declination = acos(vec.dot(axis));

    // Cost is > 0 only if declination is greater than 45 degrees
    // This works well for step 1, but not for the others
    // if (declination < 0.785398)
    //     return ob::Cost(declination / 10.0);
    // return ob::Cost(declination);

    if (declination < 0.785398)
        return ob::Cost(declination);
    else
        return ob::Cost(exp(declination));

    //return ob::Cost(exp(declination));
}

ob::Cost ompl_interface::TorsoDeclinationOptimizationObjective::motionCost(const ob::State *s1, const ob::State *s2) const
{
    return ob::Cost(std::max(stateCost(s1).value(), stateCost(s2).value()));
}

