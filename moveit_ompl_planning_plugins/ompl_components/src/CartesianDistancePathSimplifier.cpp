/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2015, Rice University
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Rice University nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Ryan Luna */

#include <ros/ros.h>
#include "moveit_ompl_components/CartesianDistancePathSimplifier.h"

namespace ompl_interface
{

CartesianDistancePathSimplifier::CartesianDistancePathSimplifier(ompl::base::SpaceInformationPtr si, CartesianSpaceInterpolator* interpolator) : si_(si)
{
    mbss_ = dynamic_cast<ModelBasedStateSpace*>(si_->getStateSpace().get());
    if (!mbss_)
    {
        ROS_ERROR("Failed to cast StateSpace to type ModelBasedStateSpace");
        throw;
    }

    if (interpolator)
    {
        free_interpolator_ = false;
        interpolator_ = interpolator;
    }
    else
    {
        free_interpolator_ = true;
        interpolator_ = new CartesianSpaceInterpolator(mbss_->getJointModelGroup());
    }
    assert(interpolator_);

    work_state_.reset(new robot_state::RobotState(mbss_->getRobotModel()));
    work_state_->setToDefaultValues(); // VERY IMPORTANT
    work_state_->update();
}

CartesianDistancePathSimplifier::~CartesianDistancePathSimplifier()
{
    clear();
    if (free_interpolator_ && interpolator_)
        delete interpolator_;
}

void CartesianDistancePathSimplifier::setPathConstraints(kinematic_constraints::KinematicConstraintSetPtr constraints)
{
    path_constraints_ = constraints;
}

void CartesianDistancePathSimplifier::simplify(const ompl::geometric::PathGeometric& path, ompl::geometric::PathGeometric& simplifiedPath,
                                               unsigned int minStateCount, int interpolatorLabel)
{
    ompl::base::PlannerTerminationCondition ptc = ompl::base::plannerNonTerminatingCondition();
    simplify(ptc, path, simplifiedPath, minStateCount, interpolatorLabel);
}

void CartesianDistancePathSimplifier::simplify(const ompl::base::PlannerTerminationCondition& ptc, const ompl::geometric::PathGeometric& path,
                                               ompl::geometric::PathGeometric& simplifiedPath,
                                               unsigned int minStateCount, int interpolatorLabel)
{
    // if (path.getStateCount() < 3)
    // {
    //     ROS_INFO("CartesianDistancePathSimplifier: not simplifying input path with %lu states", path.getStateCount());
    //     simplifiedPath = path;
    //     simplifiedPath.interpolate(10);
    //     return;
    // }

    ompl::geometric::PathGeometric interpPath(path);

    // ensure a minimum number of vertices so that we have something to work with in simplification.
    // Note - this is "normal" interpolation
    interpPath.interpolate(10);

    segmentLabels_.assign(interpPath.getStateCount() - 1, interpolatorLabel);

    // Construct moveit state representation of the path
    clear();
    moveitStates_.resize(interpPath.getStateCount());
    for(size_t i = 0; i < interpPath.getStateCount(); ++i)
    {
        moveitStates_[i] = new robot_state::RobotState(mbss_->getRobotModel());
        mbss_->copyToRobotState(*(moveitStates_[i]), interpPath.getState(i));
    }

    // simplify using the moveit representation of the path
    simplify(ptc);

    assert(moveitStates_.size() >= 2);
    assert(segmentLabels_.size() == moveitStates_.size()-1);

    // translate the moveit path to the ompl path
    setFromMoveItStates(moveitStates_, simplifiedPath);

    if (minStateCount)
        simplifiedPath.interpolate(minStateCount);

    //ROS_INFO("CartesianDistancePathSimplifier: Returning simplified path with %lu states", simplifiedPath.getStateCount());
}

// void CartesianDistancePathSimplifier::simplifyAndInterpolate(const ompl::geometric::PathGeometric& path, ompl::geometric::PathGeometric& simplifiedPath, unsigned int minStateCount)
// {
//     simplify(path, simplifiedPath);
//     //interpolatePath(simplifiedPath, minStateCount);
//     simplifiedPath.interpolate(minStateCount);
// }

void CartesianDistancePathSimplifier::clear()
{
    for(size_t i = 0; i < moveitStates_.size(); ++i)
        delete moveitStates_[i];
    moveitStates_.clear();
}

void CartesianDistancePathSimplifier::simplify(const ompl::base::PlannerTerminationCondition& ptc)
{
    // Try to connect each state to the goal
    zipToGoal(ptc);

    // Randomly shortcut existing states
    reduceVertices(ptc);

    // Randomly shortcut any configuration between states
    shortcutPath(ptc);
}

void CartesianDistancePathSimplifier::zipToGoal(const ompl::base::PlannerTerminationCondition& ptc)
{
    if (moveitStates_.size() < 3)
        return;

    for(size_t i = 0; i < moveitStates_.size() - 2 && !ptc; ++i)
    {
        // Try to connect to the end directly
        if (checkMotion(moveitStates_[i], moveitStates_.back()))
        {
            segmentLabels_[i] = 1; // mark this segment as cartesian interpolator
            // Erase all labels afterward
            segmentLabels_.erase(segmentLabels_.begin() + i + 1, segmentLabels_.end());

            // Delete all states between i+1 and the end
            for(size_t j = i+1; j < moveitStates_.size() - 1; ++j)
                delete moveitStates_[j];
            moveitStates_.erase(moveitStates_.begin() + i + 1, moveitStates_.end() - 1);

            //ROS_INFO("zipToGoal: Snipped from %lu (of %lu) to the end", i+1, moveitStates_.size());
            return;
        }
    }
}

void CartesianDistancePathSimplifier::reduceVertices(const ompl::base::PlannerTerminationCondition& ptc)
{
    // Try to connect randomly selected states in the path
    unsigned int maxSteps = moveitStates_.size();
    unsigned int maxEmptySteps = moveitStates_.size();
    double rangeRatio = 0.33;
    //bool result = false;
    unsigned int nochange = 0;

    for (unsigned int i = 0; i < maxSteps && nochange < maxEmptySteps && !ptc; ++i, ++nochange)
    {
        int count = moveitStates_.size();
        int maxN  = count - 1;
        int range = 1 + (int)(floor(0.5 + (double)count * rangeRatio));

        int p1 = rng_.uniformInt(0, maxN);
        int p2 = rng_.uniformInt(std::max(p1 - range, 0), std::min(maxN, p1 + range));
        if (abs(p1 - p2) < 2)
        {
            if (p1 < maxN - 1)
                p2 = p1 + 2;
            else
                if (p1 > 1)
                    p2 = p1 - 2;
                else
                    continue;
        }

        if (p1 > p2)
            std::swap(p1, p2);

        if (checkMotion(moveitStates_[p1], moveitStates_[p2]))
        {
            //ROS_INFO("CartesianDistancePathSimplifier: 'shortcutting' between %d and %d (%lu total)", p1, p2, moveitStates_.size());

            // delete states in between p1 and p2
            for(int j = p1 + 1; j < p2; ++j)
                delete moveitStates_[j];
            moveitStates_.erase(moveitStates_.begin() + p1 + 1, moveitStates_.begin() + p2);

            // remove outgoing edge labels
            segmentLabels_.erase(segmentLabels_.begin() + p1 + 1, segmentLabels_.begin() + p2);
            segmentLabels_[p1] = 1;  // label motion as cartesian space interpolator

            nochange = 0;

            // if (freeStates_)
            //     for (int j = p1 + 1 ; j < p2 ; ++j)
            //         si->freeState(states[j]);
            // states.erase(states.begin() + p1 + 1, states.begin() + p2);
            // nochange = 0;
            // result = true;
        }
    }
}

void CartesianDistancePathSimplifier::shortcutPath(const ompl::base::PlannerTerminationCondition& ptc)
{
    if (moveitStates_.size() < 3)
        return;
    if (ptc)
        return;

    const robot_model::JointModelGroup* group = mbss_->getJointModelGroup();

    unsigned int maxSteps = moveitStates_.size()*3;
    unsigned int maxEmptySteps = moveitStates_.size();
    double rangeRatio = 0.33;
    double snapToVertex = 0.005;

    // dists[i] contains the cumulative length of the path up to and include state i
    std::vector<double> dists(moveitStates_.size(), 0.0);
    for (unsigned int i = 1; i < dists.size(); ++i)
        dists[i] = dists[i-1] + moveitStates_[i-1]->distance(*moveitStates_[i]); //si->distance(states[i-1], states[i]);

    // Sampled states closer than 'threshold' distance to any existing state in the path
    // are snapped to the close state
    double threshold = dists.back() * snapToVertex;
    // The range (distance) of a single connection that will be attempted
    double rd = rangeRatio * dists.back();

    //base::State *temp0 = si->allocState();
    //base::State *temp1 = si->allocState();
    robot_state::RobotState* temp0 = new robot_state::RobotState(mbss_->getRobotModel());
    robot_state::RobotState* temp1 = new robot_state::RobotState(mbss_->getRobotModel());
    temp0->setToDefaultValues();
    temp0->update();
    temp1->setToDefaultValues();
    temp1->update();

    bool result = false;
    unsigned int nochange = 0;

    // Attempt shortcutting maxSteps times or when no improvement is found after
    // maxEmptySteps attempts, whichever comes first
    for (unsigned int i = 0; i < maxSteps && nochange < maxEmptySteps && !ptc; ++i, ++nochange)
    {
        // Sample a random state anywhere along the path
        robot_state::RobotState* s0 = NULL;
        int index0 = -1;
        double t0 = 0.0;
        double p0 = rng_.uniformReal(0.0, dists.back());                                        // sample a random point (p0) along the path
        std::vector<double>::iterator pit = std::lower_bound(dists.begin(), dists.end(), p0);   // find the NEXT waypoint after the random point
        int pos0 = pit == dists.end() ? dists.size() - 1 : pit - dists.begin();                 // get the index of the NEXT waypoint after the point

        if (pos0 == 0 || dists[pos0] - p0 < threshold) // snap to the NEXT waypoint
            index0 = pos0;
        else
        {
            while (pos0 > 0 && p0 < dists[pos0])
                --pos0;
            if (p0 - dists[pos0] < threshold)  // snap to the PREVIOUS waypoint
                index0 = pos0;
        }

        // Sample a random point within rd distance of the previously sampled point
        robot_state::RobotState* s1 = NULL;
        int index1 = -1;
        double t1 = 0.0;
        double p1 = rng_.uniformReal(std::max(0.0, p0 - rd), std::min(p0 + rd, dists.back()));  // sample a random point (p1) near p0
        pit = std::lower_bound(dists.begin(), dists.end(), p1);                                 // find the NEXT waypoint after the random point
        int pos1 = pit == dists.end() ? dists.size() - 1 : pit - dists.begin();                 // get the index of the NEXT waypoint after the point

        if (pos1 == 0 || dists[pos1] - p1 < threshold) // snap to the NEXT waypoint
            index1 = pos1;
        else
        {
            while (pos1 > 0 && p1 < dists[pos1])
                --pos1;
            if (p1 - dists[pos1] < threshold)  // snap to the PREVIOUS waypoint
                index1 = pos1;
        }

        // Don't waste time on points that are on the same path segment
        if (pos0 == pos1 || index0 == pos1 || index1 == pos0 ||
            pos0 + 1 == index1 || pos1 + 1 == index0 ||
            (index0 >=0 && index1 >= 0 && abs(index0 - index1) < 2))
            continue;

        // Get the state pointer for p0
        if (index0 >= 0)
            s0 = moveitStates_[index0];
        else
        {
            t0 = (p0 - dists[pos0]) / (dists[pos0 + 1] - dists[pos0]);
            if (segmentLabels_[pos0] == 0) // normal interpolation
                regularInterpolate(group, moveitStates_[pos0], moveitStates_[pos0+1], t0, temp0);
            else
                cartesianInterpolate(group, moveitStates_[pos0], moveitStates_[pos0+1], t0, temp0);
            s0 = temp0;
        }

        // Get the state pointer for p1
        if (index1 >= 0)
            s1 = moveitStates_[index1];
        else
        {
            t1 = (p1 - dists[pos1]) / (dists[pos1 + 1] - dists[pos1]);
            if (segmentLabels_[pos1] == 0) // normal interpolation
                regularInterpolate(group, moveitStates_[pos1], moveitStates_[pos1+1], t1, temp1);
            else
                cartesianInterpolate(group, moveitStates_[pos1], moveitStates_[pos1+1], t1, temp1);
            s1 = temp1;
        }

        // Check for validity between s0 and s1
        if (checkMotion(s0, s1))
        {
            //ROS_INFO("Real shortcut!");
            if (pos0 > pos1)
            {
                std::swap(pos0, pos1);
                std::swap(index0, index1);
                std::swap(s0, s1);
                std::swap(t0, t1);
            }

            // Modify the path with the new, shorter result
            if (index0 < 0 && index1 < 0)  // both states were interpolated
            {
                // interpolated states are on adjacent edges
                // copy s0 into the state that got snipped and add a new state s1 afterwards
                if (pos0 + 1 == pos1)
                {
                    *(moveitStates_[pos1]) = *s0;
                    moveitStates_.insert(moveitStates_.begin() + pos1 + 1, new robot_state::RobotState(*s1));

                    // set segment label after the new state to whatever the existing interpolation scheme was
                    segmentLabels_.insert(segmentLabels_.begin() + pos1 + 1, segmentLabels_[pos1]);
                    // mark transition between s0 and s1 as Cartesian interpolation
                    segmentLabels_[pos1] = 1;

                }

                // interpolated states are not on adjacent edges
                else
                {
                    // snip states that were bypassed
                    for (int j = pos0 + 2 ; j < pos1 ; ++j)
                        delete moveitStates_[j];

                    *(moveitStates_[pos0 + 1]) = *s0;
                    *(moveitStates_[pos1]) = *s1;
                    moveitStates_.erase(moveitStates_.begin() + pos0 + 2, moveitStates_.begin() + pos1);
                    segmentLabels_.erase(segmentLabels_.begin() + pos0 + 2, segmentLabels_.begin() + pos1);
                    segmentLabels_[pos0+1] = 1;
                }
            }
            else
            {
                // both states were NOT interpolated.  Just snip the states in between
                if (index0 >= 0 && index1 >= 0)
                {
                    for(int j = index0 + 1; j < index1; ++j)
                        delete moveitStates_[j];
                    moveitStates_.erase(moveitStates_.begin() + index0 + 1, moveitStates_.begin() + index1);
                    segmentLabels_.erase(segmentLabels_.begin() + index0 + 1, segmentLabels_.begin() + index1);
                    segmentLabels_[index0] = 1;
                }
                else
                    // only s0 was interpolated
                    if (index0 < 0 && index1 >= 0)
                    {
                        // delete states after s0 and before s1
                        for(int j = pos0 + 2; j < index1; ++j)
                            delete moveitStates_[j];

                        *(moveitStates_[pos0 + 1]) = *s0;
                        moveitStates_.erase(moveitStates_.begin() + pos0 + 2, moveitStates_.begin() + index1);
                        segmentLabels_.erase(segmentLabels_.begin() + pos0 + 2, segmentLabels_.begin() + index1);
                        segmentLabels_[pos0 + 1] = 1;
                    }
                    else
                    {
                        // only s1 was interpolated
                        if (index0 >= 0 && index1 < 0)
                        {
                            // delete all states after s0 and before pos1
                            for(int j = index0 + 1; j < pos1 ; ++j)
                                delete moveitStates_[j];

                            *(moveitStates_[pos1]) = *s1;
                            moveitStates_.erase(moveitStates_.begin() + index0 + 1, moveitStates_.begin() + pos1);
                            segmentLabels_.erase(segmentLabels_.begin() + index0 + 1, segmentLabels_.begin() + pos1);
                            segmentLabels_[index0] = 1;
                        }
                        else throw; // something really bad happened.  This should never occur
                    }
            }

            //fix the helper variables
            dists.resize(moveitStates_.size(), 0.0);
            for(unsigned int j = pos0 + 1; j < dists.size(); ++j)
                dists[j] = dists[j-1] + moveitStates_[j-1]->distance(*moveitStates_[j]);
            threshold = dists.back() * snapToVertex;
            rd = rangeRatio * dists.back();
            result = true;
            nochange = 0;
        }
    }

    delete temp0;
    delete temp1;
}

// Linear motion checking using cartesian distance interpolator
bool CartesianDistancePathSimplifier::checkMotion(const robot_state::RobotState* s1, const robot_state::RobotState* s2)
{
    // Assuming s1 and s2 are already valid
    bool result = true;

    double d = s1->distance(*s2);
    int nd = mbss_->getValidSegmentCountFactor() * (unsigned int)ceil(d) / mbss_->getLongestValidSegmentLength();

    if (nd > 1)
    {
        // TODO: make this a member variable
        ompl::base::State *test = si_->allocState();

        for(int j = 1; j < nd && result; ++j)
        {
            cartesianInterpolate(mbss_->getJointModelGroup(), s1, s2, (double)j / nd, work_state_.get());
            mbss_->copyToOMPLState(test, *work_state_);
            result = si_->isValid(test);
        }

        si_->freeState(test);
    }

    return result;
}

void CartesianDistancePathSimplifier::interpolatePath(ompl::geometric::PathGeometric& path, unsigned int stateCount)
{
}

void CartesianDistancePathSimplifier::setFromMoveItStates(const std::vector<robot_state::RobotState*>& moveitStates_, ompl::geometric::PathGeometric& path)
{
    // TODO: make this a member variable
    ompl::base::State *test = si_->allocState();

    for(size_t i = 0; i < moveitStates_.size()-1; ++i)
    {
        mbss_->copyToOMPLState(test, *moveitStates_[i]);
        path.append(test);

        // for cartesian space interpolated edges, interpolate a few states along the edge to preserve the intent of the validated motion to the next state
        if (segmentLabels_[i] != 0)
        {
            double delta = 0.10;
            double t = delta;
            while (t < 1.0 - std::numeric_limits<double>::epsilon())
            {
                cartesianInterpolate(mbss_->getJointModelGroup(), moveitStates_[i], moveitStates_[i+1], t, work_state_.get());
                mbss_->copyToOMPLState(test, *work_state_);
                path.append(test);

                t += delta;
            }
        }
    }

    // add the last state
    mbss_->copyToOMPLState(test, *moveitStates_.back());
    path.append(test);

    si_->freeState(test);
}

void CartesianDistancePathSimplifier::cartesianInterpolate(const robot_model::JointModelGroup* group,
                                                           const robot_state::RobotState* a, const robot_state::RobotState* b,
                                                           double t, robot_state::RobotState* result)
{
    interpolator_->interpolate(a, b, t, result);

    // fix path constraints (single pose only)
    if (path_constraints_)
    {
        const std::vector<moveit_msgs::PositionConstraint>& pos_constraints = path_constraints_->getPositionConstraints();
        const std::vector<moveit_msgs::OrientationConstraint>& orn_constraints = path_constraints_->getOrientationConstraints();

        // This only works for a single pose (position+orientation) constraint
        if (pos_constraints.size() != 1 || orn_constraints.size() != 1)
            return;
        if (pos_constraints[0].link_name != orn_constraints[0].link_name)
            return;

        const std::string& link_name = pos_constraints[0].link_name;

        // Both of the input states should satisfy path constraints, so just pick one of them
        assert(path_constraints_->decide(*a).satisfied);
        shiftStateByError(result, a->getRobotModel()->getRootJoint(),
                          link_name, a->getGlobalLinkTransform(link_name));
    }
}

void CartesianDistancePathSimplifier::regularInterpolate(const robot_model::JointModelGroup* group,
                                                         const robot_state::RobotState* a, const robot_state::RobotState* b,
                                                         double t, robot_state::RobotState* result)
{
    a->interpolate(*b, t, *result);

    // fix path constraints (single pose only)
    if (path_constraints_)
    {
        const std::vector<moveit_msgs::PositionConstraint>& pos_constraints = path_constraints_->getPositionConstraints();
        const std::vector<moveit_msgs::OrientationConstraint>& orn_constraints = path_constraints_->getOrientationConstraints();

        // This only works for a single pose (position+orientation) constraint
        if (pos_constraints.size() != 1 || orn_constraints.size() != 1)
            return;
        if (pos_constraints[0].link_name != orn_constraints[0].link_name)
            return;

        const std::string& link_name = pos_constraints[0].link_name;

        // Both of the input states should satisfy path constraints, so just pick one of them
        assert(path_constraints_->decide(*a).satisfied);
        shiftStateByError(result, a->getRobotModel()->getRootJoint(),
                          link_name, a->getGlobalLinkTransform(link_name));
    }
}

void CartesianDistancePathSimplifier::shiftStateByError(robot_state::RobotState* state, const robot_state::JointModel* base_joint,
                                                        const std::string& link, const Eigen::Affine3d& desired_pose) const
{
    if (base_joint->getType() != robot_state::JointModel::FLOATING)
    {
        ROS_ERROR("%s: Expected root joint '%s' to be floating.  This joint is %s",
                  __FUNCTION__, base_joint->getName().c_str(), base_joint->getTypeName().c_str());
        return;
    }

    Eigen::Affine3d actual_pose = state->getGlobalLinkTransform(link);

    // This is the error in the pose
    Eigen::Affine3d diff(desired_pose * actual_pose.inverse());

    // Apply the difference to the (unconstrained) base frame
    // VERY important that diff is multiplied on left side
    Eigen::Affine3d new_base_frame = diff * state->getGlobalLinkTransform(base_joint->getChildLinkModel()->getName());
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
    state->setJointPositions(base_joint, new_variables);
    state->update();
}

// not used
/*void CartesianDistancePathSimplifier::interpolate(const ompl::base::State *from, const ompl::base::State *to,
                                                  const double t, ompl::base::State *state)
{
    //boost::mutex::scoped_lock(work_state_lock_);

    const robot_model::JointModelGroup* group = mbss_->getJointModelGroup();
    work_state_1_->setJointGroupPositions(group, from->as<ModelBasedStateSpace::StateType>()->values);
    work_state_1_->update();
    work_state_2_->setJointGroupPositions(group, to->as<ModelBasedStateSpace::StateType>()->values);
    work_state_2_->update();

    // Interpolate, then store the result in work_state_3
    CartesianSpaceInterpolator::Interpolate(group, work_state_1_, work_state_2_, t, work_state_3_);
    work_state_3_->update();

    // fix work_state_3_ to satisfy to the path constraints
    // if (path_constraints_)
    // {
    //     const std::vector<moveit_msgs::PositionConstraint>& pos_constraints = path_constraints_->getPositionConstraints();
    //     const std::vector<moveit_msgs::OrientationConstraint>& orn_constraints = path_constraints_->getOrientationConstraints();
    //     assert(pos_constraints.size() == 1);
    //     assert(orn_constraints.size() == 1);
    //     assert(pos_constraints[0].link_name == orn_constraints[0].link_name);

    //     const std::string& link_name = pos_constraints[0].link_name;

    //     // One of the input states must satisfy the path constraints - typically the 'from' state.
    //     // Depending on the planner, however, we may interpolate 'backward', so we have to figure out
    //     // at runtime which state is the satisfying one.
    //     if (path_constraints_->decide(*(work_state_2_.get())).satisfied)
    //         shiftStateByError(work_state_3_, work_state_3_->getRobotModel()->getRootJoint(),
    //                           link_name, work_state_2_->getGlobalLinkTransform(link_name));
    //     else
    //     {
    //         assert(path_constraints_->decide(*(work_state_1_.get())).satisfied);
    //         shiftStateByError(work_state_3_, work_state_3_->getRobotModel()->getRootJoint(),
    //                           link_name, work_state_1_->getGlobalLinkTransform(link_name));
    //     }
    // }

    mbss_->copyToOMPLState(state, *(work_state_3_.get()));
}*/

}