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

#ifndef CARTESIAN_DISTANCE_PATH_SIMPLIFIER_
#define CARTESIAN_DISTANCE_PATH_SIMPLIFIER_

#include <ompl/base/SpaceInformation.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/util/RandomNumbers.h>
#include <ompl/base/PlannerTerminationCondition.h>

#include <moveit/robot_state/robot_state.h>
#include "moveit/ompl_interface/parameterization/model_based_state_space.h"
#include "moveit/kinematic_constraints/kinematic_constraint.h"
#include "moveit_ompl_components/CartesianSpaceInterpolator.h"


namespace ompl_interface
{

class CartesianDistancePathSimplifier
{
public:
    CartesianDistancePathSimplifier(ompl::base::SpaceInformationPtr si);
    CartesianDistancePathSimplifier(ompl::base::SpaceInformationPtr si, CartesianSpaceInterpolator* interpolator);
    ~CartesianDistancePathSimplifier();

    void setPathConstraints(kinematic_constraints::KinematicConstraintSetPtr constraints);

    // Simplify path.  Store result in simplifiedPath.  If minStateCount > 0, the
    // simplified path is interpolated to have a minimum number of states
    // interpolator label = 0 means "traditional interpolation"
    // interpolator label = 1 means cartesian space interpolator
    void simplify(const ompl::geometric::PathGeometric& path, ompl::geometric::PathGeometric& simplifiedPath,
                  unsigned int minStateCount = 0, int interpolatorLabel = 0);

    // Simplify path until done or ptc (whichever is first).  Store result in simplifiedPath.  If minStateCount > 0, the
    // simplified path is interpolated to have a minimum number of states
    // interpolator label = 0 means "traditional interpolation"
    // interpolator label = 1 means cartesian space interpolator
    void simplify(const ompl::base::PlannerTerminationCondition& ptc, const ompl::geometric::PathGeometric& path, ompl::geometric::PathGeometric& simplifiedPath,
                  unsigned int minStateCount = 0, int interpolatorLabel = 0);

protected:
    void clear();
    void simplify(const ompl::base::PlannerTerminationCondition& ptc);
    void zipToGoal(const ompl::base::PlannerTerminationCondition& ptc);
    void reduceVertices(const ompl::base::PlannerTerminationCondition& ptc);
    void shortcutPath(const ompl::base::PlannerTerminationCondition& ptc);

    void interpolatePath(ompl::geometric::PathGeometric& path, unsigned int stateCount);  // not used

    void setFromMoveItStates(const std::vector<robot_state::RobotState*>& moveitStates_, ompl::geometric::PathGeometric& path);

    // not used
    //void interpolate(const ompl::base::State *from, const ompl::base::State *to, const double t, ompl::base::State *state);

    bool checkMotion(const robot_state::RobotState* s1, const robot_state::RobotState* s2);


    void cartesianInterpolate(const robot_model::JointModelGroup* group,
                              const robot_state::RobotState* a, const robot_state::RobotState* b,
                              double t, robot_state::RobotState* result);

    void regularInterpolate(const robot_model::JointModelGroup* group,
                            const robot_state::RobotState* a, const robot_state::RobotState* b,
                            double t, robot_state::RobotState* result);

    // In the given state, force link to achieve desired_pose (in the world frame) by adjusting
    // the base_joint.  base_joint MUST be a floating joint.
    void shiftStateByError(robot_state::RobotState* state, const robot_state::JointModel* base_joint,
                           const std::string& link, const Eigen::Affine3d& desired_pose) const;

    ompl::base::SpaceInformationPtr si_;
    ModelBasedStateSpace* mbss_;
    std::vector<int> segmentLabels_;

    ompl::RNG rng_;

    std::vector<robot_state::RobotState*> moveitStates_;

    robot_state::RobotStatePtr work_state_;

    kinematic_constraints::KinematicConstraintSetPtr path_constraints_;

    bool free_interpolator_;
    CartesianSpaceInterpolator* interpolator_;
};

}

#endif