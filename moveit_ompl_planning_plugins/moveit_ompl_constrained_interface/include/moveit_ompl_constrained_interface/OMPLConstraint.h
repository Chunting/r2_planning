/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2014, Rice University
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

#ifndef MOVEIT_OMPL_CONSTRAINED_INTERFACE_OMPL_CONSTRAINT_
#define MOVEIT_OMPL_CONSTRAINED_INTERFACE_OMPL_CONSTRAINT_

#include <moveit/kinematic_constraints/kinematic_constraint.h>
#include <ompl/base/Constraint.h>
#include <moveit/ompl_interface/parameterization/model_based_state_space.h>

namespace moveit_ompl_constrained_interface
{

class OMPLKinematicConstraints : public ompl::base::Constraint
{
public:

    OMPLKinematicConstraints(const ompl_interface::ModelBasedStateSpacePtr& space, const kinematic_constraints::KinematicConstraintSetPtr& kc,
                             const constraint_samplers::ConstraintSamplerPtr& cs) :
                                ompl::base::Constraint(boost::static_pointer_cast<ompl::base::StateSpace>(space)),
                                model_space_(space), kc_(kc), cs_(cs)
    {
        work_state_.reset(new robot_state::RobotState(model_space_->getRobotModel()));
    }

    virtual ~OMPLKinematicConstraints() {}

    /// \brief Check whether this state satisfies the constraints
    virtual bool isSatisfied(const ompl::base::State* state) const
    {

        //robot_state::RobotState rstate(model_space_->getRobotModel());
        //model_space_->copyToRobotState(rstate, state);
        model_space_->copyToRobotState(*(work_state_.get()), state);

        //kinematic_constraints::ConstraintEvaluationResult result = kc_->decide(rstate);
        kinematic_constraints::ConstraintEvaluationResult result = kc_->decide(*(work_state_.get()));
        return result.satisfied;
    }

    /// \brief Return the distance from satisfaction of a state
    /// A state that satisfies the constraint should have distance 0.
    virtual double distance(const ompl::base::State* state) const
    {
        //robot_state::RobotState rstate(model_space_->getRobotModel());
        //model_space_->copyToRobotState(rstate, state);
        model_space_->copyToRobotState(*(work_state_.get()), state);

        //kinematic_constraints::ConstraintEvaluationResult result = kc_->decide(rstate);
        kinematic_constraints::ConstraintEvaluationResult result = kc_->decide(*(work_state_.get()));
        return result.distance;
    }

    /// \brief Sample a state given the constraints.  If a state cannot
    /// be sampled, this method will return false.
    virtual bool sample(ompl::base::State* state)
    {
        //robot_state::RobotState rstate(model_space_->getRobotModel());
        //if (cs_->sample(rstate))

        if (cs_->sample( *(work_state_.get()) ))
        {
            //model_space_->copyToOMPLState(state, rstate);
            model_space_->copyToOMPLState(state, *(work_state_.get()));
            return true;
        }
        return false;
    }

    /// \brief Project a state given the constraints.  If a valid
    /// projection cannot be found, this method will return false.
    virtual bool project(ompl::base::State* state)
    {
        //robot_state::RobotState rstate(model_space_->getRobotModel());
        //model_space_->copyToRobotState(rstate, state);

        model_space_->copyToRobotState(*(work_state_.get()), state);
        //if (cs_->project(rstate))
        if (cs_->project( *(work_state_.get()) ))
        {
            //model_space_->copyToOMPLState(state, rstate);
            model_space_->copyToOMPLState(state, *(work_state_.get()));
            return true;
        }
        return false;
    }

protected:
    ompl_interface::ModelBasedStateSpacePtr model_space_;
    kinematic_constraints::KinematicConstraintSetPtr kc_;
    constraint_samplers::ConstraintSamplerPtr cs_;

    robot_state::RobotStatePtr work_state_;
};

}

#endif