/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2016, Rice University
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

#ifndef TORSO_DECLINATION_OPTIMIZATION_OBJECTIVE_
#define TORSO_DECLINATION_OPTIMIZATION_OBJECTIVE_

#include <ompl/base/OptimizationObjective.h>
#include <moveit/ompl_interface/ompl_planning_context.h>
#include <moveit/robot_state/robot_state.h>

#include <boost/thread/mutex.hpp>

namespace ompl_interface
{
    /// \brief An optimization objective which penalizes declination of the torso
    class TorsoDeclinationOptimizationObjective : public ompl::base::OptimizationObjective
    {
    public:
        TorsoDeclinationOptimizationObjective(const OMPLPlanningContext* context, const std::string& link_name);
        ~TorsoDeclinationOptimizationObjective();

        /// \brief Returns identity cost.
        virtual ompl::base::Cost stateCost(const ompl::base::State *s) const;

        /// \brief Motion cost for this objective is defined as
        ///  the configuration space distance between \e s1 and \e
        ///  s2, using the method SpaceInformation::distance(). */
        /// \brief This is implemented as the max of two state costs
        virtual ompl::base::Cost motionCost(const ompl::base::State *s1, const ompl::base::State *s2) const;


    protected:
        //ompl::base::Cost cartesianJointDistance(const robot_state::RobotState& s1, const robot_state::RobotState& s2) const;

        /// The minimum distance between states when computing the motion cost.  Smaller is
        /// more accurate computation, but slower.
        // double minDistance_;

        // bool userSpecifiedLinks_;
        // std::vector<std::string> link_names_;

        const OMPLPlanningContext* context_;
        std::string link_name_;

        /// \brief Mutex for work_state
        boost::mutex work_state_lock_;
        /// \brief Scratch space for fast computation.  Sorry not sorry about mutable.  Blame OMPL.
        mutable robot_state::RobotStatePtr work_state_;
    };
}

#endif