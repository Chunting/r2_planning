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

#ifndef MOVEIT_OMPL_CONSTRAINED_INTERFACE_CONSTRAINED_PLANNING_CONTEXT_
#define MOVEIT_OMPL_CONSTRAINED_INTERFACE_CONSTRAINED_PLANNING_CONTEXT_

#include "moveit/ompl_interface/geometric_planning_context.h"
#include <ompl/geometric/ConstrainedSimpleSetup.h>
#include <moveit/constraint_samplers/constraint_sampler_manager.h>
#include <moveit/constraint_sampler_manager_loader/constraint_sampler_manager_loader.h>
#include <boost/thread/mutex.hpp>

namespace moveit_ompl_constrained_interface
{

/// \brief An OMPL planning context that configures planning algorithms that reason directly
/// over arbitrary constraints over the entire path.
class ConstrainedPlanningContext : public ompl_interface::GeometricPlanningContext
{
public:
    ConstrainedPlanningContext();

    virtual ~ConstrainedPlanningContext();

    virtual std::string getDescription();

    virtual void initialize(const std::string& /*ros_namespace*/, const ompl_interface::PlanningContextSpecification& spec);

    virtual void clear();

protected:

    virtual ompl::base::PlannerPtr configurePlanner(const std::string& planner_name, const std::map<std::string, std::string>& params);

    moveit_msgs::Constraints                         path_constraints_msg_;
};

}

#endif