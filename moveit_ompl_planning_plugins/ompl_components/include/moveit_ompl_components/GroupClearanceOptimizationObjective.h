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

#ifndef GROUP_CLEARANCE_OPTIMIZATION_OBJECTIVE_
#define GROUP_CLEARANCE_OPTIMIZATION_OBJECTIVE_

#include <ompl/base/OptimizationObjective.h>
#include <moveit/ompl_interface/ompl_planning_context.h>
#include <moveit/collision_detection/collision_world.h>
//#include <moveit/collision_detection/collision_robot.h>
#include "moveit_ompl_components/CollisionRobotGroup.h"

namespace ompl_interface
{
    class GroupClearanceOptimizationObjective : public ompl::base::OptimizationObjective
    {
    public:
        GroupClearanceOptimizationObjective(const OMPLPlanningContext* context) : ompl::base::OptimizationObjective(context->getOMPLSpaceInformation())
        {
            description_ = "Group clearance";
            context_ = context;
            work_state_.reset(new robot_state::RobotState(context->getOMPLStateSpace()->getRobotModel()));

            world_ = context->getPlanningScene()->getCollisionWorld();
        }

        void addGroup(const std::string& group, double scale, double cost)
        {
            CollisionRobotGroupPtr groupRobot(new CollisionRobotGroup(group, context_->getOMPLStateSpace()->getRobotModel(), scale));
            collisionGroups_.insert(CollisionGroup(groupRobot, cost));
        }

        /// \brief Compute the cost at a given state for all groups
        ///        Cost is taken as the group in collision with highest cost.
        virtual ompl::base::Cost stateCost(const ompl::base::State *s) const
        {
            /*//std::cout << __FUNCTION__ << std::endl;
            // todo: add mutex for work_state_
            context_->getOMPLStateSpace()->copyToRobotState(*(work_state_.get()), s);
            work_state_->update();

            double minClearance = std::numeric_limits<double>::infinity();
            for(size_t i = 0; i < groups_.size(); ++i)
            {
                double c = std::max(world_->distanceRobot(static_cast<collision_detection::CollisionRobot&>(*(groups_[i].get())), *(work_state_.get())), 0.0);
                //std::cout << "  Clearance for group [" << i << "]: " << c << std::endl;
                minClearance = std::min(minClearance, c);
            }

            //std::cout << "Clearance: " << minClearance << std::endl << std::endl;

            if (minClearance < 1e-8) // don't divide by zero
                return infiniteCost();

            return ompl::base::Cost(1.0/minClearance);*/

            // todo: add mutex for work_state_
            context_->getOMPLStateSpace()->copyToRobotState(*(work_state_.get()), s);
            work_state_->update();

            collision_detection::CollisionRequest req;
            req.verbose = false;
            req.distance = false;
            collision_detection::CollisionResult resp;

            ompl::base::Cost cost(0.0);
            std::set<CollisionGroup>::const_iterator it;
            // Groups are sorted in descending cost order
            for(it = collisionGroups_.begin(); it != collisionGroups_.end(); ++it)
            {
                world_->checkRobotCollision(req, resp, static_cast<collision_detection::CollisionRobot&>(*(it->group_.get())), *(work_state_.get()));
                if (resp.collision)
                {
                    cost = ompl::base::Cost(it->cost_);
                    break;
                }
            }

            return cost;
        }

        /// \brief Compute the clearance of the motion between two given states
        /// Currently, this is just the min clearance between the end points.
        virtual ompl::base::Cost motionCost(const ompl::base::State *s1, const ompl::base::State *s2) const
        {
            ompl::base::Cost c1 = stateCost(s1);
            ompl::base::Cost c2 = stateCost(s2);
            if (c2.value() < c1.value())
                c1 = c2;

            // c1 is the min cost

            double d = si_->distance(s1, s2);
            double dist_weight = 0.1;
            return ompl::base::Cost(c1.value() + dist_weight * d);
        }

        // We are never satisfied
        virtual bool isSatisfied(ompl::base::Cost /*c*/) const
        {
            return false;
        }


    protected:
        struct CollisionGroup
        {
            CollisionGroup(CollisionRobotGroupPtr group, double cost) : group_(group), cost_(cost) {}

            // Sort in descending order
            bool operator < (const CollisionGroup& other) const
            {
                return cost_ > other.cost_;
            }

            ompl_interface::CollisionRobotGroupPtr group_;
            double cost_;
        };

        const OMPLPlanningContext* context_;
        //std::vector<ompl_interface::CollisionRobotGroupPtr> groups_;
        //std::vector<double> costs_;
        std::set<CollisionGroup> collisionGroups_;
        collision_detection::CollisionWorldConstPtr world_;

        mutable robot_state::RobotStatePtr work_state_;


    };
}

#endif