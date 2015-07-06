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
#include <moveit/move_group/capability_names.h>
#include <r2_planning_interface/R2Interface.h>

#include <moveit_msgs/GetMotionPlan.h>
#include <r2_planning_msgs/R2MotionPlanRequest.h>

static const char* const R2_PLANNING_SERVICE_NAME = "r2_motion_plan";

class R2PlanningServer
{
public:
    R2PlanningServer()
    {
    }

    ~R2PlanningServer()
    {
        ROS_INFO("R2 planning server kaput");
    }

    void start()
    {
        ROS_INFO("Starting R2 planning server...");

        // wait for move group to come online.  Timeout after 1 minute
        bool good = ros::service::waitForService(move_group::PLANNER_SERVICE_NAME, ros::Duration(60));
        if (!good)
            ROS_ERROR("R2 planning server: Timeout waiting for move_group services to come online.  Motion planning may be unavailable");

        ros::ServiceServer service = nh.advertiseService(R2_PLANNING_SERVICE_NAME, &R2PlanningServer::planningService, this);
        ROS_INFO("R2 planning server started");
    }

    bool planningService(r2_planning_msgs::R2MotionPlanRequest::Request& req,
                         r2_planning_msgs::R2MotionPlanRequest::Response& resp)
    {
        if (!ros::service::exists(move_group::PLANNER_SERVICE_NAME, false))
        {
            ROS_ERROR("Move_group services not online.  R2 planning service cannot continue.");
            return false;
        }

        ROS_INFO("R2 planning server received new plan request");

        // Open up a client to the planner service
        ros::ServiceClient plan_client = nh.serviceClient<moveit_msgs::GetMotionPlan>(move_group::PLANNER_SERVICE_NAME);

        // translating request into MoveIt request
        moveit_msgs::GetMotionPlan::Request  plan_req;
        moveit_msgs::GetMotionPlan::Response plan_resp;

        plan_req.motion_plan_request.start_state = req.plan_request.start_state;
        plan_req.motion_plan_request.goal_constraints = req.plan_request.goal_constraints;
        plan_req.motion_plan_request.path_constraints = req.plan_request.path_constraints;
        plan_req.motion_plan_request.planner_id = req.plan_request.planner_id;
        plan_req.motion_plan_request.group_name = req.plan_request.group_name;
        plan_req.motion_plan_request.num_planning_attempts = req.plan_request.num_planning_attempts;
        plan_req.motion_plan_request.allowed_planning_time = req.plan_request.allowed_planning_time;

        // Update the allowed collision matrix
        std::vector<std::string> bodies;
        for(size_t i = 0; i < req.plan_request.collision_updates.size(); ++i)
            interface.enableCollisionChecking(req.plan_request.collision_updates[i].bodies, req.plan_request.collision_updates[i].allow_collision, true);

        // Motion planning
        if (plan_client.call(plan_req, plan_resp))
        {
            resp.plan_response.trajectory_start = plan_resp.motion_plan_response.trajectory_start;
            resp.plan_response.group_name = plan_resp.motion_plan_response.group_name;
            resp.plan_response.trajectory = plan_resp.motion_plan_response.trajectory;
            resp.plan_response.planning_time = plan_resp.motion_plan_response.planning_time;
            resp.plan_response.error_code = plan_resp.motion_plan_response.error_code;
            return true;
        }
        else
        {
            resp.plan_response.error_code = plan_resp.motion_plan_response.error_code;
            return false;
        }
    }

protected:
    R2Interface interface;
    ros::NodeHandle nh;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "r2_planning_server");

    R2PlanningServer server;
    server.start();

    ros::spin();
}