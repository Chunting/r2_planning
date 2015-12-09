// Climbing benchmark code for R2 in the US Lab of the ISS
// Makes use of revised MoveIt Benchmark code:
//     https://github.com/ryanluna/moveit_benchmarks
// Author: Ryan Luna
// June 2015


#include <ros/ros.h>
#include <string>

#include "moveit/benchmarks/BenchmarkOptions.h"
#include "moveit/benchmarks/BenchmarkExecutor.h"
#include <moveit/robot_state/conversions.h>

#include "r2_planning_interface/ISSWorld.h"

static const std::string LEFT_GRIPPER_LINK0     = "r2/left_leg/gripper/jaw_left";
static const std::string LEFT_GRIPPER_LINK1     = "r2/left_leg/gripper/jaw_right";
static const std::string RIGHT_GRIPPER_LINK0    = "r2/right_leg/gripper/jaw_left";
static const std::string RIGHT_GRIPPER_LINK1    = "r2/right_leg/gripper/jaw_right";
static const std::string LEFT_FOOT_LINK         = "r2/left_leg_foot";
static const std::string RIGHT_FOOT_LINK        = "r2/right_leg_foot";

// Disable collisions with handrails that we touch or will touch
void querySwitchEvent(const moveit_msgs::MotionPlanRequest& request, planning_scene::PlanningScenePtr planning_scene, const ISSWorld* world)
{
    const moveit::core::RobotModelConstPtr& model = planning_scene->getRobotModel();
    robot_state::RobotStatePtr state(new robot_state::RobotState(planning_scene->getRobotModel()));
    moveit::core::robotStateMsgToRobotState(request.start_state, *state);
    state->update(true);

    collision_detection::AllowedCollisionMatrix& acm = planning_scene->getAllowedCollisionMatrixNonConst();

    // Disable collisions with the constrained link and the handrail it is grasping
    if (request.path_constraints.position_constraints.size() &&
        request.path_constraints.orientation_constraints.size())
    {
        // Must check both feet for grasp on the start state
        Eigen::Affine3d lgripper_pose = state->getGlobalLinkTransform(LEFT_GRIPPER_LINK0);
        unsigned int lhr_idx = world->findHandrail(lgripper_pose.translation());
        if (lhr_idx >= world->numHandrails())
            ROS_ERROR("No handrail attached to left foot");
        else
        {
            std::string handrail = world->getHandrail(lhr_idx)->name;

            acm.setEntry(LEFT_GRIPPER_LINK0, handrail, true);  // true means "allow this collision"
            acm.setEntry(LEFT_GRIPPER_LINK1, handrail, true);  // true means "allow this collision"
            acm.setEntry(LEFT_FOOT_LINK,     handrail, true);  // true means "allow this collision"

            ROS_WARN("Left foot is grasping %s.  Disabling collisions.", handrail.c_str());
        }

        Eigen::Affine3d rgripper_pose = state->getGlobalLinkTransform(RIGHT_GRIPPER_LINK0);
        unsigned int rhr_idx = world->findHandrail(rgripper_pose.translation());
        if (rhr_idx >= world->numHandrails())
            ROS_ERROR("No handrail attached to right foot");
        else
        {
            std::string handrail = world->getHandrail(rhr_idx)->name;

            acm.setEntry(RIGHT_GRIPPER_LINK0, handrail, true);  // true means "allow this collision"
            acm.setEntry(RIGHT_GRIPPER_LINK1, handrail, true);  // true means "allow this collision"
            acm.setEntry(RIGHT_FOOT_LINK,     handrail, true);  // true means "allow this collision"

            ROS_WARN("Right foot is grasping %s.  Disabling collisions.", handrail.c_str());
        }
    }

    // Disable collisions for the goal link and handrail it will be grasping at the end of the path
    if (request.goal_constraints.size())
    {
        if (request.goal_constraints[0].position_constraints.size() &&
            request.goal_constraints[0].orientation_constraints.size())
        {
            // Need to disable collision checking with the constrained link and the obstacle it is "grasping"
            collision_detection::AllowedCollisionMatrix& acm = planning_scene->getAllowedCollisionMatrixNonConst();

            bool left_leg_fixed = request.goal_constraints[0].position_constraints[0].link_name.find("left") != std::string::npos;

            if (request.goal_constraints[0].position_constraints[0].constraint_region.primitive_poses.size())
            {
                Eigen::Affine3d link_pose;
                tf::poseMsgToEigen(request.goal_constraints[0].position_constraints[0].constraint_region.primitive_poses[0], link_pose);

                Eigen::Vector3d position = link_pose.translation();
                unsigned int hr_idx = world->findHandrail(position);
                if (hr_idx < world->numHandrails())
                {
                    std::string handrail = world->getHandrail(hr_idx)->name;

                    acm.setEntry(left_leg_fixed ? LEFT_GRIPPER_LINK0 : RIGHT_GRIPPER_LINK0, handrail, true);  // true means "allow this collision"
                    acm.setEntry(left_leg_fixed ? LEFT_GRIPPER_LINK1 : RIGHT_GRIPPER_LINK1, handrail, true);  // true means "allow this collision"
                    acm.setEntry(left_leg_fixed ? LEFT_FOOT_LINK     : RIGHT_FOOT_LINK,     handrail, true);  // true means "allow this collision"

                    ROS_WARN("%s foot will grasp %s.  Disabling collisions.", left_leg_fixed ? "Left" : "Right", handrail.c_str());
                }
                else
                {
                    ROS_ERROR("Did not detect goal handrail");
                }
            }
        }
    }
}

double cartesianJointDistance(const robot_state::RobotState& s1, const robot_state::RobotState& s2, const std::vector<std::string>& links)
{
    double dist = 0.0;
    for(size_t i = 0; i < links.size(); ++i)
    {
        const Eigen::Affine3d& frame_a = s1.getGlobalLinkTransform(links[i]);
        const Eigen::Affine3d& frame_b = s2.getGlobalLinkTransform(links[i]);
        dist += (frame_b.translation() - frame_a.translation()).norm();
    }
    return dist;
}

double cartesianJointDistance(const robot_state::RobotState& s1, const robot_state::RobotState& s2)
{
    std::vector<std::string> links;
    links.push_back("r2/head");
    links.push_back("r2/left_palm");
    links.push_back("r2/right_palm");
    links.push_back("r2/left_leg_foot");
    links.push_back("r2/right_leg_foot");
    return cartesianJointDistance(s1, s2, links);
}

// Aggregate the cartesian distance information for the trajectories
void postRunCartesianDistance(const moveit_msgs::MotionPlanRequest& request, const planning_interface::MotionPlanDetailedResponse& response,
                              moveit_benchmarks::BenchmarkExecutor::PlannerRunData& run_data)
{
    if (response.trajectory_.size() == 0)
        return;
    if (response.trajectory_.size() != response.description_.size())
    {
        ROS_WARN("The number of trajectories (%lu) is different from the number of descriptions (%lu)", response.trajectory_.size(), response.description_.size());
        return;
    }

    std::cout << "Post run..." << std::endl;

    for (std::size_t i = 0; i < response.trajectory_.size(); ++i)
    {
        std::cout << "   Trajectory [" << response.description_[i] << "] has " << response.trajectory_[i]->getWayPointCount() << " states" << std::endl;

        double cart_dist = 0.0;
        const robot_trajectory::RobotTrajectory &t = *response.trajectory_[i];
        for (std::size_t j = 1; j < t.getWayPointCount(); ++j)
            cart_dist += cartesianJointDistance(t.getWayPoint(j-1), t.getWayPoint(j));

        std::string description("cartesian_distance_" + response.description_[i]);
        run_data[description + " REAL"] = boost::lexical_cast<std::string>(cart_dist);
    }

    std::cout << "Post run esta muerte" << std::endl;
}

void runBenchmarks()
{
    // Load the planning representation of the handrails
    // It is assumed that the PlanningScene loaded from the
    // benchmark server is the same as the one represented
    // by ISSWorld.  Otherwise, things will probably not work.
    ISSWorld world;

    // Read benchmark options from param server
    moveit_benchmarks::BenchmarkOptions opts(ros::this_node::getName());
    // Setup benchmark server
    moveit_benchmarks::BenchmarkExecutor server;

    std::vector<std::string> plugins;
    opts.getPlannerPluginList(plugins);
    server.initialize(plugins);
    server.addQueryStartEvent(boost::bind(&querySwitchEvent, _1, _2, &world)); // disable collisions with handrails we are touching or want to touch
    server.addPostRunEvent(postRunCartesianDistance);                          // compute the cartesian distance traveled by a set of links on each path

    // Running benchmarks
    if (!server.runBenchmarks(opts))
        ROS_ERROR("Failed to run all benchmarks");
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "iss_climbing_benchmark");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    runBenchmarks();
}