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

    //std::cout << "THERE ARE " << world->numHandrails() << " handrails in the world" << std::endl;
    //std::cout << request << std::endl;

    // Disable collisions with the constrained link and the handrail it is grasping
    if (request.path_constraints.position_constraints.size() &&
        request.path_constraints.orientation_constraints.size())
    {
        // Must check both feet for grasp on the start state
        Eigen::Affine3d lgripper_pose = state->getGlobalLinkTransform(LEFT_GRIPPER_LINK0);
        unsigned int lhr_idx = world->findHandrail(lgripper_pose.translation());
        if (lhr_idx >= world->numHandrails())
        {
            ROS_ERROR("No handrail attached to left foot");
            std::cout << "Position: " << lgripper_pose.translation().transpose() << std::endl;
        }
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
        {
            ROS_ERROR("No handrail attached to right foot");
            std::cout << "Position: " << rgripper_pose.translation().transpose() << std::endl;
        }
        else
        {
            std::string handrail = world->getHandrail(rhr_idx)->name;

            acm.setEntry(RIGHT_GRIPPER_LINK0, handrail, true);  // true means "allow this collision"
            acm.setEntry(RIGHT_GRIPPER_LINK1, handrail, true);  // true means "allow this collision"
            acm.setEntry(RIGHT_FOOT_LINK,     handrail, true);  // true means "allow this collision"

            ROS_WARN("Right foot is grasping %s.  Disabling collisions.", handrail.c_str());
        }
    }

    if (request.goal_constraints.size() > 1)
        ROS_WARN("Only expected one set of goal constraints.  Got %lu", request.goal_constraints.size());

    // Disable collisions with the handrail(s) that are grasped at the end of the path
    for(size_t i = 0; i < request.goal_constraints[0].position_constraints.size(); ++i)
    {
        for(size_t j = 0; j < request.goal_constraints[0].orientation_constraints.size(); ++j)
        {
            if (request.goal_constraints[0].position_constraints[i].link_name == request.goal_constraints[0].orientation_constraints[j].link_name)
            {
                std::cout << "CHECKING OUT " << request.goal_constraints[0].position_constraints[i].link_name << std::endl;
                std::string link = request.goal_constraints[0].position_constraints[i].link_name;
                bool leg  = link.find("leg")  != std::string::npos;
                bool left = link.find("left") != std::string::npos;
                if (leg)
                {
                    Eigen::Affine3d link_pose;
                    tf::poseMsgToEigen(request.goal_constraints[0].position_constraints[i].constraint_region.primitive_poses[0], link_pose);

                    Eigen::Vector3d position = link_pose.translation();
                    unsigned int hr_idx = world->findHandrail(position);
                    if (hr_idx < world->numHandrails())
                    {
                        std::string handrail = world->getHandrail(hr_idx)->name;

                        acm.setEntry(left ? LEFT_GRIPPER_LINK0 : RIGHT_GRIPPER_LINK0, handrail, true);  // true means "allow this collision"
                        acm.setEntry(left ? LEFT_GRIPPER_LINK1 : RIGHT_GRIPPER_LINK1, handrail, true);  // true means "allow this collision"
                        acm.setEntry(left ? LEFT_FOOT_LINK     : RIGHT_FOOT_LINK,     handrail, true);  // true means "allow this collision"

                        ROS_WARN("%s foot will grasp %s.  Disabling collisions.", left ? "Left" : "Right", handrail.c_str());
                    }
                    else
                    {
                        ROS_ERROR("Did not detect goal handrail for foot");
                    }
                }
                else // assume arm
                {
                    // All the links that compose the left hand
                    std::vector<std::string> left_links;
                    left_links.push_back("r2/left_palm");
                    left_links.push_back("r2/left_thumb_base");
                    left_links.push_back("r2/left_thumb_proximal");
                    left_links.push_back("r2/left_thumb_medial_prime");
                    left_links.push_back("r2/left_thumb_medial");
                    left_links.push_back("r2/left_thumb_distal");
                    left_links.push_back("r2/left_thumb_tip");
                    left_links.push_back("r2/left_index_base");
                    left_links.push_back("r2/left_index_yaw");
                    left_links.push_back("r2/left_index_proximal");
                    left_links.push_back("r2/left_index_medial");
                    left_links.push_back("r2/left_middle_base");
                    left_links.push_back("r2/left_middle_yaw");
                    left_links.push_back("r2/left_middle_proximal");
                    left_links.push_back("r2/left_middle_medial");
                    //left_links.push_back("r2/left_middle_distal");
                    //left_links.push_back("r2/left_middle_tip");
                    left_links.push_back("r2/left_ring_proximal");
                    //left_links.push_back("r2/left_ring_medial");
                    //left_links.push_back("r2/left_ring_distal");
                    //left_links.push_back("r2/left_ring_tip");
                    left_links.push_back("r2/left_little_proximal");
                    //left_links.push_back("r2/left_little_medial");
                    //left_links.push_back("r2/left_little_distal");
                    //left_links.push_back("r2/left_little_tip");

                    // All the links that compose the right hand
                    std::vector<std::string> right_links;
                    right_links.push_back("r2/right_palm");
                    right_links.push_back("r2/right_thumb_base");
                    right_links.push_back("r2/right_thumb_proximal");
                    right_links.push_back("r2/right_thumb_medial_prime");
                    right_links.push_back("r2/right_thumb_medial");
                    right_links.push_back("r2/right_thumb_distal");
                    right_links.push_back("r2/right_thumb_tip");
                    right_links.push_back("r2/right_index_base");
                    right_links.push_back("r2/right_index_yaw");
                    right_links.push_back("r2/right_index_proximal");
                    right_links.push_back("r2/right_index_medial");
                    right_links.push_back("r2/right_middle_base");
                    right_links.push_back("r2/right_middle_yaw");
                    right_links.push_back("r2/right_middle_proximal");
                    right_links.push_back("r2/right_middle_medial");
                    //right_links.push_back("r2/right_middle_distal");
                    //right_links.push_back("r2/right_middle_tip");
                    right_links.push_back("r2/right_ring_proximal");
                    //right_links.push_back("r2/right_ring_medial");
                    //right_links.push_back("r2/right_ring_distal");
                    //right_links.push_back("r2/right_ring_tip");
                    right_links.push_back("r2/right_little_proximal");
                    //right_links.push_back("r2/right_little_medial");
                    //right_links.push_back("r2/right_little_distal");
                    //right_links.push_back("r2/right_little_tip");

                    Eigen::Affine3d link_pose;
                    tf::poseMsgToEigen(request.goal_constraints[0].position_constraints[i].constraint_region.primitive_poses[0], link_pose);

                    Eigen::Vector3d position = link_pose.translation();
                    unsigned int hr_idx = world->findHandrail(position);
                    if (hr_idx < world->numHandrails())
                    {
                        std::string handrail = world->getHandrail(hr_idx)->name;
                        ROS_WARN("%s hand will grasp %s.  Disabling collisions.", left ? "Left" : "Right", handrail.c_str());

                        for(size_t k = 0; k < left_links.size(); ++k)
                            acm.setEntry(left ? left_links[k] : right_links[k], handrail, true);  // true means "allow this collision"
                    }
                    else
                    {
                        ROS_ERROR("Did not detect goal handrail for hand");
                    }
                }
            }
        }
    }
}

double cartesianDistance(const robot_state::RobotState& s1, const robot_state::RobotState& s2, const std::string& link)
{
    const Eigen::Affine3d& frame_a = s1.getGlobalLinkTransform(link);
    const Eigen::Affine3d& frame_b = s2.getGlobalLinkTransform(link);
    return (frame_b.translation() - frame_a.translation()).norm();
}

double orientationDistance(const robot_state::RobotState& s1, const robot_state::RobotState& s2, const std::string& link)
{
    const Eigen::Affine3d& f1 = s1.getGlobalLinkTransform(link);
    const Eigen::Affine3d& f2 = s2.getGlobalLinkTransform(link);

    Eigen::Quaterniond q1(f1.rotation());
    Eigen::Quaterniond q2(f2.rotation());

    double d1 = q1.x()*q2.x() + q1.y()*q2.y() + q1.z()*q2.z() + q1.w()*q2.w();
    double d2 = q1.x()*-q2.x() + q1.y()*-q2.y() + q1.z()*-q2.z() + q1.w()*-q2.w();

    // Do this because computers are terrible at math
    if (d1 < -1.)
        d1 = -1;
    if (d2 < -1.)
        d2 = -1;
    if (d1 > 1.)
        d1 = 1;
    if (d2 > 1.)
        d2 = 1;

    return std::min(acos(d1), acos(d2));
}

// Compute the angular deviation between a point (localVec) on a moving frame (link) relative to a fixed axis.
// assuming localVec and axis are unit vectors
double orientationDeviation(const robot_state::RobotState& state, const Eigen::Vector3d& localVec, const Eigen::Vector3d& axis, std::string link)
{
    Eigen::Vector3d headVec = state.getGlobalLinkTransform(link).rotation() * localVec;
    return acos(headVec.dot(axis));
}

// Aggregate the cartesian distance information for the trajectories
void postRunDistances(const moveit_msgs::MotionPlanRequest& request, const planning_interface::MotionPlanDetailedResponse& response,
                              moveit_benchmarks::BenchmarkExecutor::PlannerRunData& run_data)
{
    if (response.trajectory_.size() == 0)
        return;
    if (response.trajectory_.size() != response.description_.size())
    {
        ROS_WARN("The number of trajectories (%lu) is different from the number of descriptions (%lu)", response.trajectory_.size(), response.description_.size());
        return;
    }

    // The set of links that we will measure distance traveled.  Assumed that the trajectory has many points interpolated along path
    std::vector<std::string> links;
    links.push_back("r2/head");
    //links.push_back("r2/left_palm");
    links.push_back("r2/right_palm");
    links.push_back("r2/left_leg_foot");
    links.push_back("r2/right_leg_foot");
    links.push_back("r2/robot_world"); // waist

    for (std::size_t i = 0; i < response.trajectory_.size(); ++i)
    {
        std::vector<double> cartDistances(links.size(), 0.);
        std::vector<double> ornDistances(links.size(), 0.);

        const robot_trajectory::RobotTrajectory &t = *response.trajectory_[i];
        for (std::size_t j = 1; j < t.getWayPointCount(); ++j)
        {
            for(size_t k = 0; k < links.size(); ++k)
            {
                cartDistances[k] += cartesianDistance(t.getWayPoint(j-1), t.getWayPoint(j), links[k]);
                ornDistances[k] += orientationDistance(t.getWayPoint(j-1), t.getWayPoint(j), links[k]);
            }
        }

        for(size_t j = 0; j < links.size(); ++j)
        {
            std::string link_name = links[j];
            std::size_t pos = link_name.find("/");
            if (pos != std::string::npos) // remove r2/ prefix, the forward slash is not a valid character in benchmark db anyway
                link_name = link_name.substr(pos+1, std::string::npos);

            std::string cdescription("cartesian_distance_" + link_name + "_" + response.description_[i]);
            run_data[cdescription + " REAL"] = boost::lexical_cast<std::string>(cartDistances[j]);

            std::string qdescription("quaternion_distance_" + link_name + "_" + response.description_[i]);
            run_data[qdescription + " REAL"] = boost::lexical_cast<std::string>(ornDistances[j]);
        }

        double totalCartDist = 0.;
        double totalOrnDist = 0.;
        for(size_t j = 0; j < cartDistances.size(); ++j)
        {
            totalCartDist += cartDistances[j];
            totalOrnDist +=  ornDistances[j];
        }

        std::string cdescription("cartesian_distance_head_feet_waist_hand_" + response.description_[i]);
        run_data[cdescription + " REAL"] = boost::lexical_cast<std::string>(totalCartDist);

        std::string qdescription("quaternion_distance_head_feet_waist_hand_" + response.description_[i]);
        run_data[qdescription + " REAL"] = boost::lexical_cast<std::string>(totalOrnDist);
    }

    /*if (response.trajectory_.size() == 0)
        return;
    if (response.trajectory_.size() != response.description_.size())
    {
        ROS_WARN("The number of trajectories (%lu) is different from the number of descriptions (%lu)", response.trajectory_.size(), response.description_.size());
        return;
    }

    for (std::size_t i = 0; i < response.trajectory_.size(); ++i)
    {
        //std::cout << "   Trajectory [" << response.description_[i] << "] has " << response.trajectory_[i]->getWayPointCount() << " states" << std::endl;

        double cart_dist = 0.0;
        const robot_trajectory::RobotTrajectory &t = *response.trajectory_[i];
        for (std::size_t j = 1; j < t.getWayPointCount(); ++j)
            cart_dist += cartesianJointDistance(t.getWayPoint(j-1), t.getWayPoint(j));

        std::string description("cartesian_distance_" + response.description_[i]);
        run_data[description + " REAL"] = boost::lexical_cast<std::string>(cart_dist);
    }*/
}

void postRunOrientationDeviation(const moveit_msgs::MotionPlanRequest& request, const planning_interface::MotionPlanDetailedResponse& response,
                                 moveit_benchmarks::BenchmarkExecutor::PlannerRunData& run_data)
{
    if (response.trajectory_.size() == 0)
        return;
    if (response.trajectory_.size() != response.description_.size())
    {
        ROS_WARN("The number of trajectories (%lu) is different from the number of descriptions (%lu)", response.trajectory_.size(), response.description_.size());
        return;
    }

    std::string link = "r2/head";
    Eigen::Vector3d axis(0,0,1); // the fixed axis to measure angular deviation from.  Should be unit vec.
    Eigen::Vector3d localPt(0,1,0); // point in the frame of 'link' that deviates.  Should be unit vec.

    double deviation = 0.0;
    for (std::size_t i = 0; i < response.trajectory_.size(); ++i)
    {
        const robot_trajectory::RobotTrajectory &t = *response.trajectory_[i];
        for (std::size_t j = 0; j < t.getWayPointCount(); ++j)
        {
            double dev = orientationDeviation(t.getWayPoint(j), localPt, axis, link);
            if (dev > deviation)
                deviation = dev;
        }

        std::string description("angular_deviation_head_" + response.description_[i]);
        run_data[description + " REAL"] = boost::lexical_cast<std::string>(deviation);
    }
}

// double maxRollPitch(const robot_state::RobotState& state, std::string link = "r2/head")
// {
//     const Eigen::Affine3d& frame = state.getGlobalLinkTransform(link);
//     Eigen::Vector3d rpy = frame.rotation().eulerAngles(0,1,2);
//     return std::max(fabs(rpy(0)), fabs(rpy(1)));
// }

// // Aggregate the cartesian distance information for the trajectories
// void postRunHeadRollPitch(const moveit_msgs::MotionPlanRequest& request, const planning_interface::MotionPlanDetailedResponse& response,
//                           moveit_benchmarks::BenchmarkExecutor::PlannerRunData& run_data)
// {
//     if (response.trajectory_.size() == 0)
//         return;
//     if (response.trajectory_.size() != response.description_.size())
//     {
//         ROS_WARN("The number of trajectories (%lu) is different from the number of descriptions (%lu)", response.trajectory_.size(), response.description_.size());
//         return;
//     }

//     for (std::size_t i = 0; i < response.trajectory_.size(); ++i)
//     {

//         const robot_trajectory::RobotTrajectory &t = *response.trajectory_[i];

//         double initialMaxRollAndPitch = maxRollPitch(t.getWayPoint(0));  // initial condition.  Rest will measure deviation from initial max roll/pitch
//         double maxRollAndPitch = 0.0;

//         for (std::size_t j = 0; j < t.getWayPointCount(); ++j)
//         {
//             double maxRP = maxRollPitch(t.getWayPoint(j));
//             double diff = fabs(maxRP - initialMaxRollAndPitch);
//             if (diff > maxRollAndPitch)
//                 maxRollAndPitch = diff;
//         }

//         std::string description("max_head_roll_pitch_" + response.description_[i]);
//         run_data[description + " REAL"] = boost::lexical_cast<std::string>(maxRollAndPitch);
//     }
// }

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
    server.addPostRunEvent(postRunDistances);                                  // compute the distance traveled by a set of links on each path, position and orientation
    server.addPostRunEvent(postRunOrientationDeviation);                       // compute the angular deviation of the head relative to the Z axis
    //server.addPostRunEvent(postRunHeadRollPitch);                              // compute the maximum orientation change by the head in roll or pitch dimensions

    // Running benchmarks
    if (!server.runBenchmarks(opts))
        ROS_ERROR("Failed to run all benchmarks");
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "iss_climbing_benchmark_arms");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    runBenchmarks();
}