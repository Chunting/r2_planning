#include <ros/ros.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <std_msgs/String.h>
#include <string>
#include <fstream>

#include "r2_planning_interface/R2Interface.h"

static const std::string EXEC_STATUS_TOPIC = "/trajectory_exec_status";
static const std::string EXEC_TRAJECTORY_TOPIC = "/execute_my_trajectory";
static const std::string RECOVER_TRAJECTORY_TOPIC = "/recover_my_trajectory";

class MyRobotTrajectory : public robot_trajectory::RobotTrajectory
{
public:
    MyRobotTrajectory(const robot_model::RobotModelConstPtr &kmodel, const std::string &group) : robot_trajectory::RobotTrajectory(kmodel, group)
    {
    }

    bool recoverTrajectory(const robot_state::RobotState& current_state, moveit_msgs::RobotTrajectory& trajectory)
    {
        // find closest state in the trajectory
        if (getWayPointCount() == 0)
        {
            ROS_ERROR("There are no waypoints in the trajectory to recover from");
            return false;
        }

        size_t closestIdx = 0;
        double dist = current_state.distance(getFirstWayPoint());

        for(size_t i = 1; i < getWayPointCount(); ++i)
        {
            double d = current_state.distance(getWayPoint(i));
            if (d < dist)
            {
                closestIdx = i;
                dist = d;
            }
        }

        ROS_INFO("Recovering trajectory.  Closest point is %lu of %lu, a distance of %f from current state", closestIdx+1, getWayPointCount(), dist);
        getRobotTrajectoryMsg(trajectory);
        if (closestIdx > 0)
        {
            trajectory.joint_trajectory.points.erase(trajectory.joint_trajectory.points.begin(), trajectory.joint_trajectory.points.begin() + closestIdx);
            if (trajectory.multi_dof_joint_trajectory.points.size() > closestIdx)
                trajectory.multi_dof_joint_trajectory.points.erase(trajectory.multi_dof_joint_trajectory.points.begin(), trajectory.multi_dof_joint_trajectory.points.begin() + closestIdx);
        }

        return true;
    }
};

class Executor
{
public:
    Executor(bool simulate = false) : interface_("robot_description")
    {
        execStatusSub_ = nh_.advertise<std_msgs::String>(EXEC_STATUS_TOPIC, 0);
        recoverTrajectorySub_ = nh_.advertise<moveit_msgs::RobotTrajectory>(RECOVER_TRAJECTORY_TOPIC, 0);
        execTrajectorySub_ = nh_.subscribe<moveit_msgs::RobotTrajectory>(EXEC_TRAJECTORY_TOPIC, 0, boost::bind(&Executor::execTrajectory, this, _1));

        simulate_ = simulate;
    }

protected:

    void execTrajectory(const moveit_msgs::RobotTrajectory::ConstPtr& msg)
    {
        moveit_msgs::RobotTrajectory trajectory = *(msg.get());
        replaceFirstPointWithCurrentJoints(trajectory.joint_trajectory);

        if (simulate_)
        {
            ROS_INFO("Simulating trajectory");
            interface_.simulateTrajectory(trajectory, false);  // false = don't simulate body moving.  The localizer should do this

            std_msgs::String statusMsg;
            statusMsg.data = "Trajectory simulated";
            execStatusSub_.publish(statusMsg);
        }
        else
        {
            // hack, but probably not that important
            const std::string group("legs");

            // Remove the body pose information from the trajectory.  This is not an actuable joint.
            trajectory.multi_dof_joint_trajectory.joint_names.clear();
            trajectory.multi_dof_joint_trajectory.points.clear();

            MyRobotTrajectory trajectoryRecovery(interface_.getCurrentRobotState().getRobotModel(), group);
            trajectoryRecovery.setRobotTrajectoryMsg(interface_.getCurrentRobotState(), *(msg.get()));

            int val = interface_.executeTrajectory(trajectory, true);  // true = block until the controller says execution is finished (or something bad happens)
            if (val == moveit_msgs::MoveItErrorCodes::PREEMPTED)
            {
                trajectoryRecovery.recoverTrajectory(interface_.getCurrentRobotState(), trajectory);

                // Send back recovered trajectory
                recoverTrajectorySub_.publish(trajectory);

                // And a message
                std_msgs::String preemptMsg;
                preemptMsg.data = "Preempt detected. Trajectory recovered";
                execStatusSub_.publish(preemptMsg);
            }
            else
            {
                std_msgs::String statusMsg;
                statusMsg.data = "Trajectory executed";
                execStatusSub_.publish(statusMsg);
            }
        }

    }

    void replaceFirstPointWithCurrentJoints(trajectory_msgs::JointTrajectory& trajectory)
    {
        if (trajectory.points.size() == 0)
            return;

        const robot_state::RobotState& currentState = interface_.getCurrentRobotState();
        for(size_t i = 0; i < trajectory.joint_names.size(); ++i)
            trajectory.points[0].positions[i] = currentState.getJointPositions(trajectory.joint_names[i])[0];
    }

    bool simulate_;
    ros::Publisher execStatusSub_;
    ros::Publisher recoverTrajectorySub_;
    ros::Subscriber execTrajectorySub_;
    ros::NodeHandle nh_;
    R2Interface interface_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "R2_executor");
    ros::AsyncSpinner spinner(0); // use # threads = num cores
    spinner.start();

    ros::NodeHandle nh("executor");

    bool execute = false;
    if (nh.hasParam("execute"))
        nh.getParam("execute", execute);
    else
        ROS_WARN("No execute param given");

    if (execute)
        ROS_WARN("You are executing trajectories FOR REALS");
    else
        ROS_WARN("Will only simulate trajectories.  No execution");

    Executor executor(!execute);
    ros::spin();
}