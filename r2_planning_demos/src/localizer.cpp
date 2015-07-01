// Dead reckoning localization for body pose of R2
// Requires operator to update the base link at runtime
// Author: Ryan Luna
// May 2015

#include "ros/ros.h"
#include "r2_planning_demos/argos_poses.h"
#include "r2_planning_interface/R2Interface.h"
#include <eigen_conversions/eigen_msg.h>

#include <boost/thread/mutex.hpp>

#include <std_msgs/String.h>

static const std::string BASE_FRAME_TOPIC = "/change_base_frame";
static const std::string RESET_FRAME_TOPIC = "/reset_base_frame";


/// Continually broadcasts the location of the robot in a global frame using dead reckoning.
class Localizer
{
public:
    Localizer(const std::string& base, unsigned int freq=10) : interface("robot_description"), rate(freq)
    {
        interface.setDefaultJointPositions();
        workState = interface.allocRobotState();
        workState->setToDefaultValues();
        usleep(500000);

        baseFrameSub = nh.subscribe<std_msgs::String>(BASE_FRAME_TOPIC, 0, boost::bind(&Localizer::changeBaseCallback, this, _1));
        resetFrameSub = nh.subscribe<std_msgs::String>(RESET_FRAME_TOPIC, 0, boost::bind(&Localizer::resetBaseCallback, this, _1));
        setBase(base);

        kill = false;
        gazebo_ = false;
    }

    ~Localizer()
    {
        kill = true;
        localizer.join();
    }

    void start(bool gazebo)
    {
        stop(); // make sure we're stopped
        kill = false;

        gazebo_ = gazebo;

        if (gazebo_)
            interface.gazeboLocalization(true);
        else
            interface.fakeLocalization(true);
        localizer = boost::thread(&Localizer::localizerThread, this);
    }

    void stop()
    {
        kill = true;
        localizer.join();
    }

    const std::string& getBase() const
    {
        return baseLink;
    }

    void setBase(const std::string& base)
    {
        baseMutex.lock();

        baseLink = base;

        if (baseLink.size() > 0)
        {
            basePose = interface.getCurrentRobotState().getGlobalLinkTransform(baseLink);
            ROS_INFO("Localizer: Base link '%s'", baseLink.c_str());
        }
        else
        {
            basePose = Eigen::Affine3d::Identity();
            ROS_INFO("Localizer: Base link 'UNSET'");
        }

        baseMutex.unlock();
    }

    void setWorldTransform(const Eigen::Affine3d& worldFrame)
    {
        baseMutex.lock();
        interface.setWorldTransform(worldFrame);
        usleep(100000);
        baseMutex.unlock();
    }

    R2Interface& getR2Interface()
    {
        return interface;
    }

    void resetBase()
    {
        baseMutex.lock();
        baseLink = ""; // this avoids a race condition on when the mutex is acquired

        // Manually setting the world transform
        Eigen::Affine3d world_transform;
        issLegsUnstowWorldTransform(world_transform);
        interface.setWorldTransform(world_transform);

        // This next bit of code only works when the robot is simulated...
        std::map<std::string, double> jointAngles;
        issLegsUnstowPose(jointAngles);
        stowArmPose(jointAngles);
        interface.setJointPositions(jointAngles);

        baseMutex.unlock();
    }

protected:
    void localizerThread()
    {
        Eigen::Affine3d worldFrame;

        ROS_INFO("Localizer starting");

        while (!kill && ros::ok())
        {
            // No base to localize around...
            if (baseLink != "")
            {
                baseMutex.lock();
                *(workState.get()) = interface.getCurrentRobotState();
                getWorldFrame((*workState.get()), worldFrame);
                interface.setWorldTransform(worldFrame);
                baseMutex.unlock();
            }

            rate.sleep();
        }
        ROS_INFO("Localizer thread killed");
    }

    void changeBaseCallback(const std_msgs::String::ConstPtr& msg)
    {
        setBase(msg->data);
    }

    void resetBaseCallback(const std_msgs::String::ConstPtr& /*msg*/)
    {
        resetBase();
    }

    void getWorldFrame(robot_state::RobotState& state, Eigen::Affine3d& worldFrame)
    {
        const moveit::core::LinkModel* parent = state.getLinkModel(baseLink);
        const moveit::core::LinkModel* child  = NULL;

        worldFrame = basePose;
        // backward kinematics to find out the pose of the world
        // dead reckoning
        while (parent->getParentJointModel()->getParentLinkModel())
        {
            child = parent;
            parent = parent->getParentJointModel()->getParentLinkModel();
            worldFrame = worldFrame * (child->getJointOriginTransform() * state.getJointTransform(child->getParentJointModel())).inverse();
        }
    }

    ros::NodeHandle nh;
    R2Interface interface;

    ros::Subscriber baseFrameSub;
    ros::Subscriber resetFrameSub;

    std::string baseLink;
    Eigen::Affine3d basePose;
    boost::mutex baseMutex;

    robot_state::RobotStatePtr workState;
    bool kill;
    boost::thread localizer;
    ros::Rate rate;

    bool gazebo_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "localizer");
    ros::AsyncSpinner spinner(0); // use # threads = num cores
    spinner.start();

    ros::NodeHandle nh("localizer");

    bool gazebo = false;
    if (nh.hasParam("gazebo"))
        nh.getParam("gazebo", gazebo);

    Localizer localizer("");
    localizer.resetBase();
    localizer.start(gazebo);

    ros::spin();
}
