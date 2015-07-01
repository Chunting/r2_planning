// Author: Ryan Luna

#ifndef RVIZ_PLANNING_INTERFACE_H_
#define RVIZ_PLANNING_INTERFACE_H_

#include <ros/ros.h>
#include <rviz/panel.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <QObject>
#include <QLabel>
#include <QPushButton>
#include <QComboBox>
#include <QRadioButton>

#include <std_msgs/String.h>
#include "r2_planning_interface/R2Interface.h"

namespace r2rviz
{
    class R2Planning : public rviz::Panel
    {
    Q_OBJECT
    public:
        R2Planning(QWidget* parent = 0);


    protected Q_SLOTS:
        void loadPressed();
        void execPressed();
        void clearPressed();
        void trajectorySelected(const QString&);
        void baseSwitch();
        void recoverPressed();
        void resetPosePressed();
        //void syncToStart();
        //void syncToEnd();

    protected:

        //void syncToPoint(unsigned int index);

        void loadTrajectoryLibrary(const std::string& directory);

        void loadTrajectory(const std::string& filename);
        void setTrajectoryLabelText(const std::string& text);

        void trajectoryStatusUpdate(const std_msgs::String::ConstPtr& status);
        void recoverTrajectoryCallback(const moveit_msgs::RobotTrajectory::ConstPtr& recover);

        bool execute_;

        QLabel* currentTrajectoryLabel_;
        QPushButton* execButton_;
        QComboBox* trajectoryComboBox_;
        QLabel* executionStatusLabel_;
        QPushButton* recoverButton_;

        QRadioButton* leftBaseButton_;
        QRadioButton* rightBaseButton_;
        QRadioButton* noBaseButton_;

        ros::NodeHandle nh_;
        ros::Publisher filenamePublisher_;
        moveit_msgs::RobotTrajectory trajectory_;

        R2Interface r2Interface;

        std::map<QString, boost::filesystem::path> trajectoryLibrary_;

        ros::Publisher changeBasePub_;
        ros::Publisher resetBasePub_;
        ros::Subscriber trajectoryStatusSub_;
        ros::Publisher execTrajectoryPub_;
        ros::Subscriber recoverTrajectorySub_;
    };
}

#endif
