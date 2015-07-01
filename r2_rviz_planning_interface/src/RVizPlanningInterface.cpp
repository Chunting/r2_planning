// Author: Ryan Luna

#include "r2_rviz_planning_interface/RVizPlanningInterface.h"

#include <QFileDialog>
#include <QGridLayout>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QCheckBox>
#include <QGroupBox>

#include <QSpinBox>
#include <fstream>

#include <std_msgs/String.h>
#include <eigen_conversions/eigen_msg.h>

namespace fs = boost::filesystem;

static const std::string BASE_FRAME_TOPIC = "/change_base_frame";
static const std::string EXEC_STATUS_TOPIC = "/trajectory_exec_status";
static const std::string EXEC_TRAJECTORY_TOPIC = "/execute_my_trajectory";
static const std::string RECOVER_TRAJECTORY_TOPIC = "/recover_my_trajectory";
static const std::string RESET_FRAME_TOPIC = "/reset_base_frame";

namespace r2rviz
{

R2Planning::R2Planning(QWidget* parent) : rviz::Panel(parent), r2Interface("robot_description")
{
    //// TRAJECTORY EXECUTION STUFF ////
    QGroupBox* trajGroup = new QGroupBox("Trajectory selection");

    currentTrajectoryLabel_ = new QLabel();
    currentTrajectoryLabel_->setTextFormat(Qt::RichText);
    setTrajectoryLabelText("No trajectory library loaded");

    QPushButton* loadButton = new QPushButton("Load", this);
    connect(loadButton, SIGNAL(clicked()), this, SLOT(loadPressed()));

    execButton_ = new QPushButton("Execute", this);
    execButton_->setEnabled(false);
    connect(execButton_, SIGNAL(clicked()), this, SLOT(execPressed()));

    QGridLayout* layout = new QGridLayout();
    layout->addWidget(loadButton, 0, 0);
    layout->addWidget(currentTrajectoryLabel_, 0, 1, 1, 2);

    trajectoryComboBox_ = new QComboBox();
    layout->addWidget(execButton_, 1, 2);
    layout->addWidget(trajectoryComboBox_, 1, 0, 1, 2);

    //QPushButton* syncStartButton = new QPushButton("Sync to start", this);
    //QPushButton* syncEndButton = new QPushButton("Sync to end", this);
    //layout->addWidget(syncStartButton, 2, 0);
    //layout->addWidget(syncEndButton, 2, 1);
    //connect(syncStartButton, SIGNAL(clicked()), this, SLOT(syncToStart()));
    //connect(syncEndButton, SIGNAL(clicked()), this, SLOT(syncToEnd()));

    connect(trajectoryComboBox_, SIGNAL(currentIndexChanged(QString)), this, SLOT(trajectorySelected(QString)));

    trajGroup->setLayout(layout);

    //// TRAJECTORY RECOVERY ////

    executionStatusLabel_ = new QLabel("No trajectory being executed");
    recoverButton_ = new QPushButton("Recover");
    recoverButton_->setEnabled(false);
    connect(recoverButton_, SIGNAL(clicked()), this, SLOT(recoverPressed()));

    QHBoxLayout* hlayout = new QHBoxLayout();
    hlayout->addWidget(executionStatusLabel_);
    hlayout->addWidget(recoverButton_);

    QGroupBox* execGroup = new QGroupBox("Trajectory status");
    execGroup->setLayout(hlayout);


    //// BASE FRAME STUFF ////
    QGroupBox* baseGroup = new QGroupBox("Base frame");

    leftBaseButton_ = new QRadioButton("Left gripper");
    rightBaseButton_ = new QRadioButton("Right gripper");
    noBaseButton_ = new QRadioButton("World");

    QPushButton* resetButton = new QPushButton("Reset Position");

    QHBoxLayout* buttonLayout = new QHBoxLayout();
    buttonLayout->addWidget(leftBaseButton_);
    buttonLayout->addWidget(rightBaseButton_);
    buttonLayout->addWidget(noBaseButton_);
    buttonLayout->addWidget(resetButton);

    connect(leftBaseButton_, SIGNAL(clicked()), this, SLOT(baseSwitch()));
    connect(rightBaseButton_, SIGNAL(clicked()), this, SLOT(baseSwitch()));
    connect(noBaseButton_, SIGNAL(clicked()), this, SLOT(baseSwitch()));
    connect(resetButton, SIGNAL(clicked()), this, SLOT(resetPosePressed()));
    //rightBaseButton_->setChecked(true);
    //baseSwitch(); // set base now.  Not invoked by setChecked above

    baseGroup->setLayout(buttonLayout);

    QVBoxLayout* panelLayout = new QVBoxLayout();
    panelLayout->addWidget(trajGroup);
    panelLayout->addWidget(execGroup);
    panelLayout->addWidget(baseGroup);
    setLayout(panelLayout);

    changeBasePub_ = nh_.advertise<std_msgs::String>(BASE_FRAME_TOPIC, 0);

    trajectoryStatusSub_ = nh_.subscribe<std_msgs::String>(EXEC_STATUS_TOPIC, 0, boost::bind(&R2Planning::trajectoryStatusUpdate, this, _1));
    execTrajectoryPub_ = nh_.advertise<moveit_msgs::RobotTrajectory>(EXEC_TRAJECTORY_TOPIC, 0);
    recoverTrajectorySub_ = nh_.subscribe<moveit_msgs::RobotTrajectory>(RECOVER_TRAJECTORY_TOPIC, 0, boost::bind(&R2Planning::recoverTrajectoryCallback, this, _1));
    resetBasePub_ = nh_.advertise<std_msgs::String>(RESET_FRAME_TOPIC, 0);
}

void R2Planning::setTrajectoryLabelText(const std::string& text)
{
    // QString labelText = "<b>";
    // labelText.append(text.c_str());
    // labelText.append("</b>");
    // currentTrajectoryLabel_->setText(labelText);

    currentTrajectoryLabel_->setText(QString(text.c_str()));
}

void R2Planning::loadPressed()
{
    QString dir = "/home";
    QString dname = QFileDialog::getExistingDirectory(this, "Load trajectory library", dir);
    if(!dname.isEmpty())
    {
        setTrajectoryLabelText(dname.toStdString());
        loadTrajectoryLibrary(dname.toStdString());
    }
}

/*
void R2Planning::syncToStart()
{
    if (trajectory_.joint_trajectory.joint_names.size() > 0 && trajectory_.joint_trajectory.points.size() > 0)
        syncToPoint(0);
}

void R2Planning::syncToEnd()
{
    if (trajectory_.joint_trajectory.joint_names.size() > 0 && trajectory_.joint_trajectory.points.size() > 0)
        syncToPoint(trajectory_.joint_trajectory.points.size()-1);
}

// WARNING: This code is not quite right.
// This code creates a new trajectory to the point in trajectory_ at index,
// interpolating several intermediate points along the way.  There is an error
// in this computation - the fixed base is not respected.  Need to adjust
// this code to figure out body pose using joint angles rather than linearly
// interpolating it.
void R2Planning::syncToPoint(unsigned int index)
{
    if (index >= trajectory_.joint_trajectory.points.size() ||
        index >= trajectory_.multi_dof_joint_trajectory.points.size())
    {
        ROS_ERROR("syncToPoint: index is out of bounds");
        return;
    }

    trajectory_msgs::JointTrajectoryPoint jtEnd = trajectory_.joint_trajectory.points[index];
    trajectory_msgs::MultiDOFJointTrajectoryPoint mdjtEnd = trajectory_.multi_dof_joint_trajectory.points[index];

    unsigned int pts = 3; // number of points in the interpolated trajectory, excluding endpoints

    robot_state::RobotStatePtr currentState = r2Interface.allocRobotState();
    *(currentState.get()) = r2Interface.getCurrentRobotState();
    robot_state::RobotStatePtr finalState = r2Interface.allocRobotState();
    *(finalState.get()) = *(currentState.get());

    const moveit::core::JointModelGroup* left_leg = currentState->getJointModelGroup("left_leg");
    const moveit::core::JointModelGroup* right_leg = currentState->getJointModelGroup("right_leg");

    for(size_t i = 0; i < trajectory_.joint_trajectory.joint_names.size(); ++i)
        finalState->setJointPositions(trajectory_.joint_trajectory.joint_names[i], &trajectory_.joint_trajectory.points[index].positions[i]);
    for(size_t i = 0; i < trajectory_.multi_dof_joint_trajectory.joint_names.size(); ++i)
    {
        geometry_msgs::Transform frameMsg = trajectory_.multi_dof_joint_trajectory.points[index].transforms[i];

        std::vector<double> xyzq(7, 0.0);
        xyzq[0] = frameMsg.translation.x;
        xyzq[1] = frameMsg.translation.y;
        xyzq[2] = frameMsg.translation.z;

        xyzq[3] = frameMsg.rotation.x;
        xyzq[4] = frameMsg.rotation.y;
        xyzq[5] = frameMsg.rotation.z;
        xyzq[6] = frameMsg.rotation.w;

        finalState->setJointPositions(trajectory_.multi_dof_joint_trajectory.joint_names[i], &xyzq[0]);
    }
    finalState->update(true);

    // Create new trajectory points and workspace
    robot_state::RobotStatePtr workState = r2Interface.allocRobotState();
    *(workState.get()) = *(currentState.get());
    trajectory_.joint_trajectory.points.resize(pts+2);
    trajectory_.multi_dof_joint_trajectory.points.resize(pts+2);

    // interpolate a few points along the way for visualization
    for(unsigned int i = 0; i <= pts; ++i)
    {
        trajectory_.multi_dof_joint_trajectory.points[i].transforms.resize(trajectory_.multi_dof_joint_trajectory.joint_names.size());
        trajectory_.joint_trajectory.points[i].positions.resize(trajectory_.joint_trajectory.joint_names.size());
        trajectory_.joint_trajectory.points[i].velocities.assign(trajectory_.joint_trajectory.joint_names.size(), 0);
        trajectory_.joint_trajectory.points[i].accelerations.assign(trajectory_.joint_trajectory.joint_names.size(), 0);

        double t = (double)i / (double)(pts+1);
        currentState->interpolate(*(finalState.get()), t, *(workState.get()), left_leg);
        currentState->interpolate(*(finalState.get()), t, *(workState.get()), right_leg);
        workState->update();

        for(size_t j = 0; j < trajectory_.joint_trajectory.joint_names.size(); ++j)
            trajectory_.joint_trajectory.points[i].positions[j] = workState->getJointPositions(trajectory_.joint_trajectory.joint_names[j])[0];
        for(size_t j = 0; j < trajectory_.multi_dof_joint_trajectory.joint_names.size(); ++j)
        {
            geometry_msgs::Transform frameMsg;
            const Eigen::Affine3d& frame = workState->getGlobalLinkTransform(workState->getJointModel(trajectory_.multi_dof_joint_trajectory.joint_names[j])->getChildLinkModel()->getName());

            tf::transformEigenToMsg(frame, frameMsg);
            trajectory_.multi_dof_joint_trajectory.points[i].transforms[j] = frameMsg;
        }

        trajectory_.joint_trajectory.points[i].time_from_start = ros::Duration(i);
        trajectory_.multi_dof_joint_trajectory.points[i].time_from_start = ros::Duration(i);
    }

    // Last point is desired configuration
    trajectory_.joint_trajectory.points[pts+1] = jtEnd;
    trajectory_.multi_dof_joint_trajectory.points[pts+1] = mdjtEnd;

    trajectory_.joint_trajectory.points.back().time_from_start = ros::Duration(pts+1);
    trajectory_.multi_dof_joint_trajectory.points.back().time_from_start = ros::Duration(pts+1);

    // Show trajectory
    visualization.viewTrajectory(r2Interface.getCurrentRobotState(), trajectory_);

    recoverButton_->setEnabled(false);
    execButton_->setEnabled(true);

    executionStatusLabel_->setText("Press execute to sync to the point");
}*/

void R2Planning::resetPosePressed()
{
    noBaseButton_->setChecked(true);
    //baseSwitch(); // set base now.  Not invoked by setChecked above

    std_msgs::String resetMsg;
    resetMsg.data = "RESET";
    resetBasePub_.publish(resetMsg);
}

void R2Planning::loadTrajectoryLibrary(const std::string& directory)
{
    fs::path path(directory);

    if (!fs::exists(path))
    {
        ROS_ERROR("'%s' does not exist", directory.c_str());
        return;
    }

    if (!fs::is_directory(path))
    {
        ROS_ERROR("'%s' is not a directory", directory.c_str());
        return;
    }

    trajectoryLibrary_.clear();
    trajectoryComboBox_->clear();

    fs::directory_iterator iter(path);
    while(iter != fs::directory_iterator())
    {
        if (fs::is_regular_file(iter->path()))
        {
            QString fname(iter->path().filename().generic_string().c_str());
            if (fname.endsWith(".trajectory"))
                trajectoryLibrary_[fname] = iter->path();
        }
        iter++;
    }

    // Add items in sorted order.  Also associate the items with their path
    for(std::map<QString, boost::filesystem::path>::const_iterator it = trajectoryLibrary_.begin(); it != trajectoryLibrary_.end(); ++it)
        trajectoryComboBox_->addItem(it->first);

    if (trajectoryLibrary_.size() > 0)
        trajectoryComboBox_->setCurrentIndex(0);
}

void R2Planning::loadTrajectory(const std::string& filename)
{
    moveit_msgs::RobotTrajectory traj;

    // Reading trajectory
    std::ifstream fin;
    fin.open(filename.c_str(), std::ios::in | std::ios::binary);

    if (!fin)
    {
        ROS_ERROR("Failed to open '%s'", filename.c_str());
        return;
    }

    fin.seekg(0, std::ios::end);
    std::streampos end = fin.tellg();
    fin.seekg(0, std::ios::beg);
    std::streampos begin = fin.tellg();

    uint32_t size = end - begin;
    ROS_INFO("Loading trajectory '%s'", filename.c_str());
    ROS_DEBUG("Deserializing trajectory of length %u", size);
    boost::shared_array<uint8_t> buffer(new uint8_t[size]);
    fin.read((char*) buffer.get(), size);
    fin.close();

    ros::serialization::IStream stream(buffer.get(), size);
    ros::serialization::deserialize(stream, traj);

    // Trajectory read successfully.
    trajectory_ = traj;
    r2Interface.viewTrajectory(traj);
    execButton_->setEnabled(true);
    recoverButton_->setEnabled(false);
}

// Callback when user selects a different trajectory file
void R2Planning::trajectorySelected(const QString& selected)
{
    if (trajectoryLibrary_.size() == 0)
        return;

    fs::path path = trajectoryLibrary_[selected];
    loadTrajectory(path.generic_string());
}

void R2Planning::baseSwitch()
{
    if (leftBaseButton_->isChecked())
    {
        std_msgs::String msg;
        msg.data = "r2/left_leg/gripper/tip";
        changeBasePub_.publish(msg);
    }
    else if (rightBaseButton_->isChecked())
    {
        std_msgs::String msg;
        msg.data = "r2/right_leg/gripper/tip";
        changeBasePub_.publish(msg);
    }
    else
    {
        std_msgs::String msg;
        msg.data = "r2/robot_world";
        changeBasePub_.publish(msg);
    }
}

void R2Planning::execPressed()
{
    executionStatusLabel_->setText("Executing trajectory...");
    execTrajectoryPub_.publish(trajectory_);
}

void R2Planning::clearPressed()
{
    r2Interface.clearTrajectory();
    trajectory_.joint_trajectory.joint_names.clear();
    trajectory_.multi_dof_joint_trajectory.joint_names.clear();

    setTrajectoryLabelText("No trajectory loaded");

    execButton_->setEnabled(false);
}

void R2Planning::trajectoryStatusUpdate(const std_msgs::String::ConstPtr& status)
{
    executionStatusLabel_->setText(QString(status->data.c_str()));
}

void R2Planning::recoverTrajectoryCallback(const moveit_msgs::RobotTrajectory::ConstPtr& recover)
{
    trajectory_ = *(recover.get());

    r2Interface.viewTrajectory(trajectory_);
    recoverButton_->setEnabled(true);
    execButton_->setEnabled(false);
}

void R2Planning::recoverPressed()
{
    executionStatusLabel_->setText("Executing recovery trajectory...");
    recoverButton_->setEnabled(false);
    execTrajectoryPub_.publish(trajectory_);
}

} // namespace r2rviz

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(r2rviz::R2Planning, rviz::Panel)