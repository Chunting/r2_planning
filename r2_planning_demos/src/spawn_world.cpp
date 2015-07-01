// Spawns ISS module and handrail configuration in RViz using marker messages
// Author: Ryan Luna
// May 2015

#include "r2_planning_interface/ISSWorld.h"
#include <moveit/move_group/capability_names.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "spawn_world");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // Wait until move_group is running to load the world
    // Timeout after 2 mins
    bool good = ros::service::waitForService(move_group::PLANNER_SERVICE_NAME, ros::Duration(120));
    if (!good)
        ROS_ERROR("Spawn_world: Timeout waiting for move_group to come up.  Spawning anyway.");

    ISSWorld world(ros::this_node::getName());
    world.visualizeWorld();
    sleep(1);  // make sure publisher publishes everything before dying
}