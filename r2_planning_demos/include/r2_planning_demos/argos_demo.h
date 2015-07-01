#include <vector>
#include <Eigen/Dense>
#include "r2_planning_interface/R2Interface.h"

void loadISSModelPlanningScene(R2Interface& interface)
{
    moveit_msgs::CollisionObject iss_model;
    iss_model.operation = moveit_msgs::CollisionObject::ADD;
    iss_model.id = "iss";
    iss_model.header.frame_id = "virtual_world";

    // shape_msgs::Mesh object
    shapes::Mesh* mesh = shapes::createMeshFromResource("package://r2_gazebo/models/iss/meshes/US_Lab_Empty_Rack_noHandrail.dae");
    shapes::ShapeMsg shape_msg;
    if (!shapes::constructMsgFromShape(mesh, shape_msg))
    {
        std::cout << "Error creating mesh message" << std::endl;
        return;
    }
    shape_msgs::Mesh& mesh_msg = boost::get<shape_msgs::Mesh>(shape_msg);
    iss_model.meshes.push_back(mesh_msg);

    delete mesh;

    geometry_msgs::Pose pose;
    pose.position.x = 0.0;
    pose.position.y = 0.0;
    pose.position.z =  0.0;
    pose.orientation.x = 0.7071067811865476;
    pose.orientation.y = 0.0;
    pose.orientation.z = 0.0;
    pose.orientation.w = 0.7071067811865476;

    iss_model.mesh_poses.push_back(pose);
    interface.addObstacleToPlanningScene(iss_model);
}

/// Create a PlanningScene message that approximates the detailed ISS model.
/// Note: The handrails object is only used to infer the size of the
/// bounding box that will approximate the exterior of the ISS.
void loadApproximateISSModelPlanningScene(R2Interface& interface, const std::string& frame="virtual_world")
{
    double minX = -3.5;
    double maxX = 3.5;

    //
    double minY = -1.1;
    double maxY = 0.9; // use 0.9 for more conservative estimate, 1.1 for realistic wall location;

    double minZ = -1.1;
    double maxZ = 1.1;

    double width = 0.1; // add width to the walls to avoid impossibly thin obstacles

    // TOP
    {
        moveit_msgs::CollisionObject top_msg;
        top_msg.operation = moveit_msgs::CollisionObject::ADD;
        top_msg.id = "ISS_overhead";
        top_msg.header.frame_id = frame;

        shape_msgs::SolidPrimitive box_msg;
        box_msg.type = shape_msgs::SolidPrimitive::BOX;
        box_msg.dimensions.push_back(maxX - minX); // X
        box_msg.dimensions.push_back(maxY - minY); // Y
        box_msg.dimensions.push_back(width); // Z

        // Center of the box
        geometry_msgs::Pose box_pose;
        box_pose.position.x = (fabs(maxX) - fabs(minX))/2.0;
        box_pose.position.y = (fabs(maxY) - fabs(minY))/2.0;
        box_pose.position.z = maxZ + width/2.0;
        box_pose.orientation.x = 0;
        box_pose.orientation.y = 0;
        box_pose.orientation.z = 0;
        box_pose.orientation.w = 1;

        top_msg.primitives.push_back(box_msg);
        top_msg.primitive_poses.push_back(box_pose);
        interface.addObstacleToPlanningScene(top_msg);
    }

    // Bottom
    {
        moveit_msgs::CollisionObject bottom_msg;
        bottom_msg.operation = moveit_msgs::CollisionObject::ADD;
        bottom_msg.id = "ISS_Deck";
        bottom_msg.header.frame_id = frame;

        shape_msgs::SolidPrimitive box_msg;
        box_msg.type = shape_msgs::SolidPrimitive::BOX;
        box_msg.dimensions.push_back(maxX - minX); // X
        box_msg.dimensions.push_back(maxY - minY); // Y
        box_msg.dimensions.push_back(width); // Z

        // Center of the box
        geometry_msgs::Pose box_pose;
        box_pose.position.x = (fabs(maxX) - fabs(minX))/2.0;
        box_pose.position.y = (fabs(maxY) - fabs(minY))/2.0;
        box_pose.position.z = (minZ - width/2.0) + 0.10;  // intentionally moving the floor up to avoid collision with handrails
        box_pose.orientation.x = 0;
        box_pose.orientation.y = 0;
        box_pose.orientation.z = 0;
        box_pose.orientation.w = 1;

        bottom_msg.primitives.push_back(box_msg);
        bottom_msg.primitive_poses.push_back(box_pose);
        interface.addObstacleToPlanningScene(bottom_msg);
    }

    // Starboard (+side of Y axis)
    // This wall is wider to avoid colliding with the task panel
    {
        moveit_msgs::CollisionObject port_msg;
        port_msg.operation = moveit_msgs::CollisionObject::ADD;
        port_msg.id = "ISS_starboard";
        port_msg.header.frame_id = frame;

        shape_msgs::SolidPrimitive box_msg;
        box_msg.type = shape_msgs::SolidPrimitive::BOX;
        box_msg.dimensions.push_back(maxX - minX); // X
        box_msg.dimensions.push_back(width * 2.5); // Y
        box_msg.dimensions.push_back(maxZ - minZ); // Z

        // Center of the box
        geometry_msgs::Pose box_pose;
        box_pose.position.x = (fabs(maxX) - fabs(minX))/2.0;
        box_pose.position.y = maxY + (width * 2.5) / 2.0;
        box_pose.position.z = (fabs(maxZ) - fabs(minZ))/2.0;
        box_pose.orientation.x = 0;
        box_pose.orientation.y = 0;
        box_pose.orientation.z = 0;
        box_pose.orientation.w = 1;

        port_msg.primitives.push_back(box_msg);
        port_msg.primitive_poses.push_back(box_pose);
        interface.addObstacleToPlanningScene(port_msg);
    }

    // Port (-side of Y axis)
    // This one gets weird because of R2's closet
    {
        /*moveit_msgs::CollisionObject port_msg;
        port_msg.operation = moveit_msgs::CollisionObject::ADD;
        port_msg.id = "ISS_port";
        port_msg.header.frame_id = frame;

        shape_msgs::SolidPrimitive box_msg;
        box_msg.type = shape_msgs::SolidPrimitive::BOX;
        box_msg.dimensions.push_back(maxX - minX - 1.20); // X
        box_msg.dimensions.push_back(width); // Y
        box_msg.dimensions.push_back(maxZ - minZ); // Z

        // Center of the box
        geometry_msgs::Pose box_pose;
        box_pose.position.x = (fabs(maxX) - fabs(minX) + 1.20)/2.0;
        box_pose.position.y = minY - width/2.0;
        box_pose.position.z = (fabs(maxZ) - fabs(minZ))/2.0;
        box_pose.orientation.x = 0;
        box_pose.orientation.y = 0;
        box_pose.orientation.z = 0;
        box_pose.orientation.w = 1;

        port_msg.primitives.push_back(box_msg);
        port_msg.primitive_poses.push_back(box_pose);
        interface.addObstacleToPlanningScene(port_msg);*/

        /*// CLOSET
        // side 1
        moveit_msgs::CollisionObject closet_side;
        closet_side.operation = moveit_msgs::CollisionObject::ADD;
        closet_side.id = "ISS_port_closet1";
        closet_side.header.frame_id = frame;

        //shape_msgs::SolidPrimitive box_msg;
        //box_msg.type = shape_msgs::SolidPrimitive::BOX;
        box_msg.dimensions[0] = 0.0;          // X
        box_msg.dimensions[1] = 1.0;          // Y
        box_msg.dimensions[2] = maxZ - minZ - 0.3;  // Z

        // Center of the box
        //geometry_msgs::Pose box_pose;
        box_pose.position.x = (minX + 0.80);
        box_pose.position.y = minY - 0.5;
        box_pose.position.z = (fabs(maxZ) - fabs(minZ))/2.0;
        box_pose.orientation.x = 0;
        box_pose.orientation.y = 0;
        box_pose.orientation.z = 0;
        box_pose.orientation.w = 1;

        closet_side.primitives.push_back(box_msg);
        closet_side.primitive_poses.push_back(box_pose);
        interface.addObstacleToPlanningScene(closet_side);

        // side 2
        closet_side.primitives.clear();
        closet_side.primitive_poses.clear();

        closet_side.id = "ISS_port_closet2";
        box_msg.dimensions[0] = 0.0;                // X size
        box_msg.dimensions[1] = 1.0;                // Y size
        box_msg.dimensions[2] = maxZ - minZ - 0.3;  // Z size
        box_pose.position.x = (minX - 0.20);        // position x

        closet_side.primitives.push_back(box_msg);
        closet_side.primitive_poses.push_back(box_pose);
        interface.addObstacleToPlanningScene(closet_side);

        // rear
        closet_side.primitives.clear();
        closet_side.primitive_poses.clear();

        closet_side.id = "ISS_port_closet3";
        box_msg.dimensions[0] = 1.0;                // X size
        box_msg.dimensions[1] = 0.0;                // Y size
        box_msg.dimensions[2] = maxZ - minZ - 0.3;  // Z size
        box_pose.position.x = (minX + 0.30);        // position x
        box_pose.position.y = minY - 1.0;

        closet_side.primitives.push_back(box_msg);
        closet_side.primitive_poses.push_back(box_pose);
        interface.addObstacleToPlanningScene(closet_side);

        // bottom
        closet_side.primitives.clear();
        closet_side.primitive_poses.clear();

        closet_side.id = "ISS_port_closet4";
        box_msg.dimensions[0] = 1.0;                // X size
        box_msg.dimensions[1] = 1.0;                // Y size
        box_msg.dimensions[2] = 0.0;                // Z size
        box_pose.position.x = (minX + 0.30);        // position x
        box_pose.position.y = minY - 0.5;
        box_pose.position.z = minZ + 0.15;

        closet_side.primitives.push_back(box_msg);
        closet_side.primitive_poses.push_back(box_pose);
        interface.addObstacleToPlanningScene(closet_side);

        // top
        closet_side.primitives.clear();
        closet_side.primitive_poses.clear();

        closet_side.id = "ISS_port_closet5";
        box_pose.position.z = maxZ - 0.15;

        closet_side.primitives.push_back(box_msg);
        closet_side.primitive_poses.push_back(box_pose);
        interface.addObstacleToPlanningScene(closet_side);*/
    }
}