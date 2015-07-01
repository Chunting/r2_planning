// Class that reads in an ISS world representation consisting of an ISS mesh model and a set of handrails
// Can also visualize the world in RViz or create a collision representation of the world for planning
// Author: Ryan Luna
// June 2015

#ifndef R2_PLANNING_INTERFACE_ISS_WORLD_H_
#define R2_PLANNING_INTERFACE_ISS_WORLD_H_

#include <ros/ros.h>
#include <string>
#include <visualization_msgs/Marker.h>
#include <moveit/move_group/capability_names.h>
#include <eigen_conversions/eigen_msg.h>
#include <geometric_shapes/shapes.h>
#include <geometric_shapes/shape_operations.h>
#include <moveit_msgs/CollisionObject.h>

// This is the marker topic where the mesh resources are published
static const char* const RVIZ_MARKER_TOPIC = "/visualization_marker";

// Class containing the representation of the world
class ISSWorld
{
public:
    struct Handrail
    {
        std::string name;

        Eigen::Affine3d pose;
        std::string frame;

        std::string group;
        std::string mesh_resource;
    };

    ISSWorld(const std::string& ns = ros::this_node::getName()) : nh_(ns)
    {
        visPublisher_ = nh_.advertise<visualization_msgs::Marker>(RVIZ_MARKER_TOPIC, 0);
        loadISSFromParam();
        loadHandrailsFromParam();
        sleep(1); // make sure publisher connects
    }

    ~ISSWorld()
    {
        for(size_t i = 0; i < handrails_.size(); ++i)
            delete handrails_[i];
    }

    unsigned int numHandrails() const
    {
        return handrails_.size();
    }

    const std::vector<Handrail*>& getHandrails() const
    {
        return handrails_;
    }

    const Handrail* getHandrail(unsigned int idx) const
    {
        return handrails_[idx];
    }

    const Handrail* getHandrail(const std::string& name) const
    {
        std::map<std::string, Handrail*>::const_iterator it = handrailMap_.find(name);
        if (it == handrailMap_.end())
        {
            ROS_ERROR("No handrail with name %s", name.c_str());
            return NULL;
        }
        return it->second;
    }

    // See if there is a handrail at the given position
    unsigned int findHandrail(const Eigen::Vector3d& position) const
    {
        for(unsigned int i = 0; i < handrails_.size(); ++i)
        {
            if (insideHandrail(handrails_[i], position))
                return i;
        }

        return std::numeric_limits<unsigned int>::max();
    }

    Eigen::Vector3d getHandrailMidpoint(const Handrail* hr) const
    {
        std::map<std::string, Eigen::Vector3d>::const_iterator it = meshExtents_.find(hr->mesh_resource);
        if (it == meshExtents_.end())
            throw;

        // Must rotate extents based on current pose of handrail
        Eigen::Vector3d extents = hr->pose * getMoveItCollisionOffset() * it->second;
        Eigen::Vector3d diff = extents - hr->pose.translation();
        return hr->pose.translation() + 0.5 * diff;
    }

    // Return a collision object message for the given handrail index
    // This is used for planning/collision purposes (as opposed to visualization)
    void getHandrailCollisionObjectMessage(unsigned int idx, moveit_msgs::CollisionObject& handrail_msg) const
    {
        if (const Handrail* hr = getHandrail(idx))
            getCollisionObjectMessage(hr->name, hr->mesh_resource, hr->pose * getMoveItCollisionOffset(), hr->frame, handrail_msg);
        else
            ROS_WARN("Failed to retrieve handrail with index %u", idx);
    }

    // Return a collision object message for the ISS model
    // This is used for planning/collision purposes (as opposed to visualization)
    void getISSCollisionObjectMessage(moveit_msgs::CollisionObject& iss_msg) const
    {
        getCollisionObjectMessage(issName_, issURI_, issPose_ * getMoveItCollisionOffset(), issPoseFrame_, iss_msg);
    }

    // Visualize the ISS and handrails in RViz
    void visualizeWorld()
    {
        visualizeISS();
        visualizeHandrails();
    }

    // Visualize the ISS in RViz
    void visualizeISS()
    {
        visualization_msgs::Marker iss_msg;
        iss_msg.header.frame_id = issPoseFrame_;
        iss_msg.header.stamp = ros::Time();
        iss_msg.id = 0;
        iss_msg.type = visualization_msgs::Marker::MESH_RESOURCE;
        iss_msg.action = visualization_msgs::Marker::ADD;
        iss_msg.lifetime = ros::Duration(); // forever
        iss_msg.ns = issName_;

        iss_msg.mesh_resource = issURI_;
        iss_msg.mesh_use_embedded_materials = true;

        geometry_msgs::Pose pose;
        tf::poseEigenToMsg(issPose_, pose);

        iss_msg.pose = pose;

        // Scale the marker
        iss_msg.scale.x = 1;
        iss_msg.scale.y = 1;
        iss_msg.scale.z = 1;

        visPublisher_.publish(iss_msg);
    }

    // Visualize the handrails in RViz
    void visualizeHandrails()
    {
        for(size_t i = 0; i < handrails_.size(); ++i)
        {
            const Handrail* hr = handrails_[i];

            if (hr->mesh_resource == "")
            {
                ROS_ERROR("No mesh specified for handrail %s.  Skipping", hr->name.c_str());
                continue;
            }

            visualization_msgs::Marker hr_msg;
            hr_msg.header.frame_id = hr->frame;
            hr_msg.header.stamp = ros::Time();
            hr_msg.id = i;
            hr_msg.type = visualization_msgs::Marker::MESH_RESOURCE;
            hr_msg.action = visualization_msgs::Marker::ADD;
            hr_msg.ns = hr->group;
            hr_msg.lifetime = ros::Duration(); // forever

            hr_msg.mesh_resource = hr->mesh_resource;
            hr_msg.mesh_use_embedded_materials = true;

            geometry_msgs::Pose pose;
            tf::poseEigenToMsg(hr->pose, pose);

            hr_msg.pose = pose;

            // Scale the marker
            hr_msg.scale.x = 1;
            hr_msg.scale.y = 1;
            hr_msg.scale.z = 1;

            visPublisher_.publish(hr_msg);
        }
    }

protected:
    Eigen::Affine3d getMoveItCollisionOffset() const
    {
        // In theory, this function should return the identity, but...
        // Not sure why the rotations in MoveIt's collision object are different from RViz Marker.
        // I suspect rotations are being done in a different order (the origin of the handrail is
        // at the end, not the center).  hack_pose fixes this difference.
        Eigen::Affine3d hack_pose = (Eigen::Translation3d(0,0,0) *
                                     Eigen::AngleAxisd(1.57079, Eigen::Vector3d::UnitX()) *
                                     Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY()) *
                                     Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitZ()));

        return hack_pose;
    }

    bool insideHandrail(const Handrail* hr, const Eigen::Vector3d& position) const
    {
        double padding = 0.03; // 3cm tolerance

        std::map<std::string, Eigen::Vector3d>::const_iterator it = meshExtents_.find(hr->mesh_resource);
        if (it == meshExtents_.end())
            throw;

        // must rotate the extents...
        Eigen::Vector3d extents = hr->pose * getMoveItCollisionOffset() * it->second;

        Eigen::Vector3d diff = extents - hr->pose.translation();
        for(int i = 0; i < 3; ++i)
        {
            double minV = std::min(hr->pose.translation()(i), hr->pose.translation()(i) + diff(i));
            double maxV = std::max(hr->pose.translation()(i), hr->pose.translation()(i) + diff(i));

            if (position(i) < (minV - padding) || position(i) > (maxV + padding))
                return false;
        }
        return true;
    }

    void getCollisionObjectMessage(const std::string& name, const std::string& meshURI, const Eigen::Affine3d& poseEigen,
                                   const std::string& poseFrame, moveit_msgs::CollisionObject& obj_msg) const
    {
        std::map<std::string, shape_msgs::Mesh>::const_iterator it = meshes_.find(meshURI);
        if (it == meshes_.end())
            throw;

        obj_msg.operation = moveit_msgs::CollisionObject::ADD;
        obj_msg.id = name;
        obj_msg.header.frame_id = poseFrame;
        obj_msg.header.stamp = ros::Time::now();

        obj_msg.meshes.push_back(it->second);

        geometry_msgs::Pose pose;
        tf::poseEigenToMsg(poseEigen, pose);
        obj_msg.mesh_poses.push_back(pose);
    }

    void loadHandrailsFromParam()
    {
        XmlRpc::XmlRpcValue handrail_configs;
        if (nh_.getParam("handrails", handrail_configs))
        {
            if (handrail_configs.getType() != XmlRpc::XmlRpcValue::TypeArray)
            {
                ROS_ERROR("Expected a list of handrails in the handrails param");
                return;
            }

            for(size_t i = 0; i < handrail_configs.size(); ++i)
                loadHandrailFromParam(handrail_configs[i]);
        }
        else
            ROS_ERROR("No handrail specification on param server");
    }

    void loadHandrailFromParam(XmlRpc::XmlRpcValue& handrail_config)
    {
        if (handrail_config.getType() != XmlRpc::XmlRpcValue::TypeStruct)
        {
            ROS_ERROR("Expected handrail type to be a struct");
            return;
        }

        Handrail* hr;
        if (!handrail_config.hasMember("name"))
        {
            ROS_ERROR("Handrail has no name");
            return;
        }

        if (!handrail_config.hasMember("pose"))
        {
            ROS_ERROR("Pose not specified for handrail");
            return;
        }

        hr = new Handrail();
        hr->name = (std::string)handrail_config["name"];
        hr->pose = readPose(handrail_config["pose"]);

        if (!handrail_config.hasMember("frame"))
            ROS_WARN("Frame not specified for handrail pose");
        else
            hr->frame = (std::string)handrail_config["frame"];

        if (!handrail_config.hasMember("group"))
            ROS_WARN("Group not specified for handrail");
        else
            hr->group = (std::string)handrail_config["group"];

        if (!handrail_config.hasMember("mesh_resource"))
            ROS_WARN("Mesh resource not specified for handrail");
        else
            hr->mesh_resource = (std::string)handrail_config["mesh_resource"];

        if (handrailMap_.find(hr->name) != handrailMap_.end())
            ROS_WARN("There is more than one handrail with the name %s", hr->name.c_str());

        handrails_.push_back(hr);
        handrailMap_[hr->name] = hr;

        // Load handrail mesh and other related info, if we have not yet done so
        if (hr->mesh_resource.size())
        {
            std::map<std::string, Eigen::Vector3d>::const_iterator it = meshExtents_.find(hr->mesh_resource);
            if (it == meshExtents_.end())
            {
                ROS_DEBUG("Loading mesh info for %s", hr->mesh_resource.c_str());

                shapes::Mesh* mesh = shapes::createMeshFromResource(hr->mesh_resource);
                shapes::ShapeMsg shape_msg;
                shapes::constructMsgFromShape(mesh, shape_msg);
                meshExtents_[hr->mesh_resource] = shapes::computeShapeExtents(mesh);
                meshes_[hr->mesh_resource] = boost::get<shape_msgs::Mesh>(shape_msg);
                delete mesh;
            }
        }
    }

    void loadISSFromParam()
    {
        XmlRpc::XmlRpcValue iss_config;
        if (nh_.getParam("iss", iss_config))
        {
            if (!iss_config.hasMember("frame"))
            {
                ROS_ERROR("ISS coordinate frame not specified");
                return;
            }
            issPoseFrame_ = (std::string)iss_config["frame"];

            if (!iss_config.hasMember("pose"))
            {
                ROS_ERROR("ISS pose not specified");
                return;
            }
            issPose_ = readPose(iss_config["pose"]);

            if (!iss_config.hasMember("mesh_resource"))
            {
                ROS_ERROR("ISS mesh not specified");
                return;
            }
            issURI_ = (std::string)iss_config["mesh_resource"];

            if (iss_config.hasMember("name"))
                issName_ = (std::string)iss_config["name"];
            else
                issName_ = "ISS";

            // Load handrail mesh and other related info, if we have not yet done so
            if (issURI_.size())
            {
                std::map<std::string, Eigen::Vector3d>::const_iterator it = meshExtents_.find(issURI_);
                if (it == meshExtents_.end())
                {
                    ROS_DEBUG("Loading mesh info for %s", issURI_.c_str());

                    shapes::Mesh* mesh = shapes::createMeshFromResource(issURI_);
                    shapes::ShapeMsg shape_msg;
                    shapes::constructMsgFromShape(mesh, shape_msg);
                    meshExtents_[issURI_] = shapes::computeShapeExtents(mesh);
                    meshes_[issURI_] = boost::get<shape_msgs::Mesh>(shape_msg);
                    delete mesh;
                }
            }
        }
    }

    Eigen::Affine3d readPose(XmlRpc::XmlRpcValue& pose_config)
    {
        Eigen::Vector3d xyz;
        xyz[0] = (double)pose_config["x"];
        xyz[1] = (double)pose_config["y"];
        xyz[2] = (double)pose_config["z"];

        Eigen::Vector3d rpy;
        rpy[0] = (double)pose_config["roll"];
        rpy[1] = (double)pose_config["pitch"];
        rpy[2] = (double)pose_config["yaw"];

        Eigen::Affine3d pose = (Eigen::Translation3d(xyz) *
                                Eigen::AngleAxisd(rpy[0], Eigen::Vector3d::UnitX()) *
                                Eigen::AngleAxisd(rpy[1], Eigen::Vector3d::UnitY()) *
                                Eigen::AngleAxisd(rpy[2], Eigen::Vector3d::UnitZ()));
        return pose;
    }

    ros::NodeHandle nh_;
    ros::Publisher visPublisher_;
    std::vector<Handrail*> handrails_;
    std::map<std::string, Handrail*> handrailMap_;

    Eigen::Affine3d issPose_;
    std::string issPoseFrame_;
    std::string issURI_;
    std::string issName_;

    std::map<std::string, shape_msgs::Mesh> meshes_;
    std::map<std::string, Eigen::Vector3d> meshExtents_;
};

#endif