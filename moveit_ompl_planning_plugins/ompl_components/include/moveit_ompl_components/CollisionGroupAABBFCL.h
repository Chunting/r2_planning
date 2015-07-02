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

#ifndef MOVEIT_OMPL_INTERPOLATION_INTERFACE_COLLISION_ROBOT_GROUP_AABB_
#define MOVEIT_OMPL_INTERPOLATION_INTERFACE_COLLISION_ROBOT_GROUP_AABB_

//#include <moveit/robot_model/robot_model.h>
//#include <moveit/collision_detection_fcl/collision_robot_fcl.h>

#include <moveit/collision_detection/collision_robot.h>
#include <moveit/collision_detection_fcl/collision_robot_fcl.h>
#include <moveit/collision_detection_fcl/collision_common.h>

namespace collision_detection
{
    MOVEIT_CLASS_FORWARD(CollisionGroupAABBFCL);

    class CollisionGroupAABBFCL : public CollisionRobotFCL // CollisionRobot
    {
        friend class CollisionWorldFCL;

    public:
        // Create the AABB BVH for all the links in group
        CollisionGroupAABBFCL(const std::string& group, const robot_model::RobotModelConstPtr& kmodel, double padding = 0.0, double scale = 1.0) : CollisionRobotFCL(kmodel, padding, scale)
        {
            const moveit::core::JointModelGroup* jmg = kmodel->getJointModelGroup(group);
            const std::vector<std::string>& linknames = jmg->getLinkModelNamesWithCollisionGeometry();

            // undo what the base class did
            geoms_.clear();

            // Create the geometry for the links in the given group
            for(size_t i = 0; i < linknames.size(); ++i)
                addLink(linknames[i]);
        }

        // Empty robot
        CollisionGroupAABBFCL(const robot_model::RobotModelConstPtr& kmodel, double padding = 0.0, double scale = 1.0) : CollisionRobotFCL(kmodel, padding, scale)
        {
            // undo what the base class did
            geoms_.clear();
        }

        virtual ~CollisionGroupAABBFCL()
        {
        }

        void addLink(const std::string& link_name)
        {
            const moveit::core::LinkModel* link = robot_model_->getLinkModel(link_name);
            if (!link)
            {
                ROS_ERROR("Link '%s' does not exist in robot model", link_name.c_str());
                return;
            }

            const std::vector<shapes::ShapeConstPtr>& shapes = link->getShapes();
            const EigenSTL::vector_Affine3d& collisionOrigin = link->getCollisionOriginTransforms();

            // Iterate through all geometries for this link and build an AABB for each.
            for(size_t j = 0; j < shapes.size(); ++j)
            {
                shapes::ShapePtr aabb;
                Eigen::Affine3d offset;
                getAABB(link, collisionOrigin[j], aabb, offset);

                FCLGeometryConstPtr g = createCollisionGeometry(shapes[j], getLinkScale(link_name), getLinkPadding(link_name), link, j);
                geoms_.push_back(g);

                aabb_.push_back(aabb);
                aabbOffsets_.push_back(offset);
            }
            linkNames_.push_back(link_name);
        }

        void clear()
        {
            aabb_.clear();
            aabbOffsets_.clear();
            geoms_.clear();
            linkNames_.clear();
        }

        size_t numLinks() const
        {
            return geoms_.size();
        }

        const std::vector<std::string>& getLinkNames() const
        {
            return linkNames_;
        }

        /*virtual void checkSelfCollision(const CollisionRequest &req, CollisionResult &res, const robot_state::RobotState &state) const
        {
            checkSelfCollisionHelper(req, res, state, NULL);
        }

        virtual void checkSelfCollision(const CollisionRequest &req, CollisionResult &res, const robot_state::RobotState &state, const AllowedCollisionMatrix &acm) const
        {
            checkSelfCollisionHelper(req, res, state, &acm);
        }

        virtual void checkSelfCollision(const CollisionRequest &req, CollisionResult &res, const robot_state::RobotState &state1, const robot_state::RobotState &state2) const
        {
            ROS_ERROR("FCL continuous collision checking not yet implemented");
            throw;
        }

        virtual void checkSelfCollision(const CollisionRequest &req, CollisionResult &res, const robot_state::RobotState &state1, const robot_state::RobotState &state2, const AllowedCollisionMatrix &acm) const
        {
            ROS_ERROR("FCL continuous collision checking not yet implemented");
            throw;
        }

        void checkSelfCollisionHelper(const CollisionRequest &req, CollisionResult &res, const robot_state::RobotState &state, const AllowedCollisionMatrix* acm) const
        {
            FCLManager manager;
            allocSelfCollisionBroadPhase(state, manager);
            CollisionData cd(&req, &res, acm);
            cd.enableGroup(getRobotModel());
            manager.manager_->collide(&cd, &collisionCallback);
            if (req.distance)
                res.distance = distanceSelfHelper(state, acm);
        }

        virtual void checkOtherCollision(const CollisionRequest &req, CollisionResult &res, const robot_state::RobotState &state,
                                         const CollisionRobot &other_robot, const robot_state::RobotState &other_state) const
        {
            checkOtherCollisionHelper(req, res, state, other_robot, other_state, NULL);
        }

        virtual void checkOtherCollision(const CollisionRequest &req, CollisionResult &res, const robot_state::RobotState &state,
                                         const CollisionRobot &other_robot, const robot_state::RobotState &other_state,
                                         const AllowedCollisionMatrix &acm) const
        {
            checkOtherCollisionHelper(req, res, state, other_robot, other_state, &acm);
        }

        virtual void checkOtherCollision(const CollisionRequest &req, CollisionResult &res, const robot_state::RobotState &state1, const robot_state::RobotState &state2,
                                         const CollisionRobot &other_robot, const robot_state::RobotState &other_state1, const robot_state::RobotState &other_state2) const
        {
            ROS_ERROR("FCL continuous collision checking not yet implemented");
            throw;
        }

        virtual void checkOtherCollision(const CollisionRequest &req, CollisionResult &res, const robot_state::RobotState &state1, const robot_state::RobotState &state2,
                                         const CollisionRobot &other_robot, const robot_state::RobotState &other_state1, const robot_state::RobotState &other_state2,
                                         const AllowedCollisionMatrix &acm) const
        {
            ROS_ERROR("FCL continuous collision checking not yet implemented");
            throw;
        }

        void checkOtherCollisionHelper(const CollisionRequest &req, CollisionResult &res, const robot_state::RobotState &state,
                                       const CollisionRobot &other_robot, const robot_state::RobotState &other_state,
                                       const AllowedCollisionMatrix *acm) const
        {
            FCLManager manager;
            allocSelfCollisionBroadPhase(state, manager);

            const CollisionGroupAABBFCL &fcl_rob = dynamic_cast<const CollisionGroupAABBFCL&>(other_robot);
            FCLObject other_fcl_obj;
            fcl_rob.constructFCLObject(other_state, other_fcl_obj);

            CollisionData cd(&req, &res, acm);
            cd.enableGroup(getRobotModel());
            for (std::size_t i = 0 ; !cd.done_ && i < other_fcl_obj.collision_objects_.size() ; ++i)
                manager.manager_->collide(other_fcl_obj.collision_objects_[i].get(), &cd, &collisionCallback);
            if (req.distance)
                res.distance = distanceOtherHelper(state, other_robot, other_state, acm);
        }

        virtual double distanceSelf(const robot_state::RobotState &state) const
        {
            return distanceSelfHelper(state, NULL);
        }

        virtual double distanceSelf(const robot_state::RobotState &state, const AllowedCollisionMatrix &acm) const
        {
            return distanceSelfHelper(state, &acm);
        }

        double distanceSelfHelper(const robot_state::RobotState &state, const AllowedCollisionMatrix *acm) const
        {
            FCLManager manager;
            allocSelfCollisionBroadPhase(state, manager);

            CollisionRequest req;
            CollisionResult res;
            CollisionData cd(&req, &res, acm);
            cd.enableGroup(getRobotModel());

            manager.manager_->distance(&cd, &distanceCallback);

            return res.distance;
        }

        virtual double distanceOther(const robot_state::RobotState &state,
                                     const CollisionRobot &other_robot,
                                     const robot_state::RobotState &other_state) const
        {
            return distanceOtherHelper(state, other_robot, other_state, NULL);
        }

        virtual double distanceOther(const robot_state::RobotState &state,
                                     const CollisionRobot &other_robot,
                                     const robot_state::RobotState &other_state,
                                     const AllowedCollisionMatrix &acm) const
        {
            return distanceOtherHelper(state, other_robot, other_state, &acm);
        }

        double distanceOtherHelper(const robot_state::RobotState &state, const CollisionRobot &other_robot,
                                   const robot_state::RobotState &other_state, const AllowedCollisionMatrix *acm) const
        {
            FCLManager manager;
            allocSelfCollisionBroadPhase(state, manager);

            const CollisionGroupAABBFCL& fcl_rob = dynamic_cast<const CollisionGroupAABBFCL&>(other_robot);
            FCLObject other_fcl_obj;
            fcl_rob.constructFCLObject(other_state, other_fcl_obj);

            CollisionRequest req;
            CollisionResult res;
            CollisionData cd(&req, &res, acm);
            cd.enableGroup(getRobotModel());
            for(std::size_t i = 0; !cd.done_ && i < other_fcl_obj.collision_objects_.size(); ++i)
                manager.manager_->distance(other_fcl_obj.collision_objects_[i].get(), &cd, &distanceCallback);

            return res.distance;
        }*/

        void getBoundingBox(unsigned int i, shapes::ShapePtr& aabb, Eigen::Affine3d& offset) const
        {
            aabb = aabb_[i];
            offset = aabbOffsets_[i];
        }

        void getBoundingBoxes(std::vector<shapes::ShapePtr>& aabbs, std::vector<Eigen::Affine3d>& offsets) const
        {
            aabbs = aabb_;
            offsets = aabbOffsets_;
        }

    protected:
        /*void allocSelfCollisionBroadPhase(const robot_state::RobotState &state, FCLManager &manager) const
        {
            fcl::DynamicAABBTreeCollisionManager* m = new fcl::DynamicAABBTreeCollisionManager();
            // m->tree_init_level = 2;
            manager.manager_.reset(m);
            constructFCLObject(state, manager.object_);
            manager.object_.registerTo(manager.manager_.get());
            // manager.manager_->update();
        }*/

        virtual void constructFCLObject(const robot_state::RobotState &state, FCLObject &fcl_obj) const
        {
            fcl_obj.collision_objects_.reserve(geoms_.size());

            for (std::size_t i = 0 ; i < geoms_.size() ; ++i)
                if (geoms_[i] && geoms_[i]->collision_geometry_)
                {
                    fcl::CollisionObject *collObj = new fcl::CollisionObject
                    (geoms_[i]->collision_geometry_, transform2fcl(state.getCollisionBodyTransform(geoms_[i]->collision_geometry_data_->ptr.link, geoms_[i]->collision_geometry_data_->shape_index) * aabbOffsets_[i]));
                    fcl_obj.collision_objects_.push_back(boost::shared_ptr<fcl::CollisionObject>(collObj));
                    // the CollisionGeometryData is already stored in the class member geoms_, so we need not copy it
                }
        }

        void getAABB(const moveit::core::LinkModel* link, const Eigen::Affine3d& collisionOrigin, shapes::ShapePtr& aabb, Eigen::Affine3d& offset)
        {
            const std::vector<shapes::ShapeConstPtr>& shapes = link->getShapes();

            offset = Eigen::Affine3d::Identity();
            offset.translation() = collisionOrigin.translation();

            Eigen::Vector3d extents;
            if (!collisionOrigin.rotation().isIdentity())
            {
                //ROS_INFO("Found collision origin for %s", link->getName().c_str());
                if (shapes[0]->type != shapes::MESH)
                    ROS_WARN("EXPECTED MESH");
                else
                {
                    // Clone the mesh and cast to the right type
                    shapes::Mesh* mesh = static_cast<shapes::Mesh*>(static_cast<const shapes::Mesh*>(shapes[0].get())->clone());

                    Eigen::Vector3d vmin(std::numeric_limits<double>::max(), std::numeric_limits<double>::max(), std::numeric_limits<double>::max());
                    // Adjust all vertices based on the collision origin
                    for (unsigned int i = 0; i < mesh->vertex_count; ++i)
                    {
                        Eigen::Vector3d pt(mesh->vertices[3*i], mesh->vertices[3*i + 1], mesh->vertices[3*i + 2]);
                        pt = collisionOrigin * pt;
                        mesh->vertices[3*i] = pt(0);
                        mesh->vertices[3*i + 1] = pt(1);
                        mesh->vertices[3*i + 2] = pt(2);

                        // Compute the minimum coordinate for the offset
                        for(size_t j = 0; j < 3; ++j)
                            if (pt(j) < vmin[j])
                                vmin[j] = pt(j);
                    }

                    Eigen::Vector3d ei = shapes::computeShapeExtents(mesh);
                    Eigen::Vector3d a = (-ei / 2.0);
                    Eigen::Vector3d b = (-a);
                    extents = b - a;

                    // The AABB origin is the center of the box, so adjust the offset
                    offset.translation() = vmin + 0.5*extents;

                    delete mesh;
                }

            }
            else
            {
                //ROS_INFO("No collision origin for %s", link->getName().c_str());

                Eigen::Vector3d ei = shapes::computeShapeExtents(shapes[0].get());
                Eigen::Vector3d a = (-ei / 2.0);
                Eigen::Vector3d b = (-a);
                extents = b - a;
            }

            aabb.reset(new shapes::Box(extents(0), extents(1), extents(2)));
        }

        //std::vector<FCLGeometryConstPtr> geoms_;
        std::vector<shapes::ShapePtr> aabb_;
        std::vector<Eigen::Affine3d> aabbOffsets_;
        std::vector<std::string> linkNames_;
    };

    /*class CollisionGroupAABBFCL : public collision_detection::CollisionRobotFCL
    {
    public:
        CollisionGroupAABBFCL(const std::string& group, const robot_model::RobotModelConstPtr& kmodel, double padding = 0.0, double scale = 1.0) :
            collision_detection::CollisionRobotFCL(kmodel, padding, scale)
        {
            const moveit::core::JointModelGroup* jmg = kmodel->getJointModelGroup(group);
            const std::vector<std::string>& linknames = jmg->getLinkModelNamesWithCollisionGeometry();

            // undo what the base class did
            geoms_.clear();

            // Create the geometry for the links in the given group
            for(size_t i = 0; i < linknames.size(); ++i)
            {
                const moveit::core::LinkModel* link = kmodel->getLinkModel(linknames[i]);
                const std::vector<shapes::ShapeConstPtr>& shapes = link->getShapes();
                const EigenSTL::vector_Affine3d& collisionOrigin = link->getCollisionOriginTransforms();

                // Iterate through all geometries for this link and build an AABB for each.
                for(size_t j = 0; j < shapes.size(); ++j)
                {
                    shapes::ShapePtr aabb;
                    Eigen::Affine3d offset;
                    getAABB(link, collisionOrigin[j], aabb, offset);

                    collision_detection::FCLGeometryConstPtr g = collision_detection::createCollisionGeometry(shapes[j], getLinkScale(linknames[i]), getLinkPadding(linknames[i]), link, j);
                    geoms_.push_back(g);

                    aabb_.push_back(aabb);
                    aabbOffsets_.push_back(offset);
                }
            }
        }
        virtual ~CollisionGroupAABBFCL()
        {
        }

        void getBoundingBox(unsigned int i, shapes::ShapePtr& aabb, Eigen::Affine3d& offset) const
        {
            aabb = aabb_[i];
            offset = aabbOffsets_[i];
        }

        void getBoundingBoxes(std::vector<shapes::ShapePtr>& aabbs, std::vector<Eigen::Affine3d>& offsets) const
        {
            aabbs = aabb_;
            offsets = aabbOffsets_;
        }

    protected:
        void getAABB(const moveit::core::LinkModel* link, const Eigen::Affine3d& collisionOrigin, shapes::ShapePtr& aabb, Eigen::Affine3d& offset)
        {
            const std::vector<shapes::ShapeConstPtr>& shapes = link->getShapes();

            offset = Eigen::Affine3d::Identity();
            offset.translation() = collisionOrigin.translation();

            Eigen::Vector3d extents;
            if (!collisionOrigin.rotation().isIdentity())
            {
                ROS_INFO("Found collision origin for %s", link->getName().c_str());
                if (shapes[0]->type != shapes::MESH)
                    ROS_WARN("EXPECTED MESH");
                else
                {
                    // Clone the mesh and cast to the right type
                    shapes::Mesh* mesh = static_cast<shapes::Mesh*>(static_cast<const shapes::Mesh*>(shapes[0].get())->clone());

                    Eigen::Vector3d vmin(std::numeric_limits<double>::max(), std::numeric_limits<double>::max(), std::numeric_limits<double>::max());
                    // Adjust all vertices based on the collision origin
                    for (unsigned int i = 0; i < mesh->vertex_count; ++i)
                    {
                        Eigen::Vector3d pt(mesh->vertices[3*i], mesh->vertices[3*i + 1], mesh->vertices[3*i + 2]);
                        pt = collisionOrigin * pt;
                        mesh->vertices[3*i] = pt(0);
                        mesh->vertices[3*i + 1] = pt(1);
                        mesh->vertices[3*i + 2] = pt(2);

                        // Compute the minimum coordinate for the offset
                        for(size_t j = 0; j < 3; ++j)
                            if (pt(j) < vmin[j])
                                vmin[j] = pt(j);
                    }

                    Eigen::Vector3d ei = shapes::computeShapeExtents(mesh);
                    Eigen::Vector3d a = (-ei / 2.0);
                    Eigen::Vector3d b = (-a);
                    extents = b - a;

                    // The AABB origin is the center of the box, so adjust the offset
                    offset.translation() = vmin + 0.5*extents;

                    delete mesh;
                }

            }
            else
            {
                ROS_INFO("No collision origin for %s", link->getName().c_str());

                Eigen::Vector3d ei = shapes::computeShapeExtents(shapes[0].get());
                Eigen::Vector3d a = (-ei / 2.0);
                Eigen::Vector3d b = (-a);
                extents = b - a;
            }

            aabb.reset(new shapes::Box(extents(0), extents(1), extents(2)));
        }

        std::vector<shapes::ShapePtr> aabb_;
        std::vector<Eigen::Affine3d> aabbOffsets_;
    };*/
}

#endif