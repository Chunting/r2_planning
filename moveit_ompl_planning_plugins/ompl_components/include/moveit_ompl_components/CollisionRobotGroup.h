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

#ifndef MOVEIT_OMPL_INTERPOLATION_INTERFACE_COLLISION_ROBOT_GROUP_
#define MOVEIT_OMPL_INTERPOLATION_INTERFACE_COLLISION_ROBOT_GROUP_

#include <moveit/robot_model/robot_model.h>
#include <moveit/collision_detection_fcl/collision_robot_fcl.h>

namespace ompl_interface
{
    MOVEIT_CLASS_FORWARD(CollisionRobotGroup);

    class CollisionRobotGroup : public collision_detection::CollisionRobotFCL
    {
    public:
        CollisionRobotGroup(const std::string& group, const robot_model::RobotModelConstPtr& kmodel, double scale = 1.0) :
            collision_detection::CollisionRobotFCL(kmodel, 0.0, scale) // no padding
        {
            const moveit::core::JointModelGroup* jmg = kmodel->getJointModelGroup(group);

            if (!jmg)
            {
                ROS_ERROR("%s: Group '%s' does NOT exist", __FUNCTION__, group.c_str());
                return;
            }

            const std::vector<std::string>& linknames = jmg->getLinkModelNamesWithCollisionGeometry();

            // undo what the base class did
            geoms_.clear();

            // Create the geometry for the links in the given group
            for(size_t i = 0; i < linknames.size(); ++i)
            {
                const moveit::core::LinkModel* link = kmodel->getLinkModel(linknames[i]);
                const std::vector<shapes::ShapeConstPtr>& shapes = link->getShapes();

                for(size_t j = 0; j < shapes.size(); ++j)
                {
                    collision_detection::FCLGeometryConstPtr g = collision_detection::createCollisionGeometry(shapes[j], getLinkScale(linknames[i]), getLinkPadding(linknames[i]), link, j);
                    if (g)
                        geoms_.push_back(g);
                    else
                        logError("Unable to construct collision geometry for link '%s'", linknames[i].c_str());
                }
            }
        }
        virtual ~CollisionRobotGroup()
        {
        }

    protected:
        /*virtual void constructFCLObject(const robot_state::RobotState &state, collision_detection::FCLObject &fcl_obj) const
        {
            //std::cout << __FUNCTION__;
            fcl_obj.collision_objects_.reserve(geoms_.size());

            for (std::size_t i = 0 ; i < geoms_.size() ; ++i)
                if (geoms_[i] && geoms_[i]->collision_geometry_)
                {
                    fcl::CollisionObject *collObj = new fcl::CollisionObject
                    (geoms_[i]->collision_geometry_, collision_detection::transform2fcl(state.getCollisionBodyTransform(geoms_[i]->collision_geometry_data_->ptr.link, geoms_[i]->collision_geometry_data_->shape_index)));
                    fcl_obj.collision_objects_.push_back(boost::shared_ptr<fcl::CollisionObject>(collObj));
                    // the CollisionGeometryData is already stored in the class member geoms_, so we need not copy it
                }

            std::vector<const robot_state::AttachedBody*> ab;
            state.getAttachedBodies(ab);
            for (std::size_t j = 0 ; j < ab.size() ; ++j)
            {
                std::vector<collision_detection::FCLGeometryConstPtr> objs;
                getAttachedBodyObjects(ab[j], objs);
                const EigenSTL::vector_Affine3d &ab_t = ab[j]->getGlobalCollisionBodyTransforms();
                for (std::size_t k = 0 ; k < objs.size() ; ++k)
                    if (objs[k]->collision_geometry_)
                    {
                        fcl::CollisionObject *collObj = new fcl::CollisionObject(objs[k]->collision_geometry_, collision_detection::transform2fcl(ab_t[k]));
                        fcl_obj.collision_objects_.push_back(boost::shared_ptr<fcl::CollisionObject>(collObj));
                        // we copy the shared ptr to the CollisionGeometryData, as this is not stored by the class itself,
                        // and would be destroyed when objs goes out of scope.
                        fcl_obj.collision_geometry_.push_back(objs[k]);
                    }
            }

            //std::cout << "... done" << std::endl;
        }*/
    };
}

#endif