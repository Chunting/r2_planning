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

#ifndef R2_XXL_POSITION_DECOMPOSITION_H_
#define R2_XXL_POSITION_DECOMPOSITION_H_

#include "moveit_r2_kinematics/moveit_r2_tree_kinematics.h"
#include "moveit/ompl_interface/parameterization/model_based_state_space.h"
#include <ompl/base/SpaceInformation.h>
#include <ompl/geometric/planners/hilo/XXLPositionDecomposition.h>
#include "random_numbers/random_numbers.h"

// This class MUST be thread safe
class R2XXLPositionDecomposition : public ompl::geometric::XXLPositionDecomposition
{
public:
    R2XXLPositionDecomposition(const ompl::base::RealVectorBounds& bounds, const std::vector<int>& slices, bool diagonalEdges,
                               ompl_interface::ModelBasedStateSpacePtr mbss, kinematics::KinematicsBasePtr legs_kinematics,
                               kinematic_constraints::KinematicConstraintSetPtr constraints,
                               ompl::base::SpaceInformationPtr si);

    virtual ~R2XXLPositionDecomposition();

    /// \brief Return the number of layers possible in this decomposition.  Must be at least 1
    virtual int numLayers() const;

    /// \brief Sample a state \e s from region r in layer 0.
    virtual bool sampleFromRegion(int r, ompl::base::State* s, const ompl::base::State* seed = NULL) const;

    /// \brief Sample a state \e s from region r in the given layer.
    virtual bool sampleFromRegion(int r, ompl::base::State* s, const ompl::base::State* seed, int layer) const;

    /// \brief Project the given State into the XXLDecomposition
    virtual void project(const ompl::base::State *s, std::vector<double>& coord, int layer = 0) const;

    /// \brief Project the state into the decomposition and retrieve the region for all valid layers
    virtual void project(const ompl::base::State *s, std::vector<int>& layers) const;

protected:
    void sampleCoordinateFromRegion(int r, std::vector<double>& coord) const;
    void initializeRequest(int reg, int layer, const ompl::base::State* seed, moveit::core::RobotStatePtr seed_state_, moveit_r2_kinematics::TreeIkRequest& request) const;
    bool sampleRemainingJoints(int layer, ompl::base::State* s, const moveit_r2_kinematics::TreeIkResponse& response) const;

    ompl_interface::ModelBasedStateSpacePtr mbss_;
    moveit_r2_kinematics::R2TreeKinematicsInterface* tree_kinematics_;

    kinematic_constraints::KinematicConstraintSetPtr path_constraints_;
    //mutable ompl::RNG rng_;
    mutable random_numbers::RandomNumberGenerator rng_;
    mutable moveit::core::RobotStatePtr work_state_;
    ompl::base::SpaceInformationPtr si_;

    std::vector<std::string> projected_links_;
    std::vector<int> all_joints_index_;
    std::vector<int> group_joint_index_;

    std::string fixed_link_name_;

    mutable boost::mutex work_state_mutex_;
};


    /*


    R2XXLSpaceDecomposition(const ompl::base::RealVectorBounds& xyzBounds, const std::vector<int>& xyzSlices,
                            const ompl::base::RealVectorBounds& rpyBounds, const std::vector<int>& rpySlices,
                            bool diagonalEdges, ompl_interface::ModelBasedStateSpacePtr mbss, kinematics::KinematicsBasePtr legs_kinematics,
                            kinematic_constraints::KinematicConstraintSetPtr constraints,
                            ompl::base::SpaceInformationPtr si) : ompl::geometric::XXLDecomposition()
    {
        mbss_ = mbss;
        if (!mbss_)
        {
            ROS_ERROR("State space is null");
            throw;
        }

        si_ = si;

        moveit_r2_kinematics::MoveItR2TreeKinematicsPlugin* treePlugin = dynamic_cast<moveit_r2_kinematics::MoveItR2TreeKinematicsPlugin*>(legs_kinematics.get());
        if (!treePlugin)
        {
            ROS_ERROR("%s: Failed to cast kinematics solver to type MoveItR2TreeKinematicsPlugin. sampleFromRegion will not work.", __FUNCTION__);
            tree_kinematics_ = NULL;
        }
        else
        {
            tree_kinematics_ = treePlugin->getTreeKinematicsInterface();
            if (!tree_kinematics_)
                ROS_ERROR("%s: R2TreeKinematicsInterface not initialized in tree kinematics solver. sampleFromRegion will not work.", __FUNCTION__);
        }

        path_constraints_ = constraints;
        work_state_.reset(new moveit::core::RobotState(mbss_->getRobotModel()));

        // Cache the variable indices for all joints in the model in the order they are treated
        // by the kinematics solver.
        const std::vector<std::string>& all_joints = tree_kinematics_->getAllJointNames();
        all_joints_index_.resize(all_joints.size());
        for (size_t i = 0; i < all_joints.size(); ++i)
            all_joints_index_[i] = mbss_->getRobotModel()->getVariableIndex(all_joints[i]);

        // Cache the variable indices for the joints in the group in the order they are treated
        // by the kinematics solver
        const std::vector<std::string>& joint_order = tree_kinematics_->getJointNames();
        for(size_t i = 0; i < joint_order.size(); ++i)
            group_joint_index_.push_back(mbss_->getRobotModel()->getVariableIndex(joint_order[i]));
    }

    /// \brief Project the given State to a pose in R^3 (x,y,z,R,P,Y).
    virtual void project(const ompl::base::State *s, std::vector<double>& coord) const
    {
        mbss_->copyToRobotState(*work_state_, s);
        work_state_->update();

        const Eigen::Affine3d& frame = work_state_->getGlobalLinkTransform("r2/robot_world");
        coord.resize(6);
        coord[0] = frame.translation()[0];
        coord[1] = frame.translation()[1];
        coord[2] = frame.translation()[2];

        Eigen::Vector3d rpy = frame.rotation().eulerAngles(0,1,2);
        coord[3] = rpy(0);
        coord[4] = rpy(1);
        coord[5] = rpy(2);
    }

    bool sampleFromRegion(int r, ompl::base::State* s, const ompl::base::State* seed) const
    {
        // This is gonna get interesting.  Need to sample a state in a particular region, not so bad.
        // The hard part is respecting path constraints while sampling from a particular region.  Much more difficult.
        //
        // Ideas:
        // 1) Rejection sampling.  Sample a state "normally", then adjust for path constraints, then check region.  Throw away if not in right region.
        //    This is likely to work poorly if the regions are small.
        //
        // 2) Use IK solver.  Task with path constraints AND region constraint.  Probably way better, but need much more machinery.
        //    The IK solver has error bounds.  May be a problem?
        //
        // 3) Use IK solver "intelligently".  Sample world pose and set it as base.  Then seed IK solver for fixed leg with current state and solve for that leg only.
        //    Then figure out something with the other leg - possibly random joint positions until we find something valid

        if (!tree_kinematics_)
        {
            ROS_ERROR("%s: Tree kinematics not initialized", __FUNCTION__);
            return false;
        }
        // #3

        // Sample a pose in the region
        std::vector<double> coord;
        sampleCoordinateFromRegion(r, coord);

        // Create the body pose.  This is the coordinate we just sampled
        Eigen::Affine3d body_pose =  Eigen::Translation3d(coord[0], coord[1], coord[2]) *
                                    (Eigen::AngleAxisd(coord[3], Eigen::Vector3d::UnitX()) *
                                     Eigen::AngleAxisd(coord[4], Eigen::Vector3d::UnitY()) *
                                     Eigen::AngleAxisd(coord[5], Eigen::Vector3d::UnitZ()));

        // Step 1 - use ik to satisfy any path constraints
        const std::vector<moveit_msgs::PositionConstraint>& pos_constraints = path_constraints_->getPositionConstraints();
        const std::vector<moveit_msgs::OrientationConstraint>& orn_constraints = path_constraints_->getOrientationConstraints();

        if (pos_constraints.size() != 1 || orn_constraints.size() != 1)
        {
            ROS_ERROR("Expected exactly one position and one orientation constraint");
            return false;
        }

        if (pos_constraints[0].link_name != orn_constraints[0].link_name)
        {
            ROS_ERROR("Expected the position and orientation to constrain the same link");
            return false;
        }

        if (pos_constraints[0].link_name == "r2/robot_world")
        {
            ROS_ERROR("%s: Cannot sample from region.  Path constrains the projected link", "R2HiloPoseDecomposition");
            return false;
        }

        bool leftLegConstrained = pos_constraints[0].link_name.find("left") != std::string::npos;
        if (!leftLegConstrained && pos_constraints[0].link_name.find("right") == std::string::npos)
        {
            ROS_ERROR("%s: Could not figure out which leg is constrained.  Constraint is on '%s'", "R2HiloPoseDecomposition", pos_constraints[0].link_name.c_str());
            return false;
        }

        // Path constraint
        geometry_msgs::Pose link_pose;
        link_pose.position = pos_constraints[0].constraint_region.primitive_poses[0].position; // TODO: This is a total hack.  Make this much more robust to constraint definition
        link_pose.orientation = orn_constraints[0].orientation;

        // Assembling IK request
        moveit_r2_kinematics::TreeIkRequest request;
        request.addFixedLink("r2/robot_world");
        request.setWorldState(body_pose);
        request.addLinkPose(pos_constraints[0].link_name, link_pose);

        unsigned int attempts = 3;
        const std::vector<std::string>& all_joints = tree_kinematics_->getAllJointNames();
        std::vector<double> joint_seed(all_joints.size(), 0.0);

        std::vector<std::string> virtual_joint_vars;
        std::string virtualJointName = "virtual_joint";
        virtual_joint_vars.push_back(virtualJointName + std::string("/trans_x"));
        virtual_joint_vars.push_back(virtualJointName + std::string("/trans_y"));
        virtual_joint_vars.push_back(virtualJointName + std::string("/trans_z"));
        virtual_joint_vars.push_back(virtualJointName + std::string("/rot_x"));
        virtual_joint_vars.push_back(virtualJointName + std::string("/rot_y"));
        virtual_joint_vars.push_back(virtualJointName + std::string("/rot_z"));
        virtual_joint_vars.push_back(virtualJointName + std::string("/rot_w"));

        for(unsigned int att = 0; att < attempts; ++att)
        {
            // If there is a seed state, and this is the first attempt, use the seed state directly
            if (att == 0 && seed)
            {
                mbss_->copyToRobotState(*work_state_, seed);
            }
            // if there is a seed and this is a subsequent attempt, seed the constrained leg and randomly set the other leg
            else if (att > 0 && seed)
            {
                const robot_model::RobotModelConstPtr& model = mbss_->getRobotModel();
                const robot_model::JointModelGroup* left_leg = model->getJointModelGroup("left_leg");
                const robot_model::JointModelGroup* right_leg = model->getJointModelGroup("right_leg");

                mbss_->copyToRobotState(*work_state_, seed);
                work_state_->setToRandomPositions(leftLegConstrained ? right_leg : left_leg);  // TODO: Gaussian sampling of joints wrt seed?
            }
            else if (!seed) // no seed.  Set all joints randomly
            {
                const robot_model::JointModelGroup* group = mbss_->getJointModelGroup();
                work_state_->setToRandomPositions(group); // randomly set the joint positions of the group
            }

            for (size_t i = 0; i < all_joints.size(); ++i)
                joint_seed[i] = work_state_->getVariablePosition(all_joints_index_[i]);
            request.setJointValues(joint_seed);

            // Call IK
            moveit_r2_kinematics::TreeIkResponse response;
            if (tree_kinematics_->getPositionIk(request, response))
            {
                // Setting joint values in work_state
                const std::vector<double>& new_values = response.getJointValues();
                for(size_t i = 0; i < group_joint_index_.size(); ++i)
                    work_state_->setVariablePosition(group_joint_index_[i], new_values[i]);

                // Must update world state too
                const Eigen::Affine3d& new_world_pose = response.getWorldState();
                Eigen::Quaternion<double> q(new_world_pose.rotation());
                std::vector<double> virtual_joint_vals(7);  // set these in same order as virtual_joint_vars
                virtual_joint_vals[0] = new_world_pose.translation()[0];
                virtual_joint_vals[1] = new_world_pose.translation()[1];
                virtual_joint_vals[2] = new_world_pose.translation()[2];
                virtual_joint_vals[3] = q.x();
                virtual_joint_vals[4] = q.y();
                virtual_joint_vals[5] = q.z();
                virtual_joint_vals[6] = q.w();
                work_state_->setVariablePositions(virtual_joint_vars, virtual_joint_vals);
                mbss_->copyToOMPLState(s, *work_state_);

                // If state isn't valid, we may try again
                if (si_->isValid(s))
                    return true;
            }
        }

        return false;
    }

protected:
    void sampleCoordinateFromRegion(int r, std::vector<double>& coord) const
    {
        coord.resize(6);
        std::vector<int> cell;
        ridToGridCell(r, cell);

        // xyz
        for(size_t i = 0; i < 3; ++i)
        {
            double range = 0;
            switch(i)
            {
                case 0:
                    range = xSize_;
                    break;
                case 1:
                    range = ySize_;
                    break;
                case 2:
                    range = zSize_;
                    break;
                default:
                    throw std::runtime_error("Something unbelievably bad happened");
            }

            double low = xyzBounds_.low[i] + (cell[i] * range);
            coord[i] = rng_.uniformReal(low, low + range);
        }

        // rpy
        // TODO: This is probably not uniform.  Look into this.
        for(size_t i = 3; i < 6; ++i)
        {
            double range = 0;
            switch(i)
            {
                case 3:
                    range = RSize_;
                    break;
                case 4:
                    range = PSize_;
                    break;
                case 5:
                    range = YSize_;
                    break;
                default:
                    throw std::runtime_error("Something unbelievably bad happened");
            }

            double low = rpyBounds_.low[i-3] + (cell[i] * range);
            coord[i] = rng_.uniformReal(low, low + range);
        }
    }


    ompl_interface::ModelBasedStateSpacePtr mbss_;
    moveit_r2_kinematics::R2TreeKinematicsInterface* tree_kinematics_;

    kinematic_constraints::KinematicConstraintSetPtr path_constraints_;
    mutable ompl::RNG rng_;
    mutable moveit::core::RobotStatePtr work_state_;
    ompl::base::SpaceInformationPtr si_;

    std::vector<int> all_joints_index_;
    std::vector<int> group_joint_index_;
};*/

#endif