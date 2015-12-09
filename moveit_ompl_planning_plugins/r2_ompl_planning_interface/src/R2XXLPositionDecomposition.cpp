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

#include <eigen_conversions/eigen_msg.h>
#include "r2_ompl_planning_interface/R2XXLPositionDecomposition.h"
#include "moveit_r2_kinematics/tree_kinematics_tolerances.h"
#include <boost/math/constants/constants.hpp>

R2XXLPositionDecomposition::R2XXLPositionDecomposition(const ompl::base::RealVectorBounds& bounds, const std::vector<int>& slices, bool diagonalEdges,
                                                       ompl_interface::ModelBasedStateSpacePtr mbss, kinematics::KinematicsBasePtr legs_kinematics,
                                                       kinematic_constraints::KinematicConstraintSetPtr constraints,
                                                       ompl::base::SpaceInformationPtr si) : ompl::geometric::XXLPositionDecomposition(bounds, slices, diagonalEdges)
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

    // Figure out which leg is 'fixed'
    const std::vector<moveit_msgs::PositionConstraint>& pos_constraints = path_constraints_->getPositionConstraints();
    const std::vector<moveit_msgs::OrientationConstraint>& orn_constraints = path_constraints_->getOrientationConstraints();

    // Complete hack to figure out what link is fixed.  Assuming there is only one link with both position and orientation constrained
    fixed_link_name_ = "";
    for(size_t i = 0; i < pos_constraints.size(); ++i)
    {
        for(size_t j = 0; j < orn_constraints.size(); ++j)
            if (pos_constraints[i].link_name == orn_constraints[j].link_name)
                fixed_link_name_ = pos_constraints[i].link_name;
    }

    if (fixed_link_name_ == "")
    {
        ROS_ERROR("%s: Could not determine which link is the base", __FUNCTION__);
        throw;
    }

    // Total hack to populate projection links
    // TODO: Make this not hackey.
    projected_links_.push_back("r2/robot_world");
    if (pos_constraints[0].link_name.find("left") != std::string::npos) // left leg is fixed, so project the right foot
        projected_links_.push_back("r2/right_leg_foot");
    else
        projected_links_.push_back("r2/left_leg_foot");
}

R2XXLPositionDecomposition::~R2XXLPositionDecomposition()
{
}

int R2XXLPositionDecomposition::numLayers() const
{
    return (int) projected_links_.size();
}

void R2XXLPositionDecomposition::sampleCoordinateFromRegion(int r, std::vector<double>& coord) const
{
    coord.resize(cellSizes_.size());
    std::vector<int> cell;
    ridToGridCell(r, cell);

    for(size_t i = 0; i < cellSizes_.size(); ++i)
    {
        double tolerance = moveit_r2_kinematics::CRITICAL_PRIO_LINEAR_TOL; // samples must be this far from boundary to guarantee correctness in IK solver

        double range = cellSizes_[i] - tolerance;
        if (range < 0)
            throw ompl::Exception("Cannot sample accurately from regions.  Cell sizes are smaller than IK solver tolerance");

        double low = bounds_.low[i] + (cell[i] * cellSizes_[i]) + tolerance; // lower bound of the cell
        coord[i] = rng_.uniformReal(low, low + range);           // some point in the valid range of the cell
    }
}

bool R2XXLPositionDecomposition::sampleFromRegion(int r, ompl::base::State* s, const ompl::base::State* seed) const
{
    return sampleFromRegion(r, s, seed, 0);
}

void R2XXLPositionDecomposition::initializeRequest(int reg, int layer, const ompl::base::State* seed, moveit::core::RobotStatePtr seed_state_, moveit_r2_kinematics::TreeIkRequest& request) const
{
    // Sample a position in the desired region
    std::vector<double> coord;
    sampleCoordinateFromRegion(reg, coord);

    // Initializing joint seed values
    std::vector<double> joint_seed;
    if (!seed)
        seed_state_->setToRandomPositions();
    // else seed_state_ is already populated with seed values
    else
    {
        // debug
        // std::vector<int> proj;
        // project(seed, proj);

        // std::cout << "Sampling from region " << reg << " (layer " << layer << ").  Seed projection: ";
        // for(size_t i = 0; i < proj.size(); ++i)
        //     std::cout << proj[i] << " ";
        // std::cout << std::endl;
    }

    joint_seed.resize(all_joints_index_.size());
    for(unsigned int i = 0; i < all_joints_index_.size(); ++i)
        joint_seed[i] = seed_state_->getVariablePosition(all_joints_index_[i]);
    request.setJointValues(joint_seed);

    // Setting up IK request.  Fix the previous link in the hierarchy, then move the next link to the desired pose.
    request.addFixedLink(fixed_link_name_);  // this link can never, ever move
    if (layer > 0)
    {
        // TODO: Put all prior projection links in here??
        request.addFixedLink(projected_links_[layer-1]); // previous projection link does not move either
    }

    request.setWorldState(seed_state_->getGlobalLinkTransform("r2/robot_world")); // this is where the body is

    //Eigen::Affine3d link_pose = seed_state_->getGlobalLinkTransform(projected_links_[layer]);
    // link_pose.translation()(0) = coord[0];
    // link_pose.translation()(1) = coord[1];
    // link_pose.translation()(2) = coord[2];

    // Gaussian sampling of rotation with mean at current rotation
    // using quaternion representation
    double stdDev = 0.1;
    double rotDev = (2. * stdDev) / boost::math::constants::root_three<double>();
    double x = rng_.gaussian(0, rotDev);
    double y = rng_.gaussian(0, rotDev);
    double z = rng_.gaussian(0, rotDev);
    double theta = std::sqrt(x*x + y*y + z*z);

    Eigen::Quaterniond qdiff;
    if (theta < std::numeric_limits<double>::epsilon())
        qdiff.setIdentity();
    else  // sample a quaternion from a Gaussian distribution
    {
        double half_theta = theta / 2.0;
        double s = sin(half_theta) / theta;

        qdiff.w() = cos(half_theta);
        qdiff.x() = s * x;
        qdiff.y() = s * y;
        qdiff.z() = s * z;
        qdiff.normalize();
    }

    Eigen::Quaterniond q(seed_state_->getGlobalLinkTransform(projected_links_[layer]).rotation());
    q = q * qdiff;

    Eigen::Affine3d link_pose(Eigen::Translation3d(coord[0], coord[1], coord[2]) * q.toRotationMatrix());
    geometry_msgs::Pose link_pose_msg;
    tf::poseEigenToMsg(link_pose, link_pose_msg);
    request.addLinkPose(projected_links_[layer], link_pose_msg);


    /// debug
    // const std::vector<double>& world_rpy = request.getWorldStateRPY();
    // std::cout << "World frame: " << std::endl;
    // std::cout << seed_state_->getGlobalLinkTransform("r2/robot_world").matrix() << std::endl;
    // if (fabs(world_rpy[0]) < 1e-6)
    //     std::cout << "world X variable: " << seed_state_->getVariablePosition("virtual_joint/trans_x") << std::endl;
    // ROS_INFO("World state: XYZ(%f %f %f)  RPY(%f %f %f)", world_rpy[0], world_rpy[1], world_rpy[2], world_rpy[3], world_rpy[4], world_rpy[5]);
    // Eigen::Vector3d rpy = link_pose.rotation().eulerAngles(0,1,2);
    // Eigen::Vector3d trans = link_pose.translation();
    // ROS_INFO("Sampled coordinate: (%f %f %f)", coord[0], coord[1], coord[2]);
    // ROS_INFO("Desired pose state: XYZ(%f %f %f)  RPY(%f %f %f)", trans[0], trans[1], trans[2], rpy[0], rpy[1], rpy[2]);
}

bool R2XXLPositionDecomposition::sampleRemainingJoints(int layer, ompl::base::State* s, const moveit_r2_kinematics::TreeIkResponse& response) const
{
    //std::cout << "sampleRemainingJoints PREstart (" << work_state_->getGlobalLinkTransform("r2/robot_world").translation().transpose() << ")" << std::endl;

    // Setting joint values in work_state
    const std::vector<double>& new_values = response.getJointValues();
    for(size_t i = 0; i < group_joint_index_.size(); ++i)
        work_state_->setVariablePosition(group_joint_index_[i], new_values[i]);

    std::vector<std::string> virtual_joint_vars;
    std::string virtualJointName = "virtual_joint";
    virtual_joint_vars.push_back(virtualJointName + std::string("/trans_x"));
    virtual_joint_vars.push_back(virtualJointName + std::string("/trans_y"));
    virtual_joint_vars.push_back(virtualJointName + std::string("/trans_z"));
    virtual_joint_vars.push_back(virtualJointName + std::string("/rot_x"));
    virtual_joint_vars.push_back(virtualJointName + std::string("/rot_y"));
    virtual_joint_vars.push_back(virtualJointName + std::string("/rot_z"));
    virtual_joint_vars.push_back(virtualJointName + std::string("/rot_w"));

    // Must update world state too
    Eigen::Affine3d new_world_pose = response.getWorldState();
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
    work_state_->update();

    //std::cout << "sampleRemainingJoints INIT done (" << work_state_->getGlobalLinkTransform("r2/robot_world").translation().transpose() << ")" << std::endl;

    if (si_->isValid(s))  // win
    {
        //ROS_WARN("sampleRemainingJoints: No real work to do");
        return true;
    }
    else if (layer == (projected_links_.size() - 1)) // there are no free joints to play with to try and find a valid state
        return false;
    else
    {
        //std::cout << "sampleRemainingJoints start (" << work_state_->getGlobalLinkTransform("r2/robot_world").translation().transpose() << ")" << std::endl;

        int maxAttempts = 5;  // try this many joint configurations
        double variance = 0.1;

        // TOTAL HACK
        bool left_leg_constrained = fixed_link_name_.find("left") != std::string::npos;
        moveit::core::RobotModelConstPtr model = work_state_->getRobotModel();
        const robot_model::JointModelGroup* free_group = model->getJointModelGroup(left_leg_constrained ? "right_leg" : "left_leg");
        for(int att = 0; att < maxAttempts; ++att)
        {
            bool random = rng_.uniform01() < 0.10;  // random joint angles, or just something nearby?
            const std::vector<std::string>& var_names = free_group->getVariableNames();
            double* variables = work_state_->getVariablePositions();

            for(size_t i = 0; i < var_names.size(); ++i)
            {
                const moveit::core::JointModel* joint = model->getJointModel(var_names[i]);
                if (random)  // WACKY AND WILD CRAZY JOINT POSITIONS!!!
                    joint->getVariableRandomPositions(rng_, &variables[model->getVariableIndex(var_names[i])]);
                else  // sample nearby
                    joint->getVariableRandomPositionsNearBy(rng_, &variables[model->getVariableIndex(var_names[i])], &variables[model->getVariableIndex(var_names[i])], variance);

                variance += variance;  // double the variance after every attempt
            }

            // Copy the state to OMPL representation and do collision checking
            mbss_->copyToOMPLState(s, *work_state_);
            if (si_->isValid(s))
                return true;

            // reset work state for next iteration
            if (att < maxAttempts - 1)
            {
                for(size_t i = 0; i < group_joint_index_.size(); ++i)
                    work_state_->setVariablePosition(group_joint_index_[i], new_values[i]);
                work_state_->setVariablePositions(virtual_joint_vars, virtual_joint_vals);
                work_state_->update();
            }
        }
    }

    return false;

}

bool R2XXLPositionDecomposition::sampleFromRegion(int r, ompl::base::State* s, const ompl::base::State* seed, int layer) const
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
    // After a painful process, #3 wins.  Implementation below.

    if (layer >= (int)projected_links_.size())
        throw ompl::Exception("Layer is out of range");
    if (!seed && layer > 0)
        throw ompl::Exception("You must set the seed value to sample from a layer > 0");

    if (!tree_kinematics_)
    {
        ROS_ERROR("%s: Tree kinematics not initialized", __FUNCTION__);
        return false;
    }
    if (!seed)  // seed is basically required for this sampler
        ROS_ERROR("%s: No seed state passed in", __FUNCTION__);

    // Make the rest thread safe
    work_state_mutex_.lock();

    int maxPoses = 3 * (layer+1);  // try up to this many IK poses.  try harder in lower levels
    bool good = false;
    for(int npose = 0; npose < maxPoses && !good; ++npose)
    {
        mbss_->copyToRobotState(*work_state_, seed);
        work_state_->update();

        // Initialize IK request to put the desired link in the given region
        moveit_r2_kinematics::TreeIkRequest request;
        initializeRequest(r, layer, seed, work_state_, request);

        // Compute IK solution.  If successful, find a collision free configuration for the rest of the links
        moveit_r2_kinematics::TreeIkResponse response;
        good = tree_kinematics_->getPositionIk(request, response) && sampleRemainingJoints(layer, s, response);
    }

    work_state_mutex_.unlock();
    return good;
}

void R2XXLPositionDecomposition::project(const ompl::base::State *s, std::vector<double>& coord, int layer) const
{
    //boost::mutex::scoped_lock(work_state_mutex_);
    work_state_mutex_.lock();

    mbss_->copyToRobotState(*work_state_, s);
    work_state_->update();

    const Eigen::Affine3d& frame = work_state_->getGlobalLinkTransform(projected_links_[layer]);

    coord.resize(3);
    coord[0] = frame.translation()(0);
    coord[1] = frame.translation()(1);
    coord[2] = frame.translation()(2);

    work_state_mutex_.unlock();
}

void R2XXLPositionDecomposition::project(const ompl::base::State *s, std::vector<int>& layers) const
{
    //boost::mutex::scoped_lock(work_state_mutex_);
    work_state_mutex_.lock();

    mbss_->copyToRobotState(*work_state_, s);
    work_state_->update();

    layers.resize(projected_links_.size());
    for(size_t i = 0; i < projected_links_.size(); ++i)
    {
        const Eigen::Affine3d& frame = work_state_->getGlobalLinkTransform(projected_links_[i]);
        std::vector<double> coord(3);
        coord[0] = frame.translation()(0);
        coord[1] = frame.translation()(1);
        coord[2] = frame.translation()(2);

        layers[i] = coordToRegion(&coord[0]);
    }

    work_state_mutex_.unlock();
}
