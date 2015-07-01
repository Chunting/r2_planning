/* Author: Ryan Luna */

#ifndef MOVEIT_R2_CONSTRAINT_SAMPLER_R2_CONSTRAINT_SAMPLER
#define MOVEIT_R2_CONSTRAINT_SAMPLER_R2_CONSTRAINT_SAMPLER

#include <vector>

#include <moveit/constraint_samplers/constraint_sampler_allocator.h>
#include <moveit/constraint_samplers/default_constraint_samplers.h>
#include <moveit/planning_scene/planning_scene.h>

#include "moveit_r2_kinematics/moveit_r2_tree_kinematics.h"

namespace moveit_r2_constraints
{

class MoveItR2PoseSamplerAllocator : public constraint_samplers::ConstraintSamplerAllocator
{
public:
    MoveItR2PoseSamplerAllocator();
    virtual ~MoveItR2PoseSamplerAllocator();

    virtual constraint_samplers::ConstraintSamplerPtr alloc(const planning_scene::PlanningSceneConstPtr &scene, const std::string &group_name, const moveit_msgs::Constraints &constr);
    virtual bool canService(const planning_scene::PlanningSceneConstPtr &scene, const std::string &group_name, const moveit_msgs::Constraints &constr) const;
};

class R2PoseSampler : public constraint_samplers::ConstraintSampler
{
public:
    R2PoseSampler(const planning_scene::PlanningSceneConstPtr &scene, const std::string &group_name);

    virtual ~R2PoseSampler()
    {
    }

    /**
     * \brief Configures the Constraint given a IKSamplingPose.
     *
     * This function performs the actual constraint configuration.  It returns true if the following are true:
     * \li The \ref IKSamplingPose has either a valid orientation or position constraint
     * \li The position and orientation constraints are specified for the same link
     *
     * \li There is a valid IK solver instance for the indicated group.
     * This will be only be the case if a group has a specific solver
     * associated with it.  For situations where the super-group doesn't
     * have a solver, but all subgroups have solvers, then use the
     * \ref ConstraintSamplerManager.
     *
     * \li The kinematic model has both the links associated with the IK
     * solver's tip and base frames.
     *
     * \li The link specified in the constraints is the tip link of the IK solver
     *
     * @param [in] sp The variable that contains the position and orientation constraints
     *
     * @return True if all conditions are met and the group specified in
     * the constructor is valid.  Otherwise, false.
     */
    //bool configure(const constraint_samplers::IKSamplingPose &sp);

    /**
     * \brief Function for configuring a constraint sampler given a Constraints message.
     * @param [in] constr The constraints from which to construct a sampler
     * @return True if the configuration is successful.  If true, \ref isValid should also true.  If false, \ref isValid should return false
     */
    virtual bool configure(const moveit_msgs::Constraints &constr);


    /**
     * \brief Samples given the constraints, populating \e state.
     * This function allows the parameter max_attempts to be set.
     * @param [out] state The state into which the values will be placed. Only values for the group are written.
     * @param [in] reference_state Reference state that will be used to do transforms or perform other actions
     * @param [in] max_attempts The maximum number of times to attempt to draw a sample.  If no sample has been drawn in this number of attempts, false will be returned.
     * @return True if a sample was successfully taken, false otherwise
     */
    virtual bool sample(robot_state::RobotState &state,
                        const robot_state::RobotState &reference_state,
                        unsigned int max_attempts);

    /**
     * \brief Project a sample given the constraints, updating the joint state
     * group. This function allows the parameter max_attempts to be set.
     * @param [out] state The state into which the values will be placed. Only values for the group are written.
     * @param [in] max_attempts The maximum number of times to attempt to draw a sample.  If no sample has been drawn in this number of attempts, false will be returned.
     * @return True if a sample was successfully projected, false otherwise
     */
    virtual bool project(robot_state::RobotState &state,
                         unsigned int max_attempts);

    /**
     * \brief Get the name of the constraint sampler, for debugging purposes
     * should be in CamelCase format.
     * \return string of name
     */
    virtual const std::string& getName() const;

protected:
    virtual void clear();

    //bool loadIKSolver();
    bool samplePose(const constraint_samplers::IKSamplingPose& sampling_pose, geometry_msgs::Pose& pose,
                    const robot_state::RobotState &ks, unsigned int max_attempts) const;

    bool validLink(const std::string& link_name) const;

    void shiftStateByError(robot_state::RobotState& state, const robot_state::JointModel* base_joint,
                           const std::string& link, const Eigen::Affine3d& desired_pose) const;


    /// The kinematics solver
    //kinematics::KinematicsBaseConstPtr    kb_;
    /// Random number generator
    mutable random_numbers::RandomNumberGenerator random_number_generator_;
    /// True if the frame associated with the kinematic model is different than the base frame of the IK solver
    //bool                                  transform_ik_;
    /// The timeout associated with IK
    //double                                ik_timeout_;
    /// The base frame of the ik solver
    //std::string                           ik_frame_;
    /// Holder for the poses used for sampling
    std::vector<constraint_samplers::IKSamplingPose>   sampling_poses_;
    /// True if the constraint is configured properly
    bool                                  is_valid_;
    /// Mapping of tip frames in IK solver to the constrained poses that will be sampled
    //std::vector<int>                      tip_pose_bijection_;
    // The list of links being sampled.  In same order as sampling_poses_
    std::vector<std::string>              link_names_;

};



}

#endif