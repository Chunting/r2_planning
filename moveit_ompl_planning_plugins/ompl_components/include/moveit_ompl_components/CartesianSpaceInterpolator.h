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

#ifndef MOVEIT_OMPL_COMPONENTS_CARTESIAN_SPACE_INTERPOLATOR_
#define MOVEIT_OMPL_COMPONENTS_CARTESIAN_SPACE_INTERPOLATOR_

#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <Eigen/Dense>

namespace ompl_interface
{
    // Interpolator for revolute joints that locally minimizes the Cartesian distance
    // that each joint travels.  This only affects revolute joints - all other joint
    // types are interpolated in the "normal" manner.
    // THE IMPLEMENTATION ASSUMES THAT Z IS THE AXIS OF ROTATION FOR REVOLUTE JOINTS
    class CartesianSpaceInterpolator
    {
    public:
        CartesianSpaceInterpolator(const robot_model::JointModelGroup* group);

        // Perform "normal" interpolation for the given joint.
        void ignoreJoint(const std::string& joint_name);

        // Interpolate the joint positions of the configured group from a->b given parameter t.
        void interpolate(const robot_state::RobotStateConstPtr& a, const robot_state::RobotStateConstPtr& b,
                         double t, robot_state::RobotStatePtr result) const;

        // Interpolate the joint positions of the configured group from a->b given parameter t.
        void interpolate(const robot_state::RobotState* a, const robot_state::RobotState* b,
                         double t, robot_state::RobotState* result) const;

    protected:
        // Interpolate the joint using Cartesian distance minimization technique
        double revoluteJointInterpolation(const robot_model::RevoluteJointModel* joint,
                                          const Eigen::Affine3d& frame,
                                          const Eigen::Vector3d& point) const;

        const robot_model::JointModelGroup* group_;
        std::vector<bool> ignore_;

    };
}

#endif