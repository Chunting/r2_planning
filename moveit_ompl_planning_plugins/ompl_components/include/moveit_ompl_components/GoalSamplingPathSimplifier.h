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

#ifndef OMPL_GEOMETRIC_GOAL_SAMPLING_PATH_SIMPLIFIER_
#define OMPL_GEOMETRIC_GOAL_SAMPLING_PATH_SIMPLIFIER_

#include <ompl/geometric/PathSimplifier.h>
#include <ompl/base/goals/GoalSampleableRegion.h>

namespace ompl
{
    namespace geometric
    {
        /** \brief This class contains routines that attempt to simplify geometric paths
            by sampling and connecting the path to different goal states.

            Some of these are in fact routines that shorten the path, and do not
            necessarily make it smoother. */
        class GoalSamplingPathSimplifier : public PathSimplifier
        {
        public:
            GoalSamplingPathSimplifier(const base::SpaceInformationPtr &si, const base::GoalSampleableRegion* goal);

            virtual ~GoalSamplingPathSimplifier() {}

            /** \brief Given a path, attempt to shorten it while
                maintaining its validity by discovering a better goal
                (end point).  Similar to shortcutPath(), this method
                attempts to shorten the path only by sampling a new
                goal state and connecting this state to another randomly
                selection point along the path.  If a new goal is
                discovered and the path is shortened, this method returns
                true if changes were made to the path.

                \param path the path to reduce vertices from

                \param samplingAttempts The number of times that the method
                will attempt to connect a point on the path to a newly
                sampled goal state.

                \param rangeRatio the maximum distance between states
                a connection is attempted, as a fraction relative to
                the total number of states (between 0 and 1).

                \param snapToVertex While sampling random points on
                the path, sometimes the points may be close to
                vertices on the original path (way-points).  This
                function will then "snap to the near vertex", if the
                distance is less than \e snapToVertex fraction of the
                total path length. This should usually be a small
                value (e.g., one percent of the total path length:
                0.01; the default is half a percent)

                \note This function assumes the triangle inequality holds and should not be run on non-metric spaces.
            */
            bool findBetterGoal(PathGeometric &path, const base::PlannerTerminationCondition &ptc, unsigned int samplingAttempts=10, double rangeRatio=0.33, double snapToVertex=0.005);

            /** \brief Given a path, attempt to shorten it while
                maintaining its validity by discovering a better goal
                (end point).  Similar to shortcutPath(), this method
                attempts to shorten the path only by sampling a new
                goal state and connecting this state to another randomly
                selection point along the path.  If a new goal is
                discovered and the path is shortened, this method returns
                true if changes were made to the path.

                \param path the path to reduce vertices from

                \param maxTime The amount of time (seconds) that the method
                may use to discover a better goal state.

                \param samplingAttempts The number of times that the method
                will attempt to connect a point on the path to a newly
                sampled goal state.

                \param rangeRatio the maximum distance between states
                a connection is attempted, as a fraction relative to
                the total number of states (between 0 and 1).

                \param snapToVertex While sampling random points on
                the path, sometimes the points may be close to
                vertices on the original path (way-points).  This
                function will then "snap to the near vertex", if the
                distance is less than \e snapToVertex fraction of the
                total path length. This should usually be a small
                value (e.g., one percent of the total path length:
                0.01; the default is half a percent)

                \note This function assumes the triangle inequality holds and should not be run on non-metric spaces.
            */
            bool findBetterGoal(PathGeometric &path, double maxTime, unsigned int samplingAttempts=10, double rangeRatio=0.33, double snapToVertex=0.005);

            /** \brief Given a path, attempt to remove vertices from it while keeping the path valid.  Then, try to smooth
                the path. This function applies the same set of default operations to the path, except in non-metric spaces,
                with the intention of simplifying it. In non-metric spaces, some operations are skipped because they do
                not work correctly when the triangle inequality may not hold. */
            virtual void simplifyMax(PathGeometric &path);

            /** \brief Run simplification algorithms on the path for at most \e maxTime seconds */
            virtual void simplify(PathGeometric &path, double maxTime);

            /** \brief Run simplification algorithms on the path as long as the termination condition does not become true */
            virtual void simplify(PathGeometric &path, const base::PlannerTerminationCondition &ptc);

        protected:

            /// \brief The goal sampleable region to use for simplification
            const base::GoalSampleableRegion* gsr_;
        };
    }
}

#endif