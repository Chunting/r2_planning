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

#include "moveit_ompl_components/GoalSamplingPathSimplifier.h"
#include <ompl/tools/config/MagicConstants.h>

ompl::geometric::GoalSamplingPathSimplifier::GoalSamplingPathSimplifier(const base::SpaceInformationPtr &si,
                                                                        const base::GoalSampleableRegion* goal) : PathSimplifier(si), gsr_(goal)
{
}

bool ompl::geometric::GoalSamplingPathSimplifier::findBetterGoal(PathGeometric &path, double maxTime, unsigned int samplingAttempts,
                                                                 double rangeRatio, double snapToVertex)
{
    return findBetterGoal(path, base::timedPlannerTerminationCondition(maxTime), samplingAttempts, rangeRatio, snapToVertex);
}

bool ompl::geometric::GoalSamplingPathSimplifier::findBetterGoal(PathGeometric &path, const base::PlannerTerminationCondition &ptc,
                                                                 unsigned int samplingAttempts, double rangeRatio, double snapToVertex)
{
    if (path.getStateCount() < 2)
        return false;

    unsigned int max_goals = std::min((unsigned)10, gsr_->maxSampleCount()); // the number of goals we will sample
    unsigned int failed_tries = 0;
    bool better_goal = false;

    const base::SpaceInformationPtr &si = path.getSpaceInformation();
    const base::StateSpacePtr& ss = si->getStateSpace();
    std::vector<base::State*> &states = path.getStates();

    // dists[i] contains the cumulative length of the path up to and including state i
    std::vector<double> dists(states.size(), 0.0);
    for (unsigned int i = 1 ; i < dists.size() ; ++i)
        dists[i] = dists[i-1] + si->distance(states[i-1], states[i]);

    // Sampled states closer than 'threshold' distance to any existing state in the path
    // are snapped to the close state
    double threshold = dists.back() * snapToVertex;
    // The range (distance) of a single connection that will be attempted
    double rd = rangeRatio * dists.back();

    base::State* temp = si->allocState();
    base::State* temp_goal = si->allocState();

    const base::State* current_goal = path.getState(path.getStateCount()-1);

    while(!ptc && failed_tries++ < max_goals && !better_goal)
    {
        gsr_->sampleGoal(temp_goal);

        unsigned int num_samples = 0;
        while (!ptc && num_samples++ < samplingAttempts && !better_goal)
        {
            // sample a state within rangeRatio
            double t = rng_.uniformReal(std::max(dists.back() - rd, 0.0), dists.back());    // Sample a random point within rd of the end of the path

            std::vector<double>::iterator end = std::lower_bound(dists.begin(), dists.end(), t);
            std::vector<double>::iterator start = end;
            while(start != dists.begin() && *start >= t)
                start -= 1;

            int start_index = start - dists.begin();
            int end_index = end - dists.begin();

            // Snap the random point to the nearest vertex, if within the threshold
            if (t - (*start) < threshold) // snap to the starting waypoint
                end_index = start_index;
            if ((*end) - t < threshold)  // snap to the ending waypoint
                start_index = end_index;

            // Compute the state value and the accumulated cost up to that state
            double cost_to_come = dists[start_index];
            base::State* state;
            if (start_index == end_index)
            {
                state = states[start_index];
            }
            else
            {
                double t_seg = (t - (*start)) / (*end - *start);
                ss->interpolate(states[start_index], states[end_index], t_seg, temp);
                state = temp;

                cost_to_come += si->distance(states[start_index], state);
            }

            double cost_to_go = si->distance(state, temp_goal);
            double candidate_cost = cost_to_come + cost_to_go;

            // Make sure we improve before attempting validation
            if (candidate_cost < dists.back() && si->checkMotion(state, temp_goal))
            {
                // insert the new states
                if (start_index == end_index)
                {
                    // new intermediate state
                    si->copyState(states[start_index], state);
                    // new goal state
                    si->copyState(states[start_index+1], temp_goal);

                    if (freeStates_)
                    {
                        for(size_t i = start_index + 2; i < states.size(); ++i)
                            si->freeState(states[i]);
                    }
                    states.erase(states.begin() + start_index + 2, states.end());
                }
                else
                {
                    // overwriting the end of the segment with the new state
                    si->copyState(states[end_index], state);
                    if (end_index == states.size()-1)
                    {
                        path.append(temp_goal);
                    }
                    else
                    {
                        // adding goal new goal state
                        si->copyState(states[end_index + 1], temp_goal);
                        if (freeStates_)
                        {
                            for(size_t i = end_index + 2; i < states.size(); ++i)
                                si->freeState(states[i]);
                        }
                        states.erase(states.begin() + end_index + 2, states.end());
                    }
                }

                // fix the helper variables
                dists.resize(states.size(), 0.0);
                for (unsigned int j = start_index; j < dists.size() ; ++j)
                    dists[j] = dists[j-1] + si->distance(states[j-1], states[j]);

                better_goal = true;
            }
        }
    }

    si->freeState(temp);
    si->freeState(temp_goal);

    return better_goal;
}

void ompl::geometric::GoalSamplingPathSimplifier::simplify(PathGeometric &path, double maxTime)
{
    simplify(path, base::timedPlannerTerminationCondition(maxTime));
}


void ompl::geometric::GoalSamplingPathSimplifier::simplify(PathGeometric &path, const base::PlannerTerminationCondition &ptc)
{
    if (path.getStateCount() < 3)
        return;

    // try a randomized step of connecting vertices
    bool tryMore = false;
    if (ptc == false)
        tryMore = reduceVertices(path);

    // try to collapse close-by vertices
    if (ptc == false)
        collapseCloseVertices(path);

    // try to reduce vertices some more, if there is any point in doing so
    int times = 0;
    while (tryMore && ptc == false && ++times <= 5)
        tryMore = reduceVertices(path);

    // if the space is metric, we can do some additional smoothing
    if(path.getSpaceInformation()->getStateSpace()->isMetricSpace())
    {
        bool tryMore = true;
        unsigned int times = 0;
        do
        {
            bool shortcut = shortcutPath(path);
            bool better_goal = findBetterGoal(path, ptc);

            tryMore = shortcut || better_goal;
        } while(ptc == false && tryMore && ++times <= 5);

        // smooth the path with BSpline interpolation
        if(ptc == false)
            smoothBSpline(path, 3, path.length()/100.0);

        // we always run this if the metric-space algorithms were run.  In non-metric spaces this does not work.
        const std::pair<bool, bool> &p = path.checkAndRepair(magic::MAX_VALID_SAMPLE_ATTEMPTS);
        if (!p.second)
            OMPL_WARN("Solution path may slightly touch on an invalid region of the state space");
        else if (!p.first)
            OMPL_DEBUG("The solution path was slightly touching on an invalid region of the state space, but it was successfully fixed.");
    }
}

void ompl::geometric::GoalSamplingPathSimplifier::simplifyMax(PathGeometric &path)
{
    ompl::base::PlannerTerminationCondition never_terminate = base::plannerNonTerminatingCondition();
    simplify(path, never_terminate);
}