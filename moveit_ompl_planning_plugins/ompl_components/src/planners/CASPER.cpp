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

#include "ompl/geometric/planners/casper/CASPER.h"
#include "ompl/base/objectives/MechanicalWorkOptimizationObjective.h"
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/tools/config/SelfConfig.h"
#include "ompl/tools/config/MagicConstants.h"
#include <limits>
#include <boost/math/constants/constants.hpp>

#ifdef LOG_EXPLORATION
    #include <fstream>
    static void writeExploreInformation(const ompl::base::SpaceInformationPtr &si, ompl::geometric::CASPER::ExploreEntry* entry, std::ofstream& out)
    {
        out << "EXPLORE ";
        out << "FROM ID " << entry->fromID << ", ";
        out << "FROM CELL "; for(size_t i = 0; i < entry->cellFrom.size(); ++i) out << entry->cellFrom[i] << " ";  out << ", ";
        out << "FROM VALUES "; for(size_t i = 0; i < entry->from.size(); ++i) out << entry->from(i) << " ";  out << ", ";
        out << "TO ID " << entry->toID << ", ";
        out << "TO CELL "; for(size_t i = 0; i < entry->cellTo.size(); ++i) out << entry->cellTo[i] << " ";  out << ", ";
        out << "TO VALUES "; for(size_t i = 0; i < entry->to.size(); ++i) out << entry->to(i) << " ";
        out << std::endl;
    }

    static void writeRefineInformation(const ompl::base::SpaceInformationPtr &si, ompl::geometric::CASPER::RefineEntry* entry, std::ofstream& out)
    {
        out << "REFINE ";
        for(size_t i = 0; i < entry->ids.size(); ++i)
        {
            out << "ID " << entry->ids[i] << " ";
            out << "CELL ";
            for(size_t j = 0; j < entry->cells[i].size(); ++j)
                out << entry->cells[i][j] << " ";
            out << "VALUES ";
            for(size_t j = 0; j < entry->refinement[i].size(); ++j)
                out << entry->refinement[i](j) << " ";
            out << ", ";
        }
        out << std::endl;
    }

    static void writeStateInformation(const ompl::base::SpaceInformationPtr &si, ompl::geometric::CASPER::StateEntry* entry, std::ofstream& out)
    {
        out << "STATE ";
        out << "CELL "; for(size_t i = 0; i < entry->cell.size(); ++i) out << entry->cell[i] << " ";  out << ", ";
        out << "VALUES "; for(size_t i = 0; i < entry->state.size(); ++i) out << entry->state(i) << " ";  out << ", ";
        out << "ID " << entry->id << ", ";
        out << "GOAL " << entry->goal << std::endl;
    }

    static void writeLogInformation(const ompl::base::SpaceInformationPtr &si, const std::vector<ompl::geometric::CASPER::LogEntry*>& casperLog,
                                    ompl::base::ProjectionEvaluatorPtr projectionEvaluator)
    {
        std::string filename = "/home/rluna/ROS/casper.log";
        std::ofstream fout;
        fout.open(filename.c_str());
        if (!fout)
        {
            OMPL_ERROR("Failed to open %s for writing", filename.c_str());
            return;
        }

        // Header:
        fout << casperLog.size() << " ENTRIES" << std::endl;
        fout << "DIMENSION " << projectionEvaluator->getDimension() << std::endl;
        fout << "CELL SIZES ";
        const std::vector<double>& sizes = projectionEvaluator->getCellSizes();
        for(size_t i = 0; i < sizes.size(); ++i)
            fout << sizes[i] << " ";
        fout << std::endl;

        for(size_t i = 0; i < casperLog.size(); ++i)
        {
            switch(casperLog[i]->getAction())
            {
                case ompl::geometric::CASPER::LogEntry::EXPLORE:
                    writeExploreInformation(si, static_cast<ompl::geometric::CASPER::ExploreEntry*>(casperLog[i]), fout);
                    break;

                case ompl::geometric::CASPER::LogEntry::REFINE:
                    writeRefineInformation(si, static_cast<ompl::geometric::CASPER::RefineEntry*>(casperLog[i]), fout);
                    break;

                case ompl::geometric::CASPER::LogEntry::STATE:
                    writeStateInformation(si, static_cast<ompl::geometric::CASPER::StateEntry*>(casperLog[i]), fout);
                    break;

                default:
                    OMPL_ERROR("Unknown LogEntry");
            }
        }

        fout.close();
    }
#endif

ompl::geometric::CASPER::CASPER(const base::SpaceInformationPtr &si) : base::Planner(si, "CASPER"), disc_(boost::bind(&CASPER::freeMotion, this, _1))
{
    specs_.approximateSolutions = false;
    specs_.directed = true;
    goalBias_ = 0.05;
    maxDistance_ = 0.0;
    lastGoalMotion_ = NULL;
    failedExpansionScoreFactor_ = 0.5;
    exploreTime_ = 0.1;

    //tempChangeFactor_ = exp(0.1);

    Planner::declareParam<double>("range", this, &CASPER::setRange, &CASPER::getRange, "0.:1.:10000.");
    Planner::declareParam<double>("goal_bias", this, &CASPER::setGoalBias, &CASPER::getGoalBias, "0.:.05:1.");
    Planner::declareParam<double>("explore_time", this, &CASPER::setExploreTime, &CASPER::getExploreTime, "0.01:0.1:10.");
    Planner::declareParam<double>("k", this, &CASPER::setK, &CASPER::getK, "1:1:20.");

    addPlannerProgressProperty("best cost REAL", boost::bind(&CASPER::getBestCost, this));
}

ompl::geometric::CASPER::~CASPER()
{
    disc_.clear();
}

void ompl::geometric::CASPER::clear()
{
    Planner::clear();
    disc_.clear();
    //tree_.grid.clear();
    //tree_.size = 0;
    lastGoalMotion_ = NULL;

    //bestCost_ = worstCost_ = opt_->identityCost();
    //temp_ = 100;

    goalMotions_.clear();
}

void ompl::geometric::CASPER::setup()
{
    Planner::setup();
    tools::SelfConfig sc(si_, getName());
    if (maxDistance_ == 0.0)
    {
        sc.configurePlannerRange(maxDistance_);
        maxDistance_ *= magic::COST_MAX_MOTION_LENGTH_AS_SPACE_EXTENT_FRACTION;
        maxDistance_ *= 0.5;
    }
    OMPL_INFORM("%s: Range is %f", getName().c_str(), maxDistance_);

    sampler_ = si_->allocStateSampler();

    // Make sure there is an optimization objective
    if (!pdef_ || !pdef_->hasOptimizationObjective())
    {
        OMPL_INFORM("%s: No optimization objective specified.  Using default objective.", getName().c_str());
        opt_.reset(new base::MechanicalWorkOptimizationObjective(si_));
    }
    else
    {
        opt_ = pdef_->getOptimizationObjective();
    }

    sc.configureProjectionEvaluator(projectionEvaluator_);
    disc_.setDimension(projectionEvaluator_->getDimension());

    //bestCost_ = worstCost_ = opt_->identityCost();
    //temp_ = 100;
}

void ompl::geometric::CASPER::freeMotion(Motion* motion)
{
    if (motion)
    {
        si_->freeState(motion->state);
        delete motion;
    }
}

namespace ompl
{
    namespace magic
    {
        static const double UNIQUE_GOAL_DISTANCE = 1e-4;
    }
}

bool ompl::geometric::CASPER::explore(const base::PlannerTerminationCondition &ptc, Motion* xmotion, base::Cost& threshold)
{
    base::Goal                 *goal = pdef_->getGoal().get();
    base::GoalSampleableRegion *gsr  = dynamic_cast<base::GoalSampleableRegion*>(goal);
    CasperDiscretization<Motion>::Coord xcoord;

    double minSegmentLength = si_->getStateSpace()->getLongestValidSegmentLength();

    bool foundSolution = false;
    while (!ptc)
    {
        Motion *existing = NULL;
        CasperDiscretization<Motion>::Cell* ecell = NULL;
        disc_.selectMotion(ptc, existing, ecell, opt_, threshold);  // select a motion that does NOT exceed the cost threshold

        if (!existing) // this happens if ptc has been triggered
            continue;

        // Do not expand outward from a goal state
        if (existing->goal)
            continue;

        base::State* xstate = xmotion->state;

        bool exists = false;
        unsigned int existsIdx = 0;

        // Sample a state near the existing motion (with a goal bias)
        if (gsr && rng_.uniform01() < goalBias_ && gsr->canSample())
        {
            gsr->sampleGoal(xstate);

            // Make sure goal states are diverse
            bool unique = true;
            for(size_t i = 0; i < goalMotions_.size() && unique; ++i)
                unique = si_->distance(goalMotions_[i]->state, xstate) > magic::UNIQUE_GOAL_DISTANCE;

            // If we cannot find a unique goal, sample an existing one
            if (!unique)
            {
                exists = true;
                existsIdx = rng_.uniformInt(0, goalMotions_.size()-1);
                si_->copyState(xstate, goalMotions_[existsIdx]->state);
            }
        }
        else
            sampler_->sampleUniformNear(xstate, existing->state, maxDistance_);

        // Ensure the distance is no more than maxDistance_
        //double d = si_->distance(existing->state, xstate);
        //if (d > maxDistance_)
        //{
        //    si_->getStateSpace()->interpolate(existing->state, xstate, maxDistance_ / d, xstate);
        //    exists = false;
        //}
        //if (d < minSegmentLength)
        //{
        //    // Make this 20% of max distance
        //    double t = (maxDistance_ / d) * 0.20;
        //    si_->getStateSpace()->interpolate(existing->state, xstate, t, xstate);
        //    exists = false;
        //}


        if (!si_->isValid(xstate))
        {
            ecell->data->score *= failedExpansionScoreFactor_;
            continue;
        }

        base::Cost motion_cost = opt_->motionCost(existing->state, xstate);
        base::Cost cost_to_come = opt_->combineCosts(existing->cost, motion_cost);

        // The cost-to-come to the new state exceeds threshold.  Just throw it out
        if (!opt_->isCostBetterThan(cost_to_come, threshold))
            continue;

        // The motion is not valid.  Throw it out
        if (!si_->checkMotion(existing->state, xstate))
        {
            ecell->data->score *= failedExpansionScoreFactor_;  // failed to expand from this cell
        }
        else
        {
            if (exists)
            {
                projectionEvaluator_->computeCoordinates(xstate, xcoord);
                Motion* motion = goalMotions_[existsIdx];
                if (opt_->isCostBetterThan(cost_to_come, motion->cost))
                {
                    motion->parent = existing;
                    motion->cost = cost_to_come;
                    existing->children.push_back(motion);
                }

                if (motion->children.size() > 0)
                    throw ompl::Exception ("Did not expect motion to have children");
                if (!motion->goal)
                    throw ompl::Exception("Expected motion to be at the goal");


                // DEBUG COST
                /*std::vector<Motion*> path;
                while(motion != NULL)
                {
                    path.push_back(motion);
                    motion = motion->parent;
                }

                std::reverse(path.begin(), path.end());
                base::Cost cost = path[0]->cost;
                for(size_t i = 1; i < path.size(); ++i)
                {
                    base::Cost motionCost = opt_->motionCost(path[i-1]->state, path[i]->state);
                    base::Cost ctc = opt_->combineCosts(path[i-1]->cost, motionCost);

                    double diff = fabs(ctc.value() - path[i]->cost.value());
                    if (diff > 1e-6)
                        std::cout << "(GOAL) Cost at [" << i+1 << " / " << path.size() << "] should be " << ctc.value() << "  but the tree says " << path[i]->cost.value() << std::endl;
                }
                // std::cout << std::endl;
                // END DEBUG COST */
            }
            else
            {
                projectionEvaluator_->computeCoordinates(xstate, xcoord);
                Motion *motion = new Motion(si_);
                si_->copyState(motion->state, xstate);
                motion->cost = cost_to_come;
                motion->parent = existing;

                existing->children.push_back(motion);

                double distGoal;
                bool atGoal = goal->isSatisfied(xstate, &distGoal);
                disc_.addMotion(motion, xcoord, distGoal);

#ifdef LOG_EXPLORATION
                ExploreEntry* exploreEntry = new ExploreEntry(si_, projectionEvaluator_, existing, ecell->coord, motion, xcoord);
                exploreLog_.push_back(exploreEntry);
#endif


                // Check for goal
                if (atGoal)
                {
                    goalMotions_.push_back(motion);
                    motion->goal = true;
                    foundSolution = true;

#ifdef LOG_EXPLORATION
                    StateEntry* stateEntry = new StateEntry(si_, projectionEvaluator_, motion, xcoord, true);
                    exploreLog_.push_back(stateEntry);
#endif
                }
                // Goal "tug"
                //else if (distGoal < maxDistance_ && gsr && gsr->canSample() && goalBias_ > 0.0)
                //{
                //
                //}

                /*// DEBUG COST
                std::vector<Motion*> path;
                while(motion != NULL)
                {
                    path.push_back(motion);
                    motion = motion->parent;
                }

                std::reverse(path.begin(), path.end());
                base::Cost cost = path[0]->cost;
                for(size_t i = 1; i < path.size(); ++i)
                {
                    base::Cost motionCost = opt_->motionCost(path[i-1]->state, path[i]->state);
                    cost = opt_->combineCosts(cost, motionCost);

                    double diff = fabs(cost.value() - path[i]->cost.value());
                    if (diff > 1e-6)
                        std::cout << "Cost at [" << i+1 << " / " << path.size() << "] should be " << cost.value() << "  but the tree says " << path[i]->cost.value() << std::endl;                }
                //std::cout << std::endl;
                // END DEBUG COST*/
            }
        }

        disc_.updateCell(ecell);
    }

    return foundSolution;
}

void ompl::geometric::CASPER::shortcutVertices(const base::PlannerTerminationCondition &ptc, Motion* xmotion)
{

    ///// SHORTCUT BY DEFORMING A PATH SEGMENT, RATHER THAN INSERTING A NEW ONE
    // There is nothing to refine
    /*if (disc_.getMotionCount() == 0)
        return;

    if (goalMotions_.size() == 0)
        return;

    base::GoalPtr goal = pdef_->getGoal();
    base::State* scratch = si_->allocState();

    while(!ptc)
    {
        // construct the solution path (backward)
        std::vector<Motion*> slnPath;
        extractMotionSolutionPath(slnPath);
        std::reverse(slnPath.begin(), slnPath.end());

        //std::cout << "Shortcutting a path with " << slnPath.size() << " motions" << std::endl;

        if (slnPath.size() < 3)
            break;

        // The maximum number of segments to try to shortcut.
        unsigned int maxSegments = slnPath.size() / 2;

        unsigned int startIdx = rng_.uniformInt(0, slnPath.size() - maxSegments - 1);
        Motion* start = slnPath[startIdx];

        unsigned int endIdx = rng_.uniformInt(startIdx + 1, startIdx + maxSegments -1);
        Motion* end = slnPath[endIdx];
        //Motion* end = slnPath[slnPath.size() - endIdx - 1];

        if (endIdx == startIdx)
            continue;

        unsigned int shortcutStates = endIdx - startIdx - 1;
        if (shortcutStates == 0)
            continue;


        base::Cost ctc = start->cost;
        base::State* previous = scratch;
        si_->copyState(previous, start->state);
        base::State* current = xmotion->state;

        // Create a straight line of states equal to the number of shortcutStates
        bool valid = true;
        for(unsigned int i = 0; i < shortcutStates && valid; ++i)
        {
            double t = (i+1) / (double)(shortcutStates + 1);
            //std::cout << "t = " << t << "  startIdx = " << startIdx << "  endIdx = " << endIdx << "  shortcutStates = " << shortcutStates << "  i = " << i << std::endl;
            si_->getStateSpace()->interpolate(start->state, end->state, t, current);

            base::Cost motion_cost = opt_->motionCost(previous, current);
            ctc = opt_->combineCosts(ctc, motion_cost);

            // fail if the state is invalid or the cost increases
            if (!si_->checkMotion(previous, current) || !opt_->isCostBetterThan(ctc, end->cost))
                valid = false;
            else
                std::swap(previous, current);
        }

        if (valid)
        {
            Motion* previous = slnPath[startIdx];

            // Update the tree with the shortcut states
            for(unsigned int i = 0; i < shortcutStates; ++i)
            {
                Motion* current = slnPath[startIdx + 1 + i];

                double t = (i+1) / (double)(shortcutStates + 2);
                si_->getStateSpace()->interpolate(start->state, end->state, t, xmotion->state);

                current->cost = opt_->combineCosts(previous->cost, opt_->motionCost(previous->state, xmotion->state));
                si_->copyState(current->state, xmotion->state);

                previous = current;
            }

            // Update the end state and trickle the cost update down
            //std::cout << "WIN.  Old cost = " << end->cost.value();
            end->cost = opt_->combineCosts(previous->cost, opt_->motionCost(previous->state, end->state));
            //std::cout << "  New cost = " << end->cost.value() << std::endl;
            updateChildCosts(ptc, end);
        }
    }

    si_->freeState(scratch);*/

    // There is nothing to refine
    if (disc_.getMotionCount() == 0)
        return;

    // construct the solution path (backward)
    std::vector<Motion*> slnPath;
    extractMotionSolutionPath(slnPath);

    base::GoalPtr goal = pdef_->getGoal();
    while(!ptc)
    {
        if (slnPath.size() < 3)
            break;

        // The maximum number of segments to try to shortcut.
        unsigned int maxSegments = slnPath.size() / 2;

        unsigned int startIdx = rng_.uniformInt(0, slnPath.size() - maxSegments - 1);
        Motion* start = slnPath[slnPath.size() - startIdx - 1];  // remember, the path is backward

        unsigned int endIdx = rng_.uniformInt(startIdx + 2, startIdx + maxSegments);
        Motion* end = slnPath[slnPath.size() - endIdx - 1];

        // Try to shortcut.  Note that the shortcut could be very long, so we split it into
        // segments of bounded length
        std::vector<Motion*> shortcut;
        shortcut.push_back(start);

        bool reachesEnd = false;
        while (!reachesEnd && !ptc)
        {
            bool atEnd = false;
            double d = si_->distance(start->state, end->state);
            if (d > maxDistance_)
                si_->getStateSpace()->interpolate(start->state, end->state, maxDistance_ / d, xmotion->state);
            else
            {
                si_->copyState(xmotion->state, end->state);
                atEnd = true;
            }

            base::Cost motion_cost = opt_->motionCost(start->state, xmotion->state);
            base::Cost cost_to_come = opt_->combineCosts(start->cost, motion_cost);

            // Shortcut does not improve cost
            if (!opt_->isCostBetterThan(cost_to_come, end->cost))
                break;

            // Motion is invalid :(
            if (!si_->isValid(xmotion->state) || !si_->checkMotion(start->state, xmotion->state))
                break;
            else
            {
                // don't create a new motion for the end
                if (atEnd)
                {
                    reachesEnd = true;
                    end->cost = cost_to_come;
                    end->parent = start;
                    shortcut.push_back(end);
                }
                else
                {
                    Motion *motion = new Motion(si_);
                    si_->copyState(motion->state, xmotion->state);
                    motion->cost = cost_to_come;
                    motion->parent = start;

                    shortcut.push_back(motion);
                    start = motion;
                }
            }
        }

        if (ptc)
        {
            for(size_t i = 1; i < shortcut.size(); ++i)
            {
                si_->freeState(shortcut[i]->state);
                delete shortcut[i];
            }
            break;
        }

        if (reachesEnd)
        {
            // Must add intermediate states to tree
            for(size_t i = 1; i < shortcut.size()-1; ++i)
            {
                // update children of start and intermediate
                shortcut[i-1]->children.push_back(shortcut[i]);

                // Adding intermediate to tree
                CasperDiscretization<Motion>::Coord xcoord;
                projectionEvaluator_->computeCoordinates(shortcut[i]->state, xcoord);

                double distGoal;
                goal->isSatisfied(shortcut[i]->state, &distGoal);
                disc_.addMotion(shortcut[i], xcoord, distGoal);
            }

            // Update children for last intermediate state of shortcut
            if (shortcut.size() > 2)
                shortcut[shortcut.size()-2]->children.push_back(shortcut.back());

            updateChildCosts(ptc, end);


#ifdef LOG_EXPLORATION
            //std::vector<Motion*> original;
            //for(unsigned int i = slnPath.size() - startIdx - 1; i >= slnPath.size() - endIdx - 1; --i)  // slnPath is backward
            //    original.push_back(slnPath[i]);
            //if (original.size() != shortcut.size())
            //    OMPL_WARN("Original size = %lu,  shortcut size = %lu.  startIdx %lu   endIdx %lu   path.size() %lu", original.size(), shortcut.size(), startIdx, endIdx, slnPath.size());

            //RefineEntry* refineEntry = new RefineEntry(si_, projectionEvaluator_, original, shortcut);
            RefineEntry* refineEntry = new RefineEntry(si_, projectionEvaluator_, shortcut);
            exploreLog_.push_back(refineEntry);
#endif
        }
        else
        {
            for(size_t i = 1; i < shortcut.size(); ++i)
            {
                si_->freeState(shortcut[i]->state);
                delete shortcut[i];
            }
        }
    }
}

struct CostCompare
{
    bool operator()(const ompl::base::Cost& c1, const ompl::base::Cost& c2)
    {
        return c1.value() < c2.value();
    }
};

void ompl::geometric::CASPER::shortcutPath(const base::PlannerTerminationCondition &ptc, Motion* xmotion)
{
    /*// There is nothing to shortcut
    if (goalMotions_.size() == 0)
        return;

    base::GoalPtr goal = pdef_->getGoal();
    base::State* temp0 = si_->allocState();
    base::State* temp1 = si_->allocState();

    while (!ptc)
    {
        // Extract a path to shortcut
        std::vector<Motion*> slnPath;
        extractMotionSolutionPath(slnPath);
        std::reverse(slnPath.begin(), slnPath.end()); // todo: Remove this

        if (slnPath.size() < 3)
            break;

        // Compute the motion cost for each segment in the path
        std::vector<base::Cost> motionCosts(slnPath.size()-1, opt_->identityCost());
        // The total cost accumulated through each segment
        std::vector<base::Cost> costs(slnPath.size(), opt_->identityCost());

        for(size_t i = 0; i < slnPath.size()-1; ++i)
        {
            motionCosts[i] = opt_->motionCost(slnPath[i]->state, slnPath[i+1]->state);
            costs[i+1] = opt_->combineCosts(costs[i], motionCosts[i]);
        }

        double t0 = rng_.uniformReal(opt_->identityCost().value(), costs.back().value());
        double t1 = rng_.uniformReal(opt_->identityCost().value(), costs.back().value());
        if (t0 > t1)  // ensure t0 comes before t1
            std::swap(t0, t1);

        unsigned int seg0 = slnPath.size();
        unsigned int seg1 = slnPath.size();
        // Compute the states, parameterized by t0/1 in the cost space
        for(size_t i = 0; i < slnPath.size() && (seg0 == slnPath.size() || seg1 == slnPath.size()); ++i)
        {
            if (costs[i].value() > t0 && seg0 == slnPath.size())
            {
                assert(i > 0);
                seg0 = i-1;

                // Normalize t0 in the segment cost interval [i-1]->[i].
                double t = (t0 - costs[i-1].value()) / motionCosts[i-1].value();
                //t0 = (t0 - costs[i-1].value()) / motionCosts[i-1].value();

                // Assuming motion costs are linear
                si_->getStateSpace()->interpolate(slnPath[i-1]->state, slnPath[i]->state, t, temp0);
            }

            if (costs[i].value() > t1 && seg1 == slnPath.size())
            {
                assert(i > 0);
                seg1 = i-1;

                // Normalize t1 in the segment cost interval [i-1]->[i].
                //t1 = (t1 - costs[i-1].value()) / motionCosts[i-1].value();
                double t = (t - costs[i-1].value()) / motionCosts[i-1].value();

                // Assuming motion costs are linear
                si_->getStateSpace()->interpolate(slnPath[i-1]->state, slnPath[i]->state, t, temp1);
            }
        }

        if (seg0 == seg1)  // don't try to shortcut states in the same segment
            continue;

        if (seg0 == slnPath.size() || seg1 == slnPath.size())
        {
            throw ompl::Exception("Logic error in shortcut path");
        }



        // Cost-to-come to temp0 via current path
        base::Cost ctc0 = opt_->combineCosts(costs[seg0], opt_->motionCost(slnPath[seg0]->state, temp0));
        // Cost-to-come to temp1 via current path
        base::Cost ctcEnd = opt_->combineCosts(costs[seg1], opt_->motionCost(slnPath[seg1]->state, temp1));

        // total cost estimate to temp1 through the shortcut
        base::Cost candidateCost = opt_->combineCosts(ctc0, opt_->motionCost(temp0, temp1));
        if (opt_->isCostBetterThan(candidateCost, ctcEnd))
        {
            Motion* start = new Motion(si_);
            si_->copyState(start->state, temp0);
            start->cost = ctc0;
            start->parent = slnPath[seg0];

            std::vector<Motion*> shortcut;
            shortcut.push_back(start);

            base::State* end = temp1;

            bool reachesEnd = false;
            while (!reachesEnd && !ptc)
            {
                bool atEnd = false;
                double d = si_->distance(start->state, end);
                if (d > maxDistance_)
                    si_->getStateSpace()->interpolate(start->state, end, maxDistance_ / d, xmotion->state);
                else
                {
                    si_->copyState(xmotion->state, end);
                    atEnd = true;
                }

                base::Cost motion_cost = opt_->motionCost(start->state, xmotion->state);
                base::Cost cost_to_come = opt_->combineCosts(start->cost, motion_cost);

                // Shortcut does not actually improve cost
                if (!opt_->isCostBetterThan(cost_to_come, ctcEnd))
                    break;

                // Motion is invalid :(
                if (!si_->checkMotion(start->state, xmotion->state))
                    break;
                else
                {
                    Motion* motion = new Motion(si_);
                    si_->copyState(motion->state, xmotion->state);
                    motion->cost = cost_to_come;
                    motion->parent = start;
                    shortcut.push_back(motion);

                    start = motion;
                    reachesEnd = atEnd;
                }
            }

            if (ptc)
                break;

            if (reachesEnd)
            {

                // Add shortcut to tree
                for(size_t i = 0; i < shortcut.size(); ++i)
                {
                    // Update children of parent motions
                    shortcut[i]->parent->children.push_back(shortcut[i]);

                    // Adding intermediate to tree
                    CasperDiscretization<Motion>::Coord xcoord;
                    projectionEvaluator_->computeCoordinates(shortcut[i]->state, xcoord);

                    double distGoal;
                    goal->isSatisfied(shortcut[i]->state, &distGoal);
                    disc_.addMotion(shortcut[i], xcoord, distGoal);

                }

                // Update parent of node after end of shortcut
                shortcut.back()->children.push_back(slnPath[seg1+1]);
                base::Cost ctc = opt_->combineCosts(shortcut.back()->cost, opt_->motionCost(shortcut.back()->state, slnPath[seg1+1]->state));
                if (opt_->isCostBetterThan(ctc, slnPath[seg1+1]->cost))
                {
                    slnPath[seg1+1]->parent = shortcut.back();
                    slnPath[seg1+1]->cost = ctc;
                }

                // Need to update cost for children of seg1+1
                for(size_t i = 0; i < slnPath[seg1+1]->children.size() && !ptc; ++i)
                {
                    bool update = slnPath[seg1+1]->children[i]->parent == slnPath[seg1+1];

                    // also check if this should be the new parent
                    if (!update && opt_->isCostBetterThan(slnPath[seg1+1]->cost, slnPath[seg1+1]->children[i]->parent->cost))
                    {
                        slnPath[seg1+1]->children[i]->parent = slnPath[seg1+1];
                        update = true;
                    }

                    if (update)
                        updateChildCosts(ptc, slnPath[seg1+1]->children[i]);
                }
            }
            else
            {
                for(size_t i = 0; i < shortcut.size(); ++i)
                {
                    si_->freeState(shortcut[i]->state);
                    delete shortcut[i];
                }
            }
        }
    }

    si_->freeState(temp0);
    si_->freeState(temp1);*/


    // Random shortcuts.  Works reasonably well, but we can probably do better.

    // There is nothing to refine
    if (disc_.getMotionCount() == 0)
        return;
    if (goalMotions_.size() == 0) // no solution path to shortcut
        return;

    base::GoalPtr goal = pdef_->getGoal();
    base::State* temp0 = si_->allocState();
    base::State* temp1 = si_->allocState();

    while (!ptc)
    {
        // construct the solution path (backward)
        std::vector<Motion*> slnPath;
        extractMotionSolutionPath(slnPath);
        std::reverse(slnPath.begin(), slnPath.end()); // todo: Remove this

        if (slnPath.size() < 3)
            break;

        //std::vector<base::Cost> costs(slnPath.size(), base::Cost(0.0));
        //for(size_t i = 0; i < slnPath.size(); ++i)
        //    costs[i] = slnPath[i]->cost;

        // Select two random points along two (different) random segments
        unsigned int seg0 = rng_.uniformInt(0, slnPath.size()-2);
        unsigned int seg1 = rng_.uniformInt(0, slnPath.size()-2);
        while (seg1 == seg0)
            seg1 = rng_.uniformInt(0, slnPath.size()-2);

        if (seg1 < seg0)
            std::swap(seg0, seg1);  // ensure seg0 precedes seg 1

        double         t0 = rng_.uniform01();
        double         t1 = rng_.uniform01();

        si_->getStateSpace()->interpolate(slnPath[seg0]->state, slnPath[seg0+1]->state, t0, temp0);
        si_->getStateSpace()->interpolate(slnPath[seg1]->state, slnPath[seg1+1]->state, t1, temp1);

        if (!si_->isValid(temp0) || !si_->isValid(temp1))
            continue;

        // Cost-to-come to temp0 via current path
        //base::Cost ctc0 = opt_->combineCosts(costs[seg0], opt_->motionCost(slnPath[seg0]->state, temp0));
        base::Cost ctc0 = opt_->combineCosts(slnPath[seg0]->cost, opt_->motionCost(slnPath[seg0]->state, temp0));
        // Cost-to-come to temp1 via current path
        //base::Cost ctcEnd = opt_->combineCosts(costs[seg1], opt_->motionCost(slnPath[seg1]->state, temp1));
        base::Cost ctcEnd = opt_->combineCosts(slnPath[seg1]->cost, opt_->motionCost(slnPath[seg1]->state, temp1));

        // total cost to temp1 through the shortcut
        //base::Cost candidateCost = opt_->combineCosts(ctc0, opt_->motionCost(temp0, temp1));
        //if (opt_->isCostBetterThan(candidateCost, ctcEnd))
        {
            Motion* start = new Motion(si_);
            si_->copyState(start->state, temp0);
            start->cost = ctc0;
            start->parent = slnPath[seg0];

            std::vector<Motion*> shortcut;
            shortcut.push_back(start);

            base::State* end = temp1;

            bool reachesEnd = false;
            while (!reachesEnd && !ptc)
            {
                bool atEnd = false;
                double d = si_->distance(start->state, end);
                if (d > maxDistance_)
                    si_->getStateSpace()->interpolate(start->state, end, maxDistance_ / d, xmotion->state);
                else
                {
                    si_->copyState(xmotion->state, end);
                    atEnd = true;
                }

                base::Cost cost_to_come = opt_->combineCosts(start->cost, opt_->motionCost(start->state, xmotion->state));

                // Shortcut does not actually improve cost
                if (!opt_->isCostBetterThan(cost_to_come, ctcEnd))
                    break;

                // Motion is invalid :(
                if (!si_->checkMotion(start->state, xmotion->state))
                    break;
                else
                {
                    Motion* motion = new Motion(si_);
                    si_->copyState(motion->state, xmotion->state);
                    motion->cost = cost_to_come;
                    motion->parent = start;
                    shortcut.push_back(motion);

                    start = motion;
                    reachesEnd = atEnd;
                }
            }

            if (ptc)
            {
                for(size_t i = 0; i < shortcut.size(); ++i)
                {
                    si_->freeState(shortcut[i]->state);
                    delete shortcut[i];
                }
                break;
            }

            if (reachesEnd)
            {

                // Add shortcut to tree
                for(size_t i = 0; i < shortcut.size(); ++i)
                {
                    // Update children of parent motions
                    shortcut[i]->parent->children.push_back(shortcut[i]);

                    // update children of start and intermediate
                    //shortcut[i-1]->children.push_back(shortcut[i]);

                    // Adding intermediate to tree
                    CasperDiscretization<Motion>::Coord xcoord;
                    projectionEvaluator_->computeCoordinates(shortcut[i]->state, xcoord);

                    double distGoal;
                    goal->isSatisfied(shortcut[i]->state, &distGoal);
                    disc_.addMotion(shortcut[i], xcoord, distGoal);

                }

                // Update children for last intermediate state of shortcut
                //if (shortcut.size() > 2)
                //    shortcut[shortcut.size()-2]->children.push_back(shortcut.back());

                // Update parent of node after end of shortcut
                shortcut.back()->children.push_back(slnPath[seg1+1]);
                base::Cost ctc = opt_->combineCosts(shortcut.back()->cost, opt_->motionCost(shortcut.back()->state, slnPath[seg1+1]->state));
                //bool parentChanged = false;
                if (opt_->isCostBetterThan(ctc, slnPath[seg1+1]->cost))
                {
                    slnPath[seg1+1]->parent = shortcut.back();
                    slnPath[seg1+1]->cost = ctc;
                    //parentChanged = true;
                    updateChildCosts(ptc, slnPath[seg1+1]);
                }



                /*// Need to update cost for children of seg1+1
                for(size_t i = 0; i < slnPath[seg1+1]->children.size(); ++i)
                {
                    bool update = slnPath[seg1+1]->children[i]->parent == slnPath[seg1+1];
                    if (!update && opt_->isCostBetterThan(slnPath[seg1+1]->cost, slnPath[seg1+1]->children[i]->parent->cost))
                    {
                        slnPath[seg1+1]->children[i]->parent = slnPath[seg1+1];
                        update = true;
                    }

                    if (update)
                        updateChildCosts(ptc, slnPath[seg1+1]->children[i]);
                }*/
            }
            else
            {
                for(size_t i = 0; i < shortcut.size(); ++i)
                {
                    si_->freeState(shortcut[i]->state);
                    delete shortcut[i];
                }
            }
        }
    }

    si_->freeState(temp0);
    si_->freeState(temp1);
}

void ompl::geometric::CASPER::gradientDescentStates(const base::PlannerTerminationCondition& ptc, Motion* xmotion)
{
    // pick any state and nudge it in the direction of the negative gradient

    while (!ptc)
    {
        // Pick a state, any (non-goal) state
        unsigned int n = rng_.uniformInt(0, disc_.getMotionCount()-1);
        Motion* existing = disc_.nthMotion(n);

        // Don't move start states
        if (!existing->parent)
            continue;

        std::vector<double> gradient;
        //opt_->costGradient(existing->state, gradient);

        std::vector<double> state;
        si_->getStateSpace()->copyToReals(state, existing->state);

        // Nudge the state in the direction of the gradient
        //double nudge_factor = 1.0;
        double nudge_factor = rng_.uniform01();  // randomized nudge factor
        for(size_t i = 0; i < state.size(); ++i)
            state[i] += (nudge_factor * gradient[i]);

        // Make it a real state again
        si_->getStateSpace()->copyFromReals(xmotion->state, state);
        if (!si_->isValid(xmotion->state))
            continue;

        base::Cost ctc = opt_->combineCosts(existing->parent->cost, opt_->motionCost(existing->parent->state, xmotion->state));

        // Check goodity
        if (opt_->isCostBetterThan(ctc, existing->cost))
        {
            // check validity
            bool valid = si_->checkMotion(existing->parent->state, xmotion->state);

            // check all childrens
            for(size_t i = 0; i < existing->children.size() && valid; ++i)
                valid = si_->checkMotion(xmotion->state, existing->children[i]->state);

            if (valid)
            {
                //std::cout << "WIN: " << existing->cost.value() << " -> " << ctc.value() << std::endl;
                si_->copyState(existing->state, xmotion->state);
                existing->cost = ctc;
                updateChildCosts(ptc, existing);

                // TODO: We may have moved in the discretization...
            }
        }
    }
}

void ompl::geometric::CASPER::gradientDescentPath(const base::PlannerTerminationCondition& ptc, Motion* xmotion)
{
    // pick any state and nudge it in the direction of the negative gradient

    while (!ptc)
    {
        // construct a solution path (backward)
        std::vector<Motion*> slnPath;
        extractMotionSolutionPath(slnPath);

        if (slnPath.size() < 3)
            continue;

        unsigned int n = rng_.uniformInt(0, slnPath.size()-2); // don't move start state (slnPath is backward)
        Motion* existing = slnPath[n];

        std::vector<double> gradient;
        //opt_->costGradient(existing->state, gradient);

        std::vector<double> state;
        si_->getStateSpace()->copyToReals(state, existing->state);

        // Nudge the state in the direction of the gradient
        //double nudge_factor = 1.0;
        double nudge_factor = rng_.uniform01();  // randomized nudge factor
        for(size_t i = 0; i < state.size(); ++i)
            state[i] += (nudge_factor * gradient[i]);

        // Make it a real state again
        si_->getStateSpace()->copyFromReals(xmotion->state, state);
        if (!si_->satisfiesBounds(xmotion->state))
            si_->enforceBounds(xmotion->state);

        if (!si_->isValid(xmotion->state))
            continue;

        base::Cost ctc = opt_->combineCosts(existing->parent->cost, opt_->motionCost(existing->parent->state, xmotion->state));

        // Check goodity
        if (opt_->isCostBetterThan(ctc, existing->cost))
        {
            // check validity
            bool valid = si_->checkMotion(existing->parent->state, xmotion->state);

            // check all childrens
            for(size_t i = 0; i < existing->children.size() && valid; ++i)
                valid = si_->checkMotion(xmotion->state, existing->children[i]->state);

            if (valid)
            {
                //std::cout << "WIN: " << existing->cost.value() << " -> " << ctc.value() << std::endl;
                si_->copyState(existing->state, xmotion->state);
                existing->cost = ctc;
                updateChildCosts(ptc, existing);

                // TODO: We may have moved in the discretization...
            }
        }
    }
}

/*
// Performs gradient descent on a segment of the solution path
void ompl::geometric::CASPER::gradientDescentStates(const base::PlannerTerminationCondition& ptc, Motion* xmotion)
{
    // There is nothing to refine
    if (disc_.getMotionCount() == 0)
        return;

    base::GoalPtr goal = pdef_->getGoal();
    while(!ptc)
    {
        // construct a solution path (backward)
        std::vector<Motion*> slnPath;
        extractMotionSolutionPath(slnPath);
        //std::reverse(slnPath.begin(), slnPath.end()); // todo: Remove this

        if (slnPath.size() <= 1)
            break;

        int start, end;

        if (slnPath.size() == 2) // path is a straight line, so lets deform the middle
        {
            si_->getStateSpace()->interpolate(slnPath.back()->state, slnPath[0]->state, 0.5, xmotion->state);
            start = 1; // slnPath is backward
            end = 0;
        }
        else
        {
            // Pick a state at random
            unsigned int idx = rng_.uniformInt(1, slnPath.size()-2);
            si_->copyState(xmotion->state, slnPath[idx]->state);

            start = idx + 1; // slnPath is backward
            end = idx - 1;
        }

        // Compute the gradient at the selected state
        std::vector<double> gradient;
        opt_->costGradient(xmotion->state, gradient);
        std::vector<double> state;
        si_->getStateSpace()->copyToReals(state, xmotion->state);

        // Nudge xmotion->state in the direction of the gradient
        double nudge_factor = 0.1;
        for(size_t i = 0; i < state.size(); ++i)
            state[i] += (nudge_factor * gradient[i]);

        si_->getStateSpace()->copyFromReals(xmotion->state, state);

        // See if this state is valid AND improves the cost
        if (si_->checkMotion(slnPath[start]->state, xmotion->state) && si_->checkMotion(xmotion->state, slnPath[end]->state))
        {
            base::Cost ctcNewState = opt_->combineCosts(slnPath[start]->cost, opt_->motionCost(slnPath[start]->state, xmotion->state));
            base::Cost ctcNewSegment = opt_->combineCosts(ctcNewState, opt_->motionCost(xmotion->state, slnPath[end]->state));

            // win!
            if (opt_->isCostBetterThan(ctcNewSegment, slnPath[end]->cost))
            {
                if (start - end == 1) // we interpolated this point, so insert the new motion
                {
                    Motion* motion = new Motion(si_);
                    si_->copyState(motion->state, xmotion->state);

                    motion->cost = ctcNewState;
                    motion->parent = slnPath[start];
                    motion->children.push_back(slnPath[end]);

                    slnPath[end]->parent = motion;
                    updateChildCosts(ptc, motion);
                }
                else
                {
                    Motion* motion = slnPath[start-1]; // slnPath is backward
                    //motion->cost = ctcNewState;
                    si_->copyState(motion->state, xmotion->state);
                    updateChildCosts(ptc, motion);

                    // NOTE: The other predecessors and children of motion MUST
                    // be checked for validity!
                }
            }
        }
    }
}*/

void ompl::geometric::CASPER::gradientDescentPathShortcut(const base::PlannerTerminationCondition& ptc, Motion* xmotion)
{
    // There is nothing to refine
    if (disc_.getMotionCount() == 0)
        return;

    if (goalMotions_.size() == 0)
        return;

    base::GoalPtr goal = pdef_->getGoal();
    base::State* scratch = si_->allocState();

    double nudge_factor = 1.0;

    while(!ptc)
    {
        // construct the solution path (backward)
        std::vector<Motion*> slnPath;
        extractMotionSolutionPath(slnPath);
        std::reverse(slnPath.begin(), slnPath.end());

        //std::cout << "Shortcutting a path with " << slnPath.size() << " motions" << std::endl;

        if (slnPath.size() < 3)
            break;

        // The maximum number of segments to try to shortcut.
        unsigned int maxSegments = slnPath.size() / 2;

        unsigned int startIdx = rng_.uniformInt(0, slnPath.size() - maxSegments - 1);
        Motion* start = slnPath[startIdx];

        unsigned int endIdx = rng_.uniformInt(startIdx + 1, startIdx + maxSegments -1);
        Motion* end = slnPath[endIdx];
        //Motion* end = slnPath[slnPath.size() - endIdx - 1];

        if (endIdx == startIdx)
            continue;

        unsigned int shortcutStates = endIdx - startIdx - 1;
        if (shortcutStates == 0)
            continue;


        base::Cost ctc = start->cost;
        base::State* previous = scratch;
        si_->copyState(previous, start->state);
        base::State* current = xmotion->state;

        // Create a straight line of states equal to the number of shortcutStates
        bool valid = true;
        for(unsigned int i = 0; i < shortcutStates && valid; ++i)
        {
            double t = (i+1) / (double)(shortcutStates + 1);
            //std::cout << "t = " << t << "  startIdx = " << startIdx << "  endIdx = " << endIdx << "  shortcutStates = " << shortcutStates << "  i = " << i << std::endl;
            si_->getStateSpace()->interpolate(start->state, end->state, t, current);

            // Compute the gradient at the selected state
            std::vector<double> gradient;
            //opt_->costGradient(current, gradient);
            std::vector<double> stateVals;
            si_->getStateSpace()->copyToReals(stateVals, current);

            // Nudge the state in the direction of the gradient
            double nudge_factor = 0.1;
            for(size_t i = 0; i < stateVals.size(); ++i)
                stateVals[i] += (nudge_factor * gradient[i]);

            si_->getStateSpace()->copyFromReals(current, stateVals);

            base::Cost motion_cost = opt_->motionCost(previous, current);
            ctc = opt_->combineCosts(ctc, motion_cost);

            // fail if the state is invalid or the cost increases
            if (!si_->checkMotion(previous, current) || !opt_->isCostBetterThan(ctc, end->cost))
                valid = false;
            else
                std::swap(previous, current);
        }

        if (valid)
        {
            Motion* previous = slnPath[startIdx];

            // Update the tree with the shortcut states
            for(unsigned int i = 0; i < shortcutStates; ++i)
            {
                Motion* current = slnPath[startIdx + 1 + i];

                double t = (i+1) / (double)(shortcutStates + 2);
                si_->getStateSpace()->interpolate(start->state, end->state, t, xmotion->state);

                // Compute the gradient at the selected state
                std::vector<double> gradient;
                //opt_->costGradient(xmotion->state, gradient);
                std::vector<double> stateVals;
                si_->getStateSpace()->copyToReals(stateVals, xmotion->state);

                // Nudge the state in the direction of the gradient
                double nudge_factor = 0.1;
                for(size_t i = 0; i < stateVals.size(); ++i)
                    stateVals[i] += (nudge_factor * gradient[i]);

                si_->getStateSpace()->copyFromReals(xmotion->state, stateVals);

                current->cost = opt_->combineCosts(previous->cost, opt_->motionCost(previous->state, xmotion->state));
                si_->copyState(current->state, xmotion->state);

                previous = current;
            }

            // Update the end state and trickle the cost update down
            //std::cout << "WIN.  Old cost = " << end->cost.value();
            end->cost = opt_->combineCosts(previous->cost, opt_->motionCost(previous->state, end->state));
            //std::cout << "  New cost = " << end->cost.value() << std::endl;
            updateChildCosts(ptc, end);
        }
    }

    si_->freeState(scratch);
}

void ompl::geometric::CASPER::extractMotionSolutionPath(std::vector<Motion*>& path)
{
    if (goalMotions_.size() == 0)
        return;

    //Motion* solution = goalMotions_[0];
    //for(size_t i = 1; i < goalMotions_.size(); ++i)
    //    if (opt_->isCostBetterThan(goalMotions_[i]->cost, solution->cost))
    //        solution = goalMotions_[i];

    // Select an existing solution path uniformly at random
    Motion* solution = goalMotions_[rng_.uniformInt(0, goalMotions_.size()-1)];

    // construct the solution path (backward)
    path.clear();
    while (solution != NULL)
    {
        path.push_back(solution);
        solution = solution->parent;
    }
}

void ompl::geometric::CASPER::updateChildCosts(const base::PlannerTerminationCondition &ptc, Motion* motion)
{
    base::Cost motion_cost = opt_->motionCost(motion->parent->state, motion->state);
    base::Cost cost_to_come = opt_->combineCosts(motion->parent->cost, motion_cost);

    //if (opt_->isCostBetterThan(cost_to_come, motion->cost))
    {
        motion->cost = cost_to_come;

        CasperDiscretization<Motion>::Coord xcoord;
        projectionEvaluator_->computeCoordinates(motion->state, xcoord);
        disc_.updateMotion(motion, xcoord);

        for (size_t i = 0; i < motion->children.size() && !ptc; ++i)
        {
            bool update = motion->children[i]->parent == motion;

            // also check if this should be the new parent
            if (!update)
            {
                if (opt_->isCostBetterThan(motion->cost, motion->children[i]->parent->cost))
                {
                    motion->children[i]->parent = motion;
                    update = true;
                }
            }

            if (update)
                updateChildCosts(ptc, motion->children[i]);
        }
    }
}

ompl::base::PlannerStatus ompl::geometric::CASPER::solve(const base::PlannerTerminationCondition &ptc)
{
    checkValidity();  // throws exception if things are obviously not properly initialized

    // Add start state(s) to the tree
    while (const base::State *st = pis_.nextStart())
    {
        Motion *motion = new Motion(si_);
        si_->copyState(motion->state, st);
        motion->cost = opt_->stateCost(motion->state);

        OMPL_INFORM("%s: Start state has initial cost of %f", getName().c_str(), motion->cost.value());

        CasperDiscretization<Motion>::Coord xcoord;
        projectionEvaluator_->computeCoordinates(motion->state, xcoord);
        disc_.addMotion(motion, xcoord, 1.0);

#ifdef LOG_EXPLORATION
        StateEntry* stateEntry = new StateEntry(si_, projectionEvaluator_, motion, xcoord, false);
        exploreLog_.push_back(stateEntry);
#endif
    }

    // Oops... no start states
    if (disc_.getMotionCount() == 0)
    {
        OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
        return base::PlannerStatus::INVALID_START;
    }

    bool hasGradient = false; //opt_->hasGradient();
    if (hasGradient)
        OMPL_INFORM("%s: Optimization objective IS equipped with a gradient", getName().c_str());
    else OMPL_INFORM("%s: Optimization objective is NOT equipped with a gradient", getName().c_str());

    OMPL_INFORM("%s: The exploration time is %.3f secs", getName().c_str(), exploreTime_);
    OMPL_INFORM("%s: K = %lu", getName().c_str(), disc_.getK());

    Motion* xmotion = new Motion(si_);

    bool solution_found = false;
    bool exploration = true;

    //Motion* solution;
    while (!ptc)
    {
        // Similar to PRM, maintain a 2:1 ratio of refinement to exploration

        // TODO: Have explore return the new solution motion, if one is found
        //       Have refine operate explicitly over motions in the solution

        if (exploration)
        {
            // The threshold is the best solution cost found so far
            base::Cost threshold = opt_->infiniteCost();
            //for(size_t i = 0; i < goalMotions_.size(); ++i)
            //{
            //    if (opt_->isCostBetterThan(goalMotions_[i]->cost, threshold))
            //    {
            //        threshold = goalMotions_[i]->cost;
            //        // Check that the solution satisfies the cost objective - we can stop planning now
            //        if (opt_->isSatisfied(threshold))
            //            break;
            //    }
            //}

            base::PlannerTerminationCondition newptc = base::plannerOrTerminationCondition(ptc, base::timedPlannerTerminationCondition(exploreTime_));
            solution_found |= explore(newptc, xmotion, threshold);

            //if (solution_found)
            //    disc_.setBorderFraction(0.5); // less aggressive with the exterior cells
        }
        else
        {
            base::PlannerTerminationCondition newptc = base::plannerOrTerminationCondition(ptc, base::timedPlannerTerminationCondition(0.5*exploreTime_));
            shortcutVertices(newptc, xmotion);

            if (hasGradient)
            {
                //newptc = base::plannerOrTerminationCondition(ptc, base::timedPlannerTerminationCondition(t));
                //gradientDescentStates(newptc, xmotion);


                // use this one
                newptc = base::plannerOrTerminationCondition(ptc, base::timedPlannerTerminationCondition(0.5*exploreTime_));
                gradientDescentPath(newptc, xmotion);



                //newptc = base::plannerOrTerminationCondition(ptc, base::timedPlannerTerminationCondition(1.5*t));
                //gradientDescentPathShortcut(newptc, xmotion);
            }
        }

        exploration = !exploration;

        // The threshold is the best solution cost found so far
        //base::Cost threshold = opt_->infiniteCost();
        //solution_found = explore(ptc, xmotion, threshold);
    }

    si_->freeState(xmotion->state);
    delete xmotion;

    OMPL_INFORM("%s: Created %u states (%u goal states) in %u cells (%u internal + %u external)",
                getName().c_str(),
                disc_.getMotionCount(), goalMotions_.size(), disc_.getCellCount(),
                disc_.getGrid().countInternal(), disc_.getGrid().countExternal());

    if (goalMotions_.size() == 0)
        return base::PlannerStatus::TIMEOUT;

    Motion* solution = goalMotions_[0];
    for(size_t i = 1; i < goalMotions_.size(); ++i)
        if (opt_->isCostBetterThan(goalMotions_[i]->cost, solution->cost))
            solution = goalMotions_[i];

    lastGoalMotion_ = solution;

    OMPL_INFORM("%s: Solution path has cost %f", getName().c_str(), solution->cost.value());

    //std::cout << "CASPER: Solution has cost: " << solution->cost.value() << std::endl;

    // construct the solution path (backward)
    std::vector<Motion*> slnPath;
    while (solution != NULL)
    {
        slnPath.push_back(solution);
        solution = solution->parent;
    }

    PathGeometric *path = new PathGeometric(si_);
    for (int i = slnPath.size() - 1 ; i >= 0 ; --i)
        path->append(slnPath[i]->state);
    pdef_->addSolutionPath(base::PathPtr(path), false, 0.0, getName());


#ifdef LOG_EXPLORATION
    writeLogInformation(si_, exploreLog_, projectionEvaluator_);
#endif

    return base::PlannerStatus::EXACT_SOLUTION;
}

void ompl::geometric::CASPER::getPlannerData(base::PlannerData &data) const
{
    Planner::getPlannerData(data);
    disc_.getPlannerData(data, 0, true, lastGoalMotion_);
}
