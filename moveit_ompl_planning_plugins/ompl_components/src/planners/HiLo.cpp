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

#include "ompl/geometric/planners/hilo/HiLo.h"
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/tools/config/SelfConfig.h"
#include <limits>
#include <queue>
#include <algorithm>

ompl::geometric::HiLo::HiLo(const base::SpaceInformationPtr &si) : ompl::base::Planner(si, "HiLo")
{
    numMotions_ = 0;
}

ompl::geometric::HiLo::HiLo(const base::SpaceInformationPtr &si, const HiLoDecompositionPtr& decomp) : ompl::base::Planner(si, "HiLo")
{
    numMotions_ = 0;
    setDecomposition(decomp);
}

ompl::geometric::HiLo::~HiLo()
{
    freeMemory();
}

void ompl::geometric::HiLo::clear()
{
    Planner::clear();

    freeMemory();

    startMotions_.clear();
    goalRegions_.clear();
    numMotions_ = 0;

    regionWeights_.assign(decomposition_->getNumRegions(), std::numeric_limits<double>::infinity());
    regionSelections_.assign(decomposition_->getNumRegions(), 0);
    regionMotions_.assign(decomposition_->getNumRegions(), 0);
}

void ompl::geometric::HiLo::freeMemory()
{
    for(size_t i = 0; i < motionsPerRegion_.size(); ++i)
    {
        for (size_t j = 0; j < motionsPerRegion_[i].size(); ++j)
        {
            si_->freeState(motionsPerRegion_[i][j]->state);
            delete motionsPerRegion_[i][j];
        }
        motionsPerRegion_[i].clear();
    }
}

void ompl::geometric::HiLo::setDecomposition(const HiLoDecompositionPtr& decomp)
{
    decomposition_ = decomp;

    predecessors_.resize(decomposition_->getNumRegions());
    closedList_.resize(decomposition_->getNumRegions());
    //graph_.reset(new HiLoGraph(decomposition_));
}

void ompl::geometric::HiLo::setup()
{
    Planner::setup();

    if (!decomposition_)
    {
        OMPL_ERROR("%s: Decomposition is not set.  Cannot continue setup.", getName().c_str());
        return;
    }

    motionsPerRegion_.assign(decomposition_->getNumRegions(), std::vector<Motion*>());
    regionWeights_.assign(decomposition_->getNumRegions(), std::numeric_limits<double>::infinity());
    regionSelections_.assign(decomposition_->getNumRegions(), 0);
    regionMotions_.assign(decomposition_->getNumRegions(), 0);
    numMotions_ = 0;

    goalBias_ = 0.05;

    tools::SelfConfig sc(si_, getName());
    if (range_ < std::numeric_limits<double>::epsilon())
    {
        sc.configurePlannerRange(range_);
        //maxDistance_ *= magic::COST_MAX_MOTION_LENGTH_AS_SPACE_EXTENT_FRACTION;
        //maxDistance_ *= 0.5;
    }

    sampler_ = si_->allocStateSampler();
}

#ifdef DEBUG_LEADS
#include <fstream>
#endif
ompl::base::PlannerStatus ompl::geometric::HiLo::solve(const base::PlannerTerminationCondition &ptc)
{
    checkValidity();

    if (!decomposition_)
    {
        OMPL_ERROR("%s: Decomposition is not set.  Cannot continue.", getName().c_str());
        return ompl::base::PlannerStatus::UNKNOWN;
    }

    while (const base::State *s = pis_.nextStart())
    {
        int region = decomposition_->locateRegion(s);

        Motion* start = new Motion(si_);
        si_->copyState(start->state, s);
        start->parent = NULL;

        motionsPerRegion_[region].push_back(start);
        regionMotions_[region]++;

        startMotions_.push_back(std::make_pair(start, region));
        numMotions_++;
    }

    if (startMotions_.size() == 0)
    {
        OMPL_ERROR("%s: No valid start states", getName().c_str());
        return base::PlannerStatus::INVALID_START;
    }

    if (goalRegions_.empty())
    {
        if (const base::State *g = pis_.nextGoal(ptc))
            goalRegions_.push_back(decomposition_->locateRegion(g));
        else
        {
            OMPL_ERROR("%s: Unable to sample a valid goal state", getName().c_str());
            return base::PlannerStatus::INVALID_GOAL;
        }
    }

    for(size_t i = 0; i < startMotions_.size(); ++i)
        updateRegionWeight(startMotions_[i].second);

    OMPL_INFORM("%s: Operating over %d dimensional decomposition with %d total regions", getName().c_str(), decomposition_->getDimension(), decomposition_->getNumRegions());

    // params
    unsigned int maxRegionExpansions_ = 20;
    int maxTreeExpansions = 5;

    bool solved = false;
    Motion* solution = NULL;
    unsigned int numLeads = 0;

    #ifdef DEBUG_LEADS
    std::ofstream fout;
    fout.open("leads.txt");
    #endif

    while (!ptc && !solved)
    {
        std::vector<int> lead;
        computeLead(lead);
        numLeads++;

        assert(lead.size() > 1);

        #ifdef DEBUG_LEADS
        for(size_t i = 0; i < lead.size(); ++i)
            fout << lead[i] << " ";
        fout << std::endl;
        #endif

        unsigned int numExpansions = 0;
        while (!ptc && !solved && numExpansions < maxRegionExpansions_)
        {
            //solution = expandFromRegion(ptc, selectRegion(lead), maxTreeExpansions);

            int from, to;
            selectRegion(lead, from, to);
            solution = expandFromRegion(ptc, from, to, maxTreeExpansions);
            numExpansions++;

            solved = solution != NULL;
        }
    }
    #ifdef DEBUG_LEADS
    fout.close();
    #endif

    OMPL_INFORM("%s: Created %u states using %u total leads", getName().c_str(), numMotions_, numLeads);

    if (solved)
    {
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

        return ompl::base::PlannerStatus::EXACT_SOLUTION;
    }

    return ompl::base::PlannerStatus::TIMEOUT;
}

void ompl::geometric::HiLo::computeLead(std::vector<int>& lead)
{
    int start, end;
    int tries = 0;
    int maxTries = 5;
    bool success = false;
    do
    {
        start = startMotions_[rng_.uniformInt(0, startMotions_.size()-1)].second;
        end = goalRegions_[rng_.uniformInt(0, goalRegions_.size()-1)];
        tries++;

        success = shortestPath(start, end, lead);
    }
    while (!success && tries < maxTries);

    if (!success)
        throw ompl::Exception("%s: Failed to compute lead", getName().c_str());
}

/*struct RegionSorter
{
    RegionSorter(const std::vector<double>& weights) : weights_(weights) {}

    bool operator() (int r1, int r2) const
    {
        return 1.0 / weights_[r1] < 1.0 / weights_[r2];
    }

    const std::vector<double>& weights_;
};

int ompl::geometric::HiLo::selectRegion(const std::vector<int>& lead)
{
    // Probabilistically select region based on inverse weight
    std::vector<int> sortedLead;
    double totalWeight = 0.0;
    for(size_t i = 0; i < lead.size(); ++i)
    {
        if (motionsPerRegion_[lead[i]].size() == 0)
            break;
        sortedLead.push_back(lead[i]);
        totalWeight += 1.0 / regionWeights_[lead[i]];
        //std::cout << " (" << lead[i] << ") Weight " << i+1 << " of " << lead.size() << " is " << regionWeights_[lead[i]] << std::endl;
    }

    assert(sortedLead.size() > 0);

    double r = rng_.uniformReal(0.0, totalWeight);

    //std::cout << "Total weight: " << totalWeight << "   r = " << r << std::endl;

    RegionSorter sorter(regionWeights_);
    std::sort(sortedLead.begin(), sortedLead.end(), sorter);

    int reg = -1;
    double w = 0.0;
    for(size_t i = 0; i < sortedLead.size(); ++i)
    {
        w += 1.0 / regionWeights_[sortedLead[i]];
        if (r < w)
        {
            reg = sortedLead[i];
            break;
        }
    }

    assert(reg != -1);
    return reg;
}*/

void ompl::geometric::HiLo::selectRegion(const std::vector<int>& lead, int& from, int& to)
{
    std::vector<double> leadWeights;
    double w = 0.0;
    for(size_t i = 0; i < lead.size(); ++i)
    {
        if (motionsPerRegion_[lead[i]].size() == 0)
            break;

        w += 1.0 / regionWeights_[lead[i]];
        leadWeights.push_back(w);
    }

    if (leadWeights.size() == 0)
        throw;
    double totalWeight = leadWeights.back();


    double r = rng_.uniformReal(0.0, totalWeight);
    int idx = -1;
    for(int i = 0; i < leadWeights.size(); ++i)
    {
        if (r < leadWeights[i])
        {
            idx = i;
            break;
        }
    }

    if (idx == -1)
        throw;


    if (idx == lead.size()-1)
        idx--;
    if (idx < 0)
        throw;

    from = lead[idx];
    to = lead[idx+1];
}

/*ompl::geometric::HiLo::Motion* ompl::geometric::HiLo::expandFromRegion(const base::PlannerTerminationCondition &ptc, int region, int maxExpansions)
{
    assert(motionsPerRegion_[region].size() > 0);

    base::Goal* goal = pdef_->getGoal().get();
    base::GoalSampleableRegion* gsr = dynamic_cast<base::GoalSampleableRegion*>(goal);

    base::State* xstate = si_->allocState();

    std::set<int> regionsChanged;

    Motion* goalMotion = NULL;

    for(int i = 0; i < maxExpansions && !ptc; ++i)
    {
        Motion* existing = selectMotionFromRegion(region);

        // Sample a state nearby (with goal bias)
        if (gsr && rng_.uniform01() < goalBias_ && gsr->canSample())
        {
            gsr->sampleGoal(xstate);

            // // Make sure goal states are diverse
            // bool unique = true;
            // for(size_t i = 0; i < goalMotions_.size() && unique; ++i)
            //     unique = si_->distance(goalMotions_[i]->state, xstate) > magic::UNIQUE_GOAL_DISTANCE;

            // // If we cannot find a unique goal, sample an existing one
            // if (!unique)
            // {
            //     exists = true;
            //     existsIdx = rng_.uniformInt(0, goalMotions_.size()-1);
            //     si_->copyState(xstate, goalMotions_[existsIdx]->state);
            // }
        }
        else
            sampler_->sampleUniformNear(xstate, existing->state, range_);

        // state is not valid
        if (!si_->isValid(xstate))
            continue;

        // motion is not valid
        if (!si_->checkMotion(existing->state, xstate))
            continue;

        // Add new motion
        Motion* motion = new Motion(si_);
        si_->copyState(motion->state, xstate);
        motion->parent = existing;
        int new_region = decomposition_->locateRegion(motion->state);
        motionsPerRegion_[new_region].push_back(motion);

        // update region stats

        regionMotions_[new_region]++;
        regionsChanged.insert(new_region);

        numMotions_++;

        // done
        if (goal->isSatisfied(motion->state))
        {
            goalMotion = motion;
            break;
        }
    }

    regionSelections_[region]++;
    regionsChanged.insert(region);

    for(std::set<int>::iterator it = regionsChanged.begin(); it != regionsChanged.end(); ++it)
        updateRegionWeight(*it);

    si_->freeState(xstate);

    return goalMotion;
}*/

ompl::geometric::HiLo::Motion* ompl::geometric::HiLo::expandFromRegion(const base::PlannerTerminationCondition &ptc, int from, int to, int maxExpansions)
{
    assert(motionsPerRegion_[from].size() > 0);

    base::Goal* goal = pdef_->getGoal().get();
    base::GoalSampleableRegion* gsr = dynamic_cast<base::GoalSampleableRegion*>(goal);

    base::State* xstate = si_->allocState();

    std::set<int> regionsChanged;

    Motion* goalMotion = NULL;

    for(int i = 0; i < maxExpansions && !ptc; ++i)
    {
        Motion* existing = selectMotionFromRegion(from);

        // Sample a state in region to (with goal bias)
        if (gsr && rng_.uniform01() < goalBias_ && gsr->canSample())
        {
            gsr->sampleGoal(xstate);

            // // Make sure goal states are diverse
            // bool unique = true;
            // for(size_t i = 0; i < goalMotions_.size() && unique; ++i)
            //     unique = si_->distance(goalMotions_[i]->state, xstate) > magic::UNIQUE_GOAL_DISTANCE;

            // // If we cannot find a unique goal, sample an existing one
            // if (!unique)
            // {
            //     exists = true;
            //     existsIdx = rng_.uniformInt(0, goalMotions_.size()-1);
            //     si_->copyState(xstate, goalMotions_[existsIdx]->state);
            // }
        }
        else
           if (!decomposition_->sampleFromRegion(to, xstate, existing->state))
            continue;

        // motion is not valid
        if (!si_->checkMotion(existing->state, xstate))
            continue;

        // Add new motion
        Motion* motion = new Motion(si_);
        si_->copyState(motion->state, xstate);
        motion->parent = existing;
        int new_region = decomposition_->locateRegion(motion->state);
        motionsPerRegion_[new_region].push_back(motion);

        // update region stats

        regionMotions_[new_region]++;
        regionsChanged.insert(new_region);

        numMotions_++;

        // done
        if (goal->isSatisfied(motion->state))
            goalMotion = motion;

        // added a motion... we're done here
        break;
    }

    regionSelections_[from]++;
    regionsChanged.insert(from);

    for(std::set<int>::iterator it = regionsChanged.begin(); it != regionsChanged.end(); ++it)
        updateRegionWeight(*it);

    si_->freeState(xstate);

    return goalMotion;
}

ompl::geometric::HiLo::Motion* ompl::geometric::HiLo::selectMotionFromRegion(int region)
{
    //assert(motionsPerRegion_[region].size() > 0);

    if (region >= motionsPerRegion_.size())
        throw std::runtime_error("Region is out of bounds");
    if (motionsPerRegion_[region].size() == 0)
        throw std::runtime_error("No motions in region to sample from");

    return motionsPerRegion_[region][rng_.uniformInt(0, motionsPerRegion_[region].size()-1)];
}

void ompl::geometric::HiLo::updateRegionWeight(int region)
{
    // weight is the distance estimate of the region to the goal, weighted by selections^2 / motions^2;
    double h = decomposition_->distanceHeuristic(region, goalRegions_[0]); // TODO: make better.  Cache this somewhere?
    double r = (1.0 + regionSelections_[region] * regionSelections_[region]) / (1.0 + regionMotions_[region] * regionMotions_[region]);

    // std::cout << __FUNCTION__ << " (" << region << ")" << std::endl;
    // std::cout << "  h = " << h << "   r = " << r << std::endl;

    // add 1 to heuristic to prohibit weight of zero at goal
    regionWeights_[region] = (1.0 + h) * r;
}

void ompl::geometric::HiLo::getNeighbors(int rid, std::vector<std::pair<int, double> >& neighbors) const
{
    std::vector<int> nbrs;
    decomposition_->getNeighbors(rid, nbrs);

    for(size_t i = 0; i < nbrs.size(); ++i)
    {
        double w = (regionWeights_[nbrs[i]] != std::numeric_limits<double>::infinity()) ? regionWeights_[nbrs[i]] : 1.0;
        neighbors.push_back(std::make_pair(nbrs[i], w));
    }
}

struct OpenListNode
{
    int id;
    int parent;
    double g, h;

    OpenListNode() : id(-1), parent(-1), g(0.0), h(std::numeric_limits<double>::infinity()) {}
    OpenListNode(int _id) : id(_id), parent(-1), g(0.0), h(std::numeric_limits<double>::infinity()) {}

    bool operator < (const OpenListNode& other) const
    {
        return (g + h) > (other.g + other.h); // priority queue is a max heap, but we want nodes with smaller g+h to have higher priority
    }
};

bool ompl::geometric::HiLo::shortestPath(int r1, int r2, std::vector<int>& path)
{
    if (r1 < 0 || r1 >= decomposition_->getNumRegions())
    {
        OMPL_ERROR("Start region (%d) is not valid", r1);
        return false;
    }

    if (r2 < 0 || r2 >= decomposition_->getNumRegions())
    {
        OMPL_ERROR("Goal region (%d) is not valid", r2);
        return false;
    }

    // Initialize predecessors and open list
    std::fill(predecessors_.begin(), predecessors_.end(), -1);
    std::fill(closedList_.begin(), closedList_.end(), false);

    // Create empty open list
    std::priority_queue<OpenListNode> openList;

    // Add start node to open list
    OpenListNode start(r1);
    start.g = 0.0;
    start.h = decomposition_->distanceHeuristic(r1, r2);
    start.parent = r1; // start has a self-transition to parent
    openList.push(start);

    // A* search
    bool solution = false;
    while (!openList.empty())
    {
        OpenListNode node = openList.top();
        openList.pop();

        // been here before
        if (closedList_[node.id])
            continue;

        // mark node as 'been here'
        closedList_[node.id] = true;
        predecessors_[node.id] = node.parent;

        // found solution!
        if (node.id == r2)
        {
            solution = true;
            break;
        }

        // Go through neighbors and add them to open list, if necessary
        std::vector<std::pair<int, double> > neighbors;
        getNeighbors(node.id, neighbors);
        for(size_t i = 0; i < neighbors.size(); ++i)
        {
            // only add neighbors we have not visited
            if (!closedList_[neighbors[i].first])
            {
                OpenListNode nbr(neighbors[i].first);
                nbr.g = node.g + neighbors[i].second;
                nbr.h = decomposition_->distanceHeuristic(neighbors[i].first, r2);
                nbr.parent = node.id;

                openList.push(nbr);
            }
        }
    }

    if (solution)
    {
        path.clear();
        int current = r2;
        while (predecessors_[current] != current)
        {
            path.insert(path.begin(), current);
            current = predecessors_[current];
        }

        path.insert(path.begin(), current); // add start state
    }

    return solution;
}

void ompl::geometric::HiLo::getPlannerData(base::PlannerData &data) const
{
    Planner::getPlannerData(data);
    for(size_t i = 0; i < motionsPerRegion_.size(); ++i)
    {
        for(size_t j = 0; j < motionsPerRegion_[i].size(); ++j)
        {
            if (motionsPerRegion_[i][j]->parent)
            {
                data.addEdge(base::PlannerDataVertex(motionsPerRegion_[i][j]->parent->state, 0),
                             base::PlannerDataVertex(motionsPerRegion_[i][j]->state, 0));
            }
            else
                data.addVertex(base::PlannerDataVertex(motionsPerRegion_[i][j]->state, 0));
        }
    }
}
