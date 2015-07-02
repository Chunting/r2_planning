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

#include "ompl/geometric/planners/hilo/BiHiLo.h"
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/tools/config/SelfConfig.h"
#include <limits>
#include <queue>
#include <algorithm>

ompl::geometric::BiHiLo::BiHiLo(const base::SpaceInformationPtr &si) : ompl::base::Planner(si, "BiHiLo")
{
}

ompl::geometric::BiHiLo::BiHiLo(const base::SpaceInformationPtr &si, const HiLoDecompositionPtr& decomp) : ompl::base::Planner(si, "BiHiLo")
{
    setDecomposition(decomp);
}

ompl::geometric::BiHiLo::~BiHiLo()
{
    freeMemory();
}

void ompl::geometric::BiHiLo::clear()
{
    Planner::clear();

    freeMemory();

    treeRoots_.clear();
    numMotions_ = 0;

    trees_.clear();
    //regionWeights_.assign(decomposition_->getNumRegions(), std::numeric_limits<double>::infinity());
    startWeights_.assign(decomposition_->getNumRegions(), std::numeric_limits<double>::infinity());
    goalWeights_.assign(decomposition_->getNumRegions(), std::numeric_limits<double>::infinity());

    regionSelections_.assign(decomposition_->getNumRegions(), 0);
    regionMotions_.assign(decomposition_->getNumRegions(), 0);
}

void ompl::geometric::BiHiLo::freeMemory()
{
    for(size_t i = 0; i < trees_.size(); ++i) // iterate over all trees
    {
        for(size_t j = 0; j < trees_[i].size(); ++j) // iterate the regions this tree touches
        {
            for(size_t k = 0; k < trees_[i][j].size(); ++k) // iterate over all motions in this region
            {
                si_->freeState(trees_[i][j][k]->state);
                delete trees_[i][j][k];
            }
        }
    }
}

void ompl::geometric::BiHiLo::setDecomposition(const HiLoDecompositionPtr& decomp)
{
    decomposition_ = decomp;
    //graph_.reset(new HiLoGraph(decomposition_));
}

void ompl::geometric::BiHiLo::setup()
{
    Planner::setup();

    if (!decomposition_)
    {
        OMPL_ERROR("%s: Decomposition is not set.  Cannot continue setup.", getName().c_str());
        return;
    }

    trees_.clear();
    treeRoots_.clear();

    //regionWeights_.assign(decomposition_->getNumRegions(), std::numeric_limits<double>::infinity());
    startWeights_.assign(decomposition_->getNumRegions(), std::numeric_limits<double>::infinity());
    goalWeights_.assign(decomposition_->getNumRegions(), 10);
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

int ompl::geometric::BiHiLo::addNewTree(const base::State* state)
{
    int region = decomposition_->locateRegion(state);

    Motion* motion = new Motion(si_);
    si_->copyState(motion->state, state);
    motion->parent = NULL;
    motion->root = motion;

    // Create a new tree and add the motion to the tree
    trees_.push_back(RegionMotionVec(decomposition_->getNumRegions(), std::vector<Motion*>()));
    trees_.back()[region].push_back(motion);

    // Update the set of root nodes
    treeRoots_.push_back(std::make_pair(motion, region));

    // And the number of motions
    regionMotions_[region]++;
    numMotions_++;

    return region;
}

#ifdef DEBUG_LEADS
#include <fstream>
#endif
ompl::base::PlannerStatus ompl::geometric::BiHiLo::solve(const base::PlannerTerminationCondition &ptc)
{
    checkValidity();

    if (!decomposition_)
    {
        OMPL_ERROR("%s: Decomposition is not set.  Cannot continue.", getName().c_str());
        return ompl::base::PlannerStatus::UNKNOWN;
    }

    bool added_start = false;
    // Extract one start state if there are no existing trees
    if (trees_.size() == 0)
    {
        while (const base::State *s = pis_.nextStart())
        {
            addNewTree(s);
            added_start = true;
            break;
        }
    }

    if (trees_.size() == 0)
    {
        OMPL_ERROR("%s: No valid start states", getName().c_str());
        return base::PlannerStatus::INVALID_START;
    }

    bool added_goal = false;
    // Extract one goal state if we do not have any goal trees
    if (trees_.size() == 1)
    {
        while (const base::State *g = pis_.nextGoal(ptc))
        {
            addNewTree(g);
            added_goal = true;
            break;
        }
    }


    if (trees_.size() < 2)
    {
        OMPL_ERROR("%s: Unable to sample a valid goal state", getName().c_str());
        return base::PlannerStatus::INVALID_GOAL;
    }

    if (added_start)
        updateRegionWeight(treeRoots_[0].second);
    if (added_goal)
        updateRegionWeight(treeRoots_[1].second);

    OMPL_INFORM("%s: Operating over %d dimensional decomposition with %d total regions", getName().c_str(), decomposition_->getDimension(), decomposition_->getNumRegions());

    // params
    unsigned int maxRegionExpansions_ = 20;
    int maxTreeExpansions = 5;

    bool solved = false;
    unsigned int numLeads = 0;

    #ifdef DEBUG_LEADS
    std::ofstream fout;
    fout.open("leads.txt");
    #endif

    bool startTree = true;

    base::State* xstate = si_->allocState();

    while (!ptc && !solved)
    {
        // randomly select a goal tree
        int tree = startTree ? 0 : rng_.uniformInt(1, trees_.size()-1);

        std::vector<int> lead;
        computeLead(lead, tree);
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
            int from, to;
            selectRegion(lead, tree, from, to);
            solved = expandFromRegion(ptc, tree, from, to, maxTreeExpansions, xstate); // expand from to to and then try to join trees. if trees are joined, true is returned
            numExpansions++;
        }

        startTree = !startTree;
    }

    si_->freeState(xstate);

    #ifdef DEBUG_LEADS
    fout.close();
    #endif

    OMPL_INFORM("%s: Created %u states using %u total leads", getName().c_str(), numMotions_, numLeads);

    if (solved)
        return ompl::base::PlannerStatus::EXACT_SOLUTION;

    return ompl::base::PlannerStatus::TIMEOUT;
}

void ompl::geometric::BiHiLo::computeLead(std::vector<int>& lead, int tree)
{
    int start, end;
    int tries = 0;
    int maxTries = 5;
    bool success = false;

    start = treeRoots_[tree].second;
    do
    {
        end = tree != 0 ? treeRoots_[0].second : treeRoots_[rng_.uniformInt(1, treeRoots_.size()-1)].second; // random goal tree
        tries++;

        success = shortestPath(start, end, tree == 0, lead);
    }
    while (!success && tries < maxTries);

    if (!success)
        throw ompl::Exception("%s: Failed to compute lead", getName().c_str());
}

void ompl::geometric::BiHiLo::selectRegion(const std::vector<int>& lead, int tree, int& from, int& to)
{
    std::vector<double> leadWeights;
    double w = 0.0;
    for(size_t i = 0; i < lead.size(); ++i)
    {
        if (trees_[tree][lead[i]].size() == 0)
            break;

        //w += 1.0 / regionWeights_[lead[i]];
        w += 1.0 / (tree == 0 ? startWeights_[lead[i]] : goalWeights_[lead[i]]);
        leadWeights.push_back(w);
    }

    double totalWeight = leadWeights.back();
    if (leadWeights.size() == 0)
        throw;

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

bool ompl::geometric::BiHiLo::expandFromRegion(const base::PlannerTerminationCondition &ptc, int tree,
                                               int from, int to, int maxExpansions, base::State* xstate)
{
    assert(trees_[tree][from].size() > 0);

    base::Goal* goal = pdef_->getGoal().get();
    //base::GoalSampleableRegion* gsr = dynamic_cast<base::GoalSampleableRegion*>(goal);
    //base::State* xstate = si_->allocState();

    std::set<int> regionsChanged;

    for(int i = 0; i < maxExpansions && !ptc; ++i)
    {
        Motion* existing = selectMotionFromRegion(tree, from);

        // Sample a state in region to
        if (!decomposition_->sampleFromRegion(to, xstate, existing->state))
            continue;

        // motion is not valid
        if (!si_->checkMotion(existing->state, xstate))
            continue;

        // Add new motion
        Motion* motion = new Motion(si_);
        si_->copyState(motion->state, xstate);
        motion->parent = existing;
        motion->root = existing->root;
        int new_region = decomposition_->locateRegion(motion->state);
        trees_[tree][new_region].push_back(motion);

        // update region stats
        regionMotions_[new_region]++;
        regionsChanged.insert(new_region);

        numMotions_++;

        // Try to connect the motion we just added to another tree
        if (joinTrees(ptc, tree, motion, new_region))
            return true;
    }

    regionSelections_[from]++;
    regionsChanged.insert(from);

    for(std::set<int>::iterator it = regionsChanged.begin(); it != regionsChanged.end(); ++it)
        updateRegionWeight(*it);

    //si_->freeState(xstate);

    return false;
}

bool ompl::geometric::BiHiLo::joinTrees(const base::PlannerTerminationCondition &ptc, int tree, Motion* motion, int region)
{
    if (tree == 0)
    {
        // check that there are motions from the goal tree(s) in the same region
        for(size_t i = 1; i < trees_.size(); ++i)
            if (trees_[i][region].size() > 0 && joinTrees(ptc, tree, motion, i, region))
                return true;
    }
    else
    {
        if (trees_[0][region].size() > 0 && joinTrees(ptc, tree, motion, 0, region))
            return true;
    }

    return false;
}

bool ompl::geometric::BiHiLo::joinTrees(const base::PlannerTerminationCondition &ptc, int tree, Motion* motion, int otherTree, int region)
{
    assert(trees_[otherTree][region].size() > 0);

    int tries = std::min((int)trees_[otherTree][region].size(), 5); // cap the number of tries
    std::vector<bool> tried(trees_[otherTree][region].size(), false);

    for(int i = 0; i < tries && !ptc; ++i)
    {
        Motion* other;
        int idx = -1;

        if (tries == tried.size())
        {
            idx = i;
        }
        else
        {
            // randomly select one that we haven't tried
            do
            {
                idx = rng_.uniformInt(0, tried.size()-1);
            } while(tried[idx]);

            assert(tried[idx] == false);
        }

        assert(idx >= 0);

        other = trees_[otherTree][region][idx];
        tried[idx] = true;

        // win.  Construct solution path
        if (si_->checkMotion(motion->state, other->state))
        {
            std::vector<Motion*> startPath, goalPath;
            Motion* solution = tree == 0 ? motion : other;
            while (solution != NULL)
            {
                startPath.push_back(solution);
                solution = solution->parent;
            }

            solution = tree == 0 ? other : motion;
            while (solution != NULL)
            {
                goalPath.push_back(solution);
                solution = solution->parent;
            }

            PathGeometric *path = new PathGeometric(si_);
            path->getStates().reserve(startPath.size() + goalPath.size());
            for(int i = startPath.size()-1; i >= 0; --i)
                path->append(startPath[i]->state);
            for(size_t i = 0; i < goalPath.size(); ++i)
                path->append(goalPath[i]->state);

            pdef_->addSolutionPath(base::PathPtr(path), false, 0.0, getName());
            return true;
        }
    }

    return false;
}

ompl::geometric::BiHiLo::Motion* ompl::geometric::BiHiLo::selectMotionFromRegion(int tree, int region)
{
    //assert(motionsPerRegion_[region].size() > 0);

    if (region >= trees_[tree].size())
        throw std::runtime_error("Region is out of bounds");
    if (trees_[tree][region].size() == 0)
        throw std::runtime_error("No motions in region to sample from");

    return trees_[tree][region][rng_.uniformInt(0, trees_[tree][region].size()-1)];
}

void ompl::geometric::BiHiLo::updateRegionWeight(int region)
{
    // update start weight

    // weight is the distance estimate of the region to the goal, weighted by selections^2 / motions^2;
    int goalRegion = treeRoots_[1].second;
    double h = decomposition_->distanceHeuristic(region, goalRegion); // TODO: make better.  Cache this somewhere?  Iterate over all goals and pick the smallest one?
    double r = (1.0 + regionSelections_[region] * regionSelections_[region]) / (1.0 + regionMotions_[region] * regionMotions_[region]);

    // add 1 to heuristic to prohibit weight of zero at goal
    startWeights_[region] = (1.0 + h) * r;


    // update goal weight
    goalRegion = treeRoots_[0].second;
    h = decomposition_->distanceHeuristic(region, goalRegion); // TODO: make better.  Cache this somewhere?
    // add 1 to heuristic to prohibit weight of zero at goal
    goalWeights_[region] = (1.0 + h) * r;

    // Bias goal weights lower in regions explored by the start tree
    //if (trees_[0][region].size() > 0)
    //    goalWeights_[region] = 1.0 / trees_[0][region].size();
}

void ompl::geometric::BiHiLo::getNeighbors(int rid, bool startWeights, std::vector<std::pair<int, double> >& neighbors) const
{
    std::vector<int> nbrs;
    decomposition_->getNeighbors(rid, nbrs);

    for(size_t i = 0; i < nbrs.size(); ++i)
    {
        double w;
        if (startWeights)
            w = (startWeights_[nbrs[i]] != std::numeric_limits<double>::infinity()) ? startWeights_[nbrs[i]] : 1.0;
        else w = (goalWeights_[nbrs[i]] != std::numeric_limits<double>::infinity()) ? goalWeights_[nbrs[i]] : 1.0;

        //double w = (regionWeights_[nbrs[i]] != std::numeric_limits<double>::infinity()) ? regionWeights_[nbrs[i]] : 1.0;
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

bool ompl::geometric::BiHiLo::shortestPath(int r1, int r2, bool startToGoal, std::vector<int>& path) const
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

    std::vector<int> predecessors(decomposition_->getNumRegions(), -1);
    std::vector<bool> closedList(decomposition_->getNumRegions(), false);
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
        if (closedList[node.id])
            continue;

        // mark node as 'been here'
        closedList[node.id] = true;
        predecessors[node.id] = node.parent;

        // found solution!
        if (node.id == r2)
        {
            solution = true;
            break;
        }

        // Go through neighbors and add them to open list, if necessary
        std::vector<std::pair<int, double> > neighbors;
        getNeighbors(node.id, startToGoal, neighbors);
        for(size_t i = 0; i < neighbors.size(); ++i)
        {
            // only add neighbors we have not visited
            if (!closedList[neighbors[i].first])
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
        while (predecessors[current] != current)
        {
            path.insert(path.begin(), current);
            current = predecessors[current];
        }

        path.insert(path.begin(), current); // add start state
    }

    return solution;
}

void ompl::geometric::BiHiLo::getPlannerData(base::PlannerData &data) const
{
    Planner::getPlannerData(data);
    for(size_t i = 0; i < trees_.size(); ++i)
    {
        for(size_t j = 0; j < trees_[i].size(); ++j)
        {
            for(size_t k = 0; k < trees_[i][j].size(); ++k)
            {
                if (trees_[i][j][k]->parent)
                {
                    data.addEdge(base::PlannerDataVertex(trees_[i][j][k]->parent->state, 0),
                                 base::PlannerDataVertex(trees_[i][j][k]->state, 0));
                }
                else
                    data.addVertex(base::PlannerDataVertex(trees_[i][j][k]->state, 0));
            }
        }
    }
}
