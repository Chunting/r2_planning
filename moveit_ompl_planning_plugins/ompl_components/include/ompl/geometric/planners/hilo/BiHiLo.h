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

#ifndef OMPL_GEOMETRIC_PLANNERS_HILO_BIHILO_
#define OMPL_GEOMETRIC_PLANNERS_HILO_BIHILO_

#include "ompl/geometric/planners/PlannerIncludes.h"
#include "ompl/geometric/planners/hilo/HiLoDecomposition.h"

namespace ompl
{
    namespace geometric
    {
        /// \brief Bi-directional HiLo
        class BiHiLo : public base::Planner
        {
        public:
            // Standard planner constructor.  Must call setDecomposition before calling solve()
            BiHiLo(const base::SpaceInformationPtr &si);

            // Initialize BiHiLo with the given decomposition
            BiHiLo(const base::SpaceInformationPtr &si, const HiLoDecompositionPtr& decomp);

            virtual ~BiHiLo();

            virtual void getPlannerData(base::PlannerData &data) const;

            virtual base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc);

            virtual void clear();

            virtual void setup();

            void setDecomposition(const HiLoDecompositionPtr& decomp);

        protected:
            /// \brief The definition of a motion in the search tree
            class Motion
            {
            public:

                Motion() : state(NULL), parent(NULL)
                {
                }

                /// \brief Constructor that allocates memory for the state
                Motion(const base::SpaceInformationPtr &si) : state(si->allocState()), parent(NULL)
                {
                }

                ~Motion()
                {
                }

                /// \brief The state contained by the motion
                base::State         *state;

                /// \brief The parent motion in the exploration tree
                Motion              *parent;

                /// \brief Pointer to the root of the tree this motion lies in
                const Motion        *root;

                /// \brief The children of this motion
                //std::vector<Motion*> children;
            };

            // Deallocate all memory from the planner
            void freeMemory();

            // Compute a new lead in the decomposition given current tree
            void computeLead(std::vector<int>& lead, int tree);

            // Select a region from the given lead to expand from
            //int selectRegion(const std::vector<int>& lead);

            // Select a region from the given lead to expand from
            //void selectRegion(const std::vector<int>& lead, int& from, int& to);

            void selectRegion(const std::vector<int>& lead, int tree, int& from, int& to);

            Motion* selectMotionFromRegion(int tree, int region);

            // Expand the tree from the given region.  Return a goal motion, if one is found, else null.
            //virtual Motion* expandFromRegion(const base::PlannerTerminationCondition &ptc, int region, int maxExpansions);

            // Expand the tree from the given region to an adjacent region.  Return a goal motion, if one is found, else null.
            //virtual Motion* expandFromRegion(const base::PlannerTerminationCondition &ptc, int from, int to, int maxExpansions);

            // Returns true if a solution path was found
            virtual bool expandFromRegion(const base::PlannerTerminationCondition &ptc, int tree, int from,
                                          int to, int maxExpansions, base::State* xstate);

            // Update the weight of this region
            void updateRegionWeight(int region);

            // Return a list of neighbors and the edge weights from rid
            void getNeighbors(int rid, bool startWeights, std::vector<std::pair<int, double> >& neighbors) const;

            // Shortest (weight) path from r1 to r2
            bool shortestPath(int r1, int r2, bool startToGoal, std::vector<int>& path) const;

            // Add a new tree rooted at state.  Return the region number of the tree root
            int addNewTree(const base::State* state);

            // Try to join tree at motion in region to another tree to form a solution path
            // If returns true, the trees were joined and a solution path was added
            bool joinTrees(const base::PlannerTerminationCondition &ptc, int tree, Motion* motion, int region);

            // Try to join tree and otherTree using motion in tree in region.
            bool joinTrees(const base::PlannerTerminationCondition &ptc, int tree, Motion* motion, int otherTree, int region);


            HiLoDecompositionPtr decomposition_;

            /// \brief Random number generator
            RNG rng_;

            /// \brief The maximum length of a motion to be added to a tree
            double range_;

            /// \brief The total number of states in the tree
            unsigned int numMotions_;

            /// Percentage of time we sample a goal state
            double goalBias_;

            base::StateSamplerPtr sampler_;

            // A vector of motions for each distinct region
            typedef std::vector<std::vector<Motion*> > RegionMotionVec;
            // A vector of motions for each tree
            // Assumed the first tree is start tree and subsequent tree(s) are goal trees
            std::vector<RegionMotionVec> trees_;

            // The root state of each tree
            std::vector<std::pair<Motion*, int> > treeRoots_;

            // Set of region weights for start and goal trees
            //std::vector<double> regionWeights_;
            std::vector<double> startWeights_;
            std::vector<double> goalWeights_;


            // number of times a region has been selected for expansion
            std::vector<unsigned int> regionSelections_;
            // number of motions per region
            std::vector<unsigned int> regionMotions_;


            /*std::vector<std::vector<Motion*> > motionsPerRegion_;
            std::vector<std::pair<Motion*, int> > startMotions_;
            std::vector<int> goalRegions_;

            std::vector<double> regionWeights_;
            // number of times a region has been selected for expansion
            std::vector<unsigned int> regionSelections_;
            // number of motions per region
            std::vector<unsigned int> regionMotions_;*/
        };
    }
}

#endif