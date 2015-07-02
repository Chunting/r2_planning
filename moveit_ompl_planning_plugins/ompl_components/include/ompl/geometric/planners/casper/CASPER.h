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

#ifndef OMPL_GEOMETRIC_PLANNERS_CASPER_CASPER_
#define OMPL_GEOMETRIC_PLANNERS_CASPER_CASPER_

#define LOG_EXPLORATION

#include "ompl/datastructures/Grid.h"
#include "ompl/geometric/planners/PlannerIncludes.h"
#include "ompl/base/ProjectionEvaluator.h"
#include "ompl/base/OptimizationObjective.h"
#include "ompl/geometric/planners/casper/CasperDiscretization.h"

namespace ompl
{
    namespace geometric
    {
        class CASPER : public base::Planner
        {
        public:
            CASPER(const base::SpaceInformationPtr &si);
            virtual ~CASPER();

            virtual void getPlannerData(base::PlannerData &data) const;

            virtual base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc);

            virtual void clear();

            virtual void setup();

            /// \brief Set the goal bias probability.
            ///
            /// In the process of random sampling, the algorithm may in fact
            /// choose a goal state with some probability (the goal bias).
            /// This probability is a real number between 0.0 and 1.0,
            /// typically small (e.g., 0.05).
            void setGoalBias(double goalBias)
            {
                goalBias_ = goalBias;
            }

            /// \brief Retrieve the goal bias probability.
            double getGoalBias() const
            {
                return goalBias_;
            }

            /// \brief Set the maximum distance of any single motion in the
            /// search tree.
            void setRange(double distance)
            {
                maxDistance_ = distance;
            }

            /// \brief Retrieve the maximum distance of any single motion in
            /// the search tree.
            double getRange() const
            {
                return maxDistance_;
            }

            double getExploreTime() const
            {
                return exploreTime_;
            }

            void setExploreTime(double t)
            {
                if (t < 1e-4)
                    OMPL_ERROR("Explore time must be at least 0.0001");
                else
                    exploreTime_ = t;
            }

            unsigned int getK() const
            {
                return disc_.getK();
            }

            void setK(unsigned int k)
            {
                disc_.setK(k);
            }

            /// \brief Set the projection evaluator. This class is
            /// able to compute the projection of a given state.
            void setProjectionEvaluator(const base::ProjectionEvaluatorPtr &projectionEvaluator)
            {
                projectionEvaluator_ = projectionEvaluator;
            }

            void setProjectionEvaluator(const std::string &name)
            {
                projectionEvaluator_ = si_->getStateSpace()->getProjection(name);
            }

            const base::ProjectionEvaluatorPtr& getProjectionEvaluator() const
            {
                return projectionEvaluator_;
            }

            /// \brief Returns a string representation of the best solution path cost found
            std::string getBestCost() const
            {
                if (!opt_)
                    return "inf";

                base::Cost bestCost = opt_->infiniteCost();
                for(size_t i = 0; i < goalMotions_.size(); ++i)
                    if (opt_->isCostBetterThan(goalMotions_[i]->cost, bestCost))
                        bestCost = goalMotions_[i]->cost;

                return boost::lexical_cast<std::string>(bestCost);
            }


        protected:
            /// \brief The definition of a motion in the search tree
            class Motion
            {
            public:

                Motion() : state(NULL), parent(NULL), cost(0.0), goal(false)
                {
                }

                /// \brief Constructor that allocates memory for the state
                Motion(const base::SpaceInformationPtr &si) : state(si->allocState()), parent(NULL), cost(0.0), goal(false)
                {
                }

                ~Motion()
                {
                }

                /// \brief The state contained by the motion
                base::State         *state;

                /// \brief The parent motion in the exploration tree
                Motion              *parent;

                /// \brief The total cost accumulated from the start to this motion
                base::Cost           cost;

                /// \brief The children of this motion
                std::vector<Motion*> children;

                /// \brief A flag indicating whether this motion ends at the goal
                bool                 goal;
            };

            void freeMotion(Motion* motion);

            //bool transitionTest(const base::Cost& childCost, const base::Cost& parentCost);
            //bool transitionTest(double d_cost, unsigned int iteration);

            //bool findSolution(const base::PlannerTerminationCondition &ptc, Motion*& solution, const base::Cost& costThreshold);
            //void refine(const base::PlannerTerminationCondition &ptc, Motion*& solution);

            //Motion* addMotion(Motion* parent, const base::State* state, const base::Cost& motion_cost,
            //                  double distance_goal, const CasperDiscretization<Motion>::Coord& coord);

            bool explore(const base::PlannerTerminationCondition &ptc, Motion* xmotion, base::Cost& threshold);
            void shortcutVertices(const base::PlannerTerminationCondition &ptc, Motion* xmotion);
            void shortcutPath(const base::PlannerTerminationCondition &ptc, Motion* xmotion);
            void updateChildCosts(const base::PlannerTerminationCondition &ptc, Motion* motion);

            void gradientDescentStates(const base::PlannerTerminationCondition& ptc, Motion* xmotion);
            void gradientDescentPathShortcut(const base::PlannerTerminationCondition& ptc, Motion* xmotion);

            void gradientDescentPath(const base::PlannerTerminationCondition& ptc, Motion* xmotion);

            void extractMotionSolutionPath(std::vector<Motion*>& path);

            /// \brief This algorithm uses a discretization (a grid) to guide the exploration. The exploration is imposed on a projection of the state space.
            //base::ProjectionEvaluatorPtr projectionEvaluator_;

            /// \brief The fraction of time the goal is picked as the state to expand towards (if such a state is available)
            double                       goalBias_;

            /// \brief The maximum length of a motion to be added to a tree
            double                       maxDistance_;

            /// \brief The random number generator
            RNG                          rng_;

            /// \brief The most recent goal motion.  Used for PlannerData computation
            Motion                       *lastGoalMotion_;

            /// \brief The objective (cost) being optimized.
            ompl::base::OptimizationObjectivePtr opt_;

            CasperDiscretization<Motion> disc_;

            base::StateSamplerPtr        sampler_;

            std::vector<Motion*>         goalMotions_;

            /// \brief This algorithm uses a discretization (a grid)
            /// to guide the exploration. The exploration is imposed
            /// on a projection of the state space.
            base::ProjectionEvaluatorPtr projectionEvaluator_;

            //base::Cost bestCost_;
            //base::Cost worstCost_;
            //double temp_;
            //double tempChangeFactor_;

            double failedExpansionScoreFactor_;

            // The amount of time spent exploring in one shot
            double exploreTime_;


////////////////////////////////////////////////////////////////////////////////////////////////
#ifdef LOG_EXPLORATION
        public:
            struct LogEntry
            {
                enum Action{EXPLORE, REFINE, STATE};
                Action act;

                LogEntry(Action action) : act(action) {}
                Action getAction() const { return act; }

            };

            struct ExploreEntry : public LogEntry
            {
                ExploreEntry(const base::SpaceInformationPtr &si, base::ProjectionEvaluatorPtr projectionEvaluator,
                             Motion* f, const CasperDiscretization<Motion>::Coord& cf,
                             Motion* t, const CasperDiscretization<Motion>::Coord& ct) : LogEntry(LogEntry::EXPLORE), from(projectionEvaluator->getDimension()), to(projectionEvaluator->getDimension())
                {
                    projectionEvaluator->project(f->state, from);
                    projectionEvaluator->project(t->state, to);

                    cellFrom = cf;
                    cellTo = ct;

                    fromID = f;
                    toID = t;
                }

                CasperDiscretization<Motion>::Coord cellFrom, cellTo;
                base::EuclideanProjection from, to;
                Motion *fromID,  *toID;
            };

            struct RefineEntry : public LogEntry
            {
                RefineEntry(const base::SpaceInformationPtr &si, base::ProjectionEvaluatorPtr projectionEvaluator,
                            /*const std::vector<Motion*>& original,*/ const std::vector<Motion*>& shortcut) : LogEntry(LogEntry::REFINE)
                {
                    /*ids = original;
                    //refinement.resize(shortcut.size(), base::EuclideanProjection(projectionEvaluator->getDimension()));
                    for(size_t i = 0; i < shortcut.size(); ++i)
                    {
                        base::EuclideanProjection proj(projectionEvaluator->getDimension());
                        projectionEvaluator->project(shortcut[i]->state, proj);
                        refinement.push_back(proj);
                    }

                    if (ids.size() != refinement.size())
                        OMPL_ERROR("Expected ids and refinement to have same size");*/



                    for(size_t i = 0; i < shortcut.size(); ++i)
                    {
                        base::EuclideanProjection proj(projectionEvaluator->getDimension());
                        projectionEvaluator->project(shortcut[i]->state, proj);
                        refinement.push_back(proj);
                        ids.push_back(shortcut[i]);

                        CasperDiscretization<Motion>::Coord coord;
                        projectionEvaluator->computeCoordinates(shortcut[i]->state, coord);
                        cells.push_back(coord);
                    }
                }

                std::vector<Motion*> ids;
                std::vector<base::EuclideanProjection> refinement;
                std::vector<CasperDiscretization<Motion>::Coord> cells;
            };

            struct StateEntry : public LogEntry
            {
                StateEntry(const base::SpaceInformationPtr &si, base::ProjectionEvaluatorPtr projectionEvaluator,
                           Motion* m, const CasperDiscretization<Motion>::Coord& coord, bool _goal) : LogEntry(LogEntry::STATE), state(projectionEvaluator->getDimension())
                {
                    goal = _goal;
                    projectionEvaluator->project(m->state, state);

                    cell = coord;

                    id = m;
                }

                bool goal;
                //Motion* motion;
                base::EuclideanProjection state;
                CasperDiscretization<Motion>::Coord cell;
                Motion* id;
            };

            const std::vector<LogEntry*>& getExplorationLog() const
            {
                return exploreLog_;
            }

        protected:
            std::vector<LogEntry*> exploreLog_;

#endif
////////////////////////////////////////////////////////////////////////////////////////////////
        };
    }
}

#endif