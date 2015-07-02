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

#ifndef OMPL_GEOMETRIC_PLANNERS_CASPER_CASPER_DISCRETIZATION_
#define OMPL_GEOMETRIC_PLANNERS_CASPER_CASPER_DISCRETIZATION_

#include "ompl/base/Planner.h"
#include "ompl/datastructures/GridB.h"
#include "ompl/util/Exception.h"
#include <boost/function.hpp>
#include <vector>
#include <limits>
#include <cassert>
#include <utility>
#include <cstdlib>
#include <cmath>

namespace ompl
{

    namespace geometric
    {

        /// \brief One-level Discretization used for CASPER
        template<typename Motion>
        class CasperDiscretization
        {
        protected:

            /*typedef std::vector<Motion*> MotionVec;
            /// A 1-D grid for the cost portion of the CasperDiscretization
            class CostGrid : public Grid<MotionVec>
            {
            public:
                /// Definition of a cell in this grid
                typedef typename Grid<MotionVec>::Cell Cell;
                /// Datatype for array of cells in base class
                typedef typename Grid<MotionVec>::CellArray CellArray;
                /// Datatype for cell coordinates
                typedef typename Grid<MotionVec>::Coord     Coord;

                explicit
                CostGrid() : Grid<MotionVec>(1), frontier(NULL)
                {
                }

                virtual ~CostGrid()
                {
                }

                virtual void add(Cell *cell)
                {
                    Grid<MotionVec>::add(cell);
                    if (frontier == NULL || cell->coord[0] < frontier->coord[0])
                        frontier = cell;
                }

                virtual bool remove(Cell *cell)
                {
                    // find the new frontier
                    if (cell == frontier)
                    {
                        std::vector<Coord*> coords;
                        this->getCoordinates(coords);
                        Cell* newFrontier(NULL);

                        for(size_t i = 0; i < coords.size(); ++i)
                        {
                            if (coords[i]->at(0) == cell->coord[0])
                                continue;

                            if (newFrontier == NULL || coords[i]->at(0) < newFrontier->coord[0])
                                newFrontier = this->getCell(*(coords[i]));
                        }

                        frontier = newFrontier;
                    }

                    return Grid<MotionVec>::remove(cell);
                }

                unsigned int addMotion(Motion* motion, const Coord& coord)
                {
                    Cell *cell = this->getCell(coord);

                    unsigned int created = 0;
                    if (cell)
                    {
                        cell->data.push_back(motion);
                    }
                    else
                    {
                        cell = this->createCell(coord);
                        cell->data.push_back(motion);
                        add(cell);
                        created = 1;
                    }
                    return created;
                }

                Cell* getFrontierCell() const
                {
                    return frontier;
                }

            protected:
                Cell* frontier;
            };*/

            /*/// \brief A 1-dimensional discretization of cost
            class CostDiscretization
            {
            public:
                CostDiscretization(double bucketSize = 25.0) : bucketSize_(bucketSize)
                {
                }

                void setBucketSize(double bucketSize)
                {
                    bucketSize_ = bucketSize;
                }

                double getBucketSize() const
                {
                    return bucketSize_;
                }

                int getBucket(const base::Cost& cost)
                {
                    return (int)floor(cost.value() / bucketSize_);
                }

            protected:
                double bucketSize_;
            };*/


        public:

            /** \brief The data held by a cell in the grid of motions */
            struct CellData
            {
                CellData(unsigned int k) : coverage(0.0), selections(1), score(1.0), iteration(0), importance(0.0), topk(k, NULL)
                {
                    assert(k > 0);
                }

                ~CellData()
                {
                }

                void addMotion(Motion* motion)
                {
                    motions.push_back(motion);
                    updateTopk(motion);
                }

                // Update the topk motions, given a new/updated motion
                void updateTopk(Motion* motion)
                {
                    // is this motion in top-k already?
                    bool inTopk = false;
                    for(size_t i = 0; i < topk.size(); ++i)
                    {
                        if (topk[i] == NULL)
                            break;

                        if (motion == topk[i])
                        {
                            inTopk = true;
                            break;
                        }
                    }

                    if (inTopk)
                        sortTopk();
                    else
                    {
                        double cost = motion->cost.value();

                        // update top-k, if necessary (insertion sort)
                        if (topk.back() == NULL || cost < topk.back()->cost.value())
                        {
                            size_t idx = 0;  // the index where motion should be inserted in sorted order

                            // 1) Find index to insert new element
                            for(size_t i = 0; i < topk.size(); ++i, ++idx)
                            {
                                if (topk[i] == NULL || topk[i]->cost.value() > cost)
                                    break;
                            }

                            assert(idx < topk.size());

                            // 2) Shift all elements starting at idx to the right.
                            for(unsigned int i = topk.size()-1; i > idx; --i)
                                topk[i] = topk[i-1];

                            // 3) Insert new motion at idx
                            topk[idx] = motion;
                        }
                    }
                }

                void sortTopk()
                {
                    // Bubble sort.  I know, but the topk should be
                    // mostly sorted, so we just need to switch a
                    // couple of elements
                    bool swapped = false;
                    do
                    {
                        swapped = false;
                        for(size_t i = 1; i < topk.size()-1; ++i)
                        {
                            if (topk[i] == NULL)
                                break;

                            if (topk[i-1]->cost.value() > topk[i]->cost.value())
                            {
                                std::swap(topk[i-1], topk[i]);
                                swapped = true;
                            }
                        }
                    } while (swapped);
                }

                /** \brief The set of motions contained in this grid cell */
                std::vector<Motion*> motions
;
                /** \brief A measure of coverage for this cell. For
                    this implementation, this is the sum of motion
                    lengths */
                double               coverage;

                /** \brief The number of times this cell has been
                    selected for expansion */
                unsigned int         selections;

                /** \brief A heuristic score computed based on
                    distance to goal (if available), successes and
                    failures at expanding from this cell. */
                double               score;

                /** \brief The iteration at which this cell was created */
                unsigned int         iteration;

                /** \brief The computed importance (based on other class members) */
                double               importance;

                /// \brief A discretization of the states in this cell based on their cost-to-come
                //CostGrid             costGrid;

                /// The "top-k" set of motions with minimal cost, stored in sorted order
                /// These motions are the 'low cost frontier' in this cell
                std::vector<Motion*> topk;
            };

            /** \brief Definintion of an operator passed to the Grid
                structure, to order cells by importance */
            struct OrderCellsByImportance
            {

                /** \brief Order function */
                bool operator()(const CellData * const a, const CellData * const b) const
                {
                    return a->importance > b->importance;
                }
            };

            /** \brief The datatype for the maintained grid datastructure */
            typedef GridB<CellData*, OrderCellsByImportance> BaseGrid;

            /** \brief The datatype for the maintained grid cells */
            typedef typename BaseGrid::Cell  Cell;

            /** \brief The datatype for the maintained grid coordinates */
            typedef typename BaseGrid::Coord Coord;

            /** \brief The signature of a function that frees the memory for a motion */
            typedef typename boost::function<void(Motion*)> FreeMotionFn;

            CasperDiscretization(const FreeMotionFn &freeMotion, unsigned int k = 5) : grid_(0), size_(0),
                                                                                       iteration_(1), recentCell_(NULL),
                                                                                       freeMotion_(freeMotion), k_(k)
            {
                grid_.onCellUpdate(computeImportance, NULL);
                selectBorderFraction_ = 0.9;
            }

            ~CasperDiscretization()
            {
                freeMemory();
            }

            /*void setCostDiscretizationCellSize(double costCellSize)
            {
                costDisc_.setBucketSize(costCellSize);
            }

            double getCostDiscretizationCellSize() const
            {
                return costDisc_.getBucketSize();
            }*/

            unsigned int getK() const
            {
                return k_;
            }

            void setK(unsigned int k)
            {
                k_ = k;
            }

            /** \brief Set the fraction of time for focusing on the
                border (between 0 and 1). This is the minimum fraction
                used to select cells that are exterior (minimum
                because if 95% of cells are on the border, they will
                be selected with 95% chance, even if this fraction is
                set to 90%)*/
            void setBorderFraction(double bp)
            {
                if (bp < std::numeric_limits<double>::epsilon() || bp > 1.0)
                    throw Exception("The fraction of time spent selecting border cells must be in the range (0,1]");
                selectBorderFraction_ = bp;
            }

            /** \brief Set the fraction of time for focusing on the
                border (between 0 and 1). */
            double getBorderFraction() const
            {
                return selectBorderFraction_;
            }

            /** \brief Set the dimension of the grid to be maintained */
            void setDimension(unsigned int dim)
            {
                grid_.setDimension(dim);
            }

            unsigned int getDimension() const
            {
                return grid_.getDimension();
            }

            /** \brief Restore the CasperDiscretization to its original form */
            void clear()
            {
                freeMemory();
                size_ = 0;
                iteration_ = 1;
                recentCell_ = NULL;
            }

            void countIteration()
            {
                ++iteration_;
            }

            std::size_t getMotionCount() const
            {
                return size_;
            }

            std::size_t getCellCount() const
            {
                return grid_.size();
            }

            // Return the nth motion in the discretization
            // 0 <= n < size_
            Motion* nthMotion(unsigned int n) const
            {
                std::vector<CellData*> cdata;
                grid_.getContent(cdata);
                unsigned int count = 0;

                for (unsigned int i = 0 ; i < cdata.size() ; ++i)
                {
                    if (count + cdata[i]->motions.size() <= n)
                        count += cdata[i]->motions.size();
                    else
                        return cdata[i]->motions[n - count];
                }

                std::cout << "SOMETHING BAD HAPPENED" << std::endl;
                return NULL;
            }

            /** \brief Free the memory for the motions contained in a grid */
            void freeMemory()
            {
                for (typename BaseGrid::iterator it = grid_.begin(); it != grid_.end() ; ++it)
                    freeCellData(it->second->data);
                grid_.clear();
            }

            /** \brief Add a motion to the grid containing motions. As
                a hint, \e dist specifies the distance to the goal
                from the state of the motion being added. The function
                returns the number of cells created to accommodate the
                new motion (0 or 1). The CostDiscretization takes
                ownership of the motion passed as argument, and the
                memory for the motion is freed by calling the function
                passed to the constructor. */
            unsigned int addMotion(Motion *motion, const Coord &coord, double dist = 0.0)
            {
                Cell *cell = grid_.getCell(coord);

                unsigned int created = 0;
                if (cell)
                {
                    /////cell->data->motions.push_back(motion);  // in original KPIECE discretizatino
                    cell->data->addMotion(motion);
                    cell->data->coverage += 1.0;

                    //std::vector<int> costCoord(1);
                    ////costCoord[0] = costDisc_.getBucket(motion->cost);
                    //cell->data->costGrid.addMotion(motion, costCoord);

                    grid_.update(cell);
                }
                else
                {
                    cell = grid_.createCell(coord);
                    cell->data = new CellData(k_);
                    /////cell->data->motions.push_back(motion);
                    cell->data->addMotion(motion);
                    cell->data->coverage = 1.0;
                    cell->data->iteration = iteration_;
                    cell->data->selections = 1;
                    cell->data->score = (1.0 + log((double)(iteration_))) / (1.0 + dist);

                    //std::vector<int> costCoord(1);
                    //costCoord[0] = costDisc_.getBucket(motion->cost);
                    //cell->data->costGrid.addMotion(motion, costCoord);

                    grid_.add(cell);
                    recentCell_ = cell;
                    created = 1;
                }
                ++size_;
                return created;
            }

            /** \brief Select a motion and the cell it is part of from
                the grid of motions. This is where preference is given
                to cells on the boundary of the grid.*/
            void selectMotion(const base::PlannerTerminationCondition &ptc, Motion* &smotion, Cell* &scell,
                              const base::OptimizationObjectivePtr& opt, const base::Cost& costThreshold)
            {
                do
                {
                    selectMotionHelper(smotion, scell);
                } while (!ptc && opt->isCostBetterThan(costThreshold, smotion->cost));
            }

            void selectMotionHelper(Motion* &smotion, Cell* &scell)
            {
                bool frontierCell = rng_.uniform01() < std::max(selectBorderFraction_, grid_.fracExternal());

                scell = frontierCell ? grid_.topExternal() : grid_.topInternal();

                // We are running on finite precision, so our update scheme will end up
                // with 0 values for the score. This is where we fix the problem
                if (scell->data->score < std::numeric_limits<double>::epsilon())
                {
                    std::vector<CellData*> content;
                    content.reserve(grid_.size());
                    grid_.getContent(content);
                    for (typename std::vector<CellData*>::iterator it = content.begin() ; it != content.end() ; ++it)
                        (*it)->score += 1.0 + log((double)((*it)->iteration));
                    grid_.updateAll();
                }

                assert(scell && !scell->data->motions.empty());

                ++scell->data->selections;

                double topk_pct = 0.9; // select a motion uniformly from the topk this pct of the time

                bool topk = (rng_.uniform01() < topk_pct);
                if (topk)
                    smotion = scell->data->topk[rng_.uniformInt(0, std::min(scell->data->motions.size() - 1, scell->data->topk.size() - 1))];
                else
                    smotion = scell->data->motions[rng_.halfNormalInt(0, scell->data->motions.size() - 1)];
            }

            bool removeMotion(Motion *motion, const Coord &coord)
            {
                Cell* cell = grid_.getCell(coord);
                if (cell)
                {
                    bool found = false;
                    for (unsigned int i = 0 ; i < cell->data->motions.size(); ++i)
                        if (cell->data->motions[i] == motion)
                        {
                            cell->data->motions.erase(cell->data->motions.begin() + i);
                            found = true;
                            --size_;
                            break;
                        }
                    if (cell->data->motions.empty())
                    {
                        grid_.remove(cell);
                        freeCellData(cell->data);
                        grid_.destroyCell(cell);
                    }
                    return found;
                }
                return false;
            }

            void updateMotion(Motion* motion, const Coord& coord)
            {
                Cell* cell = grid_.getCell(coord);
                if (!cell)
                    return;

                cell->data->updateTopk(motion);
            }

            void updateCell(Cell *cell)
            {
                grid_.update(cell);
            }

            const BaseGrid& getGrid() const
            {
                return grid_;
            }

            void getPlannerData(base::PlannerData &data, int tag, bool start, const Motion *lastGoalMotion) const
            {
                std::vector<CellData*> cdata;
                grid_.getContent(cdata);

                if (lastGoalMotion)
                    data.addGoalVertex (base::PlannerDataVertex(lastGoalMotion->state, tag));

                for (unsigned int i = 0 ; i < cdata.size() ; ++i)
                    for (unsigned int j = 0 ; j < cdata[i]->motions.size() ; ++j)
                    {
                        if (cdata[i]->motions[j]->parent == NULL)
                        {
                            if (start)
                                data.addStartVertex(base::PlannerDataVertex(cdata[i]->motions[j]->state, tag));
                            else
                                data.addGoalVertex(base::PlannerDataVertex(cdata[i]->motions[j]->state, tag));
                        }
                        else
                        {
                            if (start)
                                data.addEdge(base::PlannerDataVertex(cdata[i]->motions[j]->parent->state, tag),
                                             base::PlannerDataVertex(cdata[i]->motions[j]->state, tag));
                            else
                                data.addEdge(base::PlannerDataVertex(cdata[i]->motions[j]->state, tag),
                                             base::PlannerDataVertex(cdata[i]->motions[j]->parent->state, tag));
                        }
                    }
            }

        private:

            /** \brief Free the memory for the data contained in a grid cell */
            void freeCellData(CellData *cdata)
            {
                for (unsigned int i = 0 ; i < cdata->motions.size() ; ++i)
                    freeMotion_(cdata->motions[i]);
                delete cdata;
            }

            /** \brief This function is provided as a callback to the
                grid datastructure to update the importance of a
                cell */
            static void computeImportance(Cell *cell, void*)
            {
                CellData &cd = *(cell->data);
                cd.importance =  cd.score / ((cell->neighbors + 1) * cd.coverage * cd.selections);
            }

            /** \brief A grid containing motions, imposed on a
                projection of the state space */
            BaseGrid     grid_;

            /** \brief The total number of motions (there can be
                multiple per cell) in the grid */
            std::size_t  size_;

            /** \brief The number of iterations performed on this tree */
            unsigned int iteration_;

            /** \brief The most recently created cell */
            Cell        *recentCell_;

            /** \brief Method that can free the memory for a stored motion */
            FreeMotionFn freeMotion_;

            /** \brief The fraction of time to focus exploration on
                the border of the grid. */
            double       selectBorderFraction_;

            /** \brief The random number generator */
            RNG          rng_;

            // The number of low cost states to maintain
            unsigned int k_;

            /** \brief A 1D discretization of the cost-to-come for each motion */
            //CostDiscretization costDisc_;
        };
    }
}

#endif
