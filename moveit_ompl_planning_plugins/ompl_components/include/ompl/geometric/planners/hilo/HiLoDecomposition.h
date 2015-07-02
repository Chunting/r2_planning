/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2011, Rice University
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

#ifndef OMPL_GEOMETRIC_PLANNERS_HILO_DECOMPOSITION_
#define OMPL_GEOMETRIC_PLANNERS_HILO_DECOMPOSITION_

#include "ompl/base/spaces/RealVectorBounds.h"
#include "ompl/base/StateSampler.h"
#include "ompl/base/State.h"
//#include "ompl/util/Console.h"
#include "ompl/util/Exception.h"
#include "ompl/util/ClassForward.h"
#include "ompl/util/RandomNumbers.h"

namespace ompl
{
    namespace geometric
    {

        /// @cond IGNORE
        /** \brief Forward declaration of ompl::geometric::HiLoDecomposition */
        OMPL_CLASS_FORWARD(HiLoDecomposition);
        /// @endcond

        /** \class ompl::geometric::HiLoDecompositionPtr
            \brief A boost shared pointer wrapper for ompl::geometric::HiLoDecomposition */

        /** \brief */
        class HiLoDecomposition
        {
        public:

            /** \brief Constructor.  */
            HiLoDecomposition()
            {
            }

            virtual ~HiLoDecomposition()
            {
            }

            /** \brief Returns the number of regions in this HiLoDecomposition. */
            virtual int getNumRegions() const = 0;

            /** \brief Return the dimension of this HiLoDecomposition */
            virtual int getDimension() const = 0;

            /** \brief Returns the index of the region containing a given State.
             * Most often, this is obtained by first calling project().
             * Returns -1 if no region contains the State. */
            virtual int locateRegion(const base::State *s) const = 0;

            /** \brief Project the given State into the HiLoDecomposition */
            virtual void project(const base::State *s, std::vector<double>& coord) const = 0;

            /** \brief Stores the given region's neighbors into a given vector. */
            virtual void getNeighbors(int rid, std::vector<int>& neighbors) const = 0;

            /** \brief An admissible and consistent distance heuristic between two regions */
            virtual double distanceHeuristic(int r1, int r2) const = 0;

            /** \brief Sample a state \e s from region r */
            virtual bool sampleFromRegion(int r, base::State* s, const base::State* seed = NULL) const = 0;
        };
    }
}
#endif
