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

#include "ompl/geometric/planners/hilo/HiLoPoseDecomposition.h"
#include "ompl/util/Exception.h"
#include "ompl/util/Console.h"
#include <limits>

#include <boost/math/constants/constants.hpp>
#define PI boost::math::constants::pi<double>()
#define TWOPI boost::math::constants::two_pi<double>()

ompl::geometric::HiLoPoseDecomposition::HiLoPoseDecomposition(const base::RealVectorBounds& xyzBounds, const std::vector<int>& xyzSlices,
                                                              const base::RealVectorBounds& rpyBounds, const std::vector<int>& rpySlices,
                                                              bool diagonalEdges)
    : HiLoDecomposition(), diagonalEdges_(diagonalEdges), xyzBounds_(xyzBounds), rpyBounds_(rpyBounds), xyzSlices_(xyzSlices), rpySlices_(rpySlices)
{
    if (xyzSlices_.size() != 3)
        throw Exception("%s: xyzSlices must have length 3", __FUNCTION__);
    if (rpySlices_.size() != 3)
        throw Exception("%s: rpySlices must have length 3", __FUNCTION__);

    if (xyzBounds.low.size() != 3)
        throw Exception("%s: xyzBounds must be three dimensional");
    if (rpyBounds.low.size() != 3)
        throw Exception("%s: rpyBounds must be three dimensional");

    xyzBounds.check();
    rpyBounds.check();

    numRegions_ = 1;
    for(size_t i = 0; i < xyzSlices_.size(); ++i)
    {
        if (xyzSlices_[i] < 0)
            throw Exception("%s: Number of xyzSlices must be positive", __FUNCTION__);
        if (rpySlices_[i] < 0)
            throw Exception("%s: Number of rpySlices must be positive", __FUNCTION__);

        numRegions_ *= xyzSlices_[i];
        numRegions_ *= rpySlices_[i];
    }

    // region volume will be the position part only...
    dx_ = fabs(xyzBounds.high[0] - xyzBounds.low[0]);
    dy_ = fabs(xyzBounds.high[1] - xyzBounds.low[1]);
    dz_ = fabs(xyzBounds.high[2] - xyzBounds.low[2]);
    //regionVolume_ = dx_ * dy_ * dz_;

    dR_ = fabs(rpyBounds.high[0] - rpyBounds.low[0]);
    dP_ = fabs(rpyBounds.high[1] - rpyBounds.low[1]);
    dY_ = fabs(rpyBounds.high[2] - rpyBounds.low[2]);

    // The size of each grid cell in the XYZ dimensions
    xSize_ = dx_ / xyzSlices_[0];
    ySize_ = dy_ / xyzSlices_[1];
    zSize_ = dz_ / xyzSlices_[2];

    // The size of each grid cell in the RPY dimensions
    RSize_ = dR_ / rpySlices_[0];
    PSize_ = dP_ / rpySlices_[1];
    YSize_ = dY_ / rpySlices_[2];
}

ompl::geometric::HiLoPoseDecomposition::~HiLoPoseDecomposition()
{
}

int ompl::geometric::HiLoPoseDecomposition::getNumRegions() const
{
    return numRegions_;
}

int ompl::geometric::HiLoPoseDecomposition::getDimension() const
{
    return 6;
}

int ompl::geometric::HiLoPoseDecomposition::locateRegion(const base::State *s) const
{
    std::vector<double> coord;
    project(s, coord);
    return coordToRegion(coord);
}

void ompl::geometric::HiLoPoseDecomposition::getNeighbors(int rid, std::vector<int>& neighbors) const
{
    // up, down, left, right for position dimensions
    // same for orientation, but we must handle the wrap around case carefully

    if (diagonalEdges_)
        getDiagonalNeighbors(rid, neighbors);
    else
        getNonDiagonalNeighbors(rid, neighbors);
}

void ompl::geometric::HiLoPoseDecomposition::getNonDiagonalNeighbors(int rid, std::vector<int>& neighbors) const
{
    std::vector<int> ridCell;
    ridToGridCell(rid, ridCell);

    std::vector<int> cell(ridCell.begin(), ridCell.end());
    std::vector<int> workCell(ridCell.begin(), ridCell.end());

    // xyz
    for(size_t i = 0; i < 3; ++i)
    {
        // There are no neighbors in this dimension
        if (xyzSlices_[i] == 1)
            continue;

        workCell[i] -= 1;
        if (workCell[i] >= 0 && workCell[i] < xyzSlices_[i])
            neighbors.push_back(gridCellToRid(workCell));

        workCell[i] += 2;
        if (workCell[i] >= 0 && workCell[i] < xyzSlices_[i])
            neighbors.push_back(gridCellToRid(workCell));
        workCell[i] = cell[i];
    }

    // rpy
    for(size_t i = 3; i < 6; ++i)
    {
        // There are no neighbors in this dimension
        if (rpySlices_[i-3] == 1)
            continue;

        workCell[i] -= 1;
        if (workCell[i] < 0)
            workCell[i] += rpySlices_[i-3];
        else if (workCell[i] >= rpySlices_[i-3])
            workCell[i] -= rpySlices_[i-3];
        neighbors.push_back(gridCellToRid(workCell));

        workCell[i] += 2;
        if (workCell[i] < 0)
            workCell[i] += rpySlices_[i-3];
        else if (workCell[i] >= rpySlices_[i-3])
            workCell[i] -= rpySlices_[i-3];
        neighbors.push_back(gridCellToRid(workCell));

        workCell[i] = cell[i];
    }
}

void ompl::geometric::HiLoPoseDecomposition::getDiagonalNeighbors(int rid, std::vector<int>& neighbors) const
{
    std::vector<int> ridCell;
    ridToGridCell(rid, ridCell);

    std::vector<int> cell(ridCell.begin(), ridCell.end());

    for(int x = -1; x <= 1; ++x)
    {
        int tX = ridCell[0] + x;
        if (tX >= 0 && tX < xyzSlices_[0])
            cell[0] = tX;
        else continue;

        for (int y = -1; y <= 1; ++y)
        {
            int tY = ridCell[1] + y;
            if (tY >= 0 && tY < xyzSlices_[1])
                cell[1] = tY;
            else continue;

            for(int z = -1; z <= 1; ++z)
            {
                int tZ = ridCell[2] + z;
                if (tZ >= 0 && tZ < xyzSlices_[2])
                    cell[2] = tZ;
                else continue;

                for (int R = -1; R <= 1; ++R)
                {
                    // No additional neighbors in this dimension
                    if (rpySlices_[0] == 0 && R != 0)
                        continue;

                    int tR = ridCell[3] + R;
                    if (tR < 0)
                        tR += rpySlices_[0];
                    else if (tR >= rpySlices_[0])
                        tR -= rpySlices_[0];
                    cell[3] = tR;

                    for (int P = -1; P <= 1; ++P)
                    {
                        // No additional neighbors in this dimension
                        if (rpySlices_[1] == 0 && P != 0)
                            continue;

                        int tP = ridCell[4] + P;
                        if (tP < 0)
                            tP += rpySlices_[1];
                        else if (tP >= rpySlices_[1])
                            tP -= rpySlices_[1];
                        cell[4] = tP;

                        for (int Y = -1; Y <= 1; ++Y)
                        {
                            // No additional neighbors in this dimension
                            if (rpySlices_[2] == 0 && Y != 0)
                                continue;

                            // Don't add ourself as a neighbor
                            if (x == y && y == z && z == R && R == P && P == Y && Y == 0)
                                continue;

                            int tYaw = ridCell[5] + Y;
                            if (tYaw < 0)
                                tYaw += rpySlices_[2];
                            else if (tYaw >= rpySlices_[2])
                                tYaw -= rpySlices_[2];
                            cell[5] = tYaw;

                            neighbors.push_back(gridCellToRid(cell));
                        }
                    }
                }
            }
        }
    }
}

int ompl::geometric::HiLoPoseDecomposition::coordToRegion(const std::vector<double>& coord) const
{
    // must perform computation about origin
    std::vector<int> cell(6);
    cell[0] = (coord[0] - xyzBounds_.low[0]) / xSize_;  // x
    cell[1] = (coord[1] - xyzBounds_.low[1]) / ySize_;  // y
    cell[2] = (coord[2] - xyzBounds_.low[2]) / zSize_;  // z
    cell[3] = (coord[3] - rpyBounds_.low[0]) / RSize_;  // R
    cell[4] = (coord[4] - rpyBounds_.low[1]) / PSize_;  // P
    cell[5] = (coord[5] - rpyBounds_.low[2]) / YSize_;  // Y

    return gridCellToRid(cell);
}

void ompl::geometric::HiLoPoseDecomposition::ridToGridCell(int rid, std::vector<int>& cell) const
{
    cell.resize(6);
    int cellsIn5Dims = xyzSlices_[0] * xyzSlices_[1] * xyzSlices_[2] * rpySlices_[0] * rpySlices_[1];
    int cellsIn4Dims = xyzSlices_[0] * xyzSlices_[1] * xyzSlices_[2] * rpySlices_[0];
    int cellsIn3Dims = xyzSlices_[0] * xyzSlices_[1] * xyzSlices_[2];
    int cellsIn2Dims = xyzSlices_[0] * xyzSlices_[1];

    // Work backward
    cell[5] = rid / cellsIn5Dims;

    rid %= cellsIn5Dims;
    cell[4] = rid / cellsIn4Dims;

    rid %= cellsIn4Dims;
    cell[3] = rid / cellsIn3Dims;

    rid %= cellsIn3Dims;
    cell[2] = rid / cellsIn2Dims;

    rid %= cellsIn2Dims;
    cell[1] = rid / xyzSlices_[0];

    rid %= xyzSlices_[0]; // mod shouldn't actually be necessary
    cell[0] = rid;
}

int ompl::geometric::HiLoPoseDecomposition::gridCellToRid(const std::vector<int>& cell) const
{
    unsigned int xyzOffset = xyzSlices_[0] * xyzSlices_[1] * xyzSlices_[2];

    int region = cell[0];                              // x
    region += cell[1] * xyzSlices_[0];                 // y
    region += cell[2] * xyzSlices_[0] * xyzSlices_[1]; // z
    region += cell[3] * xyzOffset;                                    // R
    region += cell[4] * xyzOffset * rpySlices_[0];                    // P
    region += cell[5] * xyzOffset * rpySlices_[0] * rpySlices_[1];    // Y

    return region;
}

double ompl::geometric::HiLoPoseDecomposition::distanceHeuristic(int r1, int r2) const
{
    std::vector<int> c1, c2;
    ridToGridCell(r1, c1);
    ridToGridCell(r2, c2);

    double diff = 0;
    for(size_t i = 0; i < 3; ++i) // xyz
        diff += abs(c1[i] - c2[i]);

    // rpy wraps around, so all diffs are mod orientationSlices_/2;
    for(size_t i = 3; i < c1.size(); ++i)
    {
        int min = std::min(c1[i], c2[i]);
        int max = std::max(c1[i], c2[i]);

        diff += std::min(abs(c1[i] - c2[i]), abs((min + rpySlices_[i-3]) - max));
    }

    return diff;
}

bool ompl::geometric::HiLoPoseDecomposition::hasDiagonalEdges() const
{
    return diagonalEdges_;
}