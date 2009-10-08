/****************************************************************************
* Copyright (C) 2009      Daniel Chappuis                                  *
****************************************************************************
* This file is part of ReactPhysics3D.                                     *
*                                                                          *
* ReactPhysics3D is free software: you can redistribute it and/or modify   *
* it under the terms of the GNU Lesser General Public License as published *
* by the Free Software Foundation, either version 3 of the License, or     *
* (at your option) any later version.                                      *
*                                                                          *
* ReactPhysics3D is distributed in the hope that it will be useful,        *
* but WITHOUT ANY WARRANTY; without even the implied warranty of           *
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the             *
* GNU Lesser General Public License for more details.                      *
*                                                                          *
* You should have received a copy of the GNU Lesser General Public License *
* along with ReactPhysics3D. If not, see <http://www.gnu.org/licenses/>.   *
***************************************************************************/

// Libraries
#include "ProjectionInterval.h"
#include <iostream>

// We want to use the ReactPhysics3D namespace
using namespace reactphysics3d;

// Constructor
ProjectionInterval::ProjectionInterval()
                   :minType(VERTEX), maxType(VERTEX) {
    boundingVolume = 0;
    min = 0;
    max = 0;
}

// Constructor
ProjectionInterval::ProjectionInterval(const BoundingVolume* const boudingVolume, double min, double max, ExtremeType minType, ExtremeType maxType, std::vector<Vector3D> minProjectedPoints, std::vector<Vector3D> maxProjectedPoints)
                   :min(min), max(max), minType(minType), maxType(maxType), minProjectedPoints(minProjectedPoints), maxProjectedPoints(maxProjectedPoints) {
    this->boundingVolume = boundingVolume;
}

// Destructor
ProjectionInterval::~ProjectionInterval() {

}
