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
#include "Polygon3D.h"

// We want to use the ReactPhysics3D namespace
using namespace reactphysics3d;

// Constructor
Polygon3D::Polygon3D(const std::vector<Vector3D>& vertexList)
          :vertexList(vertexList) {

}

// Destructor
Polygon3D::~Polygon3D() {

}

// TODO : Delete this function if not needed
// Return true if a point is in the same plane and inside the polygon
// Hypothesis : The point has to be on the same plane as the polygon.
/*
bool Polygon3D::isPointInside(const Vector3D& point) const {
    // TODO : Implement this method
}
*/

// TODO : Delete this function if not needed
// Return the intersection point between a segment and an egde of the polygon.
// Hypothesis : The segment must be in the same plane as the polygon and it
//              must intersection with at least one edge of the polygon.
/*
Vector3D Polygon3D::getEdgeIntersectionWithSegment(const Segment3D& segment) const {
    // TODO : Implement this method
}
*/


