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

#ifndef POLYGON3D_H
#define POLYGON3D_H

// Libraries
#include <vector>
#include <stdexcept>
#include "Vector3D.h"

// ReactPhysics3D namespace
namespace reactphysics3d {

/*  -------------------------------------------------------------------
    Class Polygon3D :
        This class represents a 3D polygon.
    -------------------------------------------------------------------
*/
class Polygon3D {
    private :
        std::vector<Vector3D> vertexList;    // Ordered list of all vertices of the polygon

    public :
        Polygon3D(const std::vector<Vector3D>& vertexList);          // Constructor
        ~Polygon3D();                                                // Destructor

        int getNbVertex() const;                                                     // Return the number of vertex of the polygon
        Vector3D getVertex(unsigned int index) const throw(std::invalid_argument);   // Return a vertex of the polygon
        bool isPointInside(const Vector3D& point) const;                             // Return true if a point is in the same plane and inside the polygon
        // TODO : Delete the following function if not needed
        //Vector3D getEdgeIntersectionWithSegment(const Segment3D& segment) const;     // Return the intersection point between a segment and an egde of the polygon
};

// TODO : Test the Polygon3D class

// Return the number of vertex of the polygon
inline int Polygon3D::getNbVertex() const {
    return vertexList.size();
}

// Return a vertex of the polygon
inline Vector3D Polygon3D::getVertex(unsigned int index) const throw(std::invalid_argument) {
    // Check if the index argument is valid
    if (index >= 0 && index < vertexList.size()) {
        return vertexList[index];
    }
    else {
        // The index is invalid, we throw an exception
        throw std::invalid_argument("Exception : The index argument has to be between zero and the number of vertex of the polygon");
    }
}

} // End of the ReactPhysics3D namespace

#endif


