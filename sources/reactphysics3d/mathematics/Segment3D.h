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

#ifndef SEGMENT3D_H
#define SEGMENT3D_H

// Libraries
#include "Vector3D.h"

// ReactPhysics3D namespace
namespace reactphysics3d {

/*  -------------------------------------------------------------------
    Class Segment3D :
        This class represents a line segment in a 3D space. A segment is
        described by two points A and B.
    -------------------------------------------------------------------
*/
class Segment3D {
    private :
        Vector3D pointA;        // Point A of the line segment
        Vector3D pointB;        // Point B of the line segment

    public :
        Segment3D();                                        // Constructor
        Segment3D(Vector3D& pointA, Vector3D& pointB);      // Constructor with arguments
        Segment3D(const Segment3D& segment);                // Copy-constructor
        ~Segment3D();                                       // Destructor

        Vector3D getPointA() const;                         // Return the point A
        void setPointA(const Vector3D& pointA);             // Set the point A
        Vector3D getPointB() const;                         // Return the point B
        void setPointB(const Vector3D& pointB);             // Set the point B
};

inline Vector3D Segment3D::getPointA() const {
    return pointA;
}

inline void Segment3D::setPointA(const Vector3D& pointA) {
    this->pointA = pointA;
}

inline Vector3D Segment3D::getPointB() const {
    return pointB;
}

inline void Segment3D::setPointB(const Vector3D& pointB) {
    this->pointB = pointB;
}

} // End of the ReactPhysics3D namespace

#endif
