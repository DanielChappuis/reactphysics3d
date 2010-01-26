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

#ifndef BOUNDING_VOLUME_H
#define BOUNDING_VOLUME_H

// Libraries
#include "../mathematics/mathematics.h"
#include "Body.h"
#include <cassert>

// ReactPhysics3D namespace
namespace reactphysics3d {

/*  -------------------------------------------------------------------
    Class BoundingVolume :
        This class represents the volume that contains a rigid body.
        This volume will be used to compute the collisions with others
        bodies.
    -------------------------------------------------------------------
*/
class BoundingVolume {
    protected :
        Body* body;                      // Pointer to the body

    public :
        BoundingVolume();                // Constructor
        virtual ~BoundingVolume();       // Destructor

        Body* getBodyPointer() const;     // Return the body pointer
        void setBodyPointer(Body* body);  // Set the body pointer

        virtual void updateOrientation(const Vector3D& newCenter, const Quaternion& rotationQuaternion)=0;      // Update the orientation of the bounding volume according to the new orientation of the body
        virtual std::vector<Vector3D> getExtremeVertices(const Vector3D& axis) const=0;                           // Return all the vertices that are projected at the extreme of the projection of the bouding volume on the axis
        virtual void draw() const=0;                                                                                    // Display the bounding volume (only for testing purpose)
};

// Return the body pointer
inline Body* BoundingVolume::getBodyPointer() const {
    assert(body != 0);
    return body;
}

// Set the body pointer
inline void BoundingVolume::setBodyPointer(Body* body) {
    this->body = body;
}


} // End of the ReactPhysics3D namespace

#endif
