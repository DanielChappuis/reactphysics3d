/********************************************************************************
* ReactPhysics3D physics library, http://code.google.com/p/reactphysics3d/      *
* Copyright (c) 2010-2012 Daniel Chappuis                                       *
*********************************************************************************
*                                                                               *
* This software is provided 'as-is', without any express or implied warranty.   *
* In no event will the authors be held liable for any damages arising from the  *
* use of this software.                                                         *
*                                                                               *
* Permission is granted to anyone to use this software for any purpose,         *
* including commercial applications, and to alter it and redistribute it        *
* freely, subject to the following restrictions:                                *
*                                                                               *
* 1. The origin of this software must not be misrepresented; you must not claim *
*    that you wrote the original software. If you use this software in a        *
*    product, an acknowledgment in the product documentation would be           *
*    appreciated but is not required.                                           *
*                                                                               *
* 2. Altered source versions must be plainly marked as such, and must not be    *
*    misrepresented as being the original software.                             *
*                                                                               *
* 3. This notice may not be removed or altered from any source distribution.    *
*                                                                               *
********************************************************************************/

#ifndef AABB_H
#define AABB_H

// Libraries
#include "../mathematics/mathematics.h"

// ReactPhysics3D namespace
namespace reactphysics3d {

// Declaration
class Body;
    
/*  -------------------------------------------------------------------
    Class AABB :
        This class represents a bounding volume of type "Axis Aligned
        Bounding Box". It's a box where all the edges are always aligned
        with the world coordinate system. The AABB is defined by the
        minimum and maximum world coordinates of the three axis.
    -------------------------------------------------------------------
*/
class AABB {
    private :
        Vector3 minCoordinates;        // Minimum world coordinates of the AABB on the x,y and z axis
        Vector3 maxCoordinates;        // Maximum world coordinates of the AABB on the x,y and z axis
        Body* bodyPointer;             // Pointer to the owner body (not the abstract class Body but its derivative which is instanciable)

    public :
        AABB();                                                                                     // Constructor
        AABB(const Vector3& minCoordinates, const Vector3& maxCoordinates, Body* bodyPointer);      // Constructor
        AABB(const Transform& transform, const Vector3& extents);                                   // Constructor
        virtual ~AABB();                                                                            // Destructor

        Vector3 getCenter() const;                                                     // Return the center point
        const Vector3& getMin() const;                                                  // Return the minimum coordinates of the AABB
        const Vector3& getMax() const;                                      // Return the maximum coordinates of the AABB
        Body* getBodyPointer() const;                                                  // Return a pointer to the owner body
        void setBodyPointer(Body* bodyPointer);                                        // Set the body pointer
        bool testCollision(const AABB& aabb) const;                                    // Return true if the current AABB is overlapping is the AABB in argument
        virtual void update(const Transform& newTransform, const Vector3& extents);    // Update the oriented bounding box orientation according to a new orientation of the rigid body
#ifdef VISUAL_DEBUG
       virtual void draw() const;                                                      // Draw the AABB (only for testing purpose)
#endif
};

// Return the center point of the AABB in world coordinates
inline Vector3 AABB::getCenter() const {
    return (minCoordinates + maxCoordinates) * 0.5;
}

// Return the minimum coordinates of the AABB
inline const Vector3& AABB::getMin() const {
    return minCoordinates;
}

// Return the maximum coordinates of the AABB
inline const Vector3& AABB::getMax() const {
    return maxCoordinates;
}

// Return a pointer to the owner body
inline Body* AABB::getBodyPointer() const {
    return bodyPointer;
}

// Set the body pointer
inline void AABB::setBodyPointer(Body* bodyPointer) {
    this->bodyPointer = bodyPointer;
}

// Return true if the current AABB is overlapping with the AABB in argument
// Two AABB overlap if they overlap in the three x, y and z axis at the same time
inline bool AABB::testCollision(const AABB& aabb) const {
    if (maxCoordinates.getX() < aabb.minCoordinates.getX() || aabb.maxCoordinates.getX() < minCoordinates.getX()) return false;
    if (maxCoordinates.getY() < aabb.minCoordinates.getY() || aabb.maxCoordinates.getY() < minCoordinates.getY()) return false;
    if (maxCoordinates.getZ() < aabb.minCoordinates.getZ() || aabb.maxCoordinates.getZ() < minCoordinates.getZ()) return false;
    return true;
}

// Update the world minimum and maximum coordinates of the AABB on the three x,y and z axis
inline void AABB::update(const Transform& newTransform, const Vector3& extents) {
    Matrix3x3 worldAxis = newTransform.getOrientation().getMatrix().getAbsoluteMatrix();
    Vector3 worldExtents = Vector3(worldAxis.getColumn(0).dot(extents),
                                     worldAxis.getColumn(1).dot(extents),
                                     worldAxis.getColumn(2).dot(extents));
    minCoordinates = newTransform.getPosition() - worldExtents;
    maxCoordinates = newTransform.getPosition() + worldExtents;
}

}; // End of the ReactPhysics3D namespace

#endif
