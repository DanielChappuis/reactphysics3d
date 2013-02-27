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
#include "../../mathematics/mathematics.h"

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

        // -------------------- Attributes -------------------- //

        // Minimum world coordinates of the AABB on the x,y and z axis
        Vector3 mMinCoordinates;

        // Maximum world coordinates of the AABB on the x,y and z axis
        Vector3 mMaxCoordinates;

        // Pointer to the owner body (not the abstract class Body
        // but its derivative which is instanciable)
        Body* mBodyPointer;

        // -------------------- Methods -------------------- //

        // Private copy-constructor
        AABB(const AABB& aabb);

        // Private assignment operator
        AABB& operator=(const AABB& aabb);

    public :

        // -------------------- Methods -------------------- //

        // Constructor
        AABB();

        // Constructor
        AABB(const Vector3& minCoordinates, const Vector3& maxCoordinates, Body* modyPointer);

        // Constructor
        AABB(const Transform& transform, const Vector3& extents);

        // Destructor
        virtual ~AABB();

        // Return the center point
        Vector3 getCenter() const;

        // Return the minimum coordinates of the AABB
        const Vector3& getMin() const;

        // Return the maximum coordinates of the AABB
        const Vector3& getMax() const;

        // Return a pointer to the owner body
        Body* getBodyPointer() const;

        // Set the body pointer
        void setBodyPointer(Body* bodyPointer);

        // Return true if the current AABB is overlapping is the AABB in argument
        bool testCollision(const AABB& aabb) const;

        // Update the oriented bounding box orientation
        // according to a new orientation of the rigid body
        virtual void update(const Transform& newTransform, const Vector3& extents);

#ifdef VISUAL_DEBUG
       // Draw the AABB (only for testing purpose)
       virtual void draw() const;
#endif
};

// Return the center point of the AABB in world coordinates
inline Vector3 AABB::getCenter() const {
    return (mMinCoordinates + mMaxCoordinates) * 0.5;
}

// Return the minimum coordinates of the AABB
inline const Vector3& AABB::getMin() const {
    return mMinCoordinates;
}

// Return the maximum coordinates of the AABB
inline const Vector3& AABB::getMax() const {
    return mMaxCoordinates;
}

// Return a pointer to the owner body
inline Body* AABB::getBodyPointer() const {
    return mBodyPointer;
}

// Set the body pointer
inline void AABB::setBodyPointer(Body* bodyPointer) {
    mBodyPointer = bodyPointer;
}

// Return true if the current AABB is overlapping with the AABB in argument
// Two AABB overlap if they overlap in the three x, y and z axis at the same time
inline bool AABB::testCollision(const AABB& aabb) const {
    if (mMaxCoordinates.x < aabb.mMinCoordinates.x ||
        aabb.mMaxCoordinates.x < mMinCoordinates.x) return false;
    if (mMaxCoordinates.y < aabb.mMinCoordinates.y ||
        aabb.mMaxCoordinates.y < mMinCoordinates.y) return false;
    if (mMaxCoordinates.z < aabb.mMinCoordinates.z||
        aabb.mMaxCoordinates.z < mMinCoordinates.z) return false;
    return true;
}

// Update the world minimum and maximum coordinates of the AABB on the three x,y and z axis
inline void AABB::update(const Transform& newTransform, const Vector3& extents) {
    Matrix3x3 worldAxis = newTransform.getOrientation().getMatrix().getAbsoluteMatrix();
    Vector3 worldExtents = Vector3(worldAxis.getColumn(0).dot(extents),
                                     worldAxis.getColumn(1).dot(extents),
                                     worldAxis.getColumn(2).dot(extents));
    mMinCoordinates = newTransform.getPosition() - worldExtents;
    mMaxCoordinates = newTransform.getPosition() + worldExtents;
}

}; // End of the ReactPhysics3D namespace

#endif
