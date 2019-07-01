/********************************************************************************
* ReactPhysics3D physics library, http://www.reactphysics3d.com                 *
* Copyright (c) 2010-2019 Daniel Chappuis                                       *
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

#ifndef REACTPHYSICS3D_AABB_H
#define REACTPHYSICS3D_AABB_H

// Libraries
#include "mathematics/mathematics.h"

/// ReactPhysics3D namespace
namespace reactphysics3d {
    
// Class AABB
/**
 * This class represents a bounding volume of type "Axis Aligned
 * Bounding Box". It's a box where all the edges are always aligned
 * with the world coordinate system. The AABB is defined by the
 * minimum and maximum world coordinates of the three axis.
 */
class AABB {

    private :

        // -------------------- Attributes -------------------- //

        /// Minimum world coordinates of the AABB on the x,y and z axis
        Vector3 mMinCoordinates;

        /// Maximum world coordinates of the AABB on the x,y and z axis
        Vector3 mMaxCoordinates;

    public :

        // -------------------- Methods -------------------- //

        /// Constructor
        AABB() = default;

        /// Constructor
        AABB(const Vector3& minCoordinates, const Vector3& maxCoordinates);

        /// Copy-constructor
        AABB(const AABB& aabb);

        /// Destructor
        ~AABB() = default;

        /// Return the center point
        Vector3 getCenter() const;

        /// Return the minimum coordinates of the AABB
        const Vector3& getMin() const;

        /// Set the minimum coordinates of the AABB
        void setMin(const Vector3& min);

        /// Return the maximum coordinates of the AABB
        const Vector3& getMax() const;

        /// Set the maximum coordinates of the AABB
        void setMax(const Vector3& max);

        /// Return the size of the AABB in the three dimension x, y and z
        Vector3 getExtent() const;

        /// Inflate each side of the AABB by a given size
        void inflate(decimal dx, decimal dy, decimal dz);

        /// Return true if the current AABB is overlapping with the AABB in argument
        bool testCollision(const AABB& aabb) const;

        /// Return the volume of the AABB
        decimal getVolume() const;

        /// Merge the AABB in parameter with the current one
        void mergeWithAABB(const AABB& aabb);

        /// Replace the current AABB with a new AABB that is the union of two AABBs in parameters
        void mergeTwoAABBs(const AABB& aabb1, const AABB& aabb2);

        /// Return true if the current AABB contains the AABB given in parameter
        bool contains(const AABB& aabb) const;

        /// Return true if a point is inside the AABB
        bool contains(const Vector3& point) const;

        /// Return true if the AABB of a triangle intersects the AABB
        bool testCollisionTriangleAABB(const Vector3* trianglePoints) const;

        /// Return true if the ray intersects the AABB
        bool testRayIntersect(const Ray& ray) const;

        /// Create and return an AABB for a triangle
        static AABB createAABBForTriangle(const Vector3* trianglePoints);

        /// Assignment operator
        AABB& operator=(const AABB& aabb);

        // -------------------- Friendship -------------------- //

        friend class DynamicAABBTree;
};

// Return the center point of the AABB in world coordinates
inline Vector3 AABB::getCenter() const {
    return (mMinCoordinates + mMaxCoordinates) * decimal(0.5);
}

// Return the minimum coordinates of the AABB
inline const Vector3& AABB::getMin() const {
    return mMinCoordinates;
}

// Set the minimum coordinates of the AABB
inline void AABB::setMin(const Vector3& min) {
    mMinCoordinates = min;
}

// Return the maximum coordinates of the AABB
inline const Vector3& AABB::getMax() const {
    return mMaxCoordinates;
}

// Set the maximum coordinates of the AABB
inline void AABB::setMax(const Vector3& max) {
    mMaxCoordinates = max;
}

// Return the size of the AABB in the three dimension x, y and z
inline Vector3 AABB::getExtent() const {
  return  mMaxCoordinates - mMinCoordinates;
}

// Inflate each side of the AABB by a given size
inline void AABB::inflate(decimal dx, decimal dy, decimal dz) {
    mMaxCoordinates += Vector3(dx, dy, dz);
    mMinCoordinates -= Vector3(dx, dy, dz);
}

// Return true if the current AABB is overlapping with the AABB in argument.
/// Two AABBs overlap if they overlap in the three x, y and z axis at the same time
inline bool AABB::testCollision(const AABB& aabb) const {
    if (mMaxCoordinates.x < aabb.mMinCoordinates.x ||
        aabb.mMaxCoordinates.x < mMinCoordinates.x) return false;
    if (mMaxCoordinates.y < aabb.mMinCoordinates.y ||
        aabb.mMaxCoordinates.y < mMinCoordinates.y) return false;
    if (mMaxCoordinates.z < aabb.mMinCoordinates.z||
        aabb.mMaxCoordinates.z < mMinCoordinates.z) return false;
    return true;
}

// Return the volume of the AABB
inline decimal AABB::getVolume() const {
    const Vector3 diff = mMaxCoordinates - mMinCoordinates;
    return (diff.x * diff.y * diff.z);
}

// Return true if the AABB of a triangle intersects the AABB
inline bool AABB::testCollisionTriangleAABB(const Vector3* trianglePoints) const {

    if (min3(trianglePoints[0].x, trianglePoints[1].x, trianglePoints[2].x) > mMaxCoordinates.x) return false;
    if (min3(trianglePoints[0].y, trianglePoints[1].y, trianglePoints[2].y) > mMaxCoordinates.y) return false;
    if (min3(trianglePoints[0].z, trianglePoints[1].z, trianglePoints[2].z) > mMaxCoordinates.z) return false;

    if (max3(trianglePoints[0].x, trianglePoints[1].x, trianglePoints[2].x) < mMinCoordinates.x) return false;
    if (max3(trianglePoints[0].y, trianglePoints[1].y, trianglePoints[2].y) < mMinCoordinates.y) return false;
    if (max3(trianglePoints[0].z, trianglePoints[1].z, trianglePoints[2].z) < mMinCoordinates.z) return false;

    return true;
}

// Return true if a point is inside the AABB
inline bool AABB::contains(const Vector3& point) const {

    return (point.x >= mMinCoordinates.x - MACHINE_EPSILON && point.x <= mMaxCoordinates.x + MACHINE_EPSILON &&
            point.y >= mMinCoordinates.y - MACHINE_EPSILON && point.y <= mMaxCoordinates.y + MACHINE_EPSILON &&
            point.z >= mMinCoordinates.z - MACHINE_EPSILON && point.z <= mMaxCoordinates.z + MACHINE_EPSILON);
}

// Assignment operator
inline AABB& AABB::operator=(const AABB& aabb) {
    if (this != &aabb) {
        mMinCoordinates = aabb.mMinCoordinates;
        mMaxCoordinates = aabb.mMaxCoordinates;
    }
    return *this;
}

}

#endif
