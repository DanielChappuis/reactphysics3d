/********************************************************************************
* ReactPhysics3D physics library, http://www.reactphysics3d.com                 *
* Copyright (c) 2010-2024 Daniel Chappuis                                       *
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
#include <reactphysics3d/mathematics/mathematics.h>
#include <reactphysics3d/configuration.h>

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

        /// Inflate (if necessary) to make sure that a given point fit inside it
        void inflateWithPoint(const Vector3& point);

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
        bool contains(const Vector3& point, decimal epsilon = MACHINE_EPSILON) const;

        /// Return true if the AABB of a triangle intersects the AABB
        bool testCollisionTriangleAABB(const Vector3* trianglePoints) const;

        /// Return true if the ray intersects the AABB
        bool testRayIntersect(const Vector3& rayOrigin, const Vector3& rayDirectionInv, decimal rayMaxFraction) const;

        /// Compute the intersection of a ray and the AABB
        bool raycast(const Ray& ray, Vector3& hitPoint) const;

        /// Apply a scale factor to the AABB
        void applyScale(const Vector3& scale);

        /// Create and return an AABB for a triangle
        static AABB createAABBForTriangle(const Vector3* trianglePoints);

        // -------------------- Friendship -------------------- //

        friend class DynamicAABBTree;
};

// Return the center point of the AABB in world coordinates
RP3D_FORCE_INLINE Vector3 AABB::getCenter() const {
    return (mMinCoordinates + mMaxCoordinates) * decimal(0.5);
}

// Return the minimum coordinates of the AABB
RP3D_FORCE_INLINE const Vector3& AABB::getMin() const {
    return mMinCoordinates;
}

// Set the minimum coordinates of the AABB
RP3D_FORCE_INLINE void AABB::setMin(const Vector3& min) {
    mMinCoordinates = min;
}

// Return the maximum coordinates of the AABB
RP3D_FORCE_INLINE const Vector3& AABB::getMax() const {
    return mMaxCoordinates;
}

// Set the maximum coordinates of the AABB
RP3D_FORCE_INLINE void AABB::setMax(const Vector3& max) {
    mMaxCoordinates = max;
}

// Return the size of the AABB in the three dimension x, y and z
RP3D_FORCE_INLINE Vector3 AABB::getExtent() const {
  return  mMaxCoordinates - mMinCoordinates;
}

// Inflate each side of the AABB by a given size
RP3D_FORCE_INLINE void AABB::inflate(decimal dx, decimal dy, decimal dz) {
    mMaxCoordinates += Vector3(dx, dy, dz);
    mMinCoordinates -= Vector3(dx, dy, dz);
}

// Inflate (if necessary) to make sure that a given point fit inside it
RP3D_FORCE_INLINE void AABB::inflateWithPoint(const Vector3& point) {

    // Compute mesh bounds
    if (point.x > mMaxCoordinates.x) mMaxCoordinates.x = point.x;
    if (point.x < mMinCoordinates.x) mMinCoordinates.x = point.x;

    if (point.y > mMaxCoordinates.y) mMaxCoordinates.y = point.y;
    if (point.y < mMinCoordinates.y) mMinCoordinates.y = point.y;

    if (point.z > mMaxCoordinates.z) mMaxCoordinates.z = point.z;
    if (point.z < mMinCoordinates.z) mMinCoordinates.z = point.z;
}

// Return true if the current AABB is overlapping with the AABB in argument.
/// Two AABBs overlap if they overlap in the three x, y and z axis at the same time
RP3D_FORCE_INLINE bool AABB::testCollision(const AABB& aabb) const {
    if (mMaxCoordinates.x < aabb.mMinCoordinates.x ||
        aabb.mMaxCoordinates.x < mMinCoordinates.x) return false;
    if (mMaxCoordinates.y < aabb.mMinCoordinates.y ||
        aabb.mMaxCoordinates.y < mMinCoordinates.y) return false;
    if (mMaxCoordinates.z < aabb.mMinCoordinates.z||
        aabb.mMaxCoordinates.z < mMinCoordinates.z) return false;
    return true;
}

// Return the volume of the AABB
RP3D_FORCE_INLINE decimal AABB::getVolume() const {
    const Vector3 diff = mMaxCoordinates - mMinCoordinates;
    return (diff.x * diff.y * diff.z);
}

// Return true if the AABB of a triangle intersects the AABB
RP3D_FORCE_INLINE bool AABB::testCollisionTriangleAABB(const Vector3* trianglePoints) const {

    if (min3(trianglePoints[0].x, trianglePoints[1].x, trianglePoints[2].x) > mMaxCoordinates.x) return false;
    if (min3(trianglePoints[0].y, trianglePoints[1].y, trianglePoints[2].y) > mMaxCoordinates.y) return false;
    if (min3(trianglePoints[0].z, trianglePoints[1].z, trianglePoints[2].z) > mMaxCoordinates.z) return false;

    if (max3(trianglePoints[0].x, trianglePoints[1].x, trianglePoints[2].x) < mMinCoordinates.x) return false;
    if (max3(trianglePoints[0].y, trianglePoints[1].y, trianglePoints[2].y) < mMinCoordinates.y) return false;
    if (max3(trianglePoints[0].z, trianglePoints[1].z, trianglePoints[2].z) < mMinCoordinates.z) return false;

    return true;
}

// Return true if a point is inside the AABB
RP3D_FORCE_INLINE bool AABB::contains(const Vector3& point, decimal epsilon) const {

    return (point.x >= mMinCoordinates.x - epsilon && point.x <= mMaxCoordinates.x + epsilon &&
            point.y >= mMinCoordinates.y - epsilon && point.y <= mMaxCoordinates.y + epsilon &&
            point.z >= mMinCoordinates.z - epsilon && point.z <= mMaxCoordinates.z + epsilon);
}

// Apply a scale factor to the AABB
RP3D_FORCE_INLINE void AABB::applyScale(const Vector3& scale) {
    mMinCoordinates = mMinCoordinates * scale;
    mMaxCoordinates = mMaxCoordinates * scale;
}

// Merge the AABB in parameter with the current one
RP3D_FORCE_INLINE void AABB::mergeWithAABB(const AABB& aabb) {
    mMinCoordinates.x = std::min(mMinCoordinates.x, aabb.mMinCoordinates.x);
    mMinCoordinates.y = std::min(mMinCoordinates.y, aabb.mMinCoordinates.y);
    mMinCoordinates.z = std::min(mMinCoordinates.z, aabb.mMinCoordinates.z);

    mMaxCoordinates.x = std::max(mMaxCoordinates.x, aabb.mMaxCoordinates.x);
    mMaxCoordinates.y = std::max(mMaxCoordinates.y, aabb.mMaxCoordinates.y);
    mMaxCoordinates.z = std::max(mMaxCoordinates.z, aabb.mMaxCoordinates.z);
}

// Replace the current AABB with a new AABB that is the union of two AABBs in parameters
RP3D_FORCE_INLINE void AABB::mergeTwoAABBs(const AABB& aabb1, const AABB& aabb2) {
    mMinCoordinates.x = std::min(aabb1.mMinCoordinates.x, aabb2.mMinCoordinates.x);
    mMinCoordinates.y = std::min(aabb1.mMinCoordinates.y, aabb2.mMinCoordinates.y);
    mMinCoordinates.z = std::min(aabb1.mMinCoordinates.z, aabb2.mMinCoordinates.z);

    mMaxCoordinates.x = std::max(aabb1.mMaxCoordinates.x, aabb2.mMaxCoordinates.x);
    mMaxCoordinates.y = std::max(aabb1.mMaxCoordinates.y, aabb2.mMaxCoordinates.y);
    mMaxCoordinates.z = std::max(aabb1.mMaxCoordinates.z, aabb2.mMaxCoordinates.z);
}

// Return true if the current AABB contains the AABB given in parameter
RP3D_FORCE_INLINE bool AABB::contains(const AABB& aabb) const {

    bool isInside = true;
    isInside = isInside && mMinCoordinates.x <= aabb.mMinCoordinates.x;
    isInside = isInside && mMinCoordinates.y <= aabb.mMinCoordinates.y;
    isInside = isInside && mMinCoordinates.z <= aabb.mMinCoordinates.z;

    isInside = isInside && mMaxCoordinates.x >= aabb.mMaxCoordinates.x;
    isInside = isInside && mMaxCoordinates.y >= aabb.mMaxCoordinates.y;
    isInside = isInside && mMaxCoordinates.z >= aabb.mMaxCoordinates.z;
    return isInside;
}

// Create and return an AABB for a triangle
RP3D_FORCE_INLINE AABB AABB::createAABBForTriangle(const Vector3* trianglePoints) {

    Vector3 minCoords(trianglePoints[0].x, trianglePoints[0].y, trianglePoints[0].z);
    Vector3 maxCoords(trianglePoints[0].x, trianglePoints[0].y, trianglePoints[0].z);

    if (trianglePoints[1].x < minCoords.x) minCoords.x = trianglePoints[1].x;
    if (trianglePoints[1].y < minCoords.y) minCoords.y = trianglePoints[1].y;
    if (trianglePoints[1].z < minCoords.z) minCoords.z = trianglePoints[1].z;

    if (trianglePoints[2].x < minCoords.x) minCoords.x = trianglePoints[2].x;
    if (trianglePoints[2].y < minCoords.y) minCoords.y = trianglePoints[2].y;
    if (trianglePoints[2].z < minCoords.z) minCoords.z = trianglePoints[2].z;

    if (trianglePoints[1].x > maxCoords.x) maxCoords.x = trianglePoints[1].x;
    if (trianglePoints[1].y > maxCoords.y) maxCoords.y = trianglePoints[1].y;
    if (trianglePoints[1].z > maxCoords.z) maxCoords.z = trianglePoints[1].z;

    if (trianglePoints[2].x > maxCoords.x) maxCoords.x = trianglePoints[2].x;
    if (trianglePoints[2].y > maxCoords.y) maxCoords.y = trianglePoints[2].y;
    if (trianglePoints[2].z > maxCoords.z) maxCoords.z = trianglePoints[2].z;

    return AABB(minCoords, maxCoords);
}

// Return true if the ray intersects the AABB
RP3D_FORCE_INLINE bool AABB::testRayIntersect(const Vector3& rayOrigin, const Vector3& rayDirectionInverse, decimal rayMaxFraction) const {

    // This algorithm relies on the IEE floating point properties (division by zero). If the rayDirection is zero, rayDirectionInverse and
    // therfore t1 and t2 will be +-INFINITY. If the i coordinate of the ray's origin is inside the AABB (mMinCoordinates[i] < rayOrigin[i] < mMaxCordinates[i)), we have
    // t1 = -t2 = +- INFINITY. Since max(n, -INFINITY) = min(n, INFINITY) = n for all n, tMin and tMax will stay unchanged. Secondly, if the i
    // coordinate of the ray's origin is outside the box (rayOrigin[i] < mMinCoordinates[i] or rayOrigin[i] > mMaxCoordinates[i]) we have
    // t1 = t2 = +- INFINITY and therefore either tMin = +INFINITY or tMax = -INFINITY. One of those values will stay until the end and make the
    // method to return false. Unfortunately, if the ray lies exactly on a slab (rayOrigin[i] = mMinCoordinates[i] or rayOrigin[i] = mMaxCoordinates[i]) we
    // have t1 = (mMinCoordinates[i] - rayOrigin[i]) * rayDirectionInverse[i] = 0 * INFINITY = NaN which is a problem for the remaining of the algorithm.
    // This will cause the method to return true when the ray is not intersecting the AABB and therefore cause to traverse more nodes than necessary in
    // the BVH tree. Because this should be rare, it is not really a big issue.
    // Reference: https://tavianator.com/2011/ray_box.html

    decimal t1 = (mMinCoordinates[0] - rayOrigin[0]) * rayDirectionInverse[0];
    decimal t2 = (mMaxCoordinates[0] - rayOrigin[0]) * rayDirectionInverse[0];

    decimal tMin = std::min(t1, t2);
    decimal tMax = std::max(t1, t2);
    tMax = std::min(tMax, rayMaxFraction);

    for (int i = 1; i < 3; i++) {

        t1 = (mMinCoordinates[i] - rayOrigin[i]) * rayDirectionInverse[i];
        t2 = (mMaxCoordinates[i] - rayOrigin[i]) * rayDirectionInverse[i];

        tMin = std::max(tMin, std::min(t1, t2));
        tMax = std::min(tMax, std::max(t1, t2));
    }

    return tMax >= std::max(tMin, decimal(0.0));
}

// Compute the intersection of a ray and the AABB
RP3D_FORCE_INLINE bool AABB::raycast(const Ray& ray, Vector3& hitPoint) const {

    decimal tMin = decimal(0.0);
    decimal tMax = DECIMAL_LARGEST;

    const decimal epsilon = decimal(0.00001);

    const Vector3 rayDirection = ray.point2 - ray.point1;

    // For all three slabs
    for (int i=0; i < 3; i++) {

        // If the ray is parallel to the slab
        if (std::abs(rayDirection[i]) < epsilon) {

            // If origin of the ray is not inside the slab, no hit
            if (ray.point1[i] < mMinCoordinates[i] || ray.point1[i] > mMaxCoordinates[i]) return false;
        }
        else {

            decimal rayDirectionInverse = decimal(1.0) / rayDirection[i];
            decimal t1 = (mMinCoordinates[i] - ray.point1[i]) * rayDirectionInverse;
            decimal t2 = (mMaxCoordinates[i] - ray.point1[i]) * rayDirectionInverse;

            if (t1 > t2) {

                // Swap t1 and t2
                decimal tTemp = t2;
                t2 = t1;
                t1 = tTemp;
            }
            
            tMin = std::max(tMin, t1);
            tMax = std::min(tMax, t2);
            
            // Exit with no collision 
            if (tMin > tMax) return false;
        }
    }

    // Compute the hit point
    hitPoint = ray.point1 + tMin * rayDirection;

    return true;
}

}

#endif
