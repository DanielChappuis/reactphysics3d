/********************************************************************************
* ReactPhysics3D physics library, http://www.reactphysics3d.com                 *
* Copyright (c) 2010-2015 Daniel Chappuis                                       *
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

#ifndef REACTPHYSICS3D_CONCAVE_MESH_SHAPE_H
#define REACTPHYSICS3D_CONCAVE_MESH_SHAPE_H

// Libraries
#include "ConcaveShape.h"

namespace reactphysics3d {

// TODO : Implement raycasting with this collision shape

// Class ConcaveMeshShape
/**
 * This class represents a concave mesh shape. Note that collision detection
 * with a concave mesh shape can be very expensive. You should use only use
 * this shape for a static mesh.
 */
class ConcaveMeshShape : public ConcaveShape {

    protected:

        // -------------------- Methods -------------------- //

        /// Private copy-constructor
        ConcaveMeshShape(const ConcaveMeshShape& shape);

        /// Private assignment operator
        ConcaveMeshShape& operator=(const ConcaveMeshShape& shape);

        /// Return a local support point in a given direction with the object margin
        virtual Vector3 getLocalSupportPointWithMargin(const Vector3& direction,
                                                       void** cachedCollisionData) const;

        /// Return a local support point in a given direction without the object margin
        virtual Vector3 getLocalSupportPointWithoutMargin(const Vector3& direction,
                                                          void** cachedCollisionData) const;

        /// Return true if a point is inside the collision shape
        virtual bool testPointInside(const Vector3& localPoint, ProxyShape* proxyShape) const;

        /// Raycast method with feedback information
        virtual bool raycast(const Ray& ray, RaycastInfo& raycastInfo, ProxyShape* proxyShape) const;

        /// Allocate and return a copy of the object
        virtual ConcaveMeshShape* clone(void* allocatedMemory) const;

        /// Return the number of bytes used by the collision shape
        virtual size_t getSizeInBytes() const;

    public:

        /// Constructor
        ConcaveMeshShape(TriangleMesh* triangleMesh);

        /// Destructor
        ~ConcaveMeshShape();

        /// Return true if the collision shape is convex, false if it is concave
        virtual bool isConvex() const;

        /// Return the local bounds of the shape in x, y and z directions.
        virtual void getLocalBounds(Vector3& min, Vector3& max) const;

        /// Return the local inertia tensor of the collision shape
        virtual void computeLocalInertiaTensor(Matrix3x3& tensor, decimal mass) const;

        /// Update the AABB of a body using its collision shape
        virtual void computeAABB(AABB& aabb, const Transform& transform);

        /// Test equality between two sphere shapes
        virtual bool isEqualTo(const CollisionShape& otherCollisionShape) const;
};

// Return true if the collision shape is convex, false if it is concave
virtual bool isConvex() const {
    return false;
}

// Allocate and return a copy of the object
inline ConcaveMeshShape* ConcaveMeshShape::clone(void* allocatedMemory) const {
    return new (allocatedMemory) ConcaveMeshShape(*this);
}

// Return the number of bytes used by the collision shape
inline size_t ConcaveMeshShape::getSizeInBytes() const {
    return sizeof(ConcaveMeshShape);
}

// Return a local support point in a given direction with the object margin
inline Vector3 ConcaveMeshShape::getLocalSupportPointWithMargin(const Vector3& direction,
                                                           void** cachedCollisionData) const {

    // TODO : Implement this
    return Vector3(0, 0, 0);
}

// Return a local support point in a given direction without the object margin
inline Vector3 ConcaveMeshShape::getLocalSupportPointWithoutMargin(const Vector3& direction,
                                                              void** cachedCollisionData) const {
    // TODO : Implement this
    return Vector3(0.0, 0.0, 0.0);
}

// Return the local bounds of the shape in x, y and z directions.
// This method is used to compute the AABB of the box
/**
 * @param min The minimum bounds of the shape in local-space coordinates
 * @param max The maximum bounds of the shape in local-space coordinates
 */
inline void ConcaveMeshShape::getLocalBounds(Vector3& min, Vector3& max) const {

    // TODO : Implement this
}

// Return the local inertia tensor of the sphere
/**
 * @param[out] tensor The 3x3 inertia tensor matrix of the shape in local-space
 *                    coordinates
 * @param mass Mass to use to compute the inertia tensor of the collision shape
 */
inline void ConcaveMeshShape::computeLocalInertiaTensor(Matrix3x3& tensor, decimal mass) const {

    // TODO : Implement this
    decimal diag = decimal(0.4) * mass * mRadius * mRadius;
    tensor.setAllValues(diag, 0.0, 0.0,
                        0.0, diag, 0.0,
                        0.0, 0.0, diag);
}

// Update the AABB of a body using its collision shape
/**
 * @param[out] aabb The axis-aligned bounding box (AABB) of the collision shape
 *                  computed in world-space coordinates
 * @param transform Transform used to compute the AABB of the collision shape
 */
inline void ConcaveMeshShape::computeAABB(AABB& aabb, const Transform& transform) {

    // TODO : Implement this
}

// Test equality between two sphere shapes
inline bool ConcaveMeshShape::isEqualTo(const CollisionShape& otherCollisionShape) const {
    const ConcaveMeshShape& otherShape = dynamic_cast<const ConcaveMeshShape&>(otherCollisionShape);

    // TODO : Implement this

    return false;
}

// Return true if a point is inside the collision shape
inline bool ConcaveMeshShape::testPointInside(const Vector3& localPoint, ProxyShape* proxyShape) const {

    // TODO : Implement this
    return false;
}

}
#endif

