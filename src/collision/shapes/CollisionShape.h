/********************************************************************************
* ReactPhysics3D physics library, http://www.reactphysics3d.com                 *
* Copyright (c) 2010-2016 Daniel Chappuis                                       *
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

#ifndef REACTPHYSICS3D_COLLISION_SHAPE_H
#define REACTPHYSICS3D_COLLISION_SHAPE_H

// Libraries
#include <cassert>
#include <typeinfo>
#include "mathematics/Vector3.h"
#include "mathematics/Matrix3x3.h"
#include "mathematics/Ray.h"
#include "AABB.h"
#include "collision/RaycastInfo.h"
#include "memory/MemoryAllocator.h"

/// ReactPhysics3D namespace
namespace reactphysics3d {
    
/// Type of the collision shape
enum CollisionShapeType {TRIANGLE, BOX, SPHERE, CONE, CYLINDER,
                         CAPSULE, CONVEX_MESH, CONCAVE_MESH, HEIGHTFIELD};
const int NB_COLLISION_SHAPE_TYPES = 9;

// Declarations
class ProxyShape;
class CollisionBody;

// Class CollisionShape
/**
 * This abstract class represents the collision shape associated with a
 * body that is used during the narrow-phase collision detection.
 */
class CollisionShape {
        
    protected :

        // -------------------- Attributes -------------------- //

        /// Type of the collision shape
        CollisionShapeType mType;

        /// Scaling vector of the collision shape
        Vector3 mScaling;
        
        // -------------------- Methods -------------------- //

        /// Private copy-constructor
        CollisionShape(const CollisionShape& shape);

        /// Private assignment operator
        CollisionShape& operator=(const CollisionShape& shape);

        /// Return true if a point is inside the collision shape
        virtual bool testPointInside(const Vector3& worldPoint, ProxyShape* proxyShape) const=0;

        /// Raycast method with feedback information
        virtual bool raycast(const Ray& ray, RaycastInfo& raycastInfo, ProxyShape* proxyShape) const=0;

        /// Return the number of bytes used by the collision shape
        virtual size_t getSizeInBytes() const = 0;

    public :

        // -------------------- Methods -------------------- //

        /// Constructor
        CollisionShape(CollisionShapeType type);

        /// Destructor
        virtual ~CollisionShape();

        /// Return the type of the collision shapes
        CollisionShapeType getType() const;

        /// Return true if the collision shape is convex, false if it is concave
        virtual bool isConvex() const=0;

        /// Return the local bounds of the shape in x, y and z directions
        virtual void getLocalBounds(Vector3& min, Vector3& max) const=0;

        /// Return the scaling vector of the collision shape
        Vector3 getScaling() const;

        /// Set the local scaling vector of the collision shape
        virtual void setLocalScaling(const Vector3& scaling);

        /// Return the local inertia tensor of the collision shapes
        virtual void computeLocalInertiaTensor(Matrix3x3& tensor, decimal mass) const=0;

        /// Compute the world-space AABB of the collision shape given a transform
        virtual void computeAABB(AABB& aabb, const Transform& transform) const;

        /// Return true if the collision shape type is a convex shape
        static bool isConvex(CollisionShapeType shapeType);

        /// Return the maximum number of contact manifolds in an overlapping pair given two shape types
        static int computeNbMaxContactManifolds(CollisionShapeType shapeType1,
                                                CollisionShapeType shapeType2);

        // -------------------- Friendship -------------------- //

        friend class ProxyShape;
        friend class CollisionWorld;
};

// Return the type of the collision shape
/**
 * @return The type of the collision shape (box, sphere, cylinder, ...)
 */
inline CollisionShapeType CollisionShape::getType() const {
    return mType;
}

// Return true if the collision shape type is a convex shape
inline bool CollisionShape::isConvex(CollisionShapeType shapeType) {
    return shapeType != CONCAVE_MESH && shapeType != HEIGHTFIELD;
}

// Return the scaling vector of the collision shape
inline Vector3 CollisionShape::getScaling() const {
    return mScaling;
}

// Set the scaling vector of the collision shape
inline void CollisionShape::setLocalScaling(const Vector3& scaling) {
    mScaling = scaling;
}

// Return the maximum number of contact manifolds allowed in an overlapping
// pair wit the given two collision shape types
inline int CollisionShape::computeNbMaxContactManifolds(CollisionShapeType shapeType1,
                                                        CollisionShapeType shapeType2) {
    // If both shapes are convex
    if (isConvex(shapeType1) && isConvex(shapeType2)) {
        return NB_MAX_CONTACT_MANIFOLDS_CONVEX_SHAPE;
    }   // If there is at least one concave shape
    else {
        return NB_MAX_CONTACT_MANIFOLDS_CONCAVE_SHAPE;
    }
}

}

#endif
