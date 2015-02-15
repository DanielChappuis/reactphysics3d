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
enum CollisionShapeType {BOX, SPHERE, CONE, CYLINDER, CAPSULE, CONVEX_MESH};

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

        /// Current number of similar created shapes
        uint mNbSimilarCreatedShapes;

        /// Margin used for the GJK collision detection algorithm
        decimal mMargin;
        
        // -------------------- Methods -------------------- //

        /// Private copy-constructor
        CollisionShape(const CollisionShape& shape);

        /// Private assignment operator
        CollisionShape& operator=(const CollisionShape& shape);

        // Return a local support point in a given direction with the object margin
        virtual Vector3 getLocalSupportPointWithMargin(const Vector3& direction,
                                                       void** cachedCollisionData) const=0;

        /// Return a local support point in a given direction without the object margin
        virtual Vector3 getLocalSupportPointWithoutMargin(const Vector3& direction,
                                                          void** cachedCollisionData) const=0;

        /// Return true if a point is inside the collision shape
        virtual bool testPointInside(const Vector3& worldPoint, ProxyShape* proxyShape) const=0;

        /// Raycast method with feedback information
        virtual bool raycast(const Ray& ray, RaycastInfo& raycastInfo, ProxyShape* proxyShape) const=0;

        /// Return the number of similar created shapes
        uint getNbSimilarCreatedShapes() const;

        /// Allocate and return a copy of the object
        virtual CollisionShape* clone(void* allocatedMemory) const=0;

        /// Return the number of bytes used by the collision shape
        virtual size_t getSizeInBytes() const = 0;

        /// Increment the number of similar allocated collision shapes
        void incrementNbSimilarCreatedShapes();

        /// Decrement the number of similar allocated collision shapes
        void decrementNbSimilarCreatedShapes();

    public :

        // -------------------- Methods -------------------- //

        /// Constructor
        CollisionShape(CollisionShapeType type, decimal margin);

        /// Destructor
        virtual ~CollisionShape();

        /// Return the type of the collision shapes
        CollisionShapeType getType() const;

        /// Return the current object margin
        decimal getMargin() const;

        /// Return the local bounds of the shape in x, y and z directions
        virtual void getLocalBounds(Vector3& min, Vector3& max) const=0;

        /// Return the local inertia tensor of the collision shapes
        virtual void computeLocalInertiaTensor(Matrix3x3& tensor, decimal mass) const=0;

        /// Compute the world-space AABB of the collision shape given a transform
        virtual void computeAABB(AABB& aabb, const Transform& transform) const;

        /// Equality operator between two collision shapes.
        bool operator==(const CollisionShape& otherCollisionShape) const;

        /// Test equality between two collision shapes of the same type (same derived classes).
        virtual bool isEqualTo(const CollisionShape& otherCollisionShape) const=0;

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

// Return the number of similar created shapes
inline uint CollisionShape::getNbSimilarCreatedShapes() const {
    return mNbSimilarCreatedShapes;
}

// Return the current collision shape margin
/**
 * @return The margin (in meters) around the collision shape
 */
inline decimal CollisionShape::getMargin() const {
    return mMargin;
}

// Increment the number of similar allocated collision shapes
inline void CollisionShape::incrementNbSimilarCreatedShapes() {
    mNbSimilarCreatedShapes++;
}

// Decrement the number of similar allocated collision shapes
inline void CollisionShape::decrementNbSimilarCreatedShapes() {
    mNbSimilarCreatedShapes--;
}

// Equality operator between two collision shapes.
/// This methods returns true only if the two collision shapes are of the same type and
/// of the same dimensions.
inline bool CollisionShape::operator==(const CollisionShape& otherCollisionShape) const {

    // If the two collisions shapes are not of the same type (same derived classes)
    // we return false
    if (mType != otherCollisionShape.mType) return false;

    assert(typeid(*this) == typeid(otherCollisionShape));

    if (mMargin != otherCollisionShape.mMargin) return false;

    // Check if the two shapes are equal
    return otherCollisionShape.isEqualTo(*this);
}

}

#endif
