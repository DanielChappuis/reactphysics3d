/********************************************************************************
* ReactPhysics3D physics library, http://code.google.com/p/reactphysics3d/      *
* Copyright (c) 2010-2013 Daniel Chappuis                                       *
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
#include "../../mathematics/Vector3.h"
#include "../../mathematics/Matrix3x3.h"
#include "AABB.h"
#include "../../memory/MemoryAllocator.h"

/// ReactPhysics3D namespace
namespace reactphysics3d {
    
/// Type of the collision shape
enum CollisionShapeType {BOX, SPHERE, CONE, CYLINDER, CAPSULE, CONVEX_MESH};

// Declarations
class CollisionBody;
class ProxyShape;

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

    public :

        // -------------------- Methods -------------------- //

        /// Constructor
        CollisionShape(CollisionShapeType type, decimal margin);

        /// Destructor
        virtual ~CollisionShape();

        /// Allocate and return a copy of the object
        virtual CollisionShape* clone(void* allocatedMemory) const=0;

        /// Return the type of the collision shapes
        CollisionShapeType getType() const;

        /// Return the number of similar created shapes
        uint getNbSimilarCreatedShapes() const;

        /// Return the current object margin
        decimal getMargin() const;

        /// Return the number of bytes used by the collision shape
        virtual size_t getSizeInBytes() const = 0;

        /// Return the local bounds of the shape in x, y and z directions
        virtual void getLocalBounds(Vector3& min, Vector3& max) const=0;

        /// Return the local inertia tensor of the collision shapes
        virtual void computeLocalInertiaTensor(Matrix3x3& tensor, decimal mass) const=0;

        /// Compute the world-space AABB of the collision shape given a transform
        virtual void computeAABB(AABB& aabb, const Transform& transform) const;

        /// Increment the number of similar allocated collision shapes
        void incrementNbSimilarCreatedShapes();

        /// Decrement the number of similar allocated collision shapes
        void decrementNbSimilarCreatedShapes();

        /// Equality operator between two collision shapes.
        bool operator==(const CollisionShape& otherCollisionShape) const;

        /// Test equality between two collision shapes of the same type (same derived classes).
        virtual bool isEqualTo(const CollisionShape& otherCollisionShape) const=0;

        /// Create a proxy collision shape for the collision shape
        virtual ProxyShape* createProxyShape(MemoryAllocator& allocator, CollisionBody* body,
                                             const Transform& transform, decimal mass)=0;
};


// Class ProxyShape
/**
 * The CollisionShape instances are supposed to be unique for memory optimization. For instance,
 * consider two rigid bodies with the same sphere collision shape. In this situation, we will have
 * a unique instance of SphereShape but we need to differentiate between the two instances during
 * the collision detection. They do not have the same position in the world and they do not
 * belong to the same rigid body. The ProxyShape class is used for that purpose by attaching a
 * rigid body with one of its collision shape. A body can have multiple proxy shapes (one for
 * each collision shape attached to the body).
 */
class ProxyShape {

    private:

        // -------------------- Attributes -------------------- //

        /// Pointer to the parent body
        CollisionBody* mBody;

        /// Local-space to parent body-space transform (does not change over time)
        const Transform mLocalToBodyTransform;

        /// Mass (in kilogramms) of the corresponding collision shape
        decimal mMass;

        /// Pointer to the next proxy shape of the body (linked list)
        ProxyShape* mNext;

        /// Broad-phase ID (node ID in the dynamic AABB tree)
        int mBroadPhaseID;

        // -------------------- Methods -------------------- //

        /// Private copy-constructor
        ProxyShape(const ProxyShape& proxyShape);

        /// Private assignment operator
        ProxyShape& operator=(const ProxyShape& proxyShape);

        /// Return the non-const collision shape
        virtual CollisionShape* getInternalCollisionShape() const=0;

    public:

        // -------------------- Methods -------------------- //

        /// Constructor
        ProxyShape(CollisionBody* body, const Transform& transform, decimal mass);

        /// Destructor
        ~ProxyShape();

        /// Return the collision shape
        virtual const CollisionShape* getCollisionShape() const=0;

        /// Return the number of bytes used by the proxy collision shape
        virtual size_t getSizeInBytes() const=0;

        /// Return the parent body
        CollisionBody* getBody() const;

        /// Return the mass of the collision shape
        decimal getMass() const;

        /// Return the local to parent body transform
        const Transform& getLocalToBodyTransform() const;

        /// Return a local support point in a given direction with the object margin
        virtual Vector3 getLocalSupportPointWithMargin(const Vector3& direction)=0;

        /// Return a local support point in a given direction without the object margin
        virtual Vector3 getLocalSupportPointWithoutMargin(const Vector3& direction)=0;

        /// Return the current object margin
        virtual decimal getMargin() const=0;

        // -------------------- Friendship -------------------- //

        friend class OverlappingPair;
        friend class CollisionBody;
        friend class RigidBody;
        friend class BroadPhaseAlgorithm;
        friend class DynamicAABBTree;
        friend class CollisionDetection;
};

// Return the type of the collision shape
inline CollisionShapeType CollisionShape::getType() const {
    return mType;
}

// Return the number of similar created shapes
inline uint CollisionShape::getNbSimilarCreatedShapes() const {
    return mNbSimilarCreatedShapes;
}

// Return the current object margin
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

// Return the parent body
inline CollisionBody* ProxyShape::getBody() const {
    return mBody;
}

// Return the mass of the collision shape
inline decimal ProxyShape::getMass() const {
    return mMass;
}

// Return the local to parent body transform
inline const Transform& ProxyShape::getLocalToBodyTransform() const {
    return mLocalToBodyTransform;
}

}

#endif
