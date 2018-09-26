/********************************************************************************
* ReactPhysics3D physics library, http://www.reactphysics3d.com                 *
* Copyright (c) 2010-2018 Daniel Chappuis                                       *
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

#ifndef REACTPHYSICS3D_COLLISION_BODY_H
#define REACTPHYSICS3D_COLLISION_BODY_H

// Libraries
#include <cassert>
#include "Body.h"
#include "collision/shapes/AABB.h"
#include "mathematics/Transform.h"
#include "configuration.h"

/// Namespace reactphysics3d
namespace reactphysics3d {

// Declarations
struct ContactManifoldListElement;
class ProxyShape;
class CollisionWorld;
class CollisionShape;
struct RaycastInfo;
class DefaultPoolAllocator;
class Profiler;

/// Enumeration for the type of a body
/// STATIC : A static body has infinite mass, zero velocity but the position can be
///          changed manually. A static body does not collide with other static or kinematic bodies.
/// KINEMATIC : A kinematic body has infinite mass, the velocity can be changed manually and its
///             position is computed by the physics engine. A kinematic body does not collide with
///             other static or kinematic bodies.
/// DYNAMIC : A dynamic body has non-zero mass, non-zero velocity determined by forces and its
///           position is determined by the physics engine. A dynamic body can collide with other
///           dynamic, static or kinematic bodies.
enum class BodyType {STATIC, KINEMATIC, DYNAMIC};

// Class CollisionBody
/**
 * This class represents a body that is able to collide with others
 * bodies. This class inherits from the Body class.
 */
class CollisionBody : public Body {

    protected :

        // -------------------- Attributes -------------------- //

        /// Type of body (static, kinematic or dynamic)
        BodyType mType;

        /// Position and orientation of the body
        Transform mTransform;

        /// First element of the linked list of proxy collision shapes of this body
        ProxyShape* mProxyCollisionShapes;

        /// Number of collision shapes
        uint mNbCollisionShapes;

        /// First element of the linked list of contact manifolds involving this body
        ContactManifoldListElement* mContactManifoldsList;

        /// Reference to the world the body belongs to
        CollisionWorld& mWorld;

#ifdef IS_PROFILING_ACTIVE

		/// Pointer to the profiler
		Profiler* mProfiler;

#endif

        // -------------------- Methods -------------------- //

        /// Reset the contact manifold lists
        void resetContactManifoldsList();

        /// Remove all the collision shapes
        void removeAllCollisionShapes();

        /// Update the broad-phase state for this body (because it has moved for instance)
        virtual void updateBroadPhaseState() const;

        /// Update the broad-phase state of a proxy collision shape of the body
        void updateProxyShapeInBroadPhase(ProxyShape* proxyShape, bool forceReinsert = false) const;

        /// Ask the broad-phase to test again the collision shapes of the body for collision
        /// (as if the body has moved).
        void askForBroadPhaseCollisionCheck() const;

        /// Reset the mIsAlreadyInIsland variable of the body and contact manifolds
        int resetIsAlreadyInIslandAndCountManifolds();

    public :

        // -------------------- Methods -------------------- //

        /// Constructor
        CollisionBody(const Transform& transform, CollisionWorld& world, bodyindex id);

        /// Destructor
        virtual ~CollisionBody() override;

        /// Deleted copy-constructor
        CollisionBody(const CollisionBody& body) = delete;

        /// Deleted assignment operator
        CollisionBody& operator=(const CollisionBody& body) = delete;

        /// Return the type of the body
        BodyType getType() const;

        /// Set the type of the body
        void setType(BodyType type);

        /// Set whether or not the body is active
        virtual void setIsActive(bool isActive) override;

        /// Return the current position and orientation
        const Transform& getTransform() const;

        /// Set the current position and orientation
        virtual void setTransform(const Transform& transform);

        /// Add a collision shape to the body.
        virtual ProxyShape* addCollisionShape(CollisionShape* collisionShape,
                                              const Transform& transform);

        /// Remove a collision shape from the body
        virtual void removeCollisionShape(const ProxyShape* proxyShape);

        /// Return the first element of the linked list of contact manifolds involving this body
        const ContactManifoldListElement* getContactManifoldsList() const;

        /// Return true if a point is inside the collision body
        bool testPointInside(const Vector3& worldPoint) const;

        /// Raycast method with feedback information
        bool raycast(const Ray& ray, RaycastInfo& raycastInfo);

        /// Test if the collision body overlaps with a given AABB
        bool testAABBOverlap(const AABB& worldAABB) const;

        /// Compute and return the AABB of the body by merging all proxy shapes AABBs
        AABB getAABB() const;

        /// Return the linked list of proxy shapes of that body
        ProxyShape* getProxyShapesList();

        /// Return the linked list of proxy shapes of that body
        const ProxyShape* getProxyShapesList() const;

        /// Return the world-space coordinates of a point given the local-space coordinates of the body
        Vector3 getWorldPoint(const Vector3& localPoint) const;

        /// Return the world-space vector of a vector given in local-space coordinates of the body
        Vector3 getWorldVector(const Vector3& localVector) const;

        /// Return the body local-space coordinates of a point given in the world-space coordinates
        Vector3 getLocalPoint(const Vector3& worldPoint) const;

        /// Return the body local-space coordinates of a vector given in the world-space coordinates
        Vector3 getLocalVector(const Vector3& worldVector) const;

#ifdef IS_PROFILING_ACTIVE

		/// Set the profiler
		virtual void setProfiler(Profiler* profiler);

#endif

        // -------------------- Friendship -------------------- //

        friend class CollisionWorld;
        friend class DynamicsWorld;
        friend class CollisionDetection;
        friend class BroadPhaseAlgorithm;
        friend class ConvexMeshShape;
        friend class ProxyShape;
};

// Return the type of the body
/**
 * @return the type of the body (STATIC, KINEMATIC, DYNAMIC)
 */
inline BodyType CollisionBody::getType() const {
    return mType;
}

// Return the current position and orientation
/**
 * @return The current transformation of the body that transforms the local-space
 *         of the body into world-space
 */
inline const Transform& CollisionBody::getTransform() const {
    return mTransform;
}

// Return the first element of the linked list of contact manifolds involving this body
/**
 * @return A pointer to the first element of the linked-list with the contact
 *         manifolds of this body
 */
inline const ContactManifoldListElement* CollisionBody::getContactManifoldsList() const {
    return mContactManifoldsList;
}

// Return the linked list of proxy shapes of that body
/**
* @return The pointer of the first proxy shape of the linked-list of all the
*         proxy shapes of the body
*/
inline ProxyShape* CollisionBody::getProxyShapesList() {
    return mProxyCollisionShapes;
}

// Return the linked list of proxy shapes of that body
/**
* @return The pointer of the first proxy shape of the linked-list of all the
*         proxy shapes of the body
*/
inline const ProxyShape* CollisionBody::getProxyShapesList() const {
    return mProxyCollisionShapes;
}

// Return the world-space coordinates of a point given the local-space coordinates of the body
/**
* @param localPoint A point in the local-space coordinates of the body
* @return The point in world-space coordinates
*/
inline Vector3 CollisionBody::getWorldPoint(const Vector3& localPoint) const {
    return mTransform * localPoint;
}

// Return the world-space vector of a vector given in local-space coordinates of the body
/**
* @param localVector A vector in the local-space coordinates of the body
* @return The vector in world-space coordinates
*/
inline Vector3 CollisionBody::getWorldVector(const Vector3& localVector) const {
    return mTransform.getOrientation() * localVector;
}

// Return the body local-space coordinates of a point given in the world-space coordinates
/**
* @param worldPoint A point in world-space coordinates
* @return The point in the local-space coordinates of the body
*/
inline Vector3 CollisionBody::getLocalPoint(const Vector3& worldPoint) const {
    return mTransform.getInverse() * worldPoint;
}

// Return the body local-space coordinates of a vector given in the world-space coordinates
/**
* @param worldVector A vector in world-space coordinates
* @return The vector in the local-space coordinates of the body
*/
inline Vector3 CollisionBody::getLocalVector(const Vector3& worldVector) const {
    return mTransform.getOrientation().getInverse() * worldVector;
}

/// Test if the collision body overlaps with a given AABB
/**
* @param worldAABB The AABB (in world-space coordinates) that will be used to test overlap
* @return True if the given AABB overlaps with the AABB of the collision body
*/
inline bool CollisionBody::testAABBOverlap(const AABB& worldAABB) const {
    return worldAABB.testCollision(getAABB());
}

#ifdef IS_PROFILING_ACTIVE

// Set the profiler
inline void CollisionBody::setProfiler(Profiler* profiler) {
	mProfiler = profiler;
}

#endif

}

#endif
