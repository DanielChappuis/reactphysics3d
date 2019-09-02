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

#ifndef REACTPHYSICS3D_COLLISION_WORLD_H
#define REACTPHYSICS3D_COLLISION_WORLD_H

// Libraries
#include "mathematics/mathematics.h"
#include "containers/List.h"
#include "systems/CollisionDetectionSystem.h"
#include "constraint/Joint.h"
#include "memory/MemoryManager.h"
#include "engine/EntityManager.h"
#include "components/CollisionBodyComponents.h"
#include "components/RigidBodyComponents.h"
#include "components/TransformComponents.h"
#include "components/ProxyShapeComponents.h"
#include "components/JointComponents.h"
#include "components/BallAndSocketJointComponents.h"
#include "collision/CollisionCallback.h"
#include "collision/OverlapCallback.h"

/// Namespace reactphysics3d
namespace reactphysics3d {

// Declarations
class Profiler;
class Logger;
class EventListener;
class Joint;
class ContactPoint;
class OverlappingPair;
class CollisionBody;
struct RaycastInfo;
class CollisionCallback;
class OverlapCallback;

// Class CollisionWorld
/**
 * This class represent a world where it is possible to move bodies
 * by hand and to test collision between each other. In this kind of
 * world, the bodies movement is not computed using the laws of physics.
 */
class CollisionWorld {

    protected :

        // -------------------- Attributes -------------------- //

        /// Memory manager
        MemoryManager mMemoryManager;

        /// Configuration of the physics world
        WorldSettings mConfig;

        /// Entity Manager for the ECS
        EntityManager mEntityManager;

        /// Collision Body Components
        CollisionBodyComponents mCollisionBodyComponents;

        /// Rigid Body Components
        RigidBodyComponents mRigidBodyComponents;

        /// Transform Components
        TransformComponents mTransformComponents;

        /// Proxy-Shapes Components
        ProxyShapeComponents mProxyShapesComponents;

        /// Joint Components
        JointComponents mJointsComponents;

        /// Ball And Socket joints Components
        BallAndSocketJointComponents mBallAndSocketJointsComponents;

        /// Reference to the collision detection
        CollisionDetectionSystem mCollisionDetection;

        /// All the bodies (rigid and soft) of the world
        List<CollisionBody*> mBodies;

        /// Pointer to an event listener object
        EventListener* mEventListener;

        /// Name of the collision world
        std::string mName;

#ifdef IS_PROFILING_ACTIVE

		/// Real-time hierarchical profiler
        Profiler* mProfiler;
#endif

#ifdef IS_LOGGING_ACTIVE

        /// Logger
        Logger* mLogger;
#endif

        /// True if the profiler has been created by the user
        bool mIsProfilerCreatedByUser;

        /// True if the logger has been created by the user
        bool mIsLoggerCreatedByUser;

        /// Total number of worlds
        static uint mNbWorlds;

        // -------------------- Methods -------------------- //

        /// Notify the world if a body is disabled (slepping or inactive) or not
        void setBodyDisabled(Entity entity, bool isDisabled);

        /// Notify the world whether a joint is disabled or not
        void setJointDisabled(Entity jointEntity, bool isDisabled);

    public :

        // -------------------- Methods -------------------- //

        /// Constructor
        CollisionWorld(const WorldSettings& worldSettings = WorldSettings(), Logger* logger = nullptr,
                       Profiler* profiler = nullptr);

        /// Destructor
        virtual ~CollisionWorld();

        /// Deleted copy-constructor
        CollisionWorld(const CollisionWorld& world) = delete;

        /// Deleted assignment operator
        CollisionWorld& operator=(const CollisionWorld& world) = delete;

        /// Create a collision body
        CollisionBody* createCollisionBody(const Transform& transform);

        /// Destroy a collision body
        void destroyCollisionBody(CollisionBody* collisionBody);

        /// Get the collision dispatch configuration
        CollisionDispatch& getCollisionDispatch();

        /// Ray cast method
        void raycast(const Ray& ray, RaycastCallback* raycastCallback, unsigned short raycastWithCategoryMaskBits = 0xFFFF) const;

        /// Return true if two bodies overlap (collide)
        bool testOverlap(CollisionBody* body1, CollisionBody* body2);

        /// Report all the bodies that overlap (collide) with the body in parameter
        void testOverlap(CollisionBody* body, OverlapCallback& overlapCallback);

        /// Report all the bodies that overlap (collide) in the world
        void testOverlap(OverlapCallback& overlapCallback);

        /// Test collision and report contacts between two bodies.
        void testCollision(CollisionBody* body1, CollisionBody* body2, CollisionCallback& callback);

        /// Test collision and report all the contacts involving the body in parameter
        void testCollision(CollisionBody* body, CollisionCallback& callback);

        /// Test collision and report contacts between each colliding bodies in the world
        void testCollision(CollisionCallback& callback);

#ifdef IS_PROFILING_ACTIVE

        /// Return a reference to the profiler
        Profiler* getProfiler();

#endif

#ifdef IS_LOGGING_ACTIVE

        /// Return a reference to the logger
        Logger* getLogger();

#endif

        /// Return the current world-space AABB of given proxy shape
        AABB getWorldAABB(const ProxyShape* proxyShape) const;

        /// Return the name of the world
        const std::string& getName() const;

        // -------------------- Friendship -------------------- //

        friend class CollisionDetectionSystem;
        friend class CollisionBody;
        friend class RigidBody;
        friend class ProxyShape;
        friend class ConvexMeshShape;
        friend class CollisionCallback::ContactPair;
        friend class OverlapCallback::OverlapPair;
};

// Set the collision dispatch configuration
/// This can be used to replace default collision detection algorithms by your
/// custom algorithm for instance.
/**
 * @param CollisionDispatch Pointer to a collision dispatch object describing
 * which collision detection algorithm to use for two given collision shapes
 */
inline CollisionDispatch& CollisionWorld::getCollisionDispatch() {
    return mCollisionDetection.getCollisionDispatch();
}

// Ray cast method
/**
 * @param ray Ray to use for raycasting
 * @param raycastCallback Pointer to the class with the callback method
 * @param raycastWithCategoryMaskBits Bits mask corresponding to the category of
 *                                    bodies to be raycasted
 */
inline void CollisionWorld::raycast(const Ray& ray,
                                    RaycastCallback* raycastCallback,
                                    unsigned short raycastWithCategoryMaskBits) const {
    mCollisionDetection.raycast(raycastCallback, ray, raycastWithCategoryMaskBits);
}

// Test collision and report contacts between two bodies.
/// Use this method if you only want to get all the contacts between two bodies.
/// All the contacts will be reported using the callback object in paramater.
/// If you are not interested in the contacts but you only want to know if the bodies collide,
/// you can use the testOverlap() method instead.
/**
 * @param body1 Pointer to the first body to test
 * @param body2 Pointer to the second body to test
 * @param callback Pointer to the object with the callback method
 */
inline void CollisionWorld::testCollision(CollisionBody* body1, CollisionBody* body2, CollisionCallback& callback) {
    mCollisionDetection.testCollision(body1, body2, callback);
}

// Test collision and report all the contacts involving the body in parameter
/// Use this method if you only want to get all the contacts involving a given body.
/// All the contacts will be reported using the callback object in paramater.
/// If you are not interested in the contacts but you only want to know if the bodies collide,
/// you can use the testOverlap() method instead.
/**
 * @param body Pointer to the body against which we need to test collision
 * @param callback Pointer to the object with the callback method to report contacts
 */
inline void CollisionWorld::testCollision(CollisionBody* body, CollisionCallback& callback) {
    mCollisionDetection.testCollision(body, callback);
}

// Test collision and report contacts between each colliding bodies in the world
/// Use this method if you want to get all the contacts between colliding bodies in the world.
/// All the contacts will be reported using the callback object in paramater.
/// If you are not interested in the contacts but you only want to know if the bodies collide,
/// you can use the testOverlap() method instead.
/**
 * @param callback Pointer to the object with the callback method to report contacts
 */
inline void CollisionWorld::testCollision(CollisionCallback& callback) {
    mCollisionDetection.testCollision(callback);
}

// Report all the bodies that overlap (collide) with the body in parameter
/// Use this method if you are not interested in contacts but if you simply want to know
/// which bodies overlap with the body in parameter. If you want to get the contacts, you need to use the
/// testCollision() method instead.
/**
 * @param body Pointer to the collision body to test overlap with
 * @param overlapCallback Pointer to the callback class to report overlap
 */
inline void CollisionWorld::testOverlap(CollisionBody* body, OverlapCallback& overlapCallback) {
    mCollisionDetection.testOverlap(body, overlapCallback);
}

// Report all the bodies that overlap (collide) in the world
/// Use this method if you are not interested in contacts but if you simply want to know
/// which bodies overlap. If you want to get the contacts, you need to use the
/// testCollision() method instead.
inline void CollisionWorld::testOverlap(OverlapCallback& overlapCallback) {
    mCollisionDetection.testOverlap(overlapCallback);
}

// Return the name of the world
/**
 * @return Name of the world
 */
inline const std::string& CollisionWorld::getName() const {
    return mName;
}

#ifdef IS_PROFILING_ACTIVE

// Return a pointer to the profiler
/**
 * @return A pointer to the profiler
 */
inline Profiler* CollisionWorld::getProfiler() {
    return mProfiler;
}

#endif

#ifdef IS_LOGGING_ACTIVE

// Return a pointer to the logger
/**
 * @return A pointer to the logger
 */
inline Logger* CollisionWorld::getLogger() {
    return mLogger;
}

#endif

}

#endif
