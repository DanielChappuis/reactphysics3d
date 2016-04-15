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

#ifndef REACTPHYSICS3D_COLLISION_DETECTION_H
#define REACTPHYSICS3D_COLLISION_DETECTION_H

// Libraries
#include "body/CollisionBody.h"
#include "broadphase/BroadPhaseAlgorithm.h"
#include "engine/OverlappingPair.h"
#include "engine/EventListener.h"
#include "narrowphase/DefaultCollisionDispatch.h"
#include "memory/MemoryAllocator.h"
#include "constraint/ContactPoint.h"
#include <vector>
#include <map>
#include <set>
#include <utility>

/// ReactPhysics3D namespace
namespace reactphysics3d {

// Declarations
class BroadPhaseAlgorithm;
class CollisionWorld;
class CollisionCallback;

// Class TestCollisionBetweenShapesCallback
class TestCollisionBetweenShapesCallback : public NarrowPhaseCallback {

    private:

        CollisionCallback* mCollisionCallback;

    public:

        // Constructor
        TestCollisionBetweenShapesCallback(CollisionCallback* callback)
            : mCollisionCallback(callback) {

        }

        // Called by a narrow-phase collision algorithm when a new contact has been found
        virtual void notifyContact(OverlappingPair* overlappingPair,
                                   const ContactPointInfo& contactInfo);
};

// Class CollisionDetection
/**
 * This class computes the collision detection algorithms. We first
 * perform a broad-phase algorithm to know which pairs of bodies can
 * collide and then we run a narrow-phase algorithm to compute the
 * collision contacts between bodies.
 */
class CollisionDetection : public NarrowPhaseCallback {

    private :

        // -------------------- Attributes -------------------- //

        /// Collision Detection Dispatch configuration
        CollisionDispatch* mCollisionDispatch;

        /// Default collision dispatch configuration
        DefaultCollisionDispatch mDefaultCollisionDispatch;

        /// Collision detection matrix (algorithms to use)
        NarrowPhaseAlgorithm* mCollisionMatrix[NB_COLLISION_SHAPE_TYPES][NB_COLLISION_SHAPE_TYPES];

        /// Reference to the memory allocator
        MemoryAllocator& mMemoryAllocator;

        /// Pointer to the physics world
        CollisionWorld* mWorld;

        /// Broad-phase overlapping pairs
        std::map<overlappingpairid, OverlappingPair*> mOverlappingPairs;

        /// Overlapping pairs in contact (during the current Narrow-phase collision detection)
        std::map<overlappingpairid, OverlappingPair*> mContactOverlappingPairs;

        /// Broad-phase algorithm
        BroadPhaseAlgorithm mBroadPhaseAlgorithm;

        /// Narrow-phase GJK algorithm
        // TODO : Delete this
        GJKAlgorithm mNarrowPhaseGJKAlgorithm;

        /// Set of pair of bodies that cannot collide between each other
        std::set<bodyindexpair> mNoCollisionPairs;

        /// True if some collision shapes have been added previously
        bool mIsCollisionShapesAdded;

        // -------------------- Methods -------------------- //

        /// Private copy-constructor
        CollisionDetection(const CollisionDetection& collisionDetection);

        /// Private assignment operator
        CollisionDetection& operator=(const CollisionDetection& collisionDetection);

        /// Compute the broad-phase collision detection
        void computeBroadPhase();

        /// Compute the narrow-phase collision detection
        void computeNarrowPhase();

        /// Add a contact manifold to the linked list of contact manifolds of the two bodies
        /// involed in the corresponding contact.
        void addContactManifoldToBody(OverlappingPair* pair);

        /// Delete all the contact points in the currently overlapping pairs
        void clearContactPoints();

        /// Fill-in the collision detection matrix
        void fillInCollisionMatrix();

        /// Add all the contact manifold of colliding pairs to their bodies
        void addAllContactManifoldsToBodies();
   
    public :

        // -------------------- Methods -------------------- //

        /// Constructor
        CollisionDetection(CollisionWorld* world, MemoryAllocator& memoryAllocator);

        /// Destructor
        ~CollisionDetection();

        /// Set the collision dispatch configuration
        void setCollisionDispatch(CollisionDispatch* collisionDispatch);

        /// Return the Narrow-phase collision detection algorithm to use between two types of shapes
        NarrowPhaseAlgorithm* getCollisionAlgorithm(CollisionShapeType shape1Type,
                                                    CollisionShapeType shape2Type) const;

        /// Add a proxy collision shape to the collision detection
        void addProxyCollisionShape(ProxyShape* proxyShape, const AABB& aabb);

        /// Remove a proxy collision shape from the collision detection
        void removeProxyCollisionShape(ProxyShape* proxyShape);

        /// Update a proxy collision shape (that has moved for instance)
        void updateProxyCollisionShape(ProxyShape* shape, const AABB& aabb,
                                       const Vector3& displacement = Vector3(0, 0, 0), bool forceReinsert = false);

        /// Add a pair of bodies that cannot collide with each other
        void addNoCollisionPair(CollisionBody* body1, CollisionBody* body2);

        /// Remove a pair of bodies that cannot collide with each other
        void removeNoCollisionPair(CollisionBody* body1, CollisionBody* body2);

        /// Ask for a collision shape to be tested again during broad-phase.
        void askForBroadPhaseCollisionCheck(ProxyShape* shape);

        /// Compute the collision detection
        void computeCollisionDetection();

        /// Compute the collision detection
        void testCollisionBetweenShapes(CollisionCallback* callback,
                                        const std::set<uint>& shapes1,
                                        const std::set<uint>& shapes2);

        /// Report collision between two sets of shapes
        void reportCollisionBetweenShapes(CollisionCallback* callback,
                                          const std::set<uint>& shapes1,
                                          const std::set<uint>& shapes2) ;

        /// Ray casting method
        void raycast(RaycastCallback* raycastCallback, const Ray& ray,
                     unsigned short raycastWithCategoryMaskBits) const;

        /// Test if the AABBs of two bodies overlap
        bool testAABBOverlap(const CollisionBody* body1,
                             const CollisionBody* body2) const;

        /// Test if the AABBs of two proxy shapes overlap
        bool testAABBOverlap(const ProxyShape* shape1,
                             const ProxyShape* shape2) const;

        /// Allow the broadphase to notify the collision detection about an overlapping pair.
        void broadPhaseNotifyOverlappingPair(ProxyShape* shape1, ProxyShape* shape2);

        /// Compute the narrow-phase collision detection
        void computeNarrowPhaseBetweenShapes(CollisionCallback* callback,
                                             const std::set<uint>& shapes1,
                                             const std::set<uint>& shapes2);

        /// Return a pointer to the world
        CollisionWorld* getWorld();

        /// Return the world event listener
        EventListener* getWorldEventListener();

        /// Return a reference to the world memory allocator
        MemoryAllocator& getWorldMemoryAllocator();

        /// Called by a narrow-phase collision algorithm when a new contact has been found
        virtual void notifyContact(OverlappingPair* overlappingPair, const ContactPointInfo& contactInfo);

        /// Create a new contact
        void createContact(OverlappingPair* overlappingPair, const ContactPointInfo& contactInfo);

        // -------------------- Friendship -------------------- //

        friend class DynamicsWorld;
        friend class ConvexMeshShape;
};

// Return the Narrow-phase collision detection algorithm to use between two types of shapes
inline NarrowPhaseAlgorithm* CollisionDetection::getCollisionAlgorithm(CollisionShapeType shape1Type,
                                                                       CollisionShapeType shape2Type) const {
    return mCollisionMatrix[shape1Type][shape2Type];
}

// Set the collision dispatch configuration
inline void CollisionDetection::setCollisionDispatch(CollisionDispatch* collisionDispatch) {
    mCollisionDispatch = collisionDispatch;

    mCollisionDispatch->init(this, &mMemoryAllocator);

    // Fill-in the collision matrix with the new algorithms to use
    fillInCollisionMatrix();
}

// Add a body to the collision detection
inline void CollisionDetection::addProxyCollisionShape(ProxyShape* proxyShape,
                                                       const AABB& aabb) {
    
    // Add the body to the broad-phase
    mBroadPhaseAlgorithm.addProxyCollisionShape(proxyShape, aabb);

    mIsCollisionShapesAdded = true;
}  

// Add a pair of bodies that cannot collide with each other
inline void CollisionDetection::addNoCollisionPair(CollisionBody* body1,
                                                   CollisionBody* body2) {
    mNoCollisionPairs.insert(OverlappingPair::computeBodiesIndexPair(body1, body2));
}

// Remove a pair of bodies that cannot collide with each other
inline void CollisionDetection::removeNoCollisionPair(CollisionBody* body1,
                                                      CollisionBody* body2) {
    mNoCollisionPairs.erase(OverlappingPair::computeBodiesIndexPair(body1, body2));
}

// Ask for a collision shape to be tested again during broad-phase.
/// We simply put the shape in the list of collision shape that have moved in the
/// previous frame so that it is tested for collision again in the broad-phase.
inline void CollisionDetection::askForBroadPhaseCollisionCheck(ProxyShape* shape) {
    mBroadPhaseAlgorithm.addMovedCollisionShape(shape->mBroadPhaseID);
}

// Update a proxy collision shape (that has moved for instance)
inline void CollisionDetection::updateProxyCollisionShape(ProxyShape* shape, const AABB& aabb,
                                                          const Vector3& displacement, bool forceReinsert) {
    mBroadPhaseAlgorithm.updateProxyCollisionShape(shape, aabb, displacement);
}

// Ray casting method
inline void CollisionDetection::raycast(RaycastCallback* raycastCallback,
                                        const Ray& ray,
                                        unsigned short raycastWithCategoryMaskBits) const {

    PROFILE("CollisionDetection::raycast()");

    RaycastTest rayCastTest(raycastCallback);

    // Ask the broad-phase algorithm to call the testRaycastAgainstShape()
    // callback method for each proxy shape hit by the ray in the broad-phase
    mBroadPhaseAlgorithm.raycast(ray, rayCastTest, raycastWithCategoryMaskBits);
}

// Test if the AABBs of two proxy shapes overlap
inline bool CollisionDetection::testAABBOverlap(const ProxyShape* shape1,
                                                const ProxyShape* shape2) const {

    // If one of the shape's body is not active, we return no overlap
    if (!shape1->getBody()->isActive() || !shape2->getBody()->isActive()) {
        return false;
    }

    return mBroadPhaseAlgorithm.testOverlappingShapes(shape1, shape2);
}

// Return a pointer to the world
inline CollisionWorld* CollisionDetection::getWorld() {
    return mWorld;
}

}

#endif
