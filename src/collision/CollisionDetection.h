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

#ifndef REACTPHYSICS3D_COLLISION_DETECTION_H
#define REACTPHYSICS3D_COLLISION_DETECTION_H

// Libraries
#include "../body/CollisionBody.h"
#include "broadphase/BroadPhaseAlgorithm.h"
#include "../engine/OverlappingPair.h"
#include "narrowphase/GJK/GJKAlgorithm.h"
#include "narrowphase/SphereVsSphereAlgorithm.h"
#include "../memory/MemoryAllocator.h"
#include "../constraint/ContactPoint.h"
#include <vector>
#include <map>
#include <set>
#include <utility>

/// ReactPhysics3D namespace
namespace reactphysics3d {

// Declarations
class BroadPhaseAlgorithm;
class CollisionWorld;

// Class CollisionDetection
/**
 * This class computes the collision detection algorithms. We first
 * perform a broad-phase algorithm to know which pairs of bodies can
 * collide and then we run a narrow-phase algorithm to compute the
 * collision contacts between bodies.
 */
class CollisionDetection {

    private :

        // -------------------- Attributes -------------------- //

        /// Pointer to the physics world
        CollisionWorld* mWorld;

        /// Broad-phase overlapping pairs
        std::map<overlappingpairid, OverlappingPair*> mOverlappingPairs;

        /// Broad-phase algorithm
        BroadPhaseAlgorithm mBroadPhaseAlgorithm;

        /// Narrow-phase GJK algorithm
        GJKAlgorithm mNarrowPhaseGJKAlgorithm;

        /// Narrow-phase Sphere vs Sphere algorithm
        SphereVsSphereAlgorithm mNarrowPhaseSphereVsSphereAlgorithm;

        /// Set of pair of bodies that cannot collide between each other
        std::set<bodyindexpair> mNoCollisionPairs;

        /// True if some collision shapes have been added previously
        bool mIsCollisionShapesAdded;

        /// All the contact constraints
        // TODO : Remove this variable (we will use the ones in the island now)
        std::vector<ContactManifold*> mContactManifolds;

        // -------------------- Methods -------------------- //

        /// Private copy-constructor
        CollisionDetection(const CollisionDetection& collisionDetection);

        /// Private assignment operator
        CollisionDetection& operator=(const CollisionDetection& collisionDetection);

        /// Compute the broad-phase collision detection
        void computeBroadPhase();

        /// Compute the narrow-phase collision detection
        void computeNarrowPhase();

        /// Select the narrow phase algorithm to use given two collision shapes
        NarrowPhaseAlgorithm& SelectNarrowPhaseAlgorithm(const CollisionShape* collisionShape1,
                                                         const CollisionShape* collisionShape2);

        /// Create a new contact
        void createContact(OverlappingPair* overlappingPair, const ContactPointInfo* contactInfo);

        /// Add a contact manifold to the linked list of contact manifolds of the two bodies
        /// involed in the corresponding contact.
        void addContactManifoldToBody(ContactManifold* contactManifold,
                                      CollisionBody *body1, CollisionBody *body2);
   
    public :

        // -------------------- Methods -------------------- //

        /// Constructor
        CollisionDetection(CollisionWorld* world, MemoryAllocator& memoryAllocator);

        /// Destructor
        ~CollisionDetection();

        /// Add a proxy collision shape to the collision detection
        void addProxyCollisionShape(ProxyShape* proxyShape);

        /// Remove a proxy collision shape from the collision detection
        void removeProxyCollisionShape(ProxyShape* proxyShape);

        /// Update a proxy collision shape (that has moved for instance)
        void updateProxyCollisionShape(ProxyShape* shape, const AABB& aabb);

        /// Add a pair of bodies that cannot collide with each other
        void addNoCollisionPair(CollisionBody* body1, CollisionBody* body2);

        /// Remove a pair of bodies that cannot collide with each other
        void removeNoCollisionPair(CollisionBody* body1, CollisionBody* body2);

        /// Compute the collision detection
        void computeCollisionDetection();

        /// Allow the broadphase to notify the collision detection about an overlapping pair.
        void broadPhaseNotifyOverlappingPair(ProxyShape* shape1, ProxyShape* shape2);

        // -------------------- Friendship -------------------- //

        // TODO : REMOVE THIS
        friend class DynamicsWorld;
};

// Select the narrow-phase collision algorithm to use given two collision shapes
inline NarrowPhaseAlgorithm& CollisionDetection::SelectNarrowPhaseAlgorithm(
                             const CollisionShape* collisionShape1,
                             const CollisionShape* collisionShape2) {
    
    // Sphere vs Sphere algorithm
    if (collisionShape1->getType() == SPHERE && collisionShape2->getType() == SPHERE) {
        return mNarrowPhaseSphereVsSphereAlgorithm;
    }
    else {   // GJK algorithm
        return mNarrowPhaseGJKAlgorithm;
    }
}  

// Add a body to the collision detection
inline void CollisionDetection::addProxyCollisionShape(ProxyShape* proxyShape) {
    
    // Add the body to the broad-phase
    mBroadPhaseAlgorithm.addProxyCollisionShape(proxyShape);

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

// Update a proxy collision shape (that has moved for instance)
inline void CollisionDetection::updateProxyCollisionShape(ProxyShape* shape, const AABB& aabb) {
    mBroadPhaseAlgorithm.updateProxyCollisionShape(shape, aabb);
}

}

#endif
