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

// Libraries
#include "CollisionWorld.h"
#include <algorithm>

// Namespaces
using namespace reactphysics3d;
using namespace std;

// Constructor
CollisionWorld::CollisionWorld()
               : mCollisionDetection(this, mMemoryManager), mCurrentBodyID(0),
                 mEventListener(nullptr) {

#ifdef IS_PROFILING_ACTIVE

	// Set the profiler
	mCollisionDetection.setProfiler(&mProfiler);

#endif

}

// Destructor
CollisionWorld::~CollisionWorld() {

    // Destroy all the collision bodies that have not been removed
    std::set<CollisionBody*>::iterator itBodies;
    for (itBodies = mBodies.begin(); itBodies != mBodies.end(); ) {
         std::set<CollisionBody*>::iterator itToRemove = itBodies;
         ++itBodies;
        destroyCollisionBody(*itToRemove);
    }

    assert(mBodies.empty());
}

// Create a collision body and add it to the world
/**
 * @param transform Transformation mapping the local-space of the body to world-space
 * @return A pointer to the body that has been created in the world
 */
CollisionBody* CollisionWorld::createCollisionBody(const Transform& transform) {

    // Get the next available body ID
    bodyindex bodyID = computeNextAvailableBodyID();

    // Largest index cannot be used (it is used for invalid index)
    assert(bodyID < std::numeric_limits<reactphysics3d::bodyindex>::max());

    // Create the collision body
    CollisionBody* collisionBody = new (mMemoryManager.allocate(MemoryManager::AllocationType::Pool,
                                        sizeof(CollisionBody)))
                                        CollisionBody(transform, *this, bodyID);

    assert(collisionBody != nullptr);

    // Add the collision body to the world
    mBodies.insert(collisionBody);

#ifdef IS_PROFILING_ACTIVE

	collisionBody->setProfiler(&mProfiler);

#endif

    // Return the pointer to the rigid body
    return collisionBody;
}

// Destroy a collision body
/**
 * @param collisionBody Pointer to the body to destroy
 */
void CollisionWorld::destroyCollisionBody(CollisionBody* collisionBody) {

    // Remove all the collision shapes of the body
    collisionBody->removeAllCollisionShapes();

    // Add the body ID to the list of free IDs
    mFreeBodiesIDs.push_back(collisionBody->getID());

    // Call the destructor of the collision body
    collisionBody->~CollisionBody();

    // Remove the collision body from the list of bodies
    mBodies.erase(collisionBody);

    // Free the object from the memory allocator
    mMemoryManager.release(MemoryManager::AllocationType::Pool, collisionBody, sizeof(CollisionBody));
}

// Return the next available body ID
bodyindex CollisionWorld::computeNextAvailableBodyID() {

    // Compute the body ID
    bodyindex bodyID;
    if (!mFreeBodiesIDs.empty()) {
        bodyID = mFreeBodiesIDs.back();
        mFreeBodiesIDs.pop_back();
    }
    else {
        bodyID = mCurrentBodyID;
        mCurrentBodyID++;
    }

    return bodyID;
}

// Reset all the contact manifolds linked list of each body
void CollisionWorld::resetContactManifoldListsOfBodies() {

    // For each rigid body of the world
    for (std::set<CollisionBody*>::iterator it = mBodies.begin(); it != mBodies.end(); ++it) {

        // Reset the contact manifold list of the body
        (*it)->resetContactManifoldsList();
    }
}

// Test if the AABBs of two bodies overlap
/**
 * @param body1 Pointer to the first body to test
 * @param body2 Pointer to the second body to test
 * @return True if the AABBs of the two bodies overlap and false otherwise
 */
bool CollisionWorld::testAABBOverlap(const CollisionBody* body1,
                                     const CollisionBody* body2) const {

    // If one of the body is not active, we return no overlap
    if (!body1->isActive() || !body2->isActive()) return false;

    // Compute the AABBs of both bodies
    AABB body1AABB = body1->getAABB();
    AABB body2AABB = body2->getAABB();

    // Return true if the two AABBs overlap
    return body1AABB.testCollision(body2AABB);
}

// Report all the bodies that overlap with the aabb in parameter
/**
 * @param aabb AABB used to test for overlap
 * @param overlapCallback Pointer to the callback class to report overlap
 * @param categoryMaskBits bits mask used to filter the bodies to test overlap with
 */
inline void CollisionWorld::testAABBOverlap(const AABB& aabb, OverlapCallback* overlapCallback, unsigned short categoryMaskBits) {
    mCollisionDetection.testAABBOverlap(aabb, overlapCallback, categoryMaskBits);
}

// Return true if two bodies overlap
bool CollisionWorld::testOverlap(CollisionBody* body1, CollisionBody* body2) {
    return mCollisionDetection.testOverlap(body1, body2);
}


// Return the current world-space AABB of given proxy shape
AABB CollisionWorld::getWorldAABB(const ProxyShape* proxyShape) const {

    if (proxyShape->mBroadPhaseID == -1) {
        return AABB();
    }

   return mCollisionDetection.getWorldAABB(proxyShape);
}
