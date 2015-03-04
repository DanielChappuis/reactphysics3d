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

// Libraries
#include "CollisionWorld.h"
#include <algorithm>

// Namespaces
using namespace reactphysics3d;
using namespace std;

// Constructor
CollisionWorld::CollisionWorld()
               : mCollisionDetection(this, mMemoryAllocator), mCurrentBodyID(0),
                 mEventListener(NULL) {

}

// Destructor
CollisionWorld::~CollisionWorld() {
    assert(mCollisionShapes.empty());
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
    CollisionBody* collisionBody = new (mMemoryAllocator.allocate(sizeof(CollisionBody)))
                                        CollisionBody(transform, *this, bodyID);

    assert(collisionBody != NULL);

    // Add the collision body to the world
    mBodies.insert(collisionBody);

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
    mMemoryAllocator.release(collisionBody, sizeof(CollisionBody));
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

// Create a new collision shape in the world.
/// First, this methods checks that the new collision shape does not exist yet in the
/// world. If it already exists, we do not allocate memory for a new one but instead
/// we reuse the existing one. The goal is to only allocate memory for a single
/// collision shape if this one is used for several bodies in the world. To allocate
/// memory for a new collision shape, we use the memory allocator.
CollisionShape* CollisionWorld::createCollisionShape(const CollisionShape& collisionShape) {

    // Check if there is already a similar collision shape in the world
    std::list<CollisionShape*>::iterator it;
    for (it = mCollisionShapes.begin(); it != mCollisionShapes.end(); ++it) {

        if (collisionShape == (*(*it))) {

            // Increment the number of similar created shapes
            (*it)->incrementNbSimilarCreatedShapes();

            // A similar collision shape already exists in the world, so we do not
            // create a new one but we simply return a pointer to the existing one
            return (*it);
        }
    }

    // A similar collision shape does not already exist in the world, so we create a
    // new one and add it to the world
    void* allocatedMemory = mMemoryAllocator.allocate(collisionShape.getSizeInBytes());
    CollisionShape* newCollisionShape = collisionShape.clone(allocatedMemory);
    mCollisionShapes.push_back(newCollisionShape);

    newCollisionShape->incrementNbSimilarCreatedShapes();

    // Return a pointer to the new collision shape
    return newCollisionShape;
}


// Remove a collision shape.
/// First, we check if another body is still using the same collision shape. If so,
/// we keep the allocated collision shape. If it is not the case, we can deallocate
/// the memory associated with the collision shape.
void CollisionWorld::removeCollisionShape(CollisionShape* collisionShape) {

    assert(collisionShape->getNbSimilarCreatedShapes() != 0);

    // Decrement the number of bodies using the same collision shape
    collisionShape->decrementNbSimilarCreatedShapes();

    // If no other body is using the collision shape in the world
    if (collisionShape->getNbSimilarCreatedShapes() == 0) {

        // Remove the shape from the set of shapes in the world
        mCollisionShapes.remove(collisionShape);

        // Compute the size (in bytes) of the collision shape
        size_t nbBytesShape = collisionShape->getSizeInBytes();

        // Call the destructor of the collision shape
        collisionShape->~CollisionShape();

        // Deallocate the memory used by the collision shape
        mMemoryAllocator.release(collisionShape, nbBytesShape);
    }
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

// Test and report collisions between a given shape and all the others
// shapes of the world.
/**
 * @param shape Pointer to the proxy shape to test
 * @param callback Pointer to the object with the callback method
 */
void CollisionWorld::testCollision(const ProxyShape* shape,
                                   CollisionCallback* callback) {

    // Reset all the contact manifolds lists of each body
    resetContactManifoldListsOfBodies();

    // Create the sets of shapes
    std::set<uint> shapes;
    shapes.insert(shape->mBroadPhaseID);
    std::set<uint> emptySet;

    // Perform the collision detection and report contacts
    mCollisionDetection.testCollisionBetweenShapes(callback, shapes, emptySet);
}

// Test and report collisions between two given shapes
/**
 * @param shape1 Pointer to the first proxy shape to test
 * @param shape2 Pointer to the second proxy shape to test
 * @param callback Pointer to the object with the callback method
 */
void CollisionWorld::testCollision(const ProxyShape* shape1,
                                   const ProxyShape* shape2,
                                   CollisionCallback* callback) {

    // Reset all the contact manifolds lists of each body
    resetContactManifoldListsOfBodies();

    // Create the sets of shapes
    std::set<uint> shapes1;
    shapes1.insert(shape1->mBroadPhaseID);
    std::set<uint> shapes2;
    shapes2.insert(shape2->mBroadPhaseID);

    // Perform the collision detection and report contacts
    mCollisionDetection.testCollisionBetweenShapes(callback, shapes1, shapes2);
}

// Test and report collisions between a body and all the others bodies of the
// world
/**
 * @param body Pointer to the first body to test
 * @param callback Pointer to the object with the callback method
 */
void CollisionWorld::testCollision(const CollisionBody* body,
                                   CollisionCallback* callback) {

    // Reset all the contact manifolds lists of each body
    resetContactManifoldListsOfBodies();

    // Create the sets of shapes
    std::set<uint> shapes1;

    // For each shape of the body
    for (const ProxyShape* shape=body->getProxyShapesList(); shape != NULL;
         shape = shape->getNext()) {
        shapes1.insert(shape->mBroadPhaseID);
    }

    std::set<uint> emptySet;

    // Perform the collision detection and report contacts
    mCollisionDetection.testCollisionBetweenShapes(callback, shapes1, emptySet);
}

// Test and report collisions between two bodies
/**
 * @param body1 Pointer to the first body to test
 * @param body2 Pointer to the second body to test
 * @param callback Pointer to the object with the callback method
 */
void CollisionWorld::testCollision(const CollisionBody* body1,
                                   const CollisionBody* body2,
                                   CollisionCallback* callback) {

    // Reset all the contact manifolds lists of each body
    resetContactManifoldListsOfBodies();

    // Create the sets of shapes
    std::set<uint> shapes1;
    for (const ProxyShape* shape=body1->getProxyShapesList(); shape != NULL;
         shape = shape->getNext()) {
        shapes1.insert(shape->mBroadPhaseID);
    }

    std::set<uint> shapes2;
    for (const ProxyShape* shape=body2->getProxyShapesList(); shape != NULL;
         shape = shape->getNext()) {
        shapes2.insert(shape->mBroadPhaseID);
    }

    // Perform the collision detection and report contacts
    mCollisionDetection.testCollisionBetweenShapes(callback, shapes1, shapes2);
}

// Test and report collisions between all shapes of the world
/**
 * @param callback Pointer to the object with the callback method
 */
void CollisionWorld::testCollision(CollisionCallback* callback) {

    // Reset all the contact manifolds lists of each body
    resetContactManifoldListsOfBodies();

    std::set<uint> emptySet;

    // Perform the collision detection and report contacts
    mCollisionDetection.testCollisionBetweenShapes(callback, emptySet, emptySet);
}

