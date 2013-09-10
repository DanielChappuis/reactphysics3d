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

// Libraries
#include "CollisionWorld.h"
#include <algorithm>

// Namespaces
using namespace reactphysics3d;
using namespace std;

// Constructor
CollisionWorld::CollisionWorld()
               : mCollisionDetection(this, mMemoryAllocator), mCurrentBodyID(0) {

}

// Destructor
CollisionWorld::~CollisionWorld() {
    assert(mCollisionShapes.empty());
    assert(mBodies.empty());
}

// Notify the world about a new broad-phase overlapping pair
void CollisionWorld::notifyAddedOverlappingPair(const BroadPhasePair* addedPair) {

    // TODO : Implement this method
}

// Notify the world about a removed broad-phase overlapping pair
void CollisionWorld::notifyRemovedOverlappingPair(const BroadPhasePair* removedPair) {

    // TODO : Implement this method
}

// Notify the world about a new narrow-phase contact
void CollisionWorld::notifyNewContact(const BroadPhasePair* broadPhasePair,
                                      const ContactPointInfo* contactInfo) {

    // TODO : Implement this method
}

// Update the overlapping pair
inline void CollisionWorld::updateOverlappingPair(const BroadPhasePair* pair) {

}

// Create a collision body and add it to the world
CollisionBody* CollisionWorld::createCollisionBody(const Transform& transform,
                                                   CollisionShape* collisionShape) {

    // Get the next available body ID
    bodyindex bodyID = computeNextAvailableBodyID();

    // Largest index cannot be used (it is used for invalid index)
    assert(bodyID < std::numeric_limits<reactphysics3d::bodyindex>::max());

    // Create the collision body
    CollisionBody* collisionBody = new (mMemoryAllocator.allocate(sizeof(CollisionBody)))
                                        CollisionBody(transform, collisionShape, bodyID);

    assert(collisionBody != NULL);

    // Add the collision body to the world
    mBodies.insert(collisionBody);

    // Add the collision body to the collision detection
    mCollisionDetection.addBody(collisionBody);

    // Return the pointer to the rigid body
    return collisionBody;
}

// Destroy a collision body
void CollisionWorld::destroyCollisionBody(CollisionBody* collisionBody) {

    // Remove the body from the collision detection
    mCollisionDetection.removeBody(collisionBody);

    // Add the body ID to the list of free IDs
    mFreeBodiesIDs.push_back(collisionBody->getID());

    // Call the destructor of the collision body
    collisionBody->CollisionBody::~CollisionBody();

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

// Create a new collision shape.
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
        collisionShape->CollisionShape::~CollisionShape();

        // Deallocate the memory used by the collision shape
        mMemoryAllocator.release(collisionShape, nbBytesShape);
    }
}


