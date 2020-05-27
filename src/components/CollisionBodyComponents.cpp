/********************************************************************************
* ReactPhysics3D physics library, http://www.reactphysics3d.com                 *
* Copyright (c) 2010-2020 Daniel Chappuis                                       *
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
#include <reactphysics3d/components/CollisionBodyComponents.h>
#include <reactphysics3d/engine/EntityManager.h>
#include <cassert>
#include <random>

// We want to use the ReactPhysics3D namespace
using namespace reactphysics3d;

// Constructor
CollisionBodyComponents::CollisionBodyComponents(MemoryAllocator& allocator)
                    :Components(allocator, sizeof(Entity) + sizeof(CollisionBody*) + sizeof(List<Entity>) +
                                sizeof(bool) + sizeof(void*)) {

    // Allocate memory for the components data
    allocate(INIT_NB_ALLOCATED_COMPONENTS);
}

// Allocate memory for a given number of components
void CollisionBodyComponents::allocate(uint32 nbComponentsToAllocate) {

    assert(nbComponentsToAllocate > mNbAllocatedComponents);

    // Size for the data of a single component (in bytes)
    const size_t totalSizeBytes = nbComponentsToAllocate * mComponentDataSize;

    // Allocate memory
    void* newBuffer = mMemoryAllocator.allocate(totalSizeBytes);
    assert(newBuffer != nullptr);

    // New pointers to components data
    Entity* newBodiesEntities = static_cast<Entity*>(newBuffer);
    CollisionBody** newBodies = reinterpret_cast<CollisionBody**>(newBodiesEntities + nbComponentsToAllocate);
    List<Entity>* newColliders = reinterpret_cast<List<Entity>*>(newBodies + nbComponentsToAllocate);
    bool* newIsActive = reinterpret_cast<bool*>(newColliders + nbComponentsToAllocate);
    void** newUserData = reinterpret_cast<void**>(newIsActive + nbComponentsToAllocate);

    // If there was already components before
    if (mNbComponents > 0) {

        // Copy component data from the previous buffer to the new one
        memcpy(newBodiesEntities, mBodiesEntities, mNbComponents * sizeof(Entity));
        memcpy(newBodies, mBodies, mNbComponents * sizeof(CollisionBody*));
        memcpy(newColliders, mColliders, mNbComponents * sizeof(List<Entity>));
        memcpy(newIsActive, mIsActive, mNbComponents * sizeof(bool));
        memcpy(newUserData, mUserData, mNbComponents * sizeof(void*));

        // Deallocate previous memory
        mMemoryAllocator.release(mBuffer, mNbAllocatedComponents * mComponentDataSize);
    }

    mBuffer = newBuffer;
    mBodiesEntities = newBodiesEntities;
    mBodies = newBodies;
    mColliders = newColliders;
    mIsActive = newIsActive;
    mUserData = newUserData;
    mNbAllocatedComponents = nbComponentsToAllocate;
}

// Add a component
void CollisionBodyComponents::addComponent(Entity bodyEntity, bool isSleeping, const CollisionBodyComponent& component) {

    // Prepare to add new component (allocate memory if necessary and compute insertion index)
    uint32 index = prepareAddComponent(isSleeping);

    // Insert the new component data
    new (mBodiesEntities + index) Entity(bodyEntity);
    mBodies[index] = component.body;
    new (mColliders + index) List<Entity>(mMemoryAllocator);
    mIsActive[index] = true;
    mUserData[index] = nullptr;

    // Map the entity with the new component lookup index
    mMapEntityToComponentIndex.add(Pair<Entity, uint32>(bodyEntity, index));

    mNbComponents++;

    assert(mDisabledStartIndex <= mNbComponents);
    assert(mNbComponents == static_cast<uint32>(mMapEntityToComponentIndex.size()));
}

// Move a component from a source to a destination index in the components array
// The destination location must contain a constructed object
void CollisionBodyComponents::moveComponentToIndex(uint32 srcIndex, uint32 destIndex) {

    const Entity entity = mBodiesEntities[srcIndex];

    // Copy the data of the source component to the destination location
    new (mBodiesEntities + destIndex) Entity(mBodiesEntities[srcIndex]);
    mBodies[destIndex] = mBodies[srcIndex];
    new (mColliders + destIndex) List<Entity>(mColliders[srcIndex]);
    mIsActive[destIndex] = mIsActive[srcIndex];
    mUserData[destIndex] = mUserData[srcIndex];

    // Destroy the source component
    destroyComponent(srcIndex);

    assert(!mMapEntityToComponentIndex.containsKey(entity));

    // Update the entity to component index mapping
    mMapEntityToComponentIndex.add(Pair<Entity, uint32>(entity, destIndex));

    assert(mMapEntityToComponentIndex[mBodiesEntities[destIndex]] == destIndex);
}

// Swap two components in the array
void CollisionBodyComponents::swapComponents(uint32 index1, uint32 index2) {

    // Copy component 1 data
    Entity entity1(mBodiesEntities[index1]);
    CollisionBody* body1 = mBodies[index1];
    List<Entity> colliders1(mColliders[index1]);
    bool isActive1 = mIsActive[index1];
    void* userData1 = mUserData[index1];

    // Destroy component 1
    destroyComponent(index1);

    moveComponentToIndex(index2, index1);

    // Reconstruct component 1 at component 2 location
    new (mBodiesEntities + index2) Entity(entity1);
    new (mColliders + index2) List<Entity>(colliders1);
    mBodies[index2] = body1;
    mIsActive[index2] = isActive1;
    mUserData[index2] = userData1;

    // Update the entity to component index mapping
    mMapEntityToComponentIndex.add(Pair<Entity, uint32>(entity1, index2));

    assert(mMapEntityToComponentIndex[mBodiesEntities[index1]] == index1);
    assert(mMapEntityToComponentIndex[mBodiesEntities[index2]] == index2);
    assert(mNbComponents == static_cast<uint32>(mMapEntityToComponentIndex.size()));
}

// Destroy a component at a given index
void CollisionBodyComponents::destroyComponent(uint32 index) {

    Components::destroyComponent(index);

    assert(mMapEntityToComponentIndex[mBodiesEntities[index]] == index);

    mMapEntityToComponentIndex.remove(mBodiesEntities[index]);

    mBodiesEntities[index].~Entity();
    mBodies[index] = nullptr;
    mColliders[index].~List<Entity>();
    mUserData[index] = nullptr;
}
