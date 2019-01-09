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

// Libraries
#include "ProxyShapesComponents.h"
#include "engine/EntityManager.h"
#include <cassert>
#include <random>

// We want to use the ReactPhysics3D namespace
using namespace reactphysics3d;

// Constructor
ProxyShapesComponents::ProxyShapesComponents(MemoryAllocator& allocator)
                    :mMemoryAllocator(allocator), mNbComponents(0), mNbAllocatedComponents(0),
                     mSleepingStartIndex(0), mBuffer(nullptr), mMapEntityToComponentIndex(allocator) {

    // Allocate memory for the components data
    allocate(INIT_ALLOCATED_COMPONENTS);
}

// Destructor
ProxyShapesComponents::~ProxyShapesComponents() {

    if (mNbAllocatedComponents > 0) {

        // Destroy all the remaining components
        for (uint32 i = 0; i < mNbComponents; i++) {

            // TODO : MAke sure we do not delete already deleted components
            destroyComponent(i);
        }

        // Size for the data of a single component (in bytes)
        const size_t totalSizeBytes = mNbAllocatedComponents * COMPONENT_DATA_SIZE;

        // Release the allocated memory
        mMemoryAllocator.release(mBuffer, totalSizeBytes);
    }
}

// Allocate memory for a given number of components
void ProxyShapesComponents::allocate(uint32 nbComponentsToAllocate) {

    assert(nbComponentsToAllocate > mNbAllocatedComponents);

    // Size for the data of a single component (in bytes)
    const size_t totalSizeBytes = nbComponentsToAllocate * COMPONENT_DATA_SIZE;

    // Allocate memory
    void* newBuffer = mMemoryAllocator.allocate(totalSizeBytes);
    assert(newBuffer != nullptr);

    // New pointers to components data
    Entity* newEntities = static_cast<Entity*>(newBuffer);
    int* newBroadPhaseIds = reinterpret_cast<int*>(newEntities + nbComponentsToAllocate);
    AABB* newLocalBounds = reinterpret_cast<AABB*>(newBroadPhaseIds + nbComponentsToAllocate);
    Transform* newLocalToBodyTransforms = reinterpret_cast<Transform*>(newLocalBounds + nbComponentsToAllocate);
    CollisionShape** newCollisionShapes = reinterpret_cast<CollisionShape**>(newLocalToBodyTransforms + nbComponentsToAllocate);
    decimal* newMasses = reinterpret_cast<decimal*>(newCollisionShapes + nbComponentsToAllocate);
    uint32* newPreviousBodyProxyShapes = reinterpret_cast<uint32*>(newMasses + nbComponentsToAllocate);
    uint32* newNextBodyProxyShapes = reinterpret_cast<uint32*>(newPreviousBodyProxyShapes + nbComponentsToAllocate);

    // If there was already components before
    if (mNbComponents > 0) {

        // Copy component data from the previous buffer to the new one
        memcpy(newEntities, mEntities, mNbComponents * sizeof(Entity));
        memcpy(newBroadPhaseIds, mBroadPhaseIds, mNbComponents * sizeof(int));
        memcpy(newLocalBounds, mLocalBounds, mNbComponents * sizeof(AABB));
        memcpy(newLocalToBodyTransforms, mLocalToBodyTransforms, mNbComponents * sizeof(Transform));
        memcpy(newCollisionShapes, mCollisionShapes, mNbComponents * sizeof(CollisionShape*));
        memcpy(newMasses, mMasses, mNbComponents * sizeof(decimal));
        memcpy(newPreviousBodyProxyShapes, mPreviousBodyProxyShapes, mNbComponents * sizeof(uint32));
        memcpy(newNextBodyProxyShapes, mNextBodyProxyShapes, mNbComponents * sizeof(uint32));

        // Deallocate previous memory
        mMemoryAllocator.release(mBuffer, mNbAllocatedComponents * COMPONENT_DATA_SIZE);
    }

    mBuffer = newBuffer;
    mEntities = newEntities;
    mBroadPhaseIds = newBroadPhaseIds;
    mLocalBounds = newLocalBounds;
    mLocalToBodyTransforms = newLocalToBodyTransforms;
    mCollisionShapes = newCollisionShapes;
    mMasses = newMasses;
    mPreviousBodyProxyShapes = newPreviousBodyProxyShapes;
    mNextBodyProxyShapes = newNextBodyProxyShapes;

    mNbAllocatedComponents = nbComponentsToAllocate;
}

// Add a new proxy-shape at the beginning of the linked-list of proxy-shapes of a given entity
// If it is the first proxy-shape for the entity, it will create the first item of the linked-list
void ProxyShapesComponents::linkProxyShapeWithEntity(Entity entity, uint32 proxyShapeComponentIndex) {

    auto it = mMapEntityToComponentIndex.find(entity);
    if (it != mMapEntityToComponentIndex.end()) {

        // Get the first proxy-shape of the linked-list
        uint32 firstProxyShapeIndex = (*it).second;

        assert(!hasPreviousProxyShape(firstProxyShapeIndex));

        // Update the previous index of the first item of the list
        mPreviousBodyProxyShapes[firstProxyShapeIndex] = proxyShapeComponentIndex;

        // Map the entity to the new head of the linked-list
        mMapEntityToComponentIndex[entity] = proxyShapeComponentIndex;

        // Add the new proxy-shape at the beginning of the linked-list
        const uint32 nextIndex = firstProxyShapeIndex;
        const uint32 previousIndex = proxyShapeComponentIndex;
        new (mNextBodyProxyShapes + proxyShapeComponentIndex) uint32(nextIndex);
        new (mPreviousBodyProxyShapes + proxyShapeComponentIndex) uint32(previousIndex);

        assert(mPreviousBodyProxyShapes[mNextBodyProxyShapes[proxyShapeComponentIndex]] == proxyShapeComponentIndex);
    }
    else {      // If the entity does not have a proxy-shape yet

        mMapEntityToComponentIndex.add(Pair<Entity, uint32>(entity, proxyShapeComponentIndex));

        // The new proxy-shape has no previous/next components in the linked-list
        new (mNextBodyProxyShapes + proxyShapeComponentIndex) uint32(proxyShapeComponentIndex);
        new (mPreviousBodyProxyShapes + proxyShapeComponentIndex) uint32(proxyShapeComponentIndex);

        assert(!hasNextProxyShape(proxyShapeComponentIndex));
    }

    assert(!hasPreviousProxyShape(proxyShapeComponentIndex));
}

// Add a component
void ProxyShapesComponents::addComponent(Entity entity, bool isSleeping, const ProxyShapeComponent& component) {

    // If we need to allocate more components
    if (mNbComponents == mNbAllocatedComponents) {
        allocate(mNbAllocatedComponents * 2);
    }

    uint32 index;

    // If the component to add is part of a sleeping entity
    if (isSleeping) {

        // Add the component at the end of the array
        index = mNbComponents;

        mSleepingStartIndex = index;
    }
    // If the component to add is not part of a sleeping entity
    else {

        // If there already are sleeping components
        if (mSleepingStartIndex != mNbComponents) {

            // Move the first sleeping component to the end of the array
            moveComponentToIndex(mSleepingStartIndex, mNbComponents);
        }

        index = mSleepingStartIndex;

        mSleepingStartIndex++;
    }

    // Insert the new component data
    new (mEntities + index) Entity(entity);
    new (mBroadPhaseIds + index) int(component.broadPhaseId);
    new (mLocalBounds + index) AABB(component.localBounds);
    new (mLocalToBodyTransforms + index) Transform(component.localToBodyTransform);
    mCollisionShapes[index] = component.collisionShape;
    new (mMasses + index) decimal(component.mass);

    mNbComponents++;

    // Map the entity with the new component lookup index
    linkProxyShapeWithEntity(entity, index);

    assert(mSleepingStartIndex <= mNbComponents);
    assert(mPreviousBodyProxyShapes[mNextBodyProxyShapes[index]] == index || !hasNextProxyShape(index));
    assert(mNextBodyProxyShapes[mPreviousBodyProxyShapes[index]] == index || !hasPreviousProxyShape(index));
}

// Move a component from a source to a destination index in the components array
// The destination location must contain a constructed object
void ProxyShapesComponents::moveComponentToIndex(uint32 srcIndex, uint32 destIndex) {

    const Entity entity = mEntities[srcIndex];

    const bool isFirstProxyShapeOfBody = mMapEntityToComponentIndex[entity] == srcIndex;

    assert(isFirstProxyShapeOfBody || hasPreviousProxyShape(srcIndex));

    // Copy the data of the source component to the destination location
    new (mEntities + destIndex) Entity(mEntities[srcIndex]);
    new (mBroadPhaseIds + destIndex) int(mBroadPhaseIds[srcIndex]);
    new (mLocalBounds + destIndex) AABB(mLocalBounds[srcIndex]);
    new (mLocalToBodyTransforms + destIndex) Transform(mLocalToBodyTransforms[srcIndex]);
    mCollisionShapes[destIndex] = mCollisionShapes[srcIndex];
    new (mMasses + destIndex) decimal(mMasses[srcIndex]);
    new (mPreviousBodyProxyShapes + destIndex) uint32(hasPreviousProxyShape(srcIndex) ? mPreviousBodyProxyShapes[srcIndex] : destIndex);
    new (mNextBodyProxyShapes + destIndex) uint32(hasNextProxyShape(srcIndex) ? mNextBodyProxyShapes[srcIndex] : destIndex);

    // Update the the next proxy-shape index of the previous proxy-shape
    if (hasPreviousProxyShape(destIndex)) {
        assert(mNextBodyProxyShapes[mPreviousBodyProxyShapes[destIndex]] == srcIndex);
        mNextBodyProxyShapes[mPreviousBodyProxyShapes[destIndex]] = destIndex;
    }

    // Update the the previous proxy-shape index of the next proxy-shape
    if (hasNextProxyShape(destIndex)) {
        assert(mPreviousBodyProxyShapes[mNextBodyProxyShapes[destIndex]] == srcIndex);
        mPreviousBodyProxyShapes[mNextBodyProxyShapes[destIndex]] = destIndex;
    }

    // Destroy the source component
    destroyComponent(srcIndex);

    // Update the entity to component index mapping if it is the first proxy-shape of the body
    if (isFirstProxyShapeOfBody) {

        mMapEntityToComponentIndex[entity] = destIndex;
        assert(mMapEntityToComponentIndex[mEntities[destIndex]] == destIndex);
    }

    assert(mPreviousBodyProxyShapes[mNextBodyProxyShapes[destIndex]] == destIndex || !hasNextProxyShape(destIndex));
    assert(mNextBodyProxyShapes[mPreviousBodyProxyShapes[destIndex]] == destIndex || !hasPreviousProxyShape(destIndex));
}

// Swap two components in the array
void ProxyShapesComponents::swapComponents(uint32 index1, uint32 index2) {

    // Copy component 1 data
    Entity entity1(mEntities[index1]);
    int broadPhaseId1 = mBroadPhaseIds[index1];
    AABB localBounds1 = mLocalBounds[index1];
    Transform localToBodyTransform1 = mLocalToBodyTransforms[index1];
    CollisionShape* collisionShape1 = mCollisionShapes[index1];
    decimal mass1 = mMasses[index1];
    uint32 previousProxyShape1 = hasPreviousProxyShape(index1) ? mPreviousBodyProxyShapes[index1] : index2;
    uint32 nextProxyShape1 = hasNextProxyShape(index1) ? mNextBodyProxyShapes[index1] : index2;

    const bool isFirstBodyProxyShape1 = mMapEntityToComponentIndex[mEntities[index1]] == index1;

    // Destroy component 1
    destroyComponent(index1);

    moveComponentToIndex(index2, index1);

    // Reconstruct component 1 at component 2 location
    new (mEntities + index2) Entity(entity1);
    new (mBroadPhaseIds + index2) int(broadPhaseId1);
    new (mLocalBounds + index2) AABB(localBounds1);
    new (mLocalToBodyTransforms + index2) Transform(localToBodyTransform1);
    mCollisionShapes[index2] = collisionShape1;
    new (mMasses + index2) decimal(mass1);
    new (mPreviousBodyProxyShapes + index2) uint32(previousProxyShape1);
    new (mNextBodyProxyShapes + index2) uint32(nextProxyShape1);

    // Update the the next proxy-shape index of the previous proxy-shape
    if (hasPreviousProxyShape(index2)) {
        assert(mNextBodyProxyShapes[mPreviousBodyProxyShapes[index2]] == index1);
        mNextBodyProxyShapes[mPreviousBodyProxyShapes[index2]] = index2;
    }

    // Update the the previous proxy-shape index of the next proxy-shape
    if (hasNextProxyShape(index2)) {
        assert(mPreviousBodyProxyShapes[mNextBodyProxyShapes[index2]] == index1);
        mPreviousBodyProxyShapes[mNextBodyProxyShapes[index2]] = index2;
    }

    // Update the entity to component index mapping if it is the first body proxy-shape
    if (isFirstBodyProxyShape1) {
        assert(!hasPreviousProxyShape(index2));
        mMapEntityToComponentIndex[entity1] = index2;
    }

    assert(mPreviousBodyProxyShapes[mNextBodyProxyShapes[index1]] == index1 || !hasNextProxyShape(index1));
    assert(mPreviousBodyProxyShapes[mNextBodyProxyShapes[index2]] == index2 || !hasNextProxyShape(index2));
    assert(mNextBodyProxyShapes[mPreviousBodyProxyShapes[index1]] == index1 || !hasPreviousProxyShape(index1));
    assert(mNextBodyProxyShapes[mPreviousBodyProxyShapes[index2]] == index2 || !hasPreviousProxyShape(index2));
}

// Remove a component at a given index
void ProxyShapesComponents::removeComponent(uint32 index) {

    assert(index < mNbComponents);

    // We want to keep the arrays tightly packed. Therefore, when a component is removed,
    // we replace it with the last element of the array. But we need to make sure that sleeping
    // and non-sleeping components stay grouped together.

    // If the proxy-shape to destroy does not have a previous proxy-shape in the linked-list of proxy-shapes of its body
    if (!hasPreviousProxyShape(index)) {

        // Remove the mapping from entity to component index
        mMapEntityToComponentIndex.remove(mEntities[index]);

        // If the proxy-shape has a next proxy-shape in the linked-list
        if (hasNextProxyShape(index)) {

            assert(mEntities[index] == mEntities[mNextBodyProxyShapes[index]]);

            mMapEntityToComponentIndex.add(Pair<Entity, uint32>(mEntities[mNextBodyProxyShapes[index]], mNextBodyProxyShapes[index]));
        }
    }

    // Update the linked-list of proxy-shapes of a body when a proxy-shape is removed
    if (hasPreviousProxyShape(index)) {

        assert(mNextBodyProxyShapes[mPreviousBodyProxyShapes[index]] == index);
        assert(mEntities[index] == mEntities[mPreviousBodyProxyShapes[index]]);

        // If the current proxy-shape to remove has a next proxy-shape in the linked-list
        if (hasNextProxyShape(index)) {

            assert(mPreviousBodyProxyShapes[mNextBodyProxyShapes[index]] == index);

            // Make sure the next proxy-shape will follow the previous proxy-shape in the linked-list
            mNextBodyProxyShapes[mPreviousBodyProxyShapes[index]] = mNextBodyProxyShapes[index];
        }
        else {

            mNextBodyProxyShapes[mPreviousBodyProxyShapes[index]] = mPreviousBodyProxyShapes[index];
        }
    }

    // Update the linked-list of proxy-shapes of a body when a proxy-shape is removed
    if (hasNextProxyShape(index)) {

        assert(mPreviousBodyProxyShapes[mNextBodyProxyShapes[index]] == index);
        assert(mEntities[index] == mEntities[mNextBodyProxyShapes[index]]);

        // If the current proxy-shape to remove has a previous proxy-shape in the linked-list
        if (hasPreviousProxyShape(index)) {

            // Make sure the previous proxy-shape will precede the next proxy-shape in the linked-list
            mPreviousBodyProxyShapes[mNextBodyProxyShapes[index]] = mPreviousBodyProxyShapes[index];
        }
        else {

            mPreviousBodyProxyShapes[mNextBodyProxyShapes[index]] = mNextBodyProxyShapes[index];
        }
    }

    // Destroy the component
    destroyComponent(index);

    // If the component to remove is sleeping
    if (index >= mSleepingStartIndex) {

        // If the component is not the last one
        if (index != mNbComponents - 1) {

            // We replace it by the last sleeping component
            moveComponentToIndex(mNbComponents - 1, index);
        }
    }
    else {   // If the component to remove is not sleeping

        // If it not the last awake component
        if (index != mSleepingStartIndex - 1) {

            // We replace it by the last awake component
            moveComponentToIndex(mSleepingStartIndex - 1, index);
        }

        // If there are sleeping components at the end
        if (mSleepingStartIndex != mNbComponents) {

            // We replace the last awake component by the last sleeping component
            moveComponentToIndex(mNbComponents - 1, mSleepingStartIndex - 1);
        }

        mSleepingStartIndex--;
    }

    assert(mSleepingStartIndex <= mNbComponents);
    mNbComponents--;
}

// Return true if a given proxy-shape component has a previous proxy-shape in the linked-list of proxy-shapes of a body
bool ProxyShapesComponents::hasPreviousProxyShape(uint32 index) const {
    assert(index < mNbComponents);
    return mPreviousBodyProxyShapes[index] != index;
}

// Return true if a given proxy-shape component has a next proxy-shape in the linked-list of proxy-shapes of a body
bool ProxyShapesComponents::hasNextProxyShape(uint32 index) const {
    assert(index < mNbComponents);
    return mNextBodyProxyShapes[index] != index;
}

// Destroy a component at a given index
void ProxyShapesComponents::destroyComponent(uint32 index) {

    assert(index < mNbComponents);

    mEntities[index].~Entity();
    mLocalBounds[index].~AABB();
    mLocalToBodyTransforms[index].~Transform();
    mCollisionShapes[index] = nullptr;
}

// Notify if a given entity is sleeping or not
void ProxyShapesComponents::setIsEntitySleeping(Entity entity, bool isSleeping) {

    const uint32 index = mMapEntityToComponentIndex[entity];

    // If the component was sleeping and is not sleeping anymore
    if (!isSleeping && index >= mSleepingStartIndex) {

        assert(mSleepingStartIndex < mNbComponents);

        // If the sleeping component is not the first sleeping component
        if (mSleepingStartIndex != index) {

            // Swap the first sleeping component with the one we need to wake up
            swapComponents(index, mSleepingStartIndex);
        }

        mSleepingStartIndex++;
    }
    // If the component was awake and must now go to sleep
    else if (isSleeping && index < mSleepingStartIndex) {

        assert(mSleepingStartIndex > 0);

        // If the awake component is not the only awake component
        if (index != mSleepingStartIndex - 1) {

            // Swap the last awake component with the one we need to put to sleep
            swapComponents(index, mSleepingStartIndex - 1);
        }

        mSleepingStartIndex--;
    }

    assert(mSleepingStartIndex <= mNbComponents);
    assert(mPreviousBodyProxyShapes[mNextBodyProxyShapes[index]] == index || !hasNextProxyShape(index));
    assert(mNextBodyProxyShapes[mPreviousBodyProxyShapes[index]] == index || !hasPreviousProxyShape(index));
}

// Remove all the components of a given entity
void ProxyShapesComponents::removeComponents(Entity entity) {

   auto it = mMapEntityToComponentIndex.find(entity);

   // While there are components for this entity
   while (it != mMapEntityToComponentIndex.end()) {

       // Remove the component
       removeComponent(it->second);

       it = mMapEntityToComponentIndex.find(entity);
   }

   assert(!mMapEntityToComponentIndex.containsKey(entity));
}
