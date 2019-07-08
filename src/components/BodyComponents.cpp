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
#include "BodyComponents.h"
#include "engine/EntityManager.h"
#include <cassert>
#include <random>

// We want to use the ReactPhysics3D namespace
using namespace reactphysics3d;

// Constructor
BodyComponents::BodyComponents(MemoryAllocator& allocator)
                    :Components(allocator, sizeof(Entity) + sizeof(Body*) + sizeof(List<Entity>) +
                                sizeof(bool) + sizeof(bool) + sizeof(bool) + sizeof(decimal) +
                                sizeof(void*)) {

    // Allocate memory for the components data
    allocate(INIT_NB_ALLOCATED_COMPONENTS);
}

// Allocate memory for a given number of components
void BodyComponents::allocate(uint32 nbComponentsToAllocate) {

    assert(nbComponentsToAllocate > mNbAllocatedComponents);

    // Size for the data of a single component (in bytes)
    const size_t totalSizeBytes = nbComponentsToAllocate * mComponentDataSize;

    // Allocate memory
    void* newBuffer = mMemoryAllocator.allocate(totalSizeBytes);
    assert(newBuffer != nullptr);

    // New pointers to components data
    Entity* newBodiesEntities = static_cast<Entity*>(newBuffer);
    Body** newBodies = reinterpret_cast<Body**>(newBodiesEntities + nbComponentsToAllocate);
    List<Entity>* newProxyShapes = reinterpret_cast<List<Entity>*>(newBodies + nbComponentsToAllocate);
    bool* newIsAllowedToSleep = reinterpret_cast<bool*>(newProxyShapes + nbComponentsToAllocate);
    bool* newIsActive = reinterpret_cast<bool*>(newIsAllowedToSleep + nbComponentsToAllocate);
    bool* newIsSleeping = reinterpret_cast<bool*>(newIsActive + nbComponentsToAllocate);
    decimal* newSleepTimes = reinterpret_cast<decimal*>(newIsSleeping + nbComponentsToAllocate);
    void** newUserData = reinterpret_cast<void**>(newIsSleeping + nbComponentsToAllocate);

    // If there was already components before
    if (mNbComponents > 0) {

        // Copy component data from the previous buffer to the new one
        memcpy(newBodiesEntities, mBodiesEntities, mNbComponents * sizeof(Entity));
        memcpy(newBodies, mBodies, mNbComponents * sizeof(Body*));
        memcpy(newProxyShapes, mProxyShapes, mNbComponents * sizeof(List<Entity>));
        memcpy(newIsAllowedToSleep, mIsAllowedToSleep, mNbComponents * sizeof(bool));
        memcpy(newIsActive, mIsActive, mNbComponents * sizeof(bool));
        memcpy(newIsSleeping, mIsSleeping, mNbComponents * sizeof(bool));
        memcpy(newSleepTimes, mSleepTimes, mNbComponents * sizeof(bool));
        memcpy(newUserData, mUserData, mNbComponents * sizeof(void*));

        // Deallocate previous memory
        mMemoryAllocator.release(mBuffer, mNbAllocatedComponents * mComponentDataSize);
    }

    mBuffer = newBuffer;
    mBodiesEntities = newBodiesEntities;
    mBodies = newBodies;
    mProxyShapes = newProxyShapes;
    mIsAllowedToSleep = newIsAllowedToSleep;
    mIsActive = newIsActive;
    mIsSleeping = newIsSleeping;
    mSleepTimes = newSleepTimes;
    mUserData = newUserData;
    mNbAllocatedComponents = nbComponentsToAllocate;
}

// Add a component
void BodyComponents::addComponent(Entity bodyEntity, bool isSleeping, const BodyComponent& component) {

    // Prepare to add new component (allocate memory if necessary and compute insertion index)
    uint32 index = prepareAddComponent(isSleeping);

    // Insert the new component data
    new (mBodiesEntities + index) Entity(bodyEntity);
    mBodies[index] = component.body;
    new (mProxyShapes + index) List<Entity>(mMemoryAllocator);
    mIsAllowedToSleep[index] = true;
    mIsActive[index] = true;
    mIsSleeping[index] = false;
    mSleepTimes[index] = decimal(0);
    mUserData[index] = nullptr;

    // Map the entity with the new component lookup index
    mMapEntityToComponentIndex.add(Pair<Entity, uint32>(bodyEntity, index));

    mNbComponents++;

    assert(mDisabledStartIndex <= mNbComponents);
    assert(mNbComponents == static_cast<uint32>(mMapEntityToComponentIndex.size()));
}

// Move a component from a source to a destination index in the components array
// The destination location must contain a constructed object
void BodyComponents::moveComponentToIndex(uint32 srcIndex, uint32 destIndex) {

    const Entity entity = mBodiesEntities[srcIndex];

    // Copy the data of the source component to the destination location
    new (mBodiesEntities + destIndex) Entity(mBodiesEntities[srcIndex]);
    mBodies[destIndex] = mBodies[srcIndex];
    new (mProxyShapes + destIndex) List<Entity>(mProxyShapes[srcIndex]);
    mIsAllowedToSleep[destIndex] = mIsAllowedToSleep[srcIndex];
    mIsActive[destIndex] = mIsActive[srcIndex];
    mIsSleeping[destIndex] = mIsSleeping[srcIndex];
    mSleepTimes[destIndex] = mSleepTimes[srcIndex];
    mUserData[destIndex] = mUserData[srcIndex];

    // Destroy the source component
    destroyComponent(srcIndex);

    assert(!mMapEntityToComponentIndex.containsKey(entity));

    // Update the entity to component index mapping
    mMapEntityToComponentIndex.add(Pair<Entity, uint32>(entity, destIndex));

    assert(mMapEntityToComponentIndex[mBodiesEntities[destIndex]] == destIndex);
}

// Swap two components in the array
void BodyComponents::swapComponents(uint32 index1, uint32 index2) {

    // Copy component 1 data
    Entity entity1(mBodiesEntities[index1]);
    Body* body1 = mBodies[index1];
    List<Entity> proxyShapes1(mProxyShapes[index1]);
    bool isAllowedToSleep1 = mIsAllowedToSleep[index1];
    bool isActive1 = mIsActive[index1];
    bool isSleeping1 = mIsSleeping[index1];
    decimal sleepTime1 = mSleepTimes[index1];
    void* userData1 = mUserData[index1];

    // Destroy component 1
    destroyComponent(index1);

    moveComponentToIndex(index2, index1);

    // Reconstruct component 1 at component 2 location
    new (mBodiesEntities + index2) Entity(entity1);
    new (mProxyShapes + index2) List<Entity>(proxyShapes1);
    mBodies[index2] = body1;
    mIsAllowedToSleep[index2] = isAllowedToSleep1;
    mIsActive[index2] = isActive1;
    mIsSleeping[index2] = isSleeping1;
    mUserData[index2] = userData1;

    // Update the entity to component index mapping
    mMapEntityToComponentIndex.add(Pair<Entity, uint32>(entity1, index2));

    assert(mMapEntityToComponentIndex[mBodiesEntities[index1]] == index1);
    assert(mMapEntityToComponentIndex[mBodiesEntities[index2]] == index2);
    assert(mNbComponents == static_cast<uint32>(mMapEntityToComponentIndex.size()));
}

// Destroy a component at a given index
void BodyComponents::destroyComponent(uint32 index) {

    Components::destroyComponent(index);

    assert(mMapEntityToComponentIndex[mBodiesEntities[index]] == index);

    mMapEntityToComponentIndex.remove(mBodiesEntities[index]);

    mBodiesEntities[index].~Entity();
    mBodies[index] = nullptr;
    mProxyShapes[index].~List<Entity>();
    mUserData[index] = nullptr;
}
