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
#include "RigidBodyComponents.h"
#include "engine/EntityManager.h"
#include "body/RigidBody.h"
#include <cassert>
#include <random>

// We want to use the ReactPhysics3D namespace
using namespace reactphysics3d;

// Constructor
RigidBodyComponents::RigidBodyComponents(MemoryAllocator& allocator)
                    :Components(allocator, sizeof(Entity) + sizeof(RigidBody*) +
                                sizeof(bool) + sizeof(bool) + sizeof(decimal) + sizeof(BodyType) +
                                sizeof(Vector3) + sizeof(Vector3)) {

    // Allocate memory for the components data
    allocate(INIT_NB_ALLOCATED_COMPONENTS);
}

// Allocate memory for a given number of components
void RigidBodyComponents::allocate(uint32 nbComponentsToAllocate) {

    assert(nbComponentsToAllocate > mNbAllocatedComponents);

    // Size for the data of a single component (in bytes)
    const size_t totalSizeBytes = nbComponentsToAllocate * mComponentDataSize;

    // Allocate memory
    void* newBuffer = mMemoryAllocator.allocate(totalSizeBytes);
    assert(newBuffer != nullptr);

    // New pointers to components data
    Entity* newBodiesEntities = static_cast<Entity*>(newBuffer);
    RigidBody** newBodies = reinterpret_cast<RigidBody**>(newBodiesEntities + nbComponentsToAllocate);
    bool* newIsAllowedToSleep = reinterpret_cast<bool*>(newBodies + nbComponentsToAllocate);
    bool* newIsSleeping = reinterpret_cast<bool*>(newIsAllowedToSleep + nbComponentsToAllocate);
    decimal* newSleepTimes = reinterpret_cast<decimal*>(newIsSleeping + nbComponentsToAllocate);
    BodyType* newBodyTypes = reinterpret_cast<BodyType*>(newSleepTimes + nbComponentsToAllocate);
    Vector3* newLinearVelocities = reinterpret_cast<Vector3*>(newBodyTypes + nbComponentsToAllocate);
    Vector3* newAngularVelocities = reinterpret_cast<Vector3*>(newLinearVelocities + nbComponentsToAllocate);

    // If there was already components before
    if (mNbComponents > 0) {

        // Copy component data from the previous buffer to the new one
        memcpy(newBodiesEntities, mBodiesEntities, mNbComponents * sizeof(Entity));
        memcpy(newBodies, mRigidBodies, mNbComponents * sizeof(RigidBody*));
        memcpy(newIsAllowedToSleep, mIsAllowedToSleep, mNbComponents * sizeof(bool));
        memcpy(newIsSleeping, mIsSleeping, mNbComponents * sizeof(bool));
        memcpy(newSleepTimes, mSleepTimes, mNbComponents * sizeof(bool));
        memcpy(newBodyTypes, mBodyTypes, mNbComponents * sizeof(BodyType));
        memcpy(newLinearVelocities, mLinearVelocities, mNbComponents * sizeof(Vector3));
        memcpy(newAngularVelocities, mAngularVelocities, mNbComponents * sizeof(Vector3));

        // Deallocate previous memory
        mMemoryAllocator.release(mBuffer, mNbAllocatedComponents * mComponentDataSize);
    }

    mBuffer = newBuffer;
    mBodiesEntities = newBodiesEntities;
    mRigidBodies = newBodies;
    mIsAllowedToSleep = newIsAllowedToSleep;
    mIsSleeping = newIsSleeping;
    mSleepTimes = newSleepTimes;
    mNbAllocatedComponents = nbComponentsToAllocate;
    mBodyTypes = newBodyTypes;
    mLinearVelocities = newLinearVelocities;
    mAngularVelocities = newAngularVelocities;
}

// Add a component
void RigidBodyComponents::addComponent(Entity bodyEntity, bool isSleeping, const RigidBodyComponent& component) {

    // Prepare to add new component (allocate memory if necessary and compute insertion index)
    uint32 index = prepareAddComponent(isSleeping);

    // Insert the new component data
    new (mBodiesEntities + index) Entity(bodyEntity);
    mRigidBodies[index] = component.body;
    mIsAllowedToSleep[index] = true;
    mIsSleeping[index] = false;
    mSleepTimes[index] = decimal(0);
    mBodyTypes[index] = component.bodyType;
    new (mLinearVelocities + index) Vector3(0, 0, 0);
    new (mAngularVelocities + index) Vector3(0, 0, 0);

    // Map the entity with the new component lookup index
    mMapEntityToComponentIndex.add(Pair<Entity, uint32>(bodyEntity, index));

    mNbComponents++;

    assert(mDisabledStartIndex <= mNbComponents);
    assert(mNbComponents == static_cast<uint32>(mMapEntityToComponentIndex.size()));
}

// Move a component from a source to a destination index in the components array
// The destination location must contain a constructed object
void RigidBodyComponents::moveComponentToIndex(uint32 srcIndex, uint32 destIndex) {

    const Entity entity = mBodiesEntities[srcIndex];

    // Copy the data of the source component to the destination location
    new (mBodiesEntities + destIndex) Entity(mBodiesEntities[srcIndex]);
    mRigidBodies[destIndex] = mRigidBodies[srcIndex];
    mIsAllowedToSleep[destIndex] = mIsAllowedToSleep[srcIndex];
    mIsSleeping[destIndex] = mIsSleeping[srcIndex];
    mSleepTimes[destIndex] = mSleepTimes[srcIndex];
    mBodyTypes[destIndex] = mBodyTypes[srcIndex];
    new (mLinearVelocities + destIndex) Vector3(mLinearVelocities[srcIndex]);
    new (mAngularVelocities + destIndex) Vector3(mAngularVelocities[srcIndex]);

    // Destroy the source component
    destroyComponent(srcIndex);

    assert(!mMapEntityToComponentIndex.containsKey(entity));

    // Update the entity to component index mapping
    mMapEntityToComponentIndex.add(Pair<Entity, uint32>(entity, destIndex));

    assert(mMapEntityToComponentIndex[mBodiesEntities[destIndex]] == destIndex);
}

// Swap two components in the array
void RigidBodyComponents::swapComponents(uint32 index1, uint32 index2) {

    // Copy component 1 data
    Entity entity1(mBodiesEntities[index1]);
    RigidBody* body1 = mRigidBodies[index1];
    bool isAllowedToSleep1 = mIsAllowedToSleep[index1];
    bool isSleeping1 = mIsSleeping[index1];
    decimal sleepTime1 = mSleepTimes[index1];
    BodyType bodyType1 = mBodyTypes[index1];
    Vector3 linearVelocity1(mLinearVelocities[index1]);
    Vector3 angularVelocity1(mAngularVelocities[index1]);

    // Destroy component 1
    destroyComponent(index1);

    moveComponentToIndex(index2, index1);

    // Reconstruct component 1 at component 2 location
    new (mBodiesEntities + index2) Entity(entity1);
    mRigidBodies[index2] = body1;
    mIsAllowedToSleep[index2] = isAllowedToSleep1;
    mIsSleeping[index2] = isSleeping1;
    mSleepTimes[index2] = sleepTime1;
    mBodyTypes[index2] = bodyType1;
    new (mLinearVelocities + index2) Vector3(linearVelocity1);
    new (mAngularVelocities + index2) Vector3(angularVelocity1);

    // Update the entity to component index mapping
    mMapEntityToComponentIndex.add(Pair<Entity, uint32>(entity1, index2));

    assert(mMapEntityToComponentIndex[mBodiesEntities[index1]] == index1);
    assert(mMapEntityToComponentIndex[mBodiesEntities[index2]] == index2);
    assert(mNbComponents == static_cast<uint32>(mMapEntityToComponentIndex.size()));
}

// Destroy a component at a given index
void RigidBodyComponents::destroyComponent(uint32 index) {

    Components::destroyComponent(index);

    assert(mMapEntityToComponentIndex[mBodiesEntities[index]] == index);

    mMapEntityToComponentIndex.remove(mBodiesEntities[index]);

    mBodiesEntities[index].~Entity();
    mRigidBodies[index] = nullptr;
    mLinearVelocities[index].~Vector3();
    mAngularVelocities[index].~Vector3();
}
