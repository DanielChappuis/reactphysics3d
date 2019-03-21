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
#include "DynamicsComponents.h"
#include "engine/EntityManager.h"
#include <cassert>
#include <random>

// We want to use the ReactPhysics3D namespace
using namespace reactphysics3d;


// Constructor
DynamicsComponents::DynamicsComponents(MemoryAllocator& allocator)
                    :Components(allocator, sizeof(Entity) + sizeof(Vector3) + sizeof (Vector3)) {

    // Allocate memory for the components data
    allocate(INIT_NB_ALLOCATED_COMPONENTS);
}

// Allocate memory for a given number of components
void DynamicsComponents::allocate(uint32 nbComponentsToAllocate) {

    assert(nbComponentsToAllocate > mNbAllocatedComponents);

    // Size for the data of a single component (in bytes)
    const size_t totalSizeBytes = nbComponentsToAllocate * mComponentDataSize;

    // Allocate memory
    void* newBuffer = mMemoryAllocator.allocate(totalSizeBytes);
    assert(newBuffer != nullptr);

    // New pointers to components data
    Entity* newEntities = static_cast<Entity*>(newBuffer);
    Vector3* newLinearVelocities = reinterpret_cast<Vector3*>(newEntities + nbComponentsToAllocate);
    Vector3* newAngularVelocities = reinterpret_cast<Vector3*>(newLinearVelocities + nbComponentsToAllocate);

    // If there was already components before
    if (mNbComponents > 0) {

        // Copy component data from the previous buffer to the new one
        memcpy(newLinearVelocities, mLinearVelocities, mNbComponents * sizeof(Transform));
        memcpy(newAngularVelocities, mAngularVelocities, mNbComponents * sizeof(Entity));

        // Deallocate previous memory
        mMemoryAllocator.release(mBuffer, mNbAllocatedComponents * mComponentDataSize);
    }

    mBuffer = newBuffer;
    mBodies = newEntities;
    mLinearVelocities = newLinearVelocities;
    mAngularVelocities = newAngularVelocities;
    mNbAllocatedComponents = nbComponentsToAllocate;
}

// Add a component
void DynamicsComponents::addComponent(Entity bodyEntity, bool isSleeping, const DynamicsComponent& component) {

    // Prepare to add new component (allocate memory if necessary and compute insertion index)
    uint32 index = prepareAddComponent(isSleeping);

    // Insert the new component data
    new (mBodies + index) Entity(bodyEntity);
    new (mLinearVelocities + index) Vector3(component.linearVelocity);
    new (mAngularVelocities + index) Vector3(component.angularVelocity);

    // Map the entity with the new component lookup index
    mMapEntityToComponentIndex.add(Pair<Entity, uint32>(bodyEntity, index));

    mNbComponents++;

    assert(mDisabledStartIndex <= mNbComponents);
    assert(mNbComponents == static_cast<uint32>(mMapEntityToComponentIndex.size()));
}

// Move a component from a source to a destination index in the components array
// The destination location must contain a constructed object
void DynamicsComponents::moveComponentToIndex(uint32 srcIndex, uint32 destIndex) {

    const Entity entity = mBodies[srcIndex];

    // Copy the data of the source component to the destination location
    new (mBodies + destIndex) Entity(mBodies[srcIndex]);
    new (mLinearVelocities + destIndex) Vector3(mLinearVelocities[srcIndex]);
    new (mAngularVelocities + destIndex) Vector3(mAngularVelocities[srcIndex]);

    // Destroy the source component
    destroyComponent(srcIndex);

    assert(!mMapEntityToComponentIndex.containsKey(entity));

    // Update the entity to component index mapping
    mMapEntityToComponentIndex.add(Pair<Entity, uint32>(entity, destIndex));

    assert(mMapEntityToComponentIndex[mBodies[destIndex]] == destIndex);
}

// Swap two components in the array
void DynamicsComponents::swapComponents(uint32 index1, uint32 index2) {

    // Copy component 1 data
    Entity entity1(mBodies[index1]);
    Vector3 linearVelocity1(mLinearVelocities[index1]);
    Vector3 angularVelocity1(mAngularVelocities[index1]);

    // Destroy component 1
    destroyComponent(index1);

    moveComponentToIndex(index2, index1);

    // Reconstruct component 1 at component 2 location
    new (mBodies + index2) Entity(entity1);
    new (mLinearVelocities + index2) Vector3(linearVelocity1);
    new (mAngularVelocities + index2) Vector3(angularVelocity1);

    // Update the entity to component index mapping
    mMapEntityToComponentIndex.add(Pair<Entity, uint32>(entity1, index2));

    assert(mMapEntityToComponentIndex[mBodies[index1]] == index1);
    assert(mMapEntityToComponentIndex[mBodies[index2]] == index2);
    assert(mNbComponents == static_cast<uint32>(mMapEntityToComponentIndex.size()));
}

// Destroy a component at a given index
void DynamicsComponents::destroyComponent(uint32 index) {

    Components::destroyComponent(index);

    assert(mMapEntityToComponentIndex[mBodies[index]] == index);

    mMapEntityToComponentIndex.remove(mBodies[index]);

    mBodies[index].~Entity();
    mLinearVelocities[index].~Vector3();
    mAngularVelocities[index].~Vector3();
}
