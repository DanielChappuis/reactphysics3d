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
#include "JointComponents.h"
#include "engine/EntityManager.h"
#include <cassert>

// We want to use the ReactPhysics3D namespace
using namespace reactphysics3d;

// Constructor
JointComponents::JointComponents(MemoryAllocator& allocator)
                    :Components(allocator, sizeof(Entity) + sizeof(Entity) + sizeof(Entity) +
                                sizeof(Joint*)) {

    // Allocate memory for the components data
    allocate(INIT_NB_ALLOCATED_COMPONENTS);
}

// Allocate memory for a given number of components
void JointComponents::allocate(uint32 nbComponentsToAllocate) {

    assert(nbComponentsToAllocate > mNbAllocatedComponents);

    // Size for the data of a single component (in bytes)
    const size_t totalSizeBytes = nbComponentsToAllocate * mComponentDataSize;

    // Allocate memory
    void* newBuffer = mMemoryAllocator.allocate(totalSizeBytes);
    assert(newBuffer != nullptr);

    // New pointers to components data
    Entity* newJointsEntities = static_cast<Entity*>(newBuffer);
    Entity* newBody1Entities = reinterpret_cast<Entity*>(newJointsEntities + nbComponentsToAllocate);
    Entity* newBody2Entities = reinterpret_cast<Entity*>(newBody1Entities + nbComponentsToAllocate);
    Joint** newJoints = reinterpret_cast<Joint**>(newBody2Entities + nbComponentsToAllocate);

    // If there was already components before
    if (mNbComponents > 0) {

        // Copy component data from the previous buffer to the new one
        memcpy(newJointsEntities, mJointEntities, mNbComponents * sizeof(Entity));
        memcpy(newBody1Entities, mBody1Entities, mNbComponents * sizeof(Entity));
        memcpy(newBody2Entities, mBody2Entities, mNbComponents * sizeof(Entity));
        memcpy(newJoints, mJoints, mNbComponents * sizeof(Joint*));

        // Deallocate previous memory
        mMemoryAllocator.release(mBuffer, mNbAllocatedComponents * mComponentDataSize);
    }

    mBuffer = newBuffer;
    mNbAllocatedComponents = nbComponentsToAllocate;
    mJointEntities = newJointsEntities;
    mBody1Entities = newBody1Entities;
    mBody2Entities = newBody2Entities;
    mJoints = newJoints;
}

// Add a component
void JointComponents::addComponent(Entity jointEntity, bool isSleeping, const JointComponent& component) {

    // Prepare to add new component (allocate memory if necessary and compute insertion index)
    uint32 index = prepareAddComponent(isSleeping);

    // Insert the new component data
    new (mJointEntities + index) Entity(jointEntity);
    new (mBody1Entities + index) Entity(component.body1Entity);
    new (mBody2Entities + index) Entity(component.body2Entity);
    mJoints[index] = component.joint;

    // Map the entity with the new component lookup index
    mMapEntityToComponentIndex.add(Pair<Entity, uint32>(jointEntity, index));

    mNbComponents++;

    assert(mDisabledStartIndex <= mNbComponents);
    assert(mNbComponents == static_cast<uint32>(mMapEntityToComponentIndex.size()));
}

// Move a component from a source to a destination index in the components array
// The destination location must contain a constructed object
void JointComponents::moveComponentToIndex(uint32 srcIndex, uint32 destIndex) {

    const Entity entity = mJointEntities[srcIndex];

    // Copy the data of the source component to the destination location
    new (mJointEntities + destIndex) Entity(mJointEntities[srcIndex]);
    new (mBody1Entities + destIndex) Entity(mBody1Entities[srcIndex]);
    new (mBody2Entities + destIndex) Entity(mBody2Entities[srcIndex]);
    mJoints[destIndex] = mJoints[srcIndex];

    // Destroy the source component
    destroyComponent(srcIndex);

    assert(!mMapEntityToComponentIndex.containsKey(entity));

    // Update the entity to component index mapping
    mMapEntityToComponentIndex.add(Pair<Entity, uint32>(entity, destIndex));

    assert(mMapEntityToComponentIndex[mJointEntities[destIndex]] == destIndex);
}

// Swap two components in the array
void JointComponents::swapComponents(uint32 index1, uint32 index2) {

    // Copy component 1 data
    Entity jointEntity1(mJointEntities[index1]);
    Entity body1Entity1(mBody1Entities[index1]);
    Entity body2Entity1(mBody2Entities[index1]);
    Joint* joint1 = mJoints[index1];

    // Destroy component 1
    destroyComponent(index1);

    moveComponentToIndex(index2, index1);

    // Reconstruct component 1 at component 2 location
    new (mJointEntities + index2) Entity(jointEntity1);
    new (mBody1Entities + index2) Entity(body1Entity1);
    new (mBody2Entities + index2) Entity(body2Entity1);
    mJoints[index2] = joint1;

    // Update the entity to component index mapping
    mMapEntityToComponentIndex.add(Pair<Entity, uint32>(jointEntity1, index2));

    assert(mMapEntityToComponentIndex[mJointEntities[index1]] == index1);
    assert(mMapEntityToComponentIndex[mJointEntities[index2]] == index2);
    assert(mNbComponents == static_cast<uint32>(mMapEntityToComponentIndex.size()));
}

// Destroy a component at a given index
void JointComponents::destroyComponent(uint32 index) {

    Components::destroyComponent(index);

    assert(mMapEntityToComponentIndex[mJointEntities[index]] == index);

    mMapEntityToComponentIndex.remove(mJointEntities[index]);

    mJointEntities[index].~Entity();
    mBody1Entities[index].~Entity();
    mBody2Entities[index].~Entity();
    mJoints[index] = nullptr;
}
