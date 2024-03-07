/********************************************************************************
* ReactPhysics3D physics library, http://www.reactphysics3d.com                 *
* Copyright (c) 2010-2024 Daniel Chappuis                                       *
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
#include <reactphysics3d/components/BodyComponents.h>
#include <reactphysics3d/engine/EntityManager.h>
#include <cassert>
#include <random>

// We want to use the ReactPhysics3D namespace
using namespace reactphysics3d;

// Constructor
BodyComponents::BodyComponents(MemoryAllocator& allocator)
                    :Components(allocator, sizeof(Entity) + sizeof(Body*) + sizeof(Array<Entity>) +
                                sizeof(bool) + sizeof(void*) + sizeof(bool), 6 * GLOBAL_ALIGNMENT) {

}

// Allocate memory for a given number of components
void BodyComponents::allocate(uint32 nbComponentsToAllocate) {

    assert(nbComponentsToAllocate > mNbAllocatedComponents);

    // Make sure capacity is an integral multiple of alignment
    nbComponentsToAllocate = std::ceil(nbComponentsToAllocate / float(GLOBAL_ALIGNMENT)) * GLOBAL_ALIGNMENT;

    // Size for the data of a single component (in bytes)
    const size_t totalSizeBytes = nbComponentsToAllocate * mComponentDataSize + mAlignmentMarginSize;

    // Allocate memory
    void* newBuffer = mMemoryAllocator.allocate(totalSizeBytes);
    assert(newBuffer != nullptr);
    assert(reinterpret_cast<uintptr_t>(newBuffer) % GLOBAL_ALIGNMENT == 0);

    // New pointers to components data
    Entity* newBodiesEntities = static_cast<Entity*>(newBuffer);
    assert(reinterpret_cast<uintptr_t>(newBodiesEntities) % GLOBAL_ALIGNMENT == 0);
    Body** newBodies = reinterpret_cast<Body**>(MemoryAllocator::alignAddress(newBodiesEntities + nbComponentsToAllocate, GLOBAL_ALIGNMENT));
    assert(reinterpret_cast<uintptr_t>(newBodies) % GLOBAL_ALIGNMENT == 0);
    Array<Entity>* newColliders = reinterpret_cast<Array<Entity>*>(MemoryAllocator::alignAddress(newBodies + nbComponentsToAllocate, GLOBAL_ALIGNMENT));
    assert(reinterpret_cast<uintptr_t>(newColliders) % GLOBAL_ALIGNMENT == 0);
    bool* newIsActive = reinterpret_cast<bool*>(MemoryAllocator::alignAddress(newColliders + nbComponentsToAllocate, GLOBAL_ALIGNMENT));
    assert(reinterpret_cast<uintptr_t>(newIsActive) % GLOBAL_ALIGNMENT == 0);
    void** newUserData = reinterpret_cast<void**>(MemoryAllocator::alignAddress(newIsActive + nbComponentsToAllocate, GLOBAL_ALIGNMENT));
    assert(reinterpret_cast<uintptr_t>(newUserData) % GLOBAL_ALIGNMENT == 0);
    bool* newHasSimulationCollider = reinterpret_cast<bool*>(MemoryAllocator::alignAddress(newUserData + nbComponentsToAllocate, GLOBAL_ALIGNMENT));
    assert(reinterpret_cast<uintptr_t>(newHasSimulationCollider) % GLOBAL_ALIGNMENT == 0);
    assert(reinterpret_cast<uintptr_t>(newHasSimulationCollider + nbComponentsToAllocate) <= reinterpret_cast<uintptr_t>(newBuffer) + totalSizeBytes);

    // If there was already components before
    if (mNbComponents > 0) {

        // Copy component data from the previous buffer to the new one
        memcpy(newBodiesEntities, mBodiesEntities, mNbComponents * sizeof(Entity));
        memcpy(newBodies, mBodies, mNbComponents * sizeof(Body*));
        memcpy(newColliders, mColliders, mNbComponents * sizeof(Array<Entity>));
        memcpy(newIsActive, mIsActive, mNbComponents * sizeof(bool));
        memcpy(newUserData, mUserData, mNbComponents * sizeof(void*));
        memcpy(newHasSimulationCollider, mHasSimulationCollider, mNbComponents * sizeof(bool));

        // Deallocate previous memory
        mMemoryAllocator.release(mBuffer, mNbAllocatedComponents * mComponentDataSize);
    }

    mBuffer = newBuffer;
    mBodiesEntities = newBodiesEntities;
    mBodies = newBodies;
    mColliders = newColliders;
    mIsActive = newIsActive;
    mUserData = newUserData;
    mHasSimulationCollider = newHasSimulationCollider;
    mNbAllocatedComponents = nbComponentsToAllocate;
}

// Add a component
void BodyComponents::addComponent(Entity bodyEntity, bool isDisabled, const BodyComponent& component) {

    // Prepare to add new component (allocate memory if necessary and compute insertion index)
    uint32 index = prepareAddComponent(isDisabled);

    // Insert the new component data
    new (mBodiesEntities + index) Entity(bodyEntity);
    mBodies[index] = component.body;
    new (mColliders + index) Array<Entity>(mMemoryAllocator);
    mIsActive[index] = true;
    mUserData[index] = nullptr;
    mHasSimulationCollider[index] = false;

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
    new (mColliders + destIndex) Array<Entity>(mColliders[srcIndex]);
    mIsActive[destIndex] = mIsActive[srcIndex];
    mUserData[destIndex] = mUserData[srcIndex];
    mHasSimulationCollider[destIndex] = mHasSimulationCollider[srcIndex];

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
    Array<Entity> colliders1(mColliders[index1]);
    bool isActive1 = mIsActive[index1];
    void* userData1 = mUserData[index1];
    bool hasSimulationCollider = mHasSimulationCollider[index1];

    // Destroy component 1
    destroyComponent(index1);

    moveComponentToIndex(index2, index1);

    // Reconstruct component 1 at component 2 location
    new (mBodiesEntities + index2) Entity(entity1);
    new (mColliders + index2) Array<Entity>(colliders1);
    mBodies[index2] = body1;
    mIsActive[index2] = isActive1;
    mUserData[index2] = userData1;
    mHasSimulationCollider[index2] = hasSimulationCollider;

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
    mColliders[index].~Array<Entity>();
    mUserData[index] = nullptr;
}
