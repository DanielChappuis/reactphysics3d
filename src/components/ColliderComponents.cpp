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
#include <reactphysics3d/components/ColliderComponents.h>
#include <reactphysics3d/engine/EntityManager.h>
#include <reactphysics3d/collision/Collider.h>
#include <cassert>
#include <random>

// We want to use the ReactPhysics3D namespace
using namespace reactphysics3d;

// Constructor
ColliderComponents::ColliderComponents(MemoryAllocator& allocator)
                    :Components(allocator, sizeof(Entity) + sizeof(Entity) + sizeof(Collider*) + sizeof(int32) +
                sizeof(Transform) + sizeof(CollisionShape*) + sizeof(unsigned short) +
                sizeof(unsigned short) + sizeof(Transform) + sizeof(Array<uint64>) + sizeof(bool) +
                sizeof(bool) + sizeof(bool) + sizeof(bool) + sizeof(Material), 15 * GLOBAL_ALIGNMENT) {

}

// Allocate memory for a given number of components
void ColliderComponents::allocate(uint32 nbComponentsToAllocate) {

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
    Entity* newCollidersEntities = static_cast<Entity*>(newBuffer);
    Entity* newBodiesEntities = reinterpret_cast<Entity*>(MemoryAllocator::alignAddress(newCollidersEntities + nbComponentsToAllocate, GLOBAL_ALIGNMENT));
    assert(reinterpret_cast<uintptr_t>(newBodiesEntities) % GLOBAL_ALIGNMENT == 0);
    Collider** newColliders = reinterpret_cast<Collider**>(MemoryAllocator::alignAddress(newBodiesEntities + nbComponentsToAllocate, GLOBAL_ALIGNMENT));
    assert(reinterpret_cast<uintptr_t>(newColliders) % GLOBAL_ALIGNMENT == 0);
    int32* newBroadPhaseIds = reinterpret_cast<int32*>(MemoryAllocator::alignAddress(newColliders + nbComponentsToAllocate, GLOBAL_ALIGNMENT));
    assert(reinterpret_cast<uintptr_t>(newBroadPhaseIds) % GLOBAL_ALIGNMENT == 0);
    Transform* newLocalToBodyTransforms = reinterpret_cast<Transform*>(MemoryAllocator::alignAddress(newBroadPhaseIds + nbComponentsToAllocate, GLOBAL_ALIGNMENT));
    assert(reinterpret_cast<uintptr_t>(newLocalToBodyTransforms) % GLOBAL_ALIGNMENT == 0);
    CollisionShape** newCollisionShapes = reinterpret_cast<CollisionShape**>(MemoryAllocator::alignAddress(newLocalToBodyTransforms + nbComponentsToAllocate, GLOBAL_ALIGNMENT));
    assert(reinterpret_cast<uintptr_t>(newCollisionShapes) % GLOBAL_ALIGNMENT == 0);
    unsigned short* newCollisionCategoryBits = reinterpret_cast<unsigned short*>(MemoryAllocator::alignAddress(newCollisionShapes + nbComponentsToAllocate, GLOBAL_ALIGNMENT));
    assert(reinterpret_cast<uintptr_t>(newCollisionCategoryBits) % GLOBAL_ALIGNMENT == 0);
    unsigned short* newCollideWithMaskBits = reinterpret_cast<unsigned short*>(MemoryAllocator::alignAddress(newCollisionCategoryBits + nbComponentsToAllocate, GLOBAL_ALIGNMENT));
    assert(reinterpret_cast<uintptr_t>(newCollideWithMaskBits) % GLOBAL_ALIGNMENT == 0);
    Transform* newLocalToWorldTransforms = reinterpret_cast<Transform*>(MemoryAllocator::alignAddress(newCollideWithMaskBits + nbComponentsToAllocate, GLOBAL_ALIGNMENT));
    assert(reinterpret_cast<uintptr_t>(newLocalToWorldTransforms) % GLOBAL_ALIGNMENT == 0);
    Array<uint64>* newOverlappingPairs = reinterpret_cast<Array<uint64>*>(MemoryAllocator::alignAddress(newLocalToWorldTransforms + nbComponentsToAllocate, GLOBAL_ALIGNMENT));
    assert(reinterpret_cast<uintptr_t>(newOverlappingPairs) % GLOBAL_ALIGNMENT == 0);
    bool* hasCollisionShapeChangedSize = reinterpret_cast<bool*>(MemoryAllocator::alignAddress(newOverlappingPairs + nbComponentsToAllocate, GLOBAL_ALIGNMENT));
    assert(reinterpret_cast<uintptr_t>(hasCollisionShapeChangedSize) % GLOBAL_ALIGNMENT == 0);
    bool* isTrigger = reinterpret_cast<bool*>(MemoryAllocator::alignAddress(hasCollisionShapeChangedSize + nbComponentsToAllocate, GLOBAL_ALIGNMENT));
    assert(reinterpret_cast<uintptr_t>(isTrigger) % GLOBAL_ALIGNMENT == 0);
    bool* isSimulationCollider = reinterpret_cast<bool*>(MemoryAllocator::alignAddress(isTrigger + nbComponentsToAllocate, GLOBAL_ALIGNMENT));
    assert(reinterpret_cast<uintptr_t>(isSimulationCollider) % GLOBAL_ALIGNMENT == 0);
    bool* isWorldQueryCollider = reinterpret_cast<bool*>(MemoryAllocator::alignAddress(isSimulationCollider + nbComponentsToAllocate, GLOBAL_ALIGNMENT));
    assert(reinterpret_cast<uintptr_t>(isWorldQueryCollider) % GLOBAL_ALIGNMENT == 0);
    Material* materials = reinterpret_cast<Material*>(MemoryAllocator::alignAddress(isWorldQueryCollider + nbComponentsToAllocate, GLOBAL_ALIGNMENT));
    assert(reinterpret_cast<uintptr_t>(materials) % GLOBAL_ALIGNMENT == 0);
    assert(reinterpret_cast<uintptr_t>(materials + nbComponentsToAllocate) <= reinterpret_cast<uintptr_t>(newBuffer) + totalSizeBytes);

    // If there was already components before
    if (mNbComponents > 0) {

        // Copy component data from the previous buffer to the new one
        memcpy(newCollidersEntities, mCollidersEntities, mNbComponents * sizeof(Entity));
        memcpy(newBodiesEntities, mBodiesEntities, mNbComponents * sizeof(Entity));
        memcpy(newColliders, mColliders, mNbComponents * sizeof(Collider*));
        memcpy(newBroadPhaseIds, mBroadPhaseIds, mNbComponents * sizeof(int32));
        memcpy(newLocalToBodyTransforms, mLocalToBodyTransforms, mNbComponents * sizeof(Transform));
        memcpy(newCollisionShapes, mCollisionShapes, mNbComponents * sizeof(CollisionShape*));
        memcpy(newCollisionCategoryBits, mCollisionCategoryBits, mNbComponents * sizeof(unsigned short));
        memcpy(newCollideWithMaskBits, mCollideWithMaskBits, mNbComponents * sizeof(unsigned short));
        memcpy(newLocalToWorldTransforms, mLocalToWorldTransforms, mNbComponents * sizeof(Transform));
        memcpy(newOverlappingPairs, mOverlappingPairs, mNbComponents * sizeof(Array<uint64>));
        memcpy(hasCollisionShapeChangedSize, mHasCollisionShapeChangedSize, mNbComponents * sizeof(bool));
        memcpy(isTrigger, mIsTrigger, mNbComponents * sizeof(bool));
        memcpy(isSimulationCollider, mIsSimulationCollider, mNbComponents * sizeof(bool));
        memcpy(isWorldQueryCollider, mIsWorldQueryCollider, mNbComponents * sizeof(bool));
        memcpy(materials, mMaterials, mNbComponents * sizeof(Material));

        // Deallocate previous memory
        mMemoryAllocator.release(mBuffer, mNbAllocatedComponents * mComponentDataSize);
    }

    mBuffer = newBuffer;
    mBodiesEntities = newBodiesEntities;
    mCollidersEntities = newCollidersEntities;
    mColliders = newColliders;
    mBroadPhaseIds = newBroadPhaseIds;
    mLocalToBodyTransforms = newLocalToBodyTransforms;
    mCollisionShapes = newCollisionShapes;
    mCollisionCategoryBits = newCollisionCategoryBits;
    mCollideWithMaskBits = newCollideWithMaskBits;
    mLocalToWorldTransforms = newLocalToWorldTransforms;
    mOverlappingPairs = newOverlappingPairs;
    mHasCollisionShapeChangedSize = hasCollisionShapeChangedSize;
    mIsTrigger = isTrigger;
    mIsSimulationCollider = isSimulationCollider;
    mIsWorldQueryCollider = isWorldQueryCollider;
    mMaterials = materials;

    mNbAllocatedComponents = nbComponentsToAllocate;
}

// Add a component
void ColliderComponents::addComponent(Entity colliderEntity, bool isDisabled, const ColliderComponent& component) {

    // Prepare to add new component (allocate memory if necessary and compute insertion index)
    uint32 index = prepareAddComponent(isDisabled);

    // Insert the new component data
    new (mCollidersEntities + index) Entity(colliderEntity);
    new (mBodiesEntities + index) Entity(component.bodyEntity);
    mColliders[index] = component.collider;
    new (mBroadPhaseIds + index) int32(-1);
    new (mLocalToBodyTransforms + index) Transform(component.localToBodyTransform);
    mCollisionShapes[index] = component.collisionShape;
    new (mCollisionCategoryBits + index) unsigned short(component.collisionCategoryBits);
    new (mCollideWithMaskBits + index) unsigned short(component.collideWithMaskBits);
    new (mLocalToWorldTransforms + index) Transform(component.localToWorldTransform);
    new (mOverlappingPairs + index) Array<uint64>(mMemoryAllocator);
    mHasCollisionShapeChangedSize[index] = false;
    mIsTrigger[index] = false;
    mIsSimulationCollider[index] = true;
    mIsWorldQueryCollider[index] = true;
    mMaterials[index] = component.material;

    // Map the entity with the new component lookup index
    mMapEntityToComponentIndex.add(Pair<Entity, uint32>(colliderEntity, index));

    mNbComponents++;

    assert(mDisabledStartIndex <= mNbComponents);
}

// Move a component from a source to a destination index in the components array
// The destination location must contain a constructed object
void ColliderComponents::moveComponentToIndex(uint32 srcIndex, uint32 destIndex) {

    const Entity colliderEntity = mCollidersEntities[srcIndex];

    // Copy the data of the source component to the destination location
    new (mCollidersEntities + destIndex) Entity(mCollidersEntities[srcIndex]);
    new (mBodiesEntities + destIndex) Entity(mBodiesEntities[srcIndex]);
    mColliders[destIndex] = mColliders[srcIndex];
    new (mBroadPhaseIds + destIndex) int32(mBroadPhaseIds[srcIndex]);
    new (mLocalToBodyTransforms + destIndex) Transform(mLocalToBodyTransforms[srcIndex]);
    mCollisionShapes[destIndex] = mCollisionShapes[srcIndex];
    new (mCollisionCategoryBits + destIndex) unsigned short(mCollisionCategoryBits[srcIndex]);
    new (mCollideWithMaskBits + destIndex) unsigned short(mCollideWithMaskBits[srcIndex]);
    new (mLocalToWorldTransforms + destIndex) Transform(mLocalToWorldTransforms[srcIndex]);
    new (mOverlappingPairs + destIndex) Array<uint64>(mOverlappingPairs[srcIndex]);
    mHasCollisionShapeChangedSize[destIndex] = mHasCollisionShapeChangedSize[srcIndex];
    mIsTrigger[destIndex] = mIsTrigger[srcIndex];
    mIsSimulationCollider[destIndex] = mIsSimulationCollider[srcIndex];
    mIsWorldQueryCollider[destIndex] = mIsWorldQueryCollider[srcIndex];
    mMaterials[destIndex] = mMaterials[srcIndex];

    // Destroy the source component
    destroyComponent(srcIndex);

    assert(!mMapEntityToComponentIndex.containsKey(colliderEntity));

    // Update the entity to component index mapping
    mMapEntityToComponentIndex.add(Pair<Entity, uint32>(colliderEntity, destIndex));

    assert(mMapEntityToComponentIndex[mCollidersEntities[destIndex]] == destIndex);
}

// Swap two components in the array
void ColliderComponents::swapComponents(uint32 index1, uint32 index2) {

    // Copy component 1 data
    Entity colliderEntity1(mCollidersEntities[index1]);
    Entity bodyEntity1(mBodiesEntities[index1]);
    Collider* collider1 = mColliders[index1];
    int32 broadPhaseId1 = mBroadPhaseIds[index1];
    Transform localToBodyTransform1 = mLocalToBodyTransforms[index1];
    CollisionShape* collisionShape1 = mCollisionShapes[index1];
    unsigned short collisionCategoryBits1 = mCollisionCategoryBits[index1];
    unsigned short collideWithMaskBits1 = mCollideWithMaskBits[index1];
    Transform localToWorldTransform1 = mLocalToWorldTransforms[index1];
    Array<uint64> overlappingPairs = mOverlappingPairs[index1];
    bool hasCollisionShapeChangedSize = mHasCollisionShapeChangedSize[index1];
    bool isTrigger = mIsTrigger[index1];
    bool isSimulationCollider = mIsSimulationCollider[index1];
    bool isWorldQueryCollider = mIsWorldQueryCollider[index1];
    Material material = mMaterials[index1];

    // Destroy component 1
    destroyComponent(index1);

    moveComponentToIndex(index2, index1);

    // Reconstruct component 1 at component 2 location
    new (mCollidersEntities + index2) Entity(colliderEntity1);
    new (mBodiesEntities + index2) Entity(bodyEntity1);
    mColliders[index2] = collider1;
    new (mBroadPhaseIds + index2) int32(broadPhaseId1);
    new (mLocalToBodyTransforms + index2) Transform(localToBodyTransform1);
    mCollisionShapes[index2] = collisionShape1;
    new (mCollisionCategoryBits + index2) unsigned short(collisionCategoryBits1);
    new (mCollideWithMaskBits + index2) unsigned short(collideWithMaskBits1);
    new (mLocalToWorldTransforms + index2) Transform(localToWorldTransform1);
    new (mOverlappingPairs + index2) Array<uint64>(overlappingPairs);
    mHasCollisionShapeChangedSize[index2] = hasCollisionShapeChangedSize;
    mIsTrigger[index2] = isTrigger;
    mIsSimulationCollider[index2] = isSimulationCollider;
    mIsWorldQueryCollider[index2] = isWorldQueryCollider;
    mMaterials[index2] = material;

    // Update the entity to component index mapping
    mMapEntityToComponentIndex.add(Pair<Entity, uint32>(colliderEntity1, index2));

    assert(mMapEntityToComponentIndex[mCollidersEntities[index1]] == index1);
    assert(mMapEntityToComponentIndex[mCollidersEntities[index2]] == index2);
    assert(mNbComponents == static_cast<uint32>(mMapEntityToComponentIndex.size()));
}

// Destroy a component at a given index
void ColliderComponents::destroyComponent(uint32 index) {

    Components::destroyComponent(index);

    assert(mMapEntityToComponentIndex[mCollidersEntities[index]] == index);

    mMapEntityToComponentIndex.remove(mCollidersEntities[index]);

    mCollidersEntities[index].~Entity();
    mBodiesEntities[index].~Entity();
    mColliders[index] = nullptr;
    mLocalToBodyTransforms[index].~Transform();
    mCollisionShapes[index] = nullptr;
    mLocalToWorldTransforms[index].~Transform();
    mOverlappingPairs[index].~Array<uint64>();
    mMaterials[index].~Material();
}
