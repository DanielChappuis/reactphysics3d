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
#include "ProxyShapeComponents.h"
#include "engine/EntityManager.h"
#include "collision/ProxyShape.h"
#include <cassert>
#include <random>

// We want to use the ReactPhysics3D namespace
using namespace reactphysics3d;

// Constructor
ProxyShapeComponents::ProxyShapeComponents(MemoryAllocator& allocator)
                    :Components(allocator, sizeof(Entity) + sizeof(Entity) + sizeof(ProxyShape*) + sizeof(int) + sizeof(AABB) +
                sizeof(Transform) + sizeof(CollisionShape*) + sizeof(decimal) + sizeof(unsigned short) +
                sizeof(unsigned short)) {

    // Allocate memory for the components data
    allocate(INIT_NB_ALLOCATED_COMPONENTS);
}

// Allocate memory for a given number of components
void ProxyShapeComponents::allocate(uint32 nbComponentsToAllocate) {

    assert(nbComponentsToAllocate > mNbAllocatedComponents);

    // Size for the data of a single component (in bytes)
    const size_t totalSizeBytes = nbComponentsToAllocate * mComponentDataSize;

    // Allocate memory
    void* newBuffer = mMemoryAllocator.allocate(totalSizeBytes);
    assert(newBuffer != nullptr);

    // New pointers to components data
    Entity* newProxyShapesEntities = static_cast<Entity*>(newBuffer);
    Entity* newBodiesEntities = reinterpret_cast<Entity*>(newProxyShapesEntities + nbComponentsToAllocate);
    ProxyShape** newProxyShapes = reinterpret_cast<ProxyShape**>(newBodiesEntities + nbComponentsToAllocate);
    int* newBroadPhaseIds = reinterpret_cast<int*>(newProxyShapes + nbComponentsToAllocate);
    AABB* newLocalBounds = reinterpret_cast<AABB*>(newBroadPhaseIds + nbComponentsToAllocate);
    Transform* newLocalToBodyTransforms = reinterpret_cast<Transform*>(newLocalBounds + nbComponentsToAllocate);
    CollisionShape** newCollisionShapes = reinterpret_cast<CollisionShape**>(newLocalToBodyTransforms + nbComponentsToAllocate);
    decimal* newMasses = reinterpret_cast<decimal*>(newCollisionShapes + nbComponentsToAllocate);
    unsigned short* newCollisionCategoryBits = reinterpret_cast<unsigned short*>(newMasses + nbComponentsToAllocate);
    unsigned short* newCollideWithMaskBits = reinterpret_cast<unsigned short*>(newCollisionCategoryBits + nbComponentsToAllocate);

    // If there was already components before
    if (mNbComponents > 0) {

        // Copy component data from the previous buffer to the new one
        memcpy(newProxyShapesEntities, mProxyShapesEntities, mNbComponents * sizeof(Entity));
        memcpy(newBodiesEntities, mBodiesEntities, mNbComponents * sizeof(Entity));
        memcpy(newProxyShapes, mProxyShapes, mNbComponents * sizeof(ProxyShape*));
        memcpy(newBroadPhaseIds, mBroadPhaseIds, mNbComponents * sizeof(int));
        memcpy(newLocalBounds, mLocalBounds, mNbComponents * sizeof(AABB));
        memcpy(newLocalToBodyTransforms, mLocalToBodyTransforms, mNbComponents * sizeof(Transform));
        memcpy(newCollisionShapes, mCollisionShapes, mNbComponents * sizeof(CollisionShape*));
        memcpy(newMasses, mMasses, mNbComponents * sizeof(decimal));
        memcpy(newCollisionCategoryBits, mCollisionCategoryBits, mNbComponents * sizeof(unsigned short));
        memcpy(newCollideWithMaskBits, mCollideWithMaskBits, mNbComponents * sizeof(unsigned short));

        // Deallocate previous memory
        mMemoryAllocator.release(mBuffer, mNbAllocatedComponents * mComponentDataSize);
    }

    mBuffer = newBuffer;
    mProxyShapesEntities = newProxyShapesEntities;
    mBodiesEntities = newBodiesEntities;
    mProxyShapesEntities = newProxyShapesEntities;
    mProxyShapes = newProxyShapes;
    mBroadPhaseIds = newBroadPhaseIds;
    mLocalBounds = newLocalBounds;
    mLocalToBodyTransforms = newLocalToBodyTransforms;
    mCollisionShapes = newCollisionShapes;
    mMasses = newMasses;
    mCollisionCategoryBits = newCollisionCategoryBits;
    mCollideWithMaskBits = newCollideWithMaskBits;

    mNbAllocatedComponents = nbComponentsToAllocate;
}

// Add a component
void ProxyShapeComponents::addComponent(Entity proxyShapeEntity, bool isSleeping, const ProxyShapeComponent& component) {

    // Prepare to add new component (allocate memory if necessary and compute insertion index)
    uint32 index = prepareAddComponent(isSleeping);

    // Insert the new component data
    new (mProxyShapesEntities + index) Entity(proxyShapeEntity);
    new (mBodiesEntities + index) Entity(component.bodyEntity);
    mProxyShapes[index] = component.proxyShape;
    new (mBroadPhaseIds + index) int(component.broadPhaseId);
    new (mLocalBounds + index) AABB(component.localBounds);
    new (mLocalToBodyTransforms + index) Transform(component.localToBodyTransform);
    mCollisionShapes[index] = component.collisionShape;
    new (mMasses + index) decimal(component.mass);
    new (mCollisionCategoryBits + index) unsigned short(component.collisionCategoryBits);
    new (mCollideWithMaskBits + index) unsigned short(component.collideWithMaskBits);

    // Map the entity with the new component lookup index
    mMapEntityToComponentIndex.add(Pair<Entity, uint32>(proxyShapeEntity, index));

    mNbComponents++;

    assert(mDisabledStartIndex <= mNbComponents);
}

// Move a component from a source to a destination index in the components array
// The destination location must contain a constructed object
void ProxyShapeComponents::moveComponentToIndex(uint32 srcIndex, uint32 destIndex) {

    const Entity proxyShapeEntity = mProxyShapesEntities[srcIndex];

    // Copy the data of the source component to the destination location
    new (mProxyShapesEntities + destIndex) Entity(mProxyShapesEntities[srcIndex]);
    new (mBodiesEntities + destIndex) Entity(mBodiesEntities[srcIndex]);
    mProxyShapes[destIndex] = mProxyShapes[srcIndex];
    new (mBroadPhaseIds + destIndex) int(mBroadPhaseIds[srcIndex]);
    new (mLocalBounds + destIndex) AABB(mLocalBounds[srcIndex]);
    new (mLocalToBodyTransforms + destIndex) Transform(mLocalToBodyTransforms[srcIndex]);
    mCollisionShapes[destIndex] = mCollisionShapes[srcIndex];
    new (mMasses + destIndex) decimal(mMasses[srcIndex]);
    new (mCollisionCategoryBits + destIndex) unsigned short(mCollisionCategoryBits[srcIndex]);
    new (mCollideWithMaskBits + destIndex) unsigned short(mCollideWithMaskBits[srcIndex]);

    // Destroy the source component
    destroyComponent(srcIndex);

    assert(!mMapEntityToComponentIndex.containsKey(proxyShapeEntity));

    // Update the entity to component index mapping
    mMapEntityToComponentIndex.add(Pair<Entity, uint32>(proxyShapeEntity, destIndex));

    assert(mMapEntityToComponentIndex[mProxyShapesEntities[destIndex]] == destIndex);
}

// Swap two components in the array
void ProxyShapeComponents::swapComponents(uint32 index1, uint32 index2) {

    // Copy component 1 data
    Entity proxyShapeEntity1(mProxyShapesEntities[index1]);
    Entity bodyEntity1(mBodiesEntities[index1]);
    ProxyShape* proxyShape1 = mProxyShapes[index1];
    int broadPhaseId1 = mBroadPhaseIds[index1];
    AABB localBounds1 = mLocalBounds[index1];
    Transform localToBodyTransform1 = mLocalToBodyTransforms[index1];
    CollisionShape* collisionShape1 = mCollisionShapes[index1];
    decimal mass1 = mMasses[index1];
    unsigned short collisionCategoryBits1 = mCollisionCategoryBits[index1];
    unsigned short collideWithMaskBits1 = mCollideWithMaskBits[index1];

    // Destroy component 1
    destroyComponent(index1);

    moveComponentToIndex(index2, index1);

    // Reconstruct component 1 at component 2 location
    new (mProxyShapesEntities + index2) Entity(proxyShapeEntity1);
    new (mBodiesEntities + index2) Entity(bodyEntity1);
    mProxyShapes[index2] = proxyShape1;
    new (mBroadPhaseIds + index2) int(broadPhaseId1);
    new (mLocalBounds + index2) AABB(localBounds1);
    new (mLocalToBodyTransforms + index2) Transform(localToBodyTransform1);
    mCollisionShapes[index2] = collisionShape1;
    new (mMasses + index2) decimal(mass1);
    new (mCollisionCategoryBits + index2) unsigned short(collisionCategoryBits1);
    new (mCollideWithMaskBits + index2) unsigned short(collideWithMaskBits1);

    // Update the entity to component index mapping
    mMapEntityToComponentIndex.add(Pair<Entity, uint32>(proxyShapeEntity1, index2));

    assert(mMapEntityToComponentIndex[mProxyShapesEntities[index1]] == index1);
    assert(mMapEntityToComponentIndex[mProxyShapesEntities[index2]] == index2);
    assert(mNbComponents == static_cast<uint32>(mMapEntityToComponentIndex.size()));
}

// Destroy a component at a given index
void ProxyShapeComponents::destroyComponent(uint32 index) {

    Components::destroyComponent(index);

    assert(mMapEntityToComponentIndex[mProxyShapesEntities[index]] == index);

    mMapEntityToComponentIndex.remove(mProxyShapesEntities[index]);

    mProxyShapesEntities[index].~Entity();
    mBodiesEntities[index].~Entity();
    mProxyShapes[index] = nullptr;
    mLocalBounds[index].~AABB();
    mLocalToBodyTransforms[index].~Transform();
    mCollisionShapes[index] = nullptr;
}
