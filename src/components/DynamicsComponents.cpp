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
                    :Components(allocator, sizeof(Entity) + sizeof(Vector3) + sizeof(Vector3) + sizeof(Vector3) + sizeof(Vector3) +
                                           sizeof(Vector3) + sizeof(Vector3) + sizeof(Vector3) + sizeof(Vector3) + sizeof(decimal) +
                                           sizeof(decimal) + sizeof(decimal) + sizeof(decimal) + sizeof(bool)) {

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
    Entity* newBodies = static_cast<Entity*>(newBuffer);
    Vector3* newLinearVelocities = reinterpret_cast<Vector3*>(newBodies + nbComponentsToAllocate);
    Vector3* newAngularVelocities = reinterpret_cast<Vector3*>(newLinearVelocities + nbComponentsToAllocate);
    Vector3* newConstrainedLinearVelocities = reinterpret_cast<Vector3*>(newAngularVelocities + nbComponentsToAllocate);
    Vector3* newConstrainedAngularVelocities = reinterpret_cast<Vector3*>(newConstrainedLinearVelocities + nbComponentsToAllocate);
    Vector3* newSplitLinearVelocities = reinterpret_cast<Vector3*>(newConstrainedAngularVelocities + nbComponentsToAllocate);
    Vector3* newSplitAngularVelocities = reinterpret_cast<Vector3*>(newSplitLinearVelocities + nbComponentsToAllocate);
    Vector3* newExternalForces = reinterpret_cast<Vector3*>(newSplitAngularVelocities + nbComponentsToAllocate);
    Vector3* newExternalTorques = reinterpret_cast<Vector3*>(newExternalForces + nbComponentsToAllocate);
    decimal* newLinearDampings = reinterpret_cast<decimal*>(newExternalTorques + nbComponentsToAllocate);
    decimal* newAngularDampings = reinterpret_cast<decimal*>(newLinearDampings + nbComponentsToAllocate);
    decimal* newInitMasses = reinterpret_cast<decimal*>(newAngularDampings + nbComponentsToAllocate);
    decimal* newInverseMasses = reinterpret_cast<decimal*>(newInitMasses + nbComponentsToAllocate);
    bool* newIsAlreadyInIsland = reinterpret_cast<bool*>(newInverseMasses + nbComponentsToAllocate);

    // If there was already components before
    if (mNbComponents > 0) {

        // Copy component data from the previous buffer to the new one
        memcpy(newBodies, mBodies, mNbComponents * sizeof(Entity));
        memcpy(newLinearVelocities, mLinearVelocities, mNbComponents * sizeof(Vector3));
        memcpy(newAngularVelocities, mAngularVelocities, mNbComponents * sizeof(Vector3));
        memcpy(newConstrainedLinearVelocities, mConstrainedLinearVelocities, mNbComponents * sizeof(Vector3));
        memcpy(newConstrainedAngularVelocities, mConstrainedAngularVelocities, mNbComponents * sizeof(Vector3));
        memcpy(newSplitLinearVelocities, mSplitLinearVelocities, mNbComponents * sizeof(Vector3));
        memcpy(newSplitAngularVelocities, mSplitAngularVelocities, mNbComponents * sizeof(Vector3));
        memcpy(newExternalForces, mExternalForces, mNbComponents * sizeof(Vector3));
        memcpy(newExternalTorques, mExternalTorques, mNbComponents * sizeof(Vector3));
        memcpy(newLinearDampings, mLinearDampings, mNbComponents * sizeof(decimal));
        memcpy(newAngularDampings, mAngularDampings, mNbComponents * sizeof(decimal));
        memcpy(newInitMasses, mInitMasses, mNbComponents * sizeof(decimal));
        memcpy(newInverseMasses, mInverseMasses, mNbComponents * sizeof(decimal));
        memcpy(newIsAlreadyInIsland, mIsAlreadyInIsland, mNbComponents * sizeof(bool));

        // Deallocate previous memory
        mMemoryAllocator.release(mBuffer, mNbAllocatedComponents * mComponentDataSize);
    }

    mBuffer = newBuffer;
    mBodies = newBodies;
    mLinearVelocities = newLinearVelocities;
    mAngularVelocities = newAngularVelocities;
    mConstrainedLinearVelocities = newConstrainedLinearVelocities;
    mConstrainedAngularVelocities = newConstrainedAngularVelocities;
    mSplitLinearVelocities = newSplitLinearVelocities;
    mSplitAngularVelocities = newSplitAngularVelocities;
    mExternalForces = newExternalForces;
    mExternalTorques = newExternalTorques;
    mLinearDampings = newLinearDampings;
    mAngularDampings = newAngularDampings;
    mInitMasses = newInitMasses;
    mInverseMasses = newInverseMasses;
    mIsAlreadyInIsland = newIsAlreadyInIsland;
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
    new (mConstrainedLinearVelocities + index) Vector3(0, 0, 0);
    new (mConstrainedAngularVelocities + index) Vector3(0, 0, 0);
    new (mSplitLinearVelocities + index) Vector3(0, 0, 0);
    new (mSplitAngularVelocities + index) Vector3(0, 0, 0);
    new (mExternalForces + index) Vector3(0, 0, 0);
    new (mExternalTorques + index) Vector3(0, 0, 0);
    mLinearDampings[index] = decimal(0.0);
    mAngularDampings[index] = decimal(0.0);
    mInitMasses[index] = decimal(1.0);
    mInverseMasses[index] = decimal(1.0);
    mIsAlreadyInIsland[index] = false;

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
    new (mConstrainedLinearVelocities + destIndex) Vector3(mConstrainedLinearVelocities[srcIndex]);
    new (mConstrainedAngularVelocities + destIndex) Vector3(mConstrainedAngularVelocities[srcIndex]);
    new (mSplitLinearVelocities + destIndex) Vector3(mSplitLinearVelocities[srcIndex]);
    new (mSplitAngularVelocities + destIndex) Vector3(mSplitAngularVelocities[srcIndex]);
    new (mExternalForces + destIndex) Vector3(mExternalForces[srcIndex]);
    new (mExternalTorques + destIndex) Vector3(mExternalTorques[srcIndex]);
    mLinearDampings[destIndex] = mLinearDampings[srcIndex];
    mAngularDampings[destIndex] = mAngularDampings[srcIndex];
    mInitMasses[destIndex] = mInitMasses[srcIndex];
    mInverseMasses[destIndex] = mInverseMasses[srcIndex];
    mIsAlreadyInIsland[destIndex] = mIsAlreadyInIsland[srcIndex];

    // Destroy the source component
    destroyComponent(srcIndex);

    assert(!mMapEntityToComponentIndex.containsKey(entity));

    // Update the entity to component index mapping
    mMapEntityToComponentIndex.add(Pair<Entity, uint32>(entity, destIndex));

    assert(mMapEntityToComponentIndex[mBodies[destIndex]] == destIndex);
}

// Swap two components in the array
void DynamicsComponents::swapComponents(uint32 index1, uint32 index2) {

    assert(mMapEntityToComponentIndex[mBodies[index1]] == index1);
    assert(mMapEntityToComponentIndex[mBodies[index2]] == index2);

    // Copy component 1 data
    Entity entity1(mBodies[index1]);
    Vector3 linearVelocity1(mLinearVelocities[index1]);
    Vector3 angularVelocity1(mAngularVelocities[index1]);
    Vector3 constrainedLinearVelocity1(mConstrainedLinearVelocities[index1]);
    Vector3 constrainedAngularVelocity1(mConstrainedAngularVelocities[index1]);
    Vector3 splitLinearVelocity1(mSplitLinearVelocities[index1]);
    Vector3 splitAngularVelocity1(mSplitAngularVelocities[index1]);
    Vector3 externalForce1(mExternalForces[index1]);
    Vector3 externalTorque1(mExternalTorques[index1]);
    decimal linearDamping1 = mLinearDampings[index1];
    decimal angularDamping1 = mAngularDampings[index1];
    decimal initMass1 = mInitMasses[index1];
    decimal inverseMass1 = mInverseMasses[index1];
    bool isAlreadyInIsland1 = mIsAlreadyInIsland[index1];

    // Destroy component 1
    destroyComponent(index1);

    moveComponentToIndex(index2, index1);

    // Reconstruct component 1 at component 2 location
    new (mBodies + index2) Entity(entity1);
    new (mLinearVelocities + index2) Vector3(linearVelocity1);
    new (mAngularVelocities + index2) Vector3(angularVelocity1);
    new (mConstrainedLinearVelocities + index2) Vector3(constrainedLinearVelocity1);
    new (mConstrainedAngularVelocities + index2) Vector3(constrainedAngularVelocity1);
    new (mSplitLinearVelocities + index2) Vector3(splitLinearVelocity1);
    new (mSplitAngularVelocities + index2) Vector3(splitAngularVelocity1);
    new (mExternalForces + index2) Vector3(externalForce1);
    new (mExternalTorques + index2) Vector3(externalTorque1);
    mLinearDampings[index2] = linearDamping1;
    mAngularDampings[index2] = angularDamping1;
    mInitMasses[index2] = initMass1;
    mInverseMasses[index2] = inverseMass1;
    mIsAlreadyInIsland[index2] = isAlreadyInIsland1;

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
    mConstrainedLinearVelocities[index].~Vector3();
    mConstrainedAngularVelocities[index].~Vector3();
    mSplitLinearVelocities[index].~Vector3();
    mSplitAngularVelocities[index].~Vector3();
    mExternalForces[index].~Vector3();
    mExternalTorques[index].~Vector3();
}
