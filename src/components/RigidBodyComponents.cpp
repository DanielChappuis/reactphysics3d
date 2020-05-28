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
#include <reactphysics3d/components/RigidBodyComponents.h>
#include <reactphysics3d/engine/EntityManager.h>
#include <reactphysics3d/body/RigidBody.h>
#include <cassert>
#include <random>

// We want to use the ReactPhysics3D namespace
using namespace reactphysics3d;

// Constructor
RigidBodyComponents::RigidBodyComponents(MemoryAllocator& allocator)
                    :Components(allocator, sizeof(Entity) + sizeof(RigidBody*) +
                                sizeof(bool) + sizeof(bool) + sizeof(decimal) + sizeof(BodyType) +
                                sizeof(Vector3) + sizeof(Vector3) + sizeof(Vector3) +
                                sizeof(Vector3) + sizeof(decimal) + sizeof(decimal) +
                                sizeof(decimal) + sizeof(decimal) + sizeof(Vector3) +
                                sizeof(Vector3) + sizeof(Vector3) + sizeof(Vector3) +
                                sizeof(Vector3) + sizeof(Vector3) + sizeof(Vector3) +
                                sizeof(Quaternion) + sizeof(Vector3) + sizeof(Vector3) +
                                sizeof(bool) + sizeof(bool) + sizeof(List<Entity>)) {

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
    Vector3* newExternalForces = reinterpret_cast<Vector3*>(newAngularVelocities + nbComponentsToAllocate);
    Vector3* newExternalTorques = reinterpret_cast<Vector3*>(newExternalForces + nbComponentsToAllocate);
    decimal* newLinearDampings = reinterpret_cast<decimal*>(newExternalTorques + nbComponentsToAllocate);
    decimal* newAngularDampings = reinterpret_cast<decimal*>(newLinearDampings + nbComponentsToAllocate);
    decimal* newMasses = reinterpret_cast<decimal*>(newAngularDampings + nbComponentsToAllocate);
    decimal* newInverseMasses = reinterpret_cast<decimal*>(newMasses + nbComponentsToAllocate);
    Vector3* newInertiaTensorLocal = reinterpret_cast<Vector3*>(newInverseMasses + nbComponentsToAllocate);
    Vector3* newInertiaTensorLocalInverses = reinterpret_cast<Vector3*>(newInertiaTensorLocal + nbComponentsToAllocate);
    Vector3* newConstrainedLinearVelocities = reinterpret_cast<Vector3*>(newInertiaTensorLocalInverses + nbComponentsToAllocate);
    Vector3* newConstrainedAngularVelocities = reinterpret_cast<Vector3*>(newConstrainedLinearVelocities + nbComponentsToAllocate);
    Vector3* newSplitLinearVelocities = reinterpret_cast<Vector3*>(newConstrainedAngularVelocities + nbComponentsToAllocate);
    Vector3* newSplitAngularVelocities = reinterpret_cast<Vector3*>(newSplitLinearVelocities + nbComponentsToAllocate);
    Vector3* newConstrainedPositions = reinterpret_cast<Vector3*>(newSplitAngularVelocities + nbComponentsToAllocate);
    Quaternion* newConstrainedOrientations = reinterpret_cast<Quaternion*>(newConstrainedPositions + nbComponentsToAllocate);
    Vector3* newCentersOfMassLocal = reinterpret_cast<Vector3*>(newConstrainedOrientations + nbComponentsToAllocate);
    Vector3* newCentersOfMassWorld = reinterpret_cast<Vector3*>(newCentersOfMassLocal + nbComponentsToAllocate);
    bool* newIsGravityEnabled = reinterpret_cast<bool*>(newCentersOfMassWorld + nbComponentsToAllocate);
    bool* newIsAlreadyInIsland = reinterpret_cast<bool*>(newIsGravityEnabled + nbComponentsToAllocate);
    List<Entity>* newJoints = reinterpret_cast<List<Entity>*>(newIsAlreadyInIsland + nbComponentsToAllocate);

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
        memcpy(newExternalForces, mExternalForces, mNbComponents * sizeof(Vector3));
        memcpy(newExternalTorques, mExternalTorques, mNbComponents * sizeof(Vector3));
        memcpy(newLinearDampings, mLinearDampings, mNbComponents * sizeof(decimal));
        memcpy(newAngularDampings, mAngularDampings, mNbComponents * sizeof(decimal));
        memcpy(newMasses, mMasses, mNbComponents * sizeof(decimal));
        memcpy(newInverseMasses, mInverseMasses, mNbComponents * sizeof(decimal));
        memcpy(newInertiaTensorLocal, mLocalInertiaTensors, mNbComponents * sizeof(Vector3));
        memcpy(newInertiaTensorLocalInverses, mInverseInertiaTensorsLocal, mNbComponents * sizeof(Vector3));
        memcpy(newConstrainedLinearVelocities, mConstrainedLinearVelocities, mNbComponents * sizeof(Vector3));
        memcpy(newConstrainedAngularVelocities, mConstrainedAngularVelocities, mNbComponents * sizeof(Vector3));
        memcpy(newSplitLinearVelocities, mSplitLinearVelocities, mNbComponents * sizeof(Vector3));
        memcpy(newSplitAngularVelocities, mSplitAngularVelocities, mNbComponents * sizeof(Vector3));
        memcpy(newConstrainedPositions, mConstrainedPositions, mNbComponents * sizeof(Vector3));
        memcpy(newConstrainedOrientations, mConstrainedOrientations, mNbComponents * sizeof(Quaternion));
        memcpy(newCentersOfMassLocal, mCentersOfMassLocal, mNbComponents * sizeof(Vector3));
        memcpy(newCentersOfMassWorld, mCentersOfMassWorld, mNbComponents * sizeof(Vector3));
        memcpy(newIsGravityEnabled, mIsGravityEnabled, mNbComponents * sizeof(bool));
        memcpy(newIsAlreadyInIsland, mIsAlreadyInIsland, mNbComponents * sizeof(bool));
        memcpy(newJoints, mJoints, mNbComponents * sizeof(List<Entity>));

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
    mExternalForces = newExternalForces;
    mExternalTorques = newExternalTorques;
    mLinearDampings = newLinearDampings;
    mAngularDampings = newAngularDampings;
    mMasses = newMasses;
    mInverseMasses = newInverseMasses;
    mLocalInertiaTensors = newInertiaTensorLocal;
    mInverseInertiaTensorsLocal = newInertiaTensorLocalInverses;
    mConstrainedLinearVelocities = newConstrainedLinearVelocities;
    mConstrainedAngularVelocities = newConstrainedAngularVelocities;
    mSplitLinearVelocities = newSplitLinearVelocities;
    mSplitAngularVelocities = newSplitAngularVelocities;
    mConstrainedPositions = newConstrainedPositions;
    mConstrainedOrientations = newConstrainedOrientations;
    mCentersOfMassLocal = newCentersOfMassLocal;
    mCentersOfMassWorld = newCentersOfMassWorld;
    mIsGravityEnabled = newIsGravityEnabled;
    mIsAlreadyInIsland = newIsAlreadyInIsland;
    mJoints = newJoints;
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
    new (mExternalForces + index) Vector3(0, 0, 0);
    new (mExternalTorques + index) Vector3(0, 0, 0);
    mLinearDampings[index] = decimal(0.0);
    mAngularDampings[index] = decimal(0.0);
    mMasses[index] = decimal(1.0);
    mInverseMasses[index] = decimal(1.0);
    new (mLocalInertiaTensors + index) Vector3(1.0, 1.0, 1.0);
    new (mInverseInertiaTensorsLocal + index) Vector3(1.0, 1.0, 1.0);
    new (mConstrainedLinearVelocities + index) Vector3(0, 0, 0);
    new (mConstrainedAngularVelocities + index) Vector3(0, 0, 0);
    new (mSplitLinearVelocities + index) Vector3(0, 0, 0);
    new (mSplitAngularVelocities + index) Vector3(0, 0, 0);
    new (mConstrainedPositions + index) Vector3(0, 0, 0);
    new (mConstrainedOrientations + index) Quaternion(0, 0, 0, 1);
    new (mCentersOfMassLocal + index) Vector3(0, 0, 0);
    new (mCentersOfMassWorld + index) Vector3(component.worldPosition);
    mIsGravityEnabled[index] = true;
    mIsAlreadyInIsland[index] = false;
    new (mJoints + index) List<Entity>(mMemoryAllocator);

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
    new (mExternalForces + destIndex) Vector3(mExternalForces[srcIndex]);
    new (mExternalTorques + destIndex) Vector3(mExternalTorques[srcIndex]);
    mLinearDampings[destIndex] = mLinearDampings[srcIndex];
    mAngularDampings[destIndex] = mAngularDampings[srcIndex];
    mMasses[destIndex] = mMasses[srcIndex];
    mInverseMasses[destIndex] = mInverseMasses[srcIndex];
    new (mLocalInertiaTensors + destIndex) Vector3(mLocalInertiaTensors[srcIndex]);
    new (mInverseInertiaTensorsLocal + destIndex) Vector3(mInverseInertiaTensorsLocal[srcIndex]);
    new (mConstrainedLinearVelocities + destIndex) Vector3(mConstrainedLinearVelocities[srcIndex]);
    new (mConstrainedAngularVelocities + destIndex) Vector3(mConstrainedAngularVelocities[srcIndex]);
    new (mSplitLinearVelocities + destIndex) Vector3(mSplitLinearVelocities[srcIndex]);
    new (mSplitAngularVelocities + destIndex) Vector3(mSplitAngularVelocities[srcIndex]);
    new (mConstrainedPositions + destIndex) Vector3(mConstrainedPositions[srcIndex]);
    new (mConstrainedOrientations + destIndex) Quaternion(mConstrainedOrientations[srcIndex]);
    new (mCentersOfMassLocal + destIndex) Vector3(mCentersOfMassLocal[srcIndex]);
    new (mCentersOfMassWorld + destIndex) Vector3(mCentersOfMassWorld[srcIndex]);
    mIsGravityEnabled[destIndex] = mIsGravityEnabled[srcIndex];
    mIsAlreadyInIsland[destIndex] = mIsAlreadyInIsland[srcIndex];
    new (mJoints + destIndex) List<Entity>(mJoints[srcIndex]);

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
    Vector3 externalForce1(mExternalForces[index1]);
    Vector3 externalTorque1(mExternalTorques[index1]);
    decimal linearDamping1 = mLinearDampings[index1];
    decimal angularDamping1 = mAngularDampings[index1];
    decimal mass1 = mMasses[index1];
    decimal inverseMass1 = mInverseMasses[index1];
    Vector3 inertiaTensorLocal1 = mLocalInertiaTensors[index1];
    Vector3 inertiaTensorLocalInverse1 = mInverseInertiaTensorsLocal[index1];
    Vector3 constrainedLinearVelocity1(mConstrainedLinearVelocities[index1]);
    Vector3 constrainedAngularVelocity1(mConstrainedAngularVelocities[index1]);
    Vector3 splitLinearVelocity1(mSplitLinearVelocities[index1]);
    Vector3 splitAngularVelocity1(mSplitAngularVelocities[index1]);
    Vector3 constrainedPosition1 = mConstrainedPositions[index1];
    Quaternion constrainedOrientation1 = mConstrainedOrientations[index1];
    Vector3 centerOfMassLocal1 = mCentersOfMassLocal[index1];
    Vector3 centerOfMassWorld1 = mCentersOfMassWorld[index1];
    bool isGravityEnabled1 = mIsGravityEnabled[index1];
    bool isAlreadyInIsland1 = mIsAlreadyInIsland[index1];
    List<Entity> joints1 = mJoints[index1];

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
    new (mExternalForces + index2) Vector3(externalForce1);
    new (mExternalTorques + index2) Vector3(externalTorque1);
    mLinearDampings[index2] = linearDamping1;
    mAngularDampings[index2] = angularDamping1;
    mMasses[index2] = mass1;
    mInverseMasses[index2] = inverseMass1;
    mLocalInertiaTensors[index2] = inertiaTensorLocal1;
    mInverseInertiaTensorsLocal[index2] = inertiaTensorLocalInverse1;
    new (mConstrainedLinearVelocities + index2) Vector3(constrainedLinearVelocity1);
    new (mConstrainedAngularVelocities + index2) Vector3(constrainedAngularVelocity1);
    new (mSplitLinearVelocities + index2) Vector3(splitLinearVelocity1);
    new (mSplitAngularVelocities + index2) Vector3(splitAngularVelocity1);
    mConstrainedPositions[index2] = constrainedPosition1;
    mConstrainedOrientations[index2] = constrainedOrientation1;
    mCentersOfMassLocal[index2] = centerOfMassLocal1;
    mCentersOfMassWorld[index2] = centerOfMassWorld1;
    mIsGravityEnabled[index2] = isGravityEnabled1;
    mIsAlreadyInIsland[index2] = isAlreadyInIsland1;
    new (mJoints + index2) List<Entity>(joints1);

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
    mExternalForces[index].~Vector3();
    mExternalTorques[index].~Vector3();
    mLocalInertiaTensors[index].~Vector3();
    mInverseInertiaTensorsLocal[index].~Vector3();
    mConstrainedLinearVelocities[index].~Vector3();
    mConstrainedAngularVelocities[index].~Vector3();
    mSplitLinearVelocities[index].~Vector3();
    mSplitAngularVelocities[index].~Vector3();
    mConstrainedPositions[index].~Vector3();
    mConstrainedOrientations[index].~Quaternion();
    mCentersOfMassLocal[index].~Vector3();
    mCentersOfMassWorld[index].~Vector3();
    mJoints[index].~List<Entity>();
}
