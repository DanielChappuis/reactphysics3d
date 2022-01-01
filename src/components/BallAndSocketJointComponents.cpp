/********************************************************************************
* ReactPhysics3D physics library, http://www.reactphysics3d.com                 *
* Copyright (c) 2010-2022 Daniel Chappuis                                       *
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
#include <reactphysics3d/components/BallAndSocketJointComponents.h>
#include <reactphysics3d/engine/EntityManager.h>
#include <reactphysics3d/mathematics/Matrix3x3.h>
#include <cassert>

// We want to use the ReactPhysics3D namespace
using namespace reactphysics3d;

// Constructor
BallAndSocketJointComponents::BallAndSocketJointComponents(MemoryAllocator& allocator)
                    :Components(allocator, sizeof(Entity) + sizeof(BallAndSocketJoint*) + sizeof(Vector3) +
                                sizeof(Vector3) + sizeof(Vector3) + sizeof(Vector3) +
                                sizeof(Matrix3x3) + sizeof(Matrix3x3) + sizeof(Vector3) +
                                sizeof(Matrix3x3) + sizeof(Vector3) + sizeof(bool) + sizeof(decimal) +
                                sizeof(decimal) + sizeof(decimal) + sizeof(decimal) + sizeof(bool) + sizeof(Vector3)) {

    // Allocate memory for the components data
    allocate(INIT_NB_ALLOCATED_COMPONENTS);
}

// Allocate memory for a given number of components
void BallAndSocketJointComponents::allocate(uint32 nbComponentsToAllocate) {

    assert(nbComponentsToAllocate > mNbAllocatedComponents);

    // Size for the data of a single component (in bytes)
    const size_t totalSizeBytes = nbComponentsToAllocate * mComponentDataSize;

    // Allocate memory
    void* newBuffer = mMemoryAllocator.allocate(totalSizeBytes);
    assert(newBuffer != nullptr);

    // New pointers to components data
    Entity* newJointEntities = static_cast<Entity*>(newBuffer);
    BallAndSocketJoint** newJoints = reinterpret_cast<BallAndSocketJoint**>(newJointEntities + nbComponentsToAllocate);
    Vector3* newLocalAnchorPointBody1 = reinterpret_cast<Vector3*>(newJoints + nbComponentsToAllocate);
    Vector3* newLocalAnchorPointBody2 = reinterpret_cast<Vector3*>(newLocalAnchorPointBody1 + nbComponentsToAllocate);
    Vector3* newR1World = reinterpret_cast<Vector3*>(newLocalAnchorPointBody2 + nbComponentsToAllocate);
    Vector3* newR2World = reinterpret_cast<Vector3*>(newR1World + nbComponentsToAllocate);
    Matrix3x3* newI1 = reinterpret_cast<Matrix3x3*>(newR2World + nbComponentsToAllocate);
    Matrix3x3* newI2 = reinterpret_cast<Matrix3x3*>(newI1 + nbComponentsToAllocate);
    Vector3* newBiasVector = reinterpret_cast<Vector3*>(newI2 + nbComponentsToAllocate);
    Matrix3x3* newInverseMassMatrix = reinterpret_cast<Matrix3x3*>(newBiasVector + nbComponentsToAllocate);
    Vector3* newImpulse = reinterpret_cast<Vector3*>(newInverseMassMatrix + nbComponentsToAllocate);
    bool* newIsConeLimitEnabled = reinterpret_cast<bool*>(newImpulse + nbComponentsToAllocate);
    decimal* newConeLimitImpulse = reinterpret_cast<decimal*>(newIsConeLimitEnabled + nbComponentsToAllocate);
    decimal* newConeLimitHalfAngle = reinterpret_cast<decimal*>(newConeLimitImpulse + nbComponentsToAllocate);
    decimal* newInverseMassMatrixConeLimit = reinterpret_cast<decimal*>(newConeLimitHalfAngle + nbComponentsToAllocate);
    decimal* newBConeLimit = reinterpret_cast<decimal*>(newInverseMassMatrixConeLimit + nbComponentsToAllocate);
    bool* newIsConeLimitViolated = reinterpret_cast<bool*>(newBConeLimit + nbComponentsToAllocate);
    Vector3* newConeLimitACrossB = reinterpret_cast<Vector3*>(newIsConeLimitViolated + nbComponentsToAllocate);

    // If there was already components before
    if (mNbComponents > 0) {

        // Copy component data from the previous buffer to the new one
        memcpy(newJointEntities, mJointEntities, mNbComponents * sizeof(Entity));
        memcpy(newJoints, mJoints, mNbComponents * sizeof(BallAndSocketJoint*));
        memcpy(newLocalAnchorPointBody1, mLocalAnchorPointBody1, mNbComponents * sizeof(Vector3));
        memcpy(newLocalAnchorPointBody2, mLocalAnchorPointBody2, mNbComponents * sizeof(Vector3));
        memcpy(newR1World, mR1World, mNbComponents * sizeof(Vector3));
        memcpy(newR2World, mR2World, mNbComponents * sizeof(Vector3));
        memcpy(newI1, mI1, mNbComponents * sizeof(Matrix3x3));
        memcpy(newI2, mI2, mNbComponents * sizeof(Matrix3x3));
        memcpy(newBiasVector, mBiasVector, mNbComponents * sizeof(Vector3));
        memcpy(newInverseMassMatrix, mInverseMassMatrix, mNbComponents * sizeof(Matrix3x3));
        memcpy(newImpulse, mImpulse, mNbComponents * sizeof(Vector3));
        memcpy(newIsConeLimitEnabled, mIsConeLimitEnabled, mNbComponents * sizeof(bool));
        memcpy(newConeLimitImpulse, mConeLimitImpulse, mNbComponents * sizeof(decimal));
        memcpy(newConeLimitHalfAngle, mConeLimitHalfAngle, mNbComponents * sizeof(decimal));
        memcpy(newInverseMassMatrixConeLimit, mInverseMassMatrixConeLimit, mNbComponents * sizeof(decimal));
        memcpy(newBConeLimit, mBConeLimit, mNbComponents * sizeof(decimal));
        memcpy(newIsConeLimitViolated, mIsConeLimitViolated, mNbComponents * sizeof(bool));
        memcpy(newConeLimitACrossB, mConeLimitACrossB, mNbComponents * sizeof(Vector3));

        // Deallocate previous memory
        mMemoryAllocator.release(mBuffer, mNbAllocatedComponents * mComponentDataSize);
    }

    mBuffer = newBuffer;
    mJointEntities = newJointEntities;
    mJoints = newJoints;
    mNbAllocatedComponents = nbComponentsToAllocate;
    mLocalAnchorPointBody1 = newLocalAnchorPointBody1;
    mLocalAnchorPointBody2 = newLocalAnchorPointBody2;
    mR1World = newR1World;
    mR2World = newR2World;
    mI1 = newI1;
    mI2 = newI2;
    mBiasVector = newBiasVector;
    mInverseMassMatrix = newInverseMassMatrix;
    mImpulse = newImpulse;
    mIsConeLimitEnabled = newIsConeLimitEnabled;
    mConeLimitImpulse = newConeLimitImpulse;
    mConeLimitHalfAngle = newConeLimitHalfAngle;
    mInverseMassMatrixConeLimit = newInverseMassMatrixConeLimit;
    mBConeLimit = newBConeLimit;
    mIsConeLimitViolated = newIsConeLimitViolated;
    mConeLimitACrossB = newConeLimitACrossB;
}

// Add a component
void BallAndSocketJointComponents::addComponent(Entity jointEntity, bool isSleeping, const BallAndSocketJointComponent& component) {

    // Prepare to add new component (allocate memory if necessary and compute insertion index)
    uint32 index = prepareAddComponent(isSleeping);

    // Insert the new component data
    new (mJointEntities + index) Entity(jointEntity);
    mJoints[index] = nullptr;
    new (mLocalAnchorPointBody1 + index) Vector3(0, 0, 0);
    new (mLocalAnchorPointBody2 + index) Vector3(0, 0, 0);
    new (mR1World + index) Vector3(0, 0, 0);
    new (mR2World + index) Vector3(0, 0, 0);
    new (mI1 + index) Matrix3x3();
    new (mI2 + index) Matrix3x3();
    new (mBiasVector + index) Vector3(0, 0, 0);
    new (mInverseMassMatrix + index) Matrix3x3();
    new (mImpulse + index) Vector3(0, 0, 0);
    mIsConeLimitEnabled[index] = component.isConeLimitEnabled;
    mConeLimitImpulse[index] = decimal(0.0);
    mConeLimitHalfAngle[index] = component.coneLimitHalfAngle;
    mInverseMassMatrixConeLimit[index] = decimal(0.0);
    mBConeLimit[index] = decimal(0.0);
    mIsConeLimitViolated[index] = false;
    new (mConeLimitACrossB + index) Vector3(0, 0, 0);

    // Map the entity with the new component lookup index
    mMapEntityToComponentIndex.add(Pair<Entity, uint32>(jointEntity, index));

    mNbComponents++;

    assert(mDisabledStartIndex <= mNbComponents);
    assert(mNbComponents == static_cast<uint32>(mMapEntityToComponentIndex.size()));
}

// Move a component from a source to a destination index in the components array
// The destination location must contain a constructed object
void BallAndSocketJointComponents::moveComponentToIndex(uint32 srcIndex, uint32 destIndex) {

    const Entity entity = mJointEntities[srcIndex];

    // Copy the data of the source component to the destination location
    new (mJointEntities + destIndex) Entity(mJointEntities[srcIndex]);
    mJoints[destIndex] = mJoints[srcIndex];
    new (mLocalAnchorPointBody1 + destIndex) Vector3(mLocalAnchorPointBody1[srcIndex]);
    new (mLocalAnchorPointBody2 + destIndex) Vector3(mLocalAnchorPointBody2[srcIndex]);
    new (mR1World + destIndex) Vector3(mR1World[srcIndex]);
    new (mR2World + destIndex) Vector3(mR2World[srcIndex]);
    new (mI1 + destIndex) Matrix3x3(mI1[srcIndex]);
    new (mI2 + destIndex) Matrix3x3(mI2[srcIndex]);
    new (mBiasVector + destIndex) Vector3(mBiasVector[srcIndex]);
    new (mInverseMassMatrix + destIndex) Matrix3x3(mInverseMassMatrix[srcIndex]);
    new (mImpulse + destIndex) Vector3(mImpulse[srcIndex]);
    mIsConeLimitEnabled[destIndex] = mIsConeLimitEnabled[srcIndex];
    mConeLimitImpulse[destIndex] = mConeLimitImpulse[srcIndex];
    mConeLimitHalfAngle[destIndex] = mConeLimitHalfAngle[srcIndex];
    mInverseMassMatrixConeLimit[destIndex] = mInverseMassMatrixConeLimit[srcIndex];
    mBConeLimit[destIndex] = mBConeLimit[srcIndex];
    mIsConeLimitViolated[destIndex] = mIsConeLimitViolated[srcIndex];
    new (mConeLimitACrossB + destIndex) Vector3(mConeLimitACrossB[srcIndex]);

    // Destroy the source component
    destroyComponent(srcIndex);

    assert(!mMapEntityToComponentIndex.containsKey(entity));

    // Update the entity to component index mapping
    mMapEntityToComponentIndex.add(Pair<Entity, uint32>(entity, destIndex));

    assert(mMapEntityToComponentIndex[mJointEntities[destIndex]] == destIndex);
}

// Swap two components in the array
void BallAndSocketJointComponents::swapComponents(uint32 index1, uint32 index2) {

    // Copy component 1 data
    Entity jointEntity1(mJointEntities[index1]);
    BallAndSocketJoint* joint1 = mJoints[index1];
    Vector3 localAnchorPointBody1(mLocalAnchorPointBody1[index1]);
    Vector3 localAnchorPointBody2(mLocalAnchorPointBody2[index1]);
    Vector3 r1World1(mR1World[index1]);
    Vector3 r2World1(mR2World[index1]);
    Matrix3x3 i11(mI1[index1]);
    Matrix3x3 i21(mI2[index1]);
    Vector3 biasVector1(mBiasVector[index1]);
    Matrix3x3 inverseMassMatrix1(mInverseMassMatrix[index1]);
    Vector3 impulse1(mImpulse[index1]);
    bool isConeLimitEnabled1 = mIsConeLimitEnabled[index1];
    decimal coneLimitImpulse1 = mConeLimitImpulse[index1];
    decimal coneLimitHalfAngle1 = mConeLimitHalfAngle[index1];
    decimal inverseMassMatrixConeLimit1 = mInverseMassMatrixConeLimit[index1];
    decimal bConeLimit = mBConeLimit[index1];
    bool isConeLimitViolated = mIsConeLimitViolated[index1];
    Vector3 coneLimitAcrossB(mConeLimitACrossB[index1]);

    // Destroy component 1
    destroyComponent(index1);

    moveComponentToIndex(index2, index1);

    // Reconstruct component 1 at component 2 location
    new (mJointEntities + index2) Entity(jointEntity1);
    mJoints[index2] = joint1;
    new (mLocalAnchorPointBody1 + index2) Vector3(localAnchorPointBody1);
    new (mLocalAnchorPointBody2 + index2) Vector3(localAnchorPointBody2);
    new (mR1World + index2) Vector3(r1World1);
    new (mR2World + index2) Vector3(r2World1);
    new (mI1 + index2) Matrix3x3(i11);
    new (mI2 + index2) Matrix3x3(i21);
    new (mBiasVector + index2) Vector3(biasVector1);
    new (mInverseMassMatrix + index2) Matrix3x3(inverseMassMatrix1);
    new (mImpulse + index2) Vector3(impulse1);
    mIsConeLimitEnabled[index2] = isConeLimitEnabled1;
    mConeLimitImpulse[index2] = coneLimitImpulse1;
    mConeLimitHalfAngle[index2] = coneLimitHalfAngle1;
    mInverseMassMatrixConeLimit[index2] = inverseMassMatrixConeLimit1;
    mBConeLimit[index2] = bConeLimit;
    mIsConeLimitViolated[index2] = isConeLimitViolated;
    new (mConeLimitACrossB + index2) Vector3(coneLimitAcrossB);

    // Update the entity to component index mapping
    mMapEntityToComponentIndex.add(Pair<Entity, uint32>(jointEntity1, index2));

    assert(mMapEntityToComponentIndex[mJointEntities[index1]] == index1);
    assert(mMapEntityToComponentIndex[mJointEntities[index2]] == index2);
    assert(mNbComponents == static_cast<uint32>(mMapEntityToComponentIndex.size()));
}

// Destroy a component at a given index
void BallAndSocketJointComponents::destroyComponent(uint32 index) {

    Components::destroyComponent(index);

    assert(mMapEntityToComponentIndex[mJointEntities[index]] == index);

    mMapEntityToComponentIndex.remove(mJointEntities[index]);

    mJointEntities[index].~Entity();
    mJoints[index] = nullptr;
    mLocalAnchorPointBody1[index].~Vector3();
    mLocalAnchorPointBody2[index].~Vector3();
    mR1World[index].~Vector3();
    mR2World[index].~Vector3();
    mI1[index].~Matrix3x3();
    mI2[index].~Matrix3x3();
    mBiasVector[index].~Vector3();
    mInverseMassMatrix[index].~Matrix3x3();
    mImpulse[index].~Vector3();
    mConeLimitACrossB[index].~Vector3();
}
