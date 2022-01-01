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
#include <reactphysics3d/components/SliderJointComponents.h>
#include <reactphysics3d/engine/EntityManager.h>
#include <reactphysics3d/mathematics/Matrix3x3.h>
#include <cassert>

// We want to use the ReactPhysics3D namespace
using namespace reactphysics3d;

// Constructor
SliderJointComponents::SliderJointComponents(MemoryAllocator& allocator)
                    :Components(allocator, sizeof(Entity) + sizeof(SliderJoint*) + sizeof(Vector3) +
                                sizeof(Vector3) +
                                sizeof(Matrix3x3) + sizeof(Matrix3x3) + sizeof(Vector2) +
                                sizeof(Vector3) + sizeof(Matrix2x2) + sizeof(Matrix3x3) +
                                sizeof(Vector2) + sizeof(Vector3) + sizeof(Quaternion) +
                                sizeof(Vector3) + sizeof(Vector3) + sizeof(Vector3) + sizeof(Vector3) +
                                sizeof(Vector3) + sizeof(Vector3) + sizeof(decimal) + sizeof(decimal) + sizeof(decimal) +
                                sizeof(decimal) + sizeof(decimal) + sizeof(decimal) + sizeof(decimal) +
                                sizeof(bool) + sizeof(bool) + sizeof(decimal) + sizeof(decimal)  +
                                sizeof(bool) + sizeof(bool) + sizeof(decimal) + sizeof(decimal) +
                                sizeof(Vector3) + sizeof(Vector3) + sizeof(Vector3) + sizeof(Vector3) +
                                sizeof(Vector3) + sizeof(Vector3)) {

    // Allocate memory for the components data
    allocate(INIT_NB_ALLOCATED_COMPONENTS);
}

// Allocate memory for a given number of components
void SliderJointComponents::allocate(uint32 nbComponentsToAllocate) {

    assert(nbComponentsToAllocate > mNbAllocatedComponents);

    // Size for the data of a single component (in bytes)
    const size_t totalSizeBytes = nbComponentsToAllocate * mComponentDataSize;

    // Allocate memory
    void* newBuffer = mMemoryAllocator.allocate(totalSizeBytes);
    assert(newBuffer != nullptr);

    // New pointers to components data
    Entity* newJointEntities = static_cast<Entity*>(newBuffer);
    SliderJoint** newJoints = reinterpret_cast<SliderJoint**>(newJointEntities + nbComponentsToAllocate);
    Vector3* newLocalAnchorPointBody1 = reinterpret_cast<Vector3*>(newJoints + nbComponentsToAllocate);
    Vector3* newLocalAnchorPointBody2 = reinterpret_cast<Vector3*>(newLocalAnchorPointBody1 + nbComponentsToAllocate);
    Matrix3x3* newI1 = reinterpret_cast<Matrix3x3*>(newLocalAnchorPointBody2 + nbComponentsToAllocate);
    Matrix3x3* newI2 = reinterpret_cast<Matrix3x3*>(newI1 + nbComponentsToAllocate);
    Vector2* newImpulseTranslation = reinterpret_cast<Vector2*>(newI2 + nbComponentsToAllocate);
    Vector3* newImpulseRotation = reinterpret_cast<Vector3*>(newImpulseTranslation + nbComponentsToAllocate);
    Matrix2x2* newInverseMassMatrixTranslation = reinterpret_cast<Matrix2x2*>(newImpulseRotation + nbComponentsToAllocate);
    Matrix3x3* newInverseMassMatrixRotation = reinterpret_cast<Matrix3x3*>(newInverseMassMatrixTranslation + nbComponentsToAllocate);
    Vector2* newBiasTranslation = reinterpret_cast<Vector2*>(newInverseMassMatrixRotation + nbComponentsToAllocate);
    Vector3* newBiasRotation = reinterpret_cast<Vector3*>(newBiasTranslation + nbComponentsToAllocate);
    Quaternion* newInitOrientationDifferenceInv = reinterpret_cast<Quaternion*>(newBiasRotation + nbComponentsToAllocate);
    Vector3* newSliderAxisBody1 = reinterpret_cast<Vector3*>(newInitOrientationDifferenceInv + nbComponentsToAllocate);
    Vector3* newSliderAxisWorld = reinterpret_cast<Vector3*>(newSliderAxisBody1 + nbComponentsToAllocate);
    Vector3* newR1 = reinterpret_cast<Vector3*>(newSliderAxisWorld + nbComponentsToAllocate);
    Vector3* newR2 = reinterpret_cast<Vector3*>(newR1 + nbComponentsToAllocate);
    Vector3* newN1 = reinterpret_cast<Vector3*>(newR2 + nbComponentsToAllocate);
    Vector3* newN2 = reinterpret_cast<Vector3*>(newN1 + nbComponentsToAllocate);
    decimal* newImpulseLowerLimit = reinterpret_cast<decimal*>(newN2 + nbComponentsToAllocate);
    decimal* newImpulseUpperLimit = reinterpret_cast<decimal*>(newImpulseLowerLimit + nbComponentsToAllocate);
    decimal* newImpulseMotor = reinterpret_cast<decimal*>(newImpulseUpperLimit + nbComponentsToAllocate);
    decimal* newInverseMassMatrixLimit = reinterpret_cast<decimal*>(newImpulseMotor + nbComponentsToAllocate);
    decimal* newInverseMassMatrixMotor = reinterpret_cast<decimal*>(newInverseMassMatrixLimit + nbComponentsToAllocate);
    decimal* newBLowerLimit = reinterpret_cast<decimal*>(newInverseMassMatrixMotor + nbComponentsToAllocate);
    decimal* newBUpperLimit = reinterpret_cast<decimal*>(newBLowerLimit + nbComponentsToAllocate);
    bool* newIsLimitEnabled = reinterpret_cast<bool*>(newBUpperLimit + nbComponentsToAllocate);
    bool* newIsMotorEnabled = reinterpret_cast<bool*>(newIsLimitEnabled + nbComponentsToAllocate);
    decimal* newLowerLimit = reinterpret_cast<decimal*>(newIsMotorEnabled + nbComponentsToAllocate);
    decimal* newUpperLimit = reinterpret_cast<decimal*>(newLowerLimit + nbComponentsToAllocate);
    bool* newIsLowerLimitViolated = reinterpret_cast<bool*>(newUpperLimit + nbComponentsToAllocate);
    bool* newIsUpperLimitViolated = reinterpret_cast<bool*>(newIsLowerLimitViolated + nbComponentsToAllocate);
    decimal* newMotorSpeed = reinterpret_cast<decimal*>(newIsUpperLimitViolated + nbComponentsToAllocate);
    decimal* newMaxMotorForce = reinterpret_cast<decimal*>(newMotorSpeed + nbComponentsToAllocate);
    Vector3* newR2CrossN1 = reinterpret_cast<Vector3*>(newMaxMotorForce + nbComponentsToAllocate);
    Vector3* newR2CrossN2 = reinterpret_cast<Vector3*>(newR2CrossN1 + nbComponentsToAllocate);
    Vector3* newR2CrossSliderAxis = reinterpret_cast<Vector3*>(newR2CrossN2 + nbComponentsToAllocate);
    Vector3* newR1PlusUCrossN1 = reinterpret_cast<Vector3*>(newR2CrossSliderAxis + nbComponentsToAllocate);
    Vector3* newR1PlusUCrossN2 = reinterpret_cast<Vector3*>(newR1PlusUCrossN1 + nbComponentsToAllocate);
    Vector3* newR1PlusUCrossSliderAxis = reinterpret_cast<Vector3*>(newR1PlusUCrossN2 + nbComponentsToAllocate);

    // If there was already components before
    if (mNbComponents > 0) {

        // Copy component data from the previous buffer to the new one
        memcpy(newJointEntities, mJointEntities, mNbComponents * sizeof(Entity));
        memcpy(newJoints, mJoints, mNbComponents * sizeof(SliderJoint*));
        memcpy(newLocalAnchorPointBody1, mLocalAnchorPointBody1, mNbComponents * sizeof(Vector3));
        memcpy(newLocalAnchorPointBody2, mLocalAnchorPointBody2, mNbComponents * sizeof(Vector3));
        memcpy(newI1, mI1, mNbComponents * sizeof(Matrix3x3));
        memcpy(newI2, mI2, mNbComponents * sizeof(Matrix3x3));
        memcpy(newImpulseTranslation, mImpulseTranslation, mNbComponents * sizeof(Vector2));
        memcpy(newImpulseRotation, mImpulseRotation, mNbComponents * sizeof(Vector3));
        memcpy(newInverseMassMatrixTranslation, mInverseMassMatrixTranslation, mNbComponents * sizeof(Matrix2x2));
        memcpy(newInverseMassMatrixRotation, mInverseMassMatrixRotation, mNbComponents * sizeof(Matrix3x3));
        memcpy(newBiasTranslation, mBiasTranslation, mNbComponents * sizeof(Vector2));
        memcpy(newBiasRotation, mBiasRotation, mNbComponents * sizeof(Vector3));
        memcpy(newInitOrientationDifferenceInv, mInitOrientationDifferenceInv, mNbComponents * sizeof(Quaternion));
        memcpy(newSliderAxisBody1, mSliderAxisBody1, mNbComponents * sizeof(Vector3));
        memcpy(newSliderAxisWorld, mSliderAxisWorld, mNbComponents * sizeof(Vector3));
        memcpy(newR1, mR1, mNbComponents * sizeof(Vector3));
        memcpy(newR2, mR2, mNbComponents * sizeof(Vector3));
        memcpy(newN1, mN1, mNbComponents * sizeof(Vector3));
        memcpy(newN2, mN2, mNbComponents * sizeof(Vector3));
        memcpy(newImpulseLowerLimit, mImpulseLowerLimit, mNbComponents * sizeof(decimal));
        memcpy(newImpulseUpperLimit, mImpulseUpperLimit, mNbComponents * sizeof(decimal));
        memcpy(newImpulseMotor, mImpulseMotor, mNbComponents * sizeof(decimal));
        memcpy(newInverseMassMatrixLimit, mInverseMassMatrixLimit, mNbComponents * sizeof(decimal));
        memcpy(newInverseMassMatrixMotor, mInverseMassMatrixMotor, mNbComponents * sizeof(decimal));
        memcpy(newBLowerLimit, mBLowerLimit, mNbComponents * sizeof(decimal));
        memcpy(newBUpperLimit, mBUpperLimit, mNbComponents * sizeof(decimal));
        memcpy(newIsLimitEnabled, mIsLimitEnabled, mNbComponents * sizeof(bool));
        memcpy(newIsMotorEnabled, mIsMotorEnabled, mNbComponents * sizeof(bool));
        memcpy(newLowerLimit, mLowerLimit, mNbComponents * sizeof(decimal));
        memcpy(newUpperLimit, mUpperLimit, mNbComponents * sizeof(decimal));
        memcpy(newIsLowerLimitViolated, mIsLowerLimitViolated, mNbComponents * sizeof(bool));
        memcpy(newIsUpperLimitViolated, mIsUpperLimitViolated, mNbComponents * sizeof(bool));
        memcpy(newMotorSpeed, mMotorSpeed, mNbComponents * sizeof(decimal));
        memcpy(newMaxMotorForce, mMaxMotorForce, mNbComponents * sizeof(decimal));
        memcpy(newR2CrossN1, mR2CrossN1, mNbComponents * sizeof(decimal));
        memcpy(newR2CrossN2, mR2CrossN2, mNbComponents * sizeof(decimal));
        memcpy(newR2CrossSliderAxis, mR2CrossSliderAxis, mNbComponents * sizeof(decimal));
        memcpy(newR1PlusUCrossN1, mR1PlusUCrossN1, mNbComponents * sizeof(decimal));
        memcpy(newR1PlusUCrossN2, mR1PlusUCrossN2, mNbComponents * sizeof(decimal));
        memcpy(newR1PlusUCrossSliderAxis, mR1PlusUCrossSliderAxis, mNbComponents * sizeof(decimal));

        // Deallocate previous memory
        mMemoryAllocator.release(mBuffer, mNbAllocatedComponents * mComponentDataSize);
    }

    mBuffer = newBuffer;
    mJointEntities = newJointEntities;
    mJoints = newJoints;
    mNbAllocatedComponents = nbComponentsToAllocate;
    mLocalAnchorPointBody1 = newLocalAnchorPointBody1;
    mLocalAnchorPointBody2 = newLocalAnchorPointBody2;
    mI1 = newI1;
    mI2 = newI2;
    mImpulseTranslation = newImpulseTranslation;
    mImpulseRotation = newImpulseRotation;
    mInverseMassMatrixTranslation = newInverseMassMatrixTranslation;
    mInverseMassMatrixRotation = newInverseMassMatrixRotation;
    mBiasTranslation = newBiasTranslation;
    mBiasRotation = newBiasRotation;
    mInitOrientationDifferenceInv = newInitOrientationDifferenceInv;
    mSliderAxisBody1 = newSliderAxisBody1;
    mSliderAxisWorld = newSliderAxisWorld;
    mR1 = newR1;
    mR2 = newR2;
    mN1 = newN1;
    mN2 = newN2;
    mImpulseLowerLimit = newImpulseLowerLimit;
    mImpulseUpperLimit = newImpulseUpperLimit;
    mImpulseMotor = newImpulseMotor;
    mInverseMassMatrixLimit = newInverseMassMatrixLimit;
    mInverseMassMatrixMotor = newInverseMassMatrixMotor;
    mBLowerLimit = newBLowerLimit;
    mBUpperLimit = newBUpperLimit;
    mIsLimitEnabled = newIsLimitEnabled;
    mIsMotorEnabled = newIsMotorEnabled;
    mLowerLimit = newLowerLimit;
    mUpperLimit = newUpperLimit;
    mIsLowerLimitViolated = newIsLowerLimitViolated;
    mIsUpperLimitViolated = newIsUpperLimitViolated;
    mMotorSpeed = newMotorSpeed;
    mMaxMotorForce = newMaxMotorForce;
    mR2CrossN1 = newR2CrossN1;
    mR2CrossN2 = newR2CrossN2;
    mR2CrossSliderAxis = newR2CrossSliderAxis;
    mR1PlusUCrossN1 = newR1PlusUCrossN1;
    mR1PlusUCrossN2 = newR1PlusUCrossN2;
    mR1PlusUCrossSliderAxis = newR1PlusUCrossSliderAxis;
}

// Add a component
void SliderJointComponents::addComponent(Entity jointEntity, bool isSleeping, const SliderJointComponent& component) {

    // Prepare to add new component (allocate memory if necessary and compute insertion index)
    uint32 index = prepareAddComponent(isSleeping);

    // Insert the new component data
    new (mJointEntities + index) Entity(jointEntity);
    mJoints[index] = nullptr;
    new (mLocalAnchorPointBody1 + index) Vector3(0, 0, 0);
    new (mLocalAnchorPointBody2 + index) Vector3(0, 0, 0);
    new (mI1 + index) Matrix3x3();
    new (mI2 + index) Matrix3x3();
    new (mImpulseTranslation + index) Vector2(0, 0);
    new (mImpulseRotation + index) Vector3(0, 0, 0);
    new (mInverseMassMatrixTranslation + index) Matrix2x2();
    new (mInverseMassMatrixRotation + index) Matrix3x3();
    new (mBiasTranslation + index) Vector2(0, 0);
    new (mBiasRotation + index) Vector3(0, 0, 0);
    new (mInitOrientationDifferenceInv + index) Quaternion(0, 0, 0, 0);
    new (mSliderAxisBody1 + index) Vector3(0, 0, 0);
    new (mSliderAxisWorld + index) Vector3(0, 0, 0);
    new (mR1 + index) Vector3(0, 0, 0);
    new (mR2 + index) Vector3(0, 0, 0);
    new (mN1 + index) Vector3(0, 0, 0);
    new (mN2 + index) Vector3(0, 0, 0);
    mImpulseLowerLimit[index] = decimal(0.0);
    mImpulseUpperLimit[index] = decimal(0.0);
    mImpulseMotor[index] = decimal(0.0);
    mInverseMassMatrixLimit[index] = decimal(0.0);
    mInverseMassMatrixMotor[index] = decimal(0.0);
    mBLowerLimit[index] = decimal(0.0);
    mBUpperLimit[index] = decimal(0.0);
    mIsLimitEnabled[index] = component.isLimitEnabled;
    mIsMotorEnabled[index] = component.isMotorEnabled;
    mLowerLimit[index] = component.lowerLimit;
    mUpperLimit[index] = component.upperLimit;
    mIsLowerLimitViolated[index] = false;
    mIsUpperLimitViolated[index] = false;
    mMotorSpeed[index] = component.motorSpeed;
    mMaxMotorForce[index] = component.maxMotorForce;
    new (mR2CrossN1 + index) Vector3(0, 0, 0);
    new (mR2CrossN2 + index) Vector3(0, 0, 0);
    new (mR2CrossSliderAxis + index) Vector3(0, 0, 0);
    new (mR1PlusUCrossN1 + index) Vector3(0, 0, 0);
    new (mR1PlusUCrossN2 + index) Vector3(0, 0, 0);
    new (mR1PlusUCrossSliderAxis + index) Vector3(0, 0, 0);

    // Map the entity with the new component lookup index
    mMapEntityToComponentIndex.add(Pair<Entity, uint32>(jointEntity, index));

    mNbComponents++;

    assert(mDisabledStartIndex <= mNbComponents);
    assert(mNbComponents == static_cast<uint32>(mMapEntityToComponentIndex.size()));
}

// Move a component from a source to a destination index in the components array
// The destination location must contain a constructed object
void SliderJointComponents::moveComponentToIndex(uint32 srcIndex, uint32 destIndex) {

    const Entity entity = mJointEntities[srcIndex];

    // Copy the data of the source component to the destination location
    new (mJointEntities + destIndex) Entity(mJointEntities[srcIndex]);
    mJoints[destIndex] = mJoints[srcIndex];
    new (mLocalAnchorPointBody1 + destIndex) Vector3(mLocalAnchorPointBody1[srcIndex]);
    new (mLocalAnchorPointBody2 + destIndex) Vector3(mLocalAnchorPointBody2[srcIndex]);
    new (mI1 + destIndex) Matrix3x3(mI1[srcIndex]);
    new (mI2 + destIndex) Matrix3x3(mI2[srcIndex]);
    new (mImpulseTranslation + destIndex) Vector2(mImpulseTranslation[srcIndex]);
    new (mImpulseRotation + destIndex) Vector3(mImpulseRotation[srcIndex]);
    new (mInverseMassMatrixTranslation + destIndex) Matrix2x2(mInverseMassMatrixTranslation[srcIndex]);
    new (mInverseMassMatrixRotation + destIndex) Matrix3x3(mInverseMassMatrixRotation[srcIndex]);
    new (mBiasTranslation + destIndex) Vector2(mBiasTranslation[srcIndex]);
    new (mBiasRotation + destIndex) Vector3(mBiasRotation[srcIndex]);
    new (mInitOrientationDifferenceInv + destIndex) Quaternion(mInitOrientationDifferenceInv[srcIndex]);
    new (mSliderAxisBody1 + destIndex) Vector3(mSliderAxisBody1[srcIndex]);
    new (mSliderAxisWorld + destIndex) Vector3(mSliderAxisWorld[srcIndex]);
    new (mR1 + destIndex) Vector3(mR1[srcIndex]);
    new (mR2 + destIndex) Vector3(mR2[srcIndex]);
    new (mN1 + destIndex) Vector3(mN1[srcIndex]);
    new (mN2 + destIndex) Vector3(mN2[srcIndex]);
    mImpulseLowerLimit[destIndex] = mImpulseLowerLimit[srcIndex];
    mImpulseUpperLimit[destIndex] = mImpulseUpperLimit[srcIndex];
    mImpulseMotor[destIndex] = mImpulseMotor[srcIndex];
    mInverseMassMatrixLimit[destIndex] = mInverseMassMatrixLimit[srcIndex];
    mInverseMassMatrixMotor[destIndex] = mInverseMassMatrixMotor[srcIndex];
    mBLowerLimit[destIndex] = mBLowerLimit[srcIndex];
    mBUpperLimit[destIndex] = mBUpperLimit[srcIndex];
    mIsLimitEnabled[destIndex] = mIsLimitEnabled[srcIndex];
    mIsMotorEnabled[destIndex] = mIsMotorEnabled[srcIndex];
    mLowerLimit[destIndex] = mLowerLimit[srcIndex];
    mUpperLimit[destIndex] = mUpperLimit[srcIndex];
    mIsLowerLimitViolated[destIndex] = mIsLowerLimitViolated[srcIndex];
    mIsUpperLimitViolated[destIndex] = mIsUpperLimitViolated[srcIndex];
    mMotorSpeed[destIndex] = mMotorSpeed[srcIndex];
    mMaxMotorForce[destIndex] = mMaxMotorForce[srcIndex];
    new (mR2CrossN1 + destIndex) Vector3(mR2CrossN1[srcIndex]);
    new (mR2CrossN2 + destIndex) Vector3(mR2CrossN2[srcIndex]);
    new (mR2CrossSliderAxis + destIndex) Vector3(mR2CrossSliderAxis[srcIndex]);
    new (mR1PlusUCrossN1 + destIndex) Vector3(mR1PlusUCrossN1[srcIndex]);
    new (mR1PlusUCrossN2 + destIndex) Vector3(mR1PlusUCrossN2[srcIndex]);
    new (mR1PlusUCrossSliderAxis + destIndex) Vector3(mR1PlusUCrossSliderAxis[srcIndex]);

    // Destroy the source component
    destroyComponent(srcIndex);

    assert(!mMapEntityToComponentIndex.containsKey(entity));

    // Update the entity to component index mapping
    mMapEntityToComponentIndex.add(Pair<Entity, uint32>(entity, destIndex));

    assert(mMapEntityToComponentIndex[mJointEntities[destIndex]] == destIndex);
}

// Swap two components in the array
void SliderJointComponents::swapComponents(uint32 index1, uint32 index2) {

    // Copy component 1 data
    Entity jointEntity1(mJointEntities[index1]);
    SliderJoint* joint1 = mJoints[index1];
    Vector3 localAnchorPointBody1(mLocalAnchorPointBody1[index1]);
    Vector3 localAnchorPointBody2(mLocalAnchorPointBody2[index1]);
    Matrix3x3 i11(mI1[index1]);
    Matrix3x3 i21(mI2[index1]);
    Vector2 impulseTranslation1(mImpulseTranslation[index1]);
    Vector3 impulseRotation1(mImpulseRotation[index1]);
    Matrix2x2 inverseMassMatrixTranslation1(mInverseMassMatrixTranslation[index1]);
    Matrix3x3 inverseMassMatrixRotation1(mInverseMassMatrixRotation[index1]);
    Vector2 biasTranslation1(mBiasTranslation[index1]);
    Vector3 biasRotation1(mBiasRotation[index1]);
    Quaternion initOrientationDifferenceInv1(mInitOrientationDifferenceInv[index1]);
    Vector3 sliderAxisBody1(mSliderAxisBody1[index1]);
    Vector3 sliderAxisWorld(mSliderAxisWorld[index1]);
    Vector3 r1(mR1[index1]);
    Vector3 r2(mR2[index1]);
    Vector3 n1(mN1[index1]);
    Vector3 n2(mN2[index1]);
    decimal impulseLowerLimit(mImpulseLowerLimit[index1]);
    decimal impulseUpperLimit(mImpulseUpperLimit[index1]);
    decimal impulseMotor(mImpulseMotor[index1]);
    decimal inverseMassMatrixLimit(mInverseMassMatrixLimit[index1]);
    decimal inverseMassMatrixMotor(mInverseMassMatrixMotor[index1]);
    decimal bLowerLimit(mBLowerLimit[index1]);
    decimal bUpperLimit(mUpperLimit[index1]);
    bool isLimitEnabled(mIsLimitEnabled[index1]);
    bool isMotorEnabled(mIsMotorEnabled[index1]);
    decimal lowerLimit(mLowerLimit[index1]);
    decimal upperLimit(mUpperLimit[index1]);
    bool isLowerLimitViolated(mIsLowerLimitViolated[index1]);
    bool isUpperLimitViolated(mIsUpperLimitViolated[index1]);
    decimal motorSpeed(mMotorSpeed[index1]);
    decimal maxMotorForce(mMaxMotorForce[index1]);
    Vector3 r2CrossN1(mR2CrossN1[index1]);
    Vector3 r2CrossN2(mR2CrossN2[index1]);
    Vector3 r2CrossSliderAxis(mR2CrossSliderAxis[index1]);
    Vector3 r1PlusUCrossN1(mR1PlusUCrossN1[index1]);
    Vector3 r1PlusUCrossN2(mR1PlusUCrossN2[index1]);
    Vector3 r1PlusUCrossSliderAxis(mR1PlusUCrossSliderAxis[index1]);

    // Destroy component 1
    destroyComponent(index1);

    moveComponentToIndex(index2, index1);

    // Reconstruct component 1 at component 2 location
    new (mJointEntities + index2) Entity(jointEntity1);
    mJoints[index2] = joint1;
    new (mLocalAnchorPointBody1 + index2) Vector3(localAnchorPointBody1);
    new (mLocalAnchorPointBody2 + index2) Vector3(localAnchorPointBody2);
    new (mI1 + index2) Matrix3x3(i11);
    new (mI2 + index2) Matrix3x3(i21);
    new (mImpulseTranslation + index2) Vector2(impulseTranslation1);
    new (mImpulseRotation + index2) Vector3(impulseRotation1);
    new (mInverseMassMatrixTranslation + index2) Matrix2x2(inverseMassMatrixTranslation1);
    new (mInverseMassMatrixRotation + index2) Matrix3x3(inverseMassMatrixRotation1);
    new (mBiasTranslation + index2) Vector2(biasTranslation1);
    new (mBiasRotation + index2) Vector3(biasRotation1);
    new (mInitOrientationDifferenceInv + index2) Quaternion(initOrientationDifferenceInv1);
    new (mSliderAxisBody1 + index2) Vector3(sliderAxisBody1);
    new (mSliderAxisWorld + index2) Vector3(sliderAxisWorld);
    new (mR1 + index2) Vector3(r1);
    new (mR2 + index2) Vector3(r2);
    new (mN1 + index2) Vector3(n1);
    new (mN2 + index2) Vector3(n2);
    mImpulseLowerLimit[index2] = impulseLowerLimit;
    mImpulseUpperLimit[index2] = impulseUpperLimit;
    mImpulseMotor[index2] = impulseMotor;
    mInverseMassMatrixLimit[index2] = inverseMassMatrixLimit;
    mInverseMassMatrixMotor[index2] = inverseMassMatrixMotor;
    mBLowerLimit[index2] = bLowerLimit;
    mBUpperLimit[index2] = bUpperLimit;
    mIsLimitEnabled[index2] = isLimitEnabled;
    mIsMotorEnabled[index2] = isMotorEnabled;
    mLowerLimit[index2] = lowerLimit;
    mUpperLimit[index2] = upperLimit;
    mIsLowerLimitViolated[index2] = isLowerLimitViolated;
    mIsUpperLimitViolated[index2] = isUpperLimitViolated;
    mMotorSpeed[index2] = motorSpeed;
    mMaxMotorForce[index2] = maxMotorForce;
    new (mR2CrossN1 + index2) Vector3(r2CrossN1);
    new (mR2CrossN2 + index2) Vector3(r2CrossN2);
    new (mR2CrossSliderAxis + index2) Vector3(r2CrossSliderAxis);
    new (mR1PlusUCrossN1 + index2) Vector3(r1PlusUCrossN1);
    new (mR1PlusUCrossN2 + index2) Vector3(r1PlusUCrossN2);
    new (mR1PlusUCrossSliderAxis + index2) Vector3(r1PlusUCrossSliderAxis);

    // Update the entity to component index mapping
    mMapEntityToComponentIndex.add(Pair<Entity, uint32>(jointEntity1, index2));

    assert(mMapEntityToComponentIndex[mJointEntities[index1]] == index1);
    assert(mMapEntityToComponentIndex[mJointEntities[index2]] == index2);
    assert(mNbComponents == static_cast<uint32>(mMapEntityToComponentIndex.size()));
}

// Destroy a component at a given index
void SliderJointComponents::destroyComponent(uint32 index) {

    Components::destroyComponent(index);

    assert(mMapEntityToComponentIndex[mJointEntities[index]] == index);

    mMapEntityToComponentIndex.remove(mJointEntities[index]);

    mJointEntities[index].~Entity();
    mJoints[index] = nullptr;
    mLocalAnchorPointBody1[index].~Vector3();
    mLocalAnchorPointBody2[index].~Vector3();
    mI1[index].~Matrix3x3();
    mI2[index].~Matrix3x3();
    mImpulseTranslation[index].~Vector2();
    mImpulseRotation[index].~Vector3();
    mInverseMassMatrixTranslation[index].~Matrix2x2();
    mInverseMassMatrixRotation[index].~Matrix3x3();
    mBiasTranslation[index].~Vector2();
    mBiasRotation[index].~Vector3();
    mInitOrientationDifferenceInv[index].~Quaternion();
    mSliderAxisBody1[index].~Vector3();
    mSliderAxisWorld[index].~Vector3();
    mR1[index].~Vector3();
    mR2[index].~Vector3();
    mN1[index].~Vector3();
    mN2[index].~Vector3();
    mR2CrossN1[index].~Vector3();
    mR2CrossN2[index].~Vector3();
    mR2CrossSliderAxis[index].~Vector3();
    mR1PlusUCrossN1[index].~Vector3();
    mR1PlusUCrossN2[index].~Vector3();
    mR1PlusUCrossSliderAxis[index].~Vector3();
}
