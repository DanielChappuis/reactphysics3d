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
#include <reactphysics3d/components/FixedJointComponents.h>
#include <reactphysics3d/engine/EntityManager.h>
#include <reactphysics3d/mathematics/Matrix3x3.h>
#include <cassert>

// We want to use the ReactPhysics3D namespace
using namespace reactphysics3d;

// Constructor
FixedJointComponents::FixedJointComponents(MemoryAllocator& allocator)
                    :Components(allocator, sizeof(Entity) + sizeof(FixedJoint*) + sizeof(Vector3) +
                                sizeof(Vector3) + sizeof(Vector3) + sizeof(Vector3) +
                                sizeof(Matrix3x3) + sizeof(Matrix3x3) + sizeof(Vector3) +
                                sizeof(Vector3) + sizeof(Matrix3x3) + sizeof(Matrix3x3) +
                                sizeof(Vector3) + sizeof(Vector3) + sizeof(Quaternion), 15 * GLOBAL_ALIGNMENT) {

}

// Allocate memory for a given number of components
void FixedJointComponents::allocate(uint32 nbComponentsToAllocate) {

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
    Entity* newJointEntities = static_cast<Entity*>(newBuffer);
    FixedJoint** newJoints = reinterpret_cast<FixedJoint**>(MemoryAllocator::alignAddress(newJointEntities + nbComponentsToAllocate, GLOBAL_ALIGNMENT));
    assert(reinterpret_cast<uintptr_t>(newJoints) % GLOBAL_ALIGNMENT == 0);
    Vector3* newLocalAnchorPointBody1 = reinterpret_cast<Vector3*>(MemoryAllocator::alignAddress(newJoints + nbComponentsToAllocate, GLOBAL_ALIGNMENT));
    assert(reinterpret_cast<uintptr_t>(newLocalAnchorPointBody1) % GLOBAL_ALIGNMENT == 0);
    Vector3* newLocalAnchorPointBody2 = reinterpret_cast<Vector3*>(MemoryAllocator::alignAddress(newLocalAnchorPointBody1 + nbComponentsToAllocate, GLOBAL_ALIGNMENT));
    assert(reinterpret_cast<uintptr_t>(newLocalAnchorPointBody2) % GLOBAL_ALIGNMENT == 0);
    Vector3* newR1World = reinterpret_cast<Vector3*>(MemoryAllocator::alignAddress(newLocalAnchorPointBody2 + nbComponentsToAllocate, GLOBAL_ALIGNMENT));
    assert(reinterpret_cast<uintptr_t>(newR1World) % GLOBAL_ALIGNMENT == 0);
    Vector3* newR2World = reinterpret_cast<Vector3*>(MemoryAllocator::alignAddress(newR1World + nbComponentsToAllocate, GLOBAL_ALIGNMENT));
    assert(reinterpret_cast<uintptr_t>(newR2World) % GLOBAL_ALIGNMENT == 0);
    Matrix3x3* newI1 = reinterpret_cast<Matrix3x3*>(MemoryAllocator::alignAddress(newR2World + nbComponentsToAllocate, GLOBAL_ALIGNMENT));
    assert(reinterpret_cast<uintptr_t>(newI1) % GLOBAL_ALIGNMENT == 0);
    Matrix3x3* newI2 = reinterpret_cast<Matrix3x3*>(MemoryAllocator::alignAddress(newI1 + nbComponentsToAllocate, GLOBAL_ALIGNMENT));
    assert(reinterpret_cast<uintptr_t>(newI1) % GLOBAL_ALIGNMENT == 0);
    Vector3* newImpulseTranslation = reinterpret_cast<Vector3*>(MemoryAllocator::alignAddress(newI2 + nbComponentsToAllocate, GLOBAL_ALIGNMENT));
    assert(reinterpret_cast<uintptr_t>(newImpulseTranslation) % GLOBAL_ALIGNMENT == 0);
    Vector3* newImpulseRotation = reinterpret_cast<Vector3*>(MemoryAllocator::alignAddress(newImpulseTranslation + nbComponentsToAllocate, GLOBAL_ALIGNMENT));
    assert(reinterpret_cast<uintptr_t>(newImpulseRotation) % GLOBAL_ALIGNMENT == 0);
    Matrix3x3* newInverseMassMatrixTranslation = reinterpret_cast<Matrix3x3*>(MemoryAllocator::alignAddress(newImpulseRotation + nbComponentsToAllocate, GLOBAL_ALIGNMENT));
    assert(reinterpret_cast<uintptr_t>(newInverseMassMatrixTranslation) % GLOBAL_ALIGNMENT == 0);
    Matrix3x3* newInverseMassMatrixRotation = reinterpret_cast<Matrix3x3*>(MemoryAllocator::alignAddress(newInverseMassMatrixTranslation + nbComponentsToAllocate, GLOBAL_ALIGNMENT));
    assert(reinterpret_cast<uintptr_t>(newInverseMassMatrixRotation) % GLOBAL_ALIGNMENT == 0);
    Vector3* newBiasTranslation = reinterpret_cast<Vector3*>(MemoryAllocator::alignAddress(newInverseMassMatrixRotation + nbComponentsToAllocate, GLOBAL_ALIGNMENT));
    assert(reinterpret_cast<uintptr_t>(newBiasTranslation) % GLOBAL_ALIGNMENT == 0);
    Vector3* newBiasRotation = reinterpret_cast<Vector3*>(MemoryAllocator::alignAddress(newBiasTranslation + nbComponentsToAllocate, GLOBAL_ALIGNMENT));
    assert(reinterpret_cast<uintptr_t>(newBiasRotation) % GLOBAL_ALIGNMENT == 0);
    Quaternion* newInitOrientationDifferenceInv = reinterpret_cast<Quaternion*>(MemoryAllocator::alignAddress(newBiasRotation + nbComponentsToAllocate, GLOBAL_ALIGNMENT));
    assert(reinterpret_cast<uintptr_t>(newInitOrientationDifferenceInv) % GLOBAL_ALIGNMENT == 0);
    assert(reinterpret_cast<uintptr_t>(newInitOrientationDifferenceInv + nbComponentsToAllocate) <= reinterpret_cast<uintptr_t>(newBuffer) + totalSizeBytes);

    // If there was already components before
    if (mNbComponents > 0) {

        // Copy component data from the previous buffer to the new one
        memcpy(newJointEntities, mJointEntities, mNbComponents * sizeof(Entity));
        memcpy(newJoints, mJoints, mNbComponents * sizeof(FixedJoint*));
        memcpy(newLocalAnchorPointBody1, mLocalAnchorPointBody1, mNbComponents * sizeof(Vector3));
        memcpy(newLocalAnchorPointBody2, mLocalAnchorPointBody2, mNbComponents * sizeof(Vector3));
        memcpy(newR1World, mR1World, mNbComponents * sizeof(Vector3));
        memcpy(newR2World, mR2World, mNbComponents * sizeof(Vector3));
        memcpy(newI1, mI1, mNbComponents * sizeof(Matrix3x3));
        memcpy(newI2, mI2, mNbComponents * sizeof(Matrix3x3));
        memcpy(newImpulseTranslation, mImpulseTranslation, mNbComponents * sizeof(Vector3));
        memcpy(newImpulseRotation, mImpulseRotation, mNbComponents * sizeof(Vector3));
        memcpy(newInverseMassMatrixTranslation, mInverseMassMatrixTranslation, mNbComponents * sizeof(Matrix3x3));
        memcpy(newInverseMassMatrixRotation, mInverseMassMatrixRotation, mNbComponents * sizeof(Matrix3x3));
        memcpy(newBiasTranslation, mBiasTranslation, mNbComponents * sizeof(Vector3));
        memcpy(newBiasRotation, mBiasRotation, mNbComponents * sizeof(Vector3));
        memcpy(newInitOrientationDifferenceInv, mInitOrientationDifferenceInv, mNbComponents * sizeof(Quaternion));

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
    mImpulseTranslation = newImpulseTranslation;
    mImpulseRotation = newImpulseRotation;
    mInverseMassMatrixTranslation = newInverseMassMatrixTranslation;
    mInverseMassMatrixRotation = newInverseMassMatrixRotation;
    mBiasTranslation = newBiasTranslation;
    mBiasRotation = newBiasRotation;
    mInitOrientationDifferenceInv = newInitOrientationDifferenceInv;
}

// Add a component
void FixedJointComponents::addComponent(Entity jointEntity, bool isDisabled, const FixedJointComponent& /*component*/) {

    // Prepare to add new component (allocate memory if necessary and compute insertion index)
    uint32 index = prepareAddComponent(isDisabled);

    // Insert the new component data
    new (mJointEntities + index) Entity(jointEntity);
    mJoints[index] = nullptr;
    new (mLocalAnchorPointBody1 + index) Vector3(0, 0, 0);
    new (mLocalAnchorPointBody2 + index) Vector3(0, 0, 0);
    new (mR1World + index) Vector3(0, 0, 0);
    new (mR2World + index) Vector3(0, 0, 0);
    new (mI1 + index) Matrix3x3();
    new (mI2 + index) Matrix3x3();
    new (mImpulseTranslation + index) Vector3(0, 0, 0);
    new (mImpulseRotation + index) Vector3(0, 0, 0);
    new (mInverseMassMatrixTranslation + index) Matrix3x3();
    new (mInverseMassMatrixRotation + index) Matrix3x3();
    new (mBiasTranslation + index) Vector3(0, 0, 0);
    new (mBiasRotation + index) Vector3(0, 0, 0);
    new (mInitOrientationDifferenceInv + index) Quaternion(0, 0, 0, 0);

    // Map the entity with the new component lookup index
    mMapEntityToComponentIndex.add(Pair<Entity, uint32>(jointEntity, index));

    mNbComponents++;

    assert(mDisabledStartIndex <= mNbComponents);
    assert(mNbComponents == static_cast<uint32>(mMapEntityToComponentIndex.size()));
}

// Move a component from a source to a destination index in the components array
// The destination location must contain a constructed object
void FixedJointComponents::moveComponentToIndex(uint32 srcIndex, uint32 destIndex) {

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
    new (mImpulseTranslation + destIndex) Vector3(mImpulseTranslation[srcIndex]);
    new (mImpulseRotation + destIndex) Vector3(mImpulseRotation[srcIndex]);
    new (mInverseMassMatrixTranslation + destIndex) Matrix3x3(mInverseMassMatrixTranslation[srcIndex]);
    new (mInverseMassMatrixRotation + destIndex) Matrix3x3(mInverseMassMatrixRotation[srcIndex]);
    new (mBiasTranslation + destIndex) Vector3(mBiasTranslation[srcIndex]);
    new (mBiasRotation + destIndex) Vector3(mBiasRotation[srcIndex]);
    new (mInitOrientationDifferenceInv + destIndex) Quaternion(mInitOrientationDifferenceInv[srcIndex]);

    // Destroy the source component
    destroyComponent(srcIndex);

    assert(!mMapEntityToComponentIndex.containsKey(entity));

    // Update the entity to component index mapping
    mMapEntityToComponentIndex.add(Pair<Entity, uint32>(entity, destIndex));

    assert(mMapEntityToComponentIndex[mJointEntities[destIndex]] == destIndex);
}

// Swap two components in the array
void FixedJointComponents::swapComponents(uint32 index1, uint32 index2) {

    // Copy component 1 data
    Entity jointEntity1(mJointEntities[index1]);
    FixedJoint* joint1 = mJoints[index1];
    Vector3 localAnchorPointBody1(mLocalAnchorPointBody1[index1]);
    Vector3 localAnchorPointBody2(mLocalAnchorPointBody2[index1]);
    Vector3 r1World1(mR1World[index1]);
    Vector3 r2World1(mR2World[index1]);
    Matrix3x3 i11(mI1[index1]);
    Matrix3x3 i21(mI2[index1]);
    Vector3 impulseTranslation1(mImpulseTranslation[index1]);
    Vector3 impulseRotation1(mImpulseRotation[index1]);
    Matrix3x3 inverseMassMatrixTranslation1(mInverseMassMatrixTranslation[index1]);
    Matrix3x3 inverseMassMatrixRotation1(mInverseMassMatrixRotation[index1]);
    Vector3 biasTranslation1(mBiasTranslation[index1]);
    Vector3 biasRotation1(mBiasRotation[index1]);
    Quaternion initOrientationDifferenceInv1(mInitOrientationDifferenceInv[index1]);

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
    new (mImpulseTranslation + index2) Vector3(impulseTranslation1);
    new (mImpulseRotation + index2) Vector3(impulseRotation1);
    new (mInverseMassMatrixTranslation + index2) Matrix3x3(inverseMassMatrixTranslation1);
    new (mInverseMassMatrixRotation + index2) Matrix3x3(inverseMassMatrixRotation1);
    new (mBiasTranslation + index2) Vector3(biasTranslation1);
    new (mBiasRotation + index2) Vector3(biasRotation1);
    new (mInitOrientationDifferenceInv + index2) Quaternion(initOrientationDifferenceInv1);

    // Update the entity to component index mapping
    mMapEntityToComponentIndex.add(Pair<Entity, uint32>(jointEntity1, index2));

    assert(mMapEntityToComponentIndex[mJointEntities[index1]] == index1);
    assert(mMapEntityToComponentIndex[mJointEntities[index2]] == index2);
    assert(mNbComponents == static_cast<uint32>(mMapEntityToComponentIndex.size()));
}

// Destroy a component at a given index
void FixedJointComponents::destroyComponent(uint32 index) {

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
    mImpulseTranslation[index].~Vector3();
    mImpulseRotation[index].~Vector3();
    mInverseMassMatrixTranslation[index].~Matrix3x3();
    mInverseMassMatrixRotation[index].~Matrix3x3();
    mBiasTranslation[index].~Vector3();
    mBiasRotation[index].~Vector3();
    mInitOrientationDifferenceInv[index].~Quaternion();
}
