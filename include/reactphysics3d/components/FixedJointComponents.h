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

#ifndef REACTPHYSICS3D_FIXED_JOINT_COMPONENTS_H
#define REACTPHYSICS3D_FIXED_JOINT_COMPONENTS_H

// Libraries
#include <reactphysics3d/mathematics/Transform.h>
#include <reactphysics3d/mathematics/Matrix3x3.h>
#include <reactphysics3d/engine/Entity.h>
#include <reactphysics3d/components/Components.h>
#include <reactphysics3d/containers/Map.h>

// ReactPhysics3D namespace
namespace reactphysics3d {

// Class declarations
class MemoryAllocator;
class EntityManager;
class FixedJoint;
enum class JointType;

// Class FixedJointComponents
/**
 * This class represent the component of the ECS with data for the FixedJoint.
 */
class FixedJointComponents : public Components {

    private:

        // -------------------- Attributes -------------------- //

        /// Array of joint entities
        Entity* mJointEntities;

        /// Array of pointers to the joints
        FixedJoint** mJoints;

        /// Anchor point of body 1 (in local-space coordinates of body 1)
        Vector3* mLocalAnchorPointBody1;

        /// Anchor point of body 2 (in local-space coordinates of body 2)
        Vector3* mLocalAnchorPointBody2;

        /// Vector from center of body 2 to anchor point in world-space
        Vector3* mR1World;

        /// Vector from center of body 2 to anchor point in world-space
        Vector3* mR2World;

        /// Inertia tensor of body 1 (in world-space coordinates)
        Matrix3x3* mI1;

        /// Inertia tensor of body 2 (in world-space coordinates)
        Matrix3x3* mI2;

        /// Accumulated impulse for the 3 translation constraints
        Vector3* mImpulseTranslation;

        /// Accumulate impulse for the 3 rotation constraints
        Vector3* mImpulseRotation;

        /// Inverse mass matrix K=JM^-1J^-t of the 3 translation constraints (3x3 matrix)
        Matrix3x3* mInverseMassMatrixTranslation;

        /// Inverse mass matrix K=JM^-1J^-t of the 3 rotation constraints (3x3 matrix)
        Matrix3x3* mInverseMassMatrixRotation;

        /// Bias vector for the 3 translation constraints
        Vector3* mBiasTranslation;

        /// Bias vector for the 3 rotation constraints
        Vector3* mBiasRotation;

        /// Inverse of the initial orientation difference between the two bodies
        Quaternion* mInitOrientationDifferenceInv;

        // -------------------- Methods -------------------- //

        /// Allocate memory for a given number of components
        virtual void allocate(uint32 nbComponentsToAllocate) override;

        /// Destroy a component at a given index
        virtual void destroyComponent(uint32 index) override;

        /// Move a component from a source to a destination index in the components array
        virtual void moveComponentToIndex(uint32 srcIndex, uint32 destIndex) override;

        /// Swap two components in the array
        virtual void swapComponents(uint32 index1, uint32 index2) override;

    public:

        /// Structure for the data of a transform component
        struct FixedJointComponent {

            /// Constructor
            FixedJointComponent() {

            }
        };

        // -------------------- Methods -------------------- //

        /// Constructor
        FixedJointComponents(MemoryAllocator& allocator);

        /// Destructor
        virtual ~FixedJointComponents() override = default;

        /// Add a component
        void addComponent(Entity jointEntity, bool isSleeping, const FixedJointComponent& component);

        /// Return a pointer to a given joint
        FixedJoint* getJoint(Entity jointEntity) const;

        /// Set the joint pointer to a given joint
        void setJoint(Entity jointEntity, FixedJoint* joint) const;

        /// Return the local anchor point of body 1 for a given joint
        const Vector3& getLocalAnchorPointBody1(Entity jointEntity) const;

        /// Set the local anchor point of body 1 for a given joint
        void setLocalAnchorPointBody1(Entity jointEntity, const Vector3& localAnchorPointBody1);

        /// Return the local anchor point of body 2 for a given joint
        const Vector3& getLocalAnchorPointBody2(Entity jointEntity) const;

        /// Set the local anchor point of body 2 for a given joint
        void setLocalAnchorPointBody2(Entity jointEntity, const Vector3& localAnchoirPointBody2);

        /// Return the vector from center of body 1 to anchor point in world-space
        const Vector3& getR1World(Entity jointEntity) const;

        /// Set the vector from center of body 1 to anchor point in world-space
        void setR1World(Entity jointEntity, const Vector3& r1World);

        /// Return the vector from center of body 2 to anchor point in world-space
        const Vector3& getR2World(Entity jointEntity) const;

        /// Set the vector from center of body 2 to anchor point in world-space
        void setR2World(Entity jointEntity, const Vector3& r2World);

        /// Return the inertia tensor of body 1 (in world-space coordinates)
        const Matrix3x3& getI1(Entity jointEntity) const;

        /// Set the inertia tensor of body 1 (in world-space coordinates)
        void setI1(Entity jointEntity, const Matrix3x3& i1);

        /// Return the inertia tensor of body 2 (in world-space coordinates)
        const Matrix3x3& getI2(Entity jointEntity) const;

        /// Set the inertia tensor of body 2 (in world-space coordinates)
        void setI2(Entity jointEntity, const Matrix3x3& i2);

        /// Return the translation impulse
        Vector3& getImpulseTranslation(Entity jointEntity);

        /// Set the translation impulse
        void setImpulseTranslation(Entity jointEntity, const Vector3& impulseTranslation);

        /// Return the translation impulse
        Vector3& getImpulseRotation(Entity jointEntity);

        /// Set the translation impulse
        void setImpulseRotation(Entity jointEntity, const Vector3& impulseTranslation);

        /// Return the translation inverse mass matrix of the constraint
        Matrix3x3& getInverseMassMatrixTranslation(Entity jointEntity);

        /// Set the translation inverse mass matrix of the constraint
        void setInverseMassMatrixTranslation(Entity jointEntity, const Matrix3x3& inverseMassMatrix);

        /// Return the rotation inverse mass matrix of the constraint
        Matrix3x3& getInverseMassMatrixRotation(Entity jointEntity);

        /// Set the rotation inverse mass matrix of the constraint
        void setInverseMassMatrixRotation(Entity jointEntity, const Matrix3x3& inverseMassMatrix);

        /// Return the translation bias
        Vector3& getBiasTranslation(Entity jointEntity);

        /// Set the translation impulse
        void setBiasTranslation(Entity jointEntity, const Vector3& impulseTranslation);

        /// Return the rotation bias
        Vector3& getBiasRotation(Entity jointEntity);

        /// Set the rotation impulse
        void setBiasRotation(Entity jointEntity, const Vector3 &impulseRotation);

        /// Return the initial orientation difference
        Quaternion& getInitOrientationDifferenceInv(Entity jointEntity);

        /// Set the rotation impulse
        void setInitOrientationDifferenceInv(Entity jointEntity, const Quaternion& initOrientationDifferenceInv);

        // -------------------- Friendship -------------------- //

        friend class BroadPhaseSystem;
        friend class SolveFixedJointSystem;
};

// Return a pointer to a given joint
RP3D_FORCE_INLINE FixedJoint* FixedJointComponents::getJoint(Entity jointEntity) const {

    assert(mMapEntityToComponentIndex.containsKey(jointEntity));
    return mJoints[mMapEntityToComponentIndex[jointEntity]];
}

// Set the joint pointer to a given joint
RP3D_FORCE_INLINE void FixedJointComponents::setJoint(Entity jointEntity, FixedJoint* joint) const {

    assert(mMapEntityToComponentIndex.containsKey(jointEntity));
    mJoints[mMapEntityToComponentIndex[jointEntity]] = joint;
}

// Return the local anchor point of body 1 for a given joint
RP3D_FORCE_INLINE const Vector3& FixedJointComponents::getLocalAnchorPointBody1(Entity jointEntity) const {

    assert(mMapEntityToComponentIndex.containsKey(jointEntity));
    return mLocalAnchorPointBody1[mMapEntityToComponentIndex[jointEntity]];
}

// Set the local anchor point of body 1 for a given joint
RP3D_FORCE_INLINE void FixedJointComponents::setLocalAnchorPointBody1(Entity jointEntity, const Vector3& localAnchorPointBody1) {

    assert(mMapEntityToComponentIndex.containsKey(jointEntity));
    mLocalAnchorPointBody1[mMapEntityToComponentIndex[jointEntity]] = localAnchorPointBody1;
}

// Return the local anchor point of body 2 for a given joint
RP3D_FORCE_INLINE const Vector3& FixedJointComponents::getLocalAnchorPointBody2(Entity jointEntity) const {

    assert(mMapEntityToComponentIndex.containsKey(jointEntity));
    return mLocalAnchorPointBody2[mMapEntityToComponentIndex[jointEntity]];
}

// Set the local anchor point of body 2 for a given joint
RP3D_FORCE_INLINE void FixedJointComponents::setLocalAnchorPointBody2(Entity jointEntity, const Vector3& localAnchorPointBody2) {

    assert(mMapEntityToComponentIndex.containsKey(jointEntity));
    mLocalAnchorPointBody2[mMapEntityToComponentIndex[jointEntity]] = localAnchorPointBody2;
}

// Return the vector from center of body 1 to anchor point in world-space
RP3D_FORCE_INLINE const Vector3& FixedJointComponents::getR1World(Entity jointEntity) const {

    assert(mMapEntityToComponentIndex.containsKey(jointEntity));
    return mR1World[mMapEntityToComponentIndex[jointEntity]];
}

// Set the vector from center of body 1 to anchor point in world-space
RP3D_FORCE_INLINE void FixedJointComponents::setR1World(Entity jointEntity, const Vector3& r1World) {

    assert(mMapEntityToComponentIndex.containsKey(jointEntity));
    mR1World[mMapEntityToComponentIndex[jointEntity]] = r1World;
}

// Return the vector from center of body 2 to anchor point in world-space
RP3D_FORCE_INLINE const Vector3& FixedJointComponents::getR2World(Entity jointEntity) const {

    assert(mMapEntityToComponentIndex.containsKey(jointEntity));
    return mR2World[mMapEntityToComponentIndex[jointEntity]];
}

// Set the vector from center of body 2 to anchor point in world-space
RP3D_FORCE_INLINE void FixedJointComponents::setR2World(Entity jointEntity, const Vector3& r2World) {

    assert(mMapEntityToComponentIndex.containsKey(jointEntity));
    mR2World[mMapEntityToComponentIndex[jointEntity]] = r2World;
}

// Return the inertia tensor of body 1 (in world-space coordinates)
RP3D_FORCE_INLINE const Matrix3x3& FixedJointComponents::getI1(Entity jointEntity) const {

    assert(mMapEntityToComponentIndex.containsKey(jointEntity));
    return mI1[mMapEntityToComponentIndex[jointEntity]];
}

// Set the inertia tensor of body 1 (in world-space coordinates)
RP3D_FORCE_INLINE void FixedJointComponents::setI1(Entity jointEntity, const Matrix3x3& i1) {

    assert(mMapEntityToComponentIndex.containsKey(jointEntity));
    mI1[mMapEntityToComponentIndex[jointEntity]] = i1;
}

// Return the inertia tensor of body 2 (in world-space coordinates)
RP3D_FORCE_INLINE const Matrix3x3& FixedJointComponents::getI2(Entity jointEntity) const {

    assert(mMapEntityToComponentIndex.containsKey(jointEntity));
    return mI2[mMapEntityToComponentIndex[jointEntity]];
}

// Set the inertia tensor of body 2 (in world-space coordinates)
RP3D_FORCE_INLINE void FixedJointComponents::setI2(Entity jointEntity, const Matrix3x3& i2) {

    assert(mMapEntityToComponentIndex.containsKey(jointEntity));
    mI2[mMapEntityToComponentIndex[jointEntity]] = i2;
}

// Return the translation impulse
RP3D_FORCE_INLINE Vector3& FixedJointComponents::getImpulseTranslation(Entity jointEntity) {

    assert(mMapEntityToComponentIndex.containsKey(jointEntity));
    return mImpulseTranslation[mMapEntityToComponentIndex[jointEntity]];
}

// Set the translation impulse
RP3D_FORCE_INLINE void FixedJointComponents::setImpulseTranslation(Entity jointEntity, const Vector3& impulseTranslation) {

    assert(mMapEntityToComponentIndex.containsKey(jointEntity));
    mImpulseTranslation[mMapEntityToComponentIndex[jointEntity]] = impulseTranslation;
}

// Return the translation impulse
RP3D_FORCE_INLINE Vector3& FixedJointComponents::getImpulseRotation(Entity jointEntity) {

    assert(mMapEntityToComponentIndex.containsKey(jointEntity));
    return mImpulseRotation[mMapEntityToComponentIndex[jointEntity]];
}

// Set the translation impulse
RP3D_FORCE_INLINE void FixedJointComponents::setImpulseRotation(Entity jointEntity, const Vector3& impulseTranslation) {

    assert(mMapEntityToComponentIndex.containsKey(jointEntity));
    mImpulseRotation[mMapEntityToComponentIndex[jointEntity]] = impulseTranslation;
}

// Return the translation inverse mass matrix of the constraint
RP3D_FORCE_INLINE Matrix3x3& FixedJointComponents::getInverseMassMatrixTranslation(Entity jointEntity) {

    assert(mMapEntityToComponentIndex.containsKey(jointEntity));
    return mInverseMassMatrixTranslation[mMapEntityToComponentIndex[jointEntity]];
}


// Set the translation inverse mass matrix of the constraint
RP3D_FORCE_INLINE void FixedJointComponents::setInverseMassMatrixTranslation(Entity jointEntity, const Matrix3x3& inverseMassMatrix) {

    assert(mMapEntityToComponentIndex.containsKey(jointEntity));
    mInverseMassMatrixTranslation[mMapEntityToComponentIndex[jointEntity]] = inverseMassMatrix;
}

// Return the rotation inverse mass matrix of the constraint
RP3D_FORCE_INLINE Matrix3x3& FixedJointComponents::getInverseMassMatrixRotation(Entity jointEntity) {

    assert(mMapEntityToComponentIndex.containsKey(jointEntity));
    return mInverseMassMatrixRotation[mMapEntityToComponentIndex[jointEntity]];
}

// Set the rotation inverse mass matrix of the constraint
RP3D_FORCE_INLINE void FixedJointComponents::setInverseMassMatrixRotation(Entity jointEntity, const Matrix3x3& inverseMassMatrix) {

    assert(mMapEntityToComponentIndex.containsKey(jointEntity));
    mInverseMassMatrixRotation[mMapEntityToComponentIndex[jointEntity]] = inverseMassMatrix;
}

// Return the translation bias
RP3D_FORCE_INLINE Vector3& FixedJointComponents::getBiasTranslation(Entity jointEntity) {

    assert(mMapEntityToComponentIndex.containsKey(jointEntity));
    return mBiasTranslation[mMapEntityToComponentIndex[jointEntity]];
}

// Set the translation impulse
RP3D_FORCE_INLINE void FixedJointComponents::setBiasTranslation(Entity jointEntity, const Vector3 &impulseTranslation) {

    assert(mMapEntityToComponentIndex.containsKey(jointEntity));
    mBiasTranslation[mMapEntityToComponentIndex[jointEntity]] = impulseTranslation;
}

// Return the rotation bias
RP3D_FORCE_INLINE Vector3& FixedJointComponents::getBiasRotation(Entity jointEntity) {

    assert(mMapEntityToComponentIndex.containsKey(jointEntity));
    return mBiasRotation[mMapEntityToComponentIndex[jointEntity]];
}

// Set the rotation impulse
RP3D_FORCE_INLINE void FixedJointComponents::setBiasRotation(Entity jointEntity, const Vector3& impulseRotation) {

    assert(mMapEntityToComponentIndex.containsKey(jointEntity));
    mBiasRotation[mMapEntityToComponentIndex[jointEntity]] = impulseRotation;
}

// Return the initial orientation difference
RP3D_FORCE_INLINE Quaternion& FixedJointComponents::getInitOrientationDifferenceInv(Entity jointEntity) {

    assert(mMapEntityToComponentIndex.containsKey(jointEntity));
    return mInitOrientationDifferenceInv[mMapEntityToComponentIndex[jointEntity]];
}

// Set the rotation impulse
RP3D_FORCE_INLINE void FixedJointComponents::setInitOrientationDifferenceInv(Entity jointEntity, const Quaternion& initOrientationDifferenceInv) {

    assert(mMapEntityToComponentIndex.containsKey(jointEntity));
    mInitOrientationDifferenceInv[mMapEntityToComponentIndex[jointEntity]] = initOrientationDifferenceInv;
}

}

#endif
