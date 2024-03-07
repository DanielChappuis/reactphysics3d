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

#ifndef REACTPHYSICS3D_SLIDER_JOINT_COMPONENTS_H
#define REACTPHYSICS3D_SLIDER_JOINT_COMPONENTS_H

// Libraries
#include <reactphysics3d/mathematics/Transform.h>
#include <reactphysics3d/mathematics/Matrix3x3.h>
#include <reactphysics3d/mathematics/Matrix2x2.h>
#include <reactphysics3d/engine/Entity.h>
#include <reactphysics3d/components/Components.h>
#include <reactphysics3d/containers/Map.h>

// ReactPhysics3D namespace
namespace reactphysics3d {

// Class declarations
class MemoryAllocator;
class EntityManager;
class SliderJoint;
enum class JointType;

// Class SliderJointComponents
/**
 * This class represent the component of the ECS with data for the SliderJoint.
 */
class SliderJointComponents : public Components {

    private:

        // -------------------- Attributes -------------------- //

        /// Array of joint entities
        Entity* mJointEntities;

        /// Array of pointers to the joints
        SliderJoint** mJoints;

        /// Anchor point of body 1 (in local-space coordinates of body 1)
        Vector3* mLocalAnchorPointBody1;

        /// Anchor point of body 2 (in local-space coordinates of body 2)
        Vector3* mLocalAnchorPointBody2;

        /// Inertia tensor of body 1 (in world-space coordinates)
        Matrix3x3* mI1;

        /// Inertia tensor of body 2 (in world-space coordinates)
        Matrix3x3* mI2;

        /// Accumulated impulse for the 3 translation constraints
        Vector2* mImpulseTranslation;

        /// Accumulate impulse for the 3 rotation constraints
        Vector3* mImpulseRotation;

        /// Inverse mass matrix K=JM^-1J^-t of the 3 translation constraints (3x3 matrix)
        Matrix2x2* mInverseMassMatrixTranslation;

        /// Inverse mass matrix K=JM^-1J^-t of the 3 rotation constraints (3x3 matrix)
        Matrix3x3* mInverseMassMatrixRotation;

        /// Bias vector for the 3 translation constraints
        Vector2* mBiasTranslation;

        /// Bias vector for the 3 rotation constraints
        Vector3* mBiasRotation;

        /// Inverse of the initial orientation difference between the two bodies
        Quaternion* mInitOrientationDifferenceInv;

        /// Slider axis (in local-space coordinates of body 1)
        Vector3* mSliderAxisBody1;

        /// Slider axis in world-space coordinates
        Vector3* mSliderAxisWorld;

        /// Vector r1 in world-space coordinates
        Vector3* mR1;

        /// Vector r2 in world-space coordinates
        Vector3* mR2;

        /// First vector orthogonal to the slider axis local-space of body 1
        Vector3* mN1;

        /// Second vector orthogonal to the slider axis and mN1 in local-space of body 1
        Vector3* mN2;

        /// Accumulated impulse for the lower limit constraint
        decimal* mImpulseLowerLimit;

        /// Accumulated impulse for the upper limit constraint
        decimal* mImpulseUpperLimit;

        /// Accumulated impulse for the motor constraint;
        decimal* mImpulseMotor;

        /// Inverse of mass matrix K=JM^-1J^t for the upper and lower limit constraints (1x1 matrix)
        decimal* mInverseMassMatrixLimit;

        /// Inverse of mass matrix K=JM^-1J^t for the motor
        decimal* mInverseMassMatrixMotor;

        /// Bias of the lower limit constraint
        decimal* mBLowerLimit;

        /// Bias of the upper limit constraint
        decimal* mBUpperLimit;

        /// True if the joint limits are enabled
        bool* mIsLimitEnabled;

        /// True if the motor of the joint in enabled
        bool* mIsMotorEnabled;

        /// Lower limit (minimum allowed rotation angle in radian)
        decimal* mLowerLimit;

        /// Upper limit (maximum translation distance)
        decimal* mUpperLimit;

        /// True if the lower limit is violated
        bool* mIsLowerLimitViolated;

        /// True if the upper limit is violated
        bool* mIsUpperLimitViolated;

        /// Motor speed (in rad/s)
        decimal* mMotorSpeed;

        /// Maximum motor force (in Newtons) that can be applied to reach to desired motor speed
        decimal* mMaxMotorForce;

        /// Cross product of r2 and n1
        Vector3* mR2CrossN1;

        /// Cross product of r2 and n2
        Vector3* mR2CrossN2;

        /// Cross product of r2 and the slider axis
        Vector3* mR2CrossSliderAxis;

        /// Cross product of vector (r1 + u) and n1
        Vector3* mR1PlusUCrossN1;

        /// Cross product of vector (r1 + u) and n2
        Vector3* mR1PlusUCrossN2;

        /// Cross product of vector (r1 + u) and the slider axis
        Vector3* mR1PlusUCrossSliderAxis;

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
        struct SliderJointComponent {

             bool isLimitEnabled;
             bool isMotorEnabled;
             decimal lowerLimit;
             decimal upperLimit;
             decimal motorSpeed;
             decimal maxMotorForce;

            /// Constructor
            SliderJointComponent(bool isLimitEnabled, bool isMotorEnabled, decimal lowerLimit, decimal upperLimit,
                                 decimal motorSpeed, decimal maxMotorForce)
                :isLimitEnabled(isLimitEnabled), isMotorEnabled(isMotorEnabled), lowerLimit(lowerLimit), upperLimit(upperLimit),
                 motorSpeed(motorSpeed), maxMotorForce(maxMotorForce) {

            }
        };

        // -------------------- Methods -------------------- //

        /// Constructor
        SliderJointComponents(MemoryAllocator& allocator);

        /// Destructor
        virtual ~SliderJointComponents() override = default;

        /// Add a component
        void addComponent(Entity jointEntity, bool isDisabled, const SliderJointComponent& component);

        /// Return a pointer to a given joint
        SliderJoint* getJoint(Entity jointEntity) const;

        /// Set the joint pointer to a given joint
        void setJoint(Entity jointEntity, SliderJoint* joint) const;

        /// Return the local anchor point of body 1 for a given joint
        const Vector3& getLocalAnchorPointBody1(Entity jointEntity) const;

        /// Set the local anchor point of body 1 for a given joint
        void setLocalAnchorPointBody1(Entity jointEntity, const Vector3& localAnchoirPointBody1);

        /// Return the local anchor point of body 2 for a given joint
        const Vector3& getLocalAnchorPointBody2(Entity jointEntity) const;

        /// Set the local anchor point of body 2 for a given joint
        void setLocalAnchorPointBody2(Entity jointEntity, const Vector3& localAnchoirPointBody2);

        /// Return the inertia tensor of body 1 (in world-space coordinates)
        const Matrix3x3& getI1(Entity jointEntity) const;

        /// Set the inertia tensor of body 1 (in world-space coordinates)
        void setI1(Entity jointEntity, const Matrix3x3& i1);

        /// Return the inertia tensor of body 2 (in world-space coordinates)
        const Matrix3x3& getI2(Entity jointEntity) const;

        /// Set the inertia tensor of body 2 (in world-space coordinates)
        void setI2(Entity jointEntity, const Matrix3x3& i2);

        /// Return the translation impulse
        Vector2& getImpulseTranslation(Entity jointEntity);

        /// Set the translation impulse
        void setImpulseTranslation(Entity jointEntity, const Vector2& impulseTranslation);

        /// Return the translation impulse
        Vector3& getImpulseRotation(Entity jointEntity);

        /// Set the translation impulse
        void setImpulseRotation(Entity jointEntity, const Vector3& impulseTranslation);

        /// Return the translation inverse mass matrix of the constraint
        Matrix2x2& getInverseMassMatrixTranslation(Entity jointEntity);

        /// Set the translation inverse mass matrix of the constraint
        void setInverseMassMatrixTranslation(Entity jointEntity, const Matrix2x2& inverseMassMatrix);

        /// Return the rotation inverse mass matrix of the constraint
        Matrix3x3& getInverseMassMatrixRotation(Entity jointEntity);

        /// Set the rotation inverse mass matrix of the constraint
        void setInverseMassMatrixRotation(Entity jointEntity, const Matrix3x3& inverseMassMatrix);

        /// Return the translation bias
        Vector2& getBiasTranslation(Entity jointEntity);

        /// Set the translation impulse
        void setBiasTranslation(Entity jointEntity, const Vector2& impulseTranslation);

        /// Return the rotation bias
        Vector3& getBiasRotation(Entity jointEntity);

        /// Set the rotation impulse
        void setBiasRotation(Entity jointEntity, const Vector3& impulseRotation);

        /// Return the initial orientation difference
        Quaternion& getInitOrientationDifferenceInv(Entity jointEntity);

        /// Set the rotation impulse
        void setInitOrientationDifferenceInv(Entity jointEntity, const Quaternion& initOrientationDifferenceInv);

        /// Return the slider axis (in local-space coordinates of body 1)
        Vector3& getSliderAxisBody1(Entity jointEntity);

        /// Set the slider axis (in local-space coordinates of body 1)
        void setSliderAxisBody1(Entity jointEntity, const Vector3& sliderAxisBody1);

        /// Retunr the slider axis in world-space coordinates
        Vector3& getSliderAxisWorld(Entity jointEntity);

        /// Set the slider axis in world-space coordinates
        void setSliderAxisWorld(Entity jointEntity, const Vector3& sliderAxisWorld);

        /// Return the vector r1 in world-space coordinates
        Vector3& getR1(Entity jointEntity);

        /// Set the vector r1 in world-space coordinates
        void setR1(Entity jointEntity, const Vector3& r1);

        /// Return the vector r2 in world-space coordinates
        Vector3& getR2(Entity jointEntity);

        /// Set the vector r2 in world-space coordinates
        void setR2(Entity jointEntity, const Vector3& r2);

        /// Return the first vector orthogonal to the slider axis local-space of body 1
        Vector3& getN1(Entity jointEntity);

        /// Set the first vector orthogonal to the slider axis local-space of body 1
        void setN1(Entity jointEntity, const Vector3& n1);

        /// Return the second vector orthogonal to the slider axis and mN1 in local-space of body 1
        Vector3& getN2(Entity jointEntity);

        /// Set the second vector orthogonal to the slider axis and mN1 in local-space of body 1
        void setN2(Entity jointEntity, const Vector3& n2);

        /// Return the accumulated impulse for the lower limit constraint
        decimal getImpulseLowerLimit(Entity jointEntity) const;

        /// Set the accumulated impulse for the lower limit constraint
        void setImpulseLowerLimit(Entity jointEntity, decimal impulseLowerLimit);

        /// Return the accumulated impulse for the upper limit constraint
        decimal getImpulseUpperLimit(Entity jointEntity) const;

        /// Set the accumulated impulse for the upper limit constraint
        void setImpulseUpperLimit(Entity jointEntity, decimal impulseUpperLimit) const;

        /// Return the accumulated impulse for the motor constraint;
        decimal getImpulseMotor(Entity jointEntity) const;

        /// Set the accumulated impulse for the motor constraint;
        void setImpulseMotor(Entity jointEntity, decimal impulseMotor);

        /// Return the inverse of mass matrix K=JM^-1J^t for the limits (1x1 matrix)
        decimal getInverseMassMatrixLimit(Entity jointEntity) const;

        /// Set the inverse of mass matrix K=JM^-1J^t for the limits (1x1 matrix)
        void setInverseMassMatrixLimit(Entity jointEntity, decimal inverseMassMatrixLimitMotor);

        /// Return the inverse of mass matrix K=JM^-1J^t for the motor
        decimal getInverseMassMatrixMotor(Entity jointEntity);

        /// Set the inverse of mass matrix K=JM^-1J^t for the motor
        void setInverseMassMatrixMotor(Entity jointEntity, decimal inverseMassMatrixMotor);

        /// Return the bias of the lower limit constraint
        decimal getBLowerLimit(Entity jointEntity) const;

        /// Set the bias of the lower limit constraint
        void setBLowerLimit(Entity jointEntity, decimal bLowerLimit) const;

        /// Return the bias of the upper limit constraint
        decimal getBUpperLimit(Entity jointEntity) const;

        /// Set the bias of the upper limit constraint
        void setBUpperLimit(Entity jointEntity, decimal bUpperLimit);

        /// Return true if the joint limits are enabled
        bool getIsLimitEnabled(Entity jointEntity) const;

        /// Set to true if the joint limits are enabled
        void setIsLimitEnabled(Entity jointEntity, bool isLimitEnabled);

        /// Return true if the motor of the joint in enabled
        bool getIsMotorEnabled(Entity jointEntity) const;

        /// Set to true if the motor of the joint in enabled
        void setIsMotorEnabled(Entity jointEntity, bool isMotorEnabled) const;

        /// Return the Lower limit (minimum allowed rotation angle in radian)
        decimal getLowerLimit(Entity jointEntity) const;

        /// Set the Lower limit (minimum allowed rotation angle in radian)
        void setLowerLimit(Entity jointEntity, decimal lowerLimit) const;

        /// Return the upper limit (maximum translation distance)
        decimal getUpperLimit(Entity jointEntity) const;

        /// Set the upper limit (maximum translation distance)
        void setUpperLimit(Entity jointEntity, decimal upperLimit);

        /// Return true if the lower limit is violated
        bool getIsLowerLimitViolated(Entity jointEntity) const;

        /// Set to true if the lower limit is violated
        void setIsLowerLimitViolated(Entity jointEntity, bool isLowerLimitViolated);

        /// Return true if the upper limit is violated
        bool getIsUpperLimitViolated(Entity jointEntity) const;

        /// Set to true if the upper limit is violated
        void setIsUpperLimitViolated(Entity jointEntity, bool isUpperLimitViolated) const;

        /// Return the motor speed (in rad/s)
        decimal getMotorSpeed(Entity jointEntity) const;

        /// Set the motor speed (in rad/s)
        void setMotorSpeed(Entity jointEntity, decimal motorSpeed);

        /// Return the maximum motor force (in Newtons) that can be applied to reach to desired motor speed
        decimal getMaxMotorForce(Entity jointEntity) const;

        /// Set the maximum motor force (in Newtons) that can be applied to reach to desired motor speed
        void setMaxMotorForce(Entity jointEntity, decimal maxMotorForce);

        /// Return the cross product of r2 and n1
        Vector3& getR2CrossN1(Entity jointEntity);

        /// Set the cross product of r2 and n1
        void setR2CrossN1(Entity jointEntity, const Vector3& r2CrossN1);

        /// Return the cross product of r2 and n2
        Vector3& getR2CrossN2(Entity jointEntity);

        /// Set the cross product of r2 and n2
        void setR2CrossN2(Entity jointEntity, const Vector3& r2CrossN2);

        /// Return the cross product of r2 and the slider axis
        Vector3& getR2CrossSliderAxis(Entity jointEntity);

        /// Set the cross product of r2 and the slider axis
        void setR2CrossSliderAxis(Entity jointEntity, const Vector3& r2CrossSliderAxis);

        /// Return the cross product of vector (r1 + u) and n1
        Vector3& getR1PlusUCrossN1(Entity jointEntity);

        /// Set the cross product of vector (r1 + u) and n1
        void setR1PlusUCrossN1(Entity jointEntity, const Vector3& r1PlusUCrossN1);

        /// Return the cross product of vector (r1 + u) and n2
        Vector3& getR1PlusUCrossN2(Entity jointEntity);

        /// Set the cross product of vector (r1 + u) and n2
        void setR1PlusUCrossN2(Entity jointEntity, const Vector3& r1PlusUCrossN2);

        /// Return the cross product of vector (r1 + u) and the slider axis
        Vector3& getR1PlusUCrossSliderAxis(Entity jointEntity);

        /// Set the cross product of vector (r1 + u) and the slider axis
        void setR1PlusUCrossSliderAxis(Entity jointEntity, const Vector3& r1PlusUCrossSliderAxis);

        // -------------------- Friendship -------------------- //

        friend class BroadPhaseSystem;
        friend class SolveSliderJointSystem;
        friend class SliderJoint;
};

// Return a pointer to a given joint
RP3D_FORCE_INLINE SliderJoint* SliderJointComponents::getJoint(Entity jointEntity) const {

    assert(mMapEntityToComponentIndex.containsKey(jointEntity));
    return mJoints[mMapEntityToComponentIndex[jointEntity]];
}

// Set the joint pointer to a given joint
RP3D_FORCE_INLINE void SliderJointComponents::setJoint(Entity jointEntity, SliderJoint* joint) const {

    assert(mMapEntityToComponentIndex.containsKey(jointEntity));
    mJoints[mMapEntityToComponentIndex[jointEntity]] = joint;
}

// Return the local anchor point of body 1 for a given joint
RP3D_FORCE_INLINE const Vector3& SliderJointComponents::getLocalAnchorPointBody1(Entity jointEntity) const {

    assert(mMapEntityToComponentIndex.containsKey(jointEntity));
    return mLocalAnchorPointBody1[mMapEntityToComponentIndex[jointEntity]];
}

// Set the local anchor point of body 1 for a given joint
RP3D_FORCE_INLINE void SliderJointComponents::setLocalAnchorPointBody1(Entity jointEntity, const Vector3& localAnchoirPointBody1) {

    assert(mMapEntityToComponentIndex.containsKey(jointEntity));
    mLocalAnchorPointBody1[mMapEntityToComponentIndex[jointEntity]] = localAnchoirPointBody1;
}

// Return the local anchor point of body 2 for a given joint
RP3D_FORCE_INLINE const Vector3& SliderJointComponents::getLocalAnchorPointBody2(Entity jointEntity) const {

    assert(mMapEntityToComponentIndex.containsKey(jointEntity));
    return mLocalAnchorPointBody2[mMapEntityToComponentIndex[jointEntity]];
}

// Set the local anchor point of body 2 for a given joint
RP3D_FORCE_INLINE void SliderJointComponents::setLocalAnchorPointBody2(Entity jointEntity, const Vector3& localAnchoirPointBody2) {

    assert(mMapEntityToComponentIndex.containsKey(jointEntity));
    mLocalAnchorPointBody2[mMapEntityToComponentIndex[jointEntity]] = localAnchoirPointBody2;
}

// Return the inertia tensor of body 1 (in world-space coordinates)
RP3D_FORCE_INLINE const Matrix3x3& SliderJointComponents::getI1(Entity jointEntity) const {

    assert(mMapEntityToComponentIndex.containsKey(jointEntity));
    return mI1[mMapEntityToComponentIndex[jointEntity]];
}

// Set the inertia tensor of body 1 (in world-space coordinates)
RP3D_FORCE_INLINE void SliderJointComponents::setI1(Entity jointEntity, const Matrix3x3& i1) {

    assert(mMapEntityToComponentIndex.containsKey(jointEntity));
    mI1[mMapEntityToComponentIndex[jointEntity]] = i1;
}

// Return the inertia tensor of body 2 (in world-space coordinates)
RP3D_FORCE_INLINE const Matrix3x3& SliderJointComponents::getI2(Entity jointEntity) const {

    assert(mMapEntityToComponentIndex.containsKey(jointEntity));
    return mI2[mMapEntityToComponentIndex[jointEntity]];
}

// Set the inertia tensor of body 2 (in world-space coordinates)
RP3D_FORCE_INLINE void SliderJointComponents::setI2(Entity jointEntity, const Matrix3x3& i2) {

    assert(mMapEntityToComponentIndex.containsKey(jointEntity));
    mI2[mMapEntityToComponentIndex[jointEntity]] = i2;
}

// Return the translation impulse
RP3D_FORCE_INLINE Vector2& SliderJointComponents::getImpulseTranslation(Entity jointEntity) {

    assert(mMapEntityToComponentIndex.containsKey(jointEntity));
    return mImpulseTranslation[mMapEntityToComponentIndex[jointEntity]];
}

// Set the translation impulse
RP3D_FORCE_INLINE void SliderJointComponents::setImpulseTranslation(Entity jointEntity, const Vector2& impulseTranslation) {

    assert(mMapEntityToComponentIndex.containsKey(jointEntity));
    mImpulseTranslation[mMapEntityToComponentIndex[jointEntity]] = impulseTranslation;
}

// Return the translation impulse
RP3D_FORCE_INLINE Vector3& SliderJointComponents::getImpulseRotation(Entity jointEntity) {

    assert(mMapEntityToComponentIndex.containsKey(jointEntity));
    return mImpulseRotation[mMapEntityToComponentIndex[jointEntity]];
}

// Set the translation impulse
RP3D_FORCE_INLINE void SliderJointComponents::setImpulseRotation(Entity jointEntity, const Vector3& impulseTranslation) {

    assert(mMapEntityToComponentIndex.containsKey(jointEntity));
    mImpulseRotation[mMapEntityToComponentIndex[jointEntity]] = impulseTranslation;
}

// Return the translation inverse mass matrix of the constraint
RP3D_FORCE_INLINE Matrix2x2& SliderJointComponents::getInverseMassMatrixTranslation(Entity jointEntity) {

    assert(mMapEntityToComponentIndex.containsKey(jointEntity));
    return mInverseMassMatrixTranslation[mMapEntityToComponentIndex[jointEntity]];
}

// Set the translation inverse mass matrix of the constraint
RP3D_FORCE_INLINE void SliderJointComponents::setInverseMassMatrixTranslation(Entity jointEntity, const Matrix2x2& inverseMassMatrix) {

    assert(mMapEntityToComponentIndex.containsKey(jointEntity));
    mInverseMassMatrixTranslation[mMapEntityToComponentIndex[jointEntity]] = inverseMassMatrix;
}

// Return the rotation inverse mass matrix of the constraint
RP3D_FORCE_INLINE Matrix3x3& SliderJointComponents::getInverseMassMatrixRotation(Entity jointEntity) {

    assert(mMapEntityToComponentIndex.containsKey(jointEntity));
    return mInverseMassMatrixRotation[mMapEntityToComponentIndex[jointEntity]];
}

// Set the rotation inverse mass matrix of the constraint
RP3D_FORCE_INLINE void SliderJointComponents::setInverseMassMatrixRotation(Entity jointEntity, const Matrix3x3& inverseMassMatrix) {

    assert(mMapEntityToComponentIndex.containsKey(jointEntity));
    mInverseMassMatrixRotation[mMapEntityToComponentIndex[jointEntity]] = inverseMassMatrix;
}

// Return the translation bias
RP3D_FORCE_INLINE Vector2& SliderJointComponents::getBiasTranslation(Entity jointEntity) {

    assert(mMapEntityToComponentIndex.containsKey(jointEntity));
    return mBiasTranslation[mMapEntityToComponentIndex[jointEntity]];
}

// Set the translation impulse
RP3D_FORCE_INLINE void SliderJointComponents::setBiasTranslation(Entity jointEntity, const Vector2& impulseTranslation) {

    assert(mMapEntityToComponentIndex.containsKey(jointEntity));
    mBiasTranslation[mMapEntityToComponentIndex[jointEntity]] = impulseTranslation;
}

// Return the rotation bias
RP3D_FORCE_INLINE Vector3& SliderJointComponents::getBiasRotation(Entity jointEntity) {

    assert(mMapEntityToComponentIndex.containsKey(jointEntity));
    return mBiasRotation[mMapEntityToComponentIndex[jointEntity]];
}

// Set the rotation impulse
RP3D_FORCE_INLINE void SliderJointComponents::setBiasRotation(Entity jointEntity, const Vector3& impulseRotation) {

    assert(mMapEntityToComponentIndex.containsKey(jointEntity));
    mBiasRotation[mMapEntityToComponentIndex[jointEntity]] = impulseRotation;
}

// Return the initial orientation difference
RP3D_FORCE_INLINE Quaternion& SliderJointComponents::getInitOrientationDifferenceInv(Entity jointEntity) {

    assert(mMapEntityToComponentIndex.containsKey(jointEntity));
    return mInitOrientationDifferenceInv[mMapEntityToComponentIndex[jointEntity]];
}

// Set the rotation impulse
RP3D_FORCE_INLINE void SliderJointComponents::setInitOrientationDifferenceInv(Entity jointEntity, const Quaternion& initOrientationDifferenceInv) {

    assert(mMapEntityToComponentIndex.containsKey(jointEntity));
    mInitOrientationDifferenceInv[mMapEntityToComponentIndex[jointEntity]] = initOrientationDifferenceInv;
}

// Return the slider axis (in local-space coordinates of body 1)
RP3D_FORCE_INLINE Vector3& SliderJointComponents::getSliderAxisBody1(Entity jointEntity) {

    assert(mMapEntityToComponentIndex.containsKey(jointEntity));
    return mSliderAxisBody1[mMapEntityToComponentIndex[jointEntity]];
}

// Set the slider axis (in local-space coordinates of body 1)
RP3D_FORCE_INLINE void SliderJointComponents::setSliderAxisBody1(Entity jointEntity, const Vector3& sliderAxisBody1) {

    assert(mMapEntityToComponentIndex.containsKey(jointEntity));
    mSliderAxisBody1[mMapEntityToComponentIndex[jointEntity]] = sliderAxisBody1;
}

// Retunr the slider axis in world-space coordinates
RP3D_FORCE_INLINE Vector3& SliderJointComponents::getSliderAxisWorld(Entity jointEntity) {

    assert(mMapEntityToComponentIndex.containsKey(jointEntity));
    return mSliderAxisWorld[mMapEntityToComponentIndex[jointEntity]];
}

// Set the slider axis in world-space coordinates
RP3D_FORCE_INLINE void SliderJointComponents::setSliderAxisWorld(Entity jointEntity, const Vector3& sliderAxisWorld) {

    assert(mMapEntityToComponentIndex.containsKey(jointEntity));
    mSliderAxisWorld[mMapEntityToComponentIndex[jointEntity]] = sliderAxisWorld;
}

// Return the vector r1 in world-space coordinates
RP3D_FORCE_INLINE Vector3& SliderJointComponents::getR1(Entity jointEntity) {

    assert(mMapEntityToComponentIndex.containsKey(jointEntity));
    return mR1[mMapEntityToComponentIndex[jointEntity]];
}

// Set the vector r1 in world-space coordinates
RP3D_FORCE_INLINE void SliderJointComponents::setR1(Entity jointEntity, const Vector3& r1) {

    assert(mMapEntityToComponentIndex.containsKey(jointEntity));
    mR1[mMapEntityToComponentIndex[jointEntity]] = r1;
}

// Return the vector r2 in world-space coordinates
RP3D_FORCE_INLINE Vector3& SliderJointComponents::getR2(Entity jointEntity) {

    assert(mMapEntityToComponentIndex.containsKey(jointEntity));
    return mR2[mMapEntityToComponentIndex[jointEntity]];
}

// Set the vector r2 in world-space coordinates
RP3D_FORCE_INLINE void SliderJointComponents::setR2(Entity jointEntity, const Vector3& r2) {

    assert(mMapEntityToComponentIndex.containsKey(jointEntity));
    mR2[mMapEntityToComponentIndex[jointEntity]] = r2;
}

// Return the first vector orthogonal to the slider axis local-space of body 1
RP3D_FORCE_INLINE Vector3& SliderJointComponents::getN1(Entity jointEntity) {

    assert(mMapEntityToComponentIndex.containsKey(jointEntity));
    return mN1[mMapEntityToComponentIndex[jointEntity]];
}

// Set the first vector orthogonal to the slider axis local-space of body 1
RP3D_FORCE_INLINE void SliderJointComponents::setN1(Entity jointEntity, const Vector3& n1) {

    assert(mMapEntityToComponentIndex.containsKey(jointEntity));
    mN1[mMapEntityToComponentIndex[jointEntity]] = n1;
}

// Return the second vector orthogonal to the slider axis and mN1 in local-space of body 1
RP3D_FORCE_INLINE Vector3& SliderJointComponents::getN2(Entity jointEntity) {

    assert(mMapEntityToComponentIndex.containsKey(jointEntity));
    return mN2[mMapEntityToComponentIndex[jointEntity]];
}

// Set the second vector orthogonal to the slider axis and mN1 in local-space of body 1
RP3D_FORCE_INLINE void SliderJointComponents::setN2(Entity jointEntity, const Vector3& n2) {

    assert(mMapEntityToComponentIndex.containsKey(jointEntity));
    mN2[mMapEntityToComponentIndex[jointEntity]] = n2;
}

// Return the accumulated impulse for the lower limit constraint
RP3D_FORCE_INLINE decimal SliderJointComponents::getImpulseLowerLimit(Entity jointEntity) const {

    assert(mMapEntityToComponentIndex.containsKey(jointEntity));
    return mImpulseLowerLimit[mMapEntityToComponentIndex[jointEntity]];
}

// Set the accumulated impulse for the lower limit constraint
RP3D_FORCE_INLINE void SliderJointComponents::setImpulseLowerLimit(Entity jointEntity, decimal impulseLowerLimit) {

    assert(mMapEntityToComponentIndex.containsKey(jointEntity));
    mImpulseLowerLimit[mMapEntityToComponentIndex[jointEntity]] = impulseLowerLimit;
}


// Return the accumulated impulse for the upper limit constraint
RP3D_FORCE_INLINE decimal SliderJointComponents::getImpulseUpperLimit(Entity jointEntity) const {

    assert(mMapEntityToComponentIndex.containsKey(jointEntity));
    return mImpulseUpperLimit[mMapEntityToComponentIndex[jointEntity]];
}

// Set the accumulated impulse for the upper limit constraint
RP3D_FORCE_INLINE void SliderJointComponents::setImpulseUpperLimit(Entity jointEntity, decimal impulseUpperLimit) const {

    assert(mMapEntityToComponentIndex.containsKey(jointEntity));
    mImpulseUpperLimit[mMapEntityToComponentIndex[jointEntity]] = impulseUpperLimit;
}


// Return the accumulated impulse for the motor constraint;
RP3D_FORCE_INLINE decimal SliderJointComponents::getImpulseMotor(Entity jointEntity) const {

    assert(mMapEntityToComponentIndex.containsKey(jointEntity));
    return mImpulseMotor[mMapEntityToComponentIndex[jointEntity]];
}

// Set the accumulated impulse for the motor constraint;
RP3D_FORCE_INLINE void SliderJointComponents::setImpulseMotor(Entity jointEntity, decimal impulseMotor) {

    assert(mMapEntityToComponentIndex.containsKey(jointEntity));
    mImpulseMotor[mMapEntityToComponentIndex[jointEntity]] = impulseMotor;
}

// Return the inverse of mass matrix K=JM^-1J^t for the limits (1x1 matrix)
RP3D_FORCE_INLINE decimal SliderJointComponents::getInverseMassMatrixLimit(Entity jointEntity) const {

    assert(mMapEntityToComponentIndex.containsKey(jointEntity));
    return mInverseMassMatrixLimit[mMapEntityToComponentIndex[jointEntity]];
}

// Set the inverse of mass matrix K=JM^-1J^t for the limits (1x1 matrix)
RP3D_FORCE_INLINE void SliderJointComponents::setInverseMassMatrixLimit(Entity jointEntity, decimal inverseMassMatrixLimitMotor) {

    assert(mMapEntityToComponentIndex.containsKey(jointEntity));
    mInverseMassMatrixLimit[mMapEntityToComponentIndex[jointEntity]] = inverseMassMatrixLimitMotor;
}

// Return the inverse of mass matrix K=JM^-1J^t for the motor
RP3D_FORCE_INLINE decimal SliderJointComponents::getInverseMassMatrixMotor(Entity jointEntity) {

    assert(mMapEntityToComponentIndex.containsKey(jointEntity));
    return mInverseMassMatrixMotor[mMapEntityToComponentIndex[jointEntity]];
}

// Return the inverse of mass matrix K=JM^-1J^t for the motor
RP3D_FORCE_INLINE void SliderJointComponents::setInverseMassMatrixMotor(Entity jointEntity, decimal inverseMassMatrixMotor) {

    assert(mMapEntityToComponentIndex.containsKey(jointEntity));
    mInverseMassMatrixMotor[mMapEntityToComponentIndex[jointEntity]] = inverseMassMatrixMotor;
}

// Return the bias of the lower limit constraint
RP3D_FORCE_INLINE decimal SliderJointComponents::getBLowerLimit(Entity jointEntity) const {

    assert(mMapEntityToComponentIndex.containsKey(jointEntity));
    return mBLowerLimit[mMapEntityToComponentIndex[jointEntity]];
}

// Set the bias of the lower limit constraint
RP3D_FORCE_INLINE void SliderJointComponents::setBLowerLimit(Entity jointEntity, decimal bLowerLimit) const {

    assert(mMapEntityToComponentIndex.containsKey(jointEntity));
    mBLowerLimit[mMapEntityToComponentIndex[jointEntity]] = bLowerLimit;
}

// Return the bias of the upper limit constraint
RP3D_FORCE_INLINE decimal SliderJointComponents::getBUpperLimit(Entity jointEntity) const {

    assert(mMapEntityToComponentIndex.containsKey(jointEntity));
    return mBUpperLimit[mMapEntityToComponentIndex[jointEntity]];
}

// Set the bias of the upper limit constraint
RP3D_FORCE_INLINE void SliderJointComponents::setBUpperLimit(Entity jointEntity, decimal bUpperLimit) {

    assert(mMapEntityToComponentIndex.containsKey(jointEntity));
    mBUpperLimit[mMapEntityToComponentIndex[jointEntity]] = bUpperLimit;
}

// Return true if the joint limits are enabled
RP3D_FORCE_INLINE bool SliderJointComponents::getIsLimitEnabled(Entity jointEntity) const {

    assert(mMapEntityToComponentIndex.containsKey(jointEntity));
    return mIsLimitEnabled[mMapEntityToComponentIndex[jointEntity]];
}

// Set to true if the joint limits are enabled
RP3D_FORCE_INLINE void SliderJointComponents::setIsLimitEnabled(Entity jointEntity, bool isLimitEnabled) {

    assert(mMapEntityToComponentIndex.containsKey(jointEntity));
    mIsLimitEnabled[mMapEntityToComponentIndex[jointEntity]] = isLimitEnabled;
}

// Return true if the motor of the joint in enabled
RP3D_FORCE_INLINE bool SliderJointComponents::getIsMotorEnabled(Entity jointEntity) const {

    assert(mMapEntityToComponentIndex.containsKey(jointEntity));
    return mIsMotorEnabled[mMapEntityToComponentIndex[jointEntity]];
}

// Set to true if the motor of the joint in enabled
RP3D_FORCE_INLINE void SliderJointComponents::setIsMotorEnabled(Entity jointEntity, bool isMotorEnabled) const {

    assert(mMapEntityToComponentIndex.containsKey(jointEntity));
    mIsMotorEnabled[mMapEntityToComponentIndex[jointEntity]] = isMotorEnabled;
}

// Return the Lower limit (minimum allowed rotation angle in radian)
RP3D_FORCE_INLINE decimal SliderJointComponents::getLowerLimit(Entity jointEntity) const {

    assert(mMapEntityToComponentIndex.containsKey(jointEntity));
    return mLowerLimit[mMapEntityToComponentIndex[jointEntity]];
}

// Set the Lower limit (minimum allowed rotation angle in radian)
RP3D_FORCE_INLINE void SliderJointComponents::setLowerLimit(Entity jointEntity, decimal lowerLimit) const {

    assert(mMapEntityToComponentIndex.containsKey(jointEntity));
    mLowerLimit[mMapEntityToComponentIndex[jointEntity]] = lowerLimit;
}

// Return the upper limit (maximum translation distance)
RP3D_FORCE_INLINE decimal SliderJointComponents::getUpperLimit(Entity jointEntity) const {

    assert(mMapEntityToComponentIndex.containsKey(jointEntity));
    return mUpperLimit[mMapEntityToComponentIndex[jointEntity]];
}

// Set the upper limit (maximum translation distance)
RP3D_FORCE_INLINE void SliderJointComponents::setUpperLimit(Entity jointEntity, decimal upperLimit) {

    assert(mMapEntityToComponentIndex.containsKey(jointEntity));
    mUpperLimit[mMapEntityToComponentIndex[jointEntity]] = upperLimit;
}

// Return true if the lower limit is violated
RP3D_FORCE_INLINE bool SliderJointComponents::getIsLowerLimitViolated(Entity jointEntity) const {

    assert(mMapEntityToComponentIndex.containsKey(jointEntity));
    return mIsLowerLimitViolated[mMapEntityToComponentIndex[jointEntity]];
}

// Set to true if the lower limit is violated
RP3D_FORCE_INLINE void SliderJointComponents::setIsLowerLimitViolated(Entity jointEntity, bool isLowerLimitViolated) {

    assert(mMapEntityToComponentIndex.containsKey(jointEntity));
    mIsLowerLimitViolated[mMapEntityToComponentIndex[jointEntity]] = isLowerLimitViolated;
}

// Return true if the upper limit is violated
RP3D_FORCE_INLINE bool SliderJointComponents::getIsUpperLimitViolated(Entity jointEntity) const {

    assert(mMapEntityToComponentIndex.containsKey(jointEntity));
    return mIsUpperLimitViolated[mMapEntityToComponentIndex[jointEntity]];
}

// Set to true if the upper limit is violated
RP3D_FORCE_INLINE void SliderJointComponents::setIsUpperLimitViolated(Entity jointEntity, bool isUpperLimitViolated) const {

    assert(mMapEntityToComponentIndex.containsKey(jointEntity));
    mIsUpperLimitViolated[mMapEntityToComponentIndex[jointEntity]] = isUpperLimitViolated;
}

// Return the motor speed (in rad/s)
RP3D_FORCE_INLINE decimal SliderJointComponents::getMotorSpeed(Entity jointEntity) const {

    assert(mMapEntityToComponentIndex.containsKey(jointEntity));
    return mMotorSpeed[mMapEntityToComponentIndex[jointEntity]];
}

// Set the motor speed (in rad/s)
RP3D_FORCE_INLINE void SliderJointComponents::setMotorSpeed(Entity jointEntity, decimal motorSpeed) {

    assert(mMapEntityToComponentIndex.containsKey(jointEntity));
    mMotorSpeed[mMapEntityToComponentIndex[jointEntity]] = motorSpeed;
}

// Return the maximum motor torque (in Newtons) that can be applied to reach to desired motor speed
RP3D_FORCE_INLINE decimal SliderJointComponents::getMaxMotorForce(Entity jointEntity) const {

    assert(mMapEntityToComponentIndex.containsKey(jointEntity));
    return mMaxMotorForce[mMapEntityToComponentIndex[jointEntity]];
}

// Set the maximum motor torque (in Newtons) that can be applied to reach to desired motor speed
RP3D_FORCE_INLINE void SliderJointComponents::setMaxMotorForce(Entity jointEntity, decimal maxMotorForce) {

    assert(mMapEntityToComponentIndex.containsKey(jointEntity));
    mMaxMotorForce[mMapEntityToComponentIndex[jointEntity]] = maxMotorForce;
}

// Return the cross product of r2 and n1
RP3D_FORCE_INLINE Vector3& SliderJointComponents::getR2CrossN1(Entity jointEntity) {

    assert(mMapEntityToComponentIndex.containsKey(jointEntity));
    return mR2CrossN1[mMapEntityToComponentIndex[jointEntity]];
}

// Set the cross product of r2 and n1
RP3D_FORCE_INLINE void SliderJointComponents::setR2CrossN1(Entity jointEntity, const Vector3& r2CrossN1) {

    assert(mMapEntityToComponentIndex.containsKey(jointEntity));
    mR2CrossN1[mMapEntityToComponentIndex[jointEntity]] = r2CrossN1;
}

// Return the cross product of r2 and n2
RP3D_FORCE_INLINE Vector3& SliderJointComponents::getR2CrossN2(Entity jointEntity) {

    assert(mMapEntityToComponentIndex.containsKey(jointEntity));
    return mR2CrossN2[mMapEntityToComponentIndex[jointEntity]];
}

// Set the cross product of r2 and n2
RP3D_FORCE_INLINE void SliderJointComponents::setR2CrossN2(Entity jointEntity, const Vector3& r2CrossN2) {

    assert(mMapEntityToComponentIndex.containsKey(jointEntity));
    mR2CrossN2[mMapEntityToComponentIndex[jointEntity]] = r2CrossN2;
}

// Return the cross product of r2 and the slider axis
RP3D_FORCE_INLINE Vector3& SliderJointComponents::getR2CrossSliderAxis(Entity jointEntity) {

    assert(mMapEntityToComponentIndex.containsKey(jointEntity));
    return mR2CrossSliderAxis[mMapEntityToComponentIndex[jointEntity]];
}

// Set the cross product of r2 and the slider axis
RP3D_FORCE_INLINE void SliderJointComponents::setR2CrossSliderAxis(Entity jointEntity, const Vector3& r2CrossSliderAxis) {

    assert(mMapEntityToComponentIndex.containsKey(jointEntity));
    mR2CrossSliderAxis[mMapEntityToComponentIndex[jointEntity]] = r2CrossSliderAxis;
}

// Return the cross product of vector (r1 + u) and n1
RP3D_FORCE_INLINE Vector3& SliderJointComponents::getR1PlusUCrossN1(Entity jointEntity) {

    assert(mMapEntityToComponentIndex.containsKey(jointEntity));
    return mR1PlusUCrossN1[mMapEntityToComponentIndex[jointEntity]];
}

// Set the cross product of vector (r1 + u) and n1
RP3D_FORCE_INLINE void SliderJointComponents::setR1PlusUCrossN1(Entity jointEntity, const Vector3& r1PlusUCrossN1) {

    assert(mMapEntityToComponentIndex.containsKey(jointEntity));
    mR1PlusUCrossN1[mMapEntityToComponentIndex[jointEntity]] = r1PlusUCrossN1;
}

// Return the cross product of vector (r1 + u) and n2
RP3D_FORCE_INLINE Vector3& SliderJointComponents::getR1PlusUCrossN2(Entity jointEntity) {

    assert(mMapEntityToComponentIndex.containsKey(jointEntity));
    return mR1PlusUCrossN2[mMapEntityToComponentIndex[jointEntity]];
}

// Set the cross product of vector (r1 + u) and n2
RP3D_FORCE_INLINE void SliderJointComponents::setR1PlusUCrossN2(Entity jointEntity, const Vector3& r1PlusUCrossN2) {

    assert(mMapEntityToComponentIndex.containsKey(jointEntity));
    mR1PlusUCrossN2[mMapEntityToComponentIndex[jointEntity]] = r1PlusUCrossN2;
}

// Return the cross product of vector (r1 + u) and the slider axis
RP3D_FORCE_INLINE Vector3& SliderJointComponents::getR1PlusUCrossSliderAxis(Entity jointEntity) {

    assert(mMapEntityToComponentIndex.containsKey(jointEntity));
    return mR1PlusUCrossSliderAxis[mMapEntityToComponentIndex[jointEntity]];
}

// Set the cross product of vector (r1 + u) and the slider axis
RP3D_FORCE_INLINE void SliderJointComponents::setR1PlusUCrossSliderAxis(Entity jointEntity, const Vector3& r1PlusUCrossSliderAxis) {

    assert(mMapEntityToComponentIndex.containsKey(jointEntity));
    mR1PlusUCrossSliderAxis[mMapEntityToComponentIndex[jointEntity]] = r1PlusUCrossSliderAxis;
}

}

#endif
