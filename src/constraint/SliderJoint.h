/********************************************************************************
* ReactPhysics3D physics library, http://www.reactphysics3d.com                 *
* Copyright (c) 2010-2019 Daniel Chappuis                                       *
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

#ifndef REACTPHYSICS3D_SLIDER_JOINT_H
#define REACTPHYSICS3D_SLIDER_JOINT_H

// Libraries
#include "mathematics/mathematics.h"
#include "body/RigidBody.h"
#include "Joint.h"

namespace reactphysics3d {

// Declarations
class ConstraintSolver;

// Structure SliderJointInfo
/**
 * This structure is used to gather the information needed to create a slider
 * joint. This structure will be used to create the actual slider joint.
 */
struct SliderJointInfo : public JointInfo {

    public :

        // -------------------- Attributes -------------------- //

        /// Anchor point (in world-space coordinates)
        Vector3 anchorPointWorldSpace;

        /// Slider axis (in world-space coordinates)
        Vector3 sliderAxisWorldSpace;

        /// True if the slider limits are enabled
        bool isLimitEnabled;

        /// True if the slider motor is enabled
        bool isMotorEnabled;

        /// Mininum allowed translation if limits are enabled
        decimal minTranslationLimit;

        /// Maximum allowed translation if limits are enabled
        decimal maxTranslationLimit;

        /// Motor speed
        decimal motorSpeed;

        /// Maximum motor force (in Newtons) that can be applied to reach to desired motor speed
        decimal maxMotorForce;

        /// Constructor without limits and without motor
        /**
         * @param rigidBody1 The first body of the joint
         * @param rigidBody2 The second body of the joint
         * @param initAnchorPointWorldSpace The initial anchor point in world-space
         * @param initSliderAxisWorldSpace The initial slider axis in world-space
         */
        SliderJointInfo(RigidBody* rigidBody1, RigidBody* rigidBody2,
                        const Vector3& initAnchorPointWorldSpace,
                        const Vector3& initSliderAxisWorldSpace)
                       : JointInfo(rigidBody1, rigidBody2, JointType::SLIDERJOINT),
                         anchorPointWorldSpace(initAnchorPointWorldSpace),
                         sliderAxisWorldSpace(initSliderAxisWorldSpace),
                         isLimitEnabled(false), isMotorEnabled(false), minTranslationLimit(-1.0),
                         maxTranslationLimit(1.0), motorSpeed(0), maxMotorForce(0) {}

        /// Constructor with limits and no motor
        /**
         * @param rigidBody1 The first body of the joint
         * @param rigidBody2 The second body of the joint
         * @param initAnchorPointWorldSpace The initial anchor point in world-space
         * @param initSliderAxisWorldSpace The initial slider axis in world-space
         * @param initMinTranslationLimit The initial minimum translation limit (in meters)
         * @param initMaxTranslationLimit The initial maximum translation limit (in meters)
         */
        SliderJointInfo(RigidBody* rigidBody1, RigidBody* rigidBody2,
                        const Vector3& initAnchorPointWorldSpace,
                        const Vector3& initSliderAxisWorldSpace,
                        decimal initMinTranslationLimit, decimal initMaxTranslationLimit)
                       : JointInfo(rigidBody1, rigidBody2, JointType::SLIDERJOINT),
                         anchorPointWorldSpace(initAnchorPointWorldSpace),
                         sliderAxisWorldSpace(initSliderAxisWorldSpace),
                         isLimitEnabled(true), isMotorEnabled(false),
                         minTranslationLimit(initMinTranslationLimit),
                         maxTranslationLimit(initMaxTranslationLimit), motorSpeed(0),
                         maxMotorForce(0) {}

        /// Constructor with limits and motor
        /**
         * @param rigidBody1 The first body of the joint
         * @param rigidBody2 The second body of the joint
         * @param initAnchorPointWorldSpace The initial anchor point in world-space
         * @param initSliderAxisWorldSpace The initial slider axis in world-space
         * @param initMinTranslationLimit The initial minimum translation limit (in meters)
         * @param initMaxTranslationLimit The initial maximum translation limit (in meters)
         * @param initMotorSpeed The initial speed of the joint motor (in meters per second)
         * @param initMaxMotorForce The initial maximum motor force of the joint (in Newtons x meters)
         */
        SliderJointInfo(RigidBody* rigidBody1, RigidBody* rigidBody2,
                        const Vector3& initAnchorPointWorldSpace,
                        const Vector3& initSliderAxisWorldSpace,
                        decimal initMinTranslationLimit, decimal initMaxTranslationLimit,
                        decimal initMotorSpeed, decimal initMaxMotorForce)
                       : JointInfo(rigidBody1, rigidBody2, JointType::SLIDERJOINT),
                         anchorPointWorldSpace(initAnchorPointWorldSpace),
                         sliderAxisWorldSpace(initSliderAxisWorldSpace),
                         isLimitEnabled(true), isMotorEnabled(true),
                         minTranslationLimit(initMinTranslationLimit),
                         maxTranslationLimit(initMaxTranslationLimit), motorSpeed(initMotorSpeed),
                         maxMotorForce(initMaxMotorForce) {}
};

// Class SliderJoint
/**
 * This class represents a slider joint. This joint has a one degree of freedom.
 * It only allows relative translation of the bodies along a single direction and no
 * rotation.
 */
class SliderJoint : public Joint {

    private :

        // -------------------- Constants -------------------- //

        // Beta value for the position correction bias factor
        static const decimal BETA;

        // -------------------- Attributes -------------------- //

        /// Anchor point of body 1 (in local-space coordinates of body 1)
        Vector3 mLocalAnchorPointBody1;

        /// Anchor point of body 2 (in local-space coordinates of body 2)
        Vector3 mLocalAnchorPointBody2;

        /// Slider axis (in local-space coordinates of body 1)
        Vector3 mSliderAxisBody1;

        /// Inertia tensor of body 1 (in world-space coordinates)
        Matrix3x3 mI1;

        /// Inertia tensor of body 2 (in world-space coordinates)
        Matrix3x3 mI2;

        /// Inverse of the initial orientation difference between the two bodies
        Quaternion mInitOrientationDifferenceInv;

        /// First vector orthogonal to the slider axis local-space of body 1
        Vector3 mN1;

        /// Second vector orthogonal to the slider axis and mN1 in local-space of body 1
        Vector3 mN2;

        /// Vector r1 in world-space coordinates
        Vector3 mR1;

        /// Vector r2 in world-space coordinates
        Vector3 mR2;

        /// Cross product of r2 and n1
        Vector3 mR2CrossN1;

        /// Cross product of r2 and n2
        Vector3 mR2CrossN2;

        /// Cross product of r2 and the slider axis
        Vector3 mR2CrossSliderAxis;

        /// Cross product of vector (r1 + u) and n1
        Vector3 mR1PlusUCrossN1;

        /// Cross product of vector (r1 + u) and n2
        Vector3 mR1PlusUCrossN2;

        /// Cross product of vector (r1 + u) and the slider axis
        Vector3 mR1PlusUCrossSliderAxis;

        /// Bias of the 2 translation constraints
        Vector2 mBTranslation;

        /// Bias of the 3 rotation constraints
        Vector3 mBRotation;

        /// Bias of the lower limit constraint
        decimal mBLowerLimit;

        /// Bias of the upper limit constraint
        decimal mBUpperLimit;

        /// Inverse of mass matrix K=JM^-1J^t for the translation constraint (2x2 matrix)
        Matrix2x2 mInverseMassMatrixTranslationConstraint;

        /// Inverse of mass matrix K=JM^-1J^t for the rotation constraint (3x3 matrix)
        Matrix3x3 mInverseMassMatrixRotationConstraint;

        /// Inverse of mass matrix K=JM^-1J^t for the upper and lower limit constraints (1x1 matrix)
        decimal mInverseMassMatrixLimit;

        /// Inverse of mass matrix K=JM^-1J^t for the motor
        decimal mInverseMassMatrixMotor;

        /// Accumulated impulse for the 2 translation constraints
        Vector2 mImpulseTranslation;

        /// Accumulated impulse for the 3 rotation constraints
        Vector3 mImpulseRotation;

        /// Accumulated impulse for the lower limit constraint
        decimal mImpulseLowerLimit;

        /// Accumulated impulse for the upper limit constraint
        decimal mImpulseUpperLimit;

        /// Accumulated impulse for the motor
        decimal mImpulseMotor;

        /// True if the slider limits are enabled
        bool mIsLimitEnabled;

        /// True if the motor of the joint in enabled
        bool mIsMotorEnabled;

        /// Slider axis in world-space coordinates
        Vector3 mSliderAxisWorld;

        /// Lower limit (minimum translation distance)
        decimal mLowerLimit;

        /// Upper limit (maximum translation distance)
        decimal mUpperLimit;

        /// True if the lower limit is violated
        bool mIsLowerLimitViolated;

        /// True if the upper limit is violated
        bool mIsUpperLimitViolated;

        /// Motor speed (in m/s)
        decimal mMotorSpeed;

        /// Maximum motor force (in Newtons) that can be applied to reach to desired motor speed
        decimal mMaxMotorForce;

        // -------------------- Methods -------------------- //

        /// Reset the limits
        void resetLimits();

        /// Return the number of bytes used by the joint
        virtual size_t getSizeInBytes() const override;

        /// Initialize before solving the constraint
        virtual void initBeforeSolve(const ConstraintSolverData& constraintSolverData) override;

        /// Warm start the constraint (apply the previous impulse at the beginning of the step)
        virtual void warmstart(const ConstraintSolverData& constraintSolverData) override;

        /// Solve the velocity constraint
        virtual void solveVelocityConstraint(const ConstraintSolverData& constraintSolverData) override;

        /// Solve the position constraint (for position error correction)
        virtual void solvePositionConstraint(const ConstraintSolverData& constraintSolverData) override;

    public :

        // -------------------- Methods -------------------- //

        /// Constructor
        SliderJoint(uint id, const SliderJointInfo& jointInfo);

        /// Destructor
        virtual ~SliderJoint() override = default;

        /// Deleted copy-constructor
        SliderJoint(const SliderJoint& constraint) = delete;

        /// Deleted assignment operator
        SliderJoint& operator=(const SliderJoint& constraint) = delete;

        /// Return true if the limits or the joint are enabled
        bool isLimitEnabled() const;

        /// Return true if the motor of the joint is enabled
        bool isMotorEnabled() const;

        /// Enable/Disable the limits of the joint
        void enableLimit(bool isLimitEnabled);

        /// Enable/Disable the motor of the joint
        void enableMotor(bool isMotorEnabled);

        /// Return the current translation value of the joint
        decimal getTranslation() const;

        /// Return the minimum translation limit
        decimal getMinTranslationLimit() const;

        /// Set the minimum translation limit
        void setMinTranslationLimit(decimal lowerLimit);

        /// Return the maximum translation limit
        decimal getMaxTranslationLimit() const;

        /// Set the maximum translation limit
        void setMaxTranslationLimit(decimal upperLimit);

        /// Return the motor speed
        decimal getMotorSpeed() const;

        /// Set the motor speed
        void setMotorSpeed(decimal motorSpeed);

        /// Return the maximum motor force
        decimal getMaxMotorForce() const;

        /// Set the maximum motor force
        void setMaxMotorForce(decimal maxMotorForce);

        /// Return the intensity of the current force applied for the joint motor
        decimal getMotorForce(decimal timeStep) const;

        /// Return a string representation
        virtual std::string to_string() const override;
};

// Return true if the limits or the joint are enabled
/**
 * @return True if the joint limits are enabled
 */
inline bool SliderJoint::isLimitEnabled() const {
    return mIsLimitEnabled;
}

// Return true if the motor of the joint is enabled
/**
 * @return True if the joint motor is enabled
 */
inline bool SliderJoint::isMotorEnabled() const {
    return mIsMotorEnabled;
}

// Return the minimum translation limit
/**
 * @return The minimum translation limit of the joint (in meters)
 */
inline decimal SliderJoint::getMinTranslationLimit() const {
    return mLowerLimit;
}

// Return the maximum translation limit
/**
 * @return The maximum translation limit of the joint (in meters)
 */
inline decimal SliderJoint::getMaxTranslationLimit() const {
    return mUpperLimit;
}

// Return the motor speed
/**
 * @return The current motor speed of the joint (in meters per second)
 */
inline decimal SliderJoint::getMotorSpeed() const {
    return mMotorSpeed;
}

// Return the maximum motor force
/**
 * @return The maximum force of the joint motor (in Newton x meters)
 */
inline decimal SliderJoint::getMaxMotorForce() const {
    return mMaxMotorForce;
}

// Return the intensity of the current force applied for the joint motor
/**
 * @param timeStep Time step (in seconds)
 * @return The current force of the joint motor (in Newton x meters)
 */
inline decimal SliderJoint::getMotorForce(decimal timeStep) const {
    return mImpulseMotor / timeStep;
}

// Return the number of bytes used by the joint
inline size_t SliderJoint::getSizeInBytes() const {
    return sizeof(SliderJoint);
}

// Return a string representation
inline std::string SliderJoint::to_string() const {
    return "SliderJoint{ lowerLimit=" + std::to_string(mLowerLimit) + ", upperLimit=" + std::to_string(mUpperLimit) +
            "localAnchorPointBody1=" + mLocalAnchorPointBody1.to_string() + ", localAnchorPointBody2=" +
            mLocalAnchorPointBody2.to_string() + ", sliderAxisBody1=" + mSliderAxisBody1.to_string() +
            ", initOrientationDifferenceInv=" +
            mInitOrientationDifferenceInv.to_string() + ", motorSpeed=" + std::to_string(mMotorSpeed) +
            ", maxMotorForce=" + std::to_string(mMaxMotorForce) + ", isLimitEnabled=" +
            (mIsLimitEnabled ? "true" : "false") + ", isMotorEnabled=" + (mIsMotorEnabled ? "true" : "false") + "}";
}

}

#endif
