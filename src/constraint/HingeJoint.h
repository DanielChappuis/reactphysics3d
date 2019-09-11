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

#ifndef REACTPHYSICS3D_HINGE_JOINT_H
#define REACTPHYSICS3D_HINGE_JOINT_H

// Libraries
#include "Joint.h"
#include "mathematics/mathematics.h"

namespace reactphysics3d {

// Structure HingeJointInfo
/**
 * This structure is used to gather the information needed to create a hinge joint.
 * This structure will be used to create the actual hinge joint.
 */
struct HingeJointInfo : public JointInfo {

    public :

        // -------------------- Attributes -------------------- //

        /// Anchor point (in world-space coordinates)
        Vector3 anchorPointWorldSpace;

        /// Hinge rotation axis (in world-space coordinates)
        Vector3 rotationAxisWorld;

        /// True if the hinge joint limits are enabled
        bool isLimitEnabled;

        /// True if the hinge joint motor is enabled
        bool isMotorEnabled;

        /// Minimum allowed rotation angle (in radian) if limits are enabled.
        /// The angle must be in the range [-2*pi, 0]
        decimal minAngleLimit;

        /// Maximum allowed rotation angle (in radian) if limits are enabled.
        /// The angle must be in the range [0, 2*pi]
        decimal maxAngleLimit;

        /// Motor speed (in radian/second)
        decimal motorSpeed;

        /// Maximum motor torque (in Newtons * meters) that can be applied to reach
        /// to desired motor speed
        decimal maxMotorTorque;

        /// Constructor without limits and without motor
        /**
         * @param rigidBody1 The first body of the joint
         * @param rigidBody2 The second body of the joint
         * @param initAnchorPointWorldSpace The initial anchor point in world-space
         *                                  coordinates
         * @param initRotationAxisWorld The initial rotation axis in world-space
         *                              coordinates
         */
        HingeJointInfo(RigidBody* rigidBody1, RigidBody* rigidBody2,
                               const Vector3& initAnchorPointWorldSpace,
                               const Vector3& initRotationAxisWorld)
                              : JointInfo(rigidBody1, rigidBody2, JointType::HINGEJOINT),
                                anchorPointWorldSpace(initAnchorPointWorldSpace),
                                rotationAxisWorld(initRotationAxisWorld), isLimitEnabled(false),
                                isMotorEnabled(false), minAngleLimit(-1), maxAngleLimit(1),
                                motorSpeed(0), maxMotorTorque(0) {}

        /// Constructor with limits but without motor
        /**
         * @param rigidBody1 The first body of the joint
         * @param rigidBody2 The second body of the joint
         * @param initAnchorPointWorldSpace The initial anchor point in world-space coordinates
         * @param initRotationAxisWorld The intial rotation axis in world-space coordinates
         * @param initMinAngleLimit The initial minimum limit angle (in radian)
         * @param initMaxAngleLimit The initial maximum limit angle (in radian)
         */
        HingeJointInfo(RigidBody* rigidBody1, RigidBody* rigidBody2,
                               const Vector3& initAnchorPointWorldSpace,
                               const Vector3& initRotationAxisWorld,
                               decimal initMinAngleLimit, decimal initMaxAngleLimit)
                              : JointInfo(rigidBody1, rigidBody2, JointType::HINGEJOINT),
                                anchorPointWorldSpace(initAnchorPointWorldSpace),
                                rotationAxisWorld(initRotationAxisWorld), isLimitEnabled(true),
                                isMotorEnabled(false), minAngleLimit(initMinAngleLimit),
                                maxAngleLimit(initMaxAngleLimit), motorSpeed(0),
                                maxMotorTorque(0) {}

        /// Constructor with limits and motor
        /**
         * @param rigidBody1 The first body of the joint
         * @param rigidBody2 The second body of the joint
         * @param initAnchorPointWorldSpace The initial anchor point in world-space
         * @param initRotationAxisWorld The initial rotation axis in world-space
         * @param initMinAngleLimit The initial minimum limit angle (in radian)
         * @param initMaxAngleLimit The initial maximum limit angle (in radian)
         * @param initMotorSpeed The initial motor speed of the joint (in radian per second)
         * @param initMaxMotorTorque The initial maximum motor torque (in Newtons)
         */
        HingeJointInfo(RigidBody* rigidBody1, RigidBody* rigidBody2,
                               const Vector3& initAnchorPointWorldSpace,
                               const Vector3& initRotationAxisWorld,
                               decimal initMinAngleLimit, decimal initMaxAngleLimit,
                               decimal initMotorSpeed, decimal initMaxMotorTorque)
                              : JointInfo(rigidBody1, rigidBody2, JointType::HINGEJOINT),
                                anchorPointWorldSpace(initAnchorPointWorldSpace),
                                rotationAxisWorld(initRotationAxisWorld), isLimitEnabled(true),
                                isMotorEnabled(false), minAngleLimit(initMinAngleLimit),
                                maxAngleLimit(initMaxAngleLimit), motorSpeed(initMotorSpeed),
                                maxMotorTorque(initMaxMotorTorque) {}
};

// Class HingeJoint
/**
 * This class represents a hinge joint that allows arbitrary rotation
 * between two bodies around a single axis. This joint has one degree of freedom. It
 * can be useful to simulate doors or pendulumns.
 */
class HingeJoint : public Joint {

    private :

        // -------------------- Constants -------------------- //

        // Beta value for the bias factor of position correction
        static const decimal BETA;

        // -------------------- Attributes -------------------- //


        // -------------------- Methods -------------------- //

        /// Reset the limits
        void resetLimits();

        /// Given an angle in radian, this method returns the corresponding
        /// angle in the range [-pi; pi]
        decimal computeNormalizedAngle(decimal angle) const;

        /// Given an "inputAngle" in the range [-pi, pi], this method returns an
        /// angle (modulo 2*pi) in the range [-2*pi; 2*pi] that is closest to one of the
        /// two angle limits in arguments.
        decimal computeCorrespondingAngleNearLimits(decimal inputAngle, decimal lowerLimitAngle,
                                                    decimal upperLimitAngle) const;

        /// Compute the current angle around the hinge axis
        decimal computeCurrentHingeAngle(const Quaternion& orientationBody1,
                                         const Quaternion& orientationBody2);

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
        HingeJoint(Entity entity, DynamicsWorld& world, const HingeJointInfo& jointInfo);

        /// Destructor
        virtual ~HingeJoint() override = default;

        /// Deleted copy-constructor
        HingeJoint(const HingeJoint& constraint) = delete;

        /// Deleted assignment operator
        HingeJoint& operator=(const HingeJoint& constraint) = delete;

        /// Return true if the limits or the joint are enabled
        bool isLimitEnabled() const;

        /// Return true if the motor of the joint is enabled
        bool isMotorEnabled() const;

        /// Enable/Disable the limits of the joint
        void enableLimit(bool isLimitEnabled);

        /// Enable/Disable the motor of the joint
        void enableMotor(bool isMotorEnabled);

        /// Return the minimum angle limit
        decimal getMinAngleLimit() const;

        /// Set the minimum angle limit
        void setMinAngleLimit(decimal lowerLimit);

        /// Return the maximum angle limit
        decimal getMaxAngleLimit() const;

        /// Set the maximum angle limit
        void setMaxAngleLimit(decimal upperLimit);

        /// Return the motor speed
        decimal getMotorSpeed() const;

        /// Set the motor speed
        void setMotorSpeed(decimal motorSpeed);

        /// Return the maximum motor torque
        decimal getMaxMotorTorque() const;

        /// Set the maximum motor torque
        void setMaxMotorTorque(decimal maxMotorTorque);

        /// Return the intensity of the current torque applied for the joint motor
        decimal getMotorTorque(decimal timeStep) const;

        /// Return a string representation
        virtual std::string to_string() const override;
};


}


#endif
