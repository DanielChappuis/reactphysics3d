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

#ifndef REACTPHYSICS3D_SLIDER_JOINT_H
#define REACTPHYSICS3D_SLIDER_JOINT_H

// Libraries
#include <reactphysics3d/mathematics/mathematics.h>
#include <reactphysics3d/body/RigidBody.h>
#include <reactphysics3d/constraint/Joint.h>

namespace reactphysics3d {

// Declarations
class ConstraintSolverSystem;

// Structure SliderJointInfo
/**
 * This structure is used to gather the information needed to create a slider
 * joint. This structure will be used to create the actual slider joint.
 */
struct SliderJointInfo : public JointInfo {

    public :

        // -------------------- Attributes -------------------- //

        /// True if this object has been constructed using local-space anchors
        bool isUsingLocalSpaceAnchors;

        /// Anchor point (in world-space coordinates)
        Vector3 anchorPointWorldSpace;

        /// Anchor point on body 1 (in local-space coordinates)
        Vector3 anchorPointBody1LocalSpace;

        /// Anchor point on body 2 (in local-space coordinates)
        Vector3 anchorPointBody2LocalSpace;

        /// Slider axis (in world-space coordinates)
        Vector3 sliderAxisWorldSpace;

        /// Hinge slider axis of body 1 (in local-space coordinates)
        Vector3 sliderAxisBody1Local;

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

        /// Constructor without limits and without motor with world-space anchor
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
                         isUsingLocalSpaceAnchors(false),
                         anchorPointWorldSpace(initAnchorPointWorldSpace),
                         sliderAxisWorldSpace(initSliderAxisWorldSpace),
                         isLimitEnabled(false), isMotorEnabled(false), minTranslationLimit(-1.0),
                         maxTranslationLimit(1.0), motorSpeed(0), maxMotorForce(0) {}

        /// Constructor with limits and no motor with world-space anchor
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
                         isUsingLocalSpaceAnchors(false),
                         anchorPointWorldSpace(initAnchorPointWorldSpace),
                         sliderAxisWorldSpace(initSliderAxisWorldSpace),
                         isLimitEnabled(true), isMotorEnabled(false),
                         minTranslationLimit(initMinTranslationLimit),
                         maxTranslationLimit(initMaxTranslationLimit), motorSpeed(0),
                         maxMotorForce(0) {}

        /// Constructor with limits and motor with world-space anchor
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
                         isUsingLocalSpaceAnchors(false),
                         anchorPointWorldSpace(initAnchorPointWorldSpace),
                         sliderAxisWorldSpace(initSliderAxisWorldSpace),
                         isLimitEnabled(true), isMotorEnabled(true),
                         minTranslationLimit(initMinTranslationLimit),
                         maxTranslationLimit(initMaxTranslationLimit), motorSpeed(initMotorSpeed),
                         maxMotorForce(initMaxMotorForce) {}

        /// Constructor without limits and without motor with local-space anchor
        /**
         * @param rigidBody1 The first body of the joint
         * @param rigidBody2 The second body of the joint
         * @param anchorPointBody1Local The initial anchor point on body 1 in local-space
         * @param anchorPointBody2Local The initial anchor point on body 2 in local-space
         * @param sliderAxisBody1Local The initial slider axis in body 1 local-space
         */
        SliderJointInfo(RigidBody* rigidBody1, RigidBody* rigidBody2,
                        const Vector3& anchorPointBody1Local,
                        const Vector3& anchorPointBody2Local,
                        const Vector3& sliderAxisBody1Local)
                       : JointInfo(rigidBody1, rigidBody2, JointType::SLIDERJOINT),
                         isUsingLocalSpaceAnchors(true),
                         anchorPointBody1LocalSpace(anchorPointBody1Local),
                         anchorPointBody2LocalSpace(anchorPointBody2Local),
                         sliderAxisBody1Local(sliderAxisBody1Local),
                         isLimitEnabled(false), isMotorEnabled(false), minTranslationLimit(-1.0),
                         maxTranslationLimit(1.0), motorSpeed(0), maxMotorForce(0) {}

        /// Constructor with limits and no motor with local-space anchor
        /**
         * @param rigidBody1 The first body of the joint
         * @param rigidBody2 The second body of the joint
         * @param anchorPointBody1Local The initial anchor point on body 1 in local-space
         * @param anchorPointBody2Local The initial anchor point on body 2 in local-space
         * @param sliderAxisBody1Local The initial slider axis in body 1 local-space
         * @param initMinTranslationLimit The initial minimum translation limit (in meters)
         * @param initMaxTranslationLimit The initial maximum translation limit (in meters)
         */
        SliderJointInfo(RigidBody* rigidBody1, RigidBody* rigidBody2,
                        const Vector3& anchorPointBody1Local,
                        const Vector3& anchorPointBody2Local,
                        const Vector3& sliderAxisBody1Local,
                        decimal initMinTranslationLimit, decimal initMaxTranslationLimit)
                       : JointInfo(rigidBody1, rigidBody2, JointType::SLIDERJOINT),
                         isUsingLocalSpaceAnchors(true),
                         anchorPointBody1LocalSpace(anchorPointBody1Local),
                         anchorPointBody2LocalSpace(anchorPointBody2Local),
                         sliderAxisBody1Local(sliderAxisBody1Local),
                         isLimitEnabled(true), isMotorEnabled(false),
                         minTranslationLimit(initMinTranslationLimit),
                         maxTranslationLimit(initMaxTranslationLimit), motorSpeed(0),
                         maxMotorForce(0) {}

        /// Constructor with limits and motor with local-space anchor
        /**
         * @param rigidBody1 The first body of the joint
         * @param rigidBody2 The second body of the joint
         * @param anchorPointBody1Local The initial anchor point on body 1 in local-space
         * @param anchorPointBody2Local The initial anchor point on body 2 in local-space
         * @param sliderAxisBody1Local The initial slider axis in body 1 local-space
         * @param initMinTranslationLimit The initial minimum translation limit (in meters)
         * @param initMaxTranslationLimit The initial maximum translation limit (in meters)
         * @param initMotorSpeed The initial speed of the joint motor (in meters per second)
         * @param initMaxMotorForce The initial maximum motor force of the joint (in Newtons x meters)
         */
        SliderJointInfo(RigidBody* rigidBody1, RigidBody* rigidBody2,
                        const Vector3& anchorPointBody1Local,
                        const Vector3& anchorPointBody2Local,
                        const Vector3& sliderAxisBody1Local,
                        decimal initMinTranslationLimit, decimal initMaxTranslationLimit,
                        decimal initMotorSpeed, decimal initMaxMotorForce)
                       : JointInfo(rigidBody1, rigidBody2, JointType::SLIDERJOINT),
                         isUsingLocalSpaceAnchors(true),
                         anchorPointBody1LocalSpace(anchorPointBody1Local),
                         anchorPointBody2LocalSpace(anchorPointBody2Local),
                         sliderAxisBody1Local(sliderAxisBody1Local),
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

        // -------------------- Methods -------------------- //

        /// Reset the limits
        void resetLimits();

        /// Return the number of bytes used by the joint
        virtual size_t getSizeInBytes() const override;

    public :

        // -------------------- Methods -------------------- //

        /// Constructor
        SliderJoint(Entity entity, PhysicsWorld& world, const SliderJointInfo& jointInfo);

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

        /// Return the force (in Newtons) on body 2 required to satisfy the joint constraint in world-space
        virtual Vector3 getReactionForce(decimal timeStep) const override;

        /// Return the torque (in Newtons * meters) on body 2 required to satisfy the joint constraint in world-space
        virtual Vector3 getReactionTorque(decimal timeStep) const override;

        /// Return a string representation
        virtual std::string to_string() const override;
};

// Return the number of bytes used by the joint
RP3D_FORCE_INLINE size_t SliderJoint::getSizeInBytes() const {
    return sizeof(SliderJoint);
}

}

#endif
