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
#include <reactphysics3d/constraint/SliderJoint.h>
#include <reactphysics3d/systems/ConstraintSolverSystem.h>
#include <reactphysics3d/components/RigidBodyComponents.h>
#include <reactphysics3d/engine/PhysicsWorld.h>

using namespace reactphysics3d;

// Static variables definition
const decimal SliderJoint::BETA = decimal(0.2);

// Constructor
SliderJoint::SliderJoint(Entity entity, PhysicsWorld& world, const SliderJointInfo& jointInfo)
            : Joint(entity, world) {

    assert(mWorld.mSliderJointsComponents.getUpperLimit(mEntity) >= decimal(0.0));
    assert(mWorld.mSliderJointsComponents.getLowerLimit(mEntity) <= decimal(0.0));
    assert(mWorld.mSliderJointsComponents.getMaxMotorForce(mEntity) >= decimal(0.0));

    Vector3 anchorPointBody1Local;
    Vector3 anchorPointBody2Local;
    Vector3 sliderLocalAxisBody1;
    Vector3 sliderLocalAxisBody2;

    const Transform& transform1 = mWorld.mTransformComponents.getTransform(jointInfo.body1->getEntity());
    const Transform& transform2 = mWorld.mTransformComponents.getTransform(jointInfo.body2->getEntity());

    const Transform transform2Inverse = transform2.getInverse();

    if (jointInfo.isUsingLocalSpaceAnchors) {

        anchorPointBody1Local = jointInfo.anchorPointBody1LocalSpace;
        anchorPointBody2Local = jointInfo.anchorPointBody2LocalSpace;

        sliderLocalAxisBody1 = jointInfo.sliderAxisBody1Local;
    }
    else {

        // Compute the local-space anchor point for each body
        const Transform transform1Inverse = transform1.getInverse();
        anchorPointBody1Local = transform1Inverse * jointInfo.anchorPointWorldSpace;
        anchorPointBody2Local = transform2Inverse * jointInfo.anchorPointWorldSpace;

        // Compute the slider axis in local-space of body 1
        sliderLocalAxisBody1 = transform1Inverse.getOrientation() * jointInfo.sliderAxisWorldSpace;
        sliderLocalAxisBody1.normalize();
    }

    mWorld.mSliderJointsComponents.setLocalAnchorPointBody1(mEntity, anchorPointBody1Local);
    mWorld.mSliderJointsComponents.setLocalAnchorPointBody2(mEntity, anchorPointBody2Local);

    mWorld.mSliderJointsComponents.setSliderAxisBody1(mEntity, sliderLocalAxisBody1);

    // Store inverse of initial rotation from body 1 to body 2 in body 1 space:
	//
	// q20 = q10 r0 
	// <=> r0 = q10^-1 q20 
	// <=> r0^-1 = q20^-1 q10
	//
	// where:
	//
	// q20 = initial orientation of body 2
	// q10 = initial orientation of body 1
	// r0 = initial rotation rotation from body 1 to body 2
    mWorld.mSliderJointsComponents.setInitOrientationDifferenceInv(mEntity, transform2Inverse.getOrientation() * transform1.getOrientation());
}

// Enable/Disable the limits of the joint
/**
 * @param isLimitEnabled True if you want to enable the joint limits and false
 *                       otherwise
 */
void SliderJoint::enableLimit(bool isLimitEnabled) {

    bool isEnabled = mWorld.mSliderJointsComponents.getIsLimitEnabled(mEntity);

    if (isLimitEnabled != isEnabled) {

        mWorld.mSliderJointsComponents.setIsLimitEnabled(mEntity, isLimitEnabled);

        // Reset the limits
        resetLimits();
    }
}

// Enable/Disable the motor of the joint
/**
 * @param isMotorEnabled True if you want to enable the joint motor and false
 *                       otherwise
 */
void SliderJoint::enableMotor(bool isMotorEnabled) {

    mWorld.mSliderJointsComponents.setIsMotorEnabled(mEntity, isMotorEnabled);
    mWorld.mSliderJointsComponents.setImpulseMotor(mEntity, decimal(0.0));

    // Wake up the two bodies of the joint
    awakeBodies();
}

// Return the current translation value of the joint
/**
 * @return The current translation distance of the joint (in meters)
 */
decimal SliderJoint::getTranslation() const {

    // TODO : Check if we need to compare rigid body position or center of mass here

    // Get the bodies entities
    const Entity body1Entity = mWorld.mJointsComponents.getBody1Entity(mEntity);
    const Entity body2Entity = mWorld.mJointsComponents.getBody2Entity(mEntity);

    // Get the bodies positions and orientations
    const Transform& transform1 = mWorld.mTransformComponents.getTransform(body1Entity);
    const Transform& transform2 = mWorld.mTransformComponents.getTransform(body2Entity);

    const Vector3& x1 = transform1.getPosition();
    const Vector3& x2 = transform2.getPosition();
    const Quaternion& q1 = transform1.getOrientation();
    const Quaternion& q2 = transform2.getOrientation();

    // Compute the two anchor points in world-space coordinates
    const Vector3 anchorBody1 = x1 + q1 * mWorld.mSliderJointsComponents.getLocalAnchorPointBody1(mEntity);
    const Vector3 anchorBody2 = x2 + q2 * mWorld.mSliderJointsComponents.getLocalAnchorPointBody2(mEntity);

    // Compute the vector u (difference between anchor points)
    const Vector3 u = anchorBody2 - anchorBody1;

    // Compute the slider axis in world-space
    Vector3 sliderAxisWorld = q1 * mWorld.mSliderJointsComponents.getSliderAxisBody1(mEntity);
    sliderAxisWorld.normalize();

    // Compute and return the translation value
    return u.dot(sliderAxisWorld);
}

// Set the minimum translation limit
/**
 * @param lowerLimit The minimum translation limit of the joint (in meters)
 */
void SliderJoint::setMinTranslationLimit(decimal lowerLimit) {

    assert(lowerLimit <= mWorld.mSliderJointsComponents.getUpperLimit(mEntity));

    if (lowerLimit != mWorld.mSliderJointsComponents.getLowerLimit(mEntity)) {

        mWorld.mSliderJointsComponents.setLowerLimit(mEntity, lowerLimit);

        // Reset the limits
        resetLimits();
    }
}

// Set the maximum translation limit
/**
 * @param upperLimit The maximum translation limit of the joint (in meters)
 */
void SliderJoint::setMaxTranslationLimit(decimal upperLimit) {

    assert(mWorld.mSliderJointsComponents.getLowerLimit(mEntity) <= upperLimit);

    if (upperLimit != mWorld.mSliderJointsComponents.getUpperLimit(mEntity)) {

        mWorld.mSliderJointsComponents.setUpperLimit(mEntity, upperLimit);

        // Reset the limits
        resetLimits();
    }
}

// Reset the limits
void SliderJoint::resetLimits() {

    // Reset the accumulated impulses for the limits
    mWorld.mSliderJointsComponents.setImpulseLowerLimit(mEntity, decimal(0.0));
    mWorld.mSliderJointsComponents.setImpulseUpperLimit(mEntity, decimal(0.0));

    // Wake up the two bodies of the joint
    awakeBodies();
}

// Set the motor speed
/**
 * @param motorSpeed The speed of the joint motor (in meters per second)
 */
void SliderJoint::setMotorSpeed(decimal motorSpeed) {

    if (motorSpeed != mWorld.mSliderJointsComponents.getMotorSpeed(mEntity)) {

        mWorld.mSliderJointsComponents.setMotorSpeed(mEntity, motorSpeed);

        // Wake up the two bodies of the joint
        awakeBodies();
    }
}

// Set the maximum motor force
/**
 * @param maxMotorForce The maximum force of the joint motor (in Newton x meters)
 */
void SliderJoint::setMaxMotorForce(decimal maxMotorForce) {

    const decimal maxForce = mWorld.mSliderJointsComponents.getMaxMotorForce(mEntity);

    if (maxMotorForce != maxForce) {

        assert(maxForce >= decimal(0.0));
        mWorld.mSliderJointsComponents.setMaxMotorForce(mEntity, maxMotorForce);

        // Wake up the two bodies of the joint
        awakeBodies();
    }
}

// Return the intensity of the current force applied for the joint motor
/**
 * @param timeStep Time step (in seconds)
 * @return The current force of the joint motor (in Newton x meters)
 */
decimal SliderJoint::getMotorForce(decimal timeStep) const {
    return mWorld.mSliderJointsComponents.getImpulseMotor(mEntity) / timeStep;
}

// Return true if the limits or the joint are enabled
/**
 * @return True if the joint limits are enabled
 */
bool SliderJoint::isLimitEnabled() const {

    return mWorld.mSliderJointsComponents.getIsLimitEnabled(mEntity);
}

// Return true if the motor of the joint is enabled
/**
 * @return True if the joint motor is enabled
 */
bool SliderJoint::isMotorEnabled() const {
    return mWorld.mSliderJointsComponents.getIsMotorEnabled(mEntity);
}

// Return the minimum translation limit
/**
 * @return The minimum translation limit of the joint (in meters)
 */
decimal SliderJoint::getMinTranslationLimit() const {
    return mWorld.mSliderJointsComponents.getLowerLimit(mEntity);
}

// Return the motor speed
/**
 * @return The current motor speed of the joint (in meters per second)
 */
decimal SliderJoint::getMotorSpeed() const {
    return mWorld.mSliderJointsComponents.getMotorSpeed(mEntity);
}

// Return the maximum motor force
/**
 * @return The maximum force of the joint motor (in Newton x meters)
 */
decimal SliderJoint::getMaxMotorForce() const {
    return mWorld.mSliderJointsComponents.getMaxMotorForce(mEntity);
}

// Return the maximum translation limit
/**
 * @return The maximum translation limit of the joint (in meters)
 */
decimal SliderJoint::getMaxTranslationLimit() const {
    return mWorld.mSliderJointsComponents.getUpperLimit(mEntity);
}

// Return the force (in Newtons) on body 2 required to satisfy the joint constraint in world-space
/**
 * @return The current force (in Newtons) applied on body 2
 */
Vector3 SliderJoint::getReactionForce(decimal timeStep) const {
    assert(timeStep > MACHINE_EPSILON);

    const uint32 jointIndex = mWorld.mSliderJointsComponents.getEntityIndex(mEntity);

    const Vector2 translationImpulse = mWorld.mSliderJointsComponents.mImpulseTranslation[jointIndex];
    const Vector3& n1 = mWorld.mSliderJointsComponents.mN1[jointIndex];
    const Vector3& n2 = mWorld.mSliderJointsComponents.mN2[jointIndex];
    const Vector3 impulseJoint = n1 * translationImpulse.x + n2 * translationImpulse.y;

    const Vector3& sliderAxisWorld = mWorld.mSliderJointsComponents.mSliderAxisWorld[jointIndex];
    const Vector3 impulseLowerLimit = mWorld.mSliderJointsComponents.mImpulseLowerLimit[jointIndex] * sliderAxisWorld;
    const Vector3 impulseUpperLimit = -mWorld.mSliderJointsComponents.mImpulseUpperLimit[jointIndex] * sliderAxisWorld;
    Vector3 impulseMotor = -mWorld.mSliderJointsComponents.mImpulseMotor[jointIndex] * sliderAxisWorld;

    return (impulseJoint + impulseLowerLimit + impulseUpperLimit + impulseMotor) / timeStep;
}

// Return the torque (in Newtons * meters) on body 2 required to satisfy the joint constraint in world-space
/**
 * @return The current torque (in Newtons * meters) applied on body 2
 */
Vector3 SliderJoint::getReactionTorque(decimal timeStep) const {

    assert(timeStep > MACHINE_EPSILON);
    return mWorld.mSliderJointsComponents.getImpulseRotation(mEntity) / timeStep;
}

// Return a string representation
std::string SliderJoint::to_string() const {
    return "SliderJoint{ lowerLimit=" + std::to_string(mWorld.mSliderJointsComponents.getLowerLimit(mEntity)) + ", upperLimit=" + std::to_string(mWorld.mSliderJointsComponents.getUpperLimit(mEntity)) +
            "localAnchorPointBody1=" + mWorld.mSliderJointsComponents.getLocalAnchorPointBody1(mEntity).to_string() + ", localAnchorPointBody2=" +
            mWorld.mSliderJointsComponents.getLocalAnchorPointBody2(mEntity).to_string() + ", sliderAxisBody1=" + mWorld.mSliderJointsComponents.getSliderAxisBody1(mEntity).to_string() +
            ", initOrientationDifferenceInv=" +
            mWorld.mSliderJointsComponents.getInitOrientationDifferenceInv(mEntity).to_string() + ", motorSpeed=" + std::to_string(getMotorSpeed()) +
            ", maxMotorForce=" + std::to_string(getMaxMotorForce()) + ", isLimitEnabled=" +
            (mWorld.mSliderJointsComponents.getIsLimitEnabled(mEntity) ? "true" : "false") + ", isMotorEnabled=" + (mWorld.mSliderJointsComponents.getIsMotorEnabled(mEntity) ? "true" : "false") + "}";
}
