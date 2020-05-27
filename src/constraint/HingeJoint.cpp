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
#include <reactphysics3d/constraint/HingeJoint.h>
#include <reactphysics3d/systems/ConstraintSolverSystem.h>
#include <reactphysics3d/components/RigidBodyComponents.h>
#include <reactphysics3d/engine/PhysicsWorld.h>

using namespace reactphysics3d;

// Constructor
HingeJoint::HingeJoint(Entity entity, PhysicsWorld &world, const HingeJointInfo& jointInfo) : Joint(entity, world) {

    const decimal lowerLimit = mWorld.mHingeJointsComponents.getLowerLimit(mEntity);
    const decimal upperLimit = mWorld.mHingeJointsComponents.getUpperLimit(mEntity);
    assert(lowerLimit <= decimal(0) && lowerLimit >= decimal(-2.0) * PI);
    assert(upperLimit >= decimal(0) && upperLimit <= decimal(2.0) * PI);

    // Compute the local-space anchor point for each body
    const Transform& transform1 = mWorld.mTransformComponents.getTransform(jointInfo.body1->getEntity());
    const Transform& transform2 = mWorld.mTransformComponents.getTransform(jointInfo.body2->getEntity());
    mWorld.mHingeJointsComponents.setLocalAnchorPointBody1(mEntity, transform1.getInverse() * jointInfo.anchorPointWorldSpace);
    mWorld.mHingeJointsComponents.setLocalAnchorPointBody2(mEntity, transform2.getInverse() * jointInfo.anchorPointWorldSpace);

    // Compute the local-space hinge axis
    Vector3 hingeLocalAxisBody1 = transform1.getOrientation().getInverse() * jointInfo.rotationAxisWorld;
    Vector3 hingeLocalAxisBody2 = transform2.getOrientation().getInverse() * jointInfo.rotationAxisWorld;
    hingeLocalAxisBody1.normalize();
    hingeLocalAxisBody2.normalize();
    mWorld.mHingeJointsComponents.setHingeLocalAxisBody1(mEntity, hingeLocalAxisBody1);
    mWorld.mHingeJointsComponents.setHingeLocalAxisBody2(mEntity, hingeLocalAxisBody2);

    // Compute the inverse of the initial orientation difference between the two bodies
    Quaternion initOrientationDifferenceInv = transform2.getOrientation() *
                                    transform1.getOrientation().getInverse();
    initOrientationDifferenceInv.normalize();
    initOrientationDifferenceInv.inverse();
    mWorld.mHingeJointsComponents.setInitOrientationDifferenceInv(mEntity, initOrientationDifferenceInv);
}

// Enable/Disable the limits of the joint
/**
 * @param isLimitEnabled True if you want to enable the limits of the joint and
 *                       false otherwise
 */
void HingeJoint::enableLimit(bool isLimitEnabled) {

    if (isLimitEnabled != mWorld.mHingeJointsComponents.getIsLimitEnabled(mEntity)) {

        mWorld.mHingeJointsComponents.setIsLimitEnabled(mEntity, isLimitEnabled);

        // Reset the limits
        resetLimits();
    }
}

// Enable/Disable the motor of the joint
/**
 * @param isMotorEnabled True if you want to enable the motor of the joint and
 *                       false otherwise
 */
void HingeJoint::enableMotor(bool isMotorEnabled) {

    mWorld.mHingeJointsComponents.setIsMotorEnabled(mEntity, isMotorEnabled);
    mWorld.mHingeJointsComponents.setImpulseMotor(mEntity, decimal(0.0));

    // Wake up the two bodies of the joint
    awakeBodies();
}

// Set the minimum angle limit
/**
 * @param lowerLimit The minimum limit angle of the joint (in radian)
 */
void HingeJoint::setMinAngleLimit(decimal lowerLimit) {

    const decimal limit = mWorld.mHingeJointsComponents.getLowerLimit(mEntity);

    assert(limit <= decimal(0.0) && limit >= decimal(-2.0) * PI);

    if (lowerLimit != limit) {

        mWorld.mHingeJointsComponents.setLowerLimit(mEntity, lowerLimit);

        // Reset the limits
        resetLimits();
    }
}

// Set the maximum angle limit
/**
 * @param upperLimit The maximum limit angle of the joint (in radian)
 */
void HingeJoint::setMaxAngleLimit(decimal upperLimit) {

    const decimal limit = mWorld.mHingeJointsComponents.getUpperLimit(mEntity);

    assert(limit >= decimal(0) && limit <= decimal(2.0) * PI);

    if (upperLimit != limit) {

        mWorld.mHingeJointsComponents.setUpperLimit(mEntity, upperLimit);

        // Reset the limits
        resetLimits();
    }
}

// Reset the limits
void HingeJoint::resetLimits() {

    // Reset the accumulated impulses for the limits
    mWorld.mHingeJointsComponents.setImpulseLowerLimit(mEntity, decimal(0.0));
    mWorld.mHingeJointsComponents.setImpulseUpperLimit(mEntity, decimal(0.0));

    // Wake up the two bodies of the joint
    awakeBodies();
}

// Set the motor speed
void HingeJoint::setMotorSpeed(decimal motorSpeed) {

    if (motorSpeed != mWorld.mHingeJointsComponents.getMotorSpeed(mEntity)) {

        mWorld.mHingeJointsComponents.setMotorSpeed(mEntity, motorSpeed);

        // Wake up the two bodies of the joint
        awakeBodies();
    }
}

// Set the maximum motor torque
/**
 * @param maxMotorTorque The maximum torque (in Newtons) of the joint motor
 */
void HingeJoint::setMaxMotorTorque(decimal maxMotorTorque) {

    const decimal torque = mWorld.mHingeJointsComponents.getMaxMotorTorque(mEntity);

    if (maxMotorTorque != torque) {

        assert(torque >= decimal(0.0));
        mWorld.mHingeJointsComponents.setMaxMotorTorque(mEntity, maxMotorTorque);

        // Wake up the two bodies of the joint
        awakeBodies();
    }
}


// Return true if the limits of the joint are enabled
/**
 * @return True if the limits of the joint are enabled and false otherwise
 */
bool HingeJoint::isLimitEnabled() const {
    return mWorld.mHingeJointsComponents.getIsLimitEnabled(mEntity);
}

// Return true if the motor of the joint is enabled
/**
 * @return True if the motor of joint is enabled and false otherwise
 */
bool HingeJoint::isMotorEnabled() const {
    return mWorld.mHingeJointsComponents.getIsMotorEnabled(mEntity);
}

// Return the minimum angle limit
/**
 * @return The minimum limit angle of the joint (in radian)
 */
decimal HingeJoint::getMinAngleLimit() const {
    return mWorld.mHingeJointsComponents.getLowerLimit(mEntity);
}

// Return the maximum angle limit
/**
 * @return The maximum limit angle of the joint (in radian)
 */
decimal HingeJoint::getMaxAngleLimit() const {
    return mWorld.mHingeJointsComponents.getUpperLimit(mEntity);
}

// Return the motor speed
/**
 * @return The current speed of the joint motor (in radian per second)
 */
 decimal HingeJoint::getMotorSpeed() const {
    return mWorld.mHingeJointsComponents.getMotorSpeed(mEntity);
}

// Return the maximum motor torque
/**
 * @return The maximum torque of the joint motor (in Newtons)
 */
 decimal HingeJoint::getMaxMotorTorque() const {
    return mWorld.mHingeJointsComponents.getMaxMotorTorque(mEntity);
}

// Return the intensity of the current torque applied for the joint motor
/**
 * @param timeStep The current time step (in seconds)
 * @return The intensity of the current torque (in Newtons) of the joint motor
 */
 decimal HingeJoint::getMotorTorque(decimal timeStep) const {
    return mWorld.mHingeJointsComponents.getImpulseMotor(mEntity) / timeStep;
}

// Return the number of bytes used by the joint
 size_t HingeJoint::getSizeInBytes() const {
    return sizeof(HingeJoint);
}

// Return a string representation
std::string HingeJoint::to_string() const {
    return "HingeJoint{ lowerLimit=" + std::to_string(mWorld.mHingeJointsComponents.getLowerLimit(mEntity)) +
            ", upperLimit=" + std::to_string(mWorld.mHingeJointsComponents.getUpperLimit(mEntity)) +
            "localAnchorPointBody1=" + mWorld.mHingeJointsComponents.getLocalAnchorPointBody1(mEntity).to_string() + ", localAnchorPointBody2=" +
            mWorld.mHingeJointsComponents.getLocalAnchorPointBody2(mEntity).to_string() + ", hingeLocalAxisBody1=" +
            mWorld.mHingeJointsComponents.getHingeLocalAxisBody1(mEntity).to_string() +
            ", hingeLocalAxisBody2=" + mWorld.mHingeJointsComponents.getHingeLocalAxisBody2(mEntity).to_string() +
            ", initOrientationDifferenceInv=" + mWorld.mHingeJointsComponents.getInitOrientationDifferenceInv(mEntity).to_string() +
            ", motorSpeed=" + std::to_string(mWorld.mHingeJointsComponents.getMotorSpeed(mEntity)) +
            ", maxMotorTorque=" + std::to_string(mWorld.mHingeJointsComponents.getMaxMotorTorque(mEntity)) + ", isLimitEnabled=" +
            (mWorld.mHingeJointsComponents.getIsLimitEnabled(mEntity) ? "true" : "false") + ", isMotorEnabled=" +
            (mWorld.mHingeJointsComponents.getIsMotorEnabled(mEntity) ? "true" : "false") + "}";
}
