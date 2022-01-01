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
#include <reactphysics3d/constraint/BallAndSocketJoint.h>
#include <reactphysics3d/systems/ConstraintSolverSystem.h>
#include <reactphysics3d/components/RigidBodyComponents.h>
#include <reactphysics3d/engine/PhysicsWorld.h>

using namespace reactphysics3d;

// Static variables definition
const decimal BallAndSocketJoint::BETA = decimal(0.2);

// Constructor
BallAndSocketJoint::BallAndSocketJoint(Entity entity, PhysicsWorld& world, const BallAndSocketJointInfo& jointInfo)
                   : Joint(entity, world) {

    Vector3 anchorPointBody1LocalSpace;
    Vector3 anchorPointBody2LocalSpace;

    if (jointInfo.isUsingLocalSpaceAnchors) {

        anchorPointBody1LocalSpace = jointInfo.anchorPointBody1LocalSpace;
        anchorPointBody2LocalSpace = jointInfo.anchorPointBody2LocalSpace;
    }
    else {

        // Get the transforms of the two bodies
        const Transform& body1Transform = mWorld.mTransformComponents.getTransform(jointInfo.body1->getEntity());
        const Transform& body2Transform = mWorld.mTransformComponents.getTransform(jointInfo.body2->getEntity());

        anchorPointBody1LocalSpace = body1Transform.getInverse() * jointInfo.anchorPointWorldSpace;
        anchorPointBody2LocalSpace = body2Transform.getInverse() * jointInfo.anchorPointWorldSpace;
    }

    // Compute the local-space anchor point for each body
    mWorld.mBallAndSocketJointsComponents.setLocalAnchorPointBody1(entity, anchorPointBody1LocalSpace);
    mWorld.mBallAndSocketJointsComponents.setLocalAnchorPointBody2(entity, anchorPointBody2LocalSpace);
}

// Enable/disable the cone limit of the joint
/// It is possible to enable a cone-limit for the BallAndSocketJoint in order to
/// restrict the angular motion between the two bodies. Use this method to enable/disable
/// the cone-limit.
/**
 * @param isLimitEnabled True if the limit must be enabled and false otherwise
 */
void BallAndSocketJoint::enableConeLimit(bool isLimitEnabled) {
    mWorld.mBallAndSocketJointsComponents.setIsConeLimitEnabled(mEntity, isLimitEnabled);

    resetLimits();
}

// Return true if the cone limit or the joint is enabled
/**
 * @return True if the cone-limit is enabled for this joint
 */
bool BallAndSocketJoint::isConeLimitEnabled() const {
    return mWorld.mBallAndSocketJointsComponents.getIsConeLimitEnabled(mEntity);
}

// Set the cone limit half angle
/**
 * @param coneHalfAngle The angle of the cone limit (in radian) from [0; PI]
 */
void BallAndSocketJoint::setConeLimitHalfAngle(decimal coneHalfAngle) {

    if (mWorld.mBallAndSocketJointsComponents.getConeLimitHalfAngle(mEntity) != coneHalfAngle) {

        mWorld.mBallAndSocketJointsComponents.setConeLimitHalfAngle(mEntity, coneHalfAngle);

        resetLimits();
    }
}

// Return the cone angle limit (in radians) from [0; PI]
/**
 * @return The half-angle (in radians) of the cone-limit
 */
decimal BallAndSocketJoint::getConeLimitHalfAngle() const {
    return mWorld.mBallAndSocketJointsComponents.getConeLimitHalfAngle(mEntity);
}

// Return the current cone angle in radians (in [0, pi])
/**
 * @return The current half-angle (in radians) of the joint with respect to the cone-limit axis
 */
decimal BallAndSocketJoint::getConeHalfAngle() const {

    // Get the bodies entities
    const Entity body1Entity = mWorld.mJointsComponents.getBody1Entity(mEntity);
    const Entity body2Entity = mWorld.mJointsComponents.getBody2Entity(mEntity);

    const Transform& transformBody1 = mWorld.mTransformComponents.getTransform(body1Entity);
    const Transform& transformBody2 = mWorld.mTransformComponents.getTransform(body2Entity);

    // Convert local-space cone axis of bodies to world-space
    const Vector3 r1Local = mWorld.mBallAndSocketJointsComponents.getLocalAnchorPointBody1(mEntity) - mWorld.mRigidBodyComponents.getCenterOfMassLocal(body1Entity);
    const Vector3 r2Local = mWorld.mBallAndSocketJointsComponents.getLocalAnchorPointBody2(mEntity) - mWorld.mRigidBodyComponents.getCenterOfMassLocal(body2Entity);
    const Vector3 r1World = transformBody1.getOrientation() * r1Local;
    const Vector3 r2World = transformBody2.getOrientation() * r2Local;

    return SolveBallAndSocketJointSystem::computeCurrentConeHalfAngle(r1World.getUnit(), -r2World.getUnit());
}

// Reset the limits
void BallAndSocketJoint::resetLimits() {

    // Reset the accumulated impulses for the limits
    mWorld.mBallAndSocketJointsComponents.setConeLimitImpulse(mEntity, decimal(0.0));

    // Wake up the two bodies of the joint
    awakeBodies();
}

// Return the force (in Newtons) on body 2 required to satisfy the joint constraint in world-space
/**
 * @return The current force (in Newtons) applied on body 2
 */
Vector3 BallAndSocketJoint::getReactionForce(decimal timeStep) const {
    assert(timeStep > MACHINE_EPSILON);
    return mWorld.mBallAndSocketJointsComponents.getImpulse(mEntity) / timeStep;
}

// Return the torque (in Newtons * meters) on body 2 required to satisfy the joint constraint in world-space
/**
 * @return The current torque (in Newtons * meters) on body 2
 */
Vector3 BallAndSocketJoint::getReactionTorque(decimal timeStep) const {
    assert(timeStep > MACHINE_EPSILON);
    return Vector3(0, 0, 0);
}

// Return a string representation
std::string BallAndSocketJoint::to_string() const {

    return "BallAndSocketJoint{ localAnchorPointBody1=" + mWorld.mBallAndSocketJointsComponents.getLocalAnchorPointBody1(mEntity).to_string() +
            ", localAnchorPointBody2=" + mWorld.mBallAndSocketJointsComponents.getLocalAnchorPointBody2(mEntity).to_string() + "}";
}

