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

    // Get the transforms of the two bodies
    const Transform& body1Transform = mWorld.mTransformComponents.getTransform(jointInfo.body1->getEntity());
    const Transform& body2Transform = mWorld.mTransformComponents.getTransform(jointInfo.body2->getEntity());

    // Compute the local-space anchor point for each body
    mWorld.mBallAndSocketJointsComponents.setLocalAnchorPointBody1(entity, body1Transform.getInverse() * jointInfo.anchorPointWorldSpace);
    mWorld.mBallAndSocketJointsComponents.setLocalAnchorPointBody2(entity, body2Transform.getInverse() * jointInfo.anchorPointWorldSpace);
}


// Return a string representation
std::string BallAndSocketJoint::to_string() const {

    return "BallAndSocketJoint{ localAnchorPointBody1=" + mWorld.mBallAndSocketJointsComponents.getLocalAnchorPointBody1(mEntity).to_string() +
            ", localAnchorPointBody2=" + mWorld.mBallAndSocketJointsComponents.getLocalAnchorPointBody2(mEntity).to_string() + "}";
}

