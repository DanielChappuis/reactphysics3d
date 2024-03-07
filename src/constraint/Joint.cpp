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
#include <reactphysics3d/constraint/Joint.h>
#include <reactphysics3d/engine/PhysicsWorld.h>

using namespace reactphysics3d;

// Constructor
Joint::Joint(Entity entity, PhysicsWorld& world) :mEntity(entity), mWorld(world) {

}

// Return the reference to the body 1
/**
 * @return The first body involved in the joint
 */
RigidBody* Joint::getBody1() const {
    const Entity body1Entiy = mWorld.mJointsComponents.getBody1Entity(mEntity);
    return  mWorld.mRigidBodyComponents.getRigidBody(body1Entiy);
}

// Return the reference to the body 2
/**
 * @return The second body involved in the joint
 */
RigidBody* Joint::getBody2() const {
    const Entity body2Entiy = mWorld.mJointsComponents.getBody2Entity(mEntity);
    return  mWorld.mRigidBodyComponents.getRigidBody(body2Entiy);
}


// Return the type of the joint
/**
 * @return The type of the joint
 */
JointType Joint::getType() const {
    return mWorld.mJointsComponents.getType(mEntity);
}

// Return true if the collision between the two bodies of the joint is enabled
/**
 * @return True if the collision is enabled between the two bodies of the joint
 *              is enabled and false otherwise
 */
bool Joint::isCollisionEnabled() const {
    return mWorld.mJointsComponents.getIsCollisionEnabled(mEntity);
}

// Awake the two bodies of the joint
void Joint::awakeBodies() const {

    // Get the bodies entities
    Entity body1Entity = mWorld.mJointsComponents.getBody1Entity(mEntity);
    Entity body2Entity = mWorld.mJointsComponents.getBody2Entity(mEntity);

    RigidBody* body1 = static_cast<RigidBody*>(mWorld.mRigidBodyComponents.getRigidBody(body1Entity));
    RigidBody* body2 = static_cast<RigidBody*>(mWorld.mRigidBodyComponents.getRigidBody(body2Entity));

    // Wake up the two bodies of the joint
    body1->setIsSleeping(false);
    body2->setIsSleeping(false);
}
