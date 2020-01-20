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

// Libraries
#include "collision/CollisionCallback.h"
#include "collision/ContactPair.h"
#include "constraint/ContactPoint.h"
#include "engine/CollisionWorld.h"

// We want to use the ReactPhysics3D namespace
using namespace reactphysics3d;

// Constructor
CollisionCallback::ContactPoint::ContactPoint(const reactphysics3d::ContactPoint& contactPoint) : mContactPoint(contactPoint) {

}

// Contact Pair Constructor
CollisionCallback::ContactPair::ContactPair(const reactphysics3d::ContactPair& contactPair,
                                            List<reactphysics3d::ContactPoint>* contactPoints, CollisionWorld& world)
                               :mContactPair(contactPair), mContactPoints(contactPoints),
                                mWorld(world) {

}

// Return a pointer to the first body in contact
CollisionBody* CollisionCallback::ContactPair::getBody1() const {
    return static_cast<CollisionBody*>(mWorld.mCollisionBodyComponents.getBody(mContactPair.body1Entity));
}

// Return a pointer to the second body in contact
CollisionBody* CollisionCallback::ContactPair::getBody2() const {
    return static_cast<CollisionBody*>(mWorld.mCollisionBodyComponents.getBody(mContactPair.body2Entity));
}

// Return a pointer to the first collider in contact (in body 1)
Collider* CollisionCallback::ContactPair::getCollider1() const {
    return mWorld.mCollidersComponents.getCollider(mContactPair.collider1Entity);
}

// Return a pointer to the second collider in contact (in body 1)
Collider* CollisionCallback::ContactPair::getCollider2() const {
    return mWorld.mCollidersComponents.getCollider(mContactPair.collider2Entity);
}

// CollisionCallbackInfo Constructor
CollisionCallback::CallbackData::CallbackData(List<reactphysics3d::ContactPair>* contactPairs, List<ContactManifold>* manifolds,
                                                                List<reactphysics3d::ContactPoint>* contactPoints, CollisionWorld& world)
                      :mContactPairs(contactPairs), mContactManifolds(manifolds), mContactPoints(contactPoints), mWorld(world) {

}

// Return a given contact point of the contact pair
/// Note that the returned ContactPoint object is only valid during the call of the CollisionCallback::onContact()
/// method. Therefore, you need to get contact data from it and make a copy. Do not make a copy of the ContactPoint
/// object itself because it won't be valid after the CollisionCallback::onContact() call.
CollisionCallback::ContactPoint CollisionCallback::ContactPair::getContactPoint(uint index) const {

    assert(index < getNbContactPoints());

    return CollisionCallback::ContactPoint((*mContactPoints)[mContactPair.contactPointsIndex + index]);
}

// Return a given contact pair
/// Note that the returned ContactPair object is only valid during the call of the CollisionCallback::onContact()
/// method. Therefore, you need to get contact data from it and make a copy. Do not make a copy of the ContactPair
/// object itself because it won't be valid after the CollisionCallback::onContact() call.
CollisionCallback::ContactPair CollisionCallback::CallbackData::getContactPair(uint index) const {

    assert(index < getNbContactPairs());

    return CollisionCallback::ContactPair((*mContactPairs)[index], mContactPoints, mWorld);
}
