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
#include <reactphysics3d/collision/CollisionCallback.h>
#include <reactphysics3d/collision/ContactPair.h>
#include <reactphysics3d/constraint/ContactPoint.h>
#include <reactphysics3d/engine/PhysicsWorld.h>

// We want to use the ReactPhysics3D namespace
using namespace reactphysics3d;

// Constructor
CollisionCallback::ContactPoint::ContactPoint(const reactphysics3d::ContactPoint& contactPoint) : mContactPoint(contactPoint) {

}

// Contact Pair Constructor
CollisionCallback::ContactPair::ContactPair(const reactphysics3d::ContactPair& contactPair,
                                            Array<reactphysics3d::ContactPoint>* contactPoints, PhysicsWorld& world, bool isLostContactPair)
                               :mContactPair(contactPair), mContactPoints(contactPoints),
                                mWorld(world), mIsLostContactPair(isLostContactPair) {

}

// Return a pointer to the first body in contact
Body* CollisionCallback::ContactPair::getBody1() const {
    return static_cast<Body*>(mWorld.mBodyComponents.getBody(mContactPair.body1Entity));
}

// Return a pointer to the second body in contact
Body* CollisionCallback::ContactPair::getBody2() const {
    return static_cast<Body*>(mWorld.mBodyComponents.getBody(mContactPair.body2Entity));
}

// Return a pointer to the first collider in contact (in body 1)
Collider* CollisionCallback::ContactPair::getCollider1() const {
    return mWorld.mCollidersComponents.getCollider(mContactPair.collider1Entity);
}

// Return a pointer to the second collider in contact (in body 1)
Collider* CollisionCallback::ContactPair::getCollider2() const {
    return mWorld.mCollidersComponents.getCollider(mContactPair.collider2Entity);
}

// Return the corresponding type of event for this contact pair
CollisionCallback::ContactPair::EventType CollisionCallback::ContactPair::getEventType() const {

    if (mIsLostContactPair) return EventType::ContactExit;

    if (mContactPair.collidingInPreviousFrame) return EventType::ContactStay;

    return EventType::ContactStart;
}

// Constructor
CollisionCallback::CallbackData::CallbackData(Array<reactphysics3d::ContactPair>* contactPairs, Array<ContactManifold>* manifolds,
                                              Array<reactphysics3d::ContactPoint>* contactPoints, Array<reactphysics3d::ContactPair>& lostContactPairs, PhysicsWorld& world)
                      :mContactPairs(contactPairs), mContactManifolds(manifolds), mContactPoints(contactPoints), mLostContactPairs(lostContactPairs),
                       mContactPairsIndices(world.mMemoryManager.getHeapAllocator(), contactPairs->size()), mLostContactPairsIndices(world.mMemoryManager.getHeapAllocator(), lostContactPairs.size()),
                       mWorld(world) {

    // Filter the contact pairs to only keep the contact events (not the overlap/trigger events)
    const uint64 nbContactPairs = mContactPairs->size();
    for (uint64 i=0; i < nbContactPairs; i++) {

        // If the contact pair contains contacts (and is therefore not an overlap/trigger event)
        if (!(*mContactPairs)[i].isTrigger) {
           mContactPairsIndices.add(i);
        }
    }
    // Filter the lost contact pairs to only keep the contact events (not the overlap/trigger events)
    const uint64 nbLostContactPairs = mLostContactPairs.size();
    for (uint64 i=0; i < nbLostContactPairs; i++) {

        // If the contact pair contains contacts (and is therefore not an overlap/trigger event)
        if (!mLostContactPairs[i].isTrigger) {
           mLostContactPairsIndices.add(i);
        }
    }
}

// Return a given contact point of the contact pair
/// Note that the returned ContactPoint object is only valid during the call of the CollisionCallback::onContact()
/// method. Therefore, you need to get contact data from it and make a copy. Do not make a copy of the ContactPoint
/// object itself because it won't be valid after the CollisionCallback::onContact() call.
CollisionCallback::ContactPoint CollisionCallback::ContactPair::getContactPoint(uint32 index) const {

    assert(index < getNbContactPoints());

    return CollisionCallback::ContactPoint((*mContactPoints)[mContactPair.contactPointsIndex + index]);
}

// Return a given contact pair
/// Note that the returned ContactPair object is only valid during the call of the CollisionCallback::onContact()
/// method. Therefore, you need to get contact data from it and make a copy. Do not make a copy of the ContactPair
/// object itself because it won't be valid after the CollisionCallback::onContact() call.
CollisionCallback::ContactPair CollisionCallback::CallbackData::getContactPair(uint64 index) const {

    assert(index < getNbContactPairs());

    if (index < mContactPairsIndices.size()) {
        // Return a contact pair
        return CollisionCallback::ContactPair((*mContactPairs)[mContactPairsIndices[index]], mContactPoints, mWorld, false);
    }
    else {

        // Return a lost contact pair
        return CollisionCallback::ContactPair(mLostContactPairs[mLostContactPairsIndices[index - mContactPairsIndices.size()]], mContactPoints, mWorld, true);
    }
}
