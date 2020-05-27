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
#include <reactphysics3d/collision/OverlapCallback.h>
#include <reactphysics3d/engine/PhysicsWorld.h>

// We want to use the ReactPhysics3D namespace
using namespace reactphysics3d;

// Contact Pair Constructor
OverlapCallback::OverlapPair::OverlapPair(ContactPair& contactPair, PhysicsWorld& world, bool isLostOverlappingPair)
                             : mContactPair(contactPair), mWorld(world), mIsLostOverlapPair(isLostOverlappingPair) {

}

// Return a pointer to the first collider in contact
Collider* OverlapCallback::OverlapPair::getCollider1() const {
    return static_cast<Collider*>(mWorld.mCollidersComponents.getCollider(mContactPair.collider1Entity));
}

// Return a pointer to the second collider in contact
Collider* OverlapCallback::OverlapPair::getCollider2() const {
    return static_cast<Collider*>(mWorld.mCollidersComponents.getCollider(mContactPair.collider2Entity));
}

// Return a pointer to the first body in contact
CollisionBody* OverlapCallback::OverlapPair::getBody1() const {
    return static_cast<CollisionBody*>(mWorld.mCollisionBodyComponents.getBody(mContactPair.body1Entity));
}

// Return a pointer to the second body in contact
CollisionBody* OverlapCallback::OverlapPair::getBody2() const {
    return static_cast<CollisionBody*>(mWorld.mCollisionBodyComponents.getBody(mContactPair.body2Entity));
}

// Return the corresponding type of event for this overlapping pair
OverlapCallback::OverlapPair::EventType OverlapCallback::OverlapPair::getEventType() const {

    if (mIsLostOverlapPair) return EventType::OverlapExit;

    if (mContactPair.collidingInPreviousFrame) return EventType::OverlapStay;

    return EventType::OverlapStart;
}

// CollisionCallbackData Constructor
OverlapCallback::CallbackData::CallbackData(List<ContactPair>& contactPairs, List<ContactPair>& lostContactPairs, bool onlyReportTriggers, PhysicsWorld& world)
                :mContactPairs(contactPairs), mLostContactPairs(lostContactPairs),
                 mContactPairsIndices(world.mMemoryManager.getHeapAllocator()), mLostContactPairsIndices(world.mMemoryManager.getHeapAllocator()), mWorld(world) {

    // Filter the contact pairs to only keep the overlap/trigger events (not the contact events)
    for (uint i=0; i < mContactPairs.size(); i++) {

        // If the contact pair contains contacts (and is therefore not an overlap/trigger event)
        if (!onlyReportTriggers || mContactPairs[i].isTrigger) {
           mContactPairsIndices.add(i);
        }
    }
    // Filter the lost contact pairs to only keep the overlap/trigger events (not the contact events)
    for (uint i=0; i < mLostContactPairs.size(); i++) {

        // If the contact pair contains contacts (and is therefore not an overlap/trigger event)
        if (!onlyReportTriggers || mLostContactPairs[i].isTrigger) {
           mLostContactPairsIndices.add(i);
        }
    }
}
