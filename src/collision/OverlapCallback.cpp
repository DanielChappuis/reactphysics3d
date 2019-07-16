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

// Libraries
#include "collision/OverlapCallback.h"
#include "engine/CollisionWorld.h"

// We want to use the ReactPhysics3D namespace
using namespace reactphysics3d;

// Contact Pair Constructor
OverlapCallback::OverlapPair::OverlapPair(Pair<Entity, Entity>& overlapPair, CollisionWorld& world)
                             : mOverlapPair(overlapPair), mWorld(world) {

}

// Return a pointer to the first body in contact
CollisionBody* OverlapCallback::OverlapPair::getBody1() const {
    return static_cast<CollisionBody*>(mWorld.mCollisionBodyComponents.getBody(mOverlapPair.first));
}

// Return a pointer to the second body in contact
CollisionBody* OverlapCallback::OverlapPair::getBody2() const {
    return static_cast<CollisionBody*>(mWorld.mCollisionBodyComponents.getBody(mOverlapPair.second));
}

// CollisionCallbackData Constructor
OverlapCallback::CallbackData::CallbackData(List<Pair<Entity, Entity>>& overlapBodies, CollisionWorld& world)
                :mOverlapBodies(overlapBodies), mWorld(world) {

}
