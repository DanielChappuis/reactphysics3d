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
#include <cassert>
#include "OverlappingPair.h"
#include "containers/containers_common.h"
#include "collision/ContactPointInfo.h"

using namespace reactphysics3d;

// Constructor
OverlappingPair::OverlappingPair(ProxyShape* shape1, ProxyShape* shape2,
                                 MemoryAllocator& persistentMemoryAllocator, MemoryAllocator& temporaryMemoryAllocator,
                                 const WorldSettings& worldSettings)
                : mPairID(computeID(shape1->getBroadPhaseId(), shape2->getBroadPhaseId())), mProxyShape1(shape1->getEntity()), mProxyShape2(shape2->getEntity()),
                  mPersistentAllocator(persistentMemoryAllocator), mTempMemoryAllocator(temporaryMemoryAllocator),
                  mLastFrameCollisionInfos(mPersistentAllocator), mWorldSettings(worldSettings), mNeedToTestOverlap(false) {
    
}         

// Destructor
OverlappingPair::~OverlappingPair() {

    RP3D_PROFILE("OverlappingPair::~OverlappingPair()", mProfiler);

    // Remove all the remaining last frame collision info
    for (auto it = mLastFrameCollisionInfos.begin(); it != mLastFrameCollisionInfos.end(); ++it) {

        // Call the constructor
        it->second->~LastFrameCollisionInfo();

        // Release memory
        mPersistentAllocator.release(it->second, sizeof(LastFrameCollisionInfo));
    }
}

// Add a new last frame collision info if it does not exist for the given shapes already
LastFrameCollisionInfo* OverlappingPair::addLastFrameInfoIfNecessary(uint shapeId1, uint shapeId2) {

    RP3D_PROFILE("OverlappingPair::addLastFrameInfoIfNecessary()", mProfiler);

    // Try to get the corresponding last frame collision info
    const ShapeIdPair shapeIdPair(shapeId1, shapeId2);
    auto it = mLastFrameCollisionInfos.find(shapeIdPair);

    // If there is no collision info for those two shapes already
    if (it == mLastFrameCollisionInfos.end()) {

        // Create a new collision info
        LastFrameCollisionInfo* collisionInfo = new (mPersistentAllocator.allocate(sizeof(LastFrameCollisionInfo)))
                                                LastFrameCollisionInfo();

        // Add it into the map of collision infos
        mLastFrameCollisionInfos.add(Pair<ShapeIdPair, LastFrameCollisionInfo*>(shapeIdPair, collisionInfo));

        return collisionInfo;
    }
    else {

       // The existing collision info is not obsolete
       it->second->isObsolete = false;

       return it->second;
    }
}

// Delete all the obsolete last frame collision info
void OverlappingPair::clearObsoleteLastFrameCollisionInfos() {

    RP3D_PROFILE("OverlappingPair::clearObsoleteLastFrameCollisionInfos()", mProfiler);

    // For each collision info
    for (auto it = mLastFrameCollisionInfos.begin(); it != mLastFrameCollisionInfos.end(); ) {

        // If the collision info is obsolete
        if (it->second->isObsolete) {

            // Delete it
            it->second->~LastFrameCollisionInfo();
            mPersistentAllocator.release(it->second, sizeof(LastFrameCollisionInfo));

            it = mLastFrameCollisionInfos.remove(it);
        }
        else {  // If the collision info is not obsolete

            // Do not delete it but mark it as obsolete
            it->second->isObsolete = true;

            ++it;
        }
    }
}
