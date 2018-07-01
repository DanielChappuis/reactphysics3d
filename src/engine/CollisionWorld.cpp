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
#include "CollisionWorld.h"
#include "utils/Profiler.h"
#include "utils/Logger.h"

// Namespaces
using namespace reactphysics3d;
using namespace std;

// Initialization of static fields
uint CollisionWorld::mNbWorlds = 0;

// Constructor
CollisionWorld::CollisionWorld(const WorldSettings& worldSettings, Logger* logger, Profiler* profiler)
               : mConfig(worldSettings), mCollisionDetection(this, mMemoryManager), mBodies(mMemoryManager.getPoolAllocator()), mCurrentBodyId(0),
                 mFreeBodiesIds(mMemoryManager.getPoolAllocator()), mEventListener(nullptr), mName(worldSettings.worldName),
                 mIsProfilerCreatedByUser(profiler != nullptr),
                 mIsLoggerCreatedByUser(logger != nullptr) {

    // Automatically generate a name for the world
    if (mName == "") {

        std::stringstream ss;
        ss << "world";

        if (mNbWorlds > 0) {
            ss << mNbWorlds;
        }

        mName = ss.str();
    }

#ifdef IS_PROFILING_ACTIVE

    mProfiler = profiler;

    // If the user has not provided its own profiler, we create one
    if (mProfiler == nullptr) {

       mProfiler = new Profiler();

        // Add a destination file for the profiling data
        mProfiler->addFileDestination("rp3d_profiling_" + mName + ".txt", Profiler::Format::Text);
    }


    // Set the profiler
    mCollisionDetection.setProfiler(mProfiler);

#endif

#ifdef IS_LOGGING_ACTIVE

    mLogger = logger;

    // If the user has not provided its own logger, we create one
    if (mLogger == nullptr) {

       mLogger = new Logger();

        // Add a log destination file
        uint logLevel = static_cast<uint>(Logger::Level::Information) | static_cast<uint>(Logger::Level::Warning) |
                static_cast<uint>(Logger::Level::Error);
        mLogger->addFileDestination("rp3d_log_" + mName + ".html", logLevel, Logger::Format::HTML);
    }

#endif

    mNbWorlds++;

    RP3D_LOG(mLogger, Logger::Level::Information, Logger::Category::World,
             "Collision World: Collision world " + mName + " has been created");
    RP3D_LOG(mLogger, Logger::Level::Information, Logger::Category::World,
             "Collision World: Initial world settings: " + worldSettings.to_string());
}

// Destructor
CollisionWorld::~CollisionWorld() {

    RP3D_LOG(mLogger, Logger::Level::Information, Logger::Category::World,
             "Collision World: Collision world " + mName + " has been destroyed");

    // Destroy all the collision bodies that have not been removed
    for (int i=mBodies.size() - 1 ; i >= 0; i--) {
        destroyCollisionBody(mBodies[i]);
    }

#ifdef IS_PROFILING_ACTIVE

    /// Delete the profiler
    if (!mIsProfilerCreatedByUser) {
        delete mProfiler;
    }

#endif

#ifdef IS_LOGGING_ACTIVE

    /// Delete the logger
    if (!mIsLoggerCreatedByUser) {
        delete mLogger;
    }

#endif

    assert(mBodies.size() == 0);
}

// Create a collision body and add it to the world
/**
 * @param transform Transformation mapping the local-space of the body to world-space
 * @return A pointer to the body that has been created in the world
 */
CollisionBody* CollisionWorld::createCollisionBody(const Transform& transform) {

    // Get the next available body ID
    bodyindex bodyID = computeNextAvailableBodyId();

    // Largest index cannot be used (it is used for invalid index)
    assert(bodyID < std::numeric_limits<reactphysics3d::bodyindex>::max());

    // Create the collision body
    CollisionBody* collisionBody = new (mMemoryManager.allocate(MemoryManager::AllocationType::Pool,
                                        sizeof(CollisionBody)))
                                        CollisionBody(transform, *this, bodyID);

    assert(collisionBody != nullptr);

    // Add the collision body to the world
    mBodies.add(collisionBody);

#ifdef IS_PROFILING_ACTIVE

    collisionBody->setProfiler(mProfiler);

#endif

#ifdef IS_LOGGING_ACTIVE
   collisionBody->setLogger(mLogger);
#endif

    RP3D_LOG(mLogger, Logger::Level::Information, Logger::Category::Body,
             "Body " + std::to_string(bodyID) + ": New collision body created");

    // Return the pointer to the rigid body
    return collisionBody;
}

// Destroy a collision body
/**
 * @param collisionBody Pointer to the body to destroy
 */
void CollisionWorld::destroyCollisionBody(CollisionBody* collisionBody) {

    RP3D_LOG(mLogger, Logger::Level::Information, Logger::Category::Body,
             "Body " + std::to_string(collisionBody->getId()) + ": collision body destroyed");

    // Remove all the collision shapes of the body
    collisionBody->removeAllCollisionShapes();

    // Add the body ID to the list of free IDs
    mFreeBodiesIds.add(collisionBody->getId());

    // Reset the contact manifold list of the body
    collisionBody->resetContactManifoldsList();

    // Call the destructor of the collision body
    collisionBody->~CollisionBody();

    // Remove the collision body from the list of bodies
    mBodies.remove(collisionBody);

    // Free the object from the memory allocator
    mMemoryManager.release(MemoryManager::AllocationType::Pool, collisionBody, sizeof(CollisionBody));
}

// Return the next available body ID
bodyindex CollisionWorld::computeNextAvailableBodyId() {

    // Compute the body ID
    bodyindex bodyID;
    if (mFreeBodiesIds.size() != 0) {
        bodyID = mFreeBodiesIds[mFreeBodiesIds.size() - 1];
        mFreeBodiesIds.removeAt(mFreeBodiesIds.size() - 1);
    }
    else {
        bodyID = mCurrentBodyId;
        mCurrentBodyId++;
    }

    return bodyID;
}

// Reset all the contact manifolds linked list of each body
void CollisionWorld::resetContactManifoldListsOfBodies() {

    // For each rigid body of the world
    for (List<CollisionBody*>::Iterator it = mBodies.begin(); it != mBodies.end(); ++it) {

        // Reset the contact manifold list of the body
        (*it)->resetContactManifoldsList();
    }
}

// Test if the AABBs of two bodies overlap
/**
 * @param body1 Pointer to the first body to test
 * @param body2 Pointer to the second body to test
 * @return True if the AABBs of the two bodies overlap and false otherwise
 */
bool CollisionWorld::testAABBOverlap(const CollisionBody* body1,
                                     const CollisionBody* body2) const {

    // If one of the body is not active, we return no overlap
    if (!body1->isActive() || !body2->isActive()) return false;

    // Compute the AABBs of both bodies
    AABB body1AABB = body1->getAABB();
    AABB body2AABB = body2->getAABB();

    // Return true if the two AABBs overlap
    return body1AABB.testCollision(body2AABB);
}

// Report all the bodies which have an AABB that overlaps with the AABB in parameter
/**
 * @param aabb AABB used to test for overlap
 * @param overlapCallback Pointer to the callback class to report overlap
 * @param categoryMaskBits bits mask used to filter the bodies to test overlap with
 */
void CollisionWorld::testAABBOverlap(const AABB& aabb, OverlapCallback* overlapCallback, unsigned short categoryMaskBits) {
    mCollisionDetection.testAABBOverlap(aabb, overlapCallback, categoryMaskBits);
}

// Return true if two bodies overlap
/**
 * @param body1 Pointer to the first body
 * @param body2 Pointer to a second body
 * @return True if the two bodies overlap
 */
bool CollisionWorld::testOverlap(CollisionBody* body1, CollisionBody* body2) {
    return mCollisionDetection.testOverlap(body1, body2);
}

// Return the current world-space AABB of given proxy shape
/**
 * @param proxyShape Pointer to a proxy shape
 * @return The AAABB of the proxy shape in world-space
 */
AABB CollisionWorld::getWorldAABB(const ProxyShape* proxyShape) const {

    if (proxyShape->getBroadPhaseId() == -1) {
        return AABB();
    }

   return mCollisionDetection.getWorldAABB(proxyShape);
}
