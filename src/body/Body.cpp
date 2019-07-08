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
#include "Body.h"
#include "engine/CollisionWorld.h"
#include "collision/shapes/CollisionShape.h"
#include "utils/Logger.h"

// We want to use the ReactPhysics3D namespace
using namespace reactphysics3d;

// Constructor
/**
 * @param id ID of the new body
 */
Body::Body(Entity entity, CollisionWorld& world)
     : mEntity(entity), mWorld(world) {

#ifdef IS_LOGGING_ACTIVE
        mLogger = nullptr;
#endif

}

// Set whether or not the body is active
/// An inactive body does not participate in collision detection,
/// is not simulated and will not be hit in a ray casting query.
/// A body is active by default. If you set this
/// value to "false", all the proxy shapes of this body will be
/// removed from the broad-phase. If you set this value to "true",
/// all the proxy shapes will be added to the broad-phase. A joint
/// connected to an inactive body will also be inactive.
/**
 * @param isActive True if you want to activate the body
 */
void Body::setIsActive(bool isActive) {

    setIsSleeping(!isActive);

    mWorld.mBodyComponents.setIsActive(mEntity, isActive);

    RP3D_LOG(mLogger, Logger::Level::Information, Logger::Category::Body,
             "Body " + std::to_string(mEntity.id) + ": Set isActive=" +
             (isActive ? "true" : "false"));
}

// Set the variable to know whether or not the body is sleeping
void Body::setIsSleeping(bool isSleeping) {

    bool isBodySleeping = mWorld.mBodyComponents.getIsSleeping(mEntity);

    // If the body is not active, do nothing (it is sleeping)
    if (!mWorld.mBodyComponents.getIsActive(mEntity)) {
        assert(isBodySleeping);
        return;
    }

    if (isSleeping) {
        mWorld.mBodyComponents.setSleepTime(mEntity, decimal(0.0));
    }
    else {
        if (isBodySleeping) {
            mWorld.mBodyComponents.setSleepTime(mEntity, decimal(0.0));
        }
    }

    if (isBodySleeping != isSleeping) {

        mWorld.mBodyComponents.setIsSleeping(mEntity, isSleeping);

        RP3D_LOG(mLogger, Logger::Level::Information, Logger::Category::Body,
             "Body " + std::to_string(mEntity.id) + ": Set isSleeping=" +
             (isSleeping ? "true" : "false"));
    }
}

// Set whether or not the body is allowed to go to sleep
/**
 * @param isAllowedToSleep True if the body is allowed to sleep
 */
void Body::setIsAllowedToSleep(bool isAllowedToSleep) {

    mWorld.mBodyComponents.setIsAllowedToSleep(mEntity, isAllowedToSleep);

    if (!isAllowedToSleep) setIsSleeping(false);

    RP3D_LOG(mLogger, Logger::Level::Information, Logger::Category::Body,
             "Body " + std::to_string(mEntity.id) + ": Set isAllowedToSleep=" +
             (isAllowedToSleep ? "true" : "false"));
}

// Return whether or not the body is allowed to sleep
/**
 * @return True if the body is allowed to sleep and false otherwise
 */
bool Body::isAllowedToSleep() const {
    return mWorld.mBodyComponents.getIsAllowedToSleep(mEntity);
}

// Return whether or not the body is sleeping
/**
 * @return True if the body is currently sleeping and false otherwise
 */
bool Body::isSleeping() const {
    return mWorld.mBodyComponents.getIsSleeping(mEntity);
}

// Return true if the body is active
/**
 * @return True if the body currently active and false otherwise
 */
bool Body::isActive() const {
    return mWorld.mBodyComponents.getIsActive(mEntity);
}

// Return a pointer to the user data attached to this body
/**
 * @return A pointer to the user data you have attached to the body
 */
void* Body::getUserData() const {
    return mWorld.mBodyComponents.getUserData(mEntity);
}

// Attach user data to this body
/**
 * @param userData A pointer to the user data you want to attach to the body
 */
void Body::setUserData(void* userData) {
    mWorld.mBodyComponents.setUserData(mEntity, userData);
}
