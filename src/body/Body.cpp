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
#include "collision/shapes/CollisionShape.h"
#include "utils/Logger.h"

// We want to use the ReactPhysics3D namespace
using namespace reactphysics3d;

// Constructor
/**
 * @param id ID of the new body
 */
Body::Body(bodyindex id)
     : mID(id), mIsAlreadyInIsland(false), mIsAllowedToSleep(true), mIsActive(true),
       mIsSleeping(false), mSleepTime(0), mUserData(nullptr) {

#ifdef IS_LOGGING_ACTIVE
        mLogger = nullptr;
#endif

}

// Set whether or not the body is active
/**
 * @param isActive True if you want to activate the body
 */
void Body::setIsActive(bool isActive) {
    mIsActive = isActive;

    RP3D_LOG(mLogger, Logger::Level::Information, Logger::Category::Body,
             "Body " + std::to_string(mID) + ": Set isActive=" +
             (mIsActive ? "true" : "false"));
}

// Set the variable to know whether or not the body is sleeping
void Body::setIsSleeping(bool isSleeping) {

    if (isSleeping) {
        mSleepTime = decimal(0.0);
    }
    else {
        if (mIsSleeping) {
            mSleepTime = decimal(0.0);
        }
    }

    if (mIsSleeping != isSleeping) {

        mIsSleeping = isSleeping;

        RP3D_LOG(mLogger, Logger::Level::Information, Logger::Category::Body,
             "Body " + std::to_string(mID) + ": Set isSleeping=" +
             (mIsSleeping ? "true" : "false"));
    }
}

// Set whether or not the body is allowed to go to sleep
/**
 * @param isAllowedToSleep True if the body is allowed to sleep
 */
void Body::setIsAllowedToSleep(bool isAllowedToSleep) {
    mIsAllowedToSleep = isAllowedToSleep;

    if (!mIsAllowedToSleep) setIsSleeping(false);

    RP3D_LOG(mLogger, Logger::Level::Information, Logger::Category::Body,
             "Body " + std::to_string(mID) + ": Set isAllowedToSleep=" +
             (mIsAllowedToSleep ? "true" : "false"));
}

