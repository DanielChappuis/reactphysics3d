/********************************************************************************
* ReactPhysics3D physics library, http://www.reactphysics3d.com                 *
* Copyright (c) 2010-2015 Daniel Chappuis                                       *
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

#ifndef REACTPHYSICS3D_BODY_H
#define REACTPHYSICS3D_BODY_H

// Libraries
#include <cassert>
#include <stdexcept>
#include "reactphysics3d/configuration.h"

/// ReactPhysics3D Namespace
namespace reactphysics3d {

// TODO : Make this class abstract
// Class Body
/**
 * This class to represent a body of the physics engine. You should not
 * instantiante this class but instantiate the CollisionBody or RigidBody
 * classes instead.
 */
class Body {

    protected :

        // -------------------- Attributes -------------------- //

        /// ID of the body
        bodyindex mID;

        /// True if the body has already been added in an island (for sleeping technique)
        bool mIsAlreadyInIsland;

        /// True if the body is allowed to go to sleep for better efficiency
        bool mIsAllowedToSleep;

        /// True if the body is active.
        /// An inactive body does not participate in collision detection,
        /// is not simulated and will not be hit in a ray casting query.
        /// A body is active by default. If you set this
        /// value to "false", all the proxy shapes of this body will be
        /// removed from the broad-phase. If you set this value to "true",
        /// all the proxy shapes will be added to the broad-phase. A joint
        /// connected to an inactive body will also be inactive.
        bool mIsActive;

        /// True if the body is sleeping (for sleeping technique)
        bool mIsSleeping;

        /// Elapsed time since the body velocity was bellow the sleep velocity
        decimal mSleepTime;

        /// Pointer that can be used to attach user data to the body
        void* mUserData;

        // -------------------- Methods -------------------- //

        /// Private copy-constructor
        Body(const Body& body);

        /// Private assignment operator
        Body& operator=(const Body& body);

        /// Set the variable to know whether or not the body is sleeping
        virtual void setIsSleeping(bool isSleeping);

    public :

        // -------------------- Methods -------------------- //

        /// Constructor
        Body(bodyindex id);

        /// Destructor
        virtual ~Body();

        /// Return the ID of the body
        bodyindex getID() const;

        /// Return whether or not the body is allowed to sleep
        bool isAllowedToSleep() const;

        /// Set whether or not the body is allowed to go to sleep
        void setIsAllowedToSleep(bool isAllowedToSleep);

        /// Return whether or not the body is sleeping
        bool isSleeping() const;

        /// Return true if the body is active
        bool isActive() const;

        /// Set whether or not the body is active
        virtual void setIsActive(bool isActive);

        /// Return a pointer to the user data attached to this body
        void* getUserData() const;

        /// Attach user data to this body
        void setUserData(void* userData);

        /// Smaller than operator
        bool operator<(const Body& body2) const;

        /// Larger than operator
        bool operator>(const Body& body2) const;

        /// Equal operator
        bool operator==(const Body& body2) const;

        /// Not equal operator
        bool operator!=(const Body& body2) const;

        // -------------------- Friendship -------------------- //

        friend class DynamicsWorld;
};

// Return the id of the body
/**
 * @return The ID of the body
 */
inline bodyindex Body::getID() const {
    return mID;
}

// Return whether or not the body is allowed to sleep
/**
 * @return True if the body is allowed to sleep and false otherwise
 */
inline bool Body::isAllowedToSleep() const {
    return mIsAllowedToSleep;
}

// Set whether or not the body is allowed to go to sleep
/**
 * @param isAllowedToSleep True if the body is allowed to sleep
 */
inline void Body::setIsAllowedToSleep(bool isAllowedToSleep) {
    mIsAllowedToSleep = isAllowedToSleep;

    if (!mIsAllowedToSleep) setIsSleeping(false);
}

// Return whether or not the body is sleeping
/**
 * @return True if the body is currently sleeping and false otherwise
 */
inline bool Body::isSleeping() const {
    return mIsSleeping;
}

// Return true if the body is active
/**
 * @return True if the body currently active and false otherwise
 */
inline bool Body::isActive() const {
    return mIsActive;
}

// Set whether or not the body is active
/**
 * @param isActive True if you want to activate the body
 */
inline void Body::setIsActive(bool isActive) {
    mIsActive = isActive;
}

// Set the variable to know whether or not the body is sleeping
inline void Body::setIsSleeping(bool isSleeping) {

    if (isSleeping) {
        mSleepTime = decimal(0.0);
    }
    else {
        if (mIsSleeping) {
            mSleepTime = decimal(0.0);
        }
    }

    mIsSleeping = isSleeping;
}

// Return a pointer to the user data attached to this body
/**
 * @return A pointer to the user data you have attached to the body
 */
inline void* Body::getUserData() const {
    return mUserData;
}

// Attach user data to this body
/**
 * @param userData A pointer to the user data you want to attach to the body
 */
inline void Body::setUserData(void* userData) {
    mUserData = userData;
}

// Smaller than operator
inline bool Body::operator<(const Body& body2) const {
    return (mID < body2.mID);
} 

// Larger than operator
inline bool Body::operator>(const Body& body2) const {
    return (mID > body2.mID);
} 

// Equal operator
inline bool Body::operator==(const Body& body2) const {
    return (mID == body2.mID);
}
        
// Not equal operator
inline bool Body::operator!=(const Body& body2) const {
    return (mID != body2.mID);
}               

}

 #endif
