/********************************************************************************
* ReactPhysics3D physics library, http://code.google.com/p/reactphysics3d/      *
* Copyright (c) 2010-2013 Daniel Chappuis                                       *
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
#include <stdexcept>
#include <cassert>
#include "../configuration.h"

/// Namespace reactphysics3d
namespace reactphysics3d {

// Class Body
/**
 * This class is an abstract class to represent a body of the physics engine.
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

        /// True if the body is sleeping (for sleeping technique)
        bool mIsSleeping;

        // -------------------- Methods -------------------- //

        /// Private copy-constructor
        Body(const Body& body);

        /// Private assignment operator
        Body& operator=(const Body& body);

    public :

        // -------------------- Methods -------------------- //

        /// Constructor
        Body(bodyindex id);

        /// Destructor
        virtual ~Body();

        /// Return the id of the body
        bodyindex getID() const;

        /// Return true if the body has already been added in an island (for the sleeping technique)
        bool isAlreadyInIsland() const;

        /// Set the value of to know if the body has already been added into an island
        void setIsAlreadyInIsland(bool isAlreadyInIsland);

        /// Return whether or not the body is allowed to sleep
        bool isAllowedToSleep() const;

        /// Set whether or not the body is allowed to go to sleep
        void setIsAllowedToSleep(bool isAllowedToSleep);

        /// Return whether or not the body is sleeping
        bool isSleeping() const;

        /// Set the variable to know whether or not the body is sleeping
        void setIsSleeping(bool isSleeping);

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
inline bodyindex Body::getID() const {
    return mID;
}

// Return true if the body has already been added in an island (for the sleeping technique)
inline bool Body::isAlreadyInIsland() const {
    return mIsAlreadyInIsland;
}

// Set the value of to know if the body has already been added into an island
inline void Body::setIsAlreadyInIsland(bool isAlreadyInIsland) {
    mIsAlreadyInIsland = isAlreadyInIsland;
}

// Return whether or not the body is allowed to sleep
inline bool Body::isAllowedToSleep() const {
    return mIsAllowedToSleep;
}

// Set whether or not the body is allowed to go to sleep
inline void Body::setIsAllowedToSleep(bool isAllowedToSleep) {
    mIsAllowedToSleep = isAllowedToSleep;
}

// Return whether or not the body is sleeping
inline bool Body::isSleeping() const {
    return mIsSleeping;
}

// Set the variable to know whether or not the body is sleeping
inline void Body::setIsSleeping(bool isSleeping) {
    mIsSleeping = isSleeping;
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
