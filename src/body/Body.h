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

#ifndef REACTPHYSICS3D_BODY_H
#define REACTPHYSICS3D_BODY_H

// Libraries
#include <cassert>
#include "configuration.h"
#include "engine/Entity.h"

/// Namespace reactphysics3d
namespace reactphysics3d {

// Declarations
class Logger;
class CollisionWorld;

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

        /// Identifier of the entity in the ECS
        Entity mEntity;

        /// Reference to the world the body belongs to
        CollisionWorld& mWorld;

#ifdef IS_LOGGING_ACTIVE

        /// Logger
        Logger* mLogger;
#endif

        // -------------------- Methods -------------------- //

        /// Set the variable to know whether or not the body is sleeping
        virtual void setIsSleeping(bool isSleeping);

    public :

        // -------------------- Methods -------------------- //

        /// Constructor
        Body(Entity entity, CollisionWorld& world);

        /// Deleted copy-constructor
        Body(const Body& body) = delete;

        /// Deleted assignment operator
        Body& operator=(const Body& body) = delete;

        /// Destructor
        virtual ~Body() = default;

        /// Return the corresponding entity of the body
        Entity getEntity() const;

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

#ifdef IS_LOGGING_ACTIVE

        /// Set the logger
        void setLogger(Logger* logger);
#endif

        // -------------------- Friendship -------------------- //

        friend class DynamicsWorld;
};

// Return the corresponding entity of the body
/**
 * @return The entity of the body
 */
inline Entity Body::getEntity() const {
    return mEntity;
}

#ifdef IS_LOGGING_ACTIVE

// Set the logger
inline void Body::setLogger(Logger* logger) {
    mLogger = logger;
}

#endif

}

#endif
