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

#ifndef REACTPHYSICS3D_EVENT_LISTENER_H
#define REACTPHYSICS3D_EVENT_LISTENER_H

// Libraries
#include "collision/CollisionCallback.h"

namespace reactphysics3d {

// Class EventListener
/**
 * This class can be used to receive event callbacks from the physics engine.
 * In order to receive callbacks, you need to create a new class that inherits from
 * this one and you must override the methods you need. Then, you need to register your
 * new event listener class to the physics world using the DynamicsWorld::setEventListener()
 * method.
 */
class EventListener {

    public :

        /// Constructor
        EventListener() = default;

        /// Destructor
        virtual ~EventListener() = default;

        /// Called when a new contact point is found between two bodies
        /**
         * @param contact Information about the contact
         */
        virtual void newContact(const CollisionCallback::CollisionCallbackInfo& collisionInfo) {}

        /// Called at the beginning of an internal tick of the simulation step.
        /// Each time the DynamicsWorld::update() method is called, the physics
        /// engine will do several internal simulation steps. This method is
        /// called at the beginning of each internal simulation step.
        virtual void beginInternalTick() {}

        /// Called at the end of an internal tick of the simulation step.
        /// Each time the DynamicsWorld::update() metho is called, the physics
        /// engine will do several internal simulation steps. This method is
        /// called at the end of each internal simulation step.
        virtual void endInternalTick() {}
};

}

#endif
