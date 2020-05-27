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

#ifndef REACTPHYSICS3D_EVENT_LISTENER_H
#define REACTPHYSICS3D_EVENT_LISTENER_H

// Libraries
#include <reactphysics3d/collision/CollisionCallback.h>
#include <reactphysics3d/collision/OverlapCallback.h>

namespace reactphysics3d {

// Class EventListener
/**
 * This class can be used to receive notifications about events that occur during the simulation.
 * In order to receive callbacks, you need to create a new class that inherits from
 * this one and you must override the methods that you need. Then, you will need to register your
 * new event listener class to the physics world using the PhysicsWorld::setEventListener() method.
 */
class EventListener : public CollisionCallback {

    public :

        // ---------- Methods ---------- //

        /// Constructor
        EventListener() = default;

        /// Destructor
        virtual ~EventListener() override = default;

        /// Called when some contacts occur
        /**
         * @param callbackData Contains information about all the contacts
         */
        virtual void onContact(const CollisionCallback::CallbackData& callbackData) override {}

        /// Called when some trigger events occur
        /**
         * @param callbackData Contains information about all the triggers that are colliding
         */
        virtual void onTrigger(const OverlapCallback::CallbackData& callbackData) {}
};

}

#endif
