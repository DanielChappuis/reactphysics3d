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

#ifndef PHYSICSEVENTLISTENER_H
#define PHYSICSEVENTLISTENER_H

// Libraries
#include "reactphysics3d.h"
#include "openglframework.h"

// Structure ContactPoint
struct ContactPoint {

    public:
        openglframework::Vector3 point;

        /// Constructor
        ContactPoint(const openglframework::Vector3& contactPoint) : point(contactPoint) {

        }
};

// Class PhysicsEventListener
// This class inherits from the EventListener class
// of ReactPhysics3D in order to be notified of events
// that occured in a physics world
class PhysicsEventListener : rp3d::EventListener {

    private:

        // Current contact points
        std::vector<ContactPoint> mCurrentContactPoints;

    public:

        /// Called when a new contact point is found between two bodies that were separated before
        virtual void beginContact(const rp3d::ContactPointInfo& contact);

        /// Called when a new contact point is found between two bodies
        virtual void newContact(const rp3d::ContactPointInfo& contact);
};

#endif

