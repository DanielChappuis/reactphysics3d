/***************************************************************************
* Copyright (C) 2009      Daniel Chappuis                                  *
****************************************************************************
* This file is part of ReactPhysics3D.                                     *
*                                                                          *
* ReactPhysics3D is free software: you can redistribute it and/or modify   *
* it under the terms of the GNU Lesser General Public License as published *
* by the Free Software Foundation, either version 3 of the License, or     *
* (at your option) any later version.                                      *
*                                                                          *
* ReactPhysics3D is distributed in the hope that it will be useful,        *
* but WITHOUT ANY WARRANTY; without even the implied warranty of           *
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the             *
* GNU Lesser General Public License for more details.                      *
*                                                                          *
* You should have received a copy of the GNU Lesser General Public License *
* along with ReactPhysics3D. If not, see <http://www.gnu.org/licenses/>.   *
***************************************************************************/

#ifndef COLLISIONDETECTION_H
#define COLLISIONDETECTION_H

// Libraries
#include "BroadPhaseAlgorithm.h"
#include "NarrowPhaseAlgorithm.h"
#include "../body/Body.h"
#include "../engine/PhysicsWorld.h"
#include "ContactInfo.h"
#include <vector>

// ReactPhysics3D namespace
namespace reactphysics3d {

/*  -------------------------------------------------------------------
    Class CollisionDetection :
        This class computes the collision detection algorithms. We first
        perfom a broad-phase algorithm to know wich pairs of bodies can
        collide and then we run a narrow-phase algorithm to compute the
        collision contacts between bodies.
    -------------------------------------------------------------------
*/
class CollisionDetection {
    private :
        PhysicsWorld* world;                                                         // Pointer to the physics world
        std::vector<std::pair<const OBB*, const OBB* > > possibleCollisionPairs;     // Possible collision pairs of bodies (computed by broadphase)
        std::vector<ContactInfo*> contactInfos;                                      // Contact informations (computed by narrowphase)

        // TODO : Check if we can avoid pointers for the two following classes (to avoid dynamic alocation)
        BroadPhaseAlgorithm* broadPhaseAlgorithm;                                   // Broad-phase algorithm
        NarrowPhaseAlgorithm* narrowPhaseAlgorithm;                                 // Narrow-phase algorithm

        void computeBroadPhase();                                                                                                                                   // Compute the broad-phase collision detection
        void computeNarrowPhase();                                                                                                                                  // Compute the narrow-phase collision detection
        void computeAllContacts();                                                                                                                                  // Compute all the contacts from the collision info list
        void computeContact(const ContactInfo* const contactInfo);                  // Compute a contact (and add it to the physics world) for two colliding bodies

    public :
        CollisionDetection(PhysicsWorld* physicsWorld);                             // Constructor
        ~CollisionDetection();                                                      // Destructor

        bool computeCollisionDetection();                                           // Compute the collision detection
};

} // End of the ReactPhysics3D namespace

#endif
