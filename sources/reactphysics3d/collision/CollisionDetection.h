/********************************************************************************
* ReactPhysics3D physics library, http://code.google.com/p/reactphysics3d/      *
* Copyright (c) 2010 Daniel Chappuis                                            *
*********************************************************************************
*                                                                               *
* Permission is hereby granted, free of charge, to any person obtaining a copy  *
* of this software and associated documentation files (the "Software"), to deal *
* in the Software without restriction, including without limitation the rights  *
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell     *
* copies of the Software, and to permit persons to whom the Software is         *
* furnished to do so, subject to the following conditions:                      *
*                                                                               *
* The above copyright notice and this permission notice shall be included in    *
* all copies or substantial portions of the Software.                           *
*                                                                               *
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR    *
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,      *
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE   *
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER        *
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, *
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN     *
* THE SOFTWARE.                                                                 *
********************************************************************************/

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
        PhysicsWorld* world;                                                        // Pointer to the physics world
        std::vector<std::pair<const Body*, const Body* > > possibleCollisionPairs;  // Possible collision pairs of bodies (computed by broadphase)
        std::vector<ContactInfo*> contactInfos;                                     // Contact informations (computed by narrowphase)
        BroadPhaseAlgorithm* broadPhaseAlgorithm;                                   // Broad-phase algorithm
        NarrowPhaseAlgorithm* narrowPhaseAlgorithm;                                 // Narrow-phase algorithm

        void computeBroadPhase();                                                   // Compute the broad-phase collision detection
        void computeNarrowPhase();                                                  // Compute the narrow-phase collision detection
        void computeAllContacts();                                                  // Compute all the contacts from the collision info list
        void computeContact(const ContactInfo* const contactInfo);                  // Compute a contact (and add it to the physics world) for two colliding bodies
        void computeContact2(const ContactInfo* const contactInfo);                  // Compute a contact (and add it to the physics world) for two colliding bodies

    public :
        CollisionDetection(PhysicsWorld* physicsWorld);                             // Constructor
        ~CollisionDetection();                                                      // Destructor

        bool computeCollisionDetection();                                           // Compute the collision detection
};

} // End of the ReactPhysics3D namespace

#endif
