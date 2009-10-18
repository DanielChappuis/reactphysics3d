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

// Libraries
#include "CollisionDetection.h"
#include "SeparatingAxisAABB.h"
#include "SeparatingAxisOBB.h"
#include "../body/OBB.h"
#include "../body/RigidBody.h"
#include <cassert>

// We want to use the ReactPhysics3D namespace
using namespace reactphysics3d;

// Constructor
CollisionDetection::CollisionDetection() {

    // Construct the broad-phase algorithm that will be used (Separating axis with AABB)
    broadPhaseAlgorithm = new SeparatingAxisAABB();

    // Construct the narrow-phase algorithm that will be used (Separating axis with OBB)
    narrowPhaseAlgorithm = new SeparatingAxisOBB();
}

// Destructor
CollisionDetection::~CollisionDetection() {

}

// Compute the collision detection for the time interval [0, timeMax]
// The method returns true if a collision occurs in the time interval [0, timeMax]
bool CollisionDetection::computeCollisionDetection(CollisionWorld* collisionWorld) {

    bool existsCollision = false;               // True if a collision is found in the time interval [0, timeMax]

    // For each pair of bodies in the collisionWorld
    for(std::vector<Body*>::const_iterator it1 = collisionWorld->getBodyListStartIterator(); it1 != collisionWorld->getBodyListEndIterator(); ++it1) {
        for(std::vector<Body*>::const_iterator it2 = it1; it2 != collisionWorld->getBodyListEndIterator(); ++it2) {
            // If both bodies are RigidBody and are different
            RigidBody* rigidBody1 = dynamic_cast<RigidBody*>(*it1);
            RigidBody* rigidBody2 = dynamic_cast<RigidBody*>(*it2);
            if(rigidBody1 && rigidBody2 && rigidBody1 != rigidBody2) {
                // Get the oriented bounding boxes of the two bodies
                OBB obb1 = rigidBody1->getOBB();
                OBB obb2 = rigidBody2->getOBB();

                // Use the broad-phase algorithm to decide if the two bodies can collide
                if(broadPhaseAlgorithm->testCollisionPair(&obb1, &obb2)) {
                    Contact* contact = 0;

                    // Use the narrow-phase algorithm to check if the two bodies really collide
                    if(narrowPhaseAlgorithm->testCollision(&obb1, &obb2, &contact, velocity1, velocity2, timeMax)) {
                        assert(contact != 0);

                        // Add the new collision contact into the collision world
                        collisionWorld->addConstraint(contact);
                    }
                }
            }
        }
    }

    return existsCollision;
}
