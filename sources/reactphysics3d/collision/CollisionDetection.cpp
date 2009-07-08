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

// We want to use the ReactPhysics3D namespace
using namespace reactphysics3d;

// Constructor
CollisionDetection::CollisionDetection() {

    // Construct the broad-phase algorithm that will be used
    broadPhaseAlgorithm =

    // Construct the narrow-phase algorithm that will be used
    narrowPhaseAlgorithm = new SeparatingAxis();
}

// Destructor
~CollisionDetection() {

}

// Compute all the possible collisions pairs of bodies (broad-phase)
void CollisionDetection::computePossibleCollisionPairs() {
    // TODO : Implement this method
}

// Compute all collision contacts between bodies (narrow-phase)
void CollisionDetection::computeCollisionContacts() {
    // TODO : Implement this method
}

// Compute the collision detection
void CollisionDetection::computeCollisionDetection(CollisionWorld& collisionWorld) {
    // TODO : Implement this method
}
