/****************************************************************************
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
#include "PhysicsWorld.h"
#include <algorithm>

// Namespaces
using namespace reactphysics3d;
using namespace std;

// Constructor
PhysicsWorld::PhysicsWorld(const Vector3D& gravity)
             : gravity(gravity), isGravityOn(true) {

}

// Destructor
PhysicsWorld::~PhysicsWorld() {
    // Remove and free the memory of all constraints
    removeAllConstraints();
}

// Remove all collision contacts constraints
void PhysicsWorld::removeAllContactConstraints() {
    // For all constraints
    for (vector<Constraint*>::iterator it = constraints.begin(); it != constraints.end(); ) {

        // Try a downcasting
        Contact* contact = dynamic_cast<Contact*>(*it);

        // If the constraint is a contact
        if (contact != 0) {
            // Delete the  contact
            delete (*it);
            it = constraints.erase(it);
        }
        else {
            ++it;
        }
    }
}

// Remove all constraints in the physics world and also delete them (free their memory)
void PhysicsWorld::removeAllConstraints() {
    for (vector<Constraint*>::iterator it = constraints.begin(); it != constraints.end(); it++) {
        delete *it;
    }
    constraints.clear();
}

