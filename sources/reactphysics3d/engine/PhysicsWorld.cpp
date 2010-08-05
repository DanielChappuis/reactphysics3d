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

}

// Add a body to the physics world
void PhysicsWorld::addBody(Body* body) throw(invalid_argument) {
    // Check if the body pointer is not null
    if (body != 0) {
        // Check if the body pointer isn't already in the bodyList
        for(vector<Body*>::iterator it = bodies.begin(); it != bodies.end(); ++it) {
            if (*it == body) {
                // The body is already in the bodyList, therefore we throw an exception
                throw invalid_argument("Exception in PhysicsWorld::addBody() : The argument body is already in the PhysicsWorld");
            }
        }

        // The body isn't already in the bodyList, therefore we add it to the list
        bodies.push_back(body);
        addedBodies.push_back(body);
        vector<Body*>::iterator it = find(removedBodies.begin(), removedBodies.end(), body);
        if (it != removedBodies.end()) {
            removedBodies.erase(it);
        }
    }
    else {
        // Throw an exception
        throw invalid_argument("Exception in PhysicsWorld::addBody() : The argument pointer cannot be NULL");
    }
}

// Remove a body from the physics world
void PhysicsWorld::removeBody(Body const* const body) throw(invalid_argument) {
    // Check if the body pointer is not null
    if (body != 0) {
        // Look for the body to remove in the bodyList
        vector<Body*>::iterator it = bodies.begin();
        while(it != bodies.end() && *it != body) {
            // Increment the iterator
            ++it;
        }

        // If we have found the body to remove in the bodyList
        if (*it == body) {
            // Remove the body
            bodies.erase(it);
            addedBodies.erase(it);
            removedBodies.push_back(*it);
        } else {
            // The body is not in the bodyList, therfore we throw an exception
            throw invalid_argument("Exception in PhysicsWorld::removeBody() : The argument body to remove is not in the PhysicsWorld");
        }
    }
    else {
        // Throw an exception
        throw invalid_argument("Exception in PhysicsWorld::removeBody() : The argument pointer cannot be NULL");
    }
}

// Add a constraint into the physics world
void PhysicsWorld::addConstraint(Constraint* constraint) throw(invalid_argument) {
    assert(constraint != 0);
    constraints.push_back(constraint);
}

// Remove a constraint
void PhysicsWorld::removeConstraint(Constraint* constraint) throw(invalid_argument) {
    // TODO : Implement this method
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

