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
#include "CollisionWorld.h"
#include "../constraint/Contact.h"

// We want to use the ReactPhysics3D namespace
using namespace reactphysics3d;

// Constructor
CollisionWorld::CollisionWorld(const Vector3D& gravity)
               :DynamicWorld(gravity) {

}

// Destructor
CollisionWorld::~CollisionWorld() {
    // Delete all the constraint
    for (std::vector<Constraint*>::iterator it = constraintList.begin(); it != constraintList.end(); ) {
        delete (*it);
    }
}

// Add a constraint into the collision world
void CollisionWorld::addConstraint(Constraint* constraint) throw(std::invalid_argument) {
    assert(constraint != 0);
    constraintList.push_back(constraint);
}

// Remove a constraint
void CollisionWorld::removeConstraint(Constraint* constraint) throw(std::invalid_argument) {
    // TODO : Implement this method
}

 // Remove all collision contacts constraints
void CollisionWorld::removeAllContactConstraints() {
    // For all constraints
    for (std::vector<Constraint*>::iterator it = constraintList.begin(); it != constraintList.end(); ) {

        // Try a downcasting
        Contact* contact = dynamic_cast<Contact*>(*it);

        // If the constraint is a contact
        if (contact != 0) {
            // Delete the  contact
            delete (*it);
            it = constraintList.erase(it);
        }
        else {
            ++it;
        }
    }
}
