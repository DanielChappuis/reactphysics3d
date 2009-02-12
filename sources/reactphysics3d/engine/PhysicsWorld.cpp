/****************************************************************************
* Copyright (C) 2009      Daniel Chappuis                                  *
****************************************************************************
* This file is part of ReactPhysics3D.                                     *
*                                                                          *
* ReactPhysics3D is free software: you can redistribute it and/or modify   *
* it under the terms of the GNU General Public License as published by     *
* the Free Software Foundation, either version 3 of the License, or        *
* (at your option) any later version.                                      *
*                                                                          *
* ReactPhysics3D is distributed in the hope that it will be useful,        *
* but WITHOUT ANY WARRANTY; without even the implied warranty of           *
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the             *
* GNU General Public License for more details.                             *
*                                                                          *
* You should have received a copy of the GNU General Public License        *
* along with ReactPhysics3D. If not, see <http://www.gnu.org/licenses/>.   *
***************************************************************************/

// Libraries
#include "PhysicsWorld.h"

// We want to use the ReactPhysics3D namespace
using namespace reactphysics3d;

// Constructor
PhysicsWorld::PhysicsWorld(const Vector3D& gravity) : gravity(gravity) {

}

// Copy-constructor
PhysicsWorld::PhysicsWorld(const PhysicsWorld& world) {
    bodyList = world.bodyList;
    gravity = world.gravity;
}

// Destructor
PhysicsWorld::~PhysicsWorld() {

}

// Add a body to the physics world
void PhysicsWorld::addBody(Body* body) throw(std::invalid_argument) {
    // Check if the body pointer is not null
    if (body != 0) {
        // Check if the body pointer isn't already in the bodyList
        for(std::vector<Body*>::iterator it = bodyList.begin(); it != bodyList.end(); ++it) {
            if (*it == body) {
                // The body is already in the bodyList, therefore we throw an exception
                throw std::invalid_argument("Exception in PhysicsWorld::addBody() : The argument body is already in the PhysicsWorld");
            }
        }

        // The body isn't already in the bodyList, therefore we add it to the list
        bodyList.push_back(body);
    }
    else {
        // Throw an exception
        throw std::invalid_argument("Exception in PhysicsWorld::addBody() : The argument pointer cannot be NULL");
    }
}

// Remove a body from the physics world
void PhysicsWorld::removeBody(Body const* const body) throw(std::invalid_argument) {
    // Check if the body pointer is not null
    if (body != 0) {
        // Look for the body to remove in the bodyList
        std::vector<Body*>::iterator it = bodyList.begin();
        while(it != bodyList.end() && *it != body) {
            // Increment the iterator
            ++it;
        }

        // If we have found the body to remove in the bodyList
        if (*it == body) {
            // Remove the body
            bodyList.erase(it);
        } else {
            // The body is not in the bodyList, therfore we throw an exception
            throw std::invalid_argument("Exception in PhysicsWorld::removeBody() : The argument body to remove is not in the PhysicsWorld");
        }
    }
    else {
        // Throw an exception
        throw std::invalid_argument("Exception in PhysicsWorld::removeBody() : The argument pointer cannot be NULL");
    }
}
