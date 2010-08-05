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

#ifndef PHYSICSWORLD_H
#define PHYSICSWORLD_H

// Libraries
#include <vector>
#include <stdexcept>
#include "../mathematics/mathematics.h"
#include "../body/Body.h"
#include "../constraint/Constraint.h"
#include "../constraint/Contact.h"

// Namespace reactphysics3d
namespace reactphysics3d {

/*  -------------------------------------------------------------------
    Class PhysicsWorld :
        This class represents the world of the
        physics engine. The physics world contains all the bodies of the physics
        engine.
    -------------------------------------------------------------------
*/
class PhysicsWorld {
    protected :
        std::vector<Body*> bodies;                  // list that contains all bodies of the physics world
        std::vector<Body*> addedBodies;             // Added bodies since last update
        std::vector<Body*> removedBodies;           // Removed bodies since last update
        std::vector<Constraint*> constraints;       // List that contains all the current constraints
        Vector3D gravity;                           // Gravity vector of the world
        bool isGravityOn;                           // True if the gravity force is on

    public :
        PhysicsWorld(const Vector3D& gravity);      // Constructor
        virtual ~PhysicsWorld();                    // Destructor

        void addBody(Body* body) throw(std::invalid_argument);                  // Add a body to the physics world
        void removeBody(Body const* const body) throw(std::invalid_argument);   // Remove a body from the physics world
        void clearAddedAndRemovedBodies();                                      // Clear the addedBodies and removedBodies sets
        Vector3D getGravity() const;                                            // Return the gravity vector of the world
        bool getIsGravityOn() const;                                            // Return if the gravity is on
        void setIsGratityOn(bool isGravityOn);                                  // Set the isGravityOn attribute
        void addConstraint(Constraint* constraint) throw(std::invalid_argument);            // Add a constraint
        void removeConstraint(Constraint* constraint) throw(std::invalid_argument);         // Remove a constraint
        void removeAllContactConstraints();                                                 // Remove all collision contacts constraints
        std::vector<Constraint*>::iterator getConstraintsBeginIterator();                   // Return a start iterator on the constraint list
        std::vector<Constraint*>::iterator getConstraintsEndIterator();                     // Return a end iterator on the constraint list
        std::vector<Body*>::iterator getBodiesBeginIterator();                              // Return an iterator to the beginning of the bodies of the physics world
        std::vector<Body*>::iterator getBodiesEndIterator();                                // Return an iterator to the end of the bodies of the physics world
        std::vector<Body*>& getAddedBodies();                                               // Return the added bodies since last update of the physics engine
        std::vector<Body*>& getRemovedBodies();                                             // Retrun the removed bodies since last update of the physics engine
};

// --- Inline functions --- //

// Clear the addedBodies and removedBodies sets
inline void PhysicsWorld::clearAddedAndRemovedBodies() {
    addedBodies.clear();
    removedBodies.clear();
}

// Return the gravity vector of the world
inline Vector3D PhysicsWorld::getGravity() const {
    return gravity;
}

// Return if the gravity is on
inline bool PhysicsWorld::getIsGravityOn() const {
    return isGravityOn;
}

// Set the isGravityOn attribute
inline void PhysicsWorld::setIsGratityOn(bool isGravityOn) {
    this->isGravityOn = isGravityOn;
}

// Return a start iterator on the constraint list
inline std::vector<Constraint*>::iterator PhysicsWorld::getConstraintsBeginIterator() {
    return constraints.begin();
}

// Return a end iterator on the constraint list
inline std::vector<Constraint*>::iterator PhysicsWorld::getConstraintsEndIterator() {
    return constraints.end();
}

// Return an iterator to the beginning of the bodies of the physics world
inline std::vector<Body*>::iterator PhysicsWorld::getBodiesBeginIterator() {
    return bodies.begin();
}

// Return an iterator to the end of the bodies of the physics world
inline std::vector<Body*>::iterator PhysicsWorld::getBodiesEndIterator() {
    return bodies.end();
}

// Return the added bodies since last update of the physics engine
inline std::vector<Body*>& PhysicsWorld::getAddedBodies() {
    return addedBodies;
}

// Retrun the removed bodies since last update of the physics engine
inline std::vector<Body*>& PhysicsWorld::getRemovedBodies() {
    return removedBodies;
}

}   // End of the ReactPhysics3D namespace

 #endif