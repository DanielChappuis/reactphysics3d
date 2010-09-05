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
#include <algorithm>
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

        void addBody(Body* body);                                               // Add a body to the physics world
        void removeBody(Body const* const body);                                // Remove a body from the physics world
        void clearAddedAndRemovedBodies();                                      // Clear the addedBodies and removedBodies sets
        Vector3D getGravity() const;                                            // Return the gravity vector of the world
        bool getIsGravityOn() const;                                            // Return if the gravity is on
        void setIsGratityOn(bool isGravityOn);                                  // Set the isGravityOn attribute
        void addConstraint(Constraint* constraint);                             // Add a constraint
        void removeConstraint(Constraint* constraint);                          // Remove a constraint
        void removeAllContactConstraints();                                     // Remove all collision contacts constraints
        void removeAllConstraints();                                            // Remove all constraints and delete them (free their memory)
        std::vector<Constraint*>::iterator getConstraintsBeginIterator();       // Return a start iterator on the constraint list
        std::vector<Constraint*>::iterator getConstraintsEndIterator();         // Return a end iterator on the constraint list
        std::vector<Body*>::iterator getBodiesBeginIterator();                  // Return an iterator to the beginning of the bodies of the physics world
        std::vector<Body*>::iterator getBodiesEndIterator();                    // Return an iterator to the end of the bodies of the physics world
        std::vector<Body*>& getAddedBodies();                                   // Return the added bodies since last update of the physics engine
        std::vector<Body*>& getRemovedBodies();                                 // Retrun the removed bodies since last update of the physics engine
};

// Add a body to the physics world
inline void PhysicsWorld::addBody(Body* body) {
    std::vector<Body*>::iterator it;

    assert(body);
    it = std::find(bodies.begin(), bodies.end(), body);
    assert(it == bodies.end());

    // The body isn't already in the bodyList, therefore we add it to the list
    bodies.push_back(body);
    addedBodies.push_back(body);
    it = std::find(removedBodies.begin(), removedBodies.end(), body);
    if (it != removedBodies.end()) {
        removedBodies.erase(it);
    }
}

// Remove a body from the physics world
inline void PhysicsWorld::removeBody(Body const* const body) {
    std::vector<Body*>::iterator it;

    assert(body);
    it = std::find(bodies.begin(), bodies.end(), body);
    assert(*it == body);

    // Remove the body
    bodies.erase(it);
    addedBodies.erase(it);
    removedBodies.push_back(*it);
}

// Add a constraint into the physics world
inline void PhysicsWorld::addConstraint(Constraint* constraint) {
    assert(constraint != 0);
    constraints.push_back(constraint);
}

// Remove a constraint and free its memory
inline void PhysicsWorld::removeConstraint(Constraint* constraint) {
    std::vector<Constraint*>::iterator it;

    assert(constraint);
    it = std::find(constraints.begin(), constraints.end(), constraint);
    assert(*it == constraint);
    delete *it;
    constraints.erase(it);
}

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