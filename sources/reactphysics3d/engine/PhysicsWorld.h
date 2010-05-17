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

#ifndef WORLD_H
#define WORLD_H

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
        std::vector<Constraint*> constraints;       // List that contains all the current constraints
        Vector3D gravity;                           // Gravity vector of the world
        bool isGravityOn;                           // True if the gravity force is on

    public :
        PhysicsWorld(const Vector3D& gravity);      // Constructor
        virtual ~PhysicsWorld();                    // Destructor

        void addBody(Body* body) throw(std::invalid_argument);                  // Add a body to the physics world
        void removeBody(Body const* const body) throw(std::invalid_argument);   // Remove a body from the physics world
        Vector3D getGravity() const;                                            // Return the gravity vector of the world
        bool getIsGravityOn() const;                                            // Return if the gravity is on
        void setIsGratityOn(bool isGravityOn);                                  // Set the isGravityOn attribute
        std::vector<Body*>::const_iterator getBodyListStartIterator() const;    // Return a start iterator on the body list
        std::vector<Body*>::const_iterator getBodyListEndIterator() const;      // Return a end iterator on the body list
        void addConstraint(Constraint* constraint) throw(std::invalid_argument);            // Add a constraint
        void removeConstraint(Constraint* constraint) throw(std::invalid_argument);         // Remove a constraint
        void removeAllContactConstraints();                                                 // Remove all collision contacts constraints
        std::vector<Constraint*>::const_iterator getConstraintListStartIterator() const;    // Return a start iterator on the constraint list
        std::vector<Constraint*>::const_iterator getConstraintListEndIterator() const;      // Return a end iterator on the constraint list
        std::vector<Body*>& getBodies();                                                    // Return a reference to the bodies of the physics world
        std::vector<Constraint*>& getConstraints();                                         // Return a reference to the constraints of the physics world
};

// --- Inline functions --- //

// Return the gravity vector of the world
inline Vector3D PhysicsWorld::getGravity() const {
    return gravity;
}

// Return a start iterator on the body list
inline std::vector<Body*>::const_iterator PhysicsWorld::getBodyListStartIterator() const {
    // Return an iterator on the start of the body list
    return bodies.begin();
}

// Return a end iterator on the body list
inline std::vector<Body*>::const_iterator PhysicsWorld::getBodyListEndIterator() const {
    // Return an iterator on the end of the body list
    return bodies.end();
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
inline std::vector<Constraint*>::const_iterator PhysicsWorld::getConstraintListStartIterator() const {
    return constraints.begin();
}

// Return a end iterator on the constraint list
inline std::vector<Constraint*>::const_iterator PhysicsWorld::getConstraintListEndIterator() const {
    return constraints.end();
}

// Return a reference to the bodies of the physics world
std::vector<Body*>& PhysicsWorld::getBodies() {
    return bodies;
}

// Return a reference to the constraints of the physics world
std::vector<Constraint*>& PhysicsWorld::getConstraints() {
    return constraints;
}

}   // End of the ReactPhysics3D namespace

 #endif
