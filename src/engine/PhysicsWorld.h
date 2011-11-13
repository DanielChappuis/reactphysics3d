/********************************************************************************
* ReactPhysics3D physics library, http://code.google.com/p/reactphysics3d/      *
* Copyright (c) 2010-2012 Daniel Chappuis                                       *
*********************************************************************************
*                                                                               *
* This software is provided 'as-is', without any express or implied warranty.   *
* In no event will the authors be held liable for any damages arising from the  *
* use of this software.                                                         *
*                                                                               *
* Permission is granted to anyone to use this software for any purpose,         *
* including commercial applications, and to alter it and redistribute it        *
* freely, subject to the following restrictions:                                *
*                                                                               *
* 1. The origin of this software must not be misrepresented; you must not claim *
*    that you wrote the original software. If you use this software in a        *
*    product, an acknowledgment in the product documentation would be           *
*    appreciated but is not required.                                           *
*                                                                               *
* 2. Altered source versions must be plainly marked as such, and must not be    *
*    misrepresented as being the original software.                             *
*                                                                               *
* 3. This notice may not be removed or altered from any source distribution.    *
*                                                                               *
********************************************************************************/

#ifndef PHYSICS_WORLD_H
#define PHYSICS_WORLD_H

// Libraries
#include <vector>
#include <algorithm>
#include "../mathematics/mathematics.h"
#include "../body/Body.h"
#include "../constraint/Constraint.h"
#include "../constraint/Contact.h"
#include "../memory/MemoryPool.h"

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
        std::vector<RigidBody*> rigidBodies;            // All the rigid bodies of the physics world
        std::vector<RigidBody*> addedBodies;            // Added bodies since last update
        std::vector<RigidBody*> removedBodies;          // Removed bodies since last update
        std::vector<Constraint*> constraints;           // List that contains all the current constraints
        Vector3 gravity;                                // Gravity vector of the world
        bool isGravityOn;                               // True if the gravity force is on
        long unsigned int currentBodyID;                // Current body ID
        MemoryPool<RigidBody> memoryPoolRigidBodies;    // Memory pool for rigid bodies memory allocation

        void addRigidBody(RigidBody* body);         // Add a body to the physics world
        void removeRigidBody(RigidBody* body);      // Remove a body from the physics world
        
    public :
        PhysicsWorld(const Vector3& gravity);      // Constructor
        virtual ~PhysicsWorld();                   // Destructor

        RigidBody* createRigidBody(const Transform& transform, double mass,
                                   const Matrix3x3& inertiaTensorLocal, Shape* shape);  // Create a rigid body into the physics world
        void destroyRigidBody(RigidBody* rigidBody);                                    // Destroy a rigid body
        void clearAddedAndRemovedBodies();                                              // Clear the addedBodies and removedBodies sets
        Vector3 getGravity() const;                                                     // Return the gravity vector of the world
        bool getIsGravityOn() const;                                                    // Return if the gravity is on
        void setIsGratityOn(bool isGravityOn);                                          // Set the isGravityOn attribute
        void addConstraint(Constraint* constraint);                                     // Add a constraint
        void removeConstraint(Constraint* constraint);                                  // Remove a constraint
        void removeAllContactConstraints();                                             // Remove all collision contacts constraints
        void removeAllConstraints();                                                    // Remove all constraints and delete them (free their memory)
        std::vector<Constraint*>::iterator getConstraintsBeginIterator();               // Return a start iterator on the constraint list
        std::vector<Constraint*>::iterator getConstraintsEndIterator();                 // Return a end iterator on the constraint list
        std::vector<RigidBody*>::iterator getRigidBodiesBeginIterator();                // Return an iterator to the beginning of the bodies of the physics world
        std::vector<RigidBody*>::iterator getRigidBodiesEndIterator();                  // Return an iterator to the end of the bodies of the physics world
        std::vector<RigidBody*>& getAddedRigidBodies();                                 // Return the added bodies since last update of the physics engine
        std::vector<RigidBody*>& getRemovedRigidBodies();                               // Retrun the removed bodies since last update of the physics engine
};
                         

// Add a body to the physics world
inline void PhysicsWorld::addRigidBody(RigidBody* body) {
    std::vector<RigidBody*>::iterator it;

    assert(body);
    it = std::find(rigidBodies.begin(), rigidBodies.end(), body);
    assert(it == rigidBodies.end());
    
    // The body isn't already in the bodyList, therefore we add it to the list
    rigidBodies.push_back(body);
    addedBodies.push_back(body);
    it = std::find(removedBodies.begin(), removedBodies.end(), body);
    if (it != removedBodies.end()) {
        removedBodies.erase(it);
    }
}

// Remove a body from the physics world
inline void PhysicsWorld::removeRigidBody(RigidBody* body) {
    std::vector<RigidBody*>::iterator it;    
    
    assert(body);
    it = std::find(rigidBodies.begin(), rigidBodies.end(), body);
    assert(*it == body);
    rigidBodies.erase(it);
 
    it = std::find(addedBodies.begin(), addedBodies.end(), body);
    if (it != addedBodies.end()) {
        addedBodies.erase(it);
    }
    
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
inline Vector3 PhysicsWorld::getGravity() const {
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
inline std::vector<RigidBody*>::iterator PhysicsWorld::getRigidBodiesBeginIterator() {
    return rigidBodies.begin();
}

// Return an iterator to the end of the bodies of the physics world
inline std::vector<RigidBody*>::iterator PhysicsWorld::getRigidBodiesEndIterator() {
    return rigidBodies.end();
}

// Return the added bodies since last update of the physics engine
inline std::vector<RigidBody*>& PhysicsWorld::getAddedRigidBodies() {
    return addedBodies;
}

// Retrun the removed bodies since last update of the physics engine
inline std::vector<RigidBody*>& PhysicsWorld::getRemovedRigidBodies() {
    return removedBodies;
}

}   // End of the ReactPhysics3D namespace

 #endif