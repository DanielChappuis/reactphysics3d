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
#include <set>
#include <algorithm>
#include "../mathematics/mathematics.h"
#include "../body/CollisionBody.h"
#include "../collision/CollisionDetection.h"
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
        CollisionDetection* collisionDetection;         // Reference to the collision detection
        std::set<CollisionBody*> bodies;                // All the bodies (rigid and soft) of the physics world
        std::set<RigidBody*> rigidBodies;               // All the rigid bodies of the physics world
        std::vector<luint> freeRigidBodyIDs;            // List of free ID for rigid bodies
        std::vector<Constraint*> constraints;           // List that contains all the current constraints
        Vector3 gravity;                                // Gravity vector of the world
        bool isGravityOn;                               // True if the gravity force is on
        bodyindex currentBodyID;                        // Current body ID
        MemoryPool<RigidBody> memoryPoolRigidBodies;    // Memory pool for rigid bodies memory allocation
        
    public :
        PhysicsWorld(const Vector3& gravity);      // Constructor
        virtual ~PhysicsWorld();                   // Destructor

        RigidBody* createRigidBody(const Transform& transform, decimal mass,
                                   const Matrix3x3& inertiaTensorLocal, CollisionShape* collisionShape);  // Create a rigid body into the physics world
        void destroyRigidBody(RigidBody* rigidBody);                                    // Destroy a rigid body
        Vector3 getGravity() const;                                                     // Return the gravity vector of the world
        bool getIsGravityOn() const;                                                    // Return if the gravity is on
        void setIsGratityOn(bool isGravityOn);                                          // Set the isGravityOn attribute
        void setCollisionDetection(CollisionDetection* collisionDetection);             // Set the collision detection reference
        void addConstraint(Constraint* constraint);                                     // Add a constraint
        void removeConstraint(Constraint* constraint);                                  // Remove a constraint
        void removeAllContactConstraints();                                             // Remove all collision contacts constraints
        void removeAllConstraints();                                                    // Remove all constraints and delete them (free their memory)
        std::vector<Constraint*>::iterator getConstraintsBeginIterator();               // Return a start iterator on the constraint list
        std::vector<Constraint*>::iterator getConstraintsEndIterator();                 // Return a end iterator on the constraint list
        std::set<CollisionBody*>::iterator getBodiesBeginIterator();                             // Return an iterator to the beginning of the bodies of the physics world
        std::set<CollisionBody*>::iterator getBodiesEndIterator();                               // Return an iterator to the end of the bodies of the physics world
        std::set<RigidBody*>::iterator getRigidBodiesBeginIterator();                   // Return an iterator to the beginning of the rigid bodies of the physics world
        std::set<RigidBody*>::iterator getRigidBodiesEndIterator();                     // Return an iterator to the end of the rigid bodies of the physics world
};

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

// Set the collision detection reference
inline void PhysicsWorld::setCollisionDetection(CollisionDetection* collisionDetection) {
    this->collisionDetection = collisionDetection;
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
inline std::set<CollisionBody*>::iterator PhysicsWorld::getBodiesBeginIterator() {
    return bodies.begin();
}

// Return an iterator to the end of the bodies of the physics world
inline std::set<CollisionBody*>::iterator PhysicsWorld::getBodiesEndIterator() {
    return bodies.end();
}

// Return an iterator to the beginning of the bodies of the physics world
inline std::set<RigidBody*>::iterator PhysicsWorld::getRigidBodiesBeginIterator() {
    return rigidBodies.begin();
}

// Return an iterator to the end of the bodies of the physics world
inline std::set<RigidBody*>::iterator PhysicsWorld::getRigidBodiesEndIterator() {
    return rigidBodies.end();
}

}   // End of the ReactPhysics3D namespace

 #endif
