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

// Libraries
#include "PhysicsWorld.h"
#include "PhysicsEngine.h"
#include <algorithm>

// Namespaces
using namespace reactphysics3d;
using namespace std;

// Constructor
PhysicsWorld::PhysicsWorld(const Vector3& gravity)
             : gravity(gravity), isGravityOn(true), currentBodyID(0), memoryPoolRigidBodies(NB_MAX_BODIES) {

}

// Destructor
PhysicsWorld::~PhysicsWorld() {

}

// Create a rigid body into the physics world
RigidBody* PhysicsWorld::createRigidBody(const Transform& transform, double mass, const Matrix3x3& inertiaTensorLocal, Shape* shape) {
    
    // Create the rigid body
    RigidBody* rigidBody = new (memoryPoolRigidBodies.allocateObject()) RigidBody(transform, mass, inertiaTensorLocal, shape, currentBodyID);
    
    currentBodyID++;
    
    // Add the rigid body to the physics world and return it
    addRigidBody(rigidBody);
    return rigidBody;
}       

// Destroy a rigid body
void PhysicsWorld::destroyRigidBody(RigidBody* rigidBody) {
    removeRigidBody(rigidBody);
	
	// Call the constructor of the rigid body
	rigidBody->RigidBody::~RigidBody();
	
	// Free the object from the memory pool
	memoryPoolRigidBodies.freeObject(rigidBody);
}  

// Remove all collision contacts constraints
// TODO : This method should be in the collision detection class
void PhysicsWorld::removeAllContactConstraints() {
    // For all constraints
    for (vector<Constraint*>::iterator it = constraints.begin(); it != constraints.end(); ) {

        // Try a downcasting
        Contact* contact = dynamic_cast<Contact*>(*it);

        // If the constraint is a contact
        if (contact) {
            // Remove it from the constraints of the physics world
            it = constraints.erase(it);
        }
        else {
            ++it;
        }
    }
}

// Remove all constraints in the physics world
void PhysicsWorld::removeAllConstraints() {
    constraints.clear();
}

