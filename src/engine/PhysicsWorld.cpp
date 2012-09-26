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
             : gravity(gravity), isGravityOn(true), currentBodyID(0) {
}

// Destructor
PhysicsWorld::~PhysicsWorld() {

    // Delete the remaining overlapping pairs
    for (map<std::pair<bodyindex, bodyindex>, OverlappingPair*>::iterator it=overlappingPairs.begin(); it != overlappingPairs.end(); it++) {
        // Delete the overlapping pair
        (*it).second->OverlappingPair::~OverlappingPair();
        memoryPoolOverlappingPairs.freeObject((*it).second);
    }
}

// Create a rigid body into the physics world
RigidBody* PhysicsWorld::createRigidBody(const Transform& transform, decimal mass, const Matrix3x3& inertiaTensorLocal, CollisionShape* collisionShape) {
    
    // Compute the body ID
    bodyindex bodyID;
    if (!freeRigidBodyIDs.empty()) {
        bodyID = freeRigidBodyIDs.back();
        freeRigidBodyIDs.pop_back();
    }
    else {
        bodyID = currentBodyID;
        currentBodyID++;
    }

    // Largest index cannot be used (it is used for invalid index)
    assert(bodyID < std::numeric_limits<reactphysics3d::bodyindex>::max());

    // Create the rigid body
    RigidBody* rigidBody = new (memoryPoolRigidBodies.allocateObject()) RigidBody(transform, mass, inertiaTensorLocal, collisionShape, bodyID);
    
    // Add the rigid body to the physics world
    bodies.insert(rigidBody);
    rigidBodies.insert(rigidBody);

    // Add the rigid body to the collision detection
    collisionDetection->addBody(rigidBody);

    // Return the pointer to the rigid body
    return rigidBody;
}       

// Destroy a rigid body
void PhysicsWorld::destroyRigidBody(RigidBody* rigidBody) {

    // Remove the body from the collision detection
    collisionDetection->removeBody(rigidBody);

    // Add the body ID to the list of free IDs
    freeRigidBodyIDs.push_back(rigidBody->getID());
	
    // Call the constructor of the rigid body
    rigidBody->RigidBody::~RigidBody();

    // Remove the rigid body from the list of rigid bodies
    bodies.erase(rigidBody);                                    // TOOD : Maybe use a set to make this faster
    rigidBodies.erase(rigidBody);                               // TOOD : Maybe use a set to make this faster

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

// Notify the world about a new broad-phase overlapping pair
void PhysicsWorld::notifyAddedOverlappingPair(const BroadPhasePair* addedPair) {

    // Get the pair of body index
    std::pair<bodyindex, bodyindex> indexPair = addedPair->getBodiesIndexPair();

    // Add the pair into the set of overlapping pairs (if not there yet)
    OverlappingPair* newPair = new (memoryPoolOverlappingPairs.allocateObject()) OverlappingPair(addedPair->body1, addedPair->body2, memoryPoolContacts);
    std::pair<map<std::pair<bodyindex, bodyindex>, OverlappingPair*>::iterator, bool> check = overlappingPairs.insert(make_pair(indexPair, newPair));
    assert(check.second);
}

// Notify the world about a removed broad-phase overlapping pair
void PhysicsWorld::notifyRemovedOverlappingPair(const BroadPhasePair* removedPair) {

    // Get the pair of body index
    std::pair<bodyindex, bodyindex> indexPair = removedPair->getBodiesIndexPair();

    // Remove the overlapping pair from the memory pool
    overlappingPairs[indexPair]->OverlappingPair::~OverlappingPair();
    memoryPoolOverlappingPairs.freeObject(overlappingPairs[indexPair]);
    overlappingPairs.erase(indexPair);
}

// Notify the world about a new narrow-phase contact
void PhysicsWorld::notifyNewContact(const BroadPhasePair* broadPhasePair, const ContactInfo* contactInfo) {

    RigidBody* const rigidBody1 = dynamic_cast<RigidBody* const>(broadPhasePair->body1);
    RigidBody* const rigidBody2 = dynamic_cast<RigidBody* const>(broadPhasePair->body2);

    assert(rigidBody1);
    assert(rigidBody2);

    // Create a new contact
    Contact* contact = new (memoryPoolContacts.allocateObject()) Contact(rigidBody1, rigidBody2, contactInfo);
    assert(contact);

    // Get the corresponding overlapping pair
    pair<bodyindex, bodyindex> indexPair = broadPhasePair->getBodiesIndexPair();
    OverlappingPair* overlappingPair = overlappingPairs[indexPair];
    assert(overlappingPair);

    // Add the contact to the contact cache of the corresponding overlapping pair
    overlappingPair->addContact(contact);

    // Add all the contacts in the contact cache of the two bodies
    // to the set of constraints in the physics world
    for (uint i=0; i<overlappingPair->getNbContacts(); i++) {
        addConstraint(overlappingPair->getContact(i));
    }
}


