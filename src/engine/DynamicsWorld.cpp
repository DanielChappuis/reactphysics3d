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
#include "DynamicsWorld.h"

// We want to use the ReactPhysics3D namespace
using namespace reactphysics3d;
using namespace std;

// Constructor
DynamicsWorld::DynamicsWorld(const Vector3 &gravity, decimal timeStep = DEFAULT_TIMESTEP)
              : CollisionWorld(), mTimer(timeStep), mGravity(gravity), mIsGravityOn(true), mConstraintSolver(this),
                mIsDeactivationActive(DEACTIVATION_ENABLED) {

}

// Destructor
DynamicsWorld::~DynamicsWorld() {

    // Delete the remaining overlapping pairs
    for (map<std::pair<bodyindex, bodyindex>, OverlappingPair*>::iterator it=mOverlappingPairs.begin(); it != mOverlappingPairs.end(); it++) {
        // Delete the overlapping pair
        (*it).second->OverlappingPair::~OverlappingPair();
        mMemoryPoolOverlappingPairs.freeObject((*it).second);
    }
}

// Update the physics simulation
void DynamicsWorld::update() {

    assert(mTimer.getIsRunning());
    
    // Compute the time since the last update() call and update the timer
    mTimer.update();

    // Apply the gravity force to all bodies
    applyGravity();

    // While the time accumulator is not empty
    while(mTimer.isPossibleToTakeStep()) {

        // Remove all contact manifolds
        mContactManifolds.clear();
		
        // Compute the collision detection
        mCollisionDetection.computeCollisionDetection();

        // If there are constraints or contacts
        if (!mConstraints.empty() || !mContactManifolds.empty()) {

            // Solve the constraints and contacts
            mConstraintSolver.solve(mTimer.getTimeStep());
        }

        // Update the timer
        mTimer.nextStep();

        // Reset the movement boolean variable of each body to false
        resetBodiesMovementVariable();

        // Update the position and orientation of each body
        updateAllBodiesMotion();

        // Cleanup of the constraint solver
        mConstraintSolver.cleanup();
    }

    // Compute and set the interpolation factor to all the bodies
    setInterpolationFactorToAllBodies();
}

// Compute the motion of all bodies and update their positions and orientations
// First this method compute the vector V2 = V_constraint + V_forces + V1 where
// V_constraint = dt * (M^-1 * J^T * lambda) and V_forces = dt * (M^-1 * F_ext)
// V2 is the final velocity after the timestep and V1 is the velocity before the
// timestep.
// After having computed the velocity V2, this method will update the position
// and orientation of each body.
// This method uses the semi-implicit Euler method to update the position and
// orientation of the body
void DynamicsWorld::updateAllBodiesMotion() {
    decimal dt = mTimer.getTimeStep();
    Vector3 newLinearVelocity;
    Vector3 newAngularVelocity;
    
    // For each body of thephysics world
    for (set<RigidBody*>::iterator it=getRigidBodiesBeginIterator(); it != getRigidBodiesEndIterator(); ++it) {

        RigidBody* rigidBody = *it;
        assert(rigidBody);

        // If the body is able to move
        if (rigidBody->getIsMotionEnabled()) {
            newLinearVelocity.setAllValues(0.0, 0.0, 0.0);
            newAngularVelocity.setAllValues(0.0, 0.0, 0.0);

            // If it's a constrained body
            if (mConstraintSolver.isConstrainedBody(*it)) {
                // Get the constrained linear and angular velocities from the constraint solver
                newLinearVelocity = mConstraintSolver.getConstrainedLinearVelocityOfBody(rigidBody);
                newAngularVelocity = mConstraintSolver.getConstrainedAngularVelocityOfBody(rigidBody);
            }
            else {
                // Compute V_forces = dt * (M^-1 * F_ext) which is the velocity of the body due to the
                // external forces and torques.
                newLinearVelocity += dt * rigidBody->getMassInverse() * rigidBody->getExternalForce();
                newAngularVelocity += dt * rigidBody->getInertiaTensorInverseWorld() * rigidBody->getExternalTorque();

                // Add the velocity V1 to the new velocity
                newLinearVelocity += rigidBody->getLinearVelocity();
                newAngularVelocity += rigidBody->getAngularVelocity();
            }
            
            // Update the position and the orientation of the body according to the new velocity
            updatePositionAndOrientationOfBody(*it, newLinearVelocity, newAngularVelocity);

            // Update the AABB of the rigid body
            rigidBody->updateAABB();
        }
    }
}

// Update the position and orientation of a body
// Use the Semi-Implicit Euler (Sympletic Euler) method to compute the new position and the new
// orientation of the body
void DynamicsWorld::updatePositionAndOrientationOfBody(RigidBody* rigidBody,
                                                       Vector3 newLinVelocity,
                                                       Vector3 newAngVelocity) {
    decimal dt = mTimer.getTimeStep();

    assert(rigidBody);

    // Update the old position and orientation of the body
    rigidBody->updateOldTransform();

    // Update the linear and angular velocity of the body
    rigidBody->setLinearVelocity(newLinVelocity);
    rigidBody->setAngularVelocity(newAngVelocity);

    // Split velocity (only used to update the position)
    if (mConstraintSolver.isConstrainedBody(rigidBody)) {
        newLinVelocity += mConstraintSolver.getSplitLinearVelocityOfBody(rigidBody);
        newAngVelocity += mConstraintSolver.getSplitAngularVelocityOfBody(rigidBody);
    }
    
    // Get current position and orientation of the body
    const Vector3& currentPosition = rigidBody->getTransform().getPosition();
    const Quaternion& currentOrientation = rigidBody->getTransform().getOrientation();
            
    Vector3 newPosition = currentPosition + newLinVelocity * dt;
    Quaternion newOrientation = currentOrientation + Quaternion(newAngVelocity.getX(), newAngVelocity.getY(), newAngVelocity.getZ(), 0) * currentOrientation * 0.5 * dt;

    Transform newTransform(newPosition, newOrientation.getUnit());
    rigidBody->setTransform(newTransform);
}

// Compute and set the interpolation factor to all bodies
void DynamicsWorld::setInterpolationFactorToAllBodies() {
    
    // Compute the interpolation factor
    decimal factor = mTimer.computeInterpolationFactor();
    assert(factor >= 0.0 && factor <= 1.0);

    // Set the factor to all bodies
    for (set<RigidBody*>::iterator it=getRigidBodiesBeginIterator(); it != getRigidBodiesEndIterator(); ++it) {

        RigidBody* rigidBody = dynamic_cast<RigidBody*>(*it);
        assert(rigidBody);

        rigidBody->setInterpolationFactor(factor);
    }
}

// Apply the gravity force to all bodies of the physics world
void DynamicsWorld::applyGravity() {

    // For each body of the physics world
    for (set<RigidBody*>::iterator it=getRigidBodiesBeginIterator(); it != getRigidBodiesEndIterator(); ++it) {

        RigidBody* rigidBody = dynamic_cast<RigidBody*>(*it);
        assert(rigidBody);
   
        // If the gravity force is on
        if(mIsGravityOn) {
            // Apply the current gravity force to the body
            rigidBody->setExternalForce(rigidBody->getMass() * mGravity);
        }
    }
}

// Create a rigid body into the physics world
RigidBody* DynamicsWorld::createRigidBody(const Transform& transform, decimal mass,
                                          const Matrix3x3& inertiaTensorLocal,
                                          CollisionShape* collisionShape) {

    // Compute the body ID
    bodyindex bodyID = computeNextAvailableBodyID();

    // Largest index cannot be used (it is used for invalid index)
    assert(bodyID < std::numeric_limits<reactphysics3d::bodyindex>::max());

    // Create the rigid body
    RigidBody* rigidBody = new (mMemoryPoolRigidBodies.allocateObject()) RigidBody(transform, mass, inertiaTensorLocal, collisionShape, bodyID);

    // Add the rigid body to the physics world
    mBodies.insert(rigidBody);
    mRigidBodies.insert(rigidBody);

    // Add the rigid body to the collision detection
    mCollisionDetection.addBody(rigidBody);

    // Return the pointer to the rigid body
    return rigidBody;
}

// Destroy a rigid body
void DynamicsWorld::destroyRigidBody(RigidBody* rigidBody) {

    // Remove the body from the collision detection
    mCollisionDetection.removeBody(rigidBody);

    // Add the body ID to the list of free IDs
    mFreeBodiesIDs.push_back(rigidBody->getID());

    // Call the constructor of the rigid body
    rigidBody->RigidBody::~RigidBody();

    // Remove the rigid body from the list of rigid bodies
    mBodies.erase(rigidBody);                                    // TOOD : Maybe use a set to make this faster
    mRigidBodies.erase(rigidBody);                               // TOOD : Maybe use a set to make this faster

    // Free the object from the memory pool
    mMemoryPoolRigidBodies.freeObject(rigidBody);
}

// Remove all constraints in the physics world
void DynamicsWorld::removeAllConstraints() {
    mConstraints.clear();
}

// Notify the world about a new broad-phase overlapping pair
void DynamicsWorld::notifyAddedOverlappingPair(const BroadPhasePair* addedPair) {

    // Get the pair of body index
    std::pair<bodyindex, bodyindex> indexPair = addedPair->getBodiesIndexPair();

    // Add the pair into the set of overlapping pairs (if not there yet)
    OverlappingPair* newPair = new (mMemoryPoolOverlappingPairs.allocateObject()) OverlappingPair(addedPair->body1, addedPair->body2, mMemoryPoolContacts);
    std::pair<map<std::pair<bodyindex, bodyindex>, OverlappingPair*>::iterator, bool> check = mOverlappingPairs.insert(make_pair(indexPair, newPair));
    assert(check.second);
}

// Notify the world about a removed broad-phase overlapping pair
void DynamicsWorld::notifyRemovedOverlappingPair(const BroadPhasePair* removedPair) {

    // Get the pair of body index
    std::pair<bodyindex, bodyindex> indexPair = removedPair->getBodiesIndexPair();

    // Remove the overlapping pair from the memory pool
    mOverlappingPairs[indexPair]->OverlappingPair::~OverlappingPair();
    mMemoryPoolOverlappingPairs.freeObject(mOverlappingPairs[indexPair]);
    mOverlappingPairs.erase(indexPair);
}

// Notify the world about a new narrow-phase contact
void DynamicsWorld::notifyNewContact(const BroadPhasePair* broadPhasePair, const ContactInfo* contactInfo) {

    RigidBody* const rigidBody1 = dynamic_cast<RigidBody* const>(broadPhasePair->body1);
    RigidBody* const rigidBody2 = dynamic_cast<RigidBody* const>(broadPhasePair->body2);

    assert(rigidBody1);
    assert(rigidBody2);

    // Create a new contact
    Contact* contact = new (mMemoryPoolContacts.allocateObject()) Contact(rigidBody1, rigidBody2, contactInfo);
    assert(contact);

    // Get the corresponding overlapping pair
    pair<bodyindex, bodyindex> indexPair = broadPhasePair->getBodiesIndexPair();
    OverlappingPair* overlappingPair = mOverlappingPairs[indexPair];
    assert(overlappingPair);

    // Add the contact to the contact cache of the corresponding overlapping pair
    overlappingPair->addContact(contact);

    // Create a contact manifold with the contact points of the two bodies
    ContactManifold contactManifold;
    contactManifold.nbContacts = 0;

    // Add all the contacts in the contact cache of the two bodies
    // to the set of constraints in the physics world
    for (uint i=0; i<overlappingPair->getNbContacts(); i++) {
        contactManifold.contacts[i] = overlappingPair->getContact(i);
        contactManifold.nbContacts++;
    }

    // Add the contact manifold to the world
    mContactManifolds.push_back(contactManifold);
}
