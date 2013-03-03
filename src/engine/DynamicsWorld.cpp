/********************************************************************************
* ReactPhysics3D physics library, http://code.google.com/p/reactphysics3d/      *
* Copyright (c) 2010-2013 Daniel Chappuis                                       *
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

// Namespaces
using namespace reactphysics3d;
using namespace std;

// Constructor
DynamicsWorld::DynamicsWorld(const Vector3 &gravity, decimal timeStep = DEFAULT_TIMESTEP)
              : CollisionWorld(), mTimer(timeStep), mGravity(gravity), mIsGravityOn(true),
                mContactSolver(*this, mConstrainedLinearVelocities, mConstrainedAngularVelocities,
                               mMapBodyToConstrainedVelocityIndex),
                mIsDeactivationActive(DEACTIVATION_ENABLED) {

}

// Destructor
DynamicsWorld::~DynamicsWorld() {

    // Delete the remaining overlapping pairs
    map<std::pair<bodyindex, bodyindex>, OverlappingPair*>::iterator it;
    for (it = mOverlappingPairs.begin(); it != mOverlappingPairs.end(); it++) {
        // Delete the overlapping pair
        (*it).second->OverlappingPair::~OverlappingPair();
        mMemoryPoolOverlappingPairs.freeObject((*it).second);
    }

    // Free the allocated memory for the constrained velocities
    cleanupConstrainedVelocitiesArray();
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

        // Initialize the constrained velocities
        initConstrainedVelocitiesArray();

        // If there are contacts
        if (!mContactManifolds.empty()) {

            // Solve the contacts
            mContactSolver.solve(mTimer.getTimeStep());
        }

        // Update the timer
        mTimer.nextStep();

        // Reset the movement boolean variable of each body to false
        resetBodiesMovementVariable();

        // Update the position and orientation of each body
        updateRigidBodiesPositionAndOrientation();

        // Cleanup of the contact solver
        mContactSolver.cleanup();

        // Cleanup the constrained velocities
        cleanupConstrainedVelocitiesArray();
    }

    // Compute and set the interpolation factor to all the bodies
    setInterpolationFactorToAllBodies();
}

// Update the position and orientation of the rigid bodies
void DynamicsWorld::updateRigidBodiesPositionAndOrientation() {
    decimal dt = mTimer.getTimeStep();
    
    // For each rigid body of the world
    set<RigidBody*>::iterator it;
    for (it = getRigidBodiesBeginIterator(); it != getRigidBodiesEndIterator(); ++it) {

        RigidBody* rigidBody = *it;
        assert(rigidBody != NULL);

        // If the body is allowed to move
        if (rigidBody->getIsMotionEnabled()) {

            // Update the old Transform of the body
            rigidBody->updateOldTransform();

            // Get the constrained velocity
            uint indexArray = mMapBodyToConstrainedVelocityIndex.find(rigidBody)->second;
            Vector3 newLinVelocity = mConstrainedLinearVelocities[indexArray];
            Vector3 newAngVelocity = mConstrainedAngularVelocities[indexArray];

            // Update the linear and angular velocity of the body
            rigidBody->setLinearVelocity(newLinVelocity);
            rigidBody->setAngularVelocity(newAngVelocity);

            // Add the split impulse velocity from Contact Solver (only used to update the position)
            if (mContactSolver.isConstrainedBody(rigidBody)) {
                newLinVelocity += mContactSolver.getSplitLinearVelocityOfBody(rigidBody);
                newAngVelocity += mContactSolver.getSplitAngularVelocityOfBody(rigidBody);
            }

            // Get current position and orientation of the body
            const Vector3& currentPosition = rigidBody->getTransform().getPosition();
            const Quaternion& currentOrientation = rigidBody->getTransform().getOrientation();

            // Compute the new position of the body
            Vector3 newPosition = currentPosition + newLinVelocity * dt;
            Quaternion newOrientation = currentOrientation + Quaternion(newAngVelocity.x,
                                                                        newAngVelocity.y,
                                                                        newAngVelocity.z, 0) *
                                                                    currentOrientation * 0.5 * dt;

            // Update the Transform of the body
            Transform newTransform(newPosition, newOrientation.getUnit());
            rigidBody->setTransform(newTransform);

            // Update the AABB of the rigid body
            rigidBody->updateAABB();
        }
    }
}

// Compute and set the interpolation factor to all bodies
void DynamicsWorld::setInterpolationFactorToAllBodies() {
    
    // Compute the interpolation factor
    decimal factor = mTimer.computeInterpolationFactor();
    assert(factor >= 0.0 && factor <= 1.0);

    // Set the factor to all bodies
    set<RigidBody*>::iterator it;
    for (it = getRigidBodiesBeginIterator(); it != getRigidBodiesEndIterator(); ++it) {

        RigidBody* rigidBody = dynamic_cast<RigidBody*>(*it);
        assert(rigidBody);

        rigidBody->setInterpolationFactor(factor);
    }
}

// Initialize the constrained velocities array at each step
void DynamicsWorld::initConstrainedVelocitiesArray() {

    // TODO : Use better memory allocation here
    mConstrainedLinearVelocities = std::vector<Vector3>(mRigidBodies.size(), Vector3(0, 0, 0));
    mConstrainedAngularVelocities = std::vector<Vector3>(mRigidBodies.size(), Vector3(0, 0, 0));

    double dt = mTimer.getTimeStep();

    // Fill in the mapping of rigid body to their index in the constrained
    // velocities arrays
    uint i = 0;
    for (std::set<RigidBody*>::iterator it = mRigidBodies.begin(); it != mRigidBodies.end(); ++it) {
        RigidBody* rigidBody = *it;
        mMapBodyToConstrainedVelocityIndex.insert(std::make_pair<RigidBody*, uint>(rigidBody, i));

        // Integrate the external force to get the new velocity of the body
        mConstrainedLinearVelocities[i] = rigidBody->getLinearVelocity() +
                   dt * rigidBody->getMassInverse() * rigidBody->getExternalForce();
        mConstrainedAngularVelocities[i] = rigidBody->getAngularVelocity() +
                   dt * rigidBody->getInertiaTensorInverseWorld() * rigidBody->getExternalTorque();

        i++;
    }
}

// Cleanup the constrained velocities array at each step
void DynamicsWorld::cleanupConstrainedVelocitiesArray() {

    // Clear the constrained velocites
    mConstrainedLinearVelocities.clear();
    mConstrainedAngularVelocities.clear();

    // Clear the rigid body to velocities array index mapping
    mMapBodyToConstrainedVelocityIndex.clear();
}

// Apply the gravity force to all bodies of the physics world
void DynamicsWorld::applyGravity() {

    // For each body of the physics world
    set<RigidBody*>::iterator it;
    for (it = getRigidBodiesBeginIterator(); it != getRigidBodiesEndIterator(); ++it) {

        RigidBody* rigidBody = dynamic_cast<RigidBody*>(*it);
        assert(rigidBody != NULL);
   
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
    RigidBody* rigidBody = new (mMemoryPoolRigidBodies.allocateObject()) RigidBody(transform, mass,
                                                                                inertiaTensorLocal,
                                                                                collisionShape,
                                                                                bodyID);
    assert(rigidBody != NULL);

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
    mBodies.erase(rigidBody);
    mRigidBodies.erase(rigidBody);

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
    OverlappingPair* newPair = new (mMemoryPoolOverlappingPairs.allocateObject()) OverlappingPair(
                                        addedPair->body1, addedPair->body2, mMemoryPoolContacts);
    assert(newPair != NULL);
    std::pair<map<bodyindexpair, OverlappingPair*>::iterator, bool> check =
            mOverlappingPairs.insert(make_pair(indexPair, newPair));
    assert(check.second);
}

// Notify the world about a removed broad-phase overlapping pair
void DynamicsWorld::notifyRemovedOverlappingPair(const BroadPhasePair* removedPair) {

    // Get the pair of body index
    std::pair<bodyindex, bodyindex> indexPair = removedPair->getBodiesIndexPair();

    // Remove the overlapping pair from the memory pool
    mOverlappingPairs.find(indexPair)->second->OverlappingPair::~OverlappingPair();
    mMemoryPoolOverlappingPairs.freeObject(mOverlappingPairs[indexPair]);
    mOverlappingPairs.erase(indexPair);
}

// Notify the world about a new narrow-phase contact
void DynamicsWorld::notifyNewContact(const BroadPhasePair* broadPhasePair,
                                     const ContactInfo* contactInfo) {

    RigidBody* const rigidBody1 = dynamic_cast<RigidBody* const>(broadPhasePair->body1);
    RigidBody* const rigidBody2 = dynamic_cast<RigidBody* const>(broadPhasePair->body2);

    assert(rigidBody1 != NULL);
    assert(rigidBody2 != NULL);

    // Create a new contact
    ContactPoint* contact = new (mMemoryPoolContacts.allocateObject()) ContactPoint(rigidBody1,
                                                                                    rigidBody2,
                                                                                    contactInfo);
    assert(contact != NULL);

    // Get the corresponding overlapping pair
    pair<bodyindex, bodyindex> indexPair = broadPhasePair->getBodiesIndexPair();
    OverlappingPair* overlappingPair = mOverlappingPairs.find(indexPair)->second;
    assert(overlappingPair != NULL);

    // Add the contact to the contact cache of the corresponding overlapping pair
    overlappingPair->addContact(contact);

    // Add the contact manifold to the world
    mContactManifolds.push_back(overlappingPair->getContactManifold());
}
