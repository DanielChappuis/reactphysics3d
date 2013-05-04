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
#include "constraint/BallAndSocketJoint.h"

// Namespaces
using namespace reactphysics3d;
using namespace std;

// Constructor
DynamicsWorld::DynamicsWorld(const Vector3 &gravity, decimal timeStep = DEFAULT_TIMESTEP)
              : CollisionWorld(), mTimer(timeStep), mGravity(gravity), mIsGravityOn(true),
                mContactSolver(mContactManifolds, mConstrainedLinearVelocities, mConstrainedAngularVelocities,
                               mMapBodyToConstrainedVelocityIndex),
                mConstraintSolver(mJoints, mConstrainedLinearVelocities, mConstrainedAngularVelocities,
                                  mMapBodyToConstrainedVelocityIndex),
                mNbSolverIterations(DEFAULT_CONSTRAINTS_SOLVER_NB_ITERATIONS),
                mIsDeactivationActive(DEACTIVATION_ENABLED) {

}

// Destructor
DynamicsWorld::~DynamicsWorld() {

    // Delete the remaining overlapping pairs
    map<std::pair<bodyindex, bodyindex>, OverlappingPair*>::iterator it;
    for (it = mOverlappingPairs.begin(); it != mOverlappingPairs.end(); it++) {
        // Delete the overlapping pair
        (*it).second->OverlappingPair::~OverlappingPair();
        mMemoryAllocator.release((*it).second, sizeof(OverlappingPair));
    }

    // Free the allocated memory for the constrained velocities
    cleanupConstrainedVelocitiesArray();

#ifdef IS_PROFILING_ACTIVE

    // Print the profiling report
    Profiler::printReport(std::cout);

    // Destroy the profiler (release the allocated memory)
    Profiler::destroy();
#endif

}

// Update the physics simulation
void DynamicsWorld::update() {

#ifdef IS_PROFILING_ACTIVE
    // Increment the frame counter of the profiler
    Profiler::incrementFrameCounter();
#endif

    PROFILE("DynamicsWorld::update()");

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

        // Integrate the velocities
        integrateRigidBodiesVelocities();

        // Solve the contacts and constraints
        solveContactsAndConstraints();

        // Update the timer
        mTimer.nextStep();

        // Reset the movement boolean variable of each body to false
        resetBodiesMovementVariable();

        // Integrate the position and orientation of each body
        integrateRigidBodiesPositions();

        // Cleanup of the contact solver
        mContactSolver.cleanup();

        // Cleanup the constrained velocities
        cleanupConstrainedVelocitiesArray();
    }

    // Compute and set the interpolation factor to all the bodies
    setInterpolationFactorToAllBodies();
}

// Integrate position and orientation of the rigid bodies.
/// The positions and orientations of the bodies are integrated using
/// the sympletic Euler time stepping scheme.
void DynamicsWorld::integrateRigidBodiesPositions() {

    PROFILE("DynamicsWorld::updateRigidBodiesPositionAndOrientation()");

    decimal dt = static_cast<decimal>(mTimer.getTimeStep());
    
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

    PROFILE("DynamicsWorld::setInterpolationFactorToAllBodies()");
    
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

// Integrate the velocities of rigid bodies.
/// This method only set the temporary velocities but does not update
/// the actual velocitiy of the bodies. The velocities updated in this method
/// might violate the constraints and will be corrected in the constraint and
/// contact solver.
void DynamicsWorld::integrateRigidBodiesVelocities() {

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

        // If the body is allowed to move
        if (rigidBody->getIsMotionEnabled()) {

            // Integrate the external force to get the new velocity of the body
            mConstrainedLinearVelocities[i] = rigidBody->getLinearVelocity() +
                    dt * rigidBody->getMassInverse() * rigidBody->getExternalForce();
            mConstrainedAngularVelocities[i] = rigidBody->getAngularVelocity() +
                    dt * rigidBody->getInertiaTensorInverseWorld() * rigidBody->getExternalTorque();
        }

        i++;
    }

    assert(mMapBodyToConstrainedVelocityIndex.size() == mRigidBodies.size());
}

// Solve the contacts and constraints
void DynamicsWorld::solveContactsAndConstraints() {

    PROFILE("DynamicsWorld::solveContactsAndConstraints()");

    // Get the current time step
    decimal dt = static_cast<decimal>(mTimer.getTimeStep());

    // Check if there are contacts and constraints to solve
    bool isConstraintsToSolve = !mJoints.empty();
    bool isContactsToSolve = !mContactManifolds.empty();
    if (!isConstraintsToSolve && !isContactsToSolve) return;

    // If there are contacts
    if (isContactsToSolve) {

        // Initialize the solver
        mContactSolver.initialize(dt);

        // Warm start the contact solver
        mContactSolver.warmStart();
    }

    // If there are constraints
    if (isConstraintsToSolve) {

        // Initialize the constraint solver
        mConstraintSolver.initialize(dt);
    }

    // For each iteration of the solver
    for (uint i=0; i<mNbSolverIterations; i++) {

        // Solve the constraints
        if (isConstraintsToSolve) mConstraintSolver.solve();

        // Solve the contacts
        if (isContactsToSolve) mContactSolver.solve();
    }

    // Cache the lambda values in order to use them in the next step
    if (isContactsToSolve) mContactSolver.storeImpulses();
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

    PROFILE("DynamicsWorld::applyGravity()");

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
                                          const CollisionShape& collisionShape) {

    // Compute the body ID
    bodyindex bodyID = computeNextAvailableBodyID();

    // Largest index cannot be used (it is used for invalid index)
    assert(bodyID < std::numeric_limits<reactphysics3d::bodyindex>::max());

    // Create a collision shape for the rigid body into the world
    CollisionShape* newCollisionShape = createCollisionShape(collisionShape);

    // Create the rigid body
    RigidBody* rigidBody = new (mMemoryAllocator.allocate(sizeof(RigidBody))) RigidBody(transform,
                                                                                mass,
                                                                                inertiaTensorLocal,
                                                                                newCollisionShape,
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

    // Remove the collision shape from the world
    removeCollisionShape(rigidBody->getCollisionShape());

    // Call the destructor of the rigid body
    rigidBody->RigidBody::~RigidBody();

    // Remove the rigid body from the list of rigid bodies
    mBodies.erase(rigidBody);
    mRigidBodies.erase(rigidBody);

    // Free the object from the memory allocator
    mMemoryAllocator.release(rigidBody, sizeof(RigidBody));
}

// Create a joint between two bodies in the world and return a pointer to the new joint
Constraint* DynamicsWorld::createJoint(const ConstraintInfo& jointInfo) {

    Constraint* newJoint = NULL;

    // Allocate memory to create the new joint
    switch(jointInfo.type) {

        // Ball-and-Socket joint
        case BALLSOCKETJOINT:
        {
            void* allocatedMemory = mMemoryAllocator.allocate(sizeof(BallAndSocketJoint));
            const BallAndSocketJointInfo& info = dynamic_cast<const BallAndSocketJointInfo&>(
                                                                                        jointInfo);
            newJoint = new (allocatedMemory) BallAndSocketJoint(info);
            break;
        }

        default:
        {
            assert(false);
            return NULL;
        }
    }

    // Add the joint into the world
    mJoints.insert(newJoint);

    // Return the pointer to the created joint
    return newJoint;
}

// Destroy a joint
void DynamicsWorld::destroyJoint(Constraint* joint) {

    assert(joint != NULL);

    // Remove the joint from the world
    mJoints.erase(joint);

    // Get the size in bytes of the joint
    size_t nbBytes = joint->getSizeInBytes();

    // Call the destructor of the joint
    joint->Constraint::~Constraint();

    // Release the allocated memory
    mMemoryAllocator.release(joint, nbBytes);
}

// Notify the world about a new broad-phase overlapping pair
void DynamicsWorld::notifyAddedOverlappingPair(const BroadPhasePair* addedPair) {

    // Get the pair of body index
    bodyindexpair indexPair = addedPair->getBodiesIndexPair();

    // Add the pair into the set of overlapping pairs (if not there yet)
    OverlappingPair* newPair = new (mMemoryAllocator.allocate(sizeof(OverlappingPair))) OverlappingPair(
                                        addedPair->body1, addedPair->body2, mMemoryAllocator);
    assert(newPair != NULL);
    std::pair<map<bodyindexpair, OverlappingPair*>::iterator, bool> check =
            mOverlappingPairs.insert(make_pair(indexPair, newPair));
    assert(check.second);
}

// Notify the world about a removed broad-phase overlapping pair
void DynamicsWorld::notifyRemovedOverlappingPair(const BroadPhasePair* removedPair) {

    // Get the pair of body index
    std::pair<bodyindex, bodyindex> indexPair = removedPair->getBodiesIndexPair();

    // Remove the overlapping pair from the memory allocator
    mOverlappingPairs.find(indexPair)->second->OverlappingPair::~OverlappingPair();
    mMemoryAllocator.release(mOverlappingPairs[indexPair], sizeof(OverlappingPair));
    mOverlappingPairs.erase(indexPair);
}

// Notify the world about a new narrow-phase contact
void DynamicsWorld::notifyNewContact(const BroadPhasePair* broadPhasePair,
                                     const ContactPointInfo* contactInfo) {

    // Create a new contact
    ContactPoint* contact = new (mMemoryAllocator.allocate(sizeof(ContactPoint))) ContactPoint(
                                                                                    *contactInfo);
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
