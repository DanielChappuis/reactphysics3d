/********************************************************************************
* ReactPhysics3D physics library, http://www.reactphysics3d.com                 *
* Copyright (c) 2010-2016 Daniel Chappuis                                       *
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
#include "constraint/SliderJoint.h"
#include "constraint/HingeJoint.h"
#include "constraint/FixedJoint.h"

// Namespaces
using namespace reactphysics3d;
using namespace std;

// Constructor
/**
 * @param gravity Gravity vector in the world (in meters per second squared)
 */
DynamicsWorld::DynamicsWorld(const Vector3 &gravity)
              : CollisionWorld(),
                mContactSolver(mMapBodyToConstrainedVelocityIndex),
                mConstraintSolver(mMapBodyToConstrainedVelocityIndex),
                mNbVelocitySolverIterations(DEFAULT_VELOCITY_SOLVER_NB_ITERATIONS),
                mNbPositionSolverIterations(DEFAULT_POSITION_SOLVER_NB_ITERATIONS),
                mIsSleepingEnabled(SPLEEPING_ENABLED), mGravity(gravity),
                mIsGravityEnabled(true), mConstrainedLinearVelocities(NULL),
                mConstrainedAngularVelocities(NULL), mSplitLinearVelocities(NULL),
                mSplitAngularVelocities(NULL), mConstrainedPositions(NULL),
                mConstrainedOrientations(NULL), mNbIslands(0),
                mNbIslandsCapacity(0), mIslands(NULL), mNbBodiesCapacity(0),
                mSleepLinearVelocity(DEFAULT_SLEEP_LINEAR_VELOCITY),
                mSleepAngularVelocity(DEFAULT_SLEEP_ANGULAR_VELOCITY),
                mTimeBeforeSleep(DEFAULT_TIME_BEFORE_SLEEP) {

}

// Destructor
DynamicsWorld::~DynamicsWorld() {

    // Destroy all the joints that have not been removed
    std::set<Joint*>::iterator itJoints;
    for (itJoints = mJoints.begin(); itJoints != mJoints.end();) {
        std::set<Joint*>::iterator itToRemove = itJoints;
        ++itJoints;
        destroyJoint(*itToRemove);
    }

    // Destroy all the rigid bodies that have not been removed
    std::set<RigidBody*>::iterator itRigidBodies;
    for (itRigidBodies = mRigidBodies.begin(); itRigidBodies != mRigidBodies.end();) {
        std::set<RigidBody*>::iterator itToRemove = itRigidBodies;
        ++itRigidBodies;
        destroyRigidBody(*itToRemove);
    }

    // Release the memory allocated for the islands
    for (uint i=0; i<mNbIslands; i++) {

        // Call the island destructor
        mIslands[i]->~Island();

        // Release the allocated memory for the island
        mMemoryAllocator.release(mIslands[i], sizeof(Island));
    }
    if (mNbIslandsCapacity > 0) {
        mMemoryAllocator.release(mIslands, sizeof(Island*) * mNbIslandsCapacity);
    }

    // Release the memory allocated for the bodies velocity arrays
    if (mNbBodiesCapacity > 0) {
        delete[] mSplitLinearVelocities;
        delete[] mSplitAngularVelocities;
        delete[] mConstrainedLinearVelocities;
        delete[] mConstrainedAngularVelocities;
        delete[] mConstrainedPositions;
        delete[] mConstrainedOrientations;
    }

    assert(mJoints.size() == 0);
    assert(mRigidBodies.size() == 0);

#ifdef IS_PROFILING_ACTIVE

    // Print the profiling report
    Profiler::printReport(std::cout);

    // Destroy the profiler (release the allocated memory)
    Profiler::destroy();
#endif

}

// Update the physics simulation
/**
 * @param timeStep The amount of time to step the simulation by (in seconds)
 */
void DynamicsWorld::update(decimal timeStep) {

#ifdef IS_PROFILING_ACTIVE
    // Increment the frame counter of the profiler
    Profiler::incrementFrameCounter();
#endif

    PROFILE("DynamicsWorld::update()");

    mTimeStep = timeStep;

    // Notify the event listener about the beginning of an internal tick
    if (mEventListener != NULL) mEventListener->beginInternalTick();

    // Reset all the contact manifolds lists of each body
    resetContactManifoldListsOfBodies();

    // Compute the collision detection
    mCollisionDetection.computeCollisionDetection();

    // Compute the islands (separate groups of bodies with constraints between each others)
    computeIslands();

    // Integrate the velocities
    integrateRigidBodiesVelocities();

    // Solve the contacts and constraints
    solveContactsAndConstraints();

    // Integrate the position and orientation of each body
    integrateRigidBodiesPositions();

    // Solve the position correction for constraints
    solvePositionCorrection();

    // Update the state (positions and velocities) of the bodies
    updateBodiesState();

    if (mIsSleepingEnabled) updateSleepingBodies();

    // Notify the event listener about the end of an internal tick
    if (mEventListener != NULL) mEventListener->endInternalTick();

    // Reset the external force and torque applied to the bodies
    resetBodiesForceAndTorque();
}

// Integrate position and orientation of the rigid bodies.
/// The positions and orientations of the bodies are integrated using
/// the sympletic Euler time stepping scheme.
void DynamicsWorld::integrateRigidBodiesPositions() {

    PROFILE("DynamicsWorld::integrateRigidBodiesPositions()");
    
    // For each island of the world
    for (uint i=0; i < mNbIslands; i++) {

        RigidBody** bodies = mIslands[i]->getBodies();

        // For each body of the island
        for (uint b=0; b < mIslands[i]->getNbBodies(); b++) {

            // Get the constrained velocity
            uint indexArray = mMapBodyToConstrainedVelocityIndex.find(bodies[b])->second;
            Vector3 newLinVelocity = mConstrainedLinearVelocities[indexArray];
            Vector3 newAngVelocity = mConstrainedAngularVelocities[indexArray];

            // Add the split impulse velocity from Contact Solver (only used
            // to update the position)
            if (mContactSolver.isSplitImpulseActive()) {

                newLinVelocity += mSplitLinearVelocities[indexArray];
                newAngVelocity += mSplitAngularVelocities[indexArray];
            }

            // Get current position and orientation of the body
            const Vector3& currentPosition = bodies[b]->mCenterOfMassWorld;
            const Quaternion& currentOrientation = bodies[b]->getTransform().getOrientation();

            // Update the new constrained position and orientation of the body
            mConstrainedPositions[indexArray] = currentPosition + newLinVelocity * mTimeStep;
            mConstrainedOrientations[indexArray] = currentOrientation +
                                                   Quaternion(0, newAngVelocity) *
                                                   currentOrientation * decimal(0.5) * mTimeStep;
        }
    }
}

// Update the postion/orientation of the bodies
void DynamicsWorld::updateBodiesState() {

    PROFILE("DynamicsWorld::updateBodiesState()");

    // For each island of the world
    for (uint islandIndex = 0; islandIndex < mNbIslands; islandIndex++) {

        // For each body of the island
        RigidBody** bodies = mIslands[islandIndex]->getBodies();

        for (uint b=0; b < mIslands[islandIndex]->getNbBodies(); b++) {

            uint index = mMapBodyToConstrainedVelocityIndex.find(bodies[b])->second;

            // Update the linear and angular velocity of the body
            bodies[b]->mLinearVelocity = mConstrainedLinearVelocities[index];
            bodies[b]->mAngularVelocity = mConstrainedAngularVelocities[index];

            // Update the position of the center of mass of the body
            bodies[b]->mCenterOfMassWorld = mConstrainedPositions[index];

            // Update the orientation of the body
            bodies[b]->mTransform.setOrientation(mConstrainedOrientations[index].getUnit());

            // Update the transform of the body (using the new center of mass and new orientation)
            bodies[b]->updateTransformWithCenterOfMass();

            // Update the broad-phase state of the body
            bodies[b]->updateBroadPhaseState();
        }
    }
}

// Initialize the bodies velocities arrays for the next simulation step.
void DynamicsWorld::initVelocityArrays() {

    // Allocate memory for the bodies velocity arrays
    uint nbBodies = mRigidBodies.size();
    if (mNbBodiesCapacity != nbBodies && nbBodies > 0) {
        if (mNbBodiesCapacity > 0) {
            delete[] mSplitLinearVelocities;
            delete[] mSplitAngularVelocities;
        }
        mNbBodiesCapacity = nbBodies;
        // TODO : Use better memory allocation here
        mSplitLinearVelocities = new Vector3[mNbBodiesCapacity];
        mSplitAngularVelocities = new Vector3[mNbBodiesCapacity];
        mConstrainedLinearVelocities = new Vector3[mNbBodiesCapacity];
        mConstrainedAngularVelocities = new Vector3[mNbBodiesCapacity];
        mConstrainedPositions = new Vector3[mNbBodiesCapacity];
        mConstrainedOrientations = new Quaternion[mNbBodiesCapacity];
        assert(mSplitLinearVelocities != NULL);
        assert(mSplitAngularVelocities != NULL);
        assert(mConstrainedLinearVelocities != NULL);
        assert(mConstrainedAngularVelocities != NULL);
        assert(mConstrainedPositions != NULL);
        assert(mConstrainedOrientations != NULL);
    }

    // Reset the velocities arrays
    for (uint i=0; i<mNbBodiesCapacity; i++) {
        mSplitLinearVelocities[i].setToZero();
        mSplitAngularVelocities[i].setToZero();
    }

    // Initialize the map of body indexes in the velocity arrays
    mMapBodyToConstrainedVelocityIndex.clear();
    std::set<RigidBody*>::const_iterator it;
    uint indexBody = 0;
    for (it = mRigidBodies.begin(); it != mRigidBodies.end(); ++it) {

        // Add the body into the map
        mMapBodyToConstrainedVelocityIndex.insert(std::make_pair(*it, indexBody));
        indexBody++;
    }
}

// Integrate the velocities of rigid bodies.
/// This method only set the temporary velocities but does not update
/// the actual velocitiy of the bodies. The velocities updated in this method
/// might violate the constraints and will be corrected in the constraint and
/// contact solver.
void DynamicsWorld::integrateRigidBodiesVelocities() {

    PROFILE("DynamicsWorld::integrateRigidBodiesVelocities()");

    // Initialize the bodies velocity arrays
    initVelocityArrays();

    // For each island of the world
    for (uint i=0; i < mNbIslands; i++) {

        RigidBody** bodies = mIslands[i]->getBodies();

        // For each body of the island
        for (uint b=0; b < mIslands[i]->getNbBodies(); b++) {

            // Insert the body into the map of constrained velocities
            uint indexBody = mMapBodyToConstrainedVelocityIndex.find(bodies[b])->second;

            assert(mSplitLinearVelocities[indexBody] == Vector3(0, 0, 0));
            assert(mSplitAngularVelocities[indexBody] == Vector3(0, 0, 0));

            // Integrate the external force to get the new velocity of the body
            mConstrainedLinearVelocities[indexBody] = bodies[b]->getLinearVelocity() +
                                        mTimeStep * bodies[b]->mMassInverse * bodies[b]->mExternalForce;
            mConstrainedAngularVelocities[indexBody] = bodies[b]->getAngularVelocity() +
                                        mTimeStep * bodies[b]->getInertiaTensorInverseWorld() *
                                        bodies[b]->mExternalTorque;

            // If the gravity has to be applied to this rigid body
            if (bodies[b]->isGravityEnabled() && mIsGravityEnabled) {

                // Integrate the gravity force
                mConstrainedLinearVelocities[indexBody] += mTimeStep * bodies[b]->mMassInverse *
                        bodies[b]->getMass() * mGravity;
            }

            // Apply the velocity damping
            // Damping force : F_c = -c' * v (c=damping factor)
            // Equation      : m * dv/dt = -c' * v
            //                 => dv/dt = -c * v (with c=c'/m)
            //                 => dv/dt + c * v = 0
            // Solution      : v(t) = v0 * e^(-c * t)
            //                 => v(t + dt) = v0 * e^(-c(t + dt))
            //                              = v0 * e^(-ct) * e^(-c * dt)
            //                              = v(t) * e^(-c * dt)
            //                 => v2 = v1 * e^(-c * dt)
            // Using Taylor Serie for e^(-x) : e^x ~ 1 + x + x^2/2! + ...
            //                              => e^(-x) ~ 1 - x
            //                 => v2 = v1 * (1 - c * dt)
            decimal linDampingFactor = bodies[b]->getLinearDamping();
            decimal angDampingFactor = bodies[b]->getAngularDamping();
            decimal linearDamping = pow(decimal(1.0) - linDampingFactor, mTimeStep);
            decimal angularDamping = pow(decimal(1.0) - angDampingFactor, mTimeStep);
            mConstrainedLinearVelocities[indexBody] *= linearDamping;
            mConstrainedAngularVelocities[indexBody] *= angularDamping;

            indexBody++;
        }
    }
}

// Solve the contacts and constraints
void DynamicsWorld::solveContactsAndConstraints() {

    PROFILE("DynamicsWorld::solveContactsAndConstraints()");

    // Set the velocities arrays
    mContactSolver.setSplitVelocitiesArrays(mSplitLinearVelocities, mSplitAngularVelocities);
    mContactSolver.setConstrainedVelocitiesArrays(mConstrainedLinearVelocities,
                                                  mConstrainedAngularVelocities);
    mConstraintSolver.setConstrainedVelocitiesArrays(mConstrainedLinearVelocities,
                                                     mConstrainedAngularVelocities);
    mConstraintSolver.setConstrainedPositionsArrays(mConstrainedPositions,
                                                    mConstrainedOrientations);

    // ---------- Solve velocity constraints for joints and contacts ---------- //

    // For each island of the world
    for (uint islandIndex = 0; islandIndex < mNbIslands; islandIndex++) {

        // Check if there are contacts and constraints to solve
        bool isConstraintsToSolve = mIslands[islandIndex]->getNbJoints() > 0;
        bool isContactsToSolve = mIslands[islandIndex]->getNbContactManifolds() > 0;
        if (!isConstraintsToSolve && !isContactsToSolve) continue;

        // If there are contacts in the current island
        if (isContactsToSolve) {

            // Initialize the solver
            mContactSolver.initializeForIsland(mTimeStep, mIslands[islandIndex]);

            // Warm start the contact solver
            mContactSolver.warmStart();
        }

        // If there are constraints
        if (isConstraintsToSolve) {

            // Initialize the constraint solver
            mConstraintSolver.initializeForIsland(mTimeStep, mIslands[islandIndex]);
        }

        // For each iteration of the velocity solver
        for (uint i=0; i<mNbVelocitySolverIterations; i++) {

            // Solve the constraints
            if (isConstraintsToSolve) {
                mConstraintSolver.solveVelocityConstraints(mIslands[islandIndex]);
            }

            // Solve the contacts
            if (isContactsToSolve) mContactSolver.solve();
        }        

        // Cache the lambda values in order to use them in the next
        // step and cleanup the contact solver
        if (isContactsToSolve) {
            mContactSolver.storeImpulses();
            mContactSolver.cleanup();
        }
    }
}

// Solve the position error correction of the constraints
void DynamicsWorld::solvePositionCorrection() {

    PROFILE("DynamicsWorld::solvePositionCorrection()");

    // Do not continue if there is no constraints
    if (mJoints.empty()) return;

    // For each island of the world
    for (uint islandIndex = 0; islandIndex < mNbIslands; islandIndex++) {

        // ---------- Solve the position error correction for the constraints ---------- //

        // For each iteration of the position (error correction) solver
        for (uint i=0; i<mNbPositionSolverIterations; i++) {

            // Solve the position constraints
            mConstraintSolver.solvePositionConstraints(mIslands[islandIndex]);
        }
    }
}

// Create a rigid body into the physics world
/**
 * @param transform Transformation from body local-space to world-space
 * @return A pointer to the body that has been created in the world
 */
RigidBody* DynamicsWorld::createRigidBody(const Transform& transform) {

    // Compute the body ID
    bodyindex bodyID = computeNextAvailableBodyID();

    // Largest index cannot be used (it is used for invalid index)
    assert(bodyID < std::numeric_limits<reactphysics3d::bodyindex>::max());

    // Create the rigid body
    RigidBody* rigidBody = new (mMemoryAllocator.allocate(sizeof(RigidBody))) RigidBody(transform,
                                                                                *this, bodyID);
    assert(rigidBody != NULL);

    // Add the rigid body to the physics world
    mBodies.insert(rigidBody);
    mRigidBodies.insert(rigidBody);

    // Return the pointer to the rigid body
    return rigidBody;
}

// Destroy a rigid body and all the joints which it belongs
/**
 * @param rigidBody Pointer to the body you want to destroy
 */
void DynamicsWorld::destroyRigidBody(RigidBody* rigidBody) {

    // Remove all the collision shapes of the body
    rigidBody->removeAllCollisionShapes();

    // Add the body ID to the list of free IDs
    mFreeBodiesIDs.push_back(rigidBody->getID());

    // Destroy all the joints in which the rigid body to be destroyed is involved
    JointListElement* element;
    for (element = rigidBody->mJointsList; element != NULL; element = element->next) {
        destroyJoint(element->joint);
    }

    // Reset the contact manifold list of the body
    rigidBody->resetContactManifoldsList();

    // Call the destructor of the rigid body
    rigidBody->~RigidBody();

    // Remove the rigid body from the list of rigid bodies
    mBodies.erase(rigidBody);
    mRigidBodies.erase(rigidBody);

    // Free the object from the memory allocator
    mMemoryAllocator.release(rigidBody, sizeof(RigidBody));
}

// Create a joint between two bodies in the world and return a pointer to the new joint
/**
 * @param jointInfo The information that is necessary to create the joint
 * @return A pointer to the joint that has been created in the world
 */
Joint* DynamicsWorld::createJoint(const JointInfo& jointInfo) {

    Joint* newJoint = NULL;

    // Allocate memory to create the new joint
    switch(jointInfo.type) {

        // Ball-and-Socket joint
        case BALLSOCKETJOINT:
        {
            void* allocatedMemory = mMemoryAllocator.allocate(sizeof(BallAndSocketJoint));
            const BallAndSocketJointInfo& info = static_cast<const BallAndSocketJointInfo&>(
                                                                                        jointInfo);
            newJoint = new (allocatedMemory) BallAndSocketJoint(info);
            break;
        }

        // Slider joint
        case SLIDERJOINT:
        {
            void* allocatedMemory = mMemoryAllocator.allocate(sizeof(SliderJoint));
            const SliderJointInfo& info = static_cast<const SliderJointInfo&>(jointInfo);
            newJoint = new (allocatedMemory) SliderJoint(info);
            break;
        }

        // Hinge joint
        case HINGEJOINT:
        {
            void* allocatedMemory = mMemoryAllocator.allocate(sizeof(HingeJoint));
            const HingeJointInfo& info = static_cast<const HingeJointInfo&>(jointInfo);
            newJoint = new (allocatedMemory) HingeJoint(info);
            break;
        }

        // Fixed joint
        case FIXEDJOINT:
        {
            void* allocatedMemory = mMemoryAllocator.allocate(sizeof(FixedJoint));
            const FixedJointInfo& info = static_cast<const FixedJointInfo&>(jointInfo);
            newJoint = new (allocatedMemory) FixedJoint(info);
            break;
        }

        default:
        {
            assert(false);
            return NULL;
        }
    }

    // If the collision between the two bodies of the constraint is disabled
    if (!jointInfo.isCollisionEnabled) {

        // Add the pair of bodies in the set of body pairs that cannot collide with each other
        mCollisionDetection.addNoCollisionPair(jointInfo.body1, jointInfo.body2);
    }

    // Add the joint into the world
    mJoints.insert(newJoint);

    // Add the joint into the joint list of the bodies involved in the joint
    addJointToBody(newJoint);

    // Return the pointer to the created joint
    return newJoint;
}

// Destroy a joint
/**
 * @param joint Pointer to the joint you want to destroy
 */
void DynamicsWorld::destroyJoint(Joint* joint) {

    assert(joint != NULL);

    // If the collision between the two bodies of the constraint was disabled
    if (!joint->isCollisionEnabled()) {

        // Remove the pair of bodies from the set of body pairs that cannot collide with each other
        mCollisionDetection.removeNoCollisionPair(joint->getBody1(), joint->getBody2());
    }

    // Wake up the two bodies of the joint
    joint->getBody1()->setIsSleeping(false);
    joint->getBody2()->setIsSleeping(false);

    // Remove the joint from the world
    mJoints.erase(joint);

    // Remove the joint from the joint list of the bodies involved in the joint
    joint->mBody1->removeJointFromJointsList(mMemoryAllocator, joint);
    joint->mBody2->removeJointFromJointsList(mMemoryAllocator, joint);

    size_t nbBytes = joint->getSizeInBytes();

    // Call the destructor of the joint
    joint->~Joint();

    // Release the allocated memory
    mMemoryAllocator.release(joint, nbBytes);
}

// Add the joint to the list of joints of the two bodies involved in the joint
void DynamicsWorld::addJointToBody(Joint* joint) {

    assert(joint != NULL);

    // Add the joint at the beginning of the linked list of joints of the first body
    void* allocatedMemory1 = mMemoryAllocator.allocate(sizeof(JointListElement));
    JointListElement* jointListElement1 = new (allocatedMemory1) JointListElement(joint,
                                                                     joint->mBody1->mJointsList);
    joint->mBody1->mJointsList = jointListElement1;

    // Add the joint at the beginning of the linked list of joints of the second body
    void* allocatedMemory2 = mMemoryAllocator.allocate(sizeof(JointListElement));
    JointListElement* jointListElement2 = new (allocatedMemory2) JointListElement(joint,
                                                                     joint->mBody2->mJointsList);
    joint->mBody2->mJointsList = jointListElement2;
}

// Compute the islands of awake bodies.
/// An island is an isolated group of rigid bodies that have constraints (joints or contacts)
/// between each other. This method computes the islands at each time step as follows: For each
/// awake rigid body, we run a Depth First Search (DFS) through the constraint graph of that body
/// (graph where nodes are the bodies and where the edges are the constraints between the bodies) to
/// find all the bodies that are connected with it (the bodies that share joints or contacts with
/// it). Then, we create an island with this group of connected bodies.
void DynamicsWorld::computeIslands() {

    PROFILE("DynamicsWorld::computeIslands()");

    uint nbBodies = mRigidBodies.size();

    // Clear all the islands
    for (uint i=0; i<mNbIslands; i++) {

        // Call the island destructor
        mIslands[i]->~Island();

        // Release the allocated memory for the island
        mMemoryAllocator.release(mIslands[i], sizeof(Island));
    }

    // Allocate and create the array of islands
    if (mNbIslandsCapacity != nbBodies && nbBodies > 0) {
        if (mNbIslandsCapacity > 0) {
            mMemoryAllocator.release(mIslands, sizeof(Island*) * mNbIslandsCapacity);
        }
        mNbIslandsCapacity = nbBodies;
        mIslands = (Island**)mMemoryAllocator.allocate(sizeof(Island*) * mNbIslandsCapacity);
    }
    mNbIslands = 0;

    int nbContactManifolds = 0;

    // Reset all the isAlreadyInIsland variables of bodies, joints and contact manifolds
    for (std::set<RigidBody*>::iterator it = mRigidBodies.begin(); it != mRigidBodies.end(); ++it) {
        int nbBodyManifolds = (*it)->resetIsAlreadyInIslandAndCountManifolds();
        nbContactManifolds += nbBodyManifolds;
    }
    for (std::set<Joint*>::iterator it = mJoints.begin(); it != mJoints.end(); ++it) {
        (*it)->mIsAlreadyInIsland = false;
    }

    // Create a stack (using an array) for the rigid bodies to visit during the Depth First Search
    size_t nbBytesStack = sizeof(RigidBody*) * nbBodies;
    RigidBody** stackBodiesToVisit = (RigidBody**)mMemoryAllocator.allocate(nbBytesStack);

    // For each rigid body of the world
    for (std::set<RigidBody*>::iterator it = mRigidBodies.begin(); it != mRigidBodies.end(); ++it) {

        RigidBody* body = *it;

        // If the body has already been added to an island, we go to the next body
        if (body->mIsAlreadyInIsland) continue;

        // If the body is static, we go to the next body
        if (body->getType() == STATIC) continue;

        // If the body is sleeping or inactive, we go to the next body
        if (body->isSleeping() || !body->isActive()) continue;

        // Reset the stack of bodies to visit
        uint stackIndex = 0;
        stackBodiesToVisit[stackIndex] = body;
        stackIndex++;
        body->mIsAlreadyInIsland = true;

        // Create the new island
        void* allocatedMemoryIsland = mMemoryAllocator.allocate(sizeof(Island));
        mIslands[mNbIslands] = new (allocatedMemoryIsland) Island(nbBodies,
                                                                  nbContactManifolds,
                                                                  mJoints.size(), mMemoryAllocator);

        // While there are still some bodies to visit in the stack
        while (stackIndex > 0) {

            // Get the next body to visit from the stack
            stackIndex--;
            RigidBody* bodyToVisit = stackBodiesToVisit[stackIndex];
            assert(bodyToVisit->isActive());

            // Awake the body if it is slepping
            bodyToVisit->setIsSleeping(false);

            // Add the body into the island
            mIslands[mNbIslands]->addBody(bodyToVisit);

            // If the current body is static, we do not want to perform the DFS
            // search across that body
            if (bodyToVisit->getType() == STATIC) continue;

            // For each contact manifold in which the current body is involded
            ContactManifoldListElement* contactElement;
            for (contactElement = bodyToVisit->mContactManifoldsList; contactElement != NULL;
                 contactElement = contactElement->next) {

                ContactManifold* contactManifold = contactElement->contactManifold;

                assert(contactManifold->getNbContactPoints() > 0);

                // Check if the current contact manifold has already been added into an island
                if (contactManifold->isAlreadyInIsland()) continue;

                // Add the contact manifold into the island
                mIslands[mNbIslands]->addContactManifold(contactManifold);
                contactManifold->mIsAlreadyInIsland = true;

                // Get the other body of the contact manifold
                RigidBody* body1 = static_cast<RigidBody*>(contactManifold->getBody1());
                RigidBody* body2 = static_cast<RigidBody*>(contactManifold->getBody2());
                RigidBody* otherBody = (body1->getID() == bodyToVisit->getID()) ? body2 : body1;

                // Check if the other body has already been added to the island
                if (otherBody->mIsAlreadyInIsland) continue;

                // Insert the other body into the stack of bodies to visit
                stackBodiesToVisit[stackIndex] = otherBody;
                stackIndex++;
                otherBody->mIsAlreadyInIsland = true;
            }

            // For each joint in which the current body is involved
            JointListElement* jointElement;
            for (jointElement = bodyToVisit->mJointsList; jointElement != NULL;
                 jointElement = jointElement->next) {

                Joint* joint = jointElement->joint;

                // Check if the current joint has already been added into an island
                if (joint->isAlreadyInIsland()) continue;

                // Add the joint into the island
                mIslands[mNbIslands]->addJoint(joint);
                joint->mIsAlreadyInIsland = true;

                // Get the other body of the contact manifold
                RigidBody* body1 = static_cast<RigidBody*>(joint->getBody1());
                RigidBody* body2 = static_cast<RigidBody*>(joint->getBody2());
                RigidBody* otherBody = (body1->getID() == bodyToVisit->getID()) ? body2 : body1;

                // Check if the other body has already been added to the island
                if (otherBody->mIsAlreadyInIsland) continue;

                // Insert the other body into the stack of bodies to visit
                stackBodiesToVisit[stackIndex] = otherBody;
                stackIndex++;
                otherBody->mIsAlreadyInIsland = true;
            }
        }

        // Reset the isAlreadyIsland variable of the static bodies so that they
        // can also be included in the other islands
        for (uint i=0; i < mIslands[mNbIslands]->mNbBodies; i++) {

            if (mIslands[mNbIslands]->mBodies[i]->getType() == STATIC) {
                mIslands[mNbIslands]->mBodies[i]->mIsAlreadyInIsland = false;
            }
        }

        mNbIslands++;
     }

    // Release the allocated memory for the stack of bodies to visit
    mMemoryAllocator.release(stackBodiesToVisit, nbBytesStack);
}

// Put bodies to sleep if needed.
/// For each island, if all the bodies have been almost still for a long enough period of
/// time, we put all the bodies of the island to sleep.
void DynamicsWorld::updateSleepingBodies() {

    PROFILE("DynamicsWorld::updateSleepingBodies()");

    const decimal sleepLinearVelocitySquare = mSleepLinearVelocity * mSleepLinearVelocity;
    const decimal sleepAngularVelocitySquare = mSleepAngularVelocity * mSleepAngularVelocity;

    // For each island of the world
    for (uint i=0; i<mNbIslands; i++) {

        decimal minSleepTime = DECIMAL_LARGEST;

        // For each body of the island
        RigidBody** bodies = mIslands[i]->getBodies();
        for (uint b=0; b < mIslands[i]->getNbBodies(); b++) {

            // Skip static bodies
            if (bodies[b]->getType() == STATIC) continue;

            // If the body is velocity is large enough to stay awake
            if (bodies[b]->getLinearVelocity().lengthSquare() > sleepLinearVelocitySquare ||
                bodies[b]->getAngularVelocity().lengthSquare() > sleepAngularVelocitySquare ||
                !bodies[b]->isAllowedToSleep()) {

                // Reset the sleep time of the body
                bodies[b]->mSleepTime = decimal(0.0);
                minSleepTime = decimal(0.0);
            }
            else {  // If the body velocity is bellow the sleeping velocity threshold

                // Increase the sleep time
                bodies[b]->mSleepTime += mTimeStep;
                if (bodies[b]->mSleepTime < minSleepTime) {
                    minSleepTime = bodies[b]->mSleepTime;
                }
            }
        }

        // If the velocity of all the bodies of the island is under the
        // sleeping velocity threshold for a period of time larger than
        // the time required to become a sleeping body
        if (minSleepTime >= mTimeBeforeSleep) {

            // Put all the bodies of the island to sleep
            for (uint b=0; b < mIslands[i]->getNbBodies(); b++) {
                bodies[b]->setIsSleeping(true);
            }
        }
    }
}

// Enable/Disable the sleeping technique.
/// The sleeping technique is used to put bodies that are not moving into sleep
/// to speed up the simulation.
/**
 * @param isSleepingEnabled True if you want to enable the sleeping technique
 *                          and false otherwise
 */
void DynamicsWorld::enableSleeping(bool isSleepingEnabled) {
    mIsSleepingEnabled = isSleepingEnabled;

    if (!mIsSleepingEnabled) {

        // For each body of the world
        std::set<RigidBody*>::iterator it;
        for (it = mRigidBodies.begin(); it != mRigidBodies.end(); ++it) {

            // Wake up the rigid body
            (*it)->setIsSleeping(false);
        }
    }
}

// Test and report collisions between a given shape and all the others
// shapes of the world.
/// This method should be called after calling the
/// DynamicsWorld::update() method that will compute the collisions.
/**
 * @param shape Pointer to the proxy shape to test
 * @param callback Pointer to the object with the callback method
 */
void DynamicsWorld::testCollision(const ProxyShape* shape,
                                   CollisionCallback* callback) {

    // Create the sets of shapes
    std::set<uint> shapes;
    shapes.insert(shape->mBroadPhaseID);
    std::set<uint> emptySet;

    // Perform the collision detection and report contacts
    mCollisionDetection.reportCollisionBetweenShapes(callback, shapes, emptySet);
}

// Test and report collisions between two given shapes.
/// This method should be called after calling the
/// DynamicsWorld::update() method that will compute the collisions.
/**
 * @param shape1 Pointer to the first proxy shape to test
 * @param shape2 Pointer to the second proxy shape to test
 * @param callback Pointer to the object with the callback method
 */
void DynamicsWorld::testCollision(const ProxyShape* shape1,
                                   const ProxyShape* shape2,
                                   CollisionCallback* callback) {

    // Create the sets of shapes
    std::set<uint> shapes1;
    shapes1.insert(shape1->mBroadPhaseID);
    std::set<uint> shapes2;
    shapes2.insert(shape2->mBroadPhaseID);

    // Perform the collision detection and report contacts
    mCollisionDetection.reportCollisionBetweenShapes(callback, shapes1, shapes2);
}

// Test and report collisions between a body and all the others bodies of the
// world.
/// This method should be called after calling the
/// DynamicsWorld::update() method that will compute the collisions.
/**
 * @param body Pointer to the first body to test
 * @param callback Pointer to the object with the callback method
 */
void DynamicsWorld::testCollision(const CollisionBody* body,
                                   CollisionCallback* callback) {

    // Create the sets of shapes
    std::set<uint> shapes1;

    // For each shape of the body
    for (const ProxyShape* shape=body->getProxyShapesList(); shape != NULL;
         shape = shape->getNext()) {
        shapes1.insert(shape->mBroadPhaseID);
    }

    std::set<uint> emptySet;

    // Perform the collision detection and report contacts
    mCollisionDetection.reportCollisionBetweenShapes(callback, shapes1, emptySet);
}

// Test and report collisions between two bodies.
/// This method should be called after calling the
/// DynamicsWorld::update() method that will compute the collisions.
/**
 * @param body1 Pointer to the first body to test
 * @param body2 Pointer to the second body to test
 * @param callback Pointer to the object with the callback method
 */
void DynamicsWorld::testCollision(const CollisionBody* body1,
                                   const CollisionBody* body2,
                                   CollisionCallback* callback) {

    // Create the sets of shapes
    std::set<uint> shapes1;
    for (const ProxyShape* shape=body1->getProxyShapesList(); shape != NULL;
         shape = shape->getNext()) {
        shapes1.insert(shape->mBroadPhaseID);
    }

    std::set<uint> shapes2;
    for (const ProxyShape* shape=body2->getProxyShapesList(); shape != NULL;
         shape = shape->getNext()) {
        shapes2.insert(shape->mBroadPhaseID);
    }

    // Perform the collision detection and report contacts
    mCollisionDetection.reportCollisionBetweenShapes(callback, shapes1, shapes2);
}

// Test and report collisions between all shapes of the world.
/// This method should be called after calling the
/// DynamicsWorld::update() method that will compute the collisions.
/**
 * @param callback Pointer to the object with the callback method
 */
void DynamicsWorld::testCollision(CollisionCallback* callback) {

    std::set<uint> emptySet;

    // Perform the collision detection and report contacts
    mCollisionDetection.reportCollisionBetweenShapes(callback, emptySet, emptySet);
}

/// Return the list of all contacts of the world
std::vector<const ContactManifold*> DynamicsWorld::getContactsList() const {

    std::vector<const ContactManifold*> contactManifolds;

    // For each currently overlapping pair of bodies
    std::map<overlappingpairid, OverlappingPair*>::const_iterator it;
    for (it = mCollisionDetection.mOverlappingPairs.begin();
         it != mCollisionDetection.mOverlappingPairs.end(); ++it) {

        OverlappingPair* pair = it->second;

        // For each contact manifold of the pair
        const ContactManifoldSet& manifoldSet = pair->getContactManifoldSet();
        for (int i=0; i<manifoldSet.getNbContactManifolds(); i++) {

            ContactManifold* manifold = manifoldSet.getContactManifold(i);

            // Get the contact manifold
            contactManifolds.push_back(manifold);
        }
    }

    // Return all the contact manifold
    return contactManifolds;
}
