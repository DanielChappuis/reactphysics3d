/********************************************************************************
* ReactPhysics3D physics library, http://www.reactphysics3d.com                 *
* Copyright (c) 2010-2019 Daniel Chappuis                                       *
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
#include "utils/Profiler.h"
#include "engine/EventListener.h"
#include "engine/Island.h"
#include "collision/ContactManifold.h"

// Namespaces
using namespace reactphysics3d;
using namespace std;

// Constructor
/**
 * @param gravity Gravity vector in the world (in meters per second squared)
 * @param worldSettings The settings of the world
 * @param logger Pointer to the logger
 * @param profiler Pointer to the profiler
 */
DynamicsWorld::DynamicsWorld(const Vector3& gravity, const WorldSettings& worldSettings,
                             Logger* logger, Profiler* profiler)
              : CollisionWorld(worldSettings, logger, profiler),
                mContactSolver(mMemoryManager, mConfig),
                mNbVelocitySolverIterations(mConfig.defaultVelocitySolverNbIterations),
                mNbPositionSolverIterations(mConfig.defaultPositionSolverNbIterations),
                mIsSleepingEnabled(mConfig.isSleepingEnabled), mRigidBodies(mMemoryManager.getPoolAllocator()),
                mJoints(mMemoryManager.getPoolAllocator()), mGravity(gravity), mTimeStep(decimal(1.0f / 60.0f)),
                mIsGravityEnabled(true), mConstrainedLinearVelocities(nullptr),
                mConstrainedAngularVelocities(nullptr), mSplitLinearVelocities(nullptr),
                mSplitAngularVelocities(nullptr), mConstrainedPositions(nullptr),
                mConstrainedOrientations(nullptr), mNbIslands(0), mIslands(nullptr),
                mSleepLinearVelocity(mConfig.defaultSleepLinearVelocity),
                mSleepAngularVelocity(mConfig.defaultSleepAngularVelocity),
                mTimeBeforeSleep(mConfig.defaultTimeBeforeSleep),
                mFreeJointsIDs(mMemoryManager.getPoolAllocator()), mCurrentJointId(0) {

#ifdef IS_PROFILING_ACTIVE

	// Set the profiler
    mConstraintSolver.setProfiler(mProfiler);
    mContactSolver.setProfiler(mProfiler);

#endif

    RP3D_LOG(mLogger, Logger::Level::Information, Logger::Category::World,
             "Dynamics World: Dynamics world " + mName + " has been created");

}

// Destructor
DynamicsWorld::~DynamicsWorld() {

    // Destroy all the joints that have not been removed
    for (int i=mJoints.size() - 1; i >= 0; i--) {
        destroyJoint(mJoints[i]);
    }

    // Destroy all the rigid bodies that have not been removed
    for (int i=mRigidBodies.size() - 1; i >= 0; i--) {
        destroyRigidBody(mRigidBodies[i]);
    }

    assert(mJoints.size() == 0);
    assert(mRigidBodies.size() == 0);

#ifdef IS_PROFILING_ACTIVE

    // Print the profiling report into the destinations
    mProfiler->printReport();
#endif

    RP3D_LOG(mLogger, Logger::Level::Information, Logger::Category::World,
             "Dynamics World: Dynamics world " + mName + " has been destroyed");
}

// Update the physics simulation
/**
 * @param timeStep The amount of time to step the simulation by (in seconds)
 */
void DynamicsWorld::update(decimal timeStep) {

#ifdef IS_PROFILING_ACTIVE
    // Increment the frame counter of the profiler
    mProfiler->incrementFrameCounter();
#endif

    RP3D_PROFILE("DynamicsWorld::update()", mProfiler);

    mTimeStep = timeStep;

    // Notify the event listener about the beginning of an internal tick
    if (mEventListener != nullptr) mEventListener->beginInternalTick();

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
    if (mEventListener != nullptr) mEventListener->endInternalTick();

    // Reset the external force and torque applied to the bodies
    resetBodiesForceAndTorque();

    // Reset the single frame memory allocator
    mMemoryManager.resetFrameAllocator();
}

// Integrate position and orientation of the rigid bodies.
/// The positions and orientations of the bodies are integrated using
/// the sympletic Euler time stepping scheme.
void DynamicsWorld::integrateRigidBodiesPositions() {

    RP3D_PROFILE("DynamicsWorld::integrateRigidBodiesPositions()", mProfiler);
    
    // For each island of the world
    for (uint i=0; i < mNbIslands; i++) {

        RigidBody** bodies = mIslands[i]->getBodies();

        // For each body of the island
        for (uint b=0; b < mIslands[i]->getNbBodies(); b++) {

            // Get the constrained velocity
            uint indexArray = bodies[b]->mArrayIndex;
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

    RP3D_PROFILE("DynamicsWorld::updateBodiesState()", mProfiler);

    // For each island of the world
    for (uint islandIndex = 0; islandIndex < mNbIslands; islandIndex++) {

        // For each body of the island
        RigidBody** bodies = mIslands[islandIndex]->getBodies();

        for (uint b=0; b < mIslands[islandIndex]->getNbBodies(); b++) {

            uint index = bodies[b]->mArrayIndex;

            // Update the linear and angular velocity of the body
            bodies[b]->mLinearVelocity = mConstrainedLinearVelocities[index];
            bodies[b]->mAngularVelocity = mConstrainedAngularVelocities[index];

            // Update the position of the center of mass of the body
            bodies[b]->mCenterOfMassWorld = mConstrainedPositions[index];

            // Update the orientation of the body
            bodies[b]->mTransform.setOrientation(mConstrainedOrientations[index].getUnit());

            // Update the transform of the body (using the new center of mass and new orientation)
            bodies[b]->updateTransformWithCenterOfMass();

            // Update the world inverse inertia tensor of the body
            bodies[b]->updateInertiaTensorInverseWorld();

            // Update the broad-phase state of the body
            bodies[b]->updateBroadPhaseState();
        }
    }
}

// Initialize the bodies velocities arrays for the next simulation step.
void DynamicsWorld::initVelocityArrays() {

    RP3D_PROFILE("DynamicsWorld::initVelocityArrays()", mProfiler);

    // Allocate memory for the bodies velocity arrays
    uint nbBodies = mRigidBodies.size();

    mSplitLinearVelocities = static_cast<Vector3*>(mMemoryManager.allocate(MemoryManager::AllocationType::Frame,
                                                                           nbBodies * sizeof(Vector3)));
    mSplitAngularVelocities = static_cast<Vector3*>(mMemoryManager.allocate(MemoryManager::AllocationType::Frame,
                                                                            nbBodies * sizeof(Vector3)));
    mConstrainedLinearVelocities = static_cast<Vector3*>(mMemoryManager.allocate(MemoryManager::AllocationType::Frame,
                                                                                 nbBodies * sizeof(Vector3)));
    mConstrainedAngularVelocities = static_cast<Vector3*>(mMemoryManager.allocate(MemoryManager::AllocationType::Frame,
                                                                                  nbBodies * sizeof(Vector3)));
    mConstrainedPositions = static_cast<Vector3*>(mMemoryManager.allocate(MemoryManager::AllocationType::Frame,
                                                                          nbBodies * sizeof(Vector3)));
    mConstrainedOrientations = static_cast<Quaternion*>(mMemoryManager.allocate(MemoryManager::AllocationType::Frame,
                                                                                nbBodies * sizeof(Quaternion)));
    assert(mSplitLinearVelocities != nullptr);
    assert(mSplitAngularVelocities != nullptr);
    assert(mConstrainedLinearVelocities != nullptr);
    assert(mConstrainedAngularVelocities != nullptr);
    assert(mConstrainedPositions != nullptr);
    assert(mConstrainedOrientations != nullptr);

    // Initialize the map of body indexes in the velocity arrays
    uint i = 0;
    for (List<RigidBody*>::Iterator it = mRigidBodies.begin(); it != mRigidBodies.end(); ++it) {

        mSplitLinearVelocities[i].setToZero();
        mSplitAngularVelocities[i].setToZero();

        (*it)->mArrayIndex = i++;
    }
}

// Integrate the velocities of rigid bodies.
/// This method only set the temporary velocities but does not update
/// the actual velocitiy of the bodies. The velocities updated in this method
/// might violate the constraints and will be corrected in the constraint and
/// contact solver.
void DynamicsWorld::integrateRigidBodiesVelocities() {

    RP3D_PROFILE("DynamicsWorld::integrateRigidBodiesVelocities()", mProfiler);

    // Initialize the bodies velocity arrays
    initVelocityArrays();

    // For each island of the world
    for (uint i=0; i < mNbIslands; i++) {

        RigidBody** bodies = mIslands[i]->getBodies();

        // For each body of the island
        for (uint b=0; b < mIslands[i]->getNbBodies(); b++) {

            // Insert the body into the map of constrained velocities
            uint indexBody = bodies[b]->mArrayIndex;

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

    RP3D_PROFILE("DynamicsWorld::solveContactsAndConstraints()", mProfiler);

    // Set the velocities arrays
    mContactSolver.setSplitVelocitiesArrays(mSplitLinearVelocities, mSplitAngularVelocities);
    mContactSolver.setConstrainedVelocitiesArrays(mConstrainedLinearVelocities,
                                                  mConstrainedAngularVelocities);
    mConstraintSolver.setConstrainedVelocitiesArrays(mConstrainedLinearVelocities,
                                                     mConstrainedAngularVelocities);
    mConstraintSolver.setConstrainedPositionsArrays(mConstrainedPositions,
                                                    mConstrainedOrientations);

    // ---------- Solve velocity constraints for joints and contacts ---------- //

    // Initialize the contact solver
    mContactSolver.init(mIslands, mNbIslands, mTimeStep);

    // For each island of the world
    for (uint islandIndex = 0; islandIndex < mNbIslands; islandIndex++) {

        // If there are constraints to solve
        if (mIslands[islandIndex]->getNbJoints() > 0) {

            // Initialize the constraint solver
            mConstraintSolver.initializeForIsland(mTimeStep, mIslands[islandIndex]);
        }
    }

    // For each iteration of the velocity solver
    for (uint i=0; i<mNbVelocitySolverIterations; i++) {

        for (uint islandIndex = 0; islandIndex < mNbIslands; islandIndex++) {

            // Solve the constraints
            if (mIslands[islandIndex]->getNbJoints() > 0) {

                mConstraintSolver.solveVelocityConstraints(mIslands[islandIndex]);
            }
        }

        mContactSolver.solve();
    }

    mContactSolver.storeImpulses();
}

// Solve the position error correction of the constraints
void DynamicsWorld::solvePositionCorrection() {

    RP3D_PROFILE("DynamicsWorld::solvePositionCorrection()", mProfiler);

    // Do not continue if there is no constraints
    if (mJoints.size() == 0) return;

    // For each island of the world
    for (uint islandIndex = 0; islandIndex < mNbIslands; islandIndex++) {

        // ---------- Solve the position error correction for the constraints ---------- //

        if (mIslands[islandIndex]->getNbJoints() > 0) {

            // For each iteration of the position (error correction) solver
            for (uint i=0; i<mNbPositionSolverIterations; i++) {

                // Solve the position constraints
                mConstraintSolver.solvePositionConstraints(mIslands[islandIndex]);
            }
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
    bodyindex bodyID = computeNextAvailableBodyId();

    // Largest index cannot be used (it is used for invalid index)
    assert(bodyID < std::numeric_limits<reactphysics3d::bodyindex>::max());

    // Create the rigid body
    RigidBody* rigidBody = new (mMemoryManager.allocate(MemoryManager::AllocationType::Pool,
                                                        sizeof(RigidBody))) RigidBody(transform, *this, bodyID);
    assert(rigidBody != nullptr);

    // Add the rigid body to the physics world
    mBodies.add(rigidBody);
    mRigidBodies.add(rigidBody);

#ifdef IS_PROFILING_ACTIVE
    rigidBody->setProfiler(mProfiler);
#endif

#ifdef IS_LOGGING_ACTIVE
   rigidBody->setLogger(mLogger);
#endif

    RP3D_LOG(mLogger, Logger::Level::Information, Logger::Category::Body,
             "Body " + std::to_string(bodyID) + ": New collision body created");

    // Return the pointer to the rigid body
    return rigidBody;
}

// Destroy a rigid body and all the joints which it belongs
/**
 * @param rigidBody Pointer to the body you want to destroy
 */
void DynamicsWorld::destroyRigidBody(RigidBody* rigidBody) {

    RP3D_LOG(mLogger, Logger::Level::Information, Logger::Category::Body,
             "Body " + std::to_string(rigidBody->getId()) + ": rigid body destroyed");

    // Remove all the collision shapes of the body
    rigidBody->removeAllCollisionShapes();

    // Add the body ID to the list of free IDs
    mFreeBodiesIds.add(rigidBody->getId());

    // Destroy all the joints in which the rigid body to be destroyed is involved
    JointListElement* element;
    for (element = rigidBody->mJointsList; element != nullptr; element = element->next) {
        destroyJoint(element->joint);
    }

    // Reset the contact manifold list of the body
    rigidBody->resetContactManifoldsList();

    // Call the destructor of the rigid body
    rigidBody->~RigidBody();

    // Remove the rigid body from the list of rigid bodies
    mBodies.remove(rigidBody);
    mRigidBodies.remove(rigidBody);

    // Free the object from the memory allocator
    mMemoryManager.release(MemoryManager::AllocationType::Pool, rigidBody, sizeof(RigidBody));
}

// Create a joint between two bodies in the world and return a pointer to the new joint
/**
 * @param jointInfo The information that is necessary to create the joint
 * @return A pointer to the joint that has been created in the world
 */
Joint* DynamicsWorld::createJoint(const JointInfo& jointInfo) {

    Joint* newJoint = nullptr;

    // Get the next available joint ID
    uint jointId = computeNextAvailableJointId();

    // Allocate memory to create the new joint
    switch(jointInfo.type) {

        // Ball-and-Socket joint
        case JointType::BALLSOCKETJOINT:
        {
            void* allocatedMemory = mMemoryManager.allocate(MemoryManager::AllocationType::Pool,
                                                            sizeof(BallAndSocketJoint));
            const BallAndSocketJointInfo& info = static_cast<const BallAndSocketJointInfo&>(
                                                                                        jointInfo);
            newJoint = new (allocatedMemory) BallAndSocketJoint(jointId, info);
            break;
        }

        // Slider joint
        case JointType::SLIDERJOINT:
        {
            void* allocatedMemory = mMemoryManager.allocate(MemoryManager::AllocationType::Pool,
                                                            sizeof(SliderJoint));
            const SliderJointInfo& info = static_cast<const SliderJointInfo&>(jointInfo);
            newJoint = new (allocatedMemory) SliderJoint(jointId, info);
            break;
        }

        // Hinge joint
        case JointType::HINGEJOINT:
        {
            void* allocatedMemory = mMemoryManager.allocate(MemoryManager::AllocationType::Pool,
                                                            sizeof(HingeJoint));
            const HingeJointInfo& info = static_cast<const HingeJointInfo&>(jointInfo);
            newJoint = new (allocatedMemory) HingeJoint(jointId, info);
            break;
        }

        // Fixed joint
        case JointType::FIXEDJOINT:
        {
            void* allocatedMemory = mMemoryManager.allocate(MemoryManager::AllocationType::Pool,
                                                            sizeof(FixedJoint));
            const FixedJointInfo& info = static_cast<const FixedJointInfo&>(jointInfo);
            newJoint = new (allocatedMemory) FixedJoint(jointId, info);
            break;
        }

        default:
        {
            assert(false);
            return nullptr;
        }
    }

    // If the collision between the two bodies of the constraint is disabled
    if (!jointInfo.isCollisionEnabled) {

        // Add the pair of bodies in the set of body pairs that cannot collide with each other
        mCollisionDetection.addNoCollisionPair(jointInfo.body1, jointInfo.body2);
    }

    // Add the joint into the world
    mJoints.add(newJoint);

    RP3D_LOG(mLogger, Logger::Level::Information, Logger::Category::Joint,
             "Joint " + std::to_string(newJoint->getId()) + ": New joint created");
    RP3D_LOG(mLogger, Logger::Level::Information, Logger::Category::Joint,
             "Joint " + std::to_string(newJoint->getId()) + ": " + newJoint->to_string());

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

    assert(joint != nullptr);

    RP3D_LOG(mLogger, Logger::Level::Information, Logger::Category::Joint,
             "Joint " + std::to_string(joint->getId()) + ": joint destroyed");

    // If the collision between the two bodies of the constraint was disabled
    if (!joint->isCollisionEnabled()) {

        // Remove the pair of bodies from the set of body pairs that cannot collide with each other
        mCollisionDetection.removeNoCollisionPair(joint->getBody1(), joint->getBody2());
    }

    // Wake up the two bodies of the joint
    joint->getBody1()->setIsSleeping(false);
    joint->getBody2()->setIsSleeping(false);

    // Remove the joint from the world
    mJoints.remove(joint);

    // Remove the joint from the joint list of the bodies involved in the joint
    joint->mBody1->removeJointFromJointsList(mMemoryManager, joint);
    joint->mBody2->removeJointFromJointsList(mMemoryManager, joint);

    size_t nbBytes = joint->getSizeInBytes();

    // Add the joint ID to the list of free IDs
    mFreeJointsIDs.add(joint->getId());

    // Call the destructor of the joint
    joint->~Joint();

    // Add the joint ID to the list of free IDs
    mFreeJointsIDs.add(joint->getId());

    // Release the allocated memory
    mMemoryManager.release(MemoryManager::AllocationType::Pool, joint, nbBytes);
}

// Add the joint to the list of joints of the two bodies involved in the joint
void DynamicsWorld::addJointToBody(Joint* joint) {

    assert(joint != nullptr);

    // Add the joint at the beginning of the linked list of joints of the first body
    void* allocatedMemory1 = mMemoryManager.allocate(MemoryManager::AllocationType::Pool,
                                                     sizeof(JointListElement));
    JointListElement* jointListElement1 = new (allocatedMemory1) JointListElement(joint,
                                                                     joint->mBody1->mJointsList);
    joint->mBody1->mJointsList = jointListElement1;

    RP3D_LOG(mLogger, Logger::Level::Information, Logger::Category::Body,
             "Body " + std::to_string(joint->mBody1->getId()) + ": Joint " + std::to_string(joint->getId()) +
             " added to body");

    // Add the joint at the beginning of the linked list of joints of the second body
    void* allocatedMemory2 = mMemoryManager.allocate(MemoryManager::AllocationType::Pool,
                                                     sizeof(JointListElement));
    JointListElement* jointListElement2 = new (allocatedMemory2) JointListElement(joint,
                                                                     joint->mBody2->mJointsList);
    joint->mBody2->mJointsList = jointListElement2;

    RP3D_LOG(mLogger, Logger::Level::Information, Logger::Category::Body,
             "Body " + std::to_string(joint->mBody2->getId()) + ": Joint " + std::to_string(joint->getId()) +
             " added to body");
}

// Return the next available joint Id
uint DynamicsWorld::computeNextAvailableJointId() {

    // Compute the joint ID
    uint jointId;
    if (mFreeJointsIDs.size() != 0) {
        jointId = mFreeJointsIDs[mFreeJointsIDs.size() - 1];
        mFreeJointsIDs.removeAt(mFreeJointsIDs.size() - 1);
    }
    else {
        jointId = mCurrentJointId;
        mCurrentJointId++;
    }

    return jointId;
}

// Compute the islands of awake bodies.
/// An island is an isolated group of rigid bodies that have constraints (joints or contacts)
/// between each other. This method computes the islands at each time step as follows: For each
/// awake rigid body, we run a Depth First Search (DFS) through the constraint graph of that body
/// (graph where nodes are the bodies and where the edges are the constraints between the bodies) to
/// find all the bodies that are connected with it (the bodies that share joints or contacts with
/// it). Then, we create an island with this group of connected bodies.
void DynamicsWorld::computeIslands() {

    RP3D_PROFILE("DynamicsWorld::computeIslands()", mProfiler);

    uint nbBodies = mRigidBodies.size();

    // Allocate and create the array of islands pointer. This memory is allocated
    // in the single frame allocator
    mIslands = static_cast<Island**>(mMemoryManager.allocate(MemoryManager::AllocationType::Frame,
                                                             sizeof(Island*) * nbBodies));
    mNbIslands = 0;

    int nbContactManifolds = 0;

    // Reset all the isAlreadyInIsland variables of bodies, joints and contact manifolds
    for (List<RigidBody*>::Iterator it = mRigidBodies.begin(); it != mRigidBodies.end(); ++it) {
        int nbBodyManifolds = (*it)->resetIsAlreadyInIslandAndCountManifolds();
        nbContactManifolds += nbBodyManifolds;
    }
    for (List<Joint*>::Iterator it = mJoints.begin(); it != mJoints.end(); ++it) {
        (*it)->mIsAlreadyInIsland = false;
    }

    // Create a stack (using an array) for the rigid bodies to visit during the Depth First Search
    size_t nbBytesStack = sizeof(RigidBody*) * nbBodies;
    RigidBody** stackBodiesToVisit = static_cast<RigidBody**>(mMemoryManager.allocate(MemoryManager::AllocationType::Frame,
                                                                                      nbBytesStack));

    // For each rigid body of the world
    for (List<RigidBody*>::Iterator it = mRigidBodies.begin(); it != mRigidBodies.end(); ++it) {

        RigidBody* body = *it;

        // If the body has already been added to an island, we go to the next body
        if (body->mIsAlreadyInIsland) continue;

        // If the body is static, we go to the next body
        if (body->getType() == BodyType::STATIC) continue;

        // If the body is sleeping or inactive, we go to the next body
        if (body->isSleeping() || !body->isActive()) continue;

        // Reset the stack of bodies to visit
        uint stackIndex = 0;
        stackBodiesToVisit[stackIndex] = body;
        stackIndex++;
        body->mIsAlreadyInIsland = true;

        // Create the new island
        void* allocatedMemoryIsland = mMemoryManager.allocate(MemoryManager::AllocationType::Frame,
                                                              sizeof(Island));
        mIslands[mNbIslands] = new (allocatedMemoryIsland) Island(nbBodies, nbContactManifolds, mJoints.size(),
                                                                  mMemoryManager);

        // While there are still some bodies to visit in the stack
        while (stackIndex > 0) {

            // Get the next body to visit from the stack
            stackIndex--;
            RigidBody* bodyToVisit = stackBodiesToVisit[stackIndex];
            assert(bodyToVisit->isActive());

            // Awake the body if it is sleeping
            bodyToVisit->setIsSleeping(false);

            // Add the body into the island
            mIslands[mNbIslands]->addBody(bodyToVisit);

            // If the current body is static, we do not want to perform the DFS
            // search across that body
            if (bodyToVisit->getType() == BodyType::STATIC) continue;

            // For each contact manifold in which the current body is involded
            ContactManifoldListElement* contactElement;
            for (contactElement = bodyToVisit->mContactManifoldsList; contactElement != nullptr;
                 contactElement = contactElement->getNext()) {

                ContactManifold* contactManifold = contactElement->getContactManifold();

                assert(contactManifold->getNbContactPoints() > 0);

                // Check if the current contact manifold has already been added into an island
                if (contactManifold->isAlreadyInIsland()) continue;

                // Get the other body of the contact manifold
                RigidBody* body1 = dynamic_cast<RigidBody*>(contactManifold->getBody1());
                RigidBody* body2 = dynamic_cast<RigidBody*>(contactManifold->getBody2());

                // If the colliding body is a RigidBody (and not a CollisionBody instead)
                if (body1 != nullptr && body2 != nullptr) {

                    // Add the contact manifold into the island
                    mIslands[mNbIslands]->addContactManifold(contactManifold);
                    contactManifold->mIsAlreadyInIsland = true;

                    RigidBody* otherBody = (body1->getId() == bodyToVisit->getId()) ? body2 : body1;

                    // Check if the other body has already been added to the island
                    if (otherBody->mIsAlreadyInIsland) continue;

                    // Insert the other body into the stack of bodies to visit
                    stackBodiesToVisit[stackIndex] = otherBody;
                    stackIndex++;
                    otherBody->mIsAlreadyInIsland = true;
                }
            }

            // For each joint in which the current body is involved
            JointListElement* jointElement;
            for (jointElement = bodyToVisit->mJointsList; jointElement != nullptr;
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
                RigidBody* otherBody = (body1->getId() == bodyToVisit->getId()) ? body2 : body1;

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

            if (mIslands[mNbIslands]->mBodies[i]->getType() == BodyType::STATIC) {
                mIslands[mNbIslands]->mBodies[i]->mIsAlreadyInIsland = false;
            }
        }

        mNbIslands++;
     }
}

// Put bodies to sleep if needed.
/// For each island, if all the bodies have been almost still for a long enough period of
/// time, we put all the bodies of the island to sleep.
void DynamicsWorld::updateSleepingBodies() {

    RP3D_PROFILE("DynamicsWorld::updateSleepingBodies()", mProfiler);

    const decimal sleepLinearVelocitySquare = mSleepLinearVelocity * mSleepLinearVelocity;
    const decimal sleepAngularVelocitySquare = mSleepAngularVelocity * mSleepAngularVelocity;

    // For each island of the world
    for (uint i=0; i<mNbIslands; i++) {

        decimal minSleepTime = DECIMAL_LARGEST;

        // For each body of the island
        RigidBody** bodies = mIslands[i]->getBodies();
        for (uint b=0; b < mIslands[i]->getNbBodies(); b++) {

            // Skip static bodies
            if (bodies[b]->getType() == BodyType::STATIC) continue;

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
        List<RigidBody*>::Iterator it;
        for (it = mRigidBodies.begin(); it != mRigidBodies.end(); ++it) {

            // Wake up the rigid body
            (*it)->setIsSleeping(false);
        }
    }

    RP3D_LOG(mLogger, Logger::Level::Information, Logger::Category::World,
             "Dynamics World: isSleepingEnabled=" + (isSleepingEnabled ? std::string("true") : std::string("false")) );
}

// Return the list of all contacts of the world
/**
 * @return A pointer to the first contact manifold in the linked-list of manifolds
 */
List<const ContactManifold*> DynamicsWorld::getContactsList() {

    List<const ContactManifold*> contactManifolds(mMemoryManager.getPoolAllocator());

    // For each currently overlapping pair of bodies
    Map<Pair<uint, uint>, OverlappingPair*>::Iterator it;
    for (it = mCollisionDetection.mOverlappingPairs.begin();
         it != mCollisionDetection.mOverlappingPairs.end(); ++it) {

        OverlappingPair* pair = it->second;

        // For each contact manifold of the pair
        const ContactManifoldSet& manifoldSet = pair->getContactManifoldSet();
        ContactManifold* manifold = manifoldSet.getContactManifolds();
        while (manifold != nullptr) {

            // Get the contact manifold
            contactManifolds.add(manifold);

            manifold = manifold->getNext();
        }
    }

    // Return all the contact manifold
    return contactManifolds;
}
