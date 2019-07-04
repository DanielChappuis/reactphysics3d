/********************************************************************************
* ReactPhysics3D physics library, http://www.reactphysics3d.com                 *
* Copyright (c) 2010-2018 Daniel Chappuis                                       *
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
#include "engine/Islands.h"
#include "collision/ContactManifold.h"
#include "containers/Stack.h"

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
DynamicsWorld::DynamicsWorld(const Vector3& gravity, const WorldSettings& worldSettings, Logger* logger, Profiler* profiler)
              : CollisionWorld(worldSettings, logger, profiler),
                mIslands(mMemoryManager.getSingleFrameAllocator()),
                mContactSolver(mMemoryManager, mIslands, mBodyComponents, mDynamicsComponents, mProxyShapesComponents, mConfig),
                mConstraintSolver(mIslands, mDynamicsComponents),
                mNbVelocitySolverIterations(mConfig.defaultVelocitySolverNbIterations),
                mNbPositionSolverIterations(mConfig.defaultPositionSolverNbIterations), 
                mIsSleepingEnabled(mConfig.isSleepingEnabled), mRigidBodies(mMemoryManager.getPoolAllocator()),
                mJoints(mMemoryManager.getPoolAllocator()), mGravity(gravity), mTimeStep(decimal(1.0f / 60.0f)),
                mIsGravityEnabled(true),  mSleepLinearVelocity(mConfig.defaultSleepLinearVelocity),
                mSleepAngularVelocity(mConfig.defaultSleepAngularVelocity), mTimeBeforeSleep(mConfig.defaultTimeBeforeSleep),
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

    // Compute the collision detection
    mCollisionDetection.computeCollisionDetection();

    // Create the islands
    createIslands();

    // Create the actual narrow-phase contacts
    mCollisionDetection.createContacts();

    // Report the contacts to the user
    mCollisionDetection.reportContacts();

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

    // Reset the islands
    mIslands.clear();

    // Reset the single frame memory allocator
    mMemoryManager.resetFrameAllocator();
}

// Integrate position and orientation of the rigid bodies.
/// The positions and orientations of the bodies are integrated using
/// the sympletic Euler time stepping scheme.
void DynamicsWorld::integrateRigidBodiesPositions() {

    RP3D_PROFILE("DynamicsWorld::integrateRigidBodiesPositions()", mProfiler);
    
    const decimal isSplitImpulseActive = mContactSolver.isSplitImpulseActive() ? decimal(1.0) : decimal(0.0);

    for (uint32 i=0; i < mDynamicsComponents.getNbEnabledComponents(); i++) {

        // Get the constrained velocity
        Vector3 newLinVelocity = mDynamicsComponents.mConstrainedLinearVelocities[i];
        Vector3 newAngVelocity = mDynamicsComponents.mConstrainedAngularVelocities[i];

        // Add the split impulse velocity from Contact Solver (only used
        // to update the position)
        newLinVelocity += isSplitImpulseActive * mDynamicsComponents.mSplitLinearVelocities[i];
        newAngVelocity += isSplitImpulseActive * mDynamicsComponents.mSplitAngularVelocities[i];

        // Get current position and orientation of the body
        const Vector3& currentPosition = mDynamicsComponents.mCentersOfMassWorld[i];
        const Quaternion& currentOrientation = mTransformComponents.getTransform(mDynamicsComponents.mBodies[i]).getOrientation();

        // Update the new constrained position and orientation of the body
        mDynamicsComponents.mConstrainedPositions[i] = currentPosition + newLinVelocity * mTimeStep;
        mDynamicsComponents.mConstrainedOrientations[i] = currentOrientation + Quaternion(0, newAngVelocity) *
                                                          currentOrientation * decimal(0.5) * mTimeStep;
    }
}

// Update the postion/orientation of the bodies
void DynamicsWorld::updateBodiesState() {

    RP3D_PROFILE("DynamicsWorld::updateBodiesState()", mProfiler);

    // TODO : Make sure we compute this in a system

    for (uint32 i=0; i < mDynamicsComponents.getNbEnabledComponents(); i++) {

        // Update the linear and angular velocity of the body
        mDynamicsComponents.mLinearVelocities[i] = mDynamicsComponents.mConstrainedLinearVelocities[i];
        mDynamicsComponents.mAngularVelocities[i] = mDynamicsComponents.mConstrainedAngularVelocities[i];

        // Update the position of the center of mass of the body
        mDynamicsComponents.mCentersOfMassWorld[i] = mDynamicsComponents.mConstrainedPositions[i];

        // Update the orientation of the body
        const Quaternion& constrainedOrientation = mDynamicsComponents.mConstrainedOrientations[i];
        mTransformComponents.getTransform(mDynamicsComponents.mBodies[i]).setOrientation(constrainedOrientation.getUnit());
    }

    // Update the transform of the body (using the new center of mass and new orientation)
    for (uint32 i=0; i < mDynamicsComponents.getNbEnabledComponents(); i++) {

        Transform& transform = mTransformComponents.getTransform(mDynamicsComponents.mBodies[i]);
        const Vector3& centerOfMassWorld = mDynamicsComponents.mCentersOfMassWorld[i];
        const Vector3& centerOfMassLocal = mDynamicsComponents.mCentersOfMassLocal[i];
        transform.setPosition(centerOfMassWorld - transform.getOrientation() * centerOfMassLocal);
    }

    // Update the world inverse inertia tensor of the body
    for (uint32 i=0; i < mDynamicsComponents.getNbEnabledComponents(); i++) {

        Matrix3x3 orientation = mTransformComponents.getTransform(mDynamicsComponents.mBodies[i]).getOrientation().getMatrix();
        const Matrix3x3& inverseInertiaLocalTensor = mDynamicsComponents.mInverseInertiaTensorsLocal[i];
        mDynamicsComponents.mInverseInertiaTensorsWorld[i] = orientation * inverseInertiaLocalTensor * orientation.getTranspose();
    }

    // Update the proxy-shapes components
    mCollisionDetection.updateProxyShapes();
}

// Reset the split velocities of the bodies
void DynamicsWorld::resetSplitVelocities() {

    for(uint32 i=0; i < mDynamicsComponents.getNbEnabledComponents(); i++) {
        mDynamicsComponents.mSplitLinearVelocities[i].setToZero();
        mDynamicsComponents.mSplitAngularVelocities[i].setToZero();
    }
}

// Integrate the velocities of rigid bodies.
/// This method only set the temporary velocities but does not update
/// the actual velocitiy of the bodies. The velocities updated in this method
/// might violate the constraints and will be corrected in the constraint and
/// contact solver.
void DynamicsWorld::integrateRigidBodiesVelocities() {

    RP3D_PROFILE("DynamicsWorld::integrateRigidBodiesVelocities()", mProfiler);

    // Reset the split velocities of the bodies
    resetSplitVelocities();

    // Integration component velocities using force/torque
    for (uint32 i=0; i < mDynamicsComponents.getNbEnabledComponents(); i++) {

        assert(mDynamicsComponents.mSplitLinearVelocities[i] == Vector3(0, 0, 0));
        assert(mDynamicsComponents.mSplitAngularVelocities[i] == Vector3(0, 0, 0));

        // Integrate the external force to get the new velocity of the body
        mDynamicsComponents.mConstrainedLinearVelocities[i] = mDynamicsComponents.mLinearVelocities[i] + mTimeStep *
                                                              mDynamicsComponents.mInverseMasses[i] * mDynamicsComponents.mExternalForces[i];
        mDynamicsComponents.mConstrainedAngularVelocities[i] = mDynamicsComponents.mAngularVelocities[i] +
                                                               mTimeStep * mDynamicsComponents.mInverseInertiaTensorsWorld[i] * mDynamicsComponents.mExternalTorques[i];
    }

    // Apply gravity force
    for (uint32 i=0; i < mDynamicsComponents.getNbEnabledComponents(); i++) {
        // If the gravity has to be applied to this rigid body
        if (mDynamicsComponents.mIsGravityEnabled[i] && mIsGravityEnabled) {

            // Integrate the gravity force
            mDynamicsComponents.mConstrainedLinearVelocities[i] = mDynamicsComponents.mConstrainedLinearVelocities[i] + mTimeStep *
                                                                  mDynamicsComponents.mInverseMasses[i] * mDynamicsComponents.mInitMasses[i] * mGravity;
        }
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
    for (uint32 i=0; i < mDynamicsComponents.getNbEnabledComponents(); i++) {

        const decimal linDampingFactor = mDynamicsComponents.mLinearDampings[i];
        const decimal angDampingFactor = mDynamicsComponents.mAngularDampings[i];
        const decimal linearDamping = pow(decimal(1.0) - linDampingFactor, mTimeStep);
        const decimal angularDamping = pow(decimal(1.0) - angDampingFactor, mTimeStep);
        mDynamicsComponents.mConstrainedLinearVelocities[i] = mDynamicsComponents.mConstrainedLinearVelocities[i] * linearDamping;
        mDynamicsComponents.mConstrainedAngularVelocities[i] = mDynamicsComponents.mConstrainedAngularVelocities[i] * angularDamping;
    }
}

// Solve the contacts and constraints
void DynamicsWorld::solveContactsAndConstraints() {

    RP3D_PROFILE("DynamicsWorld::solveContactsAndConstraints()", mProfiler);

    // ---------- Solve velocity constraints for joints and contacts ---------- //

    // Initialize the contact solver
    mContactSolver.init(mCollisionDetection.mCurrentContactManifolds, mCollisionDetection.mCurrentContactPoints, mTimeStep);

    // For each island of the world
    for (uint islandIndex = 0; islandIndex < mIslands.getNbIslands(); islandIndex++) {

        // If there are constraints to solve
        if (mIslands.joints[islandIndex].size() > 0) {

            // Initialize the constraint solver
            mConstraintSolver.initializeForIsland(mTimeStep, islandIndex);
        }
    }

    // For each iteration of the velocity solver
    for (uint i=0; i<mNbVelocitySolverIterations; i++) {

        for (uint islandIndex = 0; islandIndex < mIslands.getNbIslands(); islandIndex++) {

            // Solve the constraints
            if (mIslands.joints[islandIndex].size() > 0) {

                mConstraintSolver.solveVelocityConstraints(islandIndex);
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
    for (uint islandIndex = 0; islandIndex < mIslands.getNbIslands(); islandIndex++) {

        // ---------- Solve the position error correction for the constraints ---------- //

        if (mIslands.joints[islandIndex].size() > 0) {

            // For each iteration of the position (error correction) solver
            for (uint i=0; i<mNbPositionSolverIterations; i++) {

                // Solve the position constraints
                mConstraintSolver.solvePositionConstraints(islandIndex);
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

    // Create a new entity for the body
    Entity entity = mEntityManager.createEntity();

    mTransformComponents.addComponent(entity, false, TransformComponents::TransformComponent(transform));
    mDynamicsComponents.addComponent(entity, false, DynamicsComponents::DynamicsComponent(transform.getPosition()));

    // Create the rigid body
    RigidBody* rigidBody = new (mMemoryManager.allocate(MemoryManager::AllocationType::Pool,
                                     sizeof(RigidBody))) RigidBody(transform, *this, entity);
    assert(rigidBody != nullptr);

    BodyComponents::BodyComponent bodyComponent(rigidBody);
    mBodyComponents.addComponent(entity, false, bodyComponent);

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
             "Body " + std::to_string(entity.id) + ": New collision body created");

    // Return the pointer to the rigid body
    return rigidBody;
}

// Destroy a rigid body and all the joints which it belongs
/**
 * @param rigidBody Pointer to the body you want to destroy
 */
void DynamicsWorld::destroyRigidBody(RigidBody* rigidBody) {

    RP3D_LOG(mLogger, Logger::Level::Information, Logger::Category::Body,
             "Body " + std::to_string(rigidBody->getEntity().id) + ": rigid body destroyed");

    // Remove all the collision shapes of the body
    rigidBody->removeAllCollisionShapes();

    // Destroy all the joints in which the rigid body to be destroyed is involved
    JointListElement* element;
    for (element = rigidBody->mJointsList; element != nullptr; element = element->next) {
        destroyJoint(element->joint);
    }

    // Destroy the corresponding entity and its components
    mBodyComponents.removeComponent(rigidBody->getEntity());
    mTransformComponents.removeComponent(rigidBody->getEntity());
    mEntityManager.destroyEntity(rigidBody->getEntity());

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
             "Body " + std::to_string(joint->mBody1->getEntity().id) + ": Joint " + std::to_string(joint->getId()) +
             " added to body");

    // Add the joint at the beginning of the linked list of joints of the second body
    void* allocatedMemory2 = mMemoryManager.allocate(MemoryManager::AllocationType::Pool,
                                                     sizeof(JointListElement));
    JointListElement* jointListElement2 = new (allocatedMemory2) JointListElement(joint,
                                                                     joint->mBody2->mJointsList);
    joint->mBody2->mJointsList = jointListElement2;

    RP3D_LOG(mLogger, Logger::Level::Information, Logger::Category::Body,
             "Body " + std::to_string(joint->mBody2->getEntity().id) + ": Joint " + std::to_string(joint->getId()) +
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

// Compute the islands using potential contacts and joints
/// We compute the islands before creating the actual contacts here because we want all
/// the contact manifolds and contact points of the same island
/// to be packed together into linear arrays of manifolds and contacts for better caching.
/// An island is an isolated group of rigid bodies that have constraints (joints or contacts)
/// between each other. This method computes the islands at each time step as follows: For each
/// awake rigid body, we run a Depth First Search (DFS) through the constraint graph of that body
/// (graph where nodes are the bodies and where the edges are the constraints between the bodies) to
/// find all the bodies that are connected with it (the bodies that share joints or contacts with
/// it). Then, we create an island with this group of connected bodies.
void DynamicsWorld::createIslands() {

    // TODO : Check if we handle kinematic bodies correctly in islands creation

    // list of contact pairs involving a non-rigid body
    List<uint> nonRigidBodiesContactPairs(mMemoryManager.getSingleFrameAllocator());

    RP3D_PROFILE("DynamicsWorld::createIslands()", mProfiler);

    // Reset all the isAlreadyInIsland variables of bodies and joints
    for (uint b=0; b < mDynamicsComponents.getNbComponents(); b++) {

        mDynamicsComponents.mIsAlreadyInIsland[b] = false;
    }
    for (List<Joint*>::Iterator it = mJoints.begin(); it != mJoints.end(); ++it) {
        (*it)->mIsAlreadyInIsland = false;
    }

    // Create a stack for the bodies to visit during the Depth First Search
    Stack<Entity> bodyEntityIndicesToVisit(mMemoryManager.getSingleFrameAllocator());

    uint nbTotalManifolds = 0;

    // For each dynamic component
    // TODO : Here we iterate on dynamic component where we can have static, kinematic and dynamic bodies. Maybe we should
    //        not use a dynamic component for a static body.
    for (uint b=0; b < mDynamicsComponents.getNbEnabledComponents(); b++) {

        // If the body has already been added to an island, we go to the next body
        if (mDynamicsComponents.mIsAlreadyInIsland[b]) continue;

        // If the body is static, we go to the next body
        // TODO : Do not use pointer to rigid body here (maybe move getType() into a component)
        CollisionBody* body = static_cast<CollisionBody*>(mBodyComponents.getBody(mDynamicsComponents.mBodies[b]));
        if (body->getType() == BodyType::STATIC) continue;

        // Reset the stack of bodies to visit
        bodyEntityIndicesToVisit.clear();

        // Add the body into the stack of bodies to visit
        mDynamicsComponents.mIsAlreadyInIsland[b] = true;
        bodyEntityIndicesToVisit.push(mDynamicsComponents.mBodies[b]);

        // Create the new island
        uint32 islandIndex = mIslands.addIsland(nbTotalManifolds);

        // While there are still some bodies to visit in the stack
        while (bodyEntityIndicesToVisit.size() > 0) {

            // Get the next body to visit from the stack
            const Entity bodyToVisitEntity = bodyEntityIndicesToVisit.pop();

            // Add the body into the island
            mIslands.bodyEntities[islandIndex].add(bodyToVisitEntity);

            // If the current body is static, we do not want to perform the DFS
            // search across that body
            // TODO : Do not use pointer to rigid body here (maybe move getType() into a component)
            RigidBody* rigidBodyToVisit = static_cast<RigidBody*>(mBodyComponents.getBody(bodyToVisitEntity));

            // Awake the body if it is sleeping
            rigidBodyToVisit->setIsSleeping(false);

            if (rigidBodyToVisit->getType() == BodyType::STATIC) continue;

            // If the body is involved in contacts with other bodies
            auto itBodyContactPairs = mCollisionDetection.mMapBodyToContactPairs.find(bodyToVisitEntity);
            if (itBodyContactPairs != mCollisionDetection.mMapBodyToContactPairs.end()) {

                // For each contact pair in which the current body is involded
                List<uint>& contactPairs = itBodyContactPairs->second;
                for (uint p=0; p < contactPairs.size(); p++) {

                    ContactPair& pair = (*mCollisionDetection.mCurrentContactPairs)[contactPairs[p]];
                    assert(pair.potentialContactManifoldsIndices.size() > 0);

                    // Check if the current contact pair has already been added into an island
                    if (pair.isAlreadyInIsland) continue;

                    // Get the other body of the contact manifold
                    // TODO : Maybe avoid those casts here
                    RigidBody* body1 = dynamic_cast<RigidBody*>(mBodyComponents.getBody(pair.body1Entity));
                    RigidBody* body2 = dynamic_cast<RigidBody*>(mBodyComponents.getBody(pair.body2Entity));

                    // If the colliding body is a RigidBody (and not a CollisionBody instead)
                    if (body1 != nullptr && body2 != nullptr) {

                        nbTotalManifolds += pair.potentialContactManifoldsIndices.size();

                        // Add the pair into the list of pair to process to create contacts
                        mCollisionDetection.mContactPairsIndicesOrderingForContacts.add(pair.contactPairIndex);

                        // Add the contact manifold into the island
                        mIslands.nbContactManifolds[islandIndex] += pair.potentialContactManifoldsIndices.size();
                        pair.isAlreadyInIsland = true;

                        const Entity otherBodyEntity = pair.body1Entity == bodyToVisitEntity ? pair.body2Entity : pair.body1Entity;

                        // Check if the other body has already been added to the island
                        if (mDynamicsComponents.getIsAlreadyInIsland(otherBodyEntity)) continue;

                        // Insert the other body into the stack of bodies to visit
                        bodyEntityIndicesToVisit.push(otherBodyEntity);
                        mDynamicsComponents.setIsAlreadyInIsland(otherBodyEntity, true);
                    }
                    else {

                        // Add the contact pair index in the list of contact pairs that won't be part of islands
                        nonRigidBodiesContactPairs.add(pair.contactPairIndex);
                        pair.isAlreadyInIsland = true;
                    }
                }
            }

            // For each joint in which the current body is involved
            JointListElement* jointElement;
            for (jointElement = rigidBodyToVisit->mJointsList; jointElement != nullptr;
                 jointElement = jointElement->next) {

                Joint* joint = jointElement->joint;

                // Check if the current joint has already been added into an island
                if (joint->isAlreadyInIsland()) continue;

                // Add the joint into the island
                mIslands.joints[islandIndex].add(joint);
                joint->mIsAlreadyInIsland = true;

                const Entity body1Entity = joint->getBody1()->getEntity();
                const Entity body2Entity = joint->getBody2()->getEntity();
                const Entity otherBodyEntity = body1Entity == bodyToVisitEntity ? body2Entity : body1Entity;

                // Check if the other body has already been added to the island
                if (mDynamicsComponents.getIsAlreadyInIsland(otherBodyEntity)) continue;

                // Insert the other body into the stack of bodies to visit
                bodyEntityIndicesToVisit.push(otherBodyEntity);
                mDynamicsComponents.setIsAlreadyInIsland(otherBodyEntity, true);
            }
        }

        // Reset the isAlreadyIsland variable of the static bodies so that they
        // can also be included in the other islands
        for (uint j=0; j < mDynamicsComponents.getNbEnabledComponents(); j++) {

            // If the body is static, we go to the next body
            // TODO : Do not use pointer to rigid body here (maybe move getType() into a component)
            CollisionBody* body = static_cast<CollisionBody*>(mBodyComponents.getBody(mDynamicsComponents.mBodies[j]));
            if (body->getType() == BodyType::STATIC) {
                mDynamicsComponents.mIsAlreadyInIsland[j] = false;
            }
        }
    }

    // Add the contact pairs that are not part of islands at the end of the array of pairs for contacts creations
    mCollisionDetection.mContactPairsIndicesOrderingForContacts.addRange(nonRigidBodiesContactPairs);

    assert(mCollisionDetection.mCurrentContactPairs->size() == mCollisionDetection.mContactPairsIndicesOrderingForContacts.size());

    mCollisionDetection.mMapBodyToContactPairs.clear(true);
}

// Put bodies to sleep if needed.
/// For each island, if all the bodies have been almost still for a long enough period of
/// time, we put all the bodies of the island to sleep.
void DynamicsWorld::updateSleepingBodies() {

    RP3D_PROFILE("DynamicsWorld::updateSleepingBodies()", mProfiler);

    const decimal sleepLinearVelocitySquare = mSleepLinearVelocity * mSleepLinearVelocity;
    const decimal sleepAngularVelocitySquare = mSleepAngularVelocity * mSleepAngularVelocity;

    // For each island of the world
    for (uint i=0; i<mIslands.getNbIslands(); i++) {

        decimal minSleepTime = DECIMAL_LARGEST;

        // For each body of the island
        for (uint b=0; b < mIslands.bodyEntities[i].size(); b++) {

            const Entity bodyEntity = mIslands.bodyEntities[i][b];

            // TODO : We should not have to do this cast here to get type of body
            CollisionBody* body = static_cast<CollisionBody*>(mBodyComponents.getBody(bodyEntity));

            // Skip static bodies
            if (body->getType() == BodyType::STATIC) continue;

            // If the body is velocity is large enough to stay awake
            if (mDynamicsComponents.getLinearVelocity(bodyEntity).lengthSquare() > sleepLinearVelocitySquare ||
                mDynamicsComponents.getAngularVelocity(bodyEntity).lengthSquare() > sleepAngularVelocitySquare ||
                !body->isAllowedToSleep()) {

                // Reset the sleep time of the body
                body->mSleepTime = decimal(0.0);
                minSleepTime = decimal(0.0);
            }
            else {  // If the body velocity is below the sleeping velocity threshold

                // Increase the sleep time
                body->mSleepTime += mTimeStep;
                if (body->mSleepTime < minSleepTime) {
                    minSleepTime = body->mSleepTime;
                }
            }
        }

        // If the velocity of all the bodies of the island is under the
        // sleeping velocity threshold for a period of time larger than
        // the time required to become a sleeping body
        if (minSleepTime >= mTimeBeforeSleep) {

            // Put all the bodies of the island to sleep
            for (uint b=0; b < mIslands.bodyEntities[i].size(); b++) {

                const Entity bodyEntity = mIslands.bodyEntities[i][b];
                CollisionBody* body = static_cast<CollisionBody*>(mBodyComponents.getBody(bodyEntity));
                body->setIsSleeping(true);
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
