/********************************************************************************
* ReactPhysics3D physics library, http://www.reactphysics3d.com                 *
* Copyright (c) 2010-2022 Daniel Chappuis                                       *
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
#include <reactphysics3d/engine/PhysicsCommon.h>
#include <reactphysics3d/engine/PhysicsWorld.h>
#include <reactphysics3d/constraint/BallAndSocketJoint.h>
#include <reactphysics3d/constraint/SliderJoint.h>
#include <reactphysics3d/constraint/HingeJoint.h>
#include <reactphysics3d/constraint/FixedJoint.h>
#include <reactphysics3d/utils/Profiler.h>
#include <reactphysics3d/engine/EventListener.h>
#include <reactphysics3d/engine/Island.h>
#include <reactphysics3d/collision/ContactManifold.h>
#include <reactphysics3d/containers/Stack.h>

// Namespaces
using namespace reactphysics3d;
using namespace std;

// Static initializations

uint32 PhysicsWorld::mNbWorlds = 0;

// Constructor
/**
 * @param gravity Gravity vector in the world (in meters per second squared)
 * @param worldSettings The settings of the world
 * @param profiler Pointer to the profiler
 */
PhysicsWorld::PhysicsWorld(MemoryManager& memoryManager, PhysicsCommon& physicsCommon, const WorldSettings& worldSettings,
#ifdef IS_RP3D_PROFILING_ENABLED
                           Profiler* profiler)
#else
                           Profiler* /*profiler*/)
#endif
              : mMemoryManager(memoryManager), mConfig(worldSettings), mEntityManager(mMemoryManager.getHeapAllocator()), mDebugRenderer(mMemoryManager.getHeapAllocator()),
                mCollisionBodyComponents(mMemoryManager.getHeapAllocator()), mRigidBodyComponents(mMemoryManager.getHeapAllocator()),
                mTransformComponents(mMemoryManager.getHeapAllocator()), mCollidersComponents(mMemoryManager.getHeapAllocator()),
                mJointsComponents(mMemoryManager.getHeapAllocator()), mBallAndSocketJointsComponents(mMemoryManager.getHeapAllocator()),
                mFixedJointsComponents(mMemoryManager.getHeapAllocator()), mHingeJointsComponents(mMemoryManager.getHeapAllocator()),
                mSliderJointsComponents(mMemoryManager.getHeapAllocator()), mCollisionDetection(this, mCollidersComponents, mTransformComponents, mCollisionBodyComponents, mRigidBodyComponents,
                                        mMemoryManager, physicsCommon.mTriangleShapeHalfEdgeStructure),
                mCollisionBodies(mMemoryManager.getHeapAllocator()), mEventListener(nullptr),
                mName(worldSettings.worldName),  mIslands(mMemoryManager.getSingleFrameAllocator()), mProcessContactPairsOrderIslands(mMemoryManager.getSingleFrameAllocator()),
                mContactSolverSystem(mMemoryManager, *this, mIslands, mCollisionBodyComponents, mRigidBodyComponents,
                               mCollidersComponents, mConfig.restitutionVelocityThreshold),
                mConstraintSolverSystem(*this, mIslands, mRigidBodyComponents, mTransformComponents, mJointsComponents,
                                        mBallAndSocketJointsComponents, mFixedJointsComponents, mHingeJointsComponents,
                                        mSliderJointsComponents),
                mDynamicsSystem(*this, mCollisionBodyComponents, mRigidBodyComponents, mTransformComponents, mCollidersComponents, mIsGravityEnabled, mConfig.gravity),
                mNbVelocitySolverIterations(mConfig.defaultVelocitySolverNbIterations),
                mNbPositionSolverIterations(mConfig.defaultPositionSolverNbIterations), 
                mIsSleepingEnabled(mConfig.isSleepingEnabled), mRigidBodies(mMemoryManager.getPoolAllocator()),
                mIsGravityEnabled(true), mSleepLinearVelocity(mConfig.defaultSleepLinearVelocity),
                mSleepAngularVelocity(mConfig.defaultSleepAngularVelocity), mTimeBeforeSleep(mConfig.defaultTimeBeforeSleep) {

    // Automatically generate a name for the world
    if (mName == "") {

        std::stringstream ss;
        ss << "world";

        if (mNbWorlds > 0) {
            ss << mNbWorlds;
        }

        mName = ss.str();
    }

#ifdef IS_RP3D_PROFILING_ENABLED


    assert(profiler != nullptr);
    mProfiler = profiler;

    // Set the profiler
    mConstraintSolverSystem.setProfiler(mProfiler);
    mContactSolverSystem.setProfiler(mProfiler);
    mDynamicsSystem.setProfiler(mProfiler);
    mCollisionDetection.setProfiler(mProfiler);

#endif

    mNbWorlds++;

    RP3D_LOG(mConfig.worldName, Logger::Level::Information, Logger::Category::World,
             "Physics World: Physics world " + mName + " has been created",  __FILE__, __LINE__);
    RP3D_LOG(mConfig.worldName, Logger::Level::Information, Logger::Category::World,
             "Physics World: Initial world settings: " + worldSettings.to_string(),  __FILE__, __LINE__);

    RP3D_LOG(mConfig.worldName, Logger::Level::Information, Logger::Category::World,
             "Physics World: Physics world " + mName + " has been created",  __FILE__, __LINE__);

}

// Destructor
PhysicsWorld::~PhysicsWorld() {

    RP3D_LOG(mConfig.worldName, Logger::Level::Information, Logger::Category::World,
             "Physics World: Physics world " + mName + " has been destroyed",  __FILE__, __LINE__);

    // Destroy all the collision bodies that have not been removed
    uint32 i = static_cast<uint32>(mCollisionBodies.size());
    while (i != 0) {
        i--;
        destroyCollisionBody(mCollisionBodies[i]);
    }

#ifdef IS_RP3D_PROFILING_ENABLED


    // Print the profiling report into the destinations
    mProfiler->printReport();

#endif

    // Destroy all the joints that have not been removed
    for (uint32 i=0; i < mJointsComponents.getNbComponents(); i++) {
        destroyJoint(mJointsComponents.mJoints[i]);
    }

    // Destroy all the rigid bodies that have not been removed
    i = static_cast<uint32>(mRigidBodies.size());
    while (i != 0) {
        i--;
        destroyRigidBody(mRigidBodies[i]);
    }

    assert(mJointsComponents.getNbComponents() == 0);
    assert(mRigidBodies.size() == 0);
    assert(mCollisionBodies.size() == 0);
    assert(mCollisionBodyComponents.getNbComponents() == 0);
    assert(mTransformComponents.getNbComponents() == 0);
    assert(mCollidersComponents.getNbComponents() == 0);

    RP3D_LOG(mConfig.worldName, Logger::Level::Information, Logger::Category::World,
             "Physics World: Physics world " + mName + " has been destroyed",  __FILE__, __LINE__);
}

// Create a collision body and add it to the world
/**
 * @param transform Transformation mapping the local-space of the body to world-space
 * @return A pointer to the body that has been created in the world
 */
CollisionBody* PhysicsWorld::createCollisionBody(const Transform& transform) {

    // Create a new entity for the body
    Entity entity = mEntityManager.createEntity();

    // Check that the transform is valid
    if (!transform.isValid()) {
        RP3D_LOG(mConfig.worldName, Logger::Level::Error, Logger::Category::Body,
                 "Error when creating a collision body: the init transform is not valid",  __FILE__, __LINE__);
    }
    assert(transform.isValid());

    mTransformComponents.addComponent(entity, false, TransformComponents::TransformComponent(transform));

    // Create the collision body
    CollisionBody* collisionBody = new (mMemoryManager.allocate(MemoryManager::AllocationType::Pool,
                                        sizeof(CollisionBody)))
                                        CollisionBody(*this, entity);

    assert(collisionBody != nullptr);

    // Add the components
    CollisionBodyComponents::CollisionBodyComponent bodyComponent(collisionBody);
    mCollisionBodyComponents.addComponent(entity, false, bodyComponent);

    // Add the collision body to the world
    mCollisionBodies.add(collisionBody);

#ifdef IS_RP3D_PROFILING_ENABLED


    collisionBody->setProfiler(mProfiler);

#endif

    RP3D_LOG(mConfig.worldName, Logger::Level::Information, Logger::Category::Body,
             "Body " + std::to_string(entity.id) + ": New collision body created",  __FILE__, __LINE__);

    // Return the pointer to the rigid body
    return collisionBody;
}

// Destroy a collision body
/**
 * @param collisionBody Pointer to the body to destroy
 */
void PhysicsWorld::destroyCollisionBody(CollisionBody* collisionBody) {

    RP3D_LOG(mConfig.worldName, Logger::Level::Information, Logger::Category::Body,
             "Body " + std::to_string(collisionBody->getEntity().id) + ": collision body destroyed",  __FILE__, __LINE__);

    // Remove all the collision shapes of the body
    collisionBody->removeAllColliders();

    mCollisionBodyComponents.removeComponent(collisionBody->getEntity());
    mTransformComponents.removeComponent(collisionBody->getEntity());
    mEntityManager.destroyEntity(collisionBody->getEntity());

    // Call the destructor of the collision body
    collisionBody->~CollisionBody();

    // Remove the collision body from the array of bodies
    mCollisionBodies.remove(collisionBody);

    // Free the object from the memory allocator
    mMemoryManager.release(MemoryManager::AllocationType::Pool, collisionBody, sizeof(CollisionBody));
}

// Notify the world if a body is disabled (sleeping) or not
void PhysicsWorld::setBodyDisabled(Entity bodyEntity, bool isDisabled) {

    if (isDisabled == mCollisionBodyComponents.getIsEntityDisabled(bodyEntity)) return;

    // Notify all the components
    mCollisionBodyComponents.setIsEntityDisabled(bodyEntity, isDisabled);
    mTransformComponents.setIsEntityDisabled(bodyEntity, isDisabled);

    assert(mRigidBodyComponents.hasComponent(bodyEntity));

    mRigidBodyComponents.setIsEntityDisabled(bodyEntity, isDisabled);

    // For each collider of the body
    const Array<Entity>& collidersEntities = mCollisionBodyComponents.getColliders(bodyEntity);
    const uint32 nbColliderEntities = static_cast<uint32>(collidersEntities.size());
    for (uint32 i=0; i < nbColliderEntities; i++) {

        mCollidersComponents.setIsEntityDisabled(collidersEntities[i], isDisabled);
    }
}

// Notify the world whether a joint is disabled or not
void PhysicsWorld::setJointDisabled(Entity jointEntity, bool isDisabled) {

    if (isDisabled == mJointsComponents.getIsEntityDisabled(jointEntity)) return;

    mJointsComponents.setIsEntityDisabled(jointEntity, isDisabled);
    if (mBallAndSocketJointsComponents.hasComponent(jointEntity)) {
        mBallAndSocketJointsComponents.setIsEntityDisabled(jointEntity, isDisabled);
    }
    if (mFixedJointsComponents.hasComponent(jointEntity)) {
        mFixedJointsComponents.setIsEntityDisabled(jointEntity, isDisabled);
    }
    if (mHingeJointsComponents.hasComponent(jointEntity)) {
        mHingeJointsComponents.setIsEntityDisabled(jointEntity, isDisabled);
    }
    if (mSliderJointsComponents.hasComponent(jointEntity)) {
        mSliderJointsComponents.setIsEntityDisabled(jointEntity, isDisabled);
    }
}

// Return true if two bodies overlap
/// Use this method if you are not interested in contacts but if you simply want to know
/// if the two bodies overlap. If you want to get the contacts, you need to use the
/// testCollision() method instead.
/**
 * @param body1 Pointer to the first body
 * @param body2 Pointer to a second body
 * @return True if the two bodies overlap
 */
bool PhysicsWorld::testOverlap(CollisionBody* body1, CollisionBody* body2) {
    return mCollisionDetection.testOverlap(body1, body2);
}

// Return the current world-space AABB of given collider
/**
 * @param collider Pointer to a collider
 * @return The AAABB of the collider in world-space
 */
AABB PhysicsWorld::getWorldAABB(const Collider* collider) const {

    if (collider->getBroadPhaseId() == -1) {
        return AABB();
    }

   return mCollisionDetection.getWorldAABB(collider);
}

// Update the physics simulation
/**
 * @param timeStep The amount of time to step the simulation by (in seconds)
 */
void PhysicsWorld::update(decimal timeStep) {

#ifdef IS_RP3D_PROFILING_ENABLED

    // Increment the frame counter of the profiler
    mProfiler->incrementFrameCounter();
#endif

    RP3D_PROFILE("PhysicsWorld::update()", mProfiler);

    // Reset the debug renderer
    if (mIsDebugRenderingEnabled) {
        mDebugRenderer.reset();
    }

    // Compute the collision detection
    mCollisionDetection.computeCollisionDetection();

    // Create the islands
    createIslands();

    // Create the actual narrow-phase contacts
    mCollisionDetection.createContacts();

    // Report the contacts to the user
    mCollisionDetection.reportContactsAndTriggers();

    // Recompute the inverse inertia tensors of rigid bodies
    updateBodiesInverseWorldInertiaTensors();

    // Enable or disable the joints
    enableDisableJoints();

    // Integrate the velocities
    mDynamicsSystem.integrateRigidBodiesVelocities(timeStep);

    // Solve the contacts and constraints
    solveContactsAndConstraints(timeStep);

    // Integrate the position and orientation of each body
    mDynamicsSystem.integrateRigidBodiesPositions(timeStep, mContactSolverSystem.isSplitImpulseActive());

    // Solve the position correction for constraints
    solvePositionCorrection();

    // Update the state (positions and velocities) of the bodies
    mDynamicsSystem.updateBodiesState();

    // Update the colliders components
    mCollisionDetection.updateColliders();

    if (mIsSleepingEnabled) updateSleepingBodies(timeStep);

    // Reset the external force and torque applied to the bodies
    mDynamicsSystem.resetBodiesForceAndTorque();

    // Reset the islands
    mIslands.clear();

    mProcessContactPairsOrderIslands.clear(true);

    // Generate debug rendering primitives (if enabled)
    if (mIsDebugRenderingEnabled) {
        mDebugRenderer.computeDebugRenderingPrimitives(*this);
    }

    // Reset the single frame memory allocator
    mMemoryManager.resetFrameAllocator();
}

// Update the world inverse inertia tensors of rigid bodies
void PhysicsWorld::updateBodiesInverseWorldInertiaTensors() {

    uint32 nbComponents = mRigidBodyComponents.getNbEnabledComponents();
    for (uint32 i=0; i < nbComponents; i++) {
        const Matrix3x3 orientation = mTransformComponents.getTransform(mRigidBodyComponents.mBodiesEntities[i]).getOrientation().getMatrix();

        RigidBody::computeWorldInertiaTensorInverse(orientation, mRigidBodyComponents.mInverseInertiaTensorsLocal[i], mRigidBodyComponents.mInverseInertiaTensorsWorld[i]);
    }
}

// Solve the contacts and constraints
void PhysicsWorld::solveContactsAndConstraints(decimal timeStep) {

    RP3D_PROFILE("PhysicsWorld::solveContactsAndConstraints()", mProfiler);

    // ---------- Solve velocity constraints for joints and contacts ---------- //

    // Initialize the contact solver
    mContactSolverSystem.init(mCollisionDetection.mCurrentContactManifolds, mCollisionDetection.mCurrentContactPoints, timeStep);

    // Initialize the constraint solver
    mConstraintSolverSystem.initialize(timeStep);

    // For each iteration of the velocity solver
    for (uint32 i=0; i<mNbVelocitySolverIterations; i++) {

        mConstraintSolverSystem.solveVelocityConstraints();

        mContactSolverSystem.solve();
    }

    mContactSolverSystem.storeImpulses();

    // Reset the contact solver
    mContactSolverSystem.reset();
}

// Solve the position error correction of the constraints
void PhysicsWorld::solvePositionCorrection() {

    RP3D_PROFILE("PhysicsWorld::solvePositionCorrection()", mProfiler);

    // ---------- Solve the position error correction for the constraints ---------- //

    // For each iteration of the position (error correction) solver
    for (uint32 i=0; i<mNbPositionSolverIterations; i++) {

        // Solve the position constraints
        mConstraintSolverSystem.solvePositionConstraints();
    }
}

// Enable or disable the joints
void PhysicsWorld::enableDisableJoints() {

    const uint32 nbJointComponents = mJointsComponents.getNbComponents();

    Array<Entity> jointsEntites(mMemoryManager.getHeapAllocator(), nbJointComponents);

    // Get all the joints entities
    for (uint32 i = 0; i < nbJointComponents; i++) {
        jointsEntites.add(mJointsComponents.mJointEntities[i]);
    }

    // For each joint
    for (uint32 i = 0; i < nbJointComponents; i++) {

        uint32 jointEntityIndex = mJointsComponents.getEntityIndex(jointsEntites[i]);

        Entity body1 = mJointsComponents.mBody1Entities[jointEntityIndex];
        Entity body2 = mJointsComponents.mBody2Entities[jointEntityIndex];

        // If both bodies of the joint are disabled
        if (mCollisionBodyComponents.getIsEntityDisabled(body1) || mCollisionBodyComponents.getIsEntityDisabled(body2)) {

            // Disable the joint
            setJointDisabled(jointsEntites[i], true);
        }
        else {

            // Enable the joint
            setJointDisabled(jointsEntites[i], false);
        }
    }
}

// Create a rigid body into the physics world
/**
 * @param transform Transformation from body local-space to world-space
 * @return A pointer to the body that has been created in the world
 */
RigidBody* PhysicsWorld::createRigidBody(const Transform& transform) {

    // Create a new entity for the body
    Entity entity = mEntityManager.createEntity();

    // Check that the transform is valid
    if (!transform.isValid()) {
        RP3D_LOG(mConfig.worldName, Logger::Level::Error, Logger::Category::Body,
                 "Error when creating a rigid body: the init transform is not valid",  __FILE__, __LINE__);
    }
    assert(transform.isValid());

    mTransformComponents.addComponent(entity, false, TransformComponents::TransformComponent(transform));

    // Create the rigid body
    RigidBody* rigidBody = new (mMemoryManager.allocate(MemoryManager::AllocationType::Pool,
                                     sizeof(RigidBody))) RigidBody(*this, entity);
    assert(rigidBody != nullptr);

    CollisionBodyComponents::CollisionBodyComponent bodyComponent(rigidBody);
    mCollisionBodyComponents.addComponent(entity, false, bodyComponent);

    RigidBodyComponents::RigidBodyComponent rigidBodyComponent(rigidBody, BodyType::DYNAMIC, transform.getPosition());
    mRigidBodyComponents.addComponent(entity, false, rigidBodyComponent);

    // Compute the inverse mass
    mRigidBodyComponents.setMassInverse(entity, decimal(1.0) / mRigidBodyComponents.getMass(entity));

    // Add the rigid body to the physics world
    mRigidBodies.add(rigidBody);

#ifdef IS_RP3D_PROFILING_ENABLED

    rigidBody->setProfiler(mProfiler);
#endif

    RP3D_LOG(mConfig.worldName, Logger::Level::Information, Logger::Category::Body,
             "Body " + std::to_string(entity.id) + ": New collision body created",  __FILE__, __LINE__);

    // Return the pointer to the rigid body
    return rigidBody;
}

// Destroy a rigid body and all the joints which it belongs
/**
 * @param rigidBody Pointer to the body you want to destroy
 */
void PhysicsWorld::destroyRigidBody(RigidBody* rigidBody) {

    RP3D_LOG(mConfig.worldName, Logger::Level::Information, Logger::Category::Body,
             "Body " + std::to_string(rigidBody->getEntity().id) + ": rigid body destroyed",  __FILE__, __LINE__);

    // Remove all the collision shapes of the body
    rigidBody->removeAllColliders();

    // Destroy all the joints in which the rigid body to be destroyed is involved
    const Array<Entity>& joints = mRigidBodyComponents.getJoints(rigidBody->getEntity());
    while (joints.size() > 0) {
        destroyJoint(mJointsComponents.getJoint(joints[0]));
    }

    // Destroy the corresponding entity and its components
    mCollisionBodyComponents.removeComponent(rigidBody->getEntity());
    mRigidBodyComponents.removeComponent(rigidBody->getEntity());
    mTransformComponents.removeComponent(rigidBody->getEntity());
    mEntityManager.destroyEntity(rigidBody->getEntity());

    // Call the destructor of the rigid body
    rigidBody->~RigidBody();

    // Remove the rigid body from the array of rigid bodies
    mRigidBodies.remove(rigidBody);

    // Free the object from the memory allocator
    mMemoryManager.release(MemoryManager::AllocationType::Pool, rigidBody, sizeof(RigidBody));
}

// Create a joint between two bodies in the world and return a pointer to the new joint
/**
 * @param jointInfo The information that is necessary to create the joint
 * @return A pointer to the joint that has been created in the world
 */
Joint* PhysicsWorld::createJoint(const JointInfo& jointInfo) {

    // Create a new entity for the joint
    Entity entity = mEntityManager.createEntity();

    Joint* newJoint = nullptr;

    const bool isJointDisabled = mRigidBodyComponents.getIsEntityDisabled(jointInfo.body1->getEntity()) &&
                                 mRigidBodyComponents.getIsEntityDisabled(jointInfo.body2->getEntity());

    // Allocate memory to create the new joint
    switch(jointInfo.type) {

        // Ball-and-Socket joint
        case JointType::BALLSOCKETJOINT:
        {
            // Create a BallAndSocketJoint component
            BallAndSocketJointComponents::BallAndSocketJointComponent ballAndSocketJointComponent(false, PI_RP3D);
            mBallAndSocketJointsComponents.addComponent(entity, isJointDisabled, ballAndSocketJointComponent);

            void* allocatedMemory = mMemoryManager.allocate(MemoryManager::AllocationType::Pool,
                                                            sizeof(BallAndSocketJoint));
            const BallAndSocketJointInfo& info = static_cast<const BallAndSocketJointInfo&>(jointInfo);
            BallAndSocketJoint* joint = new (allocatedMemory) BallAndSocketJoint(entity, *this, info);

            newJoint = joint;
            mBallAndSocketJointsComponents.setJoint(entity, joint);
            break;
        }

        // Slider joint
        case JointType::SLIDERJOINT:
        {
            const SliderJointInfo& info = static_cast<const SliderJointInfo&>(jointInfo);

            // Create a SliderJoint component
            SliderJointComponents::SliderJointComponent sliderJointComponent(info.isLimitEnabled, info.isMotorEnabled,
                                                                             info.minTranslationLimit, info.maxTranslationLimit,
                                                                             info.motorSpeed, info.maxMotorForce);
            mSliderJointsComponents.addComponent(entity, isJointDisabled, sliderJointComponent);

            void* allocatedMemory = mMemoryManager.allocate(MemoryManager::AllocationType::Pool, sizeof(SliderJoint));
            SliderJoint* joint = new (allocatedMemory) SliderJoint(entity, *this, info);

            newJoint = joint;
            mSliderJointsComponents.setJoint(entity, joint);

            break;
        }

        // Hinge joint
        case JointType::HINGEJOINT:
        {
            const HingeJointInfo& info = static_cast<const HingeJointInfo&>(jointInfo);

            // Create a HingeJoint component
            HingeJointComponents::HingeJointComponent hingeJointComponent(info.isLimitEnabled, info.isMotorEnabled,
                                                                          info.minAngleLimit, info.maxAngleLimit,
                                                                          info.motorSpeed, info.maxMotorTorque);
            mHingeJointsComponents.addComponent(entity, isJointDisabled, hingeJointComponent);

            void* allocatedMemory = mMemoryManager.allocate(MemoryManager::AllocationType::Pool,
                                                            sizeof(HingeJoint));
            HingeJoint* joint = new (allocatedMemory) HingeJoint(entity, *this, info);

            newJoint = joint;
            mHingeJointsComponents.setJoint(entity, joint);
            break;
        }

        // Fixed joint
        case JointType::FIXEDJOINT:
        {
            // Create a BallAndSocketJoint component
            FixedJointComponents::FixedJointComponent fixedJointComponent;
            mFixedJointsComponents.addComponent(entity, isJointDisabled, fixedJointComponent);

            void* allocatedMemory = mMemoryManager.allocate(MemoryManager::AllocationType::Pool,
                                                            sizeof(FixedJoint));
            const FixedJointInfo& info = static_cast<const FixedJointInfo&>(jointInfo);
            FixedJoint* joint = new (allocatedMemory) FixedJoint(entity, *this, info);

            newJoint = joint;

            mFixedJointsComponents.setJoint(entity, joint);

            break;
        }

        default:
        {
            assert(false);
            return nullptr;
        }
    }

    JointComponents::JointComponent jointComponent(jointInfo.body1->getEntity(), jointInfo.body2->getEntity(), newJoint, jointInfo.type,
                                                   jointInfo.positionCorrectionTechnique, jointInfo.isCollisionEnabled);
    mJointsComponents.addComponent(entity, isJointDisabled, jointComponent);

    // If the collision between the two bodies of the constraint is disabled
    if (!jointInfo.isCollisionEnabled) {

        // Add the pair of bodies in the set of body pairs that cannot collide with each other
        mCollisionDetection.addNoCollisionPair(jointInfo.body1->getEntity(), jointInfo.body2->getEntity());
    }

    RP3D_LOG(mConfig.worldName, Logger::Level::Information, Logger::Category::Joint,
             "Joint " + std::to_string(newJoint->getEntity().id) + ": New joint created",  __FILE__, __LINE__);
    RP3D_LOG(mConfig.worldName, Logger::Level::Information, Logger::Category::Joint,
             "Joint " + std::to_string(newJoint->getEntity().id) + ": " + newJoint->to_string(),  __FILE__, __LINE__);

    // Add the joint into the joint array of the bodies involved in the joint
    addJointToBodies(jointInfo.body1->getEntity(), jointInfo.body2->getEntity(), entity);

    // Return the pointer to the created joint
    return newJoint;
}

// Destroy a joint
/**
 * @param joint Pointer to the joint you want to destroy
 */
void PhysicsWorld::destroyJoint(Joint* joint) {

    assert(joint != nullptr);

    RP3D_LOG(mConfig.worldName, Logger::Level::Information, Logger::Category::Joint,
             "Joint " + std::to_string(joint->getEntity().id) + ": joint destroyed",  __FILE__, __LINE__);

    // If the collision between the two bodies of the constraint was disabled
    if (!joint->isCollisionEnabled()) {

        // Remove the pair of bodies from the set of body pairs that cannot collide with each other
        mCollisionDetection.removeNoCollisionPair(joint->getBody1()->getEntity(), joint->getBody2()->getEntity());
    }

    RigidBody* body1 = joint->getBody1();
    RigidBody* body2 = joint->getBody2();

    // Wake up the two bodies of the joint
    body1->setIsSleeping(false);
    body2->setIsSleeping(false);

    // Remove the joint from the joint array of the bodies involved in the joint
    mRigidBodyComponents.removeJointFromBody(body1->getEntity(), joint->getEntity());
    mRigidBodyComponents.removeJointFromBody(body2->getEntity(), joint->getEntity());

    size_t nbBytes = joint->getSizeInBytes();

    Entity jointEntity = joint->getEntity();

    // Destroy the corresponding entity and its components
    mJointsComponents.removeComponent(jointEntity);
    if (mBallAndSocketJointsComponents.hasComponent(jointEntity)) {
        mBallAndSocketJointsComponents.removeComponent(jointEntity);
    }
    if (mFixedJointsComponents.hasComponent(jointEntity)) {
        mFixedJointsComponents.removeComponent(jointEntity);
    }
    if (mHingeJointsComponents.hasComponent(jointEntity)) {
        mHingeJointsComponents.removeComponent(jointEntity);
    }
    if (mSliderJointsComponents.hasComponent(jointEntity)) {
        mSliderJointsComponents.removeComponent(jointEntity);
    }
    mEntityManager.destroyEntity(jointEntity);

    // Call the destructor of the joint
    joint->~Joint();

    // Release the allocated memory
    mMemoryManager.release(MemoryManager::AllocationType::Pool, joint, nbBytes);
}


// Set the number of iterations for the velocity constraint solver
/**
 * @param nbIterations Number of iterations for the velocity solver
 */
void PhysicsWorld::setNbIterationsVelocitySolver(uint16 nbIterations) {

    mNbVelocitySolverIterations = nbIterations;

    RP3D_LOG(mConfig.worldName, Logger::Level::Information, Logger::Category::World,
             "Physics World: Set nb iterations velocity solver to " + std::to_string(nbIterations),  __FILE__, __LINE__);
}

// Add the joint to the array of joints of the two bodies involved in the joint
void PhysicsWorld::addJointToBodies(Entity body1, Entity body2, Entity joint) {

    mRigidBodyComponents.addJointToBody(body1, joint);

    RP3D_LOG(mConfig.worldName, Logger::Level::Information, Logger::Category::Body,
             "Body " + std::to_string(body1.id) + ": Joint " + std::to_string(joint.id) + " added to body",  __FILE__, __LINE__);

    mRigidBodyComponents.addJointToBody(body2, joint);

    RP3D_LOG(mConfig.worldName, Logger::Level::Information, Logger::Category::Body,
             "Body " + std::to_string(body2.id) + ": Joint " + std::to_string(joint.id) + " added to body",  __FILE__, __LINE__);
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
void PhysicsWorld::createIslands() {

    RP3D_PROFILE("PhysicsWorld::createIslands()", mProfiler);

    assert(mProcessContactPairsOrderIslands.size() == 0);

    // Reset all the isAlreadyInIsland variables of bodies and joints
    const uint32 nbRigidBodyComponents = mRigidBodyComponents.getNbComponents();
    for (uint32 b=0; b < nbRigidBodyComponents; b++) {
        mRigidBodyComponents.mIsAlreadyInIsland[b] = false;
    }
    const uint32 nbJointsComponents = mJointsComponents.getNbComponents();
    for (uint32 i=0; i < nbJointsComponents; i++) {
        mJointsComponents.mIsAlreadyInIsland[i] = false;
    }

    // Reserve memory for the islands
    mIslands.reserveMemory();

    // Create a stack for the bodies to visit during the Depth First Search
    Stack<Entity> bodyEntitiesToVisit(mMemoryManager.getSingleFrameAllocator(), mIslands.getNbMaxBodiesInIslandPreviousFrame());

    // Array of static bodies added to the current island (used to reset the isAlreadyInIsland variable of static bodies)
    Array<Entity> staticBodiesAddedToIsland(mMemoryManager.getSingleFrameAllocator(), 16);

    uint32 nbTotalManifolds = 0;

    // For each rigid body component
    for (uint32 b=0; b < mRigidBodyComponents.getNbEnabledComponents(); b++) {

        // If the body has already been added to an island, we go to the next body
        if (mRigidBodyComponents.mIsAlreadyInIsland[b]) continue;

        // If the body is static, we go to the next body
        if (mRigidBodyComponents.mBodyTypes[b] == BodyType::STATIC) continue;

        // Reset the stack of bodies to visit
        bodyEntitiesToVisit.clear();

        // Add the body into the stack of bodies to visit
        mRigidBodyComponents.mIsAlreadyInIsland[b] = true;
        bodyEntitiesToVisit.push(mRigidBodyComponents.mBodiesEntities[b]);

        // Create the new island
        uint32 islandIndex = mIslands.addIsland(nbTotalManifolds);

        // While there are still some bodies to visit in the stack
        while (bodyEntitiesToVisit.size() > 0) {

            // Get the body entity
            const Entity bodyToVisitEntity = bodyEntitiesToVisit.pop();

            // Add the body into the island
            mIslands.addBodyToIsland(bodyToVisitEntity);

            RigidBody* rigidBodyToVisit = mRigidBodyComponents.getRigidBody(bodyToVisitEntity);

            // Awake the body if it is sleeping (note that this called might change the body index in the mRigidBodyComponents array)
            rigidBodyToVisit->setIsSleeping(false);

            // Compute the body index in the array (Note that it could have changed because of the previous call to rigidBodyToVisit->setIsSleeping(false))
            const uint32 bodyToVisitIndex = mRigidBodyComponents.getEntityIndex(bodyToVisitEntity);

            // If the current body is static, we do not want to perform the DFS search across that body
            if (mRigidBodyComponents.mBodyTypes[bodyToVisitIndex] == BodyType::STATIC) {

                staticBodiesAddedToIsland.add(bodyToVisitEntity);

                // Go to the next body
                continue;
            }

            // If the body is involved in contacts with other bodies
            // For each contact pair in which the current body is involded
            const uint32 nbBodyContactPairs = static_cast<uint32>(mRigidBodyComponents.mContactPairs[bodyToVisitIndex].size());
            for (uint32 p=0; p < nbBodyContactPairs; p++) {

                const uint32 contactPairIndex = mRigidBodyComponents.mContactPairs[bodyToVisitIndex][p];
                ContactPair& pair = (*mCollisionDetection.mCurrentContactPairs)[contactPairIndex];

                // Check if the current contact pair has already been added into an island
                if (pair.isAlreadyInIsland) continue;

                const Entity otherBodyEntity = pair.body1Entity == bodyToVisitEntity ? pair.body2Entity : pair.body1Entity;

                // If the colliding body is a RigidBody (and not a CollisionBody) and is not a trigger
                uint32 otherBodyIndex;
                if (mRigidBodyComponents.hasComponentGetIndex(otherBodyEntity, otherBodyIndex)
                    && !mCollidersComponents.getIsTrigger(pair.collider1Entity) && !mCollidersComponents.getIsTrigger(pair.collider2Entity)) {

                    mProcessContactPairsOrderIslands.add(contactPairIndex);

                    assert(pair.nbPotentialContactManifolds > 0);
                    nbTotalManifolds += pair.nbPotentialContactManifolds;

                    // Add the contact manifold into the island
                    mIslands.nbContactManifolds[islandIndex] += pair.nbPotentialContactManifolds;
                    pair.isAlreadyInIsland = true;

                    // Check if the other body has already been added to the island
                    if (mRigidBodyComponents.mIsAlreadyInIsland[otherBodyIndex]) continue;

                    // Insert the other body into the stack of bodies to visit
                    bodyEntitiesToVisit.push(otherBodyEntity);
                    mRigidBodyComponents.mIsAlreadyInIsland[otherBodyIndex] = true;
                }
                else {

                    // Add the contact pair index in the array of contact pairs that won't be part of islands
                    pair.isAlreadyInIsland = true;
                }
            }

            // For each joint in which the current body is involved
            const Array<Entity>& joints = mRigidBodyComponents.getJoints(rigidBodyToVisit->getEntity());
            const uint32 nbBodyJoints = static_cast<uint32>(joints.size());
            for (uint32 i=0; i < nbBodyJoints; i++) {

                const uint32 jointComponentIndex = mJointsComponents.getEntityIndex(joints[i]);

                // Check if the current joint has already been added into an island
                if (mJointsComponents.mIsAlreadyInIsland[jointComponentIndex]) continue;

                // Add the joint into the island
                mJointsComponents.mIsAlreadyInIsland[jointComponentIndex] = true;

                const Entity body1Entity = mJointsComponents.mBody1Entities[jointComponentIndex];
                const Entity body2Entity = mJointsComponents.mBody2Entities[jointComponentIndex];
                const Entity otherBodyEntity = body1Entity == bodyToVisitEntity ? body2Entity : body1Entity;

                const uint32 otherBodyIndex = mRigidBodyComponents.getEntityIndex(otherBodyEntity);

                // Check if the other body has already been added to the island
                if (mRigidBodyComponents.mIsAlreadyInIsland[otherBodyIndex]) continue;

                // Insert the other body into the stack of bodies to visit
                bodyEntitiesToVisit.push(otherBodyEntity);
                mRigidBodyComponents.mIsAlreadyInIsland[otherBodyIndex] = true;
            }
        }

        // Reset the isAlreadyIsland variable of the static bodies so that they
        // can also be included in the other islands
        const uint32 nbStaticBodiesAddedToIsland = static_cast<uint32>(staticBodiesAddedToIsland.size());
        for (uint32 j=0; j < nbStaticBodiesAddedToIsland; j++) {

            assert(mRigidBodyComponents.getBodyType(staticBodiesAddedToIsland[j]) == BodyType::STATIC);
            mRigidBodyComponents.setIsAlreadyInIsland(staticBodiesAddedToIsland[j], false);
        }

        staticBodiesAddedToIsland.clear();
    }

    // Clear the associated contacts pairs of rigid bodies
    const uint32 nbRigidBodyEnabledComponents = mRigidBodyComponents.getNbEnabledComponents();
    for (uint32 b=0; b < nbRigidBodyEnabledComponents; b++) {
        mRigidBodyComponents.mContactPairs[b].clear();
    }
}

// Put bodies to sleep if needed.
/// For each island, if all the bodies have been almost still for a long enough period of
/// time, we put all the bodies of the island to sleep.
void PhysicsWorld::updateSleepingBodies(decimal timeStep) {

    RP3D_PROFILE("PhysicsWorld::updateSleepingBodies()", mProfiler);

    const decimal sleepLinearVelocitySquare = mSleepLinearVelocity * mSleepLinearVelocity;
    const decimal sleepAngularVelocitySquare = mSleepAngularVelocity * mSleepAngularVelocity;

    // For each island of the world
    const uint32 nbIslands = mIslands.getNbIslands();
    for (uint32 i=0; i < nbIslands; i++) {

        decimal minSleepTime = DECIMAL_LARGEST;

        // For each body of the island
        for (uint32 b=0; b < mIslands.nbBodiesInIsland[i]; b++) {

            const Entity bodyEntity = mIslands.bodyEntities[mIslands.startBodyEntitiesIndex[i] + b];
            const uint32 bodyIndex = mRigidBodyComponents.getEntityIndex(bodyEntity);

            // Skip static bodies
            if (mRigidBodyComponents.mBodyTypes[bodyIndex] == BodyType::STATIC) continue;

            // If the body is velocity is large enough to stay awake
            if (mRigidBodyComponents.mLinearVelocities[bodyIndex].lengthSquare() > sleepLinearVelocitySquare ||
                mRigidBodyComponents.mAngularVelocities[bodyIndex].lengthSquare() > sleepAngularVelocitySquare ||
                !mRigidBodyComponents.mIsAllowedToSleep[bodyIndex]) {

                // Reset the sleep time of the body
                mRigidBodyComponents.mSleepTimes[bodyIndex] = decimal(0.0);
                minSleepTime = decimal(0.0);
            }
            else {  // If the body velocity is below the sleeping velocity threshold

                // Increase the sleep time
                mRigidBodyComponents.mSleepTimes[bodyIndex] += timeStep;
                if (mRigidBodyComponents.mSleepTimes[bodyIndex] < minSleepTime) {
                    minSleepTime = mRigidBodyComponents.mSleepTimes[bodyIndex];
                }
            }
        }

        // If the velocity of all the bodies of the island is under the
        // sleeping velocity threshold for a period of time larger than
        // the time required to become a sleeping body
        if (minSleepTime >= mTimeBeforeSleep) {

            // Put all the bodies of the island to sleep
            for (uint32 b=0; b < mIslands.nbBodiesInIsland[i]; b++) {

                const Entity bodyEntity = mIslands.bodyEntities[mIslands.startBodyEntitiesIndex[i] + b];
                RigidBody* body = mRigidBodyComponents.getRigidBody(bodyEntity);
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
void PhysicsWorld::enableSleeping(bool isSleepingEnabled) {
    mIsSleepingEnabled = isSleepingEnabled;

    if (!mIsSleepingEnabled) {

        // For each body of the world
        Array<RigidBody*>::Iterator it;
        for (it = mRigidBodies.begin(); it != mRigidBodies.end(); ++it) {

            // Wake up the rigid body
            (*it)->setIsSleeping(false);
        }
    }

    RP3D_LOG(mConfig.worldName, Logger::Level::Information, Logger::Category::World,
             "Physics World: isSleepingEnabled=" + (isSleepingEnabled ? std::string("true") : std::string("false")) ,  __FILE__, __LINE__);
}

// Set the number of iterations for the position constraint solver
/**
 * @param nbIterations Number of iterations for the position solver
 */
void PhysicsWorld::setNbIterationsPositionSolver(uint32 nbIterations) {

    mNbPositionSolverIterations = nbIterations;

    RP3D_LOG(mConfig.worldName, Logger::Level::Information, Logger::Category::World,
             "Physics World: Set nb iterations position solver to " + std::to_string(nbIterations),  __FILE__, __LINE__);
}

// Set the gravity vector of the world
/**
 * @param gravity The gravity vector (in meter per seconds squared)
 */
void PhysicsWorld::setGravity(const Vector3& gravity) {

    mConfig.gravity = gravity;

    RP3D_LOG(mConfig.worldName, Logger::Level::Information, Logger::Category::World,
             "Physics World: Set gravity vector to " + gravity.to_string(),  __FILE__, __LINE__);
}

// Set the sleep linear velocity.
/// When the velocity of a body becomes smaller than the sleep linear/angular
/// velocity for a given amount of time, the body starts sleeping and does not need
/// to be simulated anymore.
/**
 * @param sleepLinearVelocity The sleep linear velocity (in meters per second)
 */
void PhysicsWorld::setSleepLinearVelocity(decimal sleepLinearVelocity) {
    assert(sleepLinearVelocity >= decimal(0.0));
    mSleepLinearVelocity = sleepLinearVelocity;

    RP3D_LOG(mConfig.worldName, Logger::Level::Information, Logger::Category::World,
             "Physics World: sleepLinearVelocity= " + std::to_string(sleepLinearVelocity),  __FILE__, __LINE__);
}

// Set the sleep angular velocity.
/// When the velocity of a body becomes smaller than the sleep linear/angular
/// velocity for a given amount of time, the body starts sleeping and does not need
/// to be simulated anymore.
/**
 * @param sleepAngularVelocity The sleep angular velocity (in radian per second)
 */
void PhysicsWorld::setSleepAngularVelocity(decimal sleepAngularVelocity) {
    assert(sleepAngularVelocity >= decimal(0.0));
    mSleepAngularVelocity = sleepAngularVelocity;

    RP3D_LOG(mConfig.worldName, Logger::Level::Information, Logger::Category::World,
             "Physics World: sleepAngularVelocity= " + std::to_string(sleepAngularVelocity),  __FILE__, __LINE__);
}

// Set the time a body is required to stay still before sleeping
/**
 * @param timeBeforeSleep Time a body is required to stay still before sleeping (in seconds)
 */
void PhysicsWorld::setTimeBeforeSleep(decimal timeBeforeSleep) {
    assert(timeBeforeSleep >= decimal(0.0));
    mTimeBeforeSleep = timeBeforeSleep;

    RP3D_LOG(mConfig.worldName, Logger::Level::Information, Logger::Category::World,
             "Physics World: timeBeforeSleep= " + std::to_string(timeBeforeSleep),  __FILE__, __LINE__);
}

// Enable/Disable the gravity
/**
 * @param isGravityEnabled True if you want to enable the gravity in the world
 *                         and false otherwise
 */
void PhysicsWorld::setIsGravityEnabled(bool isGravityEnabled) {
    mIsGravityEnabled = isGravityEnabled;

    RP3D_LOG(mConfig.worldName, Logger::Level::Information, Logger::Category::World,
             "Physics World: isGravityEnabled= " + (isGravityEnabled ? std::string("true") : std::string("false")),  __FILE__, __LINE__);
}

// Return a constant pointer to a given CollisionBody of the world
/**
 * @param index Index of a CollisionBody in the world
 * @return Constant pointer to a given CollisionBody
 */
const CollisionBody* PhysicsWorld::getCollisionBody(uint32 index) const {

    if (index >= getNbCollisionBodies()) {

        RP3D_LOG(mConfig.worldName, Logger::Level::Error, Logger::Category::World,
                 "Error when getting collision body: index is out of bounds",  __FILE__, __LINE__);
    }

    assert(index < mCollisionBodies.size());

    return mCollisionBodies[index];
}

// Return a pointer to a given CollisionBody of the world
/**
 * @param index Index of a CollisionBody in the world
 * @return Pointer to a given CollisionBody
 */
CollisionBody* PhysicsWorld::getCollisionBody(uint32 index) {

    if (index >= getNbCollisionBodies()) {

        RP3D_LOG(mConfig.worldName, Logger::Level::Error, Logger::Category::World,
                 "Error when getting collision body: index is out of bounds",  __FILE__, __LINE__);
    }

    assert(index < mCollisionBodies.size());

    return mCollisionBodies[index];
}

// Return a constant pointer to a given RigidBody of the world
/**
 * @param index Index of a RigidBody in the world
 * @return Constant pointer to a given RigidBody
 */
const RigidBody* PhysicsWorld::getRigidBody(uint32 index) const {

    if (index >= getNbRigidBodies()) {

        RP3D_LOG(mConfig.worldName, Logger::Level::Error, Logger::Category::World,
                 "Error when getting rigid body: index is out of bounds",  __FILE__, __LINE__);
    }

    assert(index < mRigidBodies.size());

    return mRigidBodies[index];
}

// Return a pointer to a given RigidBody of the world
/**
 * @param index Index of a RigidBody in the world
 * @return Pointer to a given RigidBody
 */
RigidBody* PhysicsWorld::getRigidBody(uint32 index) {

    if (index >= getNbRigidBodies()) {

        RP3D_LOG(mConfig.worldName, Logger::Level::Error, Logger::Category::World,
                 "Error when getting rigid body: index is out of bounds",  __FILE__, __LINE__);
    }

    assert(index < mRigidBodies.size());

    return mRigidBodies[index];
}

