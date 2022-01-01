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

#ifndef REACTPHYSICS3D_PHYSICS_WORLD_H
#define REACTPHYSICS3D_PHYSICS_WORLD_H

// Libraries
#include <reactphysics3d/mathematics/mathematics.h>
#include <reactphysics3d/containers/Array.h>
#include <reactphysics3d/constraint/Joint.h>
#include <reactphysics3d/memory/MemoryManager.h>
#include <reactphysics3d/engine/EntityManager.h>
#include <reactphysics3d/components/CollisionBodyComponents.h>
#include <reactphysics3d/components/RigidBodyComponents.h>
#include <reactphysics3d/components/TransformComponents.h>
#include <reactphysics3d/components/ColliderComponents.h>
#include <reactphysics3d/components/JointComponents.h>
#include <reactphysics3d/components/BallAndSocketJointComponents.h>
#include <reactphysics3d/components/FixedJointComponents.h>
#include <reactphysics3d/components/HingeJointComponents.h>
#include <reactphysics3d/components/SliderJointComponents.h>
#include <reactphysics3d/collision/CollisionCallback.h>
#include <reactphysics3d/collision/OverlapCallback.h>
#include <reactphysics3d/configuration.h>
#include <reactphysics3d/utils/Logger.h>
#include <reactphysics3d/systems/ConstraintSolverSystem.h>
#include <reactphysics3d/systems/CollisionDetectionSystem.h>
#include <reactphysics3d/systems/ContactSolverSystem.h>
#include <reactphysics3d/systems/DynamicsSystem.h>
#include <reactphysics3d/engine/Islands.h>
#include <reactphysics3d/utils/DebugRenderer.h>
#include <sstream>

/// Namespace ReactPhysics3D
namespace reactphysics3d {

// Declarations
class Island;
class RigidBody;
class PhysicsCommon;
struct JointInfo;

// Class PhysicsWorld
/**
 * This class represents a physics world.
 */
class PhysicsWorld {

    public:

        /// Structure WorldSettings
        /**
         * This class is used to describe some settings of a physics world.
         */
        struct WorldSettings {

            /// Name of the world
            std::string worldName;

            /// Gravity force vector of the world (in Newtons)
            Vector3 gravity;

            /// Distance threshold for two contact points for a valid persistent contact (in meters)
            decimal persistentContactDistanceThreshold;

            /// Default friction coefficient for a rigid body
            decimal defaultFrictionCoefficient;

            /// Default bounciness factor for a rigid body
            decimal defaultBounciness;

            /// Velocity threshold for contact velocity restitution
            decimal restitutionVelocityThreshold;

            /// True if the sleeping technique is enabled
            bool isSleepingEnabled;

            /// Number of iterations when solving the velocity constraints of the Sequential Impulse technique
            uint16 defaultVelocitySolverNbIterations;

            /// Number of iterations when solving the position constraints of the Sequential Impulse technique
            uint16 defaultPositionSolverNbIterations;

            /// Time (in seconds) that a body must stay still to be considered sleeping
            float defaultTimeBeforeSleep;

            /// A body with a linear velocity smaller than the sleep linear velocity (in m/s)
            /// might enter sleeping mode.
            decimal defaultSleepLinearVelocity;

            /// A body with angular velocity smaller than the sleep angular velocity (in rad/s)
            /// might enter sleeping mode
            decimal defaultSleepAngularVelocity;

            /// This is used to test if two contact manifold are similar (same contact normal) in order to
            /// merge them. If the cosine of the angle between the normals of the two manifold are larger
            /// than the value bellow, the manifold are considered to be similar.
            decimal cosAngleSimilarContactManifold;

            WorldSettings() {

                worldName = "";
                gravity = Vector3(0, decimal(-9.81), 0);
                persistentContactDistanceThreshold = decimal(0.03);
                defaultFrictionCoefficient = decimal(0.3);
                defaultBounciness = decimal(0.5);
                restitutionVelocityThreshold = decimal(0.5);
                isSleepingEnabled = true;
                defaultVelocitySolverNbIterations = 6;
                defaultPositionSolverNbIterations = 3;
                defaultTimeBeforeSleep = 1.0f;
                defaultSleepLinearVelocity = decimal(0.02);
                defaultSleepAngularVelocity = decimal(3.0) * (PI_RP3D / decimal(180.0));
                cosAngleSimilarContactManifold = decimal(0.95);
            }

            ~WorldSettings() = default;

            /// Return a string with the world settings
            std::string to_string() const {

                std::stringstream ss;

                ss << "worldName=" << worldName << std::endl;
                ss << "gravity=" << gravity.to_string() << std::endl;
                ss << "persistentContactDistanceThreshold=" << persistentContactDistanceThreshold << std::endl;
                ss << "defaultFrictionCoefficient=" << defaultFrictionCoefficient << std::endl;
                ss << "defaultBounciness=" << defaultBounciness << std::endl;
                ss << "restitutionVelocityThreshold=" << restitutionVelocityThreshold << std::endl;
                ss << "isSleepingEnabled=" << isSleepingEnabled << std::endl;
                ss << "defaultVelocitySolverNbIterations=" << defaultVelocitySolverNbIterations << std::endl;
                ss << "defaultPositionSolverNbIterations=" << defaultPositionSolverNbIterations << std::endl;
                ss << "defaultTimeBeforeSleep=" << defaultTimeBeforeSleep << std::endl;
                ss << "defaultSleepLinearVelocity=" << defaultSleepLinearVelocity << std::endl;
                ss << "defaultSleepAngularVelocity=" << defaultSleepAngularVelocity << std::endl;
                ss << "cosAngleSimilarContactManifold=" << cosAngleSimilarContactManifold << std::endl;

                return ss.str();
            }
        };

    protected :

        // -------------------- Attributes -------------------- //

        /// Memory manager
        MemoryManager& mMemoryManager;

        /// Configuration of the physics world
        WorldSettings mConfig;

        /// Entity Manager for the ECS
        EntityManager mEntityManager;

        /// Debug renderer
        DebugRenderer mDebugRenderer;

        /// True if debug rendering is enabled
        bool mIsDebugRenderingEnabled;

        /// Collision Body Components
        CollisionBodyComponents mCollisionBodyComponents;

        /// Rigid Body Components
        RigidBodyComponents mRigidBodyComponents;

        /// Transform Components
        TransformComponents mTransformComponents;

        /// Collider Components
        ColliderComponents mCollidersComponents;

        /// Joint Components
        JointComponents mJointsComponents;

        /// Ball And Socket joints Components
        BallAndSocketJointComponents mBallAndSocketJointsComponents;

        /// Fixed joints Components
        FixedJointComponents mFixedJointsComponents;

        /// Hinge joints Components
        HingeJointComponents mHingeJointsComponents;

        /// Slider joints Components
        SliderJointComponents mSliderJointsComponents;

        /// Reference to the collision detection
        CollisionDetectionSystem mCollisionDetection;

        /// All the collision bodies of the world
        Array<CollisionBody*> mCollisionBodies;

        /// Pointer to an event listener object
        EventListener* mEventListener;

        /// Name of the physics world
        std::string mName;

#ifdef IS_RP3D_PROFILING_ENABLED

        /// Real-time hierarchical profiler
        Profiler* mProfiler;
#endif

        /// Total number of worlds
        static uint32 mNbWorlds;

        /// All the islands of bodies of the current frame
        Islands mIslands;

        /// Order in which to process the ContactPairs for contact creation such that
        /// all the contact manifolds and contact points of a given island are packed together
        /// This array contains the indices of the ContactPairs.
        Array<uint32> mProcessContactPairsOrderIslands;

        /// Contact solver system
        ContactSolverSystem mContactSolverSystem;

        /// Constraint solver system
        ConstraintSolverSystem mConstraintSolverSystem;

        /// Dynamics system
        DynamicsSystem mDynamicsSystem;

        /// Number of iterations for the velocity solver of the Sequential Impulses technique
        uint16 mNbVelocitySolverIterations;

        /// Number of iterations for the position solver of the Sequential Impulses technique
        uint16 mNbPositionSolverIterations;

        /// True if the spleeping technique for inactive bodies is enabled
        bool mIsSleepingEnabled;

        /// All the rigid bodies of the physics world
        Array<RigidBody*> mRigidBodies;

        /// True if the gravity force is on
        bool mIsGravityEnabled;

        /// Sleep linear velocity threshold
        decimal mSleepLinearVelocity;

        /// Sleep angular velocity threshold
        decimal mSleepAngularVelocity;

        /// Time (in seconds) before a body is put to sleep if its velocity
        /// becomes smaller than the sleep velocity.
        decimal mTimeBeforeSleep;

        // -------------------- Methods -------------------- //

        /// Constructor
        PhysicsWorld(MemoryManager& memoryManager, PhysicsCommon& physicsCommon, const WorldSettings& worldSettings = WorldSettings(), Profiler* profiler = nullptr);

        /// Notify the world if a body is disabled (slepping or inactive) or not
        void setBodyDisabled(Entity entity, bool isDisabled);

        /// Notify the world whether a joint is disabled or not
        void setJointDisabled(Entity jointEntity, bool isDisabled);

        /// Solve the contacts and constraints
        void solveContactsAndConstraints(decimal timeStep);

        /// Solve the position error correction of the constraints
        void solvePositionCorrection();

        /// Compute the islands of awake bodies.
        void computeIslands();

        /// Compute the islands using potential contacts and joints and create the actual contacts.
        void createIslands();

        /// Put bodies to sleep if needed.
        void updateSleepingBodies(decimal timeStep);

        /// Add the joint to the array of joints of the two bodies involved in the joint
        void addJointToBodies(Entity body1, Entity body2, Entity joint);

        /// Update the world inverse inertia tensors of rigid bodies
        void updateBodiesInverseWorldInertiaTensors();

        /// Destructor
        ~PhysicsWorld();

    public :

        // -------------------- Methods -------------------- //

        /// Create a collision body
        CollisionBody* createCollisionBody(const Transform& transform);

        /// Destroy a collision body
        void destroyCollisionBody(CollisionBody* collisionBody);

        /// Get the collision dispatch configuration
        CollisionDispatch& getCollisionDispatch();

        /// Ray cast method
        void raycast(const Ray& ray, RaycastCallback* raycastCallback, unsigned short raycastWithCategoryMaskBits = 0xFFFF) const;

        /// Return true if two bodies overlap (collide)
        bool testOverlap(CollisionBody* body1, CollisionBody* body2);

        /// Report all the bodies that overlap (collide) with the body in parameter
        void testOverlap(CollisionBody* body, OverlapCallback& overlapCallback);

        /// Report all the bodies that overlap (collide) in the world
        void testOverlap(OverlapCallback& overlapCallback);

        /// Test collision and report contacts between two bodies.
        void testCollision(CollisionBody* body1, CollisionBody* body2, CollisionCallback& callback);

        /// Test collision and report all the contacts involving the body in parameter
        void testCollision(CollisionBody* body, CollisionCallback& callback);

        /// Test collision and report contacts between each colliding bodies in the world
        void testCollision(CollisionCallback& callback);

        /// Return a reference to the memory manager of the world
        MemoryManager& getMemoryManager();

        /// Return the current world-space AABB of given collider
        AABB getWorldAABB(const Collider* collider) const;

        /// Return the name of the world
        const std::string& getName() const;

        /// Deleted copy-constructor
        PhysicsWorld(const PhysicsWorld& world) = delete;

        /// Deleted assignment operator
        PhysicsWorld& operator=(const PhysicsWorld& world) = delete;

        /// Update the physics simulation
        void update(decimal timeStep);

        /// Get the number of iterations for the velocity constraint solver
        uint16 getNbIterationsVelocitySolver() const;

        /// Set the number of iterations for the velocity constraint solver
        void setNbIterationsVelocitySolver(uint16 nbIterations);

        /// Get the number of iterations for the position constraint solver
        uint16 getNbIterationsPositionSolver() const;

        /// Set the number of iterations for the position constraint solver
        void setNbIterationsPositionSolver(uint32 nbIterations);

        /// Set the position correction technique used for contacts
        void setContactsPositionCorrectionTechnique(ContactsPositionCorrectionTechnique technique);

        /// Create a rigid body into the physics world.
        RigidBody* createRigidBody(const Transform& transform);

        /// Disable the joints for pair of sleeping bodies
        void enableDisableJoints();

        /// Destroy a rigid body and all the joints which it belongs
        void destroyRigidBody(RigidBody* rigidBody);

        /// Create a joint between two bodies in the world and return a pointer to the new joint
        Joint* createJoint(const JointInfo& jointInfo);

        /// Destroy a joint
        void destroyJoint(Joint* joint);

        /// Return the gravity vector of the world
        Vector3 getGravity() const;

        /// Set the gravity vector of the world
        void setGravity(const Vector3& gravity);

        /// Return if the gravity is on
        bool isGravityEnabled() const;

        /// Enable/Disable the gravity
        void setIsGravityEnabled(bool isGravityEnabled);

        /// Return true if the sleeping technique is enabled
        bool isSleepingEnabled() const;

        /// Enable/Disable the sleeping technique
        void enableSleeping(bool isSleepingEnabled);

        /// Return the current sleep linear velocity
        decimal getSleepLinearVelocity() const;

        /// Set the sleep linear velocity.
        void setSleepLinearVelocity(decimal sleepLinearVelocity);

        /// Return the current sleep angular velocity
        decimal getSleepAngularVelocity() const;

        /// Set the sleep angular velocity.
        void setSleepAngularVelocity(decimal sleepAngularVelocity);

        /// Return the time a body is required to stay still before sleeping
        decimal getTimeBeforeSleep() const;

        /// Set the time a body is required to stay still before sleeping
        void setTimeBeforeSleep(decimal timeBeforeSleep);

        /// Set an event listener object to receive events callbacks.
        void setEventListener(EventListener* eventListener);

        /// Return the number of CollisionBody in the physics world
        uint32 getNbCollisionBodies() const;

        /// Return a constant pointer to a given CollisionBody of the world
        const CollisionBody* getCollisionBody(uint32 index) const;

        /// Return a pointer to a given CollisionBody of the world
        CollisionBody* getCollisionBody(uint32 index) ;

        /// Return the number of RigidBody in the physics world
        uint32 getNbRigidBodies() const;

        /// Return a constant pointer to a given RigidBody of the world
        const RigidBody* getRigidBody(uint32 index) const;

        /// Return a pointer to a given RigidBody of the world
        RigidBody* getRigidBody(uint32 index) ;

        /// Return true if the debug rendering is enabled
        bool getIsDebugRenderingEnabled() const;

        /// Set to true if debug rendering is enabled
        void setIsDebugRenderingEnabled(bool isEnabled);

        /// Return a reference to the Debug Renderer of the world
        DebugRenderer& getDebugRenderer();

#ifdef IS_RP3D_PROFILING_ENABLED

        /// Return a reference to the profiler
        Profiler* getProfiler();

#endif

        // -------------------- Friendship -------------------- //

        friend class CollisionDetectionSystem;
        friend class CollisionBody;
        friend class Collider;
        friend class ConvexMeshShape;
        friend class CollisionCallback::ContactPair;
        friend class OverlapCallback::OverlapPair;
        friend class PhysicsCommon;
        friend class RigidBody;
        friend class Joint;
        friend class BallAndSocketJoint;
        friend class FixedJoint;
        friend class HingeJoint;
        friend class SliderJoint;
        friend class CollisionCallback::CallbackData;
        friend class OverlapCallback::CallbackData;
        friend class DebugRenderer;
};

// Set the collision dispatch configuration
/// This can be used to replace default collision detection algorithms by your
/// custom algorithm for instance.
/**
 * @param CollisionDispatch Pointer to a collision dispatch object describing
 * which collision detection algorithm to use for two given collision shapes
 */
RP3D_FORCE_INLINE CollisionDispatch& PhysicsWorld::getCollisionDispatch() {
    return mCollisionDetection.getCollisionDispatch();
}

// Ray cast method
/**
 * @param ray Ray to use for raycasting
 * @param raycastCallback Pointer to the class with the callback method
 * @param raycastWithCategoryMaskBits Bits mask corresponding to the category of
 *                                    bodies to be raycasted
 */
RP3D_FORCE_INLINE void PhysicsWorld::raycast(const Ray& ray,
                                    RaycastCallback* raycastCallback,
                                    unsigned short raycastWithCategoryMaskBits) const {
    mCollisionDetection.raycast(raycastCallback, ray, raycastWithCategoryMaskBits);
}

// Test collision and report contacts between two bodies.
/// Use this method if you only want to get all the contacts between two bodies.
/// All the contacts will be reported using the callback object in paramater.
/// If you are not interested in the contacts but you only want to know if the bodies collide,
/// you can use the testOverlap() method instead.
/**
 * @param body1 Pointer to the first body to test
 * @param body2 Pointer to the second body to test
 * @param callback Pointer to the object with the callback method
 */
RP3D_FORCE_INLINE void PhysicsWorld::testCollision(CollisionBody* body1, CollisionBody* body2, CollisionCallback& callback) {
    mCollisionDetection.testCollision(body1, body2, callback);
}

// Test collision and report all the contacts involving the body in parameter
/// Use this method if you only want to get all the contacts involving a given body.
/// All the contacts will be reported using the callback object in paramater.
/// If you are not interested in the contacts but you only want to know if the bodies collide,
/// you can use the testOverlap() method instead.
/**
 * @param body Pointer to the body against which we need to test collision
 * @param callback Pointer to the object with the callback method to report contacts
 */
RP3D_FORCE_INLINE void PhysicsWorld::testCollision(CollisionBody* body, CollisionCallback& callback) {
    mCollisionDetection.testCollision(body, callback);
}

// Test collision and report contacts between each colliding bodies in the world
/// Use this method if you want to get all the contacts between colliding bodies in the world.
/// All the contacts will be reported using the callback object in paramater.
/// If you are not interested in the contacts but you only want to know if the bodies collide,
/// you can use the testOverlap() method instead.
/**
 * @param callback Pointer to the object with the callback method to report contacts
 */
RP3D_FORCE_INLINE void PhysicsWorld::testCollision(CollisionCallback& callback) {
    mCollisionDetection.testCollision(callback);
}

// Report all the bodies that overlap (collide) with the body in parameter
/// Use this method if you are not interested in contacts but if you simply want to know
/// which bodies overlap with the body in parameter. If you want to get the contacts, you need to use the
/// testCollision() method instead.
/**
 * @param body Pointer to the collision body to test overlap with
 * @param overlapCallback Pointer to the callback class to report overlap
 */
RP3D_FORCE_INLINE void PhysicsWorld::testOverlap(CollisionBody* body, OverlapCallback& overlapCallback) {
    mCollisionDetection.testOverlap(body, overlapCallback);
}

// Report all the bodies that overlap (collide) in the world
/// Use this method if you are not interested in contacts but if you simply want to know
/// which bodies overlap. If you want to get the contacts, you need to use the
/// testCollision() method instead.
/**
 * @param overlapCallback Pointer to the callback class to report overlap
 */
RP3D_FORCE_INLINE void PhysicsWorld::testOverlap(OverlapCallback& overlapCallback) {
    mCollisionDetection.testOverlap(overlapCallback);
}

// Return a reference to the memory manager of the world
RP3D_FORCE_INLINE MemoryManager& PhysicsWorld::getMemoryManager() {
    return mMemoryManager;
}

// Return the name of the world
/**
 * @return Name of the world
 */
RP3D_FORCE_INLINE const std::string& PhysicsWorld::getName() const {
    return mName;
}

#ifdef IS_RP3D_PROFILING_ENABLED

// Return a pointer to the profiler
/**
 * @return A pointer to the profiler
 */
RP3D_FORCE_INLINE Profiler* PhysicsWorld::getProfiler() {
    return mProfiler;
}

#endif

// Get the number of iterations for the velocity constraint solver
/**
 * @return The number of iterations of the velocity constraint solver
 */
RP3D_FORCE_INLINE uint16 PhysicsWorld::getNbIterationsVelocitySolver() const {
    return mNbVelocitySolverIterations;
}

// Get the number of iterations for the position constraint solver
/**
 * @return The number of iterations of the position constraint solver
 */
RP3D_FORCE_INLINE uint16 PhysicsWorld::getNbIterationsPositionSolver() const {
    return mNbPositionSolverIterations;
}

// Set the position correction technique used for contacts
/**
 * @param technique Technique used for the position correction (Baumgarte or Split Impulses)
 */
RP3D_FORCE_INLINE void PhysicsWorld::setContactsPositionCorrectionTechnique(
                              ContactsPositionCorrectionTechnique technique) {
    if (technique == ContactsPositionCorrectionTechnique::BAUMGARTE_CONTACTS) {
        mContactSolverSystem.setIsSplitImpulseActive(false);
    }
    else {
        mContactSolverSystem.setIsSplitImpulseActive(true);
    }
}

// Return the gravity vector of the world
/**
 * @return The current gravity vector (in meter per seconds squared)
 */
RP3D_FORCE_INLINE Vector3 PhysicsWorld::getGravity() const {
    return mConfig.gravity;
}

// Return if the gravity is enaled
/**
 * @return True if the gravity is enabled in the world
 */
RP3D_FORCE_INLINE bool PhysicsWorld::isGravityEnabled() const {
    return mIsGravityEnabled;
}

// Return true if the sleeping technique is enabled
/**
 * @return True if the sleeping technique is enabled and false otherwise
 */
RP3D_FORCE_INLINE bool PhysicsWorld::isSleepingEnabled() const {
    return mIsSleepingEnabled;
}

// Return the current sleep linear velocity
/**
 * @return The sleep linear velocity (in meters per second)
 */
RP3D_FORCE_INLINE decimal PhysicsWorld::getSleepLinearVelocity() const {
    return mSleepLinearVelocity;
}

// Return the current sleep angular velocity
/**
 * @return The sleep angular velocity (in radian per second)
 */
RP3D_FORCE_INLINE decimal PhysicsWorld::getSleepAngularVelocity() const {
    return mSleepAngularVelocity;
}

// Return the time a body is required to stay still before sleeping
/**
 * @return Time a body is required to stay still before sleeping (in seconds)
 */
RP3D_FORCE_INLINE decimal PhysicsWorld::getTimeBeforeSleep() const {
    return mTimeBeforeSleep;
}

// Set an event listener object to receive events callbacks.
/// If you use "nullptr" as an argument, the events callbacks will be disabled.
/**
 * @param eventListener Pointer to the event listener object that will receive
 *                      event callbacks during the simulation
 */
RP3D_FORCE_INLINE void PhysicsWorld::setEventListener(EventListener* eventListener) {
    mEventListener = eventListener;
}

// Return the number of CollisionBody in the physics world
/// Note that even if a RigidBody is also a collision body, this method does not return the rigid bodies
/**
 * @return The number of collision bodies in the physics world
 */
RP3D_FORCE_INLINE uint32 PhysicsWorld::getNbCollisionBodies() const {
   return static_cast<uint32>(mCollisionBodies.size());
}

// Return the number of RigidBody in the physics world
/**
 * @return The number of rigid bodies in the physics world
 */
RP3D_FORCE_INLINE uint32 PhysicsWorld::getNbRigidBodies() const {
   return static_cast<uint32>(mRigidBodies.size());
}

// Return true if the debug rendering is enabled
/**
 * @return True if the debug rendering is enabled and false otherwise
 */
RP3D_FORCE_INLINE bool PhysicsWorld::getIsDebugRenderingEnabled() const {
    return mIsDebugRenderingEnabled;
}

// Set to true if debug rendering is enabled
/**
 * @param isEnabled True if you want to enable the debug rendering and false otherwise
 */
RP3D_FORCE_INLINE void PhysicsWorld::setIsDebugRenderingEnabled(bool isEnabled) {
    mIsDebugRenderingEnabled = isEnabled;
}

// Return a reference to the Debug Renderer of the world
/**
 * @return A reference to the DebugRenderer object of the world
 */
RP3D_FORCE_INLINE DebugRenderer& PhysicsWorld::getDebugRenderer() {
    return mDebugRenderer;
}

}

#endif
