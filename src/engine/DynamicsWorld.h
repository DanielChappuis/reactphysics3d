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

#ifndef REACTPHYSICS3D_DYNAMICS_WORLD_H
#define REACTPHYSICS3D_DYNAMICS_WORLD_H

// Libraries
#include "CollisionWorld.h"
#include "ConstraintSolver.h"
#include "configuration.h"
#include "utils/Logger.h"
#include "engine/ContactSolver.h"

/// Namespace ReactPhysics3D
namespace reactphysics3d {

// Declarations
class CollisionDetection;
class Island;
class RigidBody;

// Class DynamicsWorld
/**
 * This class represents a dynamics world. This class inherits from
 * the CollisionWorld class. In a dynamics world, bodies can collide
 * and their movements are simulated using the laws of physics.
 */
class DynamicsWorld : public CollisionWorld {

    protected :

        // -------------------- Attributes -------------------- //

        /// Contact solver
        ContactSolver mContactSolver;

        /// Constraint solver
        ConstraintSolver mConstraintSolver;

        /// Number of iterations for the velocity solver of the Sequential Impulses technique
        uint mNbVelocitySolverIterations;

        /// Number of iterations for the position solver of the Sequential Impulses technique
        uint mNbPositionSolverIterations;

        /// True if the spleeping technique for inactive bodies is enabled
        bool mIsSleepingEnabled;

        /// All the rigid bodies of the physics world
        List<RigidBody*> mRigidBodies;

        /// All the joints of the world
        List<Joint*> mJoints;

        /// Gravity vector of the world
        Vector3 mGravity;

        /// Current frame time step (in seconds)
        decimal mTimeStep;

        /// True if the gravity force is on
        bool mIsGravityEnabled;

        /// Array of constrained linear velocities (state of the linear velocities
        /// after solving the constraints)
        Vector3* mConstrainedLinearVelocities;

        /// Array of constrained angular velocities (state of the angular velocities
        /// after solving the constraints)
        Vector3* mConstrainedAngularVelocities;

        /// Split linear velocities for the position contact solver (split impulse)
        Vector3* mSplitLinearVelocities;

        /// Split angular velocities for the position contact solver (split impulse)
        Vector3* mSplitAngularVelocities;

        /// Array of constrained rigid bodies position (for position error correction)
        Vector3* mConstrainedPositions;

        /// Array of constrained rigid bodies orientation (for position error correction)
        Quaternion* mConstrainedOrientations;

        /// Number of islands in the world
        uint mNbIslands;

        /// Array with all the islands of awaken bodies
        Island** mIslands;

        /// Sleep linear velocity threshold
        decimal mSleepLinearVelocity;

        /// Sleep angular velocity threshold
        decimal mSleepAngularVelocity;

        /// Time (in seconds) before a body is put to sleep if its velocity
        /// becomes smaller than the sleep velocity.
        decimal mTimeBeforeSleep;

        /// List of free ID for joints
        List<luint> mFreeJointsIDs;

        /// Current joint id
        uint mCurrentJointId;

        // -------------------- Methods -------------------- //

        /// Integrate the positions and orientations of rigid bodies.
        void integrateRigidBodiesPositions();

        /// Reset the external force and torque applied to the bodies
        void resetBodiesForceAndTorque();

        /// Initialize the bodies velocities arrays for the next simulation step.
        void initVelocityArrays();

        /// Integrate the velocities of rigid bodies.
        void integrateRigidBodiesVelocities();

        /// Solve the contacts and constraints
        void solveContactsAndConstraints();

        /// Solve the position error correction of the constraints
        void solvePositionCorrection();

        /// Compute the islands of awake bodies.
        void computeIslands();

        /// Update the postion/orientation of the bodies
        void updateBodiesState();

        /// Put bodies to sleep if needed.
        void updateSleepingBodies();

        /// Add the joint to the list of joints of the two bodies involved in the joint
        void addJointToBody(Joint* joint);

        /// Return the next available joint id
        uint computeNextAvailableJointId();

    public :

        // -------------------- Methods -------------------- //

        /// Constructor
        DynamicsWorld(const Vector3& mGravity, const WorldSettings& worldSettings = WorldSettings(),
                      Logger* logger = nullptr, Profiler* profiler = nullptr);

        /// Destructor
        virtual ~DynamicsWorld() override;

        /// Deleted copy-constructor
        DynamicsWorld(const DynamicsWorld& world) = delete;

        /// Deleted assignment operator
        DynamicsWorld& operator=(const DynamicsWorld& world) = delete;

        /// Update the physics simulation
        void update(decimal timeStep);

        /// Get the number of iterations for the velocity constraint solver
        uint getNbIterationsVelocitySolver() const;

        /// Set the number of iterations for the velocity constraint solver
        void setNbIterationsVelocitySolver(uint nbIterations);

        /// Get the number of iterations for the position constraint solver
        uint getNbIterationsPositionSolver() const;

        /// Set the number of iterations for the position constraint solver
        void setNbIterationsPositionSolver(uint nbIterations);

        /// Set the position correction technique used for contacts
        void setContactsPositionCorrectionTechnique(ContactsPositionCorrectionTechnique technique);

        /// Set the position correction technique used for joints
        void setJointsPositionCorrectionTechnique(JointsPositionCorrectionTechnique technique);

        /// Create a rigid body into the physics world.
        RigidBody* createRigidBody(const Transform& transform);

        /// Destroy a rigid body and all the joints which it belongs
        void destroyRigidBody(RigidBody* rigidBody);

        /// Create a joint between two bodies in the world and return a pointer to the new joint
        Joint* createJoint(const JointInfo& jointInfo);

        /// Destroy a joint
        void destroyJoint(Joint* joint);

        /// Return the gravity vector of the world
        Vector3 getGravity() const;

        /// Set the gravity vector of the world
        void setGravity(Vector3& gravity);

        /// Return if the gravity is on
        bool isGravityEnabled() const;

        /// Enable/Disable the gravity
        void setIsGratityEnabled(bool isGravityEnabled);

        /// Return the number of rigid bodies in the world
        uint getNbRigidBodies() const;

        /// Return the number of joints in the world
        uint getNbJoints() const;

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

        /// Return the list of all contacts of the world
        List<const ContactManifold*> getContactsList();

        // -------------------- Friendship -------------------- //

        friend class RigidBody;
};

// Reset the external force and torque applied to the bodies
inline void DynamicsWorld::resetBodiesForceAndTorque() {

    // For each body of the world
    List<RigidBody*>::Iterator it;
    for (it = mRigidBodies.begin(); it != mRigidBodies.end(); ++it) {
        (*it)->mExternalForce.setToZero();
        (*it)->mExternalTorque.setToZero();
    }
}

// Get the number of iterations for the velocity constraint solver
/**
 * @return The number of iterations of the velocity constraint solver
 */
inline uint DynamicsWorld::getNbIterationsVelocitySolver() const {
    return mNbVelocitySolverIterations;
}

// Set the number of iterations for the velocity constraint solver
/**
 * @param nbIterations Number of iterations for the velocity solver
 */
inline void DynamicsWorld::setNbIterationsVelocitySolver(uint nbIterations) {
    mNbVelocitySolverIterations = nbIterations;

    RP3D_LOG(mLogger, Logger::Level::Information, Logger::Category::World,
             "Dynamics World: Set nb iterations velocity solver to " + std::to_string(nbIterations));
}

// Get the number of iterations for the position constraint solver
/**
 * @return The number of iterations of the position constraint solver
 */
inline uint DynamicsWorld::getNbIterationsPositionSolver() const {
    return mNbPositionSolverIterations;
}

// Set the number of iterations for the position constraint solver
/**
 * @param nbIterations Number of iterations for the position solver
 */
inline void DynamicsWorld::setNbIterationsPositionSolver(uint nbIterations) {
    mNbPositionSolverIterations = nbIterations;

    RP3D_LOG(mLogger, Logger::Level::Information, Logger::Category::World,
             "Dynamics World: Set nb iterations position solver to " + std::to_string(nbIterations));
}

// Set the position correction technique used for contacts
/**
 * @param technique Technique used for the position correction (Baumgarte or Split Impulses)
 */
inline void DynamicsWorld::setContactsPositionCorrectionTechnique(
                              ContactsPositionCorrectionTechnique technique) {
    if (technique == ContactsPositionCorrectionTechnique::BAUMGARTE_CONTACTS) {
        mContactSolver.setIsSplitImpulseActive(false);
    }
    else {
        mContactSolver.setIsSplitImpulseActive(true);
    }
}

// Set the position correction technique used for joints
/**
 * @param technique Technique used for the joins position correction (Baumgarte or Non Linear Gauss Seidel)
 */
inline void DynamicsWorld::setJointsPositionCorrectionTechnique(
                              JointsPositionCorrectionTechnique technique) {
    if (technique == JointsPositionCorrectionTechnique::BAUMGARTE_JOINTS) {
        mConstraintSolver.setIsNonLinearGaussSeidelPositionCorrectionActive(false);
    }
    else {
        mConstraintSolver.setIsNonLinearGaussSeidelPositionCorrectionActive(true);
    }
}

// Return the gravity vector of the world
/**
 * @return The current gravity vector (in meter per seconds squared)
 */
inline Vector3 DynamicsWorld::getGravity() const {
    return mGravity;
}

// Set the gravity vector of the world
/**
 * @param gravity The gravity vector (in meter per seconds squared)
 */
inline void DynamicsWorld::setGravity(Vector3& gravity) {
    mGravity = gravity;

    RP3D_LOG(mLogger, Logger::Level::Information, Logger::Category::World,
             "Dynamics World: Set gravity vector to " + gravity.to_string());
}

// Return if the gravity is enaled
/**
 * @return True if the gravity is enabled in the world
 */
inline bool DynamicsWorld::isGravityEnabled() const {
    return mIsGravityEnabled;
}

// Enable/Disable the gravity
/**
 * @param isGravityEnabled True if you want to enable the gravity in the world
 *                         and false otherwise
 */
inline void DynamicsWorld::setIsGratityEnabled(bool isGravityEnabled) {
    mIsGravityEnabled = isGravityEnabled;

    RP3D_LOG(mLogger, Logger::Level::Information, Logger::Category::World,
             "Dynamics World: isGravityEnabled= " + (isGravityEnabled ? std::string("true") : std::string("false")));
}

// Return the number of rigid bodies in the world
/**
 * @return Number of rigid bodies in the world
 */
inline uint DynamicsWorld::getNbRigidBodies() const {
    return mRigidBodies.size();
}

/// Return the number of joints in the world
/**
 * @return Number of joints in the world
 */
inline uint DynamicsWorld::getNbJoints() const {
    return mJoints.size();
}

// Return true if the sleeping technique is enabled
/**
 * @return True if the sleeping technique is enabled and false otherwise
 */
inline bool DynamicsWorld::isSleepingEnabled() const {
    return mIsSleepingEnabled;
}

// Return the current sleep linear velocity
/**
 * @return The sleep linear velocity (in meters per second)
 */
inline decimal DynamicsWorld::getSleepLinearVelocity() const {
    return mSleepLinearVelocity;
}

// Set the sleep linear velocity.
/// When the velocity of a body becomes smaller than the sleep linear/angular
/// velocity for a given amount of time, the body starts sleeping and does not need
/// to be simulated anymore.
/**
 * @param sleepLinearVelocity The sleep linear velocity (in meters per second)
 */
inline void DynamicsWorld::setSleepLinearVelocity(decimal sleepLinearVelocity) {
    assert(sleepLinearVelocity >= decimal(0.0));
    mSleepLinearVelocity = sleepLinearVelocity;

    RP3D_LOG(mLogger, Logger::Level::Information, Logger::Category::World,
             "Dynamics World: sleepLinearVelocity= " + std::to_string(sleepLinearVelocity));
}

// Return the current sleep angular velocity
/**
 * @return The sleep angular velocity (in radian per second)
 */
inline decimal DynamicsWorld::getSleepAngularVelocity() const {
    return mSleepAngularVelocity;
}

// Set the sleep angular velocity.
/// When the velocity of a body becomes smaller than the sleep linear/angular
/// velocity for a given amount of time, the body starts sleeping and does not need
/// to be simulated anymore.
/**
 * @param sleepAngularVelocity The sleep angular velocity (in radian per second)
 */
inline void DynamicsWorld::setSleepAngularVelocity(decimal sleepAngularVelocity) {
    assert(sleepAngularVelocity >= decimal(0.0));
    mSleepAngularVelocity = sleepAngularVelocity;

    RP3D_LOG(mLogger, Logger::Level::Information, Logger::Category::World,
             "Dynamics World: sleepAngularVelocity= " + std::to_string(sleepAngularVelocity));
}

// Return the time a body is required to stay still before sleeping
/**
 * @return Time a body is required to stay still before sleeping (in seconds)
 */
inline decimal DynamicsWorld::getTimeBeforeSleep() const {
    return mTimeBeforeSleep;
}


// Set the time a body is required to stay still before sleeping
/**
 * @param timeBeforeSleep Time a body is required to stay still before sleeping (in seconds)
 */
inline void DynamicsWorld::setTimeBeforeSleep(decimal timeBeforeSleep) {
    assert(timeBeforeSleep >= decimal(0.0));
    mTimeBeforeSleep = timeBeforeSleep;

    RP3D_LOG(mLogger, Logger::Level::Information, Logger::Category::World,
             "Dynamics World: timeBeforeSleep= " + std::to_string(timeBeforeSleep));
}

// Set an event listener object to receive events callbacks.
/// If you use "nullptr" as an argument, the events callbacks will be disabled.
/**
 * @param eventListener Pointer to the event listener object that will receive
 *                      event callbacks during the simulation
 */
inline void DynamicsWorld::setEventListener(EventListener* eventListener) {
    mEventListener = eventListener;
}

}

#endif
