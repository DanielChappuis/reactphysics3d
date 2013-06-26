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

#ifndef REACTPHYSICS3D_DYNAMICS_WORLD_H
#define REACTPHYSICS3D_DYNAMICS_WORLD_H

// Libraries
#include "CollisionWorld.h"
#include "../collision/CollisionDetection.h"
#include "ContactSolver.h"
#include "ConstraintSolver.h"
#include "../body/RigidBody.h"
#include "Timer.h"
#include "../configuration.h"

/// Namespace ReactPhysics3D
namespace reactphysics3d {

// Class DynamicsWorld
/**
 * This class represents a dynamics world. This class inherits from
 * the CollisionWorld class. In a dynamics world, bodies can collide
 * and their movements are simulated using the laws of physics.
 */
class DynamicsWorld : public CollisionWorld {

    protected :

        // -------------------- Attributes -------------------- //

        /// Timer of the physics engine
        Timer mTimer;

        /// Contact solver
        ContactSolver mContactSolver;

        /// Constraint solver
        ConstraintSolver mConstraintSolver;

        /// Number of iterations for the velocity solver of the Sequential Impulses technique
        uint mNbVelocitySolverIterations;

        /// Number of iterations for the position solver of the Sequential Impulses technique
        uint mNbPositionSolverIterations;

        /// True if the deactivation (sleeping) of inactive bodies is enabled
        bool mIsDeactivationActive;

        /// All the rigid bodies of the physics world
        std::set<RigidBody*> mRigidBodies;

        /// All the contact constraints
        std::vector<ContactManifold*> mContactManifolds;

        /// All the joints of the world
        std::set<Constraint*> mJoints;

        /// Gravity vector of the world
        Vector3 mGravity;

        /// True if the gravity force is on
        bool mIsGravityOn;

        /// Array of constrained linear velocities (state of the linear velocities
        /// after solving the constraints)
        std::vector<Vector3> mConstrainedLinearVelocities;

        /// Array of constrained angular velocities (state of the angular velocities
        /// after solving the constraints)
        std::vector<Vector3> mConstrainedAngularVelocities;

        /// Array of constrained rigid bodies position (for position error correction)
        std::vector<Vector3> mConstrainedPositions;

        /// Array of constrained rigid bodies orientation (for position error correction)
        std::vector<Quaternion> mConstrainedOrientations;

        /// Map body to their index in the constrained velocities array
        std::map<RigidBody*, uint> mMapBodyToConstrainedVelocityIndex;

        // -------------------- Methods -------------------- //

        /// Private copy-constructor
        DynamicsWorld(const DynamicsWorld& world);

        /// Private assignment operator
        DynamicsWorld& operator=(const DynamicsWorld& world);

        /// Integrate the positions and orientations of rigid bodies.
        void integrateRigidBodiesPositions();

        /// Update the AABBs of the bodies
        void updateRigidBodiesAABB();

        /// Update the position and orientation of a body
        void updatePositionAndOrientationOfBody(RigidBody* body, Vector3 newLinVelocity,
                                                Vector3 newAngVelocity);

        /// Compute and set the interpolation factor to all bodies
        void setInterpolationFactorToAllBodies();

        /// Integrate the velocities of rigid bodies.
        void integrateRigidBodiesVelocities();

        /// Solve the contacts and constraints
        void solveContactsAndConstraints();

        /// Solve the position error correction of the constraints
        void solvePositionCorrection();

        /// Cleanup the constrained velocities array at each step
        void cleanupConstrainedVelocitiesArray();

        /// Apply the gravity force to all bodies
        void applyGravity();

        /// Reset the boolean movement variable of each body
        void resetBodiesMovementVariable();

        /// Update the overlapping pair
        virtual void updateOverlappingPair(const BroadPhasePair* pair);

        /// Notify the world about a new broad-phase overlapping pair
        virtual void notifyAddedOverlappingPair(const BroadPhasePair* addedPair);

        /// Notify the world about a removed broad-phase overlapping pair
        virtual void notifyRemovedOverlappingPair(const BroadPhasePair* removedPair);

        /// Notify the world about a new narrow-phase contact
        virtual void notifyNewContact(const BroadPhasePair* pair,
                                      const ContactPointInfo* contactInfo);

public :

        // -------------------- Methods -------------------- //

        /// Constructor
        DynamicsWorld(const Vector3& mGravity, decimal timeStep);

        /// Destructor
        virtual ~DynamicsWorld();

        /// Start the physics simulation
        void start();

        /// Stop the physics simulation
        void stop();

        /// Update the physics simulation
        void update();

        /// Set the number of iterations for the velocity constraint solver
        void setNbIterationsVelocitySolver(uint nbIterations);

        /// Set the number of iterations for the position constraint solver
        void setNbIterationsPositionSolver(uint nbIterations);

        /// Set the position correction technique used for contacts
        void setContactsPositionCorrectionTechnique(ContactsPositionCorrectionTechnique technique);

        /// Set the position correction technique used for joints
        void setJointsPositionCorrectionTechnique(JointsPositionCorrectionTechnique technique);

        /// Activate or deactivate the solving of friction constraints at the center of
        /// the contact manifold instead of solving them at each contact point
        void setIsSolveFrictionAtContactManifoldCenterActive(bool isActive);

        /// Create a rigid body into the physics world.
        RigidBody* createRigidBody(const Transform& transform, decimal mass,
                                   const Matrix3x3& inertiaTensorLocal,
                                   const CollisionShape& collisionShape);

        /// Destroy a rigid body and all the joints which it belongs
        void destroyRigidBody(RigidBody* rigidBody);

        /// Create a joint between two bodies in the world and return a pointer to the new joint
        Constraint* createJoint(const ConstraintInfo& jointInfo);

        /// Destroy a joint
        void destroyJoint(Constraint* joint);

        /// Return the gravity vector of the world
        Vector3 getGravity() const;

        /// Return if the gravity is on
        bool getIsGravityOn() const;

        /// Set the isGravityOn attribute
        void setIsGratityOn(bool isGravityOn);

        /// Return the number of rigid bodies in the world
        uint getNbRigidBodies() const;

        /// Return the number of joints in the world
        uint getNbJoints() const;

        /// Return the number of contact manifolds in the world
        uint getNbContactManifolds() const;

        /// Return the current physics time (in seconds)
        long double getPhysicsTime() const;

        /// Return an iterator to the beginning of the rigid bodies of the physics world
        std::set<RigidBody*>::iterator getRigidBodiesBeginIterator();

        /// Return an iterator to the end of the rigid bodies of the physics world
        std::set<RigidBody*>::iterator getRigidBodiesEndIterator();
};

// Start the physics simulation
inline void DynamicsWorld::start() {
    mTimer.start();
}

inline void DynamicsWorld::stop() {
    mTimer.stop();
}                

// Set the number of iterations for the velocity constraint solver
inline void DynamicsWorld::setNbIterationsVelocitySolver(uint nbIterations) {
    mNbVelocitySolverIterations = nbIterations;
}

// Set the number of iterations for the position constraint solver
inline void DynamicsWorld::setNbIterationsPositionSolver(uint nbIterations) {
    mNbPositionSolverIterations = nbIterations;
}

// Set the position correction technique used for contacts
inline void DynamicsWorld::setContactsPositionCorrectionTechnique(
                              ContactsPositionCorrectionTechnique technique) {
    if (technique == BAUMGARTE_CONTACTS) {
        mContactSolver.setIsSplitImpulseActive(false);
    }
    else {
        mContactSolver.setIsSplitImpulseActive(true);
    }
}

// Set the position correction technique used for joints
inline void DynamicsWorld::setJointsPositionCorrectionTechnique(
                              JointsPositionCorrectionTechnique technique) {
    if (technique == BAUMGARTE_JOINTS) {
        mConstraintSolver.setIsNonLinearGaussSeidelPositionCorrectionActive(false);
    }
    else {
        mConstraintSolver.setIsNonLinearGaussSeidelPositionCorrectionActive(true);
    }
}

// Activate or deactivate the solving of friction constraints at the center of
// the contact manifold instead of solving them at each contact point
inline void DynamicsWorld::setIsSolveFrictionAtContactManifoldCenterActive(bool isActive) {
    mContactSolver.setIsSolveFrictionAtContactManifoldCenterActive(isActive);
}

// Reset the boolean movement variable of each body
inline void DynamicsWorld::resetBodiesMovementVariable() {

    // For each rigid body
    for (std::set<RigidBody*>::iterator it = getRigidBodiesBeginIterator();
         it != getRigidBodiesEndIterator(); it++) {

        // Set the hasMoved variable to false
        (*it)->setHasMoved(false);
    }
}

// Update the overlapping pair
inline void DynamicsWorld::updateOverlappingPair(const BroadPhasePair* pair) {

    // Get the pair of body index
    std::pair<bodyindex, bodyindex> indexPair = pair->getBodiesIndexPair();

    // Get the corresponding overlapping pair
    OverlappingPair* overlappingPair = mOverlappingPairs[indexPair];

    // Update the contact cache of the overlapping pair
    overlappingPair->update();
}

// Return the gravity vector of the world
inline Vector3 DynamicsWorld::getGravity() const {
    return mGravity;
}

// Return if the gravity is on
inline bool DynamicsWorld::getIsGravityOn() const {
    return mIsGravityOn;
}

// Set the isGravityOn attribute
inline void DynamicsWorld::setIsGratityOn(bool isGravityOn) {
    mIsGravityOn = isGravityOn;
}

// Return the number of rigid bodies in the world
inline uint DynamicsWorld::getNbRigidBodies() const {
    return mRigidBodies.size();
}

/// Return the number of joints in the world
inline uint DynamicsWorld::getNbJoints() const {
    return mJoints.size();
}

// Return an iterator to the beginning of the bodies of the physics world
inline std::set<RigidBody*>::iterator DynamicsWorld::getRigidBodiesBeginIterator() {
    return mRigidBodies.begin();
}

// Return an iterator to the end of the bodies of the physics world
inline std::set<RigidBody*>::iterator DynamicsWorld::getRigidBodiesEndIterator() {
    return mRigidBodies.end();
}

// Return the number of contact manifolds in the world
inline uint DynamicsWorld::getNbContactManifolds() const {
    return mContactManifolds.size();
}

/// Return the current physics time (in seconds)
inline long double DynamicsWorld::getPhysicsTime() const {
    return mTimer.getPhysicsTime();
}

}

#endif
