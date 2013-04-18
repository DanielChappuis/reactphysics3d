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

#ifndef DYNAMICS_WORLD_H
#define DYNAMICS_WORLD_H

// Libraries
#include "CollisionWorld.h"
#include "../collision/CollisionDetection.h"
#include "ContactSolver.h"
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

        /// True if the deactivation (sleeping) of inactive bodies is enabled
        bool mIsDeactivationActive;

        /// All the rigid bodies of the physics world
        std::set<RigidBody*> mRigidBodies;

        /// All the contact constraints
        std::vector<ContactManifold*> mContactManifolds;

        /// All the constraints (except contact constraints)
        std::vector<Constraint*> mConstraints;

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

        /// Map body to their index in the constrained velocities array
        std::map<RigidBody*, uint> mMapBodyToConstrainedVelocityIndex;

        // -------------------- Methods -------------------- //

        /// Private copy-constructor
        DynamicsWorld(const DynamicsWorld& world);

        /// Private assignment operator
        DynamicsWorld& operator=(const DynamicsWorld& world);

        /// Compute the motion of all bodies and update their positions and orientations
        void updateRigidBodiesPositionAndOrientation();

        /// Update the position and orientation of a body
        void updatePositionAndOrientationOfBody(RigidBody* body, Vector3 newLinVelocity,
                                                Vector3 newAngVelocity);

        /// Compute and set the interpolation factor to all bodies
        void setInterpolationFactorToAllBodies();

        /// Initialize the constrained velocities array at each step
        void initConstrainedVelocitiesArray();

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
        virtual void notifyNewContact(const BroadPhasePair* pair, const ContactInfo* contactInfo);

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

        /// Set the number of iterations of the constraint solver
        void setNbIterationsSolver(uint nbIterations);

        /// Activate or Deactivate the split impulses for contacts
        void setIsSplitImpulseActive(bool isActive);

        /// Activate or deactivate the solving of friction constraints at the center of
        /// the contact manifold instead of solving them at each contact point
        void setIsSolveFrictionAtContactManifoldCenterActive(bool isActive);

        /// Set the isErrorCorrectionActive value
        void setIsErrorCorrectionActive(bool isErrorCorrectionActive);

        /// Create a rigid body into the physics world
        RigidBody* createRigidBody(const Transform& transform, decimal mass,
                                   const Matrix3x3& inertiaTensorLocal,
                                   CollisionShape* collisionShape);

        /// Destroy a rigid body
        void destroyRigidBody(RigidBody* rigidBody);

        /// Return the gravity vector of the world
        Vector3 getGravity() const;

        /// Return if the gravity is on
        bool getIsGravityOn() const;

        /// Set the isGravityOn attribute
        void setIsGratityOn(bool isGravityOn);

        /// Return the number of rigid bodies in the world
        uint getNbRigidBodies() const;

        /// Add a constraint
        void addConstraint(Constraint* constraint);

        /// Remove a constraint
        void removeConstraint(Constraint* constraint);

        /// Remove all constraints and delete them (free their memory)
        void removeAllConstraints();

        /// Return the number of contact constraints in the world
        uint getNbContactManifolds() const;

        /// Return a start iterator on the constraint list
        std::vector<Constraint*>::iterator getConstraintsBeginIterator();

        /// Return a end iterator on the constraint list
        std::vector<Constraint*>::iterator getConstraintsEndIterator();

        /// Return a start iterator on the contact manifolds list
        std::vector<ContactManifold*>::iterator getContactManifoldsBeginIterator();

        /// Return a end iterator on the contact manifolds list
        std::vector<ContactManifold*>::iterator getContactManifoldsEndIterator();

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

// Set the number of iterations of the constraint solver
inline void DynamicsWorld::setNbIterationsSolver(uint nbIterations) {
    mContactSolver.setNbIterationsSolver(nbIterations);
}

// Activate or Deactivate the split impulses for contacts
inline void DynamicsWorld::setIsSplitImpulseActive(bool isActive) {
    mContactSolver.setIsSplitImpulseActive(isActive);
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


// Add a constraint into the physics world
inline void DynamicsWorld::addConstraint(Constraint* constraint) {
    assert(constraint != 0);
    mConstraints.push_back(constraint);
}

// Remove a constraint and free its memory
inline void DynamicsWorld::removeConstraint(Constraint* constraint) {
    std::vector<Constraint*>::iterator it;

    assert(constraint != NULL);
    it = std::find(mConstraints.begin(), mConstraints.end(), constraint);
    assert(*it == constraint);
    delete *it;
    mConstraints.erase(it);
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

// Return a start iterator on the constraint list
inline std::vector<Constraint*>::iterator DynamicsWorld::getConstraintsBeginIterator() {
    return mConstraints.begin();
}

// Return a end iterator on the constraint list
inline std::vector<Constraint*>::iterator DynamicsWorld::getConstraintsEndIterator() {
    return mConstraints.end();
}

// Return a start iterator on the contact manifolds list
inline std::vector<ContactManifold*>::iterator DynamicsWorld::getContactManifoldsBeginIterator() {
    return mContactManifolds.begin();
}

// Return a end iterator on the contact manifolds list
inline std::vector<ContactManifold*>::iterator DynamicsWorld::getContactManifoldsEndIterator() {
    return mContactManifolds.end();
}

}

#endif
