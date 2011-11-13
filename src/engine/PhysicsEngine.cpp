/********************************************************************************
* ReactPhysics3D physics library, http://code.google.com/p/reactphysics3d/      *
* Copyright (c) 2010-2012 Daniel Chappuis                                       *
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
#include "PhysicsEngine.h"

// We want to use the ReactPhysics3D namespace
using namespace reactphysics3d;
using namespace std;

// Constructor
PhysicsEngine::PhysicsEngine(PhysicsWorld* world, double timeStep = DEFAULT_TIMESTEP)
              : world(world), timer(timeStep), collisionDetection(world), constraintSolver(world) {
    assert(world);
    assert(timeStep > 0.0);
}

// Destructor
PhysicsEngine::~PhysicsEngine() {

}

// Update the physics simulation
void PhysicsEngine::update() {
    bool existCollision = false;

    assert(timer.getIsRunning());
    
    // Compute the time since the last update() call and update the timer
    timer.update();

    // Apply the gravity force to all bodies
    applyGravity();

    // While the time accumulator is not empty
    while(timer.isPossibleToTakeStep()) {
        existCollision = false;

		
        // Compute the collision detection
        if (collisionDetection.computeCollisionDetection()) {
            existCollision = true;

            // Solve constraints
            constraintSolver.solve(timer.getTimeStep());
        }

        // Update the timer
        timer.nextStep();

        // Update the position and orientation of each body
        updateAllBodiesMotion();

        // Cleanup of the constraint solver
        if (existCollision) {
           constraintSolver.cleanup();
        }

        // Clear the added and removed bodies from last update() method call
        world->clearAddedAndRemovedBodies();
    }

    // Compute and set the interpolation factor to all the bodies
    setInterpolationFactorToAllBodies();
}

// Compute the motion of all bodies and update their positions and orientations
// First this method compute the vector V2 = V_constraint + V_forces + V1 where
// V_constraint = dt * (M^-1 * J^T * lambda) and V_forces = dt * (M^-1 * F_ext)
// V2 is the final velocity after the timestep and V1 is the velocity before the
// timestep.
// After having computed the velocity V2, this method will update the position
// and orientation of each body.
// This method uses the semi-implicit Euler method to update the position and
// orientation of the body
void PhysicsEngine::updateAllBodiesMotion() {
    double dt = timer.getTimeStep();
    Vector3 newLinearVelocity;
    Vector3 newAngularVelocity;

    // For each body of thephysics world
    for (vector<RigidBody*>::iterator it=world->getRigidBodiesBeginIterator(); it != world->getRigidBodiesEndIterator(); ++it) {

        RigidBody* rigidBody = *it;
        assert(rigidBody);

        // If the body is able to move
        if (rigidBody->getIsMotionEnabled()) {
            newLinearVelocity.setAllValues(0.0, 0.0, 0.0);
            newAngularVelocity.setAllValues(0.0, 0.0, 0.0);

            // If it's a constrained body
            if (constraintSolver.isConstrainedBody(*it)) {
                // Get the constrained linear and angular velocities from the constraint solver
                newLinearVelocity = constraintSolver.getConstrainedLinearVelocityOfBody(*it);
                newAngularVelocity = constraintSolver.getConstrainedAngularVelocityOfBody(*it);
            }

            // Compute V_forces = dt * (M^-1 * F_ext) which is the velocity of the body due to the
            // external forces and torques.
            newLinearVelocity += dt * rigidBody->getMassInverse() * rigidBody->getExternalForce();
            newAngularVelocity += dt * rigidBody->getInertiaTensorInverseWorld() * rigidBody->getExternalTorque();

            // Add the velocity V1 to the new velocity
            newLinearVelocity += rigidBody->getLinearVelocity();
            newAngularVelocity += rigidBody->getAngularVelocity();

            // Update the position and the orientation of the body according to the new velocity
            updatePositionAndOrientationOfBody(*it, newLinearVelocity, newAngularVelocity);

            // Update the AABB of the rigid body
            rigidBody->updateAABB();
        }
    }
}

// Update the position and orientation of a body
// Use the Semi-Implicit Euler (Sympletic Euler) method to compute the new position and the new
// orientation of the body
void PhysicsEngine::updatePositionAndOrientationOfBody(RigidBody* rigidBody, const Vector3& newLinVelocity, const Vector3& newAngVelocity) {
    double dt = timer.getTimeStep();

    assert(rigidBody);

    // Update the old position and orientation of the body
    rigidBody->updateOldTransform();

    // Update the linear and angular velocity of the body
    rigidBody->setLinearVelocity(newLinVelocity);
    rigidBody->setAngularVelocity(newAngVelocity);

    // Get current position and orientation of the body
    const Vector3& currentPosition = rigidBody->getTransform().getPosition();
    const Quaternion& currentOrientation = rigidBody->getTransform().getOrientation();

    Vector3 newPosition = currentPosition + newLinVelocity * dt;
    Quaternion newOrientation = currentOrientation + Quaternion(newAngVelocity.getX(), newAngVelocity.getY(), newAngVelocity.getZ(), 0) * currentOrientation * 0.5 * dt;
    Transform newTransform(newPosition, newOrientation.getUnit());
    rigidBody->setTransform(newTransform);
}

// Compute and set the interpolation factor to all bodies
void PhysicsEngine::setInterpolationFactorToAllBodies() {
    // Compute the interpolation factor
    double factor = timer.computeInterpolationFactor();
    assert(factor >= 0.0 && factor <= 1.0);

    // Set the factor to all bodies
    for (vector<RigidBody*>::iterator it=world->getRigidBodiesBeginIterator(); it != world->getRigidBodiesEndIterator(); ++it) {

        RigidBody* rigidBody = dynamic_cast<RigidBody*>(*it);
        assert(rigidBody);

        rigidBody->setInterpolationFactor(factor);
    }
}

// Apply the gravity force to all bodies of the physics world
void PhysicsEngine::applyGravity() {

    // For each body of the physics world
    for (vector<RigidBody*>::iterator it=world->getRigidBodiesBeginIterator(); it != world->getRigidBodiesEndIterator(); ++it) {

        RigidBody* rigidBody = dynamic_cast<RigidBody*>(*it);
        assert(rigidBody);
   
        // If the gravity force is on
        if(world->getIsGravityOn()) {
            // Apply the current gravity force to the body
            rigidBody->setExternalForce(rigidBody->getMass() * world->getGravity());
        }
    }
}
