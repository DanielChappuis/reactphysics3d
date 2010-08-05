/****************************************************************************
 * Copyright (C) 2009      Daniel Chappuis                                  *
 ****************************************************************************
 * This file is part of ReactPhysics3D.                                     *
 *                                                                          *
 * ReactPhysics3D is free software: you can redistribute it and/or modify   *
 * it under the terms of the GNU Lesser General Public License as published *
 * by the Free Software Foundation, either version 3 of the License, or     *
 * (at your option) any later version.                                      *
 *                                                                          *
 * ReactPhysics3D is distributed in the hope that it will be useful,        *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of           *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the             *
 * GNU Lesser General Public License for more details.                      *
 *                                                                          *
 * You should have received a copy of the GNU Lesser General Public License *
 * along with ReactPhysics3D. If not, see <http://www.gnu.org/licenses/>.   *
 ***************************************************************************/

// Libraries
#include "PhysicsEngine.h"

// We want to use the ReactPhysics3D namespace
using namespace reactphysics3d;

// Constructor
PhysicsEngine::PhysicsEngine(PhysicsWorld* world, double timeStep) throw (std::invalid_argument)
              : world(world), timer(0.0, timeStep), collisionDetection(world), constraintSolver(world) {
    // Check if the pointer to the world is not NULL
    if (world == 0) {
        // Throw an exception
        throw std::invalid_argument("Exception in PhysicsEngine constructor : World pointer cannot be NULL");
    }
}

// Destructor
PhysicsEngine::~PhysicsEngine() {

}

void PhysicsEngine::update() {
    bool existCollision = false;

    if (timer.getIsRunning()) {
        // Apply the gravity force to all bodies
        applyGravity();

        // While the time accumulator is not empty
        while(timer.getAccumulator() >= timer.getTimeStep()) {
            existCollision = false;
            // Compute the collision detection
            if (collisionDetection.computeCollisionDetection()) {
                existCollision = true;

                // Solve constraints
                constraintSolver.solve(timer.getTimeStep());
            }

            // Update the timer
            timer.update();

            // Update the position and orientation of each body
            updateAllBodiesMotion();

            // Free the allocated memory of the constraint solver
            if (existCollision) {
                constraintSolver.freeMemory();
            }

            // Clear the added and removed bodies from last update() method call
            world->clearAddedAndRemovedBodies();
        }
    }
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
    Vector3D newLinearVelocity;
    Vector3D newAngularVelocity;

    // For each body of thephysics world
    for (std::vector<Body*>::iterator it=world->getBodiesBeginIterator(); it != world->getBodiesEndIterator(); it++) {

        RigidBody* rigidBody = dynamic_cast<RigidBody*>(*it);
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
            newLinearVelocity = newLinearVelocity + dt * rigidBody->getMassInverse() * rigidBody->getExternalForce();
            newAngularVelocity = newAngularVelocity + dt * rigidBody->getInertiaTensorInverseWorld() * rigidBody->getExternalTorque();

            // Add the velocity V1 to the new velocity
            newLinearVelocity = newLinearVelocity + rigidBody->getLinearVelocity();
            newAngularVelocity = newAngularVelocity + rigidBody->getAngularVelocity();

            // Update the position and the orientation of the body according to the new velocity
            updatePositionAndOrientationOfBody(*it, newLinearVelocity, newAngularVelocity);

            // If the body state has changed, we have to update some informations in the rigid body
            rigidBody->update();
        }

        // Update the interpolation factor of the rigid body
        // This one will be used to compute the interpolated state
        rigidBody->setInterpolationFactor(timer.getInterpolationFactor());
    }
}

// Update the position and orientation of a body
// Use the Semi-Implicit Euler (Sympletic Euler) method to compute the new position and the new
// orientation of the body
 void PhysicsEngine::updatePositionAndOrientationOfBody(Body* body, const Vector3D& newLinVelocity, const Vector3D& newAngVelocity) {
    double dt = timer.getTimeStep();

    RigidBody* rigidBody = dynamic_cast<RigidBody*>(body);
    assert(rigidBody);

    // Update the old position and orientation of the body
    rigidBody->updateOldPositionAndOrientation();

    // Normalize the orientation quaternion
    rigidBody->setOrientation(rigidBody->getOrientation().getUnit());

    // Update the linear and angular velocity of the body
    rigidBody->setLinearVelocity(newLinVelocity);
    rigidBody->setAngularVelocity(newAngVelocity);

    // Update the position and the orientation of the body
    rigidBody->setPosition(rigidBody->getPosition() + newLinVelocity * dt);
    rigidBody->setOrientation(rigidBody->getOrientation() + Quaternion(newAngVelocity.getX(), newAngVelocity.getY(), newAngVelocity.getZ(), 0) * rigidBody->getOrientation() * 0.5 * dt);
}

// Apply the gravity force to all bodies of the physics world
void PhysicsEngine::applyGravity() {

    // For each body of the physics world
    for (std::vector<Body*>::iterator it=world->getBodiesBeginIterator(); it != world->getBodiesEndIterator(); it++) {

        RigidBody* rigidBody = dynamic_cast<RigidBody*>(*it);
        assert(rigidBody);
   
        // If the gravity force is on
        if(world->getIsGravityOn()) {
            // Apply the current gravity force to the body
            rigidBody->setExternalForce(rigidBody->getMass() * world->getGravity());
        }
    }
}
