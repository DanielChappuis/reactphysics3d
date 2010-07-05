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
#include "../integration/SemiImplicitEuler.h"

// We want to use the ReactPhysics3D namespace
using namespace reactphysics3d;

// Constructor
PhysicsEngine::PhysicsEngine(PhysicsWorld* world, const Time& timeStep) throw (std::invalid_argument)
              : world(world), timer(Time(0.0), timeStep), collisionDetection(world), constraintSolver(world) {
    // Check if the pointer to the world is not NULL
    if (world == 0) {
        // Throw an exception
        throw std::invalid_argument("Exception in PhysicsEngine constructor : World pointer cannot be NULL");
    }

    // Creation of the Semi-Implicit Euler integration algorithm
    integrationAlgorithm = new SemiImplicitEuler();
}

// Destructor
PhysicsEngine::~PhysicsEngine() {
    delete integrationAlgorithm;
}

void PhysicsEngine::update() {
    updateCollision();
}

/*
// TODO : Delete this method
// Update the physics simulation
void PhysicsEngine::updateDynamic() {
    // Check if the physics simulation is running
    if (timer.getIsRunning()) {
        // While the time accumulator is not empty
        while(timer.getAccumulator() >= timer.getTimeStep().getValue()) {
            // For each body in the dynamic world
            for(std::vector<Body*>::const_iterator it = world->getBodiesBeginIterator(); it != world->getBodiesEndIterator(); ++it) {
                // If the body is a RigidBody and if the rigid body motion is enabled
                RigidBody* rigidBody = dynamic_cast<RigidBody*>(*it);
                if (rigidBody && rigidBody->getIsMotionEnabled()) {
                    // Update the state of the rigid body
                    //updateBodyState(rigidBody, timer.getTimeStep());
                }
            }

            // Update the timer
            timer.update();
        }

        // For each body in the dynamic world
        for(std::vector<Body*>::const_iterator it = world->getBodiesBeginIterator(); it != world->getBodiesEndIterator(); ++it) {
            // If the body is a RigidBody and if the rigid body motion is enabled
            RigidBody* rigidBody = dynamic_cast<RigidBody*>(*it);
            if (rigidBody && rigidBody->getIsMotionEnabled()) {
                // Update the interpolation factor of the rigid body
                // This one will be used to compute the interpolated state
                rigidBody->setInterpolationFactor(timer.getInterpolationFactor());
            }
        }
    }
}
*/

// TODO : Delethe this method
// Update the physics simulation
void PhysicsEngine::updateCollision() {
    bool existCollision = false;

    // Apply the gravity force to all bodies
    applyGravity();

    // While the time accumulator is not empty
    while(timer.getAccumulator() >= timer.getTimeStep().getValue()) {
        existCollision = false;
        // Compute the collision detection
        if (collisionDetection.computeCollisionDetection()) {
            existCollision = true;

            // Solve constraints
            constraintSolver.solve(timer.getTimeStep().getValue());
        }

        // Update the timer
        timer.update();

        // Update the position and orientation of each body
        updateAllBodiesMotion();

        // Free the allocated memory of the constraint solver
        if (existCollision) {
            constraintSolver.freeMemory();
        }
    }

    /*
    // For each body in the the dynamic world
    for(std::vector<Body*>::const_iterator it = world->getBodiesBeginIterator(); it != world->getBodiesEndIterator(); ++it) {
        // If the body is a RigidBody and if the rigid body motion is enabled
        RigidBody* rigidBody = dynamic_cast<RigidBody*>(*it);
        if (rigidBody) {
            
        }
    }
     */
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
    double dt = timer.getTimeStep().getValue();
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
            newLinearVelocity = newLinearVelocity + dt * rigidBody->getCurrentBodyState().getMassInverse().getValue() * rigidBody->getCurrentBodyState().getExternalForce();
            newAngularVelocity = newAngularVelocity + dt * rigidBody->getInertiaTensorInverseWorld() * rigidBody->getCurrentBodyState().getExternalTorque();

            // Add the velocity V1 to the new velocity
            newLinearVelocity = newLinearVelocity + rigidBody->getCurrentBodyState().getLinearVelocity();
            newAngularVelocity = newAngularVelocity + rigidBody->getCurrentBodyState().getAngularVelocity();

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
    double dt = timer.getTimeStep().getValue();

    RigidBody* rigidBody = dynamic_cast<RigidBody*>(body);
    assert(rigidBody);

    // The current body state of the body becomes the previous body state
    rigidBody->updatePreviousBodyState();

    BodyState& bodyState = rigidBody->getCurrentBodyState();

    // Normalize the orientation quaternion
    bodyState.setOrientation(bodyState.getOrientation().getUnit());

    // Update the linear and angular velocity of the body
    bodyState.setLinearVelocity(newLinVelocity);
    bodyState.setAngularVelocity(newAngVelocity);

    // Update the position and the orientation of the body
    bodyState.setPosition(bodyState.getPosition() + newLinVelocity * dt);
    bodyState.setOrientation(bodyState.getOrientation() + Quaternion(newAngVelocity.getX(), newAngVelocity.getY(), newAngVelocity.getZ(), 0) * rigidBody->getCurrentBodyState().getOrientation() * 0.5 * dt);
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
            rigidBody->getCurrentBodyState().setExternalForce(world->getGravity());
        }
    }
}
