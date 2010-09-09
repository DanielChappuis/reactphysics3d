/********************************************************************************
* ReactPhysics3D physics library, http://code.google.com/p/reactphysics3d/      *
* Copyright (c) 2010 Daniel Chappuis                                            *
*********************************************************************************
*                                                                               *
* Permission is hereby granted, free of charge, to any person obtaining a copy  *
* of this software and associated documentation files (the "Software"), to deal *
* in the Software without restriction, including without limitation the rights  *
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell     *
* copies of the Software, and to permit persons to whom the Software is         *
* furnished to do so, subject to the following conditions:                      *
*                                                                               *
* The above copyright notice and this permission notice shall be included in    *
* all copies or substantial portions of the Software.                           *
*                                                                               *
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR    *
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,      *
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE   *
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER        *
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, *
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN     *
* THE SOFTWARE.                                                                 *
********************************************************************************/

// Libraries
#include "PhysicsEngine.h"

// We want to use the ReactPhysics3D namespace
using namespace reactphysics3d;
using namespace std;

// Constructor
PhysicsEngine::PhysicsEngine(PhysicsWorld* world, double timeStep) throw (invalid_argument)
              : world(world), timer(timeStep), collisionDetection(world), constraintSolver(world) {
    // Check if the pointer to the world is not NULL
    if (world == 0) {
        // Throw an exception
        throw invalid_argument("Error : The argument world to the PhysicsEngine constructor cannot be NULL");
    }

    // Check if the timeStep is positive
    if (timeStep <= 0.0) {
        // Throw an exception
        throw invalid_argument("Error : The timeStep argument to the PhysicsEngine constructor have to be greater than zero");
    }
}

// Destructor
PhysicsEngine::~PhysicsEngine() {

}

// Update the physics simulation
void PhysicsEngine::update() throw (logic_error) {
    bool existCollision = false;

    // Check that the timer is running
    if (timer.getIsRunning()) {

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
    else {  // Call to update() but the timer is not running
        // Throw an exception
        throw logic_error("Error : The PhysicsEngine::start() method have to be called before calling PhysicsEngine::update()");
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
    for (vector<Body*>::iterator it=world->getBodiesBeginIterator(); it != world->getBodiesEndIterator(); it++) {

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

// Compute and set the interpolation factor to all bodies
void PhysicsEngine::setInterpolationFactorToAllBodies() {
    // Compute the interpolation factor
    double factor = timer.computeInterpolationFactor();
    assert(factor >= 0.0 && factor <= 1.0);

    // Set the factor to all bodies
    for (vector<Body*>::iterator it=world->getBodiesBeginIterator(); it != world->getBodiesEndIterator(); it++) {

        RigidBody* rigidBody = dynamic_cast<RigidBody*>(*it);
        assert(rigidBody);

        rigidBody->setInterpolationFactor(factor);
    }
}

// Apply the gravity force to all bodies of the physics world
void PhysicsEngine::applyGravity() {

    // For each body of the physics world
    for (vector<Body*>::iterator it=world->getBodiesBeginIterator(); it != world->getBodiesEndIterator(); it++) {

        RigidBody* rigidBody = dynamic_cast<RigidBody*>(*it);
        assert(rigidBody);
   
        // If the gravity force is on
        if(world->getIsGravityOn()) {
            // Apply the current gravity force to the body
            rigidBody->setExternalForce(rigidBody->getMass() * world->getGravity());
        }
    }
}
