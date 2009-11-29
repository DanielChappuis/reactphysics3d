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
#include "DynamicEngine.h"
#include "../integration/RungeKutta4.h"
#include "../integration/ExplicitEuler.h"
#include "../integration/SemiImplicitEuler.h"

// We want to use the ReactPhysics3D namespace
using namespace reactphysics3d;

// Constructor
DynamicEngine::DynamicEngine(DynamicWorld* world, const Time& timeStep)
              :PhysicsEngine(world, timeStep) {
    // Check if the pointer to the world is not NULL
    if (world == 0) {
        // Throw an exception
        throw std::invalid_argument("Exception in PhysicsEngine constructor : World pointer cannot be NULL");
    }

    // Creation of the Semi-Implicit Euler integration algorithm
    integrationAlgorithm = new SemiImplicitEuler();
}

// Copy-constructor
DynamicEngine::DynamicEngine(const DynamicEngine& engine)
              :PhysicsEngine(engine) {
    integrationAlgorithm = engine.integrationAlgorithm;
}

// Destructor
DynamicEngine::~DynamicEngine() {
    // Destroy the integration algorithm
    delete integrationAlgorithm;
}

// Update the state of a rigid body
void DynamicEngine::updateBodyState(RigidBody* const rigidBody, const Time& timeStep) {

    // If the gravity force is on
    if(world->getIsGravityOn()) {
        // Apply the current gravity force to the body
        rigidBody->getCurrentBodyState().setForce(world->getGravity());
    }

    // The current body state of the body becomes the previous body state
    rigidBody->updatePreviousBodyState();

    // Integrate the current body state at time t to get the next state at time t + dt
    integrationAlgorithm->integrate(rigidBody->getCurrentBodyState(), timer.getTime(), timeStep);

    // If the body state has changed, we have to update some informations in the rigid body
    rigidBody->update();
}

// Update the physics simulation
void DynamicEngine::update() {
    // Check if the physics simulation is running
    if (timer.getIsRunning()) {
        // While the time accumulator is not empty
        while(timer.getAccumulator() >= timer.getTimeStep().getValue()) {
            // For each body in the dynamic world
            for(std::vector<Body*>::const_iterator it = world->getBodyListStartIterator(); it != world->getBodyListEndIterator(); ++it) {
                // If the body is a RigidBody and if the rigid body motion is enabled
                RigidBody* rigidBody = dynamic_cast<RigidBody*>(*it);
                if (rigidBody && rigidBody->getIsMotionEnabled()) {
                    // Update the state of the rigid body
                    updateBodyState(rigidBody, timer.getTimeStep());
                }
            }

            // Update the timer
            timer.update();
        }

        // For each body in the dynamic world
        for(std::vector<Body*>::const_iterator it = world->getBodyListStartIterator(); it != world->getBodyListEndIterator(); ++it) {
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
