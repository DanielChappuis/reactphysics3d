/****************************************************************************
* Copyright (C) 2009      Daniel Chappuis                                  *
****************************************************************************
* This file is part of ReactPhysics3D.                                     *
*                                                                          *
* ReactPhysics3D is free software: you can redistribute it and/or modify   *
* it under the terms of the GNU General Public License as published by     *
* the Free Software Foundation, either version 3 of the License, or        *
* (at your option) any later version.                                      *
*                                                                          *
* ReactPhysics3D is distributed in the hope that it will be useful,        *
* but WITHOUT ANY WARRANTY; without even the implied warranty of           *
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the             *
* GNU General Public License for more details.                             *
*                                                                          *
* You should have received a copy of the GNU General Public License        *
* along with ReactPhysics3D. If not, see <http://www.gnu.org/licenses/>.   *
***************************************************************************/

// Libraries
#include "DynamicEngine.h"

// We want to use the ReactPhysics3D namespace
using namespace reactphysics3d;

// Constructor
DynamicEngine::DynamicEngine(DynamicWorld& world, const Time& timeStep)
              :PhysicsEngine(world, timeStep) {

}

// Copy-constructor
DynamicEngine::DynamicEngine(const DynamicEngine& engine)
              :PhysicsEngine(engine) {
    numericalIntegrator = engine.numericalIntegrator;
}

// Destructor
DynamicEngine::~DynamicEngine() {

}

// Update the state of a rigid body
void DynamicEngine::updateBodyState(RigidBody* const rigidBody) {
    // The current body state of the body becomes the previous body state
    rigidBody->updatePreviousBodyState();

    // Integrate the current body state at time t to get the next state at time t + dt
    numericalIntegrator.integrate(rigidBody->getCurrentBodyState(), timer.getTime(), timer.getTimeStep());
}

// Update the physics simulation
void DynamicEngine::update() {

    // While the time accumulator is not empty
    while(timer.getAccumulator() >= timer.getTimeStep().getValue()) {
        // For each body in the dynamic world
        for(std::vector<Body*>::const_iterator it = world.getBodyListStartIterator(); it != world.getBodyListEndIterator(); ++it) {
            // If the body is a RigidBody and if the rigid body motion is enabled
            RigidBody* rigidBody = dynamic_cast<RigidBody*>(*it);
            if (rigidBody && rigidBody->getIsMotionEnabled()) {
                // Update the state of the rigid body
                updateBodyState(rigidBody);
            }
        }

        // Update the timer
        timer.update();
    }

    // For each body in the dynamic world
    for(std::vector<Body*>::const_iterator it = world.getBodyListStartIterator(); it != world.getBodyListEndIterator(); ++it) {
        // If the body is a RigidBody and if the rigid body motion is enabled
        RigidBody* rigidBody = dynamic_cast<RigidBody*>(*it);
        if (rigidBody && rigidBody->getIsMotionEnabled()) {
            // Update the interpolation factor of the rigid body
            // This one will be used to compute the interpolated state
            rigidBody->setInterpolationFactor(timer.getInterpolationFactor());
        }
    }
}
