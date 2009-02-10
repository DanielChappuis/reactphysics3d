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
DynamicEngine::DynamicEngine(DynamicsWorld& world, const Time& timeStep)
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

// Compute the interpolation state between the previous body state and the current body state
// This is used to avoid visual stuttering when the display and physics framerates are out of synchronization
BodyState DynamicEngine::interpolateState(const BodyState& previousBodyState, const BodyState& currentBodyState) const {
    // TODO : Implement this method
}

// Update the state of a rigid body
void DynamicEngine::updateBodyState(RigidBody* const rigidBody) {
    // TODO : Implement this method
}

// Update the physics simulation
void DynamicEngine::update() {
    // TODO : Implement this method
}
