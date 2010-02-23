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
#include "SemiImplicitEuler.h"

// We want to use the ReactPhysics3D namespace
using namespace reactphysics3d;

// Constructor
SemiImplicitEuler::SemiImplicitEuler() {

}

// Copy-constructor
SemiImplicitEuler::SemiImplicitEuler(const SemiImplicitEuler& euler) {

}

// Destructor
SemiImplicitEuler::~SemiImplicitEuler() {

}

// Integrate a body state over time. This method use the semi-implicit Euler integration algorithm
void SemiImplicitEuler::integrate(BodyState& bodyState, const Time& time, const Time& timeStep) {
    double dt = timeStep.getValue();    // Timestep

    // Compute the integrated body state
    bodyState.setLinearMomentum(bodyState.getLinearMomentum() + bodyState.getExternalForce() * dt);
    bodyState.setAngularMomentum(bodyState.getAngularMomentum() + bodyState.getExternalTorque() * dt);

    // Recalculate the secondary values of the body state
    bodyState.recalculate();

    // Compute the integrated position and orientation
    bodyState.setPosition(bodyState.getPosition() + bodyState.getLinearVelocity() * dt);
    bodyState.setOrientation(bodyState.getOrientation() + bodyState.getSpin() * dt);
}

