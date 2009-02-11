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
#include "NumericalIntegrator.h"

// We want to use the ReactPhysics3D namespace
using namespace reactphysics3d;

// Constructor
NumericalIntegrator::NumericalIntegrator() {

}

// Copy-constructor
NumericalIntegrator::NumericalIntegrator(const NumericalIntegrator& integrator) {

}

// Destructor
NumericalIntegrator::~NumericalIntegrator() {

}

// Compute a derivative body state at time t+dt
DerivativeBodyState NumericalIntegrator::evaluate(BodyState& bodyState, const Time& time, const Time& timeStep) {
    // TODO : Implement this method
}

// Compute a derivative body state at time t + dt according to the last derivative body state
DerivativeBodyState NumericalIntegrator::evaluate(BodyState& bodyState, const Time& time, const Time& timeStep,
                                                  const BodyState& lastDerivativeBodyState) {
    // TODO : Implement this method
}

// Integrate a body state over time
void NumericalIntegrator::integrate(BodyState& bodyState, const Time& t, const Time& dt) {
    // TODO : Implement this method
}
