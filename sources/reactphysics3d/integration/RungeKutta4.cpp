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
#include "RungeKutta4.h"
#include <cassert>

// We want to use the ReactPhysics3D namespace
using namespace reactphysics3d;

// Constructor
RungeKutta4::RungeKutta4() {

}

// Copy-constructor
RungeKutta4::RungeKutta4(const RungeKutta4& rk4) {

}

// Destructor
RungeKutta4::~RungeKutta4() {

}

// Compute a derivative body state at time t
DerivativeBodyState RungeKutta4::evaluate(const BodyState& bodyState, const Time& time) {

    // Compute the derivaties values at time t
    Vector3D linearVelocity = bodyState.getLinearVelocity();
    Vector3D force = bodyState.computeForce(time);
    Vector3D torque = bodyState.computeTorque(time);
    Quaternion spin = bodyState.getSpin();

    // Return the derivative body state at time t
    return DerivativeBodyState(linearVelocity, force, torque, spin);
}

// Compute a derivative body state at time t + dt according to the last derivative body state
DerivativeBodyState RungeKutta4::evaluate(BodyState bodyState, const Time& time, const Time& timeStep,
                                                  const DerivativeBodyState& lastDerivativeBodyState) {
    // Compute the bodyState at time t + dt
    bodyState.computeAtTime(timeStep, lastDerivativeBodyState);

    // Compute the derivaties values at time t
    Vector3D linearVelocity = bodyState.getLinearVelocity();
    Vector3D force = bodyState.computeForce(time + timeStep);
    Vector3D torque = bodyState.computeTorque(time + timeStep);
    Quaternion spin = bodyState.getSpin();

    // Return the derivative body state at time t
    return DerivativeBodyState(linearVelocity, force, torque, spin);
}

// Integrate a body state over time. This method use the RK4 integration algorithm
void RungeKutta4::integrate(BodyState& bodyState, const Time& time, const Time& timeStep) {

    // Compute the 4 derivatives body states at different time values.
    DerivativeBodyState a = evaluate(bodyState, time);
    DerivativeBodyState b = evaluate(bodyState, time, timeStep*0.5, a);
    DerivativeBodyState c = evaluate(bodyState, time, timeStep*0.5, b);
    DerivativeBodyState d = evaluate(bodyState, time, timeStep, c);

    double dt = timeStep.getValue();    // Timestep

    // Compute the integrated body state
    bodyState.setPosition(bodyState.getPosition() + (a.getLinearVelocity() + (b.getLinearVelocity() + c.getLinearVelocity()) * 2.0 +
                          d.getLinearVelocity()) * (1.0/6.0) * dt);
    bodyState.setLinearMomentum(bodyState.getLinearMomentum() + (a.getForce() + (b.getForce() + c.getForce())*2.0 + d.getForce()) * (1.0/6.0) * dt);
    bodyState.setOrientation(bodyState.getOrientation() + (a.getSpin() + (b.getSpin() + c.getSpin())*2.0 + d.getSpin()) * (1.0/6.0) * dt);
    bodyState.setAngularMomentum(bodyState.getAngularMomentum() + (a.getTorque() + (b.getTorque() + c.getTorque())*2.0 + d.getTorque()) * (1.0/6.0) * dt);

    // Recalculate the secondary values of the body state
    bodyState.recalculate();
}
