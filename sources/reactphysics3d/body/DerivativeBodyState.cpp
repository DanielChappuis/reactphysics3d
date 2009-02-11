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
#include "DerivativeBodyState.h"

// We want to use the ReactPhysics3D namespace
using namespace reactphysics3d;

// Constructor
DerivativeBodyState::DerivativeBodyState(const Vector3D& linearVelocity, const Vector3D& force, const Vector3D& torque,
                            const Quaternion& spin) : linearVelocity(linearVelocity), force(force), torque(torque),
                                                      spin(spin) {

}

// Copy-constructor
DerivativeBodyState::DerivativeBodyState(const DerivativeBodyState& derivativeBodyState)
                    : linearVelocity(derivativeBodyState.linearVelocity), force(derivativeBodyState.force),
                      torque(derivativeBodyState.torque), spin(derivativeBodyState.spin) {

}

// Destructor
DerivativeBodyState::~DerivativeBodyState() {

}
