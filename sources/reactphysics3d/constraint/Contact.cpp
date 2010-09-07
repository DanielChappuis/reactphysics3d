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
#include "Contact.h"

// We want to use the ReactPhysics3D namespace
using namespace reactphysics3d;

// Constructor
Contact::Contact(Body* const body1, Body* const body2, const Vector3D& normal, double penetrationDepth, const Vector3D& point)
                 :Constraint(body1, body2, 2, true), normal(normal), penetrationDepth(penetrationDepth), point(point) {

    // Compute the auxiliary lower and upper bounds
    // TODO : Now mC is only the mass of the first body but it is probably wrong
    // TODO : Now g is 9.81 but we should use the true gravity value of the physics world.
    mu_mc_g = FRICTION_COEFFICIENT * body1->getMass() * 9.81;

    // Compute the friction vectors that span the tangential friction plane
    computeFrictionVectors();
}

// Destructor
Contact::~Contact() {

}
