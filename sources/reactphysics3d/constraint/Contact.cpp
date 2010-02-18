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
#include "../body/RigidBody.h"
#include <GL/freeglut.h>        // TODO : Remove this in the final version
#include <GL/gl.h>              // TODO : Remove this in the final version

// We want to use the ReactPhysics3D namespace
using namespace reactphysics3d;

// Constructor
Contact::Contact(Body* const body1, Body* const body2, const Vector3D& normal, double penetrationDepth, const std::vector<Vector3D>& points)
                 :Constraint(body1, body2), normal(normal), penetrationDepth(penetrationDepth), points(points), nbFrictionVectors(NB_FRICTION_VECTORS) {
    active = true;
}

// Destructor
Contact::~Contact() {

}

// Evaluate the constraint
// This method computes the jacobian matrices of the constraint
// The argument "dt" is the time step of the physics simulation
void Contact::evaluate(double dt) {
    RigidBody* rigidBody1 = dynamic_cast<RigidBody*>(body1);
    RigidBody* rigidBody2 = dynamic_cast<RigidBody*>(body2);

    assert(frictionVectors.size() == NB_FRICTION_VECTORS);
    assert(rigidBody1 != 0);
    assert(rigidBody2 != 0);

    // Computation of the linear jacobian for body 1
    body1LinearJacobian = Matrix(1 + NB_FRICTION_VECTORS, 3);
    body1LinearJacobian.setValue(0, 0, -normal.getX());
    body1LinearJacobian.setValue(0, 1, -normal.getY());
    body1LinearJacobian.setValue(0, 2, -normal.getZ());
    for (unsigned int i=1; i<=NB_FRICTION_VECTORS; ++i) {
        body1LinearJacobian.setValue(i, 0, -frictionVectors.at(i-1).getX());
        body1LinearJacobian.setValue(i, 1, -frictionVectors.at(i-1).getY());
        body1LinearJacobian.setValue(i, 2, -frictionVectors.at(i-1).getZ());
    }

    // Computation of the linear jacobian for body 2
    body2LinearJacobian = Matrix(1 + NB_FRICTION_VECTORS, 3);
    body2LinearJacobian.setValue(0, 0, normal.getX());
    body2LinearJacobian.setValue(0, 1, normal.getY());
    body2LinearJacobian.setValue(0, 2, normal.getZ());
    for (unsigned int i=1; i<=NB_FRICTION_VECTORS; ++i) {
        body2LinearJacobian.setValue(i, 0, frictionVectors.at(i-1).getX());
        body2LinearJacobian.setValue(i, 1, frictionVectors.at(i-1).getY());
        body2LinearJacobian.setValue(i, 2, frictionVectors.at(i-1).getZ());
    }

    // Computation of the angular jacobian for body 1
    Vector3D r1 = rigidBody1->getCurrentBodyState().getPosition();
    Vector3D r1CrossNormal = r1.crossProduct(normal);
    body1AngularJacobian = Matrix(1 + NB_FRICTION_VECTORS, 3);
    body1AngularJacobian.setValue(0, 0, -r1CrossNormal.getX());
    body1AngularJacobian.setValue(0, 1, -r1CrossNormal.getY());
    body1AngularJacobian.setValue(0, 2, -r1CrossNormal.getZ());
    for (unsigned int i=1; i<=NB_FRICTION_VECTORS; ++i) {
        Vector3D r1CrossFrictionVector = r1.crossProduct(frictionVectors.at(i-1));
        body1AngularJacobian.setValue(i, 0, -r1CrossFrictionVector.getX());
        body1AngularJacobian.setValue(i, 1, -r1CrossFrictionVector.getY());
        body1AngularJacobian.setValue(i, 2, -r1CrossFrictionVector.getZ());
    }

    // Computation of the angular jacobian for body 2
    Vector3D r2 = rigidBody2->getCurrentBodyState().getPosition();
    Vector3D r2CrossNormal = r2.crossProduct(normal);
    body2AngularJacobian = Matrix(1 + NB_FRICTION_VECTORS, 3);
    body2AngularJacobian.setValue(0, 0, r2CrossNormal.getX());
    body2AngularJacobian.setValue(0, 1, r2CrossNormal.getY());
    body2AngularJacobian.setValue(0, 2, r2CrossNormal.getZ());
    for (unsigned int i=1; i<=NB_FRICTION_VECTORS; ++i) {
        Vector3D r2CrossFrictionVector = r2.crossProduct(frictionVectors.at(i-1));
        body2AngularJacobian.setValue(i, 0, r2CrossFrictionVector.getX());
        body2AngularJacobian.setValue(i, 1, r2CrossFrictionVector.getY());
        body2AngularJacobian.setValue(i, 2, r2CrossFrictionVector.getZ());
    }

    // Computation of the error vector
    double kFps = 1.0 / dt;
    double kCorrection = kErp * kFps;                       // Computation of the error coefficient
    errorVector = Vector(1+NB_FRICTION_VECTORS);
    errorVector.setValue(0, kCorrection * penetrationDepth);
    for (unsigned int i=1; i<=NB_FRICTION_VECTORS; ++i) {
       errorVector.setValue(i, 0.0);
    }
}

// TODO : Delete this (Used to debug collision detection)
void Contact::draw() const {
    glColor3f(1.0, 0.0, 0.0);
    glutSolidSphere(0.5, 20, 20);
}
