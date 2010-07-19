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
Contact::Contact(Body* const body1, Body* const body2, const Vector3D& normal, double penetrationDepth, const Vector3D& point)
                 :Constraint(body1, body2, 2, true), normal(normal), penetrationDepth(penetrationDepth), point(point) {
    body1Jacobian.changeSize(1,6);
    body2Jacobian.changeSize(1,6);
    auxJacobian.changeSize(nbAuxConstraints, 12);
    auxLowerBounds.changeSize(nbAuxConstraints);
    auxUpperBounds.changeSize(nbAuxConstraints);
    auxErrorValues.changeSize(nbAuxConstraints);

    body1Jacobian = Matrix(1, 6);
    body2Jacobian = Matrix(1, 6);
    auxJacobian = Matrix(nbAuxConstraints, 12);
    auxLowerBounds = Vector(nbAuxConstraints);
    auxUpperBounds = Vector(nbAuxConstraints);
    auxErrorValues = Vector(nbAuxConstraints);
}

// Destructor
Contact::~Contact() {

}

// Evaluate the constraint
// This method computes the jacobian matrices of the constraint
void Contact::evaluate() {
    RigidBody* rigidBody1 = dynamic_cast<RigidBody*>(body1);
    RigidBody* rigidBody2 = dynamic_cast<RigidBody*>(body2);

    assert(rigidBody1 != 0);
    assert(rigidBody2 != 0);

    // Compute the friction vectors that span the tangential friction plane
    computeFrictionVectors();

    Vector3D r1 = point - rigidBody1->getCurrentBodyState().getPosition();
    Vector3D r2 = point - rigidBody2->getCurrentBodyState().getPosition();
    Vector3D r1CrossN = r1.crossProduct(normal);
    Vector3D r2CrossN = r2.crossProduct(normal);

    // Compute the jacobian matrix for the body 1
    body1Jacobian.setValue(0, 0, -normal.getX());
    body1Jacobian.setValue(0, 1, -normal.getY());
    body1Jacobian.setValue(0, 2, -normal.getZ());
    body1Jacobian.setValue(0, 3, -r1CrossN.getX());
    body1Jacobian.setValue(0, 4, -r1CrossN.getY());
    body1Jacobian.setValue(0, 5, -r1CrossN.getZ());

    // Compute the jacobian matrix for the body 2
    body2Jacobian.setValue(0, 0, normal.getX());
    body2Jacobian.setValue(0, 1, normal.getY());
    body2Jacobian.setValue(0, 2, normal.getZ());
    body2Jacobian.setValue(0, 3, r2CrossN.getX());
    body2Jacobian.setValue(0, 4, r2CrossN.getY());
    body2Jacobian.setValue(0, 5, r2CrossN.getZ());

    // Compute the lower and upper bounds values
    lowerBound = 0.0;
    upperBound = INFINITY_CONST;

    // Compute the error value of the constraint
    Vector3D velocity1 = rigidBody1->getCurrentBodyState().getLinearVelocity();
    Vector3D velocity2 = rigidBody2->getCurrentBodyState().getLinearVelocity();
    double restitutionCoeff = rigidBody1->getRestitution() * rigidBody2->getRestitution();
    errorValue = restitutionCoeff * (normal.scalarProduct(velocity1) - normal.scalarProduct(velocity2)) + PENETRATION_FACTOR * penetrationDepth; // TODO : Add penetration

    // Compute the auxiliary jacobian matrix (this corresponds to the friction constraint)
    Vector3D r1CrossU1 = r1.crossProduct(frictionVectors[0]);
    Vector3D r2CrossU1 = r2.crossProduct(frictionVectors[0]);
    Vector3D r1CrossU2 = r1.crossProduct(frictionVectors[1]);
    Vector3D r2CrossU2 = r2.crossProduct(frictionVectors[1]);
    auxJacobian.setValue(0, 0, -frictionVectors[0].getX());
    auxJacobian.setValue(0, 1, -frictionVectors[0].getY());
    auxJacobian.setValue(0, 2, -frictionVectors[0].getZ());
    auxJacobian.setValue(0, 3, -r1CrossU1.getX());
    auxJacobian.setValue(0, 4, -r1CrossU1.getY());
    auxJacobian.setValue(0, 5, -r1CrossU1.getZ());
    auxJacobian.setValue(0, 6, frictionVectors[0].getX());
    auxJacobian.setValue(0, 7, frictionVectors[0].getY());
    auxJacobian.setValue(0, 8, frictionVectors[0].getZ());
    auxJacobian.setValue(0, 9, r2CrossU1.getX());
    auxJacobian.setValue(0, 10, r2CrossU1.getY());
    auxJacobian.setValue(0, 11, r2CrossU1.getZ());
    auxJacobian.setValue(1, 0, -frictionVectors[1].getX());
    auxJacobian.setValue(1, 1, -frictionVectors[1].getY());
    auxJacobian.setValue(1, 2, -frictionVectors[1].getZ());
    auxJacobian.setValue(1, 3, -r1CrossU2.getX());
    auxJacobian.setValue(1, 4, -r1CrossU2.getY());
    auxJacobian.setValue(1, 5, -r1CrossU2.getZ());
    auxJacobian.setValue(1, 6, frictionVectors[1].getX());
    auxJacobian.setValue(1, 7, frictionVectors[1].getY());
    auxJacobian.setValue(1, 8, frictionVectors[1].getZ());
    auxJacobian.setValue(1, 9, r2CrossU2.getX());
    auxJacobian.setValue(1, 10, r2CrossU2.getY());
    auxJacobian.setValue(1, 11, r2CrossU2.getZ());

    // Compute the auxiliary lower and upper bounds
    // TODO : Now mC is only the mass of the first body but it is probably wrong
    // TODO : Now g is 9.81 but we should use the true gravity value of the physics world.
    double mu_mc_g = FRICTION_COEFFICIENT * rigidBody1->getMass().getValue() * 9.81;
    auxLowerBounds.setValue(0, -mu_mc_g);
    auxLowerBounds.setValue(1, -mu_mc_g);
    auxUpperBounds.setValue(0, mu_mc_g);
    auxUpperBounds.setValue(1, mu_mc_g);

    // Compute the error auxiliary values
    auxErrorValues.setValue(0, 0.0);
    auxErrorValues.setValue(1, 0.0);
;}

// TODO : Delete this (Used to debug collision detection)
void Contact::draw() const {
    glColor3f(1.0, 0.0, 0.0);
    glutSolidSphere(0.5, 20, 20);
}
