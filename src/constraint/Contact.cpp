/********************************************************************************
* ReactPhysics3D physics library, http://code.google.com/p/reactphysics3d/      *
* Copyright (c) 2010 Daniel Chappuis                                            *
*********************************************************************************
*                                                                               *
* Permission is hereby granted, free of charge, to any person obtaining a copy  *
* of this software and associated documentation files (the "Software"), to deal *
* in the Software without restriction, including without limitation the rights  *
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell     *
* copies of the Software, and to permit persons to whom the Software is         *
* furnished to do so, subject to the following conditions:                      *
*                                                                               *
* The above copyright notice and this permission notice shall be included in    *
* all copies or substantial portions of the Software.                           *
*                                                                               *
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR    *
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,      *
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE   *
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER        *
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, *
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN     *
* THE SOFTWARE.                                                                 *
********************************************************************************/

// Libraries
#include "Contact.h"

using namespace reactphysics3d;
using namespace std;

// Constructor
Contact::Contact(Body* const body1, Body* const body2, const Vector3D& normal, double penetrationDepth, const vector<Vector3D>& points)
                 :Constraint(body1, body2, 3*points.size(), true), normal(normal), penetrationDepth(penetrationDepth), points(points), nbPoints(points.size()) {

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

// This method computes the jacobian matrix for all mathematical constraints
// The argument "J_sp" is the jacobian matrix of the constraint solver. This method
// fill in this matrix with all the jacobian matrix of the mathematical constraint
// of the contact. The argument "noConstraint", is the row were the method have
// to start to fill in the J_sp matrix.
void Contact::computeJacobian(int noConstraint, Matrix1x6**& J_sp) const {
    RigidBody* rigidBody1 = dynamic_cast<RigidBody*>(body1);
    RigidBody* rigidBody2 = dynamic_cast<RigidBody*>(body2);
    Vector3D r1;
    Vector3D r2;
    Vector3D r1CrossN;
    Vector3D r2CrossN;
    Vector3D r1CrossU1;
    Vector3D r2CrossU1;
    Vector3D r1CrossU2;
    Vector3D r2CrossU2;
    Vector3D body1Position = rigidBody1->getPosition();
    Vector3D body2Position = rigidBody2->getPosition();
    int currentIndex = noConstraint;                        // Current constraint index

    assert(rigidBody1);
    assert(rigidBody2);

    // For each point in the contact
    for (int i=0; i<nbPoints; i++) {

        r1 = points[i] - body1Position;
        r2 = points[i] - body2Position;
        r1CrossN = r1.cross(normal);
        r2CrossN = r2.cross(normal);

        // Compute the jacobian matrix for the body 1 for the contact constraint
        //J_sp[currentIndex][0].changeSize(1, 6);
        J_sp[currentIndex][0].setValue(0, -normal.getX());
        J_sp[currentIndex][0].setValue(1, -normal.getY());
        J_sp[currentIndex][0].setValue(2, -normal.getZ());
        J_sp[currentIndex][0].setValue(3, -r1CrossN.getX());
        J_sp[currentIndex][0].setValue(4, -r1CrossN.getY());
        J_sp[currentIndex][0].setValue(5, -r1CrossN.getZ());

        // Compute the jacobian matrix for the body 2 for the contact constraint
        //J_sp[currentIndex][1].changeSize(1, 6);
        J_sp[currentIndex][1].setValue(0, normal.getX());
        J_sp[currentIndex][1].setValue(1, normal.getY());
        J_sp[currentIndex][1].setValue(2, normal.getZ());
        J_sp[currentIndex][1].setValue(3, r2CrossN.getX());
        J_sp[currentIndex][1].setValue(4, r2CrossN.getY());
        J_sp[currentIndex][1].setValue(5, r2CrossN.getZ());

        currentIndex++;

        // Compute the jacobian matrix for the body 1 for the first friction constraint
        r1CrossU1 = r1.cross(frictionVectors[0]);
        r2CrossU1 = r2.cross(frictionVectors[0]);
        r1CrossU2 = r1.cross(frictionVectors[1]);
        r2CrossU2 = r2.cross(frictionVectors[1]);
        //J_sp[currentIndex][0].changeSize(1, 6);
        J_sp[currentIndex][0].setValue(0, -frictionVectors[0].getX());
        J_sp[currentIndex][0].setValue(1, -frictionVectors[0].getY());
        J_sp[currentIndex][0].setValue(2, -frictionVectors[0].getZ());
        J_sp[currentIndex][0].setValue(3, -r1CrossU1.getX());
        J_sp[currentIndex][0].setValue(4, -r1CrossU1.getY());
        J_sp[currentIndex][0].setValue(5, -r1CrossU1.getZ());

        // Compute the jacobian matrix for the body 2 for the first friction constraint
        //J_sp[currentIndex][1].changeSize(1, 6);
        J_sp[currentIndex][1].setValue(0, frictionVectors[0].getX());
        J_sp[currentIndex][1].setValue(1, frictionVectors[0].getY());
        J_sp[currentIndex][1].setValue(2, frictionVectors[0].getZ());
        J_sp[currentIndex][1].setValue(3, r2CrossU1.getX());
        J_sp[currentIndex][1].setValue(4, r2CrossU1.getY());
        J_sp[currentIndex][1].setValue(5, r2CrossU1.getZ());

        currentIndex++;

        // Compute the jacobian matrix for the body 1 for the second friction constraint
        //J_sp[currentIndex][0].changeSize(1, 6);
        J_sp[currentIndex][0].setValue(0, -frictionVectors[1].getX());
        J_sp[currentIndex][0].setValue(1, -frictionVectors[1].getY());
        J_sp[currentIndex][0].setValue(2, -frictionVectors[1].getZ());
        J_sp[currentIndex][0].setValue(3, -r1CrossU2.getX());
        J_sp[currentIndex][0].setValue(4, -r1CrossU2.getY());
        J_sp[currentIndex][0].setValue(5, -r1CrossU2.getZ());
        //J_sp[currentIndex][1].changeSize(1, 6);

        // Compute the jacobian matrix for the body 2 for the second friction constraint
        J_sp[currentIndex][1].setValue(0, frictionVectors[1].getX());
        J_sp[currentIndex][1].setValue(1, frictionVectors[1].getY());
        J_sp[currentIndex][1].setValue(2, frictionVectors[1].getZ());
        J_sp[currentIndex][1].setValue(3, r2CrossU2.getX());
        J_sp[currentIndex][1].setValue(4, r2CrossU2.getY());
        J_sp[currentIndex][1].setValue(5, r2CrossU2.getZ());

        currentIndex++;
    }
}

// Compute the lowerbounds values for all the mathematical constraints. The
// argument "lowerBounds" is the lowerbounds values vector of the constraint solver and
// this methods has to fill in this vector starting from the row "noConstraint"
void Contact::computeLowerBound(int noConstraint, Vector& lowerBounds) const {
    int index = noConstraint;

    assert(noConstraint >= 0 && noConstraint + nbConstraints <= lowerBounds.getNbComponent());

    // For each constraint
    for (int i=0; i<nbPoints; i++) {
        lowerBounds.setValue(index, 0.0);           // Lower bound for the contact constraint
        lowerBounds.setValue(index + 1, -mu_mc_g);      // Lower bound for the first friction constraint
        lowerBounds.setValue(index + 2, -mu_mc_g);      // Lower bound for the second friction constraint
        index += 3;
    }
}

// Compute the upperbounds values for all the mathematical constraints. The
// argument "upperBounds" is the upperbounds values vector of the constraint solver and
// this methods has to fill in this vector starting from the row "noConstraint"
void Contact::computeUpperBound(int noConstraint, Vector& upperBounds) const {
    int index = noConstraint;

    assert(noConstraint >= 0 && noConstraint + nbConstraints <= upperBounds.getNbComponent());

    // For each constraint
    for (int i=0; i<nbPoints; i++) {
        upperBounds.setValue(index, INFINITY_CONST);    // Upper bound for the contact constraint
        upperBounds.setValue(index + 1, mu_mc_g);       // Upper bound for the first friction constraint
        upperBounds.setValue(index + 2, mu_mc_g);       // Upper bound for the second friction constraint
        index += 3;
    }
}

// Compute the error values for all the mathematical constraints. The argument
// "errorValues" is the error values vector of the constraint solver and this
// method has to fill in this vector starting from the row "noConstraint"
void Contact::computeErrorValue(int noConstraint, Vector& errorValues) const {
    RigidBody* rigidBody1 = dynamic_cast<RigidBody*>(body1);
    RigidBody* rigidBody2 = dynamic_cast<RigidBody*>(body2);
    int index = noConstraint;

    assert(rigidBody1);
    assert(rigidBody2);
    assert(noConstraint >= 0 && noConstraint + nbConstraints <= errorValues.getNbComponent());

    // Compute the error value for the contact constraint
    Vector3D velocity1 = rigidBody1->getLinearVelocity();
    Vector3D velocity2 = rigidBody2->getLinearVelocity();
    double restitutionCoeff = rigidBody1->getRestitution() * rigidBody2->getRestitution();
    double errorValue = restitutionCoeff * (normal.dot(velocity1) - normal.dot(velocity2)) + PENETRATION_FACTOR * penetrationDepth;

    // Assign the error value to the vector of error values
    for (int i=0; i<nbPoints; i++) {
        errorValues.setValue(index, errorValue);    // Error value for contact constraint
        errorValues.setValue(index + 1, 0.0);       // Error value for friction constraint
        errorValues.setValue(index + 2, 0.0);       // Error value for friction constraint
        index += 3;
    }
}