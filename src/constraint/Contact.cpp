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
Contact::Contact(const ContactInfo* contactInfo)
        : Constraint(contactInfo->body1, contactInfo->body2, 3, true), normal(contactInfo->normal), penetrationDepth(contactInfo->penetrationDepth),
          localPointOnBody1(contactInfo->localPoint1), localPointOnBody2(contactInfo->localPoint2),
          worldPointOnBody1(contactInfo->worldPoint1), worldPointOnBody2(contactInfo->worldPoint2) {
    assert(penetrationDepth > 0.0);
    
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
void Contact::computeJacobian(int noConstraint, double J_sp[NB_MAX_CONSTRAINTS][2*6]) const {
    assert(body1);
    assert(body2);

    Vector3 body1Position = body1->getTransform().getPosition();
    Vector3 body2Position = body2->getTransform().getPosition();
    int currentIndex = noConstraint;                        // Current constraint index

    Vector3 r1 = worldPointOnBody1 - body1Position;
    Vector3 r2 = worldPointOnBody2 - body2Position;
    Vector3 r1CrossN = r1.cross(normal);
    Vector3 r2CrossN = r2.cross(normal);

    // Compute the jacobian matrix for the body 1 for the contact constraint
    J_sp[currentIndex][0] = -normal.getX();
    J_sp[currentIndex][1] = -normal.getY();
    J_sp[currentIndex][2] = -normal.getZ();
    J_sp[currentIndex][3] = -r1CrossN.getX();
    J_sp[currentIndex][4] = -r1CrossN.getY();
    J_sp[currentIndex][5] = -r1CrossN.getZ();

    // Compute the jacobian matrix for the body 2 for the contact constraint
    J_sp[currentIndex][6] = normal.getX();
    J_sp[currentIndex][7] = normal.getY();
    J_sp[currentIndex][8] = normal.getZ();
    J_sp[currentIndex][9] = r2CrossN.getX();
    J_sp[currentIndex][10] = r2CrossN.getY();
    J_sp[currentIndex][11] = r2CrossN.getZ();

    currentIndex++;

    // Compute the jacobian matrix for the body 1 for the first friction constraint
    Vector3 r1CrossU1 = r1.cross(frictionVectors[0]);
    Vector3 r2CrossU1 = r2.cross(frictionVectors[0]);
    Vector3 r1CrossU2 = r1.cross(frictionVectors[1]);
    Vector3 r2CrossU2 = r2.cross(frictionVectors[1]);
    J_sp[currentIndex][0] = -frictionVectors[0].getX();
    J_sp[currentIndex][1] = -frictionVectors[0].getY();
    J_sp[currentIndex][2] = -frictionVectors[0].getZ();
    J_sp[currentIndex][3] = -r1CrossU1.getX();
    J_sp[currentIndex][4] = -r1CrossU1.getY();
    J_sp[currentIndex][5] = -r1CrossU1.getZ();

    // Compute the jacobian matrix for the body 2 for the first friction constraint
    J_sp[currentIndex][6] = frictionVectors[0].getX();
    J_sp[currentIndex][7] = frictionVectors[0].getY();
    J_sp[currentIndex][8] = frictionVectors[0].getZ();
    J_sp[currentIndex][9] = r2CrossU1.getX();
    J_sp[currentIndex][10] = r2CrossU1.getY();
    J_sp[currentIndex][11] = r2CrossU1.getZ();

    currentIndex++;

    // Compute the jacobian matrix for the body 1 for the second friction constraint
    J_sp[currentIndex][0] = -frictionVectors[1].getX();
    J_sp[currentIndex][1] = -frictionVectors[1].getY();
    J_sp[currentIndex][2] = -frictionVectors[1].getZ();
    J_sp[currentIndex][3] = -r1CrossU2.getX();
    J_sp[currentIndex][4] = -r1CrossU2.getY();
    J_sp[currentIndex][5] = -r1CrossU2.getZ();

    // Compute the jacobian matrix for the body 2 for the second friction constraint
    J_sp[currentIndex][6] = frictionVectors[1].getX();
    J_sp[currentIndex][7] = frictionVectors[1].getY();
    J_sp[currentIndex][8] = frictionVectors[1].getZ();
    J_sp[currentIndex][9] = r2CrossU2.getX();
    J_sp[currentIndex][10] = r2CrossU2.getY();
    J_sp[currentIndex][11] = r2CrossU2.getZ();
}

// Compute the lowerbounds values for all the mathematical constraints. The
// argument "lowerBounds" is the lowerbounds values vector of the constraint solver and
// this methods has to fill in this vector starting from the row "noConstraint"
void Contact::computeLowerBound(int noConstraint, double lowerBounds[NB_MAX_CONSTRAINTS]) const {
    assert(noConstraint >= 0 && noConstraint + nbConstraints <= NB_MAX_CONSTRAINTS);

    lowerBounds[noConstraint] = 0.0;                // Lower bound for the contact constraint
    lowerBounds[noConstraint + 1] = -mu_mc_g;       // Lower bound for the first friction constraint
    lowerBounds[noConstraint + 2] = -mu_mc_g;       // Lower bound for the second friction constraint
}

// Compute the upperbounds values for all the mathematical constraints. The
// argument "upperBounds" is the upperbounds values vector of the constraint solver and
// this methods has to fill in this vector starting from the row "noConstraint"
void Contact::computeUpperBound(int noConstraint, double upperBounds[NB_MAX_CONSTRAINTS]) const {
    assert(noConstraint >= 0 && noConstraint + nbConstraints <= NB_MAX_CONSTRAINTS);

    upperBounds[noConstraint] = INFINITY_CONST;    // Upper bound for the contact constraint
    upperBounds[noConstraint + 1] = mu_mc_g;       // Upper bound for the first friction constraint
    upperBounds[noConstraint + 2] = mu_mc_g;       // Upper bound for the second friction constraint
}

// Compute the error values for all the mathematical constraints. The argument
// "errorValues" is the error values vector of the constraint solver and this
// method has to fill in this vector starting from the row "noConstraint"
void Contact::computeErrorValue(int noConstraint, double errorValues[], double penetrationFactor) const {
    assert(body1);
    assert(body2);

	// TODO : Do we need this casting anymore
    RigidBody* rigidBody1 = dynamic_cast<RigidBody*>(body1);
    RigidBody* rigidBody2 = dynamic_cast<RigidBody*>(body2);

    assert(noConstraint >= 0 && noConstraint + nbConstraints <= NB_MAX_CONSTRAINTS);

    // Compute the error value for the contact constraint
    Vector3 velocity1 = rigidBody1->getLinearVelocity();
    Vector3 velocity2 = rigidBody2->getLinearVelocity();
    double restitutionCoeff = rigidBody1->getRestitution() * rigidBody2->getRestitution();
    double errorValue = restitutionCoeff * (normal.dot(velocity1) - normal.dot(velocity2)) + penetrationFactor * penetrationDepth;

    // Assign the error value to the vector of error values
    errorValues[noConstraint] = errorValue;    // Error value for contact constraint
    errorValues[noConstraint + 1] = 0.0;       // Error value for friction constraint
    errorValues[noConstraint + 2] = 0.0;       // Error value for friction constraint
}