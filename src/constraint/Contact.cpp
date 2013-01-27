/********************************************************************************
* ReactPhysics3D physics library, http://code.google.com/p/reactphysics3d/      *
* Copyright (c) 2010-2012 Daniel Chappuis                                       *
*********************************************************************************
*                                                                               *
* This software is provided 'as-is', without any express or implied warranty.   *
* In no event will the authors be held liable for any damages arising from the  *
* use of this software.                                                         *
*                                                                               *
* Permission is granted to anyone to use this software for any purpose,         *
* including commercial applications, and to alter it and redistribute it        *
* freely, subject to the following restrictions:                                *
*                                                                               *
* 1. The origin of this software must not be misrepresented; you must not claim *
*    that you wrote the original software. If you use this software in a        *
*    product, an acknowledgment in the product documentation would be           *
*    appreciated but is not required.                                           *
*                                                                               *
* 2. Altered source versions must be plainly marked as such, and must not be    *
*    misrepresented as being the original software.                             *
*                                                                               *
* 3. This notice may not be removed or altered from any source distribution.    *
*                                                                               *
********************************************************************************/

// Libraries
#include "Contact.h"

using namespace reactphysics3d;
using namespace std;

// Constructor
Contact::Contact(RigidBody* const body1, RigidBody* const body2, const ContactInfo* contactInfo)
        : Constraint(body1, body2, 3, true, CONTACT), mNormal(contactInfo->normal),
          mPenetrationDepth(contactInfo->penetrationDepth),
          mLocalPointOnBody1(contactInfo->localPoint1),
          mLocalPointOnBody2(contactInfo->localPoint2),
          mWorldPointOnBody1(body1->getTransform() * contactInfo->localPoint1),
          mWorldPointOnBody2(body2->getTransform() * contactInfo->localPoint2),
          mIsRestingContact(false) {
    assert(mPenetrationDepth > 0.0);
    
    // Compute the auxiliary lower and upper bounds
    // TODO : Now mC is only the mass of the first body but it is probably wrong
    // TODO : Now g is 9.81 but we should use the true gravity value of the physics world.
    mMu_mc_g = FRICTION_COEFFICIENT * body1->getMass() * 9.81;

    // Compute the friction vectors that span the tangential friction plane
    computeFrictionVectors();
}

// Destructor
Contact::~Contact() {

}

void Contact::computeJacobianPenetration(decimal J_spBody1[6], decimal J_spBody2[6]) {

    Vector3 body1Position = mBody1->getTransform().getPosition();
    Vector3 body2Position = mBody2->getTransform().getPosition();

    Vector3 r1 = mWorldPointOnBody1 - body1Position;
    Vector3 r2 = mWorldPointOnBody2 - body2Position;
    Vector3 r1CrossN = r1.cross(mNormal);
    Vector3 r2CrossN = r2.cross(mNormal);

    // Compute the jacobian matrix for the body 1 for the contact constraint
    J_spBody1[0] = -mNormal.getX();
    J_spBody1[1] = -mNormal.getY();
    J_spBody1[2] = -mNormal.getZ();
    J_spBody1[3] = -r1CrossN.getX();
    J_spBody1[4] = -r1CrossN.getY();
    J_spBody1[5] = -r1CrossN.getZ();

    // Compute the jacobian matrix for the body 2 for the contact constraint
    J_spBody2[0] = mNormal.getX();
    J_spBody2[1] = mNormal.getY();
    J_spBody2[2] = mNormal.getZ();
    J_spBody2[3] = r2CrossN.getX();
    J_spBody2[4] = r2CrossN.getY();
    J_spBody2[5] = r2CrossN.getZ();
}

void Contact::computeJacobianFriction1(decimal J_spBody1[6], decimal J_spBody2[6]) {

    Vector3 body1Position = mBody1->getTransform().getPosition();
    Vector3 body2Position = mBody2->getTransform().getPosition();

    Vector3 r1 = mWorldPointOnBody1 - body1Position;
    Vector3 r2 = mWorldPointOnBody2 - body2Position;

    // Compute the jacobian matrix for the body 1 for the first friction constraint
    Vector3 r1CrossU1 = r1.cross(mFrictionVectors[0]);
    Vector3 r2CrossU1 = r2.cross(mFrictionVectors[0]);
    J_spBody1[0] = -mFrictionVectors[0].getX();
    J_spBody1[1] = -mFrictionVectors[0].getY();
    J_spBody1[2] = -mFrictionVectors[0].getZ();
    J_spBody1[3] = -r1CrossU1.getX();
    J_spBody1[4] = -r1CrossU1.getY();
    J_spBody1[5] = -r1CrossU1.getZ();

    // Compute the jacobian matrix for the body 2 for the first friction constraint
    J_spBody2[0] = mFrictionVectors[0].getX();
    J_spBody2[1] = mFrictionVectors[0].getY();
    J_spBody2[2] = mFrictionVectors[0].getZ();
    J_spBody2[3] = r2CrossU1.getX();
    J_spBody2[4] = r2CrossU1.getY();
    J_spBody2[5] = r2CrossU1.getZ();
}

void Contact::computeJacobianFriction2(decimal J_spBody1[6], decimal J_spBody2[6]) {

    Vector3 body1Position = mBody1->getTransform().getPosition();
    Vector3 body2Position = mBody2->getTransform().getPosition();

    Vector3 r1 = mWorldPointOnBody1 - body1Position;
    Vector3 r2 = mWorldPointOnBody2 - body2Position;

    Vector3 r1CrossU2 = r1.cross(mFrictionVectors[1]);
    Vector3 r2CrossU2 = r2.cross(mFrictionVectors[1]);

    // Compute the jacobian matrix for the body 1 for the second friction constraint
    J_spBody1[0] = -mFrictionVectors[1].getX();
    J_spBody1[1] = -mFrictionVectors[1].getY();
    J_spBody1[2] = -mFrictionVectors[1].getZ();
    J_spBody1[3] = -r1CrossU2.getX();
    J_spBody1[4] = -r1CrossU2.getY();
    J_spBody1[5] = -r1CrossU2.getZ();

    // Compute the jacobian matrix for the body 2 for the second friction constraint
    J_spBody2[0] = mFrictionVectors[1].getX();
    J_spBody2[1] = mFrictionVectors[1].getY();
    J_spBody2[2] = mFrictionVectors[1].getZ();
    J_spBody2[3] = r2CrossU2.getX();
    J_spBody2[4] = r2CrossU2.getY();
    J_spBody2[5] = r2CrossU2.getZ();
}

void Contact::computeLowerBoundPenetration(decimal& lowerBound) {
    lowerBound = 0.0;
}

void Contact::computeLowerBoundFriction1(decimal& lowerBound) {
    lowerBound = -mMu_mc_g;
}

void Contact::computeLowerBoundFriction2(decimal& lowerBound) {
    lowerBound = -mMu_mc_g;
}

void Contact::computeUpperBoundPenetration(decimal& upperBound) {
    upperBound = DECIMAL_INFINITY;
}

void Contact::computeUpperBoundFriction1(decimal& upperBound) {
    upperBound = mMu_mc_g;
}

void Contact::computeUpperBoundFriction2(decimal& upperBound) {
    upperBound = mMu_mc_g;
}

// This method computes the jacobian matrix for all mathematical constraints
// The argument "J_sp" is the jacobian matrix of the constraint solver. This method
// fill in this matrix with all the jacobian matrix of the mathematical constraint
// of the contact. The argument "noConstraint", is the row were the method have
// to start to fill in the J_sp matrix.
void Contact::computeJacobian(int noConstraint, decimal J_sp[NB_MAX_CONSTRAINTS][2*6]) const {
    assert(mBody1);
    assert(mBody2);

    Vector3 body1Position = mBody1->getTransform().getPosition();
    Vector3 body2Position = mBody2->getTransform().getPosition();
    int currentIndex = noConstraint;                        // Current constraint index

    Vector3 r1 = mWorldPointOnBody1 - body1Position;
    Vector3 r2 = mWorldPointOnBody2 - body2Position;
    Vector3 r1CrossN = r1.cross(mNormal);
    Vector3 r2CrossN = r2.cross(mNormal);

    // Compute the jacobian matrix for the body 1 for the contact constraint
    J_sp[currentIndex][0] = -mNormal.getX();
    J_sp[currentIndex][1] = -mNormal.getY();
    J_sp[currentIndex][2] = -mNormal.getZ();
    J_sp[currentIndex][3] = -r1CrossN.getX();
    J_sp[currentIndex][4] = -r1CrossN.getY();
    J_sp[currentIndex][5] = -r1CrossN.getZ();

    // Compute the jacobian matrix for the body 2 for the contact constraint
    J_sp[currentIndex][6] = mNormal.getX();
    J_sp[currentIndex][7] = mNormal.getY();
    J_sp[currentIndex][8] = mNormal.getZ();
    J_sp[currentIndex][9] = r2CrossN.getX();
    J_sp[currentIndex][10] = r2CrossN.getY();
    J_sp[currentIndex][11] = r2CrossN.getZ();

    currentIndex++;

    // Compute the jacobian matrix for the body 1 for the first friction constraint
    Vector3 r1CrossU1 = r1.cross(mFrictionVectors[0]);
    Vector3 r2CrossU1 = r2.cross(mFrictionVectors[0]);
    Vector3 r1CrossU2 = r1.cross(mFrictionVectors[1]);
    Vector3 r2CrossU2 = r2.cross(mFrictionVectors[1]);
    J_sp[currentIndex][0] = -mFrictionVectors[0].getX();
    J_sp[currentIndex][1] = -mFrictionVectors[0].getY();
    J_sp[currentIndex][2] = -mFrictionVectors[0].getZ();
    J_sp[currentIndex][3] = -r1CrossU1.getX();
    J_sp[currentIndex][4] = -r1CrossU1.getY();
    J_sp[currentIndex][5] = -r1CrossU1.getZ();

    // Compute the jacobian matrix for the body 2 for the first friction constraint
    J_sp[currentIndex][6] = mFrictionVectors[0].getX();
    J_sp[currentIndex][7] = mFrictionVectors[0].getY();
    J_sp[currentIndex][8] = mFrictionVectors[0].getZ();
    J_sp[currentIndex][9] = r2CrossU1.getX();
    J_sp[currentIndex][10] = r2CrossU1.getY();
    J_sp[currentIndex][11] = r2CrossU1.getZ();

    currentIndex++;

    // Compute the jacobian matrix for the body 1 for the second friction constraint
    J_sp[currentIndex][0] = -mFrictionVectors[1].getX();
    J_sp[currentIndex][1] = -mFrictionVectors[1].getY();
    J_sp[currentIndex][2] = -mFrictionVectors[1].getZ();
    J_sp[currentIndex][3] = -r1CrossU2.getX();
    J_sp[currentIndex][4] = -r1CrossU2.getY();
    J_sp[currentIndex][5] = -r1CrossU2.getZ();

    // Compute the jacobian matrix for the body 2 for the second friction constraint
    J_sp[currentIndex][6] = mFrictionVectors[1].getX();
    J_sp[currentIndex][7] = mFrictionVectors[1].getY();
    J_sp[currentIndex][8] = mFrictionVectors[1].getZ();
    J_sp[currentIndex][9] = r2CrossU2.getX();
    J_sp[currentIndex][10] = r2CrossU2.getY();
    J_sp[currentIndex][11] = r2CrossU2.getZ();
}

// Compute the lowerbounds values for all the mathematical constraints. The
// argument "lowerBounds" is the lowerbounds values vector of the constraint solver and
// this methods has to fill in this vector starting from the row "noConstraint"
void Contact::computeLowerBound(int noConstraint, decimal lowerBounds[NB_MAX_CONSTRAINTS]) const {
    assert(noConstraint >= 0 && noConstraint + mNbConstraints <= NB_MAX_CONSTRAINTS);

    lowerBounds[noConstraint] = 0.0;            // Lower bound for the contact constraint
    lowerBounds[noConstraint + 1] = -mMu_mc_g;  // Lower bound for the first friction constraint
    lowerBounds[noConstraint + 2] = -mMu_mc_g;  // Lower bound for the second friction constraint
}

// Compute the upperbounds values for all the mathematical constraints. The
// argument "upperBounds" is the upperbounds values vector of the constraint solver and
// this methods has to fill in this vector starting from the row "noConstraint"
void Contact::computeUpperBound(int noConstraint, decimal upperBounds[NB_MAX_CONSTRAINTS]) const {
    assert(noConstraint >= 0 && noConstraint + mNbConstraints <= NB_MAX_CONSTRAINTS);

    upperBounds[noConstraint] = DECIMAL_INFINITY; // Upper bound for the contact constraint
    upperBounds[noConstraint + 1] = mMu_mc_g;     // Upper bound for the first friction constraint
    upperBounds[noConstraint + 2] = mMu_mc_g;     // Upper bound for the second friction constraint
}

void Contact::computeErrorPenetration(decimal& error) {
    // TODO : Do we need this casting anymore ?
    RigidBody* rigidBody1 = dynamic_cast<RigidBody*>(mBody1);
    RigidBody* rigidBody2 = dynamic_cast<RigidBody*>(mBody2);

    // Compute the error value for the contact constraint
    Vector3 velocity1 = rigidBody1->getLinearVelocity();
    Vector3 velocity2 = rigidBody2->getLinearVelocity();
    decimal restitutionCoeff = rigidBody1->getRestitution() * rigidBody2->getRestitution();
    error = restitutionCoeff * (mNormal.dot(velocity1) - mNormal.dot(velocity2));
}

// Compute the error values for all the mathematical constraints. The argument
// "errorValues" is the error values vector of the constraint solver and this
// method has to fill in this vector starting from the row "noConstraint"
void Contact::computeErrorValue(int noConstraint, decimal errorValues[]) const {
    assert(mBody1);
    assert(mBody2);

	// TODO : Do we need this casting anymore ?
    RigidBody* rigidBody1 = dynamic_cast<RigidBody*>(mBody1);
    RigidBody* rigidBody2 = dynamic_cast<RigidBody*>(mBody2);

    assert(noConstraint >= 0 && noConstraint + mNbConstraints <= NB_MAX_CONSTRAINTS);

    // Compute the error value for the contact constraint
    Vector3 velocity1 = rigidBody1->getLinearVelocity();
    Vector3 velocity2 = rigidBody2->getLinearVelocity();
    decimal restitutionCoeff = rigidBody1->getRestitution() * rigidBody2->getRestitution();
    decimal errorValue = restitutionCoeff * (mNormal.dot(velocity1) - mNormal.dot(velocity2));
    
    // Assign the error value to the vector of error values
    errorValues[noConstraint] = errorValue;    // Error value for contact constraint
    errorValues[noConstraint + 1] = 0.0;       // Error value for friction constraint
    errorValues[noConstraint + 2] = 0.0;       // Error value for friction constraint
}
