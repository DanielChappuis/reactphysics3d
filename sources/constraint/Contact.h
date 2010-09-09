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

#ifndef CONTACT_H
#define CONTACT_H

// Libraries
#include "../typeDefinitions.h"
#include "Constraint.h"
#include "../body/RigidBody.h"
#include "../mathematics/mathematics.h"
#include <GL/freeglut.h>        // TODO : Remove this in the final version
#include <GL/gl.h>              // TODO : Remove this in the final version

// ReactPhysics3D namespace
namespace reactphysics3d {

// Constants
const double FRICTION_COEFFICIENT = 0.3;    // Friction coefficient
const double PENETRATION_FACTOR = 0.0;      // Penetration factor (between 0 and 1) which specify the importance of the
                                            // penetration depth in order to calculate the correct impulse for the contact

/*  -------------------------------------------------------------------
    Class Contact :
        This class represents a collision contact between two bodies in
        the physics engine. The contact class inherits from the
        Constraint class. The collision detection system computes
        contact informations that will be used to perform the collision
        response. Each contact contains only one contact point. A contact
        constraint has two auxiliary constraints in order two represent
        the two friction forces at the contact surface.
    -------------------------------------------------------------------
*/
class Contact : public Constraint {
    protected :
        const Vector3D normal;                          // Normal vector of the contact (From body1 toward body2)
        const double penetrationDepth;                  // Penetration depth
        const Vector3D point;                           // Contact point
        std::vector<Vector3D> frictionVectors;          // Two orthogonal vectors that span the tangential friction plane
        double mu_mc_g;
        
        void computeFrictionVectors();                  // Compute the two friction vectors that span the tangential friction plane

    public :
        Contact(Body* const body1, Body* const body2, const Vector3D& normal, double penetrationDepth, const Vector3D& point);    // Constructor
        virtual ~Contact();                                                                                                       // Destructor

        Vector3D getNormal() const;                                                                     // Return the normal vector of the contact
        Vector3D getPoint() const;                                                                      // Return the contact point
        virtual void computeJacobian(int noBody, Matrix& jacobian) const;                               // Compute a part of the jacobian for a given body
        virtual void computeAuxJacobian(int noBody, int noAuxConstraint, Matrix& auxJacobian) const;    // Compute a part of the jacobian for an auxiliary constraint
        virtual double computeLowerBound() const;                                                       // Compute the lowerbound of the constraint
        virtual double computeUpperBound() const;                                                       // Compute the upperbound of the constraint
        virtual void computeAuxLowerBounds(int beginIndex, Vector& auxLowerBounds) const;               // Compute the lowerbounds for the auxiliary constraints
        virtual void computeAuxUpperBounds(int beginIndex, Vector& auxLowerBounds) const;               // Compute the upperbounds for the auxiliary constraints
        virtual double computeErrorValue() const;                                                       // Compute the error value for the constraint
        virtual void computeAuxErrorValues(int beginIndex, Vector& errorValues) const;                  // Compute the errors values of the auxiliary constraints
        uint getNbAuxConstraints() const;                                                               // Return the number of auxiliary constraints
        double getPenetrationDepth() const;                                                             // Return the penetration depth
        void draw() const;                                                                              // TODO : Delete this (Used to debug collision detection)
};

// Compute the two unit orthogonal vectors "v1" and "v2" that span the tangential friction plane
// The two vectors have to be such that : v1 x v2 = contactNormal
inline void Contact::computeFrictionVectors() {
    // Delete the current friction vectors
    frictionVectors.clear();

    // Compute the first orthogonal vector
    Vector3D vector1 = normal.getOneOrthogonalVector();
    frictionVectors.push_back(vector1);

    // Compute the second orthogonal vector using the cross product
    frictionVectors.push_back(normal.crossProduct(vector1));
}

// Return the normal vector of the contact
inline Vector3D Contact::getNormal() const {
    return normal;
}

// Return the contact points
inline Vector3D Contact::getPoint() const {
    return point;
}

// Return the penetration depth of the contact
inline double Contact::getPenetrationDepth() const {
    return penetrationDepth;
}

// This method computes a part of the jacobian matrix for a given body.
// The argument "noBody" is 1 or 2 and corresponds to which one of the two
// bodies of the constraint we will compute the jacobian part. The argument
// "jacobian" is a 1x6 jacobian matrix of the constraint corresponding to one of
// the two bodies of the constraint.
inline void Contact::computeJacobian(int noBody, Matrix& jacobian) const {
    RigidBody* rigidBody;
    Vector3D rCrossN;
    Vector3D r;
    Vector3D norm = normal;

    assert(noBody == 1 || noBody == 2);
    assert(jacobian.getNbRow() == 1 && jacobian.getNbColumn() == 6);

    if (noBody == 1) {
        rigidBody = dynamic_cast<RigidBody*>(body1);
        assert(rigidBody);
        r = point - rigidBody->getPosition();
        rCrossN = r.crossProduct(normal).getOpposite();
        norm = normal.getOpposite();
    }
    else {
        rigidBody = dynamic_cast<RigidBody*>(body2);
        assert(rigidBody);
        r = point - rigidBody->getPosition();
        rCrossN = r.crossProduct(normal);
    }

    // Compute the jacobian matrix for the body 1
    jacobian.setValue(0, 0, norm.getX());
    jacobian.setValue(0, 1, norm.getY());
    jacobian.setValue(0, 2, norm.getZ());
    jacobian.setValue(0, 3, rCrossN.getX());
    jacobian.setValue(0, 4, rCrossN.getY());
    jacobian.setValue(0, 5, rCrossN.getZ());
}

// Compute a part of the jacobian matrix for an auxiliary constraint (given by "noAuxConstraint")
// and one of the two bodies (given by "noBody") of the contact. The argument "noBody" is 1 or 2 and
// argument auxJacobian is a 1x6 matrix.
inline void Contact::computeAuxJacobian(int noBody, int noAuxConstraint, Matrix& auxJacobian) const {
    Vector3D r;
    Vector3D rCrossU;
    RigidBody* rigidBody;
    double sign;

    assert(noBody == 1 || noBody == 2);
    assert(noAuxConstraint == 1 || noAuxConstraint == 2);
    assert(auxJacobian.getNbRow() == 1 && auxJacobian.getNbColumn() == 6);

    if (noBody == 1) {
        rigidBody = dynamic_cast<RigidBody*>(body1);
        assert(rigidBody);
        r = point - rigidBody->getPosition();
        sign = -1.0;
    }
    else {
       rigidBody = dynamic_cast<RigidBody*>(body2);
       assert(rigidBody);
       r = point - rigidBody->getPosition();
       sign = 1.0;
    }

    rCrossU = r.crossProduct(frictionVectors[noAuxConstraint-1]);

    auxJacobian.setValue(0, 0, sign * frictionVectors[noAuxConstraint-1].getX());
    auxJacobian.setValue(0, 1, sign * frictionVectors[noAuxConstraint-1].getY());
    auxJacobian.setValue(0, 2, sign * frictionVectors[noAuxConstraint-1].getZ());
    auxJacobian.setValue(0, 3, sign * rCrossU.getX());
    auxJacobian.setValue(0, 4, sign * rCrossU.getY());
    auxJacobian.setValue(0, 5, sign * rCrossU.getZ());
}

// Compute the lowerbounds for the auxiliary constraints
inline double Contact::computeLowerBound() const {
    return 0.0;
}

// Compute the upperbounds for the auxiliary constraints
inline double Contact::computeUpperBound() const {
    return INFINITY_CONST;
}

// Compute the lowerbounds for the auxiliary constraints. This method fills the "auxLowerBounds"
// vector starting at the index "beginIndex" in this vector.
inline void Contact::computeAuxLowerBounds(int beginIndex, Vector& auxLowerBounds) const {
    assert(beginIndex + nbAuxConstraints <= auxLowerBounds.getNbComponent());
    
    auxLowerBounds.setValue(beginIndex, -mu_mc_g);
    auxLowerBounds.setValue(beginIndex + 1, -mu_mc_g);
    
}

// Compute the upperbounds for the auxiliary constraints. This method fills the "auxUpperBounds"
// vector starting at the index "beginIndex" in this vector.
inline void Contact::computeAuxUpperBounds(int beginIndex, Vector& auxUpperBounds) const {
    assert(beginIndex + nbAuxConstraints <= auxUpperBounds.getNbComponent());
    
    auxUpperBounds.setValue(beginIndex, mu_mc_g);
    auxUpperBounds.setValue(beginIndex + 1, mu_mc_g);
}

// Compute the error value for the constraint
inline double Contact::computeErrorValue() const {
    RigidBody* rigidBody1 = dynamic_cast<RigidBody*>(body1);
    RigidBody* rigidBody2 = dynamic_cast<RigidBody*>(body2);

    assert(rigidBody1);
    assert(rigidBody2);
    
    Vector3D velocity1 = rigidBody1->getLinearVelocity();
    Vector3D velocity2 = rigidBody2->getLinearVelocity();
    double restitutionCoeff = rigidBody1->getRestitution() * rigidBody2->getRestitution();
    double errorValue = restitutionCoeff * (normal.scalarProduct(velocity1) - normal.scalarProduct(velocity2)) + PENETRATION_FACTOR * penetrationDepth;
    return errorValue;
}

// Compute the errors values of the auxiliary constraints
inline void Contact::computeAuxErrorValues(int beginIndex, Vector& errorValues) const {
    assert(beginIndex + nbAuxConstraints <= errorValues.getNbComponent());
    
    errorValues.setValue(beginIndex, 0.0);
    errorValues.setValue(beginIndex + 1, 0.0);
}

// TODO : Delete this (Used to debug collision detection)
inline void Contact::draw() const {
    glColor3f(1.0, 0.0, 0.0);
    glutSolidSphere(0.3, 20, 20);
}

} // End of the ReactPhysics3D namespace

#endif
