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
        Constraint class. Each Contact represent a contact between two bodies
        and can have several contact points. The Contact will have 3 mathematical
        constraints for each contact point (1 for the contact constraint, and 2
        for the friction constraints).
    -------------------------------------------------------------------
*/
class Contact : public Constraint {
    protected :
        const Vector3D normal;                          // Normal vector of the contact (From body1 toward body2)
        const double penetrationDepth;                  // Penetration depth
        const std::vector<Vector3D> points;             // Contact points between the two bodies
        const int nbPoints;                             // Number of points in the contact
        std::vector<Vector3D> frictionVectors;          // Two orthogonal vectors that span the tangential friction plane
        double mu_mc_g;
        
        void computeFrictionVectors();                  // Compute the two friction vectors that span the tangential friction plane

    public :
        Contact(Body* const body1, Body* const body2, const Vector3D& normal, double penetrationDepth, const std::vector<Vector3D>& points);      // Constructor
        virtual ~Contact();                                                                                                                             // Destructor

        Vector3D getNormal() const;                                                     // Return the normal vector of the contact
        Vector3D getPoint(int index) const;                                             // Return a contact point
        int getNbPoints() const;                                                        // Return the number of contact points
        virtual void computeJacobian(int noConstraint, Matrix1x6**& J_SP) const;           // Compute the jacobian matrix for all mathematical constraints
        virtual void computeLowerBound(int noConstraint, Vector& lowerBounds) const;    // Compute the lowerbounds values for all the mathematical constraints
        virtual void computeUpperBound(int noConstraint, Vector& upperBounds) const;    // Compute the upperbounds values for all the mathematical constraints
        virtual void computeErrorValue(int noConstraint, Vector& errorValues) const;    // Compute the error values for all the mathematical constraints
        double getPenetrationDepth() const;                                             // Return the penetration depth
        void draw() const;                                                              // TODO : Delete this (Used to debug collision detection)
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

// Return a contact points
inline Vector3D Contact::getPoint(int index) const {
    assert(index >= 0 && index < nbPoints);
    return points[index];
}

// Return the number of contact points
inline int Contact::getNbPoints() const {
    return nbPoints;
}

// Return the penetration depth of the contact
inline double Contact::getPenetrationDepth() const {
    return penetrationDepth;
}

// TODO : Delete this (Used to debug collision detection)
inline void Contact::draw() const {
    glColor3f(1.0, 0.0, 0.0);
    glutSolidSphere(0.3, 20, 20);
}

} // End of the ReactPhysics3D namespace

#endif
