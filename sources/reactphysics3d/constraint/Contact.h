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

#ifndef CONTACT_H
#define CONTACT_H

// Libraries
#include "Constraint.h"
#include "../mathematics/mathematics.h"

// ReactPhysics3D namespace
namespace reactphysics3d {

// Constants
const unsigned int NB_FRICTION_VECTORS = 4;         // Number of vectors used to describe the contact friction

/*  -------------------------------------------------------------------
    Class Contact :
        This class represents a collision contact between two bodies in
        the physics engine. The contact class inherits from the
        Constraint class. The collision detection systems compute
        contact informations that will be used to perform the collision
        response.
    -------------------------------------------------------------------
*/
class Contact : public Constraint {
    protected :
        Vector3D normal;                                // Normal vector of the contact (From body1 toward body2)
        double penetrationDepth;                        // Penetration depth
        std::vector<Vector3D> points;                   // Contact points
        unsigned int nbFrictionVectors;                 // Number of vectors used to describe the friction
        // TODO : Implement the computation of the frictionVectors in the collision detection
        std::vector<Vector3D> frictionVectors;          // Set of vectors that span the tangential friction plane

        void computeFrictionVectors(const Vector3D& vector1, const Vector3D& vector2);      // Compute all the friction vectors from two vectors that span the friction plane

    public :
        Contact(Body* const body1, Body* const body2, const Vector3D& normal, double penetrationDepth, const std::vector<Vector3D>& points);    // Constructor
        virtual ~Contact();                                                                                                                     // Destructor

        Vector3D getNormal() const;                         // Return the normal vector of the contact
        std::vector<Vector3D> getPoints() const;            // Return the contact points
        virtual unsigned int getNbJacobianRows() const;     // Return the number of rows of the Jacobian matrix
        virtual void evaluate(double dt);                   // Evaluate the constraint
        virtual int getNbAuxiliaryVars() const;             // Return the number of auxiliary variables

        void draw() const;                              // TODO : Delete this (Used to debug collision detection)
};

// Compute all the friction vectors from two vectors that span the friction cone
// The vectors "vector1" and "vector2" are two vectors that span the friction cone
inline void Contact::computeFrictionVectors(const Vector3D& vector1, const Vector3D& vector2) {
    // Delete the current friction vectors
    frictionVectors.clear();

    Vector3D vector;

    // Compute each friction vector
    for (unsigned int i=1; i<=NB_FRICTION_VECTORS; ++i) {
        vector = cos((2.0 * (i-1) * PI) / NB_FRICTION_VECTORS) * vector1 + sin((2.0 * (i-1) * PI) / NB_FRICTION_VECTORS) * vector2;
        frictionVectors.push_back(vector);
    }
}

// Return the normal vector of the contact
inline Vector3D Contact::getNormal() const {
    return normal;
}

// Return the contact points
inline std::vector<Vector3D> Contact::getPoints() const {
    return points;
}

// Return the number of rows of the Jacobian matrix
inline unsigned int Contact::getNbJacobianRows() const {
    return (1+nbFrictionVectors);
}

// Return the number of auxiliary variables
inline int Contact::getNbAuxiliaryVars() const {
    return nbFrictionVectors;
}


} // End of the ReactPhysics3D namespace

#endif
