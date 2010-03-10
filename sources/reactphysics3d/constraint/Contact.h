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

/*  -------------------------------------------------------------------
    Class Contact :
        This class represents a collision contact between two bodies in
        the physics engine. The contact class inherits from the
        Constraint class. The collision detection system computes
        contact informations that will be used to perform the collision
        response. A contact constraint has two auxiliary constraints in
        order two represent the two friction forces at the contact
        surface.
    -------------------------------------------------------------------
*/
class Contact : public Constraint {
    protected :
        Vector3D normal;                                // Normal vector of the contact (From body1 toward body2)
        double penetrationDepth;                        // Penetration depth
        std::vector<Vector3D> points;                   // Contact points
        std::vector<Vector3D> frictionVectors;          // Two orthogonal vectors that span the tangential friction plane

        void computeFrictionVectors();                  // Compute the two friction vectors that span the tangential friction plane

    public :
        Contact(Body* const body1, Body* const body2, const Vector3D& normal, double penetrationDepth, const std::vector<Vector3D>& points);    // Constructor
        virtual ~Contact();                                                                                                                     // Destructor

        Vector3D getNormal() const;                         // Return the normal vector of the contact
        std::vector<Vector3D> getPoints() const;            // Return the contact points
        virtual void evaluate();                            // Evaluate the constraint
        virtual unsigned int getNbAuxConstraints() const;   // Return the number of auxiliary constraints

        void draw() const;                              // TODO : Delete this (Used to debug collision detection)
};

// Compute the two orthogonal vectors "v1" and "v2" that span the tangential friction plane
// The two vectors have to be such that : v1 x v2 = contactNormal
inline void Contact::computeFrictionVectors() {
    // Delete the current friction vectors
    frictionVectors.clear();

    // TODO : Implement this method ...
}

// Return the normal vector of the contact
inline Vector3D Contact::getNormal() const {
    return normal;
}

// Return the contact points
inline std::vector<Vector3D> Contact::getPoints() const {
    return points;
}


} // End of the ReactPhysics3D namespace

#endif
