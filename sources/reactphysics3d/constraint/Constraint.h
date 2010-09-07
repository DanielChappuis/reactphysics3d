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

#ifndef CONSTRAINT_H
#define CONSTRAINT_H

// Libraries
#include "../body/Body.h"
#include "../mathematics/mathematics.h"

// ReactPhysics3D namespace
namespace reactphysics3d {

/*  -------------------------------------------------------------------
    Class Constraint :
        This abstract class represents a constraint in the physics engine.
        A constraint can be a collision contact or a joint for
        instance. Each constraint have a jacobian matrix associated with
        the first body of the constraint and a jacobian matrix associated
        with the second body. Each constraint can also have some auxiliary
        constraints. Those auxiliary constraint are all represented in the
        auxiliary Jacobian matrix. Auxiliary constraints represents some
        constraints associated with a main constraint. For instance, a
        contact constraint between two bodies can have two associated
        auxiliary constraints to represent the frictions force constraints
        in two directions.
    -------------------------------------------------------------------
*/
class Constraint {
    protected :
        Body* const body1;          // Pointer to the first body of the constraint
        Body* const body2;          // Pointer to the second body of the constraint
        bool active;                // True if the constraint is active
        uint nbAuxConstraints;      // Number of auxiliary constraints associated with this constraint
        
    public :
        Constraint(Body* const body1, Body* const body2, uint nbAuxConstraints, bool active);               // Constructor                                                                                                   // Constructor
        virtual ~Constraint();                                                                              // Destructor
        Body* const getBody1() const;                                                                       // Return the reference to the body 1
        Body* const getBody2() const;                                                                       // Return the reference to the body 2                                                                        // Evaluate the constraint
        bool isActive() const;                                                                              // Return true if the constraint is active                                                             // Return the jacobian matrix of body 2
        virtual void computeJacobian(int noBody, Matrix& jacobian) const=0;                                 // Compute a part of the jacobian for a given body
        virtual void computeAuxJacobian(int noBody, int noAuxConstraint, Matrix& auxJacobian) const=0;      // Compute a part of the jacobian for an auxiliary constraint
        virtual double computeLowerBound() const=0;                                                         // Compute the lowerbound of the constraint
        virtual double computeUpperBound() const=0;                                                         // Compute the upperbound of the constraint
        virtual void computeAuxLowerBounds(int beginIndex, Vector& auxLowerBounds) const=0;                 // Compute lowerbounds for the auxiliary constraints
        virtual void computeAuxUpperBounds(int beginIndex, Vector& auxUpperBounds) const=0;                 // Compute upperbounds for the auxiliary constraints
        virtual double computeErrorValue() const=0;                                                         // Compute the error value for the constraint
        virtual void computeAuxErrorValues(int beginIndex, Vector& errorValues) const=0;                    // Compute the errors values of the auxiliary constraints
        unsigned int getNbAuxConstraints() const;                                                           // Return the number of auxiliary constraints                                                                                                         // Return the number of auxiliary constraints
};

// Return the reference to the body 1
inline Body* const Constraint::getBody1() const {
    return body1;
}

// Return the reference to the body 2
inline Body* const Constraint::getBody2() const {
    return body2;
}

// Return true if the constraint is active
inline bool Constraint::isActive() const {
    return active;
}

// Return the number auxiliary constraints
inline uint Constraint::getNbAuxConstraints() const {
    return nbAuxConstraints;
}

} // End of the ReactPhysics3D namespace

#endif
