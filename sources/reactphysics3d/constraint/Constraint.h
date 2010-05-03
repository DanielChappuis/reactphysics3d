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
        Body* const body1;                      // Pointer to the first body of the constraint
        Body* const body2;                      // Pointer to the second body of the constraint
        bool active;                            // True if the constraint is active
        Matrix body1Jacobian;                   // Jacobian matrix of the constraint for body1 (dimension 1x6)
        Matrix body2Jacobian;                   // Jacobian matrix of the constraint for body2 (dimension 1x6)
        Matrix auxJacobian;                     // Jacobian matrix for all the auxiliary constraints jacobian associated with this constraint
                                                // (dimension nx12 where n is the number of auxiliary constraints)
        uint nbAuxConstraints;                  // Number of auxiliary constraints associated with this constraint
        double lowerBound;                      // Lower bound of the constraint
        double upperBound;                      // Upper bound of the constraint
        Vector auxLowerBounds;                  // Vector that contains all the lower bounds of the auxiliary constraints
        Vector auxUpperBounds;                  // Vector that contains all the upper bounds of the auxiliary constraints
        double errorValue;                      // Error value (bias) of the constraint
        Vector auxErrorValues;                  // Error values for the auxiliary constraints
        
    public :
        Constraint(Body* const body1, Body* const body2, uint nbAuxConstraints, bool active);               // Constructor                                                                                                   // Constructor
        virtual ~Constraint();                                                                              // Destructor
        Body* const getBody1() const;                                                                       // Return the reference to the body 1
        Body* const getBody2() const;                                                                       // Return the reference to the body 2
        virtual void evaluate()=0;                                                                          // Evaluate the constraint
        bool isActive() const;                                                                              // Return true if the constraint is active
        const Matrix& getBody1Jacobian() const;                                                             // Return the jacobian matrix of body 1
        const Matrix& getBody2Jacobian() const;                                                             // Return the jacobian matrix of body 2
        unsigned int getNbAuxConstraints() const;                                                           // Return the number of auxiliary constraints
        const Matrix& getAuxJacobian() const;                                                               // Return the jacobian matrix of auxiliary constraints
        double getLowerBound() const;                                                                       // Return the lower bound value of the constraint
        double getUpperBound() const;                                                                       // Return the upper bound value of the constraint
        const Vector& getAuxLowerBounds() const;                                                            // Return the vector of lower bounds values
        const Vector& getAuxUpperBounds() const;                                                            // Return the vector of the upper bounds values
        double getErrorValue() const;                                                                       // Return the error value (bias) of the constraint
        const Vector& getAuxErrorValues() const;                                                            // Return the auxiliary error values
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

// Return the jacobian matrix of body 1
inline const Matrix& Constraint::getBody1Jacobian() const {
    return body1Jacobian;
}

// Return the jacobian matrix of body 2
inline const Matrix& Constraint::getBody2Jacobian() const {
    return body2Jacobian;
}

// Return the number auxiliary constraints
inline uint Constraint::getNbAuxConstraints() const {
    return nbAuxConstraints;
}

// Return the auxiliary jacobian matrix
inline const Matrix& Constraint::getAuxJacobian() const {
    return auxJacobian;
}

// Return the lower bound value of the constraint
inline double Constraint::getLowerBound() const {
   return lowerBound;
}

 // Return the upper bound value of the constraint
inline double Constraint::getUpperBound() const {
    return upperBound;
}

// Return the vector of lower bounds values
inline const Vector& Constraint::getAuxLowerBounds() const {
    return auxLowerBounds;
}

// Return the vector of the upper bounds values
inline const Vector& Constraint::getAuxUpperBounds() const {
    return auxUpperBounds;
}

// Return the error value (bias) of the constraint
inline double Constraint::getErrorValue() const {
    return errorValue;
}

// Return the auxiliary error values
inline const Vector& Constraint::getAuxErrorValues() const {
    return auxErrorValues;
}

} // End of the ReactPhysics3D namespace

#endif
