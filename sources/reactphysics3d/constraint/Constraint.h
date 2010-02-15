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

// Constants
const double kErp = 0.8;        // Value between 0 and 1 used to compute the kCorrection value (kCorrection = kErp * kFps)

/*  -------------------------------------------------------------------
    Class Constraint :
        This abstract class represents a constraint in the physics engine.
        A constraint can be a collision contact or a joint for
        instance.
    -------------------------------------------------------------------
*/
class Constraint {
    protected :
        Body* const body1;                  // Pointer to the first body of the constraint
        Body* const body2;                  // Pointer to the second body of the constraint
        bool active;                        // True if the constraint is active
        Matrix body1LinearJacobian;         // Linear jacobian matrix of the constraint for body1
        Matrix body2LinearJacobian;         // Linear jacobian matrix of the constraint for body2
        Matrix body1AngularJacobian;        // Angular jacobian matrix of the constraint for body1
        Matrix body2AngularJacobian;        // Angular jacobian matrix of the constraint for body2
        Vector errorVector;                 // Error term vector of the constraint
        Vector rightHandSideVector;         // Right-hand-side vector
        Matrix auxiliaryRows;
        Matrix auxiliaryColumns;
        unsigned int jacobianIndex;         // Jacobian index
        unsigned int auxiliaryIndex;        // Auxiliary index

    public :
        Constraint(Body* const body1, Body* const body2);                   // Constructor
        virtual ~Constraint();                                              // Destructor

        Body* const getBody1() const;                                       // Return the reference to the body 1
        Body* const getBody2() const;                                       // Return the reference to the body 2
        virtual void evaluate(double dt)=0;                                 // Evaluate the constraint
        bool isActive() const;                                              // Return true if the constraint is active
        virtual unsigned int getNbJacobianRows() const=0;                   // Return the number of rows of the Jacobian matrix
        void setJacobianIndex(unsigned int index);                          // Set the jacobian index value
        unsigned int getJacobianIndex() const;                              // Return the jacobian index
        Matrix getBody1LinearJacobian() const;                              // Return the linear jacobian matrix of body 1
        Matrix getBody2LinearJacobian() const;                              // Return the linear jacobian matrix of body 2
        Matrix getBody1AngularJacobian() const;                             // Return the angular jacobian matrix of body 1
        Matrix getBody2AngularJacobian() const;                             // Return the angular jacobian matrix of body 2
        Vector getErrorVector() const;                                      // Return the error vector of the constraint
        virtual int getNbAuxiliaryVars() const=0;                           // Return the number of auxiliary variables
        void setAuxiliaryIndex(unsigned int index);                         // Set the auxiliary index
        unsigned int getAuxiliaryIndex() const;                             // Return the auxiliary index
        void getAuxiliaryRowsAndCols(Matrix& rows, Matrix& columns) const;  // Return the auxiliary rows and columns
        Vector getRightHandSideVector() const;                              // Return the right-hand-side vector
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

// Set the jacobian index value
inline void Constraint::setJacobianIndex(unsigned int index) {
    this->jacobianIndex = index;
}

// Return the jacobian index
inline unsigned int Constraint::getJacobianIndex() const {
    return jacobianIndex;
}

// Return the linear jacobian matrix of body 1
inline Matrix Constraint::getBody1LinearJacobian() const {
    return body1LinearJacobian;
}

// Return the linear jacobian matrix of body 2
inline Matrix Constraint::getBody2LinearJacobian() const {
    return body2LinearJacobian;
}

// Return the angular jacobian matrix of body 1
inline Matrix Constraint::getBody1AngularJacobian() const {
    return body1AngularJacobian;
}

// Return the angular jacobian matrix of body 2
inline Matrix Constraint::getBody2AngularJacobian() const {
    return body2AngularJacobian;
}

// Return the error vector of the constraint
inline Vector Constraint::getErrorVector() const {
    return errorVector;
}

// Set the auxiliary index
inline void Constraint::setAuxiliaryIndex(unsigned int index) {
    auxiliaryIndex = index;
}

// Return the auxiliary index
inline unsigned int Constraint::getAuxiliaryIndex() const {
    return auxiliaryIndex;
}

// Return the auxiliary rows and columns
inline void Constraint::getAuxiliaryRowsAndCols(Matrix& rows, Matrix& columns) const {
    rows = auxiliaryRows;
    columns = auxiliaryColumns;
}

// Return the right-hand-side vector
inline Vector Constraint::getRightHandSideVector() const {
    return rightHandSideVector;
}


} // End of the ReactPhysics3D namespace

#endif
