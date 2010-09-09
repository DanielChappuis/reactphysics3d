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
