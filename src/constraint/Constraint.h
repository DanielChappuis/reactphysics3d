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
        instance. Each constraint can be made of several "mathematical
        constraints" needed to represent the main constraint.
    -------------------------------------------------------------------
*/
class Constraint {
    protected :
        Body* const body1;                      // Pointer to the first body of the constraint
        Body* const body2;                      // Pointer to the second body of the constraint
        bool active;                            // True if the constraint is active
        uint nbConstraints;                     // Number mathematical constraints associated with this Constraint
        std::vector<double> cachedLambdas;      // Cached lambda values of each mathematical constraint for more precise initializaton of LCP solver

    public :
        Constraint(Body* const body1, Body* const body2, uint nbConstraints, bool active);  // Constructor                                                                                                   // Constructor
        virtual ~Constraint();                                                              // Destructor
        Body* const getBody1() const;                                                       // Return the reference to the body 1
        Body* const getBody2() const;                                                       // Return the reference to the body 2                                                                        // Evaluate the constraint
        bool isActive() const;                                                                                      // Return true if the constraint is active                                                             // Return the jacobian matrix of body 2
        virtual void computeJacobian(int noConstraint, double J_sp[NB_MAX_CONSTRAINTS][2*6]) const=0;               // Compute the jacobian matrix for all mathematical constraints
        virtual void computeLowerBound(int noConstraint, double lowerBounds[NB_MAX_CONSTRAINTS]) const=0;           // Compute the lowerbounds values for all the mathematical constraints
        virtual void computeUpperBound(int noConstraint, double upperBounds[NB_MAX_CONSTRAINTS]) const=0;           // Compute the upperbounds values for all the mathematical constraints
        virtual void computeErrorValue(int noConstraint, double errorValues[], double penetrationFactor) const=0;   // Compute the error values for all the mathematical constraints
        unsigned int getNbConstraints() const;                                                                      // Return the number of mathematical constraints                                                                                                         // Return the number of auxiliary constraints
        double getCachedLambda(int index) const;                                                                    // Get one cached lambda value
        void setCachedLambda(int index, double lambda);                                                             // Set on cached lambda value  
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
inline uint Constraint::getNbConstraints() const {
    return nbConstraints;
}

// Get one previous lambda value
inline double Constraint::getCachedLambda(int index) const {
    assert(index >= 0 && index < nbConstraints);
    return cachedLambdas[index];
} 

// Set on cached lambda value  
inline void Constraint::setCachedLambda(int index, double lambda) {
    assert(index >= 0 && index < nbConstraints);
    cachedLambdas[index] = lambda;
}                                                   



} // End of the ReactPhysics3D namespace

#endif
