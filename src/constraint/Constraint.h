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
#include "../body/RigidBody.h"
#include "../mathematics/mathematics.h"

// ReactPhysics3D namespace
namespace reactphysics3d {
    
// Enumeration for the type of a constraint
enum ConstraintType {CONTACT};

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

        // -------------------- Attributes -------------------- //

        // Pointer to the first body of the constraint
        RigidBody* const mBody1;

        // Pointer to the second body of the constraint
        RigidBody* const mBody2;

        // True if the constraint is active
        bool mActive;

        // Number mathematical constraints associated with this Constraint
        uint mNbConstraints;

        // Type of the constraint
        const ConstraintType mType;

        // Cached lambda values of each mathematical constraint for
        // more precise initializaton of LCP solver
        std::vector<decimal> mCachedLambdas;

        // -------------------- Methods -------------------- //

        // Private copy-constructor
        Constraint(const Constraint& constraint);

        // Private assignment operator
        Constraint& operator=(const Constraint& constraint);

    public :

        // -------------------- Methods -------------------- //

        // Constructor
        Constraint(RigidBody* const body1, RigidBody* const body2, uint nbConstraints,
                   bool active, ConstraintType type);

        // Destructor
        virtual ~Constraint();

        // Return the reference to the body 1
        RigidBody* const getBody1() const;

        // Return the reference to the body 2
        RigidBody* const getBody2() const;

        // Return true if the constraint is active
        bool isActive() const;

        // Return the type of the constraint
        ConstraintType getType() const;

        // Return the number of mathematical constraints
        unsigned int getNbConstraints() const;

        // Get one cached lambda value
        decimal getCachedLambda(int index) const;

        // Set on cached lambda value
        void setCachedLambda(int index, decimal lambda);
};

// Return the reference to the body 1
inline RigidBody* const Constraint::getBody1() const {
    return mBody1;
}

// Return the reference to the body 2
inline RigidBody* const Constraint::getBody2() const {
    return mBody2;
}

// Return true if the constraint is active
inline bool Constraint::isActive() const {
    return mActive;
}

// Return the type of the constraint
inline ConstraintType Constraint::getType() const {
    return mType;
}                                                          


// Return the number auxiliary constraints
inline uint Constraint::getNbConstraints() const {
    return mNbConstraints;
}

// Get one previous lambda value
inline decimal Constraint::getCachedLambda(int index) const {
    assert(index >= 0 && index < mNbConstraints);
    return mCachedLambdas[index];
} 

// Set on cached lambda value  
inline void Constraint::setCachedLambda(int index, decimal lambda) {
    assert(index >= 0 && index < mNbConstraints);
    mCachedLambdas[index] = lambda;
}                                                   



} // End of the ReactPhysics3D namespace

#endif
