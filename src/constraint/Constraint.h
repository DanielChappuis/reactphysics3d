/********************************************************************************
* ReactPhysics3D physics library, http://code.google.com/p/reactphysics3d/      *
* Copyright (c) 2010-2013 Daniel Chappuis                                       *
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

#ifndef REACTPHYSICS3D_CONSTRAINT_H
#define REACTPHYSICS3D_CONSTRAINT_H

// Libraries
#include "../configuration.h"
#include "../body/RigidBody.h"
#include "../mathematics/mathematics.h"

// ReactPhysics3D namespace
namespace reactphysics3d {

// Enumeration for the type of a constraint
enum ConstraintType {CONTACT, BALLSOCKETJOINT, SLIDERJOINT, HINGEJOINT, FIXEDJOINT};

// Class declarations
struct ConstraintSolverData;

// Structure ConstraintInfo
/**
 * This structure is used to gather the information needed to create a constraint.
 */
struct ConstraintInfo {

    public :

        // -------------------- Attributes -------------------- //

        /// First rigid body of the constraint
        RigidBody* body1;

        /// Second rigid body of the constraint
        RigidBody* body2;

        /// Type of the constraint
        ConstraintType type;

        /// True if the two bodies of the constraint are allowed to collide with each other
        bool isCollisionEnabled;

        /// Position correction technique used for the constraint (used for joints).
        /// By default, the BAUMGARTE technique is used
        JointsPositionCorrectionTechnique positionCorrectionTechnique;

        /// Constructor
        ConstraintInfo(ConstraintType constraintType)
                      : body1(NULL), body2(NULL), type(constraintType),
                        positionCorrectionTechnique(NON_LINEAR_GAUSS_SEIDEL),
                        isCollisionEnabled(true) {}

        /// Constructor
        ConstraintInfo(RigidBody* rigidBody1, RigidBody* rigidBody2, ConstraintType constraintType)
                      : body1(rigidBody1), body2(rigidBody2), type(constraintType),
                        positionCorrectionTechnique(NON_LINEAR_GAUSS_SEIDEL),
                        isCollisionEnabled(true) {
        }

        /// Destructor
        virtual ~ConstraintInfo() {}

};

// Class Constraint
/**
 * This abstract class represents a constraint in the physics engine.
 * A constraint can be a collision contact or a joint for
 * instance. Each constraint can be made of several "mathematical
 * constraints" needed to represent the main constraint.
 */
class Constraint {

    protected :

        // -------------------- Attributes -------------------- //

        /// Pointer to the first body of the constraint
        RigidBody* const mBody1;

        /// Pointer to the second body of the constraint
        RigidBody* const mBody2;

        /// True if the constraint is active
        bool mActive;

        /// Type of the constraint
        const ConstraintType mType;

        /// Body 1 index in the velocity array to solve the constraint
        uint mIndexBody1;

        /// Body 2 index in the velocity array to solve the constraint
        uint mIndexBody2;

        /// Position correction technique used for the constraint (used for joints)
        JointsPositionCorrectionTechnique mPositionCorrectionTechnique;

        /// True if the two bodies of the constraint are allowed to collide with each other
        bool mIsCollisionEnabled;

        // -------------------- Methods -------------------- //

        /// Private copy-constructor
        Constraint(const Constraint& constraint);

        /// Private assignment operator
        Constraint& operator=(const Constraint& constraint);

    public :

        // -------------------- Methods -------------------- //

        /// Constructor
        Constraint(const ConstraintInfo& constraintInfo);

        /// Destructor
        virtual ~Constraint();

        /// Return the reference to the body 1
        RigidBody* const getBody1() const;

        /// Return the reference to the body 2
        RigidBody* const getBody2() const;

        /// Return true if the constraint is active
        bool isActive() const;

        /// Return the type of the constraint
        ConstraintType getType() const;

        /// Return true if the collision between the two bodies of the constraint is enabled
        bool isCollisionEnabled() const;

        /// Return the number of bytes used by the constraint
        virtual size_t getSizeInBytes() const = 0;

        /// Initialize before solving the constraint
        virtual void initBeforeSolve(const ConstraintSolverData& constraintSolverData) = 0;

        /// Warm start the constraint (apply the previous impulse at the beginning of the step)
        virtual void warmstart(const ConstraintSolverData& constraintSolverData) = 0;

        /// Solve the velocity constraint
        virtual void solveVelocityConstraint(const ConstraintSolverData& constraintSolverData) = 0;

        /// Solve the position constraint
        virtual void solvePositionConstraint(const ConstraintSolverData& constraintSolverData) = 0;
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

// Return true if the collision between the two bodies of the constraint is enabled
inline bool Constraint::isCollisionEnabled() const {
    return mIsCollisionEnabled;
}

}

#endif
