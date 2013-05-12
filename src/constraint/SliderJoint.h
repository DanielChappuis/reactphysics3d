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

#ifndef REACTPHYSICS3D_SLIDER_JOINT_H
#define REACTPHYSICS3D_SLIDER_JOINT_H

// Libraries
#include "../mathematics/mathematics.h"
#include "../engine/ConstraintSolver.h"

namespace reactphysics3d {

// Structure SliderJointInfo
/**
 * This structure is used to gather the information needed to create a slider
 * joint. This structure will be used to create the actual slider joint.
 */
struct SliderJointInfo : public ConstraintInfo {

    public :

        // -------------------- Attributes -------------------- //

        /// Anchor point (in world-space coordinates)
        Vector3 anchorPointWorldSpace;

        /// Slider axis (in world-space coordinates)
        Vector3 axisWorldSpace;

        /// Constructor
        SliderJointInfo(RigidBody* rigidBody1, RigidBody* rigidBody2,
                        const Vector3& initAnchorPointWorldSpace,
                        const Vector3& initAxisWorldSpace)
                       : ConstraintInfo(rigidBody1, rigidBody2, SLIDERJOINT),
                         anchorPointWorldSpace(initAnchorPointWorldSpace),
                         axisWorldSpace(initAxisWorldSpace) {}
};

// Class SliderJoint
/**
 * This class represents a slider joint.
 */
class SliderJoint : public Constraint {

    private :

        // -------------------- Attributes -------------------- //

        /// Anchor point of body 1 (in local space coordinates)
        Vector3 mLocalAnchorPointBody1;

        /// Anchor point of body 2 (in local space coordinates)
        Vector3 mLocalAnchorPointBody2;

        /// Vector from center of body 2 to anchor point in world-space
        Vector3 mU1World;

        /// Vector from center of body 2 to anchor point in world-space
        Vector3 mU2World;

        /// First vector orthogonal to vector mU1World in world-space
        Vector3 mN1;

        /// Second vector orthogonal to vector mU1World and mN1 in world-space
        Vector3 mN2;

        /// Cross product of mU1World and mN1
        Vector3 mU1WorldCrossN1;

        /// Cross product of mU1World and mN2
        Vector3 mU1WorldCrossN2;

        /// Cross product of mU2World and mN1
        Vector3 mU2WorldCrossN1;

        /// Cross product of mU2World and mN2
        Vector3 mU2WorldCrossN2;

        /// Inverse of mass matrix K=JM^-1J^t for the translation constraint (2x2 matrix)
        Matrix2x2 mInverseMassMatrixTranslationConstraint;

        /// Inverse of mass matrix K=JM^-1J^t for the rotation constraint (3x3 matrix)
        Matrix3x3 mInverseMassMatrixRotationConstraint;

        /// Impulse for the 2 translation constraints
        Vector2 mImpulseTranslation;

        /// Impulse for the 3 rotation constraints
        Vector3 mImpulseRotation;

    public :

        // -------------------- Methods -------------------- //

        /// Constructor
        SliderJoint(const SliderJointInfo& jointInfo);

        /// Destructor
        virtual ~SliderJoint();

        /// Return the number of bytes used by the joint
        virtual size_t getSizeInBytes() const;

        /// Initialize before solving the constraint
        virtual void initBeforeSolve(const ConstraintSolverData& constraintSolverData);

        /// Warm start the constraint (apply the previous impulse at the beginning of the step)
        virtual void warmstart(const ConstraintSolverData& constraintSolverData);

        /// Solve the velocity constraint
        virtual void solveVelocityConstraint(const ConstraintSolverData& constraintSolverData);

        /// Solve the position constraint
        virtual void solvePositionConstraint(const ConstraintSolverData& constraintSolverData);
};

// Return the number of bytes used by the joint
inline size_t SliderJoint::getSizeInBytes() const {
    return sizeof(SliderJoint);
}

}

#endif
