/********************************************************************************
* ReactPhysics3D physics library, http://www.reactphysics3d.com                 *
* Copyright (c) 2010-2018 Daniel Chappuis                                       *
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

#ifndef REACTPHYSICS3D_BALL_AND_SOCKET_JOINT_H
#define REACTPHYSICS3D_BALL_AND_SOCKET_JOINT_H

// Libraries
#include "Joint.h"
#include "mathematics/mathematics.h"

namespace reactphysics3d {

// Structure BallAndSocketJointInfo
/**
 * This structure is used to gather the information needed to create a ball-and-socket
 * joint. This structure will be used to create the actual ball-and-socket joint.
 */
struct BallAndSocketJointInfo : public JointInfo {

    public :

        // -------------------- Attributes -------------------- //

        /// Anchor point (in world-space coordinates)
        Vector3 anchorPointWorldSpace;

        /// Constructor
        /**
         * @param rigidBody1 Pointer to the first body of the joint
         * @param rigidBody2 Pointer to the second body of the joint
         * @param initAnchorPointWorldSpace The anchor point in world-space
         *                                  coordinates
         */
        BallAndSocketJointInfo(RigidBody* rigidBody1, RigidBody* rigidBody2,
                               const Vector3& initAnchorPointWorldSpace)
                              : JointInfo(rigidBody1, rigidBody2, JointType::BALLSOCKETJOINT),
                                anchorPointWorldSpace(initAnchorPointWorldSpace) {}
};

// Class BallAndSocketJoint
/**
 * This class represents a ball-and-socket joint that allows arbitrary rotation
 * between two bodies. This joint has three degrees of freedom. It can be used to
 * create a chain of bodies for instance.
 */
class BallAndSocketJoint : public Joint {

    private :

        // -------------------- Constants -------------------- //

        // Beta value for the bias factor of position correction
        static const decimal BETA;

        // -------------------- Attributes -------------------- //

        /// Anchor point of body 1 (in local-space coordinates of body 1)
        Vector3 mLocalAnchorPointBody1;

        /// Anchor point of body 2 (in local-space coordinates of body 2)
        Vector3 mLocalAnchorPointBody2;

        /// Vector from center of body 2 to anchor point in world-space
        Vector3 mR1World;

        /// Vector from center of body 2 to anchor point in world-space
        Vector3 mR2World;

        /// Inertia tensor of body 1 (in world-space coordinates)
        Matrix3x3 mI1;

        /// Inertia tensor of body 2 (in world-space coordinates)
        Matrix3x3 mI2;

        /// Bias vector for the constraint
        Vector3 mBiasVector;

        /// Inverse mass matrix K=JM^-1J^-t of the constraint
        Matrix3x3 mInverseMassMatrix;

        /// Accumulated impulse
        Vector3 mImpulse;

        // -------------------- Methods -------------------- //

        /// Return the number of bytes used by the joint
        virtual size_t getSizeInBytes() const override;

        /// Initialize before solving the constraint
        virtual void initBeforeSolve(const ConstraintSolverData& constraintSolverData) override;

        /// Warm start the constraint (apply the previous impulse at the beginning of the step)
        virtual void warmstart(const ConstraintSolverData& constraintSolverData) override;

        /// Solve the velocity constraint
        virtual void solveVelocityConstraint(const ConstraintSolverData& constraintSolverData) override;

        /// Solve the position constraint (for position error correction)
        virtual void solvePositionConstraint(const ConstraintSolverData& constraintSolverData) override;

    public :

        // -------------------- Methods -------------------- //

        /// Constructor
        BallAndSocketJoint(uint id, const BallAndSocketJointInfo& jointInfo);

        /// Destructor
        virtual ~BallAndSocketJoint() override = default;

        /// Deleted copy-constructor
        BallAndSocketJoint(const BallAndSocketJoint& constraint) = delete;

        /// Return a string representation
        virtual std::string to_string() const override;

        /// Deleted assignment operator
        BallAndSocketJoint& operator=(const BallAndSocketJoint& constraint) = delete;
};

// Return the number of bytes used by the joint
inline size_t BallAndSocketJoint::getSizeInBytes() const {
    return sizeof(BallAndSocketJoint);
}

// Return a string representation
inline std::string BallAndSocketJoint::to_string() const {
    return "BallAndSocketJoint{ localAnchorPointBody1=" + mLocalAnchorPointBody1.to_string() +
            ", localAnchorPointBody2=" + mLocalAnchorPointBody2.to_string() + "}";
}

}

#endif
