/********************************************************************************
* ReactPhysics3D physics library, http://www.reactphysics3d.com                 *
* Copyright (c) 2010-2022 Daniel Chappuis                                       *
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
#include <reactphysics3d/constraint/Joint.h>
#include <reactphysics3d/mathematics/mathematics.h>

namespace reactphysics3d {

// Structure BallAndSocketJointInfo
/**
 * This structure is used to gather the information needed to create a ball-and-socket
 * joint. This structure will be used to create the actual ball-and-socket joint.
 */
struct BallAndSocketJointInfo : public JointInfo {

    public :

        // -------------------- Attributes -------------------- //

        /// True if this object has been constructed using local-space anchors
        bool isUsingLocalSpaceAnchors;

        /// Anchor point (in world-space coordinates)
        Vector3 anchorPointWorldSpace;

        /// Anchor point on body 1 (in local-space coordinates)
        Vector3 anchorPointBody1LocalSpace;

        /// Anchor point on body 2 (in local-space coordinates)
        Vector3 anchorPointBody2LocalSpace;

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
                                isUsingLocalSpaceAnchors(false),
                                anchorPointWorldSpace(initAnchorPointWorldSpace) {}

        /// Constructor
        /**
         * @param rigidBody1 Pointer to the first body of the joint
         * @param rigidBody2 Pointer to the second body of the joint
         * @param anchorPointBody1LocalSpace The anchor point on body 1 in local-space coordinates
         * @param anchorPointBody2LocalSpace The anchor point on body 2 in local-space coordinates
         */
        BallAndSocketJointInfo(RigidBody* rigidBody1, RigidBody* rigidBody2,
                               const Vector3& anchorPointBody1LocalSpace,
                               const Vector3& anchorPointBody2LocalSpace)
                              : JointInfo(rigidBody1, rigidBody2, JointType::BALLSOCKETJOINT),
                                isUsingLocalSpaceAnchors(true),
                                anchorPointBody1LocalSpace(anchorPointBody1LocalSpace),
                                anchorPointBody2LocalSpace(anchorPointBody2LocalSpace) {}
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


        // -------------------- Methods -------------------- //

        /// Return the number of bytes used by the joint
        virtual size_t getSizeInBytes() const override;

        /// Reset the limits
        void resetLimits();

    public :

        // -------------------- Methods -------------------- //

        /// Constructor
        BallAndSocketJoint(Entity entity, PhysicsWorld& world, const BallAndSocketJointInfo& jointInfo);

        /// Destructor
        virtual ~BallAndSocketJoint() override = default;

        /// Deleted copy-constructor
        BallAndSocketJoint(const BallAndSocketJoint& constraint) = delete;

        /// Enable/disable the cone limit of the joint
        void enableConeLimit(bool isLimitEnabled);

        /// Return true if the cone limit or the joint is enabled
        bool isConeLimitEnabled() const;

        /// Set the cone limit half angle
        void setConeLimitHalfAngle(decimal coneHalfAngle);

        /// Return the cone limit half angle (in radians)
        decimal getConeLimitHalfAngle() const;
        
        /// Return the current cone half angle (in radians)
        decimal getConeHalfAngle() const;

        /// Return the force (in Newtons) on body 2 required to satisfy the joint constraint in world-space
        virtual Vector3 getReactionForce(decimal timeStep) const override;

        /// Return the torque (in Newtons * meters) on body 2 required to satisfy the joint constraint in world-space
        virtual Vector3 getReactionTorque(decimal timeStep) const override;

        /// Return a string representation
        virtual std::string to_string() const override;

        /// Deleted assignment operator
        BallAndSocketJoint& operator=(const BallAndSocketJoint& constraint) = delete;
};

// Return the number of bytes used by the joint
RP3D_FORCE_INLINE size_t BallAndSocketJoint::getSizeInBytes() const {
    return sizeof(BallAndSocketJoint);
}

}

#endif
