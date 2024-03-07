/********************************************************************************
* ReactPhysics3D physics library, http://www.reactphysics3d.com                 *
* Copyright (c) 2010-2024 Daniel Chappuis                                       *
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

#ifndef REACTPHYSICS3D_BALL_AND_SOCKET_JOINT_COMPONENTS_H
#define REACTPHYSICS3D_BALL_AND_SOCKET_JOINT_COMPONENTS_H

// Libraries
#include <reactphysics3d/mathematics/Transform.h>
#include <reactphysics3d/mathematics/Matrix3x3.h>
#include <reactphysics3d/engine/Entity.h>
#include <reactphysics3d/components/Components.h>
#include <reactphysics3d/containers/Map.h>

// ReactPhysics3D namespace
namespace reactphysics3d {

// Class declarations
class MemoryAllocator;
class EntityManager;
class BallAndSocketJoint;
enum class JointType;

// Class BallAndSocketJointComponents
/**
 * This class represent the component of the ECS with data for the BallAndSocketJoint.
 */
class BallAndSocketJointComponents : public Components {

    private:

        // -------------------- Attributes -------------------- //

        /// Array of joint entities
        Entity* mJointEntities;

        /// Array of pointers to the joints
        BallAndSocketJoint** mJoints;

        /// Anchor point of body 1 (in local-space coordinates of body 1)
        Vector3* mLocalAnchorPointBody1;

        /// Anchor point of body 2 (in local-space coordinates of body 2)
        Vector3* mLocalAnchorPointBody2;

        /// Vector from center of body 2 to anchor point in world-space
        Vector3* mR1World;

        /// Vector from center of body 2 to anchor point in world-space
        Vector3* mR2World;

        /// Inertia tensor of body 1 (in world-space coordinates)
        Matrix3x3* mI1;

        /// Inertia tensor of body 2 (in world-space coordinates)
        Matrix3x3* mI2;

        /// Bias vector for the constraint
        Vector3* mBiasVector;

        /// Inverse mass matrix K=JM^-1J^-t of the constraint
        Matrix3x3* mInverseMassMatrix;

        /// Accumulated impulse
        Vector3* mImpulse;

        /// True if the joint cone limit is enabled
        bool* mIsConeLimitEnabled;

        /// Cone limit impulse
        decimal* mConeLimitImpulse;

        /// Cone limit half angle
        decimal* mConeLimitHalfAngle;

        /// Inverse of mass matrix K=JM^-1J^t for the cone limit
        decimal* mInverseMassMatrixConeLimit;

        /// Bias of the cone limit constraint
        decimal* mBConeLimit;

        /// True if the cone limit is violated
        bool* mIsConeLimitViolated;

        /// Cross product of cone limit axis of both bodies
        Vector3* mConeLimitACrossB;

        // -------------------- Methods -------------------- //

        /// Allocate memory for a given number of components
        virtual void allocate(uint32 nbComponentsToAllocate) override;

        /// Destroy a component at a given index
        virtual void destroyComponent(uint32 index) override;

        /// Move a component from a source to a destination index in the components array
        virtual void moveComponentToIndex(uint32 srcIndex, uint32 destIndex) override;

        /// Swap two components in the array
        virtual void swapComponents(uint32 index1, uint32 index2) override;

    public:

        /// Structure for the data of a transform component
        struct BallAndSocketJointComponent {

            bool isConeLimitEnabled;
            decimal coneLimitHalfAngle;

            /// Constructor
            BallAndSocketJointComponent(bool isConeLimitEnabled, decimal coneLimitHalfAngle)
                : isConeLimitEnabled(isConeLimitEnabled), coneLimitHalfAngle(coneLimitHalfAngle) {

            }
        };

        // -------------------- Methods -------------------- //

        /// Constructor
        BallAndSocketJointComponents(MemoryAllocator& allocator);

        /// Destructor
        virtual ~BallAndSocketJointComponents() override = default;

        /// Add a component
        void addComponent(Entity jointEntity, bool isDisabled, const BallAndSocketJointComponent& component);

        /// Return a pointer to a given joint
        BallAndSocketJoint* getJoint(Entity jointEntity) const;

        /// Set the joint pointer to a given joint
        void setJoint(Entity jointEntity, BallAndSocketJoint* joint) const;

        /// Return the local anchor point of body 1 for a given joint
        const Vector3& getLocalAnchorPointBody1(Entity jointEntity) const;

        /// Set the local anchor point of body 1 for a given joint
        void setLocalAnchorPointBody1(Entity jointEntity, const Vector3& localAnchoirPointBody1);

        /// Return the local anchor point of body 2 for a given joint
        const Vector3& getLocalAnchorPointBody2(Entity jointEntity) const;

        /// Set the local anchor point of body 2 for a given joint
        void setLocalAnchorPointBody2(Entity jointEntity, const Vector3& localAnchoirPointBody2);

        /// Return the vector from center of body 1 to anchor point in world-space
        const Vector3& getR1World(Entity jointEntity) const;

        /// Set the vector from center of body 1 to anchor point in world-space
        void setR1World(Entity jointEntity, const Vector3& r1World);

        /// Return the vector from center of body 2 to anchor point in world-space
        const Vector3& getR2World(Entity jointEntity) const;

        /// Set the vector from center of body 2 to anchor point in world-space
        void setR2World(Entity jointEntity, const Vector3& r2World);

        /// Return the inertia tensor of body 1 (in world-space coordinates)
        const Matrix3x3& getI1(Entity jointEntity) const;

        /// Set the inertia tensor of body 1 (in world-space coordinates)
        void setI1(Entity jointEntity, const Matrix3x3& i1);

        /// Return the inertia tensor of body 2 (in world-space coordinates)
        const Matrix3x3& getI2(Entity jointEntity) const;

        /// Set the inertia tensor of body 2 (in world-space coordinates)
        void setI2(Entity jointEntity, const Matrix3x3& i2);

        /// Return the bias vector for the constraint
        Vector3& getBiasVector(Entity jointEntity);

        /// Set the bias vector for the constraint
        void setBiasVector(Entity jointEntity, const Vector3& biasVector);

        /// Return the inverse mass matrix K=JM^-1J^-t of the constraint
        Matrix3x3& getInverseMassMatrix(Entity jointEntity);

        /// Set the inverse mass matrix K=JM^-1J^-t of the constraint
        void setInverseMassMatrix(Entity jointEntity, const Matrix3x3& inverseMassMatrix);

        /// Return the accumulated impulse
        Vector3& getImpulse(Entity jointEntity);

        /// Set the accumulated impulse
        void setImpulse(Entity jointEntity, const Vector3& impulse);

        /// Return true if the cone limit is enabled
        bool getIsConeLimitEnabled(Entity jointEntity) const;

        /// Set to true if the cone limit is enabled
        void setIsConeLimitEnabled(Entity jointEntity, bool isLimitEnabled);

        /// Return the cone limit impulse
        bool getConeLimitImpulse(Entity jointEntity) const;

        /// Set the cone limit impulse
        void setConeLimitImpulse(Entity jointEntity, decimal impulse);

        /// Return the cone limit half angle
        decimal getConeLimitHalfAngle(Entity jointEntity) const;

        /// Set the cone limit half angle
        void setConeLimitHalfAngle(Entity jointEntity, decimal halfAngle);

        /// Return the inverse mass matrix cone limit
        bool getInverseMassMatrixConeLimit(Entity jointEntity) const;

        /// Set the inverse mass matrix cone limit
        void setInverseMassMatrixConeLimit(Entity jointEntity, decimal inverseMassMatrix);

        /// Get the cone limit local axis of body 1
        Vector3 getConeLimitLocalAxisBody1(Entity jointEntity) const;

        /// Set the cone limit local axis of body 1
        void setConeLimitLocalAxisBody1(Entity jointEntity, const Vector3& localAxisBody1);

        /// Get the cone limit local axis of body 2
        Vector3 getConeLimitLocalAxisBody2(Entity jointEntity) const;

        /// Set the cone limit local axis of body 2
        void setConeLimitLocalAxisBody2(Entity jointEntity, const Vector3& localAxisBody2);

        // -------------------- Friendship -------------------- //

        friend class BroadPhaseSystem;
        friend class SolveBallAndSocketJointSystem;
};

// Return a pointer to a given joint
RP3D_FORCE_INLINE BallAndSocketJoint* BallAndSocketJointComponents::getJoint(Entity jointEntity) const {

    assert(mMapEntityToComponentIndex.containsKey(jointEntity));
    return mJoints[mMapEntityToComponentIndex[jointEntity]];
}

// Set the joint pointer to a given joint
RP3D_FORCE_INLINE void BallAndSocketJointComponents::setJoint(Entity jointEntity, BallAndSocketJoint* joint) const {

    assert(mMapEntityToComponentIndex.containsKey(jointEntity));
    mJoints[mMapEntityToComponentIndex[jointEntity]] = joint;
}

// Return the local anchor point of body 1 for a given joint
RP3D_FORCE_INLINE const Vector3& BallAndSocketJointComponents::getLocalAnchorPointBody1(Entity jointEntity) const {

    assert(mMapEntityToComponentIndex.containsKey(jointEntity));
    return mLocalAnchorPointBody1[mMapEntityToComponentIndex[jointEntity]];
}

// Set the local anchor point of body 1 for a given joint
RP3D_FORCE_INLINE void BallAndSocketJointComponents::setLocalAnchorPointBody1(Entity jointEntity, const Vector3& localAnchoirPointBody1) {

    assert(mMapEntityToComponentIndex.containsKey(jointEntity));
    mLocalAnchorPointBody1[mMapEntityToComponentIndex[jointEntity]] = localAnchoirPointBody1;
}

// Return the local anchor point of body 2 for a given joint
RP3D_FORCE_INLINE const Vector3& BallAndSocketJointComponents::getLocalAnchorPointBody2(Entity jointEntity) const {

    assert(mMapEntityToComponentIndex.containsKey(jointEntity));
    return mLocalAnchorPointBody2[mMapEntityToComponentIndex[jointEntity]];
}

// Set the local anchor point of body 2 for a given joint
RP3D_FORCE_INLINE void BallAndSocketJointComponents::setLocalAnchorPointBody2(Entity jointEntity, const Vector3& localAnchoirPointBody2) {

    assert(mMapEntityToComponentIndex.containsKey(jointEntity));
    mLocalAnchorPointBody2[mMapEntityToComponentIndex[jointEntity]] = localAnchoirPointBody2;
}

// Return the vector from center of body 1 to anchor point in world-space
RP3D_FORCE_INLINE const Vector3& BallAndSocketJointComponents::getR1World(Entity jointEntity) const {

    assert(mMapEntityToComponentIndex.containsKey(jointEntity));
    return mR1World[mMapEntityToComponentIndex[jointEntity]];
}

// Set the vector from center of body 1 to anchor point in world-space
RP3D_FORCE_INLINE void BallAndSocketJointComponents::setR1World(Entity jointEntity, const Vector3& r1World) {

    assert(mMapEntityToComponentIndex.containsKey(jointEntity));
    mR1World[mMapEntityToComponentIndex[jointEntity]] = r1World;
}

// Return the vector from center of body 2 to anchor point in world-space
RP3D_FORCE_INLINE const Vector3& BallAndSocketJointComponents::getR2World(Entity jointEntity) const {

    assert(mMapEntityToComponentIndex.containsKey(jointEntity));
    return mR2World[mMapEntityToComponentIndex[jointEntity]];
}

// Set the vector from center of body 2 to anchor point in world-space
RP3D_FORCE_INLINE void BallAndSocketJointComponents::setR2World(Entity jointEntity, const Vector3& r2World) {

    assert(mMapEntityToComponentIndex.containsKey(jointEntity));
    mR2World[mMapEntityToComponentIndex[jointEntity]] = r2World;
}

// Return the inertia tensor of body 1 (in world-space coordinates)
RP3D_FORCE_INLINE const Matrix3x3& BallAndSocketJointComponents::getI1(Entity jointEntity) const {

    assert(mMapEntityToComponentIndex.containsKey(jointEntity));
    return mI1[mMapEntityToComponentIndex[jointEntity]];
}

// Set the inertia tensor of body 1 (in world-space coordinates)
RP3D_FORCE_INLINE void BallAndSocketJointComponents::setI1(Entity jointEntity, const Matrix3x3& i1) {

    assert(mMapEntityToComponentIndex.containsKey(jointEntity));
    mI1[mMapEntityToComponentIndex[jointEntity]] = i1;
}

// Return the inertia tensor of body 2 (in world-space coordinates)
RP3D_FORCE_INLINE const Matrix3x3& BallAndSocketJointComponents::getI2(Entity jointEntity) const {

    assert(mMapEntityToComponentIndex.containsKey(jointEntity));
    return mI2[mMapEntityToComponentIndex[jointEntity]];
}

// Set the inertia tensor of body 2 (in world-space coordinates)
RP3D_FORCE_INLINE void BallAndSocketJointComponents::setI2(Entity jointEntity, const Matrix3x3& i2) {

    assert(mMapEntityToComponentIndex.containsKey(jointEntity));
    mI2[mMapEntityToComponentIndex[jointEntity]] = i2;
}

// Return the bias vector for the constraint
RP3D_FORCE_INLINE Vector3 &BallAndSocketJointComponents::getBiasVector(Entity jointEntity) {

    assert(mMapEntityToComponentIndex.containsKey(jointEntity));
    return mBiasVector[mMapEntityToComponentIndex[jointEntity]];
}

// Set the bias vector for the constraint
RP3D_FORCE_INLINE void BallAndSocketJointComponents::setBiasVector(Entity jointEntity, const Vector3& biasVector) {

    assert(mMapEntityToComponentIndex.containsKey(jointEntity));
    mBiasVector[mMapEntityToComponentIndex[jointEntity]] = biasVector;
}

// Return the inverse mass matrix K=JM^-1J^-t of the constraint
RP3D_FORCE_INLINE Matrix3x3& BallAndSocketJointComponents::getInverseMassMatrix(Entity jointEntity) {

    assert(mMapEntityToComponentIndex.containsKey(jointEntity));
    return mInverseMassMatrix[mMapEntityToComponentIndex[jointEntity]];
}

// Set the inverse mass matrix K=JM^-1J^-t of the constraint
RP3D_FORCE_INLINE void BallAndSocketJointComponents::setInverseMassMatrix(Entity jointEntity, const Matrix3x3& inverseMassMatrix) {

    assert(mMapEntityToComponentIndex.containsKey(jointEntity));
    mInverseMassMatrix[mMapEntityToComponentIndex[jointEntity]] = inverseMassMatrix;
}

// Return the accumulated impulse
RP3D_FORCE_INLINE Vector3& BallAndSocketJointComponents::getImpulse(Entity jointEntity)  {

    assert(mMapEntityToComponentIndex.containsKey(jointEntity));
    return mImpulse[mMapEntityToComponentIndex[jointEntity]];
}

// Set the accumulated impulse
RP3D_FORCE_INLINE void BallAndSocketJointComponents::setImpulse(Entity jointEntity, const Vector3& impulse) {

    assert(mMapEntityToComponentIndex.containsKey(jointEntity));
    mImpulse[mMapEntityToComponentIndex[jointEntity]] = impulse;
}

// Return true if the cone limit is enabled
RP3D_FORCE_INLINE bool BallAndSocketJointComponents::getIsConeLimitEnabled(Entity jointEntity) const {

    assert(mMapEntityToComponentIndex.containsKey(jointEntity));
    return mIsConeLimitEnabled[mMapEntityToComponentIndex[jointEntity]];
}

// Set to true if the cone limit is enabled
RP3D_FORCE_INLINE void BallAndSocketJointComponents::setIsConeLimitEnabled(Entity jointEntity, bool isLimitEnabled) {

    assert(mMapEntityToComponentIndex.containsKey(jointEntity));
    mIsConeLimitEnabled[mMapEntityToComponentIndex[jointEntity]] = isLimitEnabled;
}

// Return the cone limit impulse
RP3D_FORCE_INLINE bool BallAndSocketJointComponents::getConeLimitImpulse(Entity jointEntity) const {

    assert(mMapEntityToComponentIndex.containsKey(jointEntity));
    return mConeLimitImpulse[mMapEntityToComponentIndex[jointEntity]];
}

// Set the cone limit impulse
RP3D_FORCE_INLINE void BallAndSocketJointComponents::setConeLimitImpulse(Entity jointEntity, decimal impulse) {

    assert(mMapEntityToComponentIndex.containsKey(jointEntity));
    mConeLimitImpulse[mMapEntityToComponentIndex[jointEntity]] = impulse;
}

// Return the cone limit half angle
RP3D_FORCE_INLINE decimal BallAndSocketJointComponents::getConeLimitHalfAngle(Entity jointEntity) const {

    assert(mMapEntityToComponentIndex.containsKey(jointEntity));
    return mConeLimitHalfAngle[mMapEntityToComponentIndex[jointEntity]];
}

// Set the cone limit half angle
RP3D_FORCE_INLINE void BallAndSocketJointComponents::setConeLimitHalfAngle(Entity jointEntity, decimal halfAngle) {

    assert(mMapEntityToComponentIndex.containsKey(jointEntity));
    mConeLimitHalfAngle[mMapEntityToComponentIndex[jointEntity]] = halfAngle;
}

// Return the inverse mass matrix cone limit
RP3D_FORCE_INLINE bool BallAndSocketJointComponents::getInverseMassMatrixConeLimit(Entity jointEntity) const {

    assert(mMapEntityToComponentIndex.containsKey(jointEntity));
    return mInverseMassMatrixConeLimit[mMapEntityToComponentIndex[jointEntity]];
}

// Set the inverse mass matrix cone limit
RP3D_FORCE_INLINE void BallAndSocketJointComponents::setInverseMassMatrixConeLimit(Entity jointEntity, decimal inverseMassMatrix) {

    assert(mMapEntityToComponentIndex.containsKey(jointEntity));
    mInverseMassMatrixConeLimit[mMapEntityToComponentIndex[jointEntity]] = inverseMassMatrix;
}

}

#endif
