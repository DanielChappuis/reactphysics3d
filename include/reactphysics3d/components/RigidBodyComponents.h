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

#ifndef REACTPHYSICS3D_RIGID_BODY_COMPONENTS_H
#define REACTPHYSICS3D_RIGID_BODY_COMPONENTS_H

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
class RigidBody;
enum class BodyType;

/// Enumeration for the type of a body
/// STATIC : A static body has infinite mass, zero velocity but the position can be
///          changed manually. A static body does not collide with other static or kinematic bodies.
/// KINEMATIC : A kinematic body has infinite mass, the velocity can be changed manually and its
///             position is computed by the physics engine. A kinematic body does not collide with
///             other static or kinematic bodies.
/// DYNAMIC : A dynamic body has non-zero mass, non-zero velocity determined by forces and its
///           position is determined by the physics engine. A dynamic body can collide with other
///           dynamic, static or kinematic bodies.
enum class BodyType {STATIC, KINEMATIC, DYNAMIC};

// Class RigidBodyComponents
/**
 * This class represent the component of the ECS that contains data about a rigid body.
 * The components of the sleeping entities (bodies) are always stored at the end of the array.
 */
class RigidBodyComponents : public Components {

    private:

        // -------------------- Attributes -------------------- //

        /// Array of body entities of each component
        Entity* mBodiesEntities;

        /// Array of pointers to the corresponding rigid bodies
        RigidBody** mRigidBodies;

        /// Array of boolean values to know if the body is allowed to go to sleep
        bool* mIsAllowedToSleep;

        /// Array of boolean values to know if the body is sleeping
        bool* mIsSleeping;

        /// Array with values for elapsed time since the body velocity was below the sleep velocity
        decimal* mSleepTimes;

        /// Array with the type of bodies (static, kinematic or dynamic)
        BodyType* mBodyTypes;

        /// Array with the linear velocity of each component
        Vector3* mLinearVelocities;

        /// Array with the angular velocity of each component
        Vector3* mAngularVelocities;

        /// Array with the external force of each component
        Vector3* mExternalForces;

        /// Array with the external torque of each component
        Vector3* mExternalTorques;

        /// Array with the linear damping factor of each component
        decimal* mLinearDampings;

        /// Array with the angular damping factor of each component
        decimal* mAngularDampings;

        /// Array with the mass of each component
        decimal* mMasses;

        /// Array with the inverse mass of each component
        decimal* mInverseMasses;

        /// Array with the inertia tensor of each component
        Vector3* mLocalInertiaTensors;

        /// Array with the inverse of the local inertia tensor of each component
        Vector3* mInverseInertiaTensorsLocal;

        /// Array with the inverse of the world inertia tensor of each component
        Matrix3x3* mInverseInertiaTensorsWorld;

        /// Array with the constrained linear velocity of each component
        Vector3* mConstrainedLinearVelocities;

        /// Array with the constrained angular velocity of each component
        Vector3* mConstrainedAngularVelocities;

        /// Array with the split linear velocity of each component
        Vector3* mSplitLinearVelocities;

        /// Array with the split angular velocity of each component
        Vector3* mSplitAngularVelocities;

        /// Array with the constrained position of each component (for position error correction)
        Vector3* mConstrainedPositions;

        /// Array of constrained orientation for each component (for position error correction)
        Quaternion* mConstrainedOrientations;

        /// Array of center of mass of each component (in local-space coordinates)
        Vector3* mCentersOfMassLocal;

        /// Array of center of mass of each component (in world-space coordinates)
        Vector3* mCentersOfMassWorld;

        /// True if the gravity needs to be applied to this component
        bool* mIsGravityEnabled;

        /// Array with the boolean value to know if the body has already been added into an island
        bool* mIsAlreadyInIsland;

        /// For each body, the array of joints entities the body is part of
        Array<Entity>* mJoints;

        /// For each body, the array of the indices of contact pairs in which the body is involved
        Array<uint>* mContactPairs;

        /// For each body, the vector of lock translation vectors
        Vector3* mLinearLockAxisFactors;

        /// For each body, the vector of lock rotation vectors
        Vector3* mAngularLockAxisFactors;

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

        /// Structure for the data of a rigid body component
        struct RigidBodyComponent {

            RigidBody* body;
            BodyType bodyType;
            const Vector3& worldPosition;

            /// Constructor
            RigidBodyComponent(RigidBody* body, BodyType bodyType, const Vector3& worldPosition)
                : body(body), bodyType(bodyType), worldPosition(worldPosition) {

            }
        };

        // -------------------- Methods -------------------- //

        /// Constructor
        RigidBodyComponents(MemoryAllocator& allocator);

        /// Destructor
        virtual ~RigidBodyComponents() override = default;

        /// Add a component
        void addComponent(Entity bodyEntity, bool isSleeping, const RigidBodyComponent& component);

        /// Return a pointer to a rigid body
        RigidBody* getRigidBody(Entity bodyEntity);

        /// Return true if the body is allowed to sleep
        bool getIsAllowedToSleep(Entity bodyEntity) const;

        /// Set the value to know if the body is allowed to sleep
        void setIsAllowedToSleep(Entity bodyEntity, bool isAllowedToSleep) const;

        /// Return true if the body is sleeping
        bool getIsSleeping(Entity bodyEntity) const;

        /// Set the value to know if the body is sleeping
        void setIsSleeping(Entity bodyEntity, bool isSleeping) const;

        /// Return the sleep time
        decimal getSleepTime(Entity bodyEntity) const;

        /// Set the sleep time
        void setSleepTime(Entity bodyEntity, decimal sleepTime) const;

        /// Return the body type of a body
        BodyType getBodyType(Entity bodyEntity);

        /// Set the body type of a body
        void setBodyType(Entity bodyEntity, BodyType bodyType);

        /// Return the linear velocity of an entity
        const Vector3& getLinearVelocity(Entity bodyEntity) const;

        /// Set the linear velocity of an entity
        void setLinearVelocity(Entity bodyEntity, const Vector3& linearVelocity);

        /// Return the angular velocity of an entity
        const Vector3& getAngularVelocity(Entity bodyEntity) const;

        /// Set the angular velocity of an entity
        void setAngularVelocity(Entity bodyEntity, const Vector3& angularVelocity);

        /// Return the external force of an entity
        const Vector3& getExternalForce(Entity bodyEntity) const;

        /// Return the external torque of an entity
        const Vector3& getExternalTorque(Entity bodyEntity) const;

        /// Return the linear damping factor of an entity
        decimal getLinearDamping(Entity bodyEntity) const;

        /// Return the angular damping factor of an entity
        decimal getAngularDamping(Entity bodyEntity) const;

        /// Return the mass of an entity
        decimal getMass(Entity bodyEntity) const;

        /// Set the mass of an entity
        void setMass(Entity bodyEntity, decimal mass);

        /// Return the mass inverse of an entity
        decimal getMassInverse(Entity bodyEntity) const;

        /// Set the inverse mass of an entity
        void setMassInverse(Entity bodyEntity, decimal inverseMass);

        /// Return the local inertia tensor of an entity
        const Vector3& getLocalInertiaTensor(Entity bodyEntity);

        /// Set the local inertia tensor of an entity
        void setLocalInertiaTensor(Entity bodyEntity, const Vector3& inertiaTensorLocal);

        /// Return the inverse local inertia tensor of an entity
        const Vector3& getInertiaTensorLocalInverse(Entity bodyEntity);

        /// Return the inverse world inertia tensor of an entity
        const Matrix3x3& getInertiaTensorWorldInverse(Entity bodyEntity);

        /// Set the external force of an entity
        void setExternalForce(Entity bodyEntity, const Vector3& externalForce);

        /// Set the external force of an entity
        void setExternalTorque(Entity bodyEntity, const Vector3& externalTorque);

        /// Set the linear damping factor of an entity
        void setLinearDamping(Entity bodyEntity, decimal linearDamping);

        /// Set the angular damping factor of an entity
        void setAngularDamping(Entity bodyEntity, decimal angularDamping);

        /// Set the inverse local inertia tensor of an entity
        void setInverseInertiaTensorLocal(Entity bodyEntity, const Vector3& inertiaTensorLocalInverse);

        /// Return the constrained linear velocity of an entity
        const Vector3& getConstrainedLinearVelocity(Entity bodyEntity) const;

        /// Return the constrained angular velocity of an entity
        const Vector3& getConstrainedAngularVelocity(Entity bodyEntity) const;

        /// Return the split linear velocity of an entity
        const Vector3& getSplitLinearVelocity(Entity bodyEntity) const;

        /// Return the split angular velocity of an entity
        const Vector3& getSplitAngularVelocity(Entity bodyEntity) const;

        /// Return the constrained position of an entity
        Vector3& getConstrainedPosition(Entity bodyEntity);

        /// Return the constrained orientation of an entity
        Quaternion& getConstrainedOrientation(Entity bodyEntity);

        /// Return the local center of mass of an entity
        const Vector3& getCenterOfMassLocal(Entity bodyEntity);

        /// Return the world center of mass of an entity
        const Vector3& getCenterOfMassWorld(Entity bodyEntity);

        /// Return true if gravity is enabled for this entity
        bool getIsGravityEnabled(Entity bodyEntity) const;

        /// Return true if the entity is already in an island
        bool getIsAlreadyInIsland(Entity bodyEntity) const;

        /// Return the lock translation factor
        const Vector3& getLinearLockAxisFactor(Entity bodyEntity) const;

        /// Return the lock rotation factor
        const Vector3& getAngularLockAxisFactor(Entity bodyEntity) const;

        /// Set the constrained linear velocity of an entity
        void setConstrainedLinearVelocity(Entity bodyEntity, const Vector3& constrainedLinearVelocity);

        /// Set the constrained angular velocity of an entity
        void setConstrainedAngularVelocity(Entity bodyEntity, const Vector3& constrainedAngularVelocity);

        /// Set the split linear velocity of an entity
        void setSplitLinearVelocity(Entity bodyEntity, const Vector3& splitLinearVelocity);

        /// Set the split angular velocity of an entity
        void setSplitAngularVelocity(Entity bodyEntity, const Vector3& splitAngularVelocity);

        /// Set the constrained position of an entity
        void setConstrainedPosition(Entity bodyEntity, const Vector3& constrainedPosition);

        /// Set the constrained orientation of an entity
        void setConstrainedOrientation(Entity bodyEntity, const Quaternion& constrainedOrientation);

        /// Set the local center of mass of an entity
        void setCenterOfMassLocal(Entity bodyEntity, const Vector3& centerOfMassLocal);

        /// Set the world center of mass of an entity
        void setCenterOfMassWorld(Entity bodyEntity, const Vector3& centerOfMassWorld);

        /// Set the value to know if the gravity is enabled for this entity
        void setIsGravityEnabled(Entity bodyEntity, bool isGravityEnabled);

        /// Set the value to know if the entity is already in an island
        void setIsAlreadyInIsland(Entity bodyEntity, bool isAlreadyInIsland);

        /// Set the linear lock axis factor
        void setLinearLockAxisFactor(Entity bodyEntity, const Vector3& linearLockAxisFactor);

        /// Set the angular lock axis factor
        void setAngularLockAxisFactor(Entity bodyEntity, const Vector3& rotationTranslationFactor);

        /// Return the array of joints of a body
        const Array<Entity>& getJoints(Entity bodyEntity) const;

        /// Add a joint to a body component
        void addJointToBody(Entity bodyEntity, Entity jointEntity);

        /// Remove a joint from a body component
        void removeJointFromBody(Entity bodyEntity, Entity jointEntity);

        /// A an associated contact pairs into the contact pairs array of the body
        void addContacPair(Entity bodyEntity, uint32 contactPairIndex);

        // -------------------- Friendship -------------------- //

        friend class PhysicsWorld;
        friend class ContactSolverSystem;
        friend class CollisionDetectionSystem;
        friend class SolveBallAndSocketJointSystem;
        friend class SolveFixedJointSystem;
        friend class SolveHingeJointSystem;
        friend class SolveSliderJointSystem;
        friend class DynamicsSystem;
        friend class BallAndSocketJoint;
        friend class FixedJoint;
        friend class HingeJoint;
        friend class SliderJoint;
};

// Return a pointer to a body rigid
RP3D_FORCE_INLINE RigidBody* RigidBodyComponents::getRigidBody(Entity bodyEntity) {

    assert(mMapEntityToComponentIndex.containsKey(bodyEntity));

    return mRigidBodies[mMapEntityToComponentIndex[bodyEntity]];
}

// Return true if the body is allowed to sleep
RP3D_FORCE_INLINE bool RigidBodyComponents::getIsAllowedToSleep(Entity bodyEntity) const {

    assert(mMapEntityToComponentIndex.containsKey(bodyEntity));

    return mIsAllowedToSleep[mMapEntityToComponentIndex[bodyEntity]];
}

// Set the value to know if the body is allowed to sleep
RP3D_FORCE_INLINE void RigidBodyComponents::setIsAllowedToSleep(Entity bodyEntity, bool isAllowedToSleep) const {

    assert(mMapEntityToComponentIndex.containsKey(bodyEntity));

    mIsAllowedToSleep[mMapEntityToComponentIndex[bodyEntity]] = isAllowedToSleep;
}

// Return true if the body is sleeping
RP3D_FORCE_INLINE bool RigidBodyComponents::getIsSleeping(Entity bodyEntity) const {

    assert(mMapEntityToComponentIndex.containsKey(bodyEntity));

    return mIsSleeping[mMapEntityToComponentIndex[bodyEntity]];
}

// Set the value to know if the body is sleeping
RP3D_FORCE_INLINE void RigidBodyComponents::setIsSleeping(Entity bodyEntity, bool isSleeping) const {

    assert(mMapEntityToComponentIndex.containsKey(bodyEntity));

    mIsSleeping[mMapEntityToComponentIndex[bodyEntity]] = isSleeping;
}

// Return the sleep time
RP3D_FORCE_INLINE decimal RigidBodyComponents::getSleepTime(Entity bodyEntity) const {

    assert(mMapEntityToComponentIndex.containsKey(bodyEntity));

    return mSleepTimes[mMapEntityToComponentIndex[bodyEntity]];
}

// Set the sleep time
RP3D_FORCE_INLINE void RigidBodyComponents::setSleepTime(Entity bodyEntity, decimal sleepTime) const {

    assert(mMapEntityToComponentIndex.containsKey(bodyEntity));

    mSleepTimes[mMapEntityToComponentIndex[bodyEntity]] = sleepTime;
}

// Return the body type of a body
RP3D_FORCE_INLINE BodyType RigidBodyComponents::getBodyType(Entity bodyEntity) {

   assert(mMapEntityToComponentIndex.containsKey(bodyEntity));

   return mBodyTypes[mMapEntityToComponentIndex[bodyEntity]];
}

// Set the body type of a body
RP3D_FORCE_INLINE void RigidBodyComponents::setBodyType(Entity bodyEntity, BodyType bodyType) {

   assert(mMapEntityToComponentIndex.containsKey(bodyEntity));

   mBodyTypes[mMapEntityToComponentIndex[bodyEntity]] = bodyType;
}

// Return the linear velocity of an entity
RP3D_FORCE_INLINE const Vector3& RigidBodyComponents::getLinearVelocity(Entity bodyEntity) const {

   assert(mMapEntityToComponentIndex.containsKey(bodyEntity));

   return mLinearVelocities[mMapEntityToComponentIndex[bodyEntity]];
}

// Return the angular velocity of an entity
RP3D_FORCE_INLINE const Vector3& RigidBodyComponents::getAngularVelocity(Entity bodyEntity) const {

   assert(mMapEntityToComponentIndex.containsKey(bodyEntity));

   return mAngularVelocities[mMapEntityToComponentIndex[bodyEntity]];
}

// Set the linear velocity of an entity
RP3D_FORCE_INLINE void RigidBodyComponents::setLinearVelocity(Entity bodyEntity, const Vector3& linearVelocity) {

   assert(mMapEntityToComponentIndex.containsKey(bodyEntity));

   mLinearVelocities[mMapEntityToComponentIndex[bodyEntity]] = linearVelocity;
}

// Set the angular velocity of an entity
RP3D_FORCE_INLINE void RigidBodyComponents::setAngularVelocity(Entity bodyEntity, const Vector3& angularVelocity) {

   assert(mMapEntityToComponentIndex.containsKey(bodyEntity));

   mAngularVelocities[mMapEntityToComponentIndex[bodyEntity]] = angularVelocity;
}

// Return the external force of an entity
RP3D_FORCE_INLINE const Vector3& RigidBodyComponents::getExternalForce(Entity bodyEntity) const {

   assert(mMapEntityToComponentIndex.containsKey(bodyEntity));

   return mExternalForces[mMapEntityToComponentIndex[bodyEntity]];
}

// Return the external torque of an entity
RP3D_FORCE_INLINE const Vector3& RigidBodyComponents::getExternalTorque(Entity bodyEntity) const {

   assert(mMapEntityToComponentIndex.containsKey(bodyEntity));

   return mExternalTorques[mMapEntityToComponentIndex[bodyEntity]];
}

// Return the linear damping factor of an entity
RP3D_FORCE_INLINE decimal RigidBodyComponents::getLinearDamping(Entity bodyEntity) const {

   assert(mMapEntityToComponentIndex.containsKey(bodyEntity));

   return mLinearDampings[mMapEntityToComponentIndex[bodyEntity]];
}

// Return the angular damping factor of an entity
RP3D_FORCE_INLINE decimal RigidBodyComponents::getAngularDamping(Entity bodyEntity) const {

   assert(mMapEntityToComponentIndex.containsKey(bodyEntity));

   return mAngularDampings[mMapEntityToComponentIndex[bodyEntity]];
}

// Return the mass of an entity
RP3D_FORCE_INLINE decimal RigidBodyComponents::getMass(Entity bodyEntity) const {

   assert(mMapEntityToComponentIndex.containsKey(bodyEntity));

   return mMasses[mMapEntityToComponentIndex[bodyEntity]];
}

// Return the inverse mass of an entity
RP3D_FORCE_INLINE decimal RigidBodyComponents::getMassInverse(Entity bodyEntity) const {

   assert(mMapEntityToComponentIndex.containsKey(bodyEntity));

   return mInverseMasses[mMapEntityToComponentIndex[bodyEntity]];
}

// Return the inverse local inertia tensor of an entity
RP3D_FORCE_INLINE const Vector3& RigidBodyComponents::getInertiaTensorLocalInverse(Entity bodyEntity) {

   assert(mMapEntityToComponentIndex.containsKey(bodyEntity));

   return mInverseInertiaTensorsLocal[mMapEntityToComponentIndex[bodyEntity]];
}

// Return the inverse world inertia tensor of an entity
RP3D_FORCE_INLINE const Matrix3x3& RigidBodyComponents::getInertiaTensorWorldInverse(Entity bodyEntity) {

   assert(mMapEntityToComponentIndex.containsKey(bodyEntity));

   return mInverseInertiaTensorsWorld[mMapEntityToComponentIndex[bodyEntity]];
}

// Set the external force of an entity
RP3D_FORCE_INLINE void RigidBodyComponents::setExternalForce(Entity bodyEntity, const Vector3& externalForce) {

   assert(mMapEntityToComponentIndex.containsKey(bodyEntity));

   mExternalForces[mMapEntityToComponentIndex[bodyEntity]] = externalForce;
}

// Set the external force of an entity
RP3D_FORCE_INLINE void RigidBodyComponents::setExternalTorque(Entity bodyEntity, const Vector3& externalTorque) {

   assert(mMapEntityToComponentIndex.containsKey(bodyEntity));

   mExternalTorques[mMapEntityToComponentIndex[bodyEntity]] = externalTorque;
}

// Set the linear damping factor of an entity
RP3D_FORCE_INLINE void RigidBodyComponents::setLinearDamping(Entity bodyEntity, decimal linearDamping) {

   assert(mMapEntityToComponentIndex.containsKey(bodyEntity));

   mLinearDampings[mMapEntityToComponentIndex[bodyEntity]] = linearDamping;
}

// Set the angular damping factor of an entity
RP3D_FORCE_INLINE void RigidBodyComponents::setAngularDamping(Entity bodyEntity, decimal angularDamping) {

   assert(mMapEntityToComponentIndex.containsKey(bodyEntity));

   mAngularDampings[mMapEntityToComponentIndex[bodyEntity]] = angularDamping;
}

// Set the  mass of an entity
RP3D_FORCE_INLINE void RigidBodyComponents::setMass(Entity bodyEntity, decimal mass) {

   assert(mMapEntityToComponentIndex.containsKey(bodyEntity));

   mMasses[mMapEntityToComponentIndex[bodyEntity]] = mass;
}

// Set the mass inverse of an entity
RP3D_FORCE_INLINE void RigidBodyComponents::setMassInverse(Entity bodyEntity, decimal inverseMass) {

   assert(mMapEntityToComponentIndex.containsKey(bodyEntity));

   mInverseMasses[mMapEntityToComponentIndex[bodyEntity]] = inverseMass;
}

// Return the local inertia tensor of an entity
RP3D_FORCE_INLINE const Vector3& RigidBodyComponents::getLocalInertiaTensor(Entity bodyEntity) {

   assert(mMapEntityToComponentIndex.containsKey(bodyEntity));

   return mLocalInertiaTensors[mMapEntityToComponentIndex[bodyEntity]];
}

// Set the local inertia tensor of an entity
RP3D_FORCE_INLINE void RigidBodyComponents::setLocalInertiaTensor(Entity bodyEntity, const Vector3& inertiaTensorLocal) {

   assert(mMapEntityToComponentIndex.containsKey(bodyEntity));

   mLocalInertiaTensors[mMapEntityToComponentIndex[bodyEntity]] = inertiaTensorLocal;
}

// Set the inverse local inertia tensor of an entity
RP3D_FORCE_INLINE void RigidBodyComponents::setInverseInertiaTensorLocal(Entity bodyEntity, const Vector3& inertiaTensorLocalInverse) {

   assert(mMapEntityToComponentIndex.containsKey(bodyEntity));

   mInverseInertiaTensorsLocal[mMapEntityToComponentIndex[bodyEntity]] = inertiaTensorLocalInverse;
}

// Return the constrained linear velocity of an entity
RP3D_FORCE_INLINE const Vector3& RigidBodyComponents::getConstrainedLinearVelocity(Entity bodyEntity) const {

   assert(mMapEntityToComponentIndex.containsKey(bodyEntity));

   return mConstrainedLinearVelocities[mMapEntityToComponentIndex[bodyEntity]];
}

// Return the constrained angular velocity of an entity
RP3D_FORCE_INLINE const Vector3& RigidBodyComponents::getConstrainedAngularVelocity(Entity bodyEntity) const {

   assert(mMapEntityToComponentIndex.containsKey(bodyEntity));

   return mConstrainedAngularVelocities[mMapEntityToComponentIndex[bodyEntity]];
}

// Return the split linear velocity of an entity
RP3D_FORCE_INLINE const Vector3& RigidBodyComponents::getSplitLinearVelocity(Entity bodyEntity) const {

   assert(mMapEntityToComponentIndex.containsKey(bodyEntity));

   return mSplitLinearVelocities[mMapEntityToComponentIndex[bodyEntity]];
}

// Return the split angular velocity of an entity
RP3D_FORCE_INLINE const Vector3& RigidBodyComponents::getSplitAngularVelocity(Entity bodyEntity) const {

   assert(mMapEntityToComponentIndex.containsKey(bodyEntity));

   return mSplitAngularVelocities[mMapEntityToComponentIndex[bodyEntity]];
}

// Return the constrained position of an entity
RP3D_FORCE_INLINE Vector3& RigidBodyComponents::getConstrainedPosition(Entity bodyEntity) {

   assert(mMapEntityToComponentIndex.containsKey(bodyEntity));

   return mConstrainedPositions[mMapEntityToComponentIndex[bodyEntity]];
}

// Return the constrained orientation of an entity
RP3D_FORCE_INLINE Quaternion& RigidBodyComponents::getConstrainedOrientation(Entity bodyEntity) {

   assert(mMapEntityToComponentIndex.containsKey(bodyEntity));

   return mConstrainedOrientations[mMapEntityToComponentIndex[bodyEntity]];
}

// Return the local center of mass of an entity
RP3D_FORCE_INLINE const Vector3& RigidBodyComponents::getCenterOfMassLocal(Entity bodyEntity) {

   assert(mMapEntityToComponentIndex.containsKey(bodyEntity));

   return mCentersOfMassLocal[mMapEntityToComponentIndex[bodyEntity]];
}

// Return the world center of mass of an entity
RP3D_FORCE_INLINE const Vector3& RigidBodyComponents::getCenterOfMassWorld(Entity bodyEntity) {

   assert(mMapEntityToComponentIndex.containsKey(bodyEntity));

   return mCentersOfMassWorld[mMapEntityToComponentIndex[bodyEntity]];
}

// Set the constrained linear velocity of an entity
RP3D_FORCE_INLINE void RigidBodyComponents::setConstrainedLinearVelocity(Entity bodyEntity, const Vector3& constrainedLinearVelocity) {

   assert(mMapEntityToComponentIndex.containsKey(bodyEntity));

   mConstrainedLinearVelocities[mMapEntityToComponentIndex[bodyEntity]] = constrainedLinearVelocity;
}

// Set the constrained angular velocity of an entity
RP3D_FORCE_INLINE void RigidBodyComponents::setConstrainedAngularVelocity(Entity bodyEntity, const Vector3& constrainedAngularVelocity) {

   assert(mMapEntityToComponentIndex.containsKey(bodyEntity));

   mConstrainedAngularVelocities[mMapEntityToComponentIndex[bodyEntity]] = constrainedAngularVelocity;
}

// Set the split linear velocity of an entity
RP3D_FORCE_INLINE void RigidBodyComponents::setSplitLinearVelocity(Entity bodyEntity, const Vector3& splitLinearVelocity) {

   assert(mMapEntityToComponentIndex.containsKey(bodyEntity));

   mSplitLinearVelocities[mMapEntityToComponentIndex[bodyEntity]] = splitLinearVelocity;
}

// Set the split angular velocity of an entity
RP3D_FORCE_INLINE void RigidBodyComponents::setSplitAngularVelocity(Entity bodyEntity, const Vector3& splitAngularVelocity) {

   assert(mMapEntityToComponentIndex.containsKey(bodyEntity));

   mSplitAngularVelocities[mMapEntityToComponentIndex[bodyEntity]] = splitAngularVelocity;
}

// Set the constrained position of an entity
RP3D_FORCE_INLINE void RigidBodyComponents::setConstrainedPosition(Entity bodyEntity, const Vector3& constrainedPosition) {

   assert(mMapEntityToComponentIndex.containsKey(bodyEntity));

   mConstrainedPositions[mMapEntityToComponentIndex[bodyEntity]] = constrainedPosition;
}

// Set the constrained orientation of an entity
RP3D_FORCE_INLINE void RigidBodyComponents::setConstrainedOrientation(Entity bodyEntity, const Quaternion& constrainedOrientation) {

   assert(mMapEntityToComponentIndex.containsKey(bodyEntity));

   mConstrainedOrientations[mMapEntityToComponentIndex[bodyEntity]] = constrainedOrientation;
}

// Set the local center of mass of an entity
RP3D_FORCE_INLINE void RigidBodyComponents::setCenterOfMassLocal(Entity bodyEntity, const Vector3& centerOfMassLocal) {

   assert(mMapEntityToComponentIndex.containsKey(bodyEntity));

   mCentersOfMassLocal[mMapEntityToComponentIndex[bodyEntity]] = centerOfMassLocal;
}

// Set the world center of mass of an entity
RP3D_FORCE_INLINE void RigidBodyComponents::setCenterOfMassWorld(Entity bodyEntity, const Vector3& centerOfMassWorld) {

   assert(mMapEntityToComponentIndex.containsKey(bodyEntity));

   mCentersOfMassWorld[mMapEntityToComponentIndex[bodyEntity]] = centerOfMassWorld;
}

// Return true if gravity is enabled for this entity
RP3D_FORCE_INLINE bool RigidBodyComponents::getIsGravityEnabled(Entity bodyEntity) const {

   assert(mMapEntityToComponentIndex.containsKey(bodyEntity));

   return mIsGravityEnabled[mMapEntityToComponentIndex[bodyEntity]];
}

// Return true if the entity is already in an island
RP3D_FORCE_INLINE bool RigidBodyComponents::getIsAlreadyInIsland(Entity bodyEntity) const {

   assert(mMapEntityToComponentIndex.containsKey(bodyEntity));

   return mIsAlreadyInIsland[mMapEntityToComponentIndex[bodyEntity]];
}


// Return the linear lock axis factor
RP3D_FORCE_INLINE const Vector3& RigidBodyComponents::getLinearLockAxisFactor(Entity bodyEntity) const {

   assert(mMapEntityToComponentIndex.containsKey(bodyEntity));

   return mLinearLockAxisFactors[mMapEntityToComponentIndex[bodyEntity]];
}

// Return the angular lock axis factor
RP3D_FORCE_INLINE const Vector3& RigidBodyComponents::getAngularLockAxisFactor(Entity bodyEntity) const {

   assert(mMapEntityToComponentIndex.containsKey(bodyEntity));

   return mAngularLockAxisFactors[mMapEntityToComponentIndex[bodyEntity]];
}

// Set the value to know if the gravity is enabled for this entity
RP3D_FORCE_INLINE void RigidBodyComponents::setIsGravityEnabled(Entity bodyEntity, bool isGravityEnabled) {

   assert(mMapEntityToComponentIndex.containsKey(bodyEntity));

   mIsGravityEnabled[mMapEntityToComponentIndex[bodyEntity]] = isGravityEnabled;
}

// Set the value to know if the entity is already in an island
RP3D_FORCE_INLINE void RigidBodyComponents::setIsAlreadyInIsland(Entity bodyEntity, bool isAlreadyInIsland) {

   assert(mMapEntityToComponentIndex.containsKey(bodyEntity));
   mIsAlreadyInIsland[mMapEntityToComponentIndex[bodyEntity]] = isAlreadyInIsland;
}

// Set the linear lock axis factor
RP3D_FORCE_INLINE void RigidBodyComponents::setLinearLockAxisFactor(Entity bodyEntity, const Vector3& linearLockAxisFactor) {

   assert(mMapEntityToComponentIndex.containsKey(bodyEntity));
   mLinearLockAxisFactors[mMapEntityToComponentIndex[bodyEntity]] = linearLockAxisFactor;
}

// Set the angular lock axis factor
RP3D_FORCE_INLINE void RigidBodyComponents::setAngularLockAxisFactor(Entity bodyEntity, const Vector3& angularLockAxisFactor) {

   assert(mMapEntityToComponentIndex.containsKey(bodyEntity));
   mAngularLockAxisFactors[mMapEntityToComponentIndex[bodyEntity]] = angularLockAxisFactor;
}

// Return the array of joints of a body
RP3D_FORCE_INLINE const Array<Entity>& RigidBodyComponents::getJoints(Entity bodyEntity) const {

   assert(mMapEntityToComponentIndex.containsKey(bodyEntity));
   return mJoints[mMapEntityToComponentIndex[bodyEntity]];
}

// Add a joint to a body component
RP3D_FORCE_INLINE void RigidBodyComponents::addJointToBody(Entity bodyEntity, Entity jointEntity) {

    assert(mMapEntityToComponentIndex.containsKey(bodyEntity));
    mJoints[mMapEntityToComponentIndex[bodyEntity]].add(jointEntity);
}

// Remove a joint from a body component
RP3D_FORCE_INLINE void RigidBodyComponents::removeJointFromBody(Entity bodyEntity, Entity jointEntity) {

    assert(mMapEntityToComponentIndex.containsKey(bodyEntity));
    mJoints[mMapEntityToComponentIndex[bodyEntity]].remove(jointEntity);
}

// A an associated contact pairs into the contact pairs array of the body
RP3D_FORCE_INLINE void RigidBodyComponents::addContacPair(Entity bodyEntity, uint32 contactPairIndex) {

    assert(mMapEntityToComponentIndex.containsKey(bodyEntity));
    mContactPairs[mMapEntityToComponentIndex[bodyEntity]].add(contactPairIndex);
}

}

#endif
