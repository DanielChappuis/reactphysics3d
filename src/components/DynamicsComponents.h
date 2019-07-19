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

#ifndef REACTPHYSICS3D_DYNAMICS_COMPONENTS_H
#define REACTPHYSICS3D_DYNAMICS_COMPONENTS_H

// Libraries
#include "mathematics/Transform.h"
#include "engine/Entity.h"
#include "components/Components.h"
#include "mathematics/Matrix3x3.h"
#include "containers/Map.h"

// ReactPhysics3D namespace
namespace reactphysics3d {

// Class declarations
class MemoryAllocator;
class EntityManager;

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

// Class DynamicsComponents
/**
 * This class represent the component of the ECS that contains the variables concerning dynamics
 * like velocities. A rigid body that is not static always has a dynamics component. A rigid body
 * that is static does not have one because it is not simulated by dynamics.
 */
class DynamicsComponents : public Components {

    private:

        // -------------------- Attributes -------------------- //

        /// Array of body entities of each component
        Entity* mBodies;

        /// Array with the linear velocity of each component
        Vector3* mLinearVelocities;

        /// Array with the angular velocity of each component
        Vector3* mAngularVelocities;

        /// Array with the constrained linear velocity of each component
        Vector3* mConstrainedLinearVelocities;

        /// Array with the constrained angular velocity of each component
        Vector3* mConstrainedAngularVelocities;

        /// Array with the split linear velocity of each component
        Vector3* mSplitLinearVelocities;

        /// Array with the split angular velocity of each component
        Vector3* mSplitAngularVelocities;

        /// Array with the external force of each component
        Vector3* mExternalForces;

        /// Array with the external torque of each component
        Vector3* mExternalTorques;

        /// Array with the linear damping factor of each component
        decimal* mLinearDampings;

        /// Array with the angular damping factor of each component
        decimal* mAngularDampings;

        /// Array with the initial mass of each component
        decimal* mInitMasses;

        /// Array with the inverse mass of each component
        decimal* mInverseMasses;

        /// Array with the inverse of the inertia tensor of each component
        Matrix3x3* mInverseInertiaTensorsLocal;

        /// Array with the inverse of the world inertia tensor of each component
        Matrix3x3* mInverseInertiaTensorsWorld;

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
        struct DynamicsComponent {

            const Vector3& worldPosition;

            /// Constructor
            DynamicsComponent(const Vector3& worldPosition)
                : worldPosition(worldPosition) {

            }
        };

        // -------------------- Methods -------------------- //

        /// Constructor
        DynamicsComponents(MemoryAllocator& allocator);

        /// Destructor
        virtual ~DynamicsComponents() override = default;

        /// Add a component
        void addComponent(Entity bodyEntity, bool isSleeping, const DynamicsComponent& component);

        /// Return the linear velocity of an entity
        const Vector3& getLinearVelocity(Entity bodyEntity) const;

        /// Return the angular velocity of an entity
        const Vector3& getAngularVelocity(Entity bodyEntity) const;

        /// Return the constrained linear velocity of an entity
        const Vector3& getConstrainedLinearVelocity(Entity bodyEntity) const;

        /// Return the constrained angular velocity of an entity
        const Vector3& getConstrainedAngularVelocity(Entity bodyEntity) const;

        /// Return the split linear velocity of an entity
        const Vector3& getSplitLinearVelocity(Entity bodyEntity) const;

        /// Return the split angular velocity of an entity
        const Vector3& getSplitAngularVelocity(Entity bodyEntity) const;

        /// Return the external force of an entity
        const Vector3& getExternalForce(Entity bodyEntity) const;

        /// Return the external torque of an entity
        const Vector3& getExternalTorque(Entity bodyEntity) const;

        /// Return the linear damping factor of an entity
        decimal getLinearDamping(Entity bodyEntity) const;

        /// Return the angular damping factor of an entity
        decimal getAngularDamping(Entity bodyEntity) const;

        /// Return the initial mass of an entity
        decimal getInitMass(Entity bodyEntity) const;

        /// Return the mass inverse of an entity
        decimal getMassInverse(Entity bodyEntity) const;

        /// Return the inverse local inertia tensor of an entity
        const Matrix3x3& getInertiaTensorLocalInverse(Entity bodyEntity);

        /// Return the inverse world inertia tensor of an entity
        const Matrix3x3& getInertiaTensorWorldInverse(Entity bodyEntity);

        /// Return the constrained position of an entity
        const Vector3& getConstrainedPosition(Entity bodyEntity);

        /// Return the constrained orientation of an entity
        const Quaternion& getConstrainedOrientation(Entity bodyEntity);

        /// Return the local center of mass of an entity
        const Vector3& getCenterOfMassLocal(Entity bodyEntity);

        /// Return the world center of mass of an entity
        const Vector3& getCenterOfMassWorld(Entity bodyEntity);

        /// Return true if gravity is enabled for this entity
        bool getIsGravityEnabled(Entity bodyEntity) const;

        /// Return true if the entity is already in an island
        bool getIsAlreadyInIsland(Entity bodyEntity) const;

        /// Set the linear velocity of an entity
        void setLinearVelocity(Entity bodyEntity, const Vector3& linearVelocity);

        /// Set the angular velocity of an entity
        void setAngularVelocity(Entity bodyEntity, const Vector3& angularVelocity);

        /// Set the constrained linear velocity of an entity
        void setConstrainedLinearVelocity(Entity bodyEntity, const Vector3& constrainedLinearVelocity);

        /// Set the constrained angular velocity of an entity
        void setConstrainedAngularVelocity(Entity bodyEntity, const Vector3& constrainedAngularVelocity);

        /// Set the split linear velocity of an entity
        void setSplitLinearVelocity(Entity bodyEntity, const Vector3& splitLinearVelocity);

        /// Set the split angular velocity of an entity
        void setSplitAngularVelocity(Entity bodyEntity, const Vector3& splitAngularVelocity);

        /// Set the external force of an entity
        void setExternalForce(Entity bodyEntity, const Vector3& externalForce);

        /// Set the external force of an entity
        void setExternalTorque(Entity bodyEntity, const Vector3& externalTorque);

        /// Set the linear damping factor of an entity
        void setLinearDamping(Entity bodyEntity, decimal linearDamping);

        /// Set the angular damping factor of an entity
        void setAngularDamping(Entity bodyEntity, decimal angularDamping);

        /// Set the initial mass of an entity
        void setInitMass(Entity bodyEntity, decimal initMass);

        /// Set the inverse mass of an entity
        void setMassInverse(Entity bodyEntity, decimal inverseMass);

        /// Set the inverse local inertia tensor of an entity
        void setInverseInertiaTensorLocal(Entity bodyEntity, const Matrix3x3& inertiaTensorLocalInverse);

        /// Set the inverse world inertia tensor of an entity
        void setInverseInertiaTensorWorld(Entity bodyEntity, const Matrix3x3& inertiaTensorWorldInverse);

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

        // -------------------- Friendship -------------------- //

        friend class BroadPhaseSystem;
        friend class DynamicsWorld;
        friend class ContactSolver;
        friend class BallAndSocketJoint;
        friend class FixedJoint;
        friend class HingeJoint;
        friend class SliderJoint;
};

// Return the linear velocity of an entity
inline const Vector3& DynamicsComponents::getLinearVelocity(Entity bodyEntity) const {

   assert(mMapEntityToComponentIndex.containsKey(bodyEntity));

   return mLinearVelocities[mMapEntityToComponentIndex[bodyEntity]];
}

// Return the angular velocity of an entity
inline const Vector3 &DynamicsComponents::getAngularVelocity(Entity bodyEntity) const {

   assert(mMapEntityToComponentIndex.containsKey(bodyEntity));

   return mAngularVelocities[mMapEntityToComponentIndex[bodyEntity]];
}

// Set the linear velocity of an entity
inline void DynamicsComponents::setLinearVelocity(Entity bodyEntity, const Vector3& linearVelocity) {

   assert(mMapEntityToComponentIndex.containsKey(bodyEntity));

   mLinearVelocities[mMapEntityToComponentIndex[bodyEntity]] = linearVelocity;
}

// Set the angular velocity of an entity
inline void DynamicsComponents::setAngularVelocity(Entity bodyEntity, const Vector3& angularVelocity) {

   assert(mMapEntityToComponentIndex.containsKey(bodyEntity));

   mAngularVelocities[mMapEntityToComponentIndex[bodyEntity]] = angularVelocity;
}

// Return the constrained linear velocity of an entity
inline const Vector3& DynamicsComponents::getConstrainedLinearVelocity(Entity bodyEntity) const {

   assert(mMapEntityToComponentIndex.containsKey(bodyEntity));

   return mConstrainedLinearVelocities[mMapEntityToComponentIndex[bodyEntity]];
}

// Return the constrained angular velocity of an entity
inline const Vector3& DynamicsComponents::getConstrainedAngularVelocity(Entity bodyEntity) const {

   assert(mMapEntityToComponentIndex.containsKey(bodyEntity));

   return mConstrainedAngularVelocities[mMapEntityToComponentIndex[bodyEntity]];
}

// Return the split linear velocity of an entity
inline const Vector3& DynamicsComponents::getSplitLinearVelocity(Entity bodyEntity) const {

   assert(mMapEntityToComponentIndex.containsKey(bodyEntity));

   return mSplitLinearVelocities[mMapEntityToComponentIndex[bodyEntity]];
}

// Return the split angular velocity of an entity
inline const Vector3& DynamicsComponents::getSplitAngularVelocity(Entity bodyEntity) const {

   assert(mMapEntityToComponentIndex.containsKey(bodyEntity));

   return mSplitAngularVelocities[mMapEntityToComponentIndex[bodyEntity]];
}

// Return the external force of an entity
inline const Vector3& DynamicsComponents::getExternalForce(Entity bodyEntity) const {

   assert(mMapEntityToComponentIndex.containsKey(bodyEntity));

   return mExternalForces[mMapEntityToComponentIndex[bodyEntity]];
}

// Return the external torque of an entity
inline const Vector3& DynamicsComponents::getExternalTorque(Entity bodyEntity) const {

   assert(mMapEntityToComponentIndex.containsKey(bodyEntity));

   return mExternalTorques[mMapEntityToComponentIndex[bodyEntity]];
}

// Return the linear damping factor of an entity
inline decimal DynamicsComponents::getLinearDamping(Entity bodyEntity) const {

   assert(mMapEntityToComponentIndex.containsKey(bodyEntity));

   return mLinearDampings[mMapEntityToComponentIndex[bodyEntity]];
}

// Return the angular damping factor of an entity
inline decimal DynamicsComponents::getAngularDamping(Entity bodyEntity) const {

   assert(mMapEntityToComponentIndex.containsKey(bodyEntity));

   return mAngularDampings[mMapEntityToComponentIndex[bodyEntity]];
}

// Return the initial mass of an entity
inline decimal DynamicsComponents::getInitMass(Entity bodyEntity) const {

   assert(mMapEntityToComponentIndex.containsKey(bodyEntity));

   return mInitMasses[mMapEntityToComponentIndex[bodyEntity]];
}

// Return the inverse mass of an entity
inline decimal DynamicsComponents::getMassInverse(Entity bodyEntity) const {

   assert(mMapEntityToComponentIndex.containsKey(bodyEntity));

   return mInverseMasses[mMapEntityToComponentIndex[bodyEntity]];
}

// Return the inverse local inertia tensor of an entity
inline const Matrix3x3& DynamicsComponents::getInertiaTensorLocalInverse(Entity bodyEntity) {

   assert(mMapEntityToComponentIndex.containsKey(bodyEntity));

   return mInverseInertiaTensorsLocal[mMapEntityToComponentIndex[bodyEntity]];
}

// Return the inverse world inertia tensor of an entity
inline const Matrix3x3& DynamicsComponents::getInertiaTensorWorldInverse(Entity bodyEntity) {

   assert(mMapEntityToComponentIndex.containsKey(bodyEntity));

   return mInverseInertiaTensorsWorld[mMapEntityToComponentIndex[bodyEntity]];
}

// Return the constrained position of an entity
inline const Vector3& DynamicsComponents::getConstrainedPosition(Entity bodyEntity) {

   assert(mMapEntityToComponentIndex.containsKey(bodyEntity));

   return mConstrainedPositions[mMapEntityToComponentIndex[bodyEntity]];
}

// Return the constrained orientation of an entity
inline const Quaternion& DynamicsComponents::getConstrainedOrientation(Entity bodyEntity) {

   assert(mMapEntityToComponentIndex.containsKey(bodyEntity));

   return mConstrainedOrientations[mMapEntityToComponentIndex[bodyEntity]];
}

// Return the local center of mass of an entity
inline const Vector3& DynamicsComponents::getCenterOfMassLocal(Entity bodyEntity) {

   assert(mMapEntityToComponentIndex.containsKey(bodyEntity));

   return mCentersOfMassLocal[mMapEntityToComponentIndex[bodyEntity]];
}

// Return the world center of mass of an entity
inline const Vector3& DynamicsComponents::getCenterOfMassWorld(Entity bodyEntity) {

   assert(mMapEntityToComponentIndex.containsKey(bodyEntity));

   return mCentersOfMassWorld[mMapEntityToComponentIndex[bodyEntity]];
}

// Set the constrained linear velocity of an entity
inline void DynamicsComponents::setConstrainedLinearVelocity(Entity bodyEntity, const Vector3& constrainedLinearVelocity) {

   assert(mMapEntityToComponentIndex.containsKey(bodyEntity));

   mConstrainedLinearVelocities[mMapEntityToComponentIndex[bodyEntity]] = constrainedLinearVelocity;
}

// Set the constrained angular velocity of an entity
inline void DynamicsComponents::setConstrainedAngularVelocity(Entity bodyEntity, const Vector3& constrainedAngularVelocity) {

   assert(mMapEntityToComponentIndex.containsKey(bodyEntity));

   mConstrainedAngularVelocities[mMapEntityToComponentIndex[bodyEntity]] = constrainedAngularVelocity;
}

// Set the split linear velocity of an entity
inline void DynamicsComponents::setSplitLinearVelocity(Entity bodyEntity, const Vector3& splitLinearVelocity) {

   assert(mMapEntityToComponentIndex.containsKey(bodyEntity));

   mSplitLinearVelocities[mMapEntityToComponentIndex[bodyEntity]] = splitLinearVelocity;
}

// Set the split angular velocity of an entity
inline void DynamicsComponents::setSplitAngularVelocity(Entity bodyEntity, const Vector3& splitAngularVelocity) {

   assert(mMapEntityToComponentIndex.containsKey(bodyEntity));

   mSplitAngularVelocities[mMapEntityToComponentIndex[bodyEntity]] = splitAngularVelocity;
}

// Set the external force of an entity
inline void DynamicsComponents::setExternalForce(Entity bodyEntity, const Vector3& externalForce) {

   assert(mMapEntityToComponentIndex.containsKey(bodyEntity));

   mExternalForces[mMapEntityToComponentIndex[bodyEntity]] = externalForce;
}

// Set the external force of an entity
inline void DynamicsComponents::setExternalTorque(Entity bodyEntity, const Vector3& externalTorque) {

   assert(mMapEntityToComponentIndex.containsKey(bodyEntity));

   mExternalTorques[mMapEntityToComponentIndex[bodyEntity]] = externalTorque;
}

// Set the linear damping factor of an entity
inline void DynamicsComponents::setLinearDamping(Entity bodyEntity, decimal linearDamping) {

   assert(mMapEntityToComponentIndex.containsKey(bodyEntity));

   mLinearDampings[mMapEntityToComponentIndex[bodyEntity]] = linearDamping;
}

// Set the angular damping factor of an entity
inline void DynamicsComponents::setAngularDamping(Entity bodyEntity, decimal angularDamping) {

   assert(mMapEntityToComponentIndex.containsKey(bodyEntity));

   mAngularDampings[mMapEntityToComponentIndex[bodyEntity]] = angularDamping;
}

// Set the initial mass of an entity
inline void DynamicsComponents::setInitMass(Entity bodyEntity, decimal initMass) {

   assert(mMapEntityToComponentIndex.containsKey(bodyEntity));

   mInitMasses[mMapEntityToComponentIndex[bodyEntity]] = initMass;
}

// Set the mass inverse of an entity
inline void DynamicsComponents::setMassInverse(Entity bodyEntity, decimal inverseMass) {

   assert(mMapEntityToComponentIndex.containsKey(bodyEntity));

   mInverseMasses[mMapEntityToComponentIndex[bodyEntity]] = inverseMass;
}

// Set the inverse local inertia tensor of an entity
inline void DynamicsComponents::setInverseInertiaTensorLocal(Entity bodyEntity, const Matrix3x3& inertiaTensorLocalInverse) {

   assert(mMapEntityToComponentIndex.containsKey(bodyEntity));

   mInverseInertiaTensorsLocal[mMapEntityToComponentIndex[bodyEntity]] = inertiaTensorLocalInverse;
}

// Set the inverse world inertia tensor of an entity
inline void DynamicsComponents::setInverseInertiaTensorWorld(Entity bodyEntity, const Matrix3x3& inertiaTensorWorldInverse) {

   assert(mMapEntityToComponentIndex.containsKey(bodyEntity));

   mInverseInertiaTensorsWorld[mMapEntityToComponentIndex[bodyEntity]] = inertiaTensorWorldInverse;
}

// Set the constrained position of an entity
inline void DynamicsComponents::setConstrainedPosition(Entity bodyEntity, const Vector3& constrainedPosition) {

   assert(mMapEntityToComponentIndex.containsKey(bodyEntity));

   mConstrainedPositions[mMapEntityToComponentIndex[bodyEntity]] = constrainedPosition;
}

// Set the constrained orientation of an entity
inline void DynamicsComponents::setConstrainedOrientation(Entity bodyEntity, const Quaternion& constrainedOrientation) {

   assert(mMapEntityToComponentIndex.containsKey(bodyEntity));

   mConstrainedOrientations[mMapEntityToComponentIndex[bodyEntity]] = constrainedOrientation;
}

// Set the local center of mass of an entity
inline void DynamicsComponents::setCenterOfMassLocal(Entity bodyEntity, const Vector3& centerOfMassLocal) {

   assert(mMapEntityToComponentIndex.containsKey(bodyEntity));

   mCentersOfMassLocal[mMapEntityToComponentIndex[bodyEntity]] = centerOfMassLocal;
}

// Set the world center of mass of an entity
inline void DynamicsComponents::setCenterOfMassWorld(Entity bodyEntity, const Vector3& centerOfMassWorld) {

   assert(mMapEntityToComponentIndex.containsKey(bodyEntity));

   mCentersOfMassWorld[mMapEntityToComponentIndex[bodyEntity]] = centerOfMassWorld;
}

// Return true if gravity is enabled for this entity
inline bool DynamicsComponents::getIsGravityEnabled(Entity bodyEntity) const {

   assert(mMapEntityToComponentIndex.containsKey(bodyEntity));

   return mIsGravityEnabled[mMapEntityToComponentIndex[bodyEntity]];
}

// Return true if the entity is already in an island
inline bool DynamicsComponents::getIsAlreadyInIsland(Entity bodyEntity) const {

   assert(mMapEntityToComponentIndex.containsKey(bodyEntity));

   return mIsAlreadyInIsland[mMapEntityToComponentIndex[bodyEntity]];
}

// Set the value to know if the gravity is enabled for this entity
inline void DynamicsComponents::setIsGravityEnabled(Entity bodyEntity, bool isGravityEnabled) {

   assert(mMapEntityToComponentIndex.containsKey(bodyEntity));

   mIsGravityEnabled[mMapEntityToComponentIndex[bodyEntity]] = isGravityEnabled;
}

// Set the value to know if the entity is already in an island
inline void DynamicsComponents::setIsAlreadyInIsland(Entity bodyEntity, bool isAlreadyInIsland) {

   assert(mMapEntityToComponentIndex.containsKey(bodyEntity));

   mIsAlreadyInIsland[mMapEntityToComponentIndex[bodyEntity]] = isAlreadyInIsland;
}

}

#endif
