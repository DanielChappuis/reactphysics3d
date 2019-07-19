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

#ifndef REACTPHYSICS3D_RIGID_BODY_COMPONENTS_H
#define REACTPHYSICS3D_RIGID_BODY_COMPONENTS_H

// Libraries
#include "mathematics/Transform.h"
#include "engine/Entity.h"
#include "components/Components.h"
#include "containers/Map.h"

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

            /// Constructor
            RigidBodyComponent(RigidBody* body, BodyType bodyType) : body(body), bodyType(bodyType) {

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
};

// Return a pointer to a body rigid
inline RigidBody* RigidBodyComponents::getRigidBody(Entity bodyEntity) {

    assert(mMapEntityToComponentIndex.containsKey(bodyEntity));

    return mRigidBodies[mMapEntityToComponentIndex[bodyEntity]];
}

// Return true if the body is allowed to sleep
inline bool RigidBodyComponents::getIsAllowedToSleep(Entity bodyEntity) const {

    assert(mMapEntityToComponentIndex.containsKey(bodyEntity));

    return mIsAllowedToSleep[mMapEntityToComponentIndex[bodyEntity]];
}

// Set the value to know if the body is allowed to sleep
inline void RigidBodyComponents::setIsAllowedToSleep(Entity bodyEntity, bool isAllowedToSleep) const {

    assert(mMapEntityToComponentIndex.containsKey(bodyEntity));

    mIsAllowedToSleep[mMapEntityToComponentIndex[bodyEntity]] = isAllowedToSleep;
}

// Return true if the body is sleeping
inline bool RigidBodyComponents::getIsSleeping(Entity bodyEntity) const {

    assert(mMapEntityToComponentIndex.containsKey(bodyEntity));

    return mIsSleeping[mMapEntityToComponentIndex[bodyEntity]];
}

// Set the value to know if the body is sleeping
inline void RigidBodyComponents::setIsSleeping(Entity bodyEntity, bool isSleeping) const {

    assert(mMapEntityToComponentIndex.containsKey(bodyEntity));

    mIsSleeping[mMapEntityToComponentIndex[bodyEntity]] = isSleeping;
}

// Return the sleep time
inline decimal RigidBodyComponents::getSleepTime(Entity bodyEntity) const {

    assert(mMapEntityToComponentIndex.containsKey(bodyEntity));

    return mSleepTimes[mMapEntityToComponentIndex[bodyEntity]];
}

// Set the sleep time
inline void RigidBodyComponents::setSleepTime(Entity bodyEntity, decimal sleepTime) const {

    assert(mMapEntityToComponentIndex.containsKey(bodyEntity));

    mSleepTimes[mMapEntityToComponentIndex[bodyEntity]] = sleepTime;
}

// Return the body type of a body
inline BodyType RigidBodyComponents::getBodyType(Entity bodyEntity) {

   assert(mMapEntityToComponentIndex.containsKey(bodyEntity));

   return mBodyTypes[mMapEntityToComponentIndex[bodyEntity]];
}

// Set the body type of a body
inline void RigidBodyComponents::setBodyType(Entity bodyEntity, BodyType bodyType) {

   assert(mMapEntityToComponentIndex.containsKey(bodyEntity));

   mBodyTypes[mMapEntityToComponentIndex[bodyEntity]] = bodyType;
}

// Return the linear velocity of an entity
inline const Vector3& RigidBodyComponents::getLinearVelocity(Entity bodyEntity) const {

   assert(mMapEntityToComponentIndex.containsKey(bodyEntity));

   return mLinearVelocities[mMapEntityToComponentIndex[bodyEntity]];
}

// Return the angular velocity of an entity
inline const Vector3& RigidBodyComponents::getAngularVelocity(Entity bodyEntity) const {

   assert(mMapEntityToComponentIndex.containsKey(bodyEntity));

   return mAngularVelocities[mMapEntityToComponentIndex[bodyEntity]];
}

// Set the linear velocity of an entity
inline void RigidBodyComponents::setLinearVelocity(Entity bodyEntity, const Vector3& linearVelocity) {

   assert(mMapEntityToComponentIndex.containsKey(bodyEntity));

   mLinearVelocities[mMapEntityToComponentIndex[bodyEntity]] = linearVelocity;
}

// Set the angular velocity of an entity
inline void RigidBodyComponents::setAngularVelocity(Entity bodyEntity, const Vector3& angularVelocity) {

   assert(mMapEntityToComponentIndex.containsKey(bodyEntity));

   mAngularVelocities[mMapEntityToComponentIndex[bodyEntity]] = angularVelocity;
}


}

#endif
