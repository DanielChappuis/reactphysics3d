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

#ifndef REACTPHYSICS3D_BODY_COMPONENTS_H
#define REACTPHYSICS3D_BODY_COMPONENTS_H

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
class CollisionBody;

// Class CollisionBodyComponents
/**
 * This class represent the component of the ECS that contains data about a collision body.
 * The components of the sleeping entities (bodies) are always stored at the end of the array.
 */
class CollisionBodyComponents : public Components {

    private:

        // -------------------- Attributes -------------------- //

        /// Array of body entities of each component
        Entity* mBodiesEntities;

        /// Array of pointers to the corresponding bodies
        CollisionBody** mBodies;

        /// Array with the list of proxy-shapes of each body
        List<Entity>* mProxyShapes;

        /// Array of boolean values to know if the body is allowed to go to sleep
        bool* mIsAllowedToSleep;

        /// Array of boolean values to know if the body is active.
        bool* mIsActive;

        /// Array of boolean values to know if the body is sleeping
        bool* mIsSleeping;

        /// Array with values for elapsed time since the body velocity was below the sleep velocity
        decimal* mSleepTimes;

        /// Array of pointers that can be used to attach user data to the body
        void** mUserData;

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

        /// Structure for the data of a body component
        struct BodyComponent {

            CollisionBody* body;

            /// Constructor
            BodyComponent(CollisionBody* body) : body(body) {

            }
        };

        // -------------------- Methods -------------------- //

        /// Constructor
        CollisionBodyComponents(MemoryAllocator& allocator);

        /// Destructor
        virtual ~CollisionBodyComponents() override = default;

        /// Add a component
        void addComponent(Entity bodyEntity, bool isSleeping, const BodyComponent& component);

        /// Add a proxy-shape to a body component
        void addProxyShapeToBody(Entity bodyEntity, Entity proxyShapeEntity);

        /// Set the transform of an entity
        void removeProxyShapeFromBody(Entity bodyEntity, Entity proxyShapeEntity);

        /// Return a pointer to a body
        CollisionBody* getBody(Entity bodyEntity);

        /// Return the list of proxy-shapes of a body
        const List<Entity>& getProxyShapes(Entity bodyEntity) const;

        /// Return true if the body is allowed to sleep
        bool getIsAllowedToSleep(Entity bodyEntity) const;

        /// Set the value to know if the body is allowed to sleep
        void setIsAllowedToSleep(Entity bodyEntity, bool isAllowedToSleep) const;

        /// Return true if the body is active
        bool getIsActive(Entity bodyEntity) const;

        /// Set the value to know if the body is active
        void setIsActive(Entity bodyEntity, bool isActive) const;

        /// Return true if the body is sleeping
        bool getIsSleeping(Entity bodyEntity) const;

        /// Set the value to know if the body is sleeping
        void setIsSleeping(Entity bodyEntity, bool isSleeping) const;

        /// Return the sleep time
        decimal getSleepTime(Entity bodyEntity) const;

        /// Set the sleep time
        void setSleepTime(Entity bodyEntity, decimal sleepTime) const;

        /// Return the user data associated with the body
        void* getUserData(Entity bodyEntity) const;

        /// Set the user data associated with the body
        void setUserData(Entity bodyEntity, void* userData) const;
};

// Add a proxy-shape to a body component
inline void CollisionBodyComponents::addProxyShapeToBody(Entity bodyEntity, Entity proxyShapeEntity) {

    assert(mMapEntityToComponentIndex.containsKey(bodyEntity));

    mProxyShapes[mMapEntityToComponentIndex[bodyEntity]].add(proxyShapeEntity);
}

// Set the transform of an entity
inline void CollisionBodyComponents::removeProxyShapeFromBody(Entity bodyEntity, Entity proxyShapeEntity) {

    assert(mMapEntityToComponentIndex.containsKey(bodyEntity));

    mProxyShapes[mMapEntityToComponentIndex[bodyEntity]].remove(proxyShapeEntity);
}

// Return a pointer to a body
inline CollisionBody *CollisionBodyComponents::getBody(Entity bodyEntity) {

    assert(mMapEntityToComponentIndex.containsKey(bodyEntity));

    return mBodies[mMapEntityToComponentIndex[bodyEntity]];
}

// Return the list of proxy-shapes of a body
inline const List<Entity>& CollisionBodyComponents::getProxyShapes(Entity bodyEntity) const {

    assert(mMapEntityToComponentIndex.containsKey(bodyEntity));

    return mProxyShapes[mMapEntityToComponentIndex[bodyEntity]];
}

// Return true if the body is allowed to sleep
inline bool CollisionBodyComponents::getIsAllowedToSleep(Entity bodyEntity) const {

    assert(mMapEntityToComponentIndex.containsKey(bodyEntity));

    return mIsAllowedToSleep[mMapEntityToComponentIndex[bodyEntity]];
}

// Set the value to know if the body is allowed to sleep
inline void CollisionBodyComponents::setIsAllowedToSleep(Entity bodyEntity, bool isAllowedToSleep) const {

    assert(mMapEntityToComponentIndex.containsKey(bodyEntity));

    mIsAllowedToSleep[mMapEntityToComponentIndex[bodyEntity]] = isAllowedToSleep;
}

// Return true if the body is active
inline bool CollisionBodyComponents::getIsActive(Entity bodyEntity) const {

    assert(mMapEntityToComponentIndex.containsKey(bodyEntity));

    return mIsActive[mMapEntityToComponentIndex[bodyEntity]];
}

// Set the value to know if the body is active
inline void CollisionBodyComponents::setIsActive(Entity bodyEntity, bool isActive) const {

    assert(mMapEntityToComponentIndex.containsKey(bodyEntity));

    mIsActive[mMapEntityToComponentIndex[bodyEntity]] = isActive;
}

// Return true if the body is sleeping
inline bool CollisionBodyComponents::getIsSleeping(Entity bodyEntity) const {

    assert(mMapEntityToComponentIndex.containsKey(bodyEntity));

    return mIsSleeping[mMapEntityToComponentIndex[bodyEntity]];
}

// Set the value to know if the body is sleeping
inline void CollisionBodyComponents::setIsSleeping(Entity bodyEntity, bool isSleeping) const {

    assert(mMapEntityToComponentIndex.containsKey(bodyEntity));

    mIsSleeping[mMapEntityToComponentIndex[bodyEntity]] = isSleeping;
}

// Return the sleep time
inline decimal CollisionBodyComponents::getSleepTime(Entity bodyEntity) const {

    assert(mMapEntityToComponentIndex.containsKey(bodyEntity));

    return mSleepTimes[mMapEntityToComponentIndex[bodyEntity]];
}

// Set the sleep time
inline void CollisionBodyComponents::setSleepTime(Entity bodyEntity, decimal sleepTime) const {

    assert(mMapEntityToComponentIndex.containsKey(bodyEntity));

    mSleepTimes[mMapEntityToComponentIndex[bodyEntity]] = sleepTime;
}

// Return the user data associated with the body
inline void* CollisionBodyComponents::getUserData(Entity bodyEntity) const {

    assert(mMapEntityToComponentIndex.containsKey(bodyEntity));

    return mUserData[mMapEntityToComponentIndex[bodyEntity]];
}

// Set the user data associated with the body
inline void CollisionBodyComponents::setUserData(Entity bodyEntity, void* userData) const {

    assert(mMapEntityToComponentIndex.containsKey(bodyEntity));

    mUserData[mMapEntityToComponentIndex[bodyEntity]] = userData;
}

}

#endif
