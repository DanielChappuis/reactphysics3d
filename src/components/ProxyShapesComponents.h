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

#ifndef REACTPHYSICS3D_PROXY_SHAPES_COMPONENTS_H
#define REACTPHYSICS3D_PROXY_SHAPES_COMPONENTS_H

// Libraries
#include "mathematics/Transform.h"
#include "engine/Entity.h"
#include "containers/Map.h"
#include "collision/shapes/AABB.h"

// ReactPhysics3D namespace
namespace reactphysics3d {

// Class declarations
class MemoryAllocator;
class EntityManager;
class AABB;
class CollisionShape;

// Class ProxyShapesComponents
/**
 * This class represent the component of the ECS that contains the proxy-shapes of the
 * different entities. There can be several proxy shapes for each entity (body). We store
 * the proxy shapes in a flat array but we keep two arrays with previous/next proxy shapes
 * link information to quickly know all the proxy shapes of a given entity (body). We also make
 * sure that proxy shapes of sleeping entities (bodies) are always stored at the end of the array.
 */
class ProxyShapesComponents {

    private:

        // -------------------- Constants -------------------- //

        /// Number of components to allocated at the beginning
        const uint32 INIT_ALLOCATED_COMPONENTS = 10;

        /// Number of valid entities to hit before stopping garbage collection
        const uint32 GARBAGE_COLLECTION_MAX_VALID_ENTITIES = 5;

        const size_t COMPONENT_DATA_SIZE = sizeof(Entity) + sizeof(int) + sizeof(AABB) +
                sizeof(Transform) + sizeof(CollisionShape*) + sizeof(decimal) + sizeof(uint32) +
                sizeof(uint32);

        // -------------------- Attributes -------------------- //

        /// Memory allocator
        MemoryAllocator& mMemoryAllocator;

        /// Current number of components
        uint32 mNbComponents;

        /// Number of allocated components
        uint32 mNbAllocatedComponents;

        /// Index of the first component of a sleeping entity (sleeping components are stored at the end)
        uint32 mSleepingStartIndex;

        /// Allocated memory for all the data of the components
        void* mBuffer;

        /// Map an entity to the index of its component in the array
        Map<Entity, uint32> mMapEntityToComponentIndex;

        /// Array of entities of each component
        Entity* mEntities;

        /// Ids of the proxy-shapes for the broad-phase algorithm
        // TODO : Try to change type to uint32
        int* mBroadPhaseIds;

        /// Local-space bounds of a proxy-shape
        AABB* mLocalBounds;

        /// Transform from local-space of the proxy-shape to the body-space of its body
        Transform* mLocalToBodyTransforms;

        /// Pointers to the collision shapes of the proxy-shapes
        CollisionShape** mCollisionShapes;

        /// Masses of the proxy-shapes
        decimal* mMasses;

        /// Index of the previous proxy-shape in the same body
        /// mPreviousBodyProxyShapes[i] == i means that the proxy-shape component has no previous proxy-shape
        uint32* mPreviousBodyProxyShapes;

        /// Index of the next proxy-shape in the same body
        /// mNextBodyProxyShapes[i] == i means that the proxy-shape component has no next proxy-shape
        uint32* mNextBodyProxyShapes;

        // -------------------- Methods -------------------- //

        /// Remove a component at a given index
        void removeComponent(uint32 index);

        /// Destroy a component at a given index
        void destroyComponent(uint32 index);

        /// Move a component from a source to a destination index in the components array
        void moveComponentToIndex(uint32 srcIndex, uint32 destIndex);

        /// Swap two components in the array
        void swapComponents(uint32 index1, uint32 index2);

        /// Add a new proxy-shape at the end of the linked-list of proxy-shapes of a given entity
        void linkProxyShapeWithEntity(Entity entity, uint32 proxyShapeComponentIndex);

        /// Return true if a given proxy-shape component has a previous proxy-shape in the linked-list of proxy-shapes of a body
        bool hasPreviousProxyShape(uint32 index) const;

        /// Return true if a given proxy-shape component has a next proxy-shape in the linked-list of proxy-shapes of a body
        bool hasNextProxyShape(uint32 index) const;

    public:

        /// Structure for the data of a proxy shape component
        struct ProxyShapeComponent {

            int broadPhaseId;
            AABB localBounds;
            Transform localToBodyTransform;
            CollisionShape* collisionShape;
            decimal mass;

            /// Constructor
            ProxyShapeComponent(int broadPhaseId, AABB localBounds, Transform localToBodyTransform,
                                CollisionShape* collisionShape, decimal mass)
                 :broadPhaseId(broadPhaseId), localBounds(localBounds), localToBodyTransform(localToBodyTransform),
                  collisionShape(collisionShape), mass(mass) {

            }
        };

        // -------------------- Methods -------------------- //

        /// Constructor
        ProxyShapesComponents(MemoryAllocator& allocator);

        /// Destructor
        ~ProxyShapesComponents();

        /// Allocate memory for a given number of components
        void allocate(uint32 nbComponentsToAllocate);

        /// Add a component
        void addComponent(Entity entity, bool isSleeping, const ProxyShapeComponent& component);

        /// Remove all the components of a given entity
        void removeComponents(Entity entity);

        /// Notify if a given entity is sleeping or not
        void setIsEntitySleeping(Entity entity, bool isSleeping);
};

}

#endif
