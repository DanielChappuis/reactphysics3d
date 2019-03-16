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
class Body;

// Class BodyComponents
/**
 * This class represent the component of the ECS that contains data about a physics body.
 * The components of the sleeping entities (bodies) are always stored at the end of the array.
 */
class BodyComponents : public Components {

    private:

        // -------------------- Attributes -------------------- //

        /// Array of body entities of each component
        Entity* mBodiesEntities;

        /// Array of pointers to the corresponding bodies
        Body** mBodies;

        /// Array with the list of proxy-shapes of each body
        List<Entity>* mProxyShapes;

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

            Body* body;

            /// Constructor
            BodyComponent(Body* body) : body(body) {

            }
        };

        // -------------------- Methods -------------------- //

        /// Constructor
        BodyComponents(MemoryAllocator& allocator);

        /// Destructor
        virtual ~BodyComponents() override = default;

        /// Add a component
        void addComponent(Entity bodyEntity, bool isSleeping, const BodyComponent& component);

        /// Add a proxy-shape to a body component
        void addProxyShapeToBody(Entity bodyEntity, Entity proxyShapeEntity);

        /// Set the transform of an entity
        void removeProxyShapeFromBody(Entity bodyEntity, Entity proxyShapeEntity);

        /// Return a pointer to a body
        Body* getBody(Entity bodyEntity);

        /// Return the list of proxy-shapes of a body
        const List<Entity>& getProxyShapes(Entity bodyEntity) const;

        /// Notify if a given entity is sleeping or not
        void setIsEntitySleeping(Entity bodyEntity, bool isSleeping);
};

// Add a proxy-shape to a body component
inline void BodyComponents::addProxyShapeToBody(Entity bodyEntity, Entity proxyShapeEntity) {

    assert(mMapEntityToComponentIndex.containsKey(bodyEntity));

    mProxyShapes[mMapEntityToComponentIndex[bodyEntity]].add(proxyShapeEntity);
}

// Set the transform of an entity
inline void BodyComponents::removeProxyShapeFromBody(Entity bodyEntity, Entity proxyShapeEntity) {

    assert(mMapEntityToComponentIndex.containsKey(bodyEntity));

    mProxyShapes[mMapEntityToComponentIndex[bodyEntity]].remove(proxyShapeEntity);
}

// Return a pointer to a body
inline Body* BodyComponents::getBody(Entity bodyEntity) {

    assert(mMapEntityToComponentIndex.containsKey(bodyEntity));

    return mBodies[mMapEntityToComponentIndex[bodyEntity]];
}

// Return the list of proxy-shapes of a body
inline const List<Entity>& BodyComponents::getProxyShapes(Entity bodyEntity) const {

    assert(mMapEntityToComponentIndex.containsKey(bodyEntity));

    return mProxyShapes[mMapEntityToComponentIndex[bodyEntity]];
}

}

#endif
