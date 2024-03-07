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

#ifndef REACTPHYSICS3D_TRANSFORM_COMPONENTS_H
#define REACTPHYSICS3D_TRANSFORM_COMPONENTS_H

// Libraries
#include <reactphysics3d/mathematics/Transform.h>
#include <reactphysics3d/engine/Entity.h>
#include <reactphysics3d/components/Components.h>
#include <reactphysics3d/containers/Map.h>

// ReactPhysics3D namespace
namespace reactphysics3d {

// Class declarations
class MemoryAllocator;
class EntityManager;

// Class TransformComponents
/**
 * This class represent the component of the ECS that contains the transforms of the
 * different entities. The position and orientation of the bodies are stored there.
 * The components of the sleeping entities (bodies) are always stored at the end of the array.
 */
class TransformComponents : public Components {

    private:

        // -------------------- Attributes -------------------- //

        /// Array of body entities of each component
        Entity* mBodies;

        /// Array of transform of each component
        Transform* mTransforms;

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
        struct TransformComponent {

            const Transform& transform;

            /// Constructor
            TransformComponent(const Transform& transform) : transform(transform) {

            }
        };

        // -------------------- Methods -------------------- //

        /// Constructor
        TransformComponents(MemoryAllocator& allocator);

        /// Destructor
        virtual ~TransformComponents() override = default;

        /// Add a component
        void addComponent(Entity bodyEntity, bool isDisabled, const TransformComponent& component);

        /// Return the transform of an entity
        Transform& getTransform(Entity bodyEntity) const;

        /// Set the transform of an entity
        void setTransform(Entity bodyEntity, const Transform& transform);

        // -------------------- Friendship -------------------- //

        friend class BroadPhaseSystem;
};

// Return the transform of an entity
RP3D_FORCE_INLINE Transform& TransformComponents::getTransform(Entity bodyEntity) const {
    assert(mMapEntityToComponentIndex.containsKey(bodyEntity));
    return mTransforms[mMapEntityToComponentIndex[bodyEntity]];
}

// Set the transform of an entity
RP3D_FORCE_INLINE void TransformComponents::setTransform(Entity bodyEntity, const Transform& transform) {
    assert(mMapEntityToComponentIndex.containsKey(bodyEntity));
    mTransforms[mMapEntityToComponentIndex[bodyEntity]] = transform;
}

}

#endif
