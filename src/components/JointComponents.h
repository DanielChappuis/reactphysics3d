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

#ifndef REACTPHYSICS3D_JOINT_COMPONENTS_H
#define REACTPHYSICS3D_JOINT_COMPONENTS_H

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

// Class JointComponents
/**
 * This class represent the component of the ECS that contains generic information about
 * all the joints.
 */
class JointComponents : public Components {

    private:

        // -------------------- Attributes -------------------- //

        /// Array of joint entities
        Entity* mJointEntities;

        /// Array of body entities of the first bodies of the joints
        Entity* mBody1Entities;

        /// Array of body entities of the first bodies of the joints
        Entity* mBody2Entities;

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
        struct JointComponent {

            const Entity body1Entity;
            const Entity body2Entity;

            /// Constructor
            JointComponent(Entity body1Entity, Entity body2Entity)
                : body1Entity(body1Entity), body2Entity(body2Entity)  {

            }
        };

        // -------------------- Methods -------------------- //

        /// Constructor
        JointComponents(MemoryAllocator& allocator);

        /// Destructor
        virtual ~JointComponents() override = default;

        /// Add a component
        void addComponent(Entity jointEntity, bool isSleeping, const JointComponent& component);

        /// Return the entity of the first body of a joint
        Entity getBody1Entity(Entity jointEntity) const;

        /// Return the entity of the second body of a joint
        Entity getBody2Entity(Entity jointEntity) const;

        // -------------------- Friendship -------------------- //

        friend class BroadPhaseSystem;
};

// Return the entity of the first body of a joint
inline Entity JointComponents::getBody1Entity(Entity jointEntity) const {
    assert(mMapEntityToComponentIndex.containsKey(jointEntity));
    return mBody1Entities[mMapEntityToComponentIndex[jointEntity]];
}

// Return the entity of the second body of a joint
inline Entity JointComponents::getBody2Entity(Entity jointEntity) const {
    assert(mMapEntityToComponentIndex.containsKey(jointEntity));
    return mBody2Entities[mMapEntityToComponentIndex[jointEntity]];
}

}

#endif
