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

#ifndef REACTPHYSICS3D_ENTITY_H
#define REACTPHYSICS3D_ENTITY_H

// Libraries
#include <reactphysics3d/configuration.h>

/// Namespace reactphysics3d
namespace reactphysics3d {

// Structure Entity
/**
 * This class is used to identify an entity in the Entity-Component-System.
 * Entities are used for bodies in the physics engine. The id of an entity is
 * a 32 bits integer that is separated in two parts. The index and the generation
 * parts. The idea is that the index part directly gives us the index of the entity
 * in a lookup array. However, we want to support deletion of the entities. That's why
 * we have the generation part. This number is used to distinguish the entities created
 * at the same index in the array. If it is the case, we will increase the generation number
 * each time.
 *
 * We use 24 bits for the index. Therefore, we support 16 millions simultaneous entries.
 * We use 8 bits for the generation. Therefore, we support 256 entries created at the same index.
 * To prevent reaching the 256 entries too fast, we make sure that we do not reuse the same index
 * slot too ofen. To do that, we put recycled indices in a queue and only reuse values from that
 * queue when it contains at least MINIMUM_FREE_INDICES values. It means, an id will reappear until
 * its index has run 256 laps through the queue. It means that we must create and destroy at least
 * 256 * MINIMUM_FREE_INDICES entities until an id can reappear. This seems reasonably safe.
 *
 * This implementation is based on the following article:
 * http://bitsquid.blogspot.com/2014/08/building-data-oriented-entity-system.html
 */
struct Entity {

    private:

        /// Number of bits reserved for the index
        static const uint32 ENTITY_INDEX_BITS;

        /// Mask for the index part of the id
        static const uint32 ENTITY_INDEX_MASK;

        /// Number of bits reserved for the generation number
        static const uint32 ENTITY_GENERATION_BITS;

        /// Mask for the generation part of the id
        static const uint32 ENTITY_GENERATION_MASK;

        /// Minimum of free indices in the queue before we reuse one from the queue
        static const uint32 MINIMUM_FREE_INDICES;

    public:

        // -------------------- Attributes -------------------- //

        /// Id of the entity
        uint32 id;

        // -------------------- Methods -------------------- //

        /// Constructor
        Entity(uint32 index, uint32 generation);

        /// Return the lookup index of the entity in a array
        uint32 getIndex() const;

        /// Return the generation number of the entity
        uint32 getGeneration() const;

        /// Equality operator
        bool operator==(const Entity& entity) const;

        /// Inequality operator
        bool operator!=(const Entity& entity) const;

        // -------------------- Friendship -------------------- //

        friend class EntityManager;
};

// Return the lookup index of the entity in a array
RP3D_FORCE_INLINE uint32 Entity::getIndex() const {
    return id & ENTITY_INDEX_MASK;
}

// Return the generation number of the entity
RP3D_FORCE_INLINE uint32 Entity::getGeneration() const {
    return (id >> ENTITY_INDEX_BITS) & ENTITY_GENERATION_MASK;
}

// Equality operator
RP3D_FORCE_INLINE bool Entity::operator==(const Entity& entity) const {

    return entity.id == id;
}

// Inequality operator
RP3D_FORCE_INLINE bool Entity::operator!=(const Entity& entity) const {
    return entity.id != id;
}

}

// Hash function for a reactphysics3d Entity
namespace std {

  template <> struct hash<reactphysics3d::Entity> {

    size_t operator()(const reactphysics3d::Entity& entity) const {

        return std::hash<reactphysics3d::uint32>{}(entity.id);
    }
  };
}

#endif
