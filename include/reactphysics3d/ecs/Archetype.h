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

#ifndef REACTPHYSICS3D_ARCHETYPE_H
#define REACTPHYSICS3D_ARCHETYPE_H

// Libraries
#include <reactphysics3d/configuration.h>
#include <reactphysics3d/ecs/Entity.h>
#include <reactphysics3d/memory/ArchetypeAllocator.h>

/// Namespace reactphysics3d
namespace reactphysics3d {

// Forward declarations
class ComponentsManager;

// Class Archetype
class Archetype {

    private:

        /// Struct Column
        /// This is used to know where an array of components is stored in the chunk
        struct Column {

            void* start;
            size_t componentSize;

            // Constructor
            Column(void* start, size_t componentSize)
                : start(start), componentSize(componentSize) {

            }
        };

        // --------- Attributes ---------- //

        /// Reference to the components manager
        ComponentsManager& mComponentsManager;

        /// Signature of the archetype
        Entity::Signature mSignature;

        /// Pointer to the first chunk of the archetype
        ArchetypeAllocator::ChunkHeader* mFirstChunk = nullptr;

        /// Pointer to the last chunk of the archetype
        ArchetypeAllocator::ChunkHeader* mLastChunk = nullptr;

        /// Map an entity with its index in the arrays of the archetype
        Map<Entity, uint32> mMapEntityToIndex;

        /// Archetype allocator
        ArchetypeAllocator& mArchetypeAllocator;

        /// Map a component id to its corresponding column (pointers to the components arrays)
        Map<Entity::ComponentId, Column> mMapComponentIdToColumns;

        /// Number of entities of that archetype
        uint32 mNbEntities = 0;

        /// Maximum number of entities that can fit in a chunk for this archetype
        uint32 mNbMaxEntities = 0;

        // --------- Methods ---------- //

        /// Allocate a new chunk of memory for the components of this archetype
        void allocateChunk();

        /// Release the last chunk of memory
        void releaseChunk();

        /// Initialize the columns
        void initColumns(const Array<Entity::ComponentId>& componentIds);

    public:

        /// Constructor
        Archetype(ComponentsManager& componentsManager, const Array<Entity::ComponentId>& componentIds, MemoryAllocator& baseAllocator,
                  ArchetypeAllocator& archetypeAllocator);

        /// Return the number of entities of that archetype
        uint32 getNbEntities() const;

        /// Return a pointer to the component of a given type of the first entity
        template<typename T>
        const T* getComponentsArray() const;


        void setColumns(const Map<Entity::ComponentId, Column> &newColumns);
};

// Return the number of entities of that archetype
RP3D_FORCE_INLINE uint32 Archetype::getNbEntities() const {
    return mNbEntities;
}

}

#endif
