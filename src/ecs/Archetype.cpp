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

// Libraries
#include <reactphysics3d/ecs/Archetype.h>
#include <reactphysics3d/ecs/ComponentsManager.h>

using namespace reactphysics3d;

// Constructor
Archetype::Archetype(ComponentsManager& componentsManager, const Array<Entity::ComponentId>& componentIds, MemoryAllocator& baseAllocator,
                     ArchetypeAllocator& archetypeAllocator)
    : mComponentsManager(componentsManager), mMapEntityToIndex(baseAllocator),
      mArchetypeAllocator(archetypeAllocator), mColumns(baseAllocator) {

    // Compute archetype signature
    for (uint32 i=0; i < componentIds.size(); i++) {
        mSignature.set(componentIds[i], true);
    }

    // Allocate a new chunk of memory
    allocateChunk();

}

// Initialize the columns
void Archetype::initColumns(const Array<Entity::ComponentId>& componentIds) {

    unsigned char* start = static_cast<unsigned char*>(mFirstChunk->getDataStart());

    // For each component
    size_t totalSize = 0;
    for (uint32 i=0; i < componentIds.size(); i++) {
        totalSize += mComponentsManager.getComponentSize(componentIds[i]);
    }

    // Maximum number of components
    mNbMaxEntities = totalSize / ArchetypeAllocator::CHUNK_DATA_SIZE;

    // For each component
    for (uint32 i=0; i < componentIds.size(); i++) {

        const size_t componentSize = mComponentsManager.getComponentSize(componentIds[i]);

        // Add a column to store all the components of the same type
        mMapComponentIdToColumns.add(Pair<Entity::ComponentId, Column>(componentIds[i], Column(static_cast<void*>(start), componentSize)));

        start += componentSize * mNbMaxEntities;
    }
}

// Allocate a new chunk of memory for the components of this archetype
void Archetype::setColumns(const Map<Entity::ComponentId, Column>& newColumns)
{
    mMapComponentIdToColumns = newColumns;
}

void Archetype::allocateChunk() {

    // Allocate a new chunk of memory
    ArchetypeAllocator::ChunkHeader* newChunk = mArchetypeAllocator.allocate();

    if (mLastChunk != nullptr) {

        assert(mLastChunk->nextChunk == nullptr);
        mLastChunk->nextChunk = newChunk;
    }
    else {
        mFirstChunk = newChunk;
        mLastChunk = newChunk;
    }
}

// Release the last chunk of memory
void Archetype::releaseChunk() {

}

// Return the array of components of a given type
template<typename T>
const T* Archetype::getComponentsArray() const {
    const Column& column = mMapComponentIdToColumns[mComponentsManager.getComponentId<T>()];
    return reinterpret_cast<T*>(column.start);
}

