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

#ifndef REACTPHYSICS3D_ISLANDS_H
#define REACTPHYSICS3D_ISLANDS_H

// Libraries
#include <reactphysics3d/configuration.h>
#include <reactphysics3d/containers/Array.h>
#include <reactphysics3d/engine/Entity.h>
#include <reactphysics3d/constraint/Joint.h>

namespace reactphysics3d {

// Declarations

// Structure Islands
/**
 * This class contains all the islands of bodies during a frame.
 * An island represent an isolated group of awake bodies that are connected with each other by
 * some contraints (contacts or joints).
 */
struct Islands {

    private:

        /// Number of islands in the previous frame
        uint32 mNbIslandsPreviousFrame;

        /// Number of items in the bodyEntities array in the previous frame
        uint32 mNbBodyEntitiesPreviousFrame;

        /// Maximum number of bodies in a single island in the previous frame
        uint32 mNbMaxBodiesInIslandPreviousFrame;

        /// Maximum number of bodies in a single island in the current frame
        uint32 mNbMaxBodiesInIslandCurrentFrame;

    public:

        // -------------------- Attributes -------------------- //


        /// For each island, index of the first contact manifold of the island in the array of contact manifolds
        Array<uint> contactManifoldsIndices;

        /// For each island, number of contact manifolds in the island
        Array<uint> nbContactManifolds;

        /// Array of all the entities of the bodies in the islands (stored sequentially)
        Array<Entity> bodyEntities;

        /// For each island we store the starting index of the bodies of that island in the "bodyEntities" array
        Array<uint32> startBodyEntitiesIndex;

        /// For each island, total number of bodies in the island
        Array<uint32> nbBodiesInIsland;

        // -------------------- Methods -------------------- //

        /// Constructor
        Islands(MemoryAllocator& allocator)
            :mNbIslandsPreviousFrame(16), mNbBodyEntitiesPreviousFrame(32), mNbMaxBodiesInIslandPreviousFrame(0), mNbMaxBodiesInIslandCurrentFrame(0),
             contactManifoldsIndices(allocator), nbContactManifolds(allocator),
             bodyEntities(allocator), startBodyEntitiesIndex(allocator), nbBodiesInIsland(allocator) {

        }

        /// Destructor
        ~Islands() = default;

        /// Assignment operator
        Islands& operator=(const Islands& island) = delete;

        /// Copy-constructor
        Islands(const Islands& island) = default;

        /// Return the number of islands
        uint32 getNbIslands() const {
            return static_cast<uint32>(contactManifoldsIndices.size());
        }

        /// Add an island and return its index
        uint32 addIsland(uint32 contactManifoldStartIndex) {

            const uint32 islandIndex = static_cast<uint32>(contactManifoldsIndices.size());

            contactManifoldsIndices.add(contactManifoldStartIndex);
            nbContactManifolds.add(0);
            startBodyEntitiesIndex.add(static_cast<uint32>(bodyEntities.size()));
            nbBodiesInIsland.add(0);

            if (islandIndex > 0 && nbBodiesInIsland[islandIndex-1] > mNbMaxBodiesInIslandCurrentFrame) {
                mNbMaxBodiesInIslandCurrentFrame = nbBodiesInIsland[islandIndex-1];
            }

            return islandIndex;
        }

        void addBodyToIsland(Entity bodyEntity) {

            const uint32 islandIndex = static_cast<uint32>(contactManifoldsIndices.size());
            assert(islandIndex > 0);

            bodyEntities.add(bodyEntity);
            nbBodiesInIsland[islandIndex - 1]++;
        }

        /// Reserve memory for the current frame
        void reserveMemory() {

            contactManifoldsIndices.reserve(mNbIslandsPreviousFrame);
            nbContactManifolds.reserve(mNbIslandsPreviousFrame);
            startBodyEntitiesIndex.reserve(mNbIslandsPreviousFrame);
            nbBodiesInIsland.reserve(mNbIslandsPreviousFrame);

            bodyEntities.reserve(mNbBodyEntitiesPreviousFrame);
        }

        /// Clear all the islands
        void clear() {

            const uint32 nbIslands = static_cast<uint32>(nbContactManifolds.size());

            if (nbIslands > 0 && nbBodiesInIsland[nbIslands-1] > mNbMaxBodiesInIslandCurrentFrame) {
                mNbMaxBodiesInIslandCurrentFrame = nbBodiesInIsland[nbIslands-1];
            }

            mNbMaxBodiesInIslandPreviousFrame = mNbMaxBodiesInIslandCurrentFrame;
            mNbIslandsPreviousFrame = nbIslands;
            mNbMaxBodiesInIslandCurrentFrame = 0;
            mNbBodyEntitiesPreviousFrame = static_cast<uint32>(bodyEntities.size());

            contactManifoldsIndices.clear(true);
            nbContactManifolds.clear(true);
            bodyEntities.clear(true);
            startBodyEntitiesIndex.clear(true);
            nbBodiesInIsland.clear(true);
        }

        uint32 getNbMaxBodiesInIslandPreviousFrame() const {
            return mNbMaxBodiesInIslandPreviousFrame;
        }
};

}

#endif
