/********************************************************************************
* ReactPhysics3D physics library, http://www.reactphysics3d.com                 *
* Copyright (c) 2010-2020 Daniel Chappuis                                       *
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
#include <reactphysics3d/containers/List.h>
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

        // -------------------- Attributes -------------------- //

        /// Reference to the memory allocator
        MemoryAllocator& memoryAllocator;

    public:

        // -------------------- Attributes -------------------- //


        /// For each island, index of the first contact manifold of the island in the array of contact manifolds
        List<uint> contactManifoldsIndices;

        /// For each island, number of contact manifolds in the island
        List<uint> nbContactManifolds;

        /// For each island, list of all the entities of the bodies in the island
        List<List<Entity>> bodyEntities;

        // -------------------- Methods -------------------- //

        /// Constructor
        Islands(MemoryAllocator& allocator)
            :memoryAllocator(allocator), contactManifoldsIndices(allocator), nbContactManifolds(allocator),
             bodyEntities(allocator) {

        }

        /// Destructor
        ~Islands() = default;

        /// Assignment operator
        Islands& operator=(const Islands& island) = default;

        /// Copy-constructor
        Islands(const Islands& island) = default;

        /// Return the number of islands
        uint32 getNbIslands() const {
            return static_cast<uint32>(contactManifoldsIndices.size());
        }

        /// Add an island and return its index
        uint32 addIsland(uint32 contactManifoldStartIndex) {

            uint32 islandIndex = contactManifoldsIndices.size();

            contactManifoldsIndices.add(contactManifoldStartIndex);
            nbContactManifolds.add(0);
            bodyEntities.add(List<Entity>(memoryAllocator));

            return islandIndex;
        }

        /// Clear all the islands
        void clear() {

            contactManifoldsIndices.clear(true);
            nbContactManifolds.clear(true);
            bodyEntities.clear(true);
        }
};

}

#endif
