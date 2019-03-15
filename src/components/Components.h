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

#ifndef REACTPHYSICS3D_COMPONENTS_H
#define REACTPHYSICS3D_COMPONENTS_H

// Libraries
#include "configuration.h"
#include "engine/Entity.h"
#include "containers/Map.h"

// ReactPhysics3D namespace
namespace reactphysics3d {

// Class declarations
class MemoryAllocator;
class EntityManager;

// Class Components
/**
 * This class represent the abstract class to store components of the ECS.
 */
class Components {

    protected:

        // -------------------- Constants -------------------- //

        /// Number of components to allocated at the beginning
        const uint32 INIT_ALLOCATED_COMPONENTS = 10;

        /// Number of valid entities to hit before stopping garbage collection
        const uint32 GARBAGE_COLLECTION_MAX_VALID_ENTITIES = 5;

        // -------------------- Attributes -------------------- //

        /// Memory allocator
        MemoryAllocator& mMemoryAllocator;

        /// Current number of components
        uint32 mNbComponents;

        /// Number of allocated components
        uint32 mNbAllocatedComponents;

        /// Allocated memory for all the data of the components
        void* mBuffer;

        /// Map an entity to the index of its component in the array
        Map<Entity, uint32> mMapEntityToComponentIndex;

    public:

        // -------------------- Methods -------------------- //

        /// Constructor
        Components(MemoryAllocator& allocator)
            : mMemoryAllocator(allocator), mNbComponents(0), mNbAllocatedComponents(0),
              mBuffer(nullptr), mMapEntityToComponentIndex(allocator) {

        }

        /// Destructor
        virtual ~Components() {

        }

        /// Return the number of components
        uint32 getNbComponents() const {
            return mNbComponents;
        }
};

}

#endif
