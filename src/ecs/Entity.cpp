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
#include <reactphysics3d/engine/Entity.h>
#include <cassert>

using namespace reactphysics3d;

// Static members initialization
const uint32 Entity::ENTITY_INDEX_BITS = 24;
const uint32 Entity::ENTITY_INDEX_MASK = (1 << Entity::ENTITY_INDEX_BITS) - 1;
const uint32 Entity::ENTITY_GENERATION_BITS = 8;
const uint32 Entity::ENTITY_GENERATION_MASK = (1 << Entity::ENTITY_GENERATION_BITS) - 1;
const uint32 Entity::MINIMUM_FREE_INDICES = 1024;
const uint32 Entity::MAXIMUM_COMPONENTS = 16;

// Constructor
Entity::Entity(uint32 index, uint32 generation)
       :id((index & ENTITY_INDEX_MASK) | ((generation & ENTITY_GENERATION_MASK) << ENTITY_INDEX_BITS)) {

    assert(getIndex() == index);
    assert(getGeneration() == generation);
}
