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

#ifndef REACTPHYSICS3D_BROAD_PHASE_PAIR_H
#define REACTPHYSICS3D_BROAD_PHASE_PAIR_H

// Libraries
#include <functional>
#include <cassert>
#include"containers/Pair.h"


/// Namespace ReactPhysics3D
namespace reactphysics3d {

// Structure BroadPhasePair
/**
 * This structure represent a potential overlapping pair during the
 * broad-phase collision detection.
 */
struct BroadPhasePair {

    public:

        // -------------------- Attributes -------------------- //

        // Broad-phase id of the first collision shape
        int shape1BroadPhaseId;

        // Broad-phase id of the second collision shape
        int shape2BroadPhaseId;

        // -------------------- Methods -------------------- //

        /// Constructor
        BroadPhasePair(int shapeId1, int shapeId2)
            : shape1BroadPhaseId(std::min(shapeId1, shapeId2)),
              shape2BroadPhaseId(std::max(shapeId1, shapeId2)) {

            assert(shape1BroadPhaseId != -1);
            assert(shape2BroadPhaseId != -1);
        }

        /// Equality operator
        bool operator==(const BroadPhasePair& pair) const;

        /// Inequality operator
        bool operator!=(const BroadPhasePair& pair) const;
};

// Equality operator
inline bool BroadPhasePair::operator==(const BroadPhasePair& pair) const {

    return shape1BroadPhaseId == pair.shape1BroadPhaseId && shape2BroadPhaseId == pair.shape2BroadPhaseId;
}

// Inequality operator
inline bool BroadPhasePair::operator!=(const BroadPhasePair& pair) const {
    return shape1BroadPhaseId != pair.shape1BroadPhaseId || shape2BroadPhaseId != pair.shape2BroadPhaseId;
}

}

// Hash function for a reactphysics3d BroadPhasePair
namespace std {

  template <> struct hash<reactphysics3d::BroadPhasePair> {

    size_t operator()(const reactphysics3d::BroadPhasePair& pair) const {

        assert(pair.shape1BroadPhaseId <= pair.shape2BroadPhaseId);

        std::size_t seed = 0;
        reactphysics3d::hash_combine<int>(seed, pair.shape1BroadPhaseId);
        reactphysics3d::hash_combine<int>(seed, pair.shape2BroadPhaseId);

        return seed;
    }
  };
}

#endif
