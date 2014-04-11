/********************************************************************************
* ReactPhysics3D physics library, http://code.google.com/p/reactphysics3d/      *
* Copyright (c) 2010-2013 Daniel Chappuis                                       *
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
#include "../body/CollisionBody.h"

/// ReactPhysics3D namespace
namespace reactphysics3d {

// TODO : DELETE THIS CLASS
// Structure BroadPhasePair
/**
 * This structure represents a pair of bodies
 * during the broad-phase collision detection.
 */
struct BroadPhasePair {

    public:

        // -------------------- Attributes -------------------- //

        /// Pointer to the first body
        CollisionBody* body1;

        /// Pointer to the second body
        CollisionBody* body2;

        /// Previous cached separating axis
        Vector3 previousSeparatingAxis;

        // -------------------- Methods -------------------- //

        /// Constructor
        BroadPhasePair(CollisionBody* body1, CollisionBody* body2);

        /// Destructor
        ~BroadPhasePair();

        /// Return the pair of bodies index
        static bodyindexpair computeBodiesIndexPair(CollisionBody* body1, CollisionBody* body2);

        /// Return the pair of bodies index
        bodyindexpair getBodiesIndexPair() const;

        /// Smaller than operator
        bool operator<(const BroadPhasePair& broadPhasePair2) const;

        /// Larger than operator
        bool operator>(const BroadPhasePair& broadPhasePair2) const;

        /// Equal operator
        bool operator==(const BroadPhasePair& broadPhasePair2) const;

        /// Not equal operator
        bool operator!=(const BroadPhasePair& broadPhasePair2) const;
};



// Return the pair of bodies index
inline bodyindexpair BroadPhasePair::getBodiesIndexPair() const {

    return computeBodiesIndexPair(body1, body2);
}

// Smaller than operator
inline bool BroadPhasePair::operator<(const BroadPhasePair& broadPhasePair2) const {
    return (body1 < broadPhasePair2.body1 ? true : (body2 < broadPhasePair2.body2));
}

// Larger than operator
inline bool BroadPhasePair::operator>(const BroadPhasePair& broadPhasePair2) const {
    return (body1 > broadPhasePair2.body1 ? true : (body2 > broadPhasePair2.body2));
}

// Equal operator
inline bool BroadPhasePair::operator==(const BroadPhasePair& broadPhasePair2) const {
    return (body1 == broadPhasePair2.body1 && body2 == broadPhasePair2.body2);
}

// Not equal operator
inline bool BroadPhasePair::operator!=(const BroadPhasePair& broadPhasePair2) const {
    return (body1 != broadPhasePair2.body1 || body2 != broadPhasePair2.body2);
}

}

#endif
