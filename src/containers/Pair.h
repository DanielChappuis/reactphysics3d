/********************************************************************************
* ReactPhysics3D physics library, http://www.reactphysics3d.com                 *
* Copyright (c) 2010-2019 Daniel Chappuis                                       *
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

#ifndef REACTPHYSICS3D_PAIR_H
#define REACTPHYSICS3D_PAIR_H

// Libraries
#include "configuration.h"
#include "memory/MemoryAllocator.h"
#include "containers/containers_common.h"
#include <cstring>
#include <iterator>

namespace reactphysics3d {

// Class Pair
/**
 * This class represents a simple generic pair
  */
template<typename T1, typename T2>
class Pair {

    public:

        // -------------------- Attributes -------------------- //

        /// First element of the pair
        T1 first;

        /// Second element of the pair
        T2 second;

        // -------------------- Methods -------------------- //

        /// Constructor
        Pair(const T1& item1, const T2& item2) : first(item1), second(item2) {

        }

        /// Copy constructor
        Pair(const Pair<T1, T2>& pair) : first(pair.first), second(pair.second) {

        }

        /// Destructor
        ~Pair() = default;

        /// Overloaded equality operator
        bool operator==(const Pair<T1, T2>& pair) const {
            return first == pair.first && second == pair.second;
        }

        /// Overloaded not equal operator
        bool operator!=(const Pair<T1, T2>& pair) const {
            return !((*this) == pair);
        }

        /// Overloaded assignment operator
        Pair<T1, T2>& operator=(const Pair<T1, T2>& pair) {
            first = pair.first;
            second = pair.second;
            return *this;
        }
};

}

// Hash function for a reactphysics3d Pair
namespace std {

  template <typename T1, typename T2> struct hash<reactphysics3d::Pair<T1, T2>> {

    size_t operator()(const reactphysics3d::Pair<T1, T2>& pair) const {

        std::size_t seed = 0;
        reactphysics3d::hash_combine<T1>(seed, pair.first);
        reactphysics3d::hash_combine<T2>(seed, pair.second);

        return seed;
    }
  };
}


#endif
