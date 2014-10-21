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

#ifndef REACTPHYSICS3D_RAYCAST_INFO_H
#define REACTPHYSICS3D_RAYCAST_INFO_H

// Libraries
#include "mathematics/Vector3.h"

/// ReactPhysics3D namespace
namespace reactphysics3d {

// Declarations
class CollisionBody;
class ProxyShape;
class CollisionShape;

// Structure RaycastInfo
/**
 * This structure contains the information about a raycast hit.
 */
struct RaycastInfo {

    private:

        // -------------------- Methods -------------------- //

        /// Private copy constructor
        RaycastInfo(const RaycastInfo& raycastInfo);

        /// Private assignment operator
        RaycastInfo& operator=(const RaycastInfo& raycastInfo);

    public:

        // -------------------- Attributes -------------------- //

        /// Hit point in world-space coordinates
        Vector3 worldPoint;

        /// Surface normal at hit point in world-space coordinates
        Vector3 worldNormal;

        /// Fraction distance of the hit point between point1 and point2 of the ray
        /// The hit point "p" is such that p = point1 + hitFraction * (point2 - point1)
        decimal hitFraction;

        /// Pointer to the hit collision body
        CollisionBody* body;

        /// Pointer to the hit proxy collision shape
        ProxyShape* proxyShape;

        // -------------------- Methods -------------------- //

        /// Constructor
        RaycastInfo() : body(NULL), proxyShape(NULL) {

        }

        /// Destructor
        ~RaycastInfo() {

        }
};

}

#endif
