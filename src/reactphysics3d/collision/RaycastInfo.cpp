/********************************************************************************
* ReactPhysics3D physics library, http://www.reactphysics3d.com                 *
* Copyright (c) 2010-2015 Daniel Chappuis                                       *
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
#include "reactphysics3d/decimal.h"
#include "reactphysics3d/collision/RaycastInfo.h"
#include "reactphysics3d/collision/ProxyShape.h"

using namespace reactphysics3d;

// Ray cast test against a proxy shape
decimal RaycastTest::raycastAgainstShape(ProxyShape* shape, const Ray& ray) {

    // Ray casting test against the collision shape
    RaycastInfo raycastInfo;
    bool isHit = shape->raycast(ray, raycastInfo);

    // If the ray hit the collision shape
    if (isHit) {

        // Report the hit to the user and return the
        // user hit fraction value
        return userCallback->notifyRaycastHit(raycastInfo);
    }

    return ray.maxFraction;
}
