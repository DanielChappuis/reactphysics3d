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
#include <reactphysics3d/collision/ContactManifold.h>
#include <reactphysics3d/constraint/ContactPoint.h>
#include <reactphysics3d/collision/ContactManifoldInfo.h>

using namespace reactphysics3d;

// Constructor
ContactManifold::ContactManifold(Entity bodyEntity1, Entity bodyEntity2, Entity colliderEntity1, Entity colliderEntity2,
                                 uint32 contactPointsIndex, uint8 nbContactPoints)
                :contactPointsIndex(contactPointsIndex), bodyEntity1(bodyEntity1), bodyEntity2(bodyEntity2),
                 colliderEntity1(colliderEntity1), colliderEntity2(colliderEntity2), nbContactPoints(nbContactPoints), frictionImpulse1(0), frictionImpulse2(0),
                 frictionTwistImpulse(0), isAlreadyInIsland(false) {

}
