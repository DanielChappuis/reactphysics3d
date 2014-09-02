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

// Libraries
#include "BoxShape.h"
#include "configuration.h"
#include <vector>
#include <cassert>

using namespace reactphysics3d;

// Constructor
BoxShape::BoxShape(const Vector3& extent, decimal margin)
         : CollisionShape(BOX, margin), mExtent(extent - Vector3(margin, margin, margin)) {
    assert(extent.x > decimal(0.0) && extent.x > margin);
    assert(extent.y > decimal(0.0) && extent.y > margin);
    assert(extent.z > decimal(0.0) && extent.z > margin);
}

// Private copy-constructor
BoxShape::BoxShape(const BoxShape& shape) : CollisionShape(shape), mExtent(shape.mExtent) {

}

// Destructor
BoxShape::~BoxShape() {

}

// Return the local inertia tensor of the collision shape
void BoxShape::computeLocalInertiaTensor(Matrix3x3& tensor, decimal mass) const {
    decimal factor = (decimal(1.0) / decimal(3.0)) * mass;
    Vector3 realExtent = mExtent + Vector3(mMargin, mMargin, mMargin);
    decimal xSquare = realExtent.x * realExtent.x;
    decimal ySquare = realExtent.y * realExtent.y;
    decimal zSquare = realExtent.z * realExtent.z;
    tensor.setAllValues(factor * (ySquare + zSquare), 0.0, 0.0,
                        0.0, factor * (xSquare + zSquare), 0.0,
                        0.0, 0.0, factor * (xSquare + ySquare));
}

// Raycast method
bool BoxShape::raycast(const Ray& ray, ProxyShape* proxyShape, decimal distance) const {

    // TODO : Normalize the ray direction

    // TODO : Implement this method
    return false;
}

// Raycast method with feedback information
bool BoxShape::raycast(const Ray& ray, RaycastInfo& raycastInfo, ProxyShape* proxyShape,
                       decimal distance) const {

    // TODO : Normalize the ray direction

    // TODO : Implement this method
    return false;
}
