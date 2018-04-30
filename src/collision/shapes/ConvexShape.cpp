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

// Libraries
#include "ConvexShape.h"
#include "mathematics/Vector3.h"

// We want to use the ReactPhysics3D namespace
using namespace reactphysics3d;

// Constructor
ConvexShape::ConvexShape(CollisionShapeName name, CollisionShapeType type, decimal margin)
            : CollisionShape(name, type), mMargin(margin) {

}

// Return a local support point in a given direction with the object margin
Vector3 ConvexShape::getLocalSupportPointWithMargin(const Vector3& direction) const {

    // Get the support point without margin
    Vector3 supportPoint = getLocalSupportPointWithoutMargin(direction);

    if (mMargin != decimal(0.0)) {

        // Add the margin to the support point
        Vector3 unitVec(0.0, -1.0, 0.0);
        if (direction.lengthSquare() > MACHINE_EPSILON * MACHINE_EPSILON) {
            unitVec = direction.getUnit();
        }
        supportPoint += unitVec * mMargin;
    }

    return supportPoint;
}
