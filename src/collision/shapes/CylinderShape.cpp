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
#include "CylinderShape.h"
#include "../../configuration.h"

using namespace reactphysics3d;

// Constructor
CylinderShape::CylinderShape(decimal radius, decimal height, decimal margin)
              : CollisionShape(CYLINDER, margin), mRadius(radius),
                mHalfHeight(height/decimal(2.0)) {
    assert(radius > decimal(0.0));
    assert(height > decimal(0.0));
    assert(margin > decimal(0.0));
}

// Private copy-constructor
CylinderShape::CylinderShape(const CylinderShape& shape)
              : CollisionShape(shape), mRadius(shape.mRadius), mHalfHeight(shape.mHalfHeight) {

}

// Destructor
CylinderShape::~CylinderShape() {

}

// Return a local support point in a given direction with the object margin
Vector3 CylinderShape::getLocalSupportPointWithMargin(const Vector3& direction) const {

    // Compute the support point without the margin
    Vector3 supportPoint = getLocalSupportPointWithoutMargin(direction);

    // Add the margin to the support point
    Vector3 unitVec(0.0, 1.0, 0.0);
    if (direction.lengthSquare() > MACHINE_EPSILON * MACHINE_EPSILON) {
        unitVec = direction.getUnit();
    }
    supportPoint += unitVec * mMargin;

    return supportPoint;
}

// Return a local support point in a given direction without the object margin
Vector3 CylinderShape::getLocalSupportPointWithoutMargin(const Vector3& direction) const {

    Vector3 supportPoint(0.0, 0.0, 0.0);
    decimal uDotv = direction.y;
    Vector3 w(direction.x, 0.0, direction.z);
    decimal lengthW = sqrt(direction.x * direction.x + direction.z * direction.z);

    if (lengthW > MACHINE_EPSILON) {
        if (uDotv < 0.0) supportPoint.y = -mHalfHeight;
        else supportPoint.y = mHalfHeight;
        supportPoint += (mRadius / lengthW) * w;
    }
    else {
         if (uDotv < 0.0) supportPoint.y = -mHalfHeight;
         else supportPoint.y = mHalfHeight;
    }

    return supportPoint;
}

// Constructor
ProxyCylinderShape::ProxyCylinderShape(CylinderShape* cylinderShape, CollisionBody* body,
                                       const Transform& transform, decimal mass)
                   :ProxyShape(body, transform, mass), mCollisionShape(cylinderShape){

}

// Destructor
ProxyCylinderShape::~ProxyCylinderShape() {

}
