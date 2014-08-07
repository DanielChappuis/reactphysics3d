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
#include <complex>
#include "configuration.h"
#include "ConeShape.h"

using namespace reactphysics3d;

// Constructor
ConeShape::ConeShape(decimal radius, decimal height, decimal margin)
          : CollisionShape(CONE, margin), mRadius(radius), mHalfHeight(height * decimal(0.5)) {
    assert(mRadius > decimal(0.0));
    assert(mHalfHeight > decimal(0.0));
    
    // Compute the sine of the semi-angle at the apex point
    mSinTheta = mRadius / (sqrt(mRadius * mRadius + height * height));
}

// Private copy-constructor
ConeShape::ConeShape(const ConeShape& shape)
          : CollisionShape(shape), mRadius(shape.mRadius), mHalfHeight(shape.mHalfHeight),
            mSinTheta(shape.mSinTheta){

}

// Destructor
ConeShape::~ConeShape() {

}

// Return a local support point in a given direction with the object margin
Vector3 ConeShape::getLocalSupportPointWithMargin(const Vector3& direction,
                                                  void** cachedCollisionData) const {

    // Compute the support point without the margin
    Vector3 supportPoint = getLocalSupportPointWithoutMargin(direction, cachedCollisionData);

    // Add the margin to the support point
    Vector3 unitVec(0.0, -1.0, 0.0);
    if (direction.lengthSquare() > MACHINE_EPSILON * MACHINE_EPSILON) {
        unitVec = direction.getUnit();
    }
    supportPoint += unitVec * mMargin;

    return supportPoint;
}

// Return a local support point in a given direction without the object margin
Vector3 ConeShape::getLocalSupportPointWithoutMargin(const Vector3& direction,
                                                     void** cachedCollisionData) const {

    const Vector3& v = direction;
    decimal sinThetaTimesLengthV = mSinTheta * v.length();
    Vector3 supportPoint;

    if (v.y > sinThetaTimesLengthV) {
        supportPoint = Vector3(0.0, mHalfHeight, 0.0);
    }
    else {
        decimal projectedLength = sqrt(v.x * v.x + v.z * v.z);
        if (projectedLength > MACHINE_EPSILON) {
            decimal d = mRadius / projectedLength;
            supportPoint = Vector3(v.x * d, -mHalfHeight, v.z * d);
        }
        else {
            supportPoint = Vector3(0.0, -mHalfHeight, 0.0);
        }
    }

    return supportPoint;
}

// Raycast method
bool ConeShape::raycast(const Ray& ray, decimal distance) const {
    // TODO : Implement this method
    return false;
}

// Raycast method with feedback information
bool ConeShape::raycast(const Ray& ray, RaycastInfo& raycastInfo, decimal distance) const {
    // TODO : Implement this method
    return false;
}

// Return true if a point is inside the collision shape
bool ConeShape::testPointInside(const Vector3& localPoint) const {
    // TODO : Implement this method
    return false;
}
