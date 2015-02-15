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
#include "AABB.h"
#include "configuration.h"
#include <cassert>

using namespace reactphysics3d;
using namespace std;

// Constructor
AABB::AABB() {

}

// Constructor
AABB::AABB(const Vector3& minCoordinates, const Vector3& maxCoordinates)
     :mMinCoordinates(minCoordinates), mMaxCoordinates(maxCoordinates) {

}

// Copy-constructor
AABB::AABB(const AABB& aabb)
     : mMinCoordinates(aabb.mMinCoordinates), mMaxCoordinates(aabb.mMaxCoordinates) {

}

// Destructor
AABB::~AABB() {

}

// Merge the AABB in parameter with the current one
void AABB::mergeWithAABB(const AABB& aabb) {
    mMinCoordinates.x = std::min(mMinCoordinates.x, aabb.mMinCoordinates.x);
    mMinCoordinates.y = std::min(mMinCoordinates.y, aabb.mMinCoordinates.y);
    mMinCoordinates.z = std::min(mMinCoordinates.z, aabb.mMinCoordinates.z);

    mMaxCoordinates.x = std::max(mMaxCoordinates.x, aabb.mMaxCoordinates.x);
    mMaxCoordinates.y = std::max(mMaxCoordinates.y, aabb.mMaxCoordinates.y);
    mMaxCoordinates.z = std::max(mMaxCoordinates.z, aabb.mMaxCoordinates.z);
}

// Replace the current AABB with a new AABB that is the union of two AABBs in parameters
void AABB::mergeTwoAABBs(const AABB& aabb1, const AABB& aabb2) {
    mMinCoordinates.x = std::min(aabb1.mMinCoordinates.x, aabb2.mMinCoordinates.x);
    mMinCoordinates.y = std::min(aabb1.mMinCoordinates.y, aabb2.mMinCoordinates.y);
    mMinCoordinates.z = std::min(aabb1.mMinCoordinates.z, aabb2.mMinCoordinates.z);

    mMaxCoordinates.x = std::max(aabb1.mMaxCoordinates.x, aabb2.mMaxCoordinates.x);
    mMaxCoordinates.y = std::max(aabb1.mMaxCoordinates.y, aabb2.mMaxCoordinates.y);
    mMaxCoordinates.z = std::max(aabb1.mMaxCoordinates.z, aabb2.mMaxCoordinates.z);
}

// Return true if the current AABB contains the AABB given in parameter
bool AABB::contains(const AABB& aabb) {

    bool isInside = true;
    isInside = isInside && mMinCoordinates.x <= aabb.mMinCoordinates.x;
    isInside = isInside && mMinCoordinates.y <= aabb.mMinCoordinates.y;
    isInside = isInside && mMinCoordinates.z <= aabb.mMinCoordinates.z;

    isInside = isInside && mMaxCoordinates.x >= aabb.mMaxCoordinates.x;
    isInside = isInside && mMaxCoordinates.y >= aabb.mMaxCoordinates.y;
    isInside = isInside && mMaxCoordinates.z >= aabb.mMaxCoordinates.z;
    return isInside;
}

