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
#include "TriangleShape.h"
#include "collision/ProxyShape.h"
#include "configuration.h"
#include <cassert>

using namespace reactphysics3d;

// Constructor
/**
 * @param point1 First point of the triangle
 * @param point2 Second point of the triangle
 * @param point3 Third point of the triangle
 * @param margin The collision margin (in meters) around the collision shape
 */
TriangleShape::TriangleShape(const Vector3& point1, const Vector3& point2, const Vector3& point3)
              : ConvexShape(TRIANGLE, 0) {
    mPoints[0] = point1;
    mPoints[1] = point2;
    mPoints[2] = point3;
}

// Destructor
TriangleShape::~TriangleShape() {

}

// Raycast method with feedback information
bool TriangleShape::raycast(const Ray& ray, RaycastInfo& raycastInfo, ProxyShape* proxyShape) const {

    // TODO : Implement this

    return false;
}

