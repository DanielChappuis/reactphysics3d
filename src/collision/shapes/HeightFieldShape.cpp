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
#include "HeightFieldShape.h"

using namespace reactphysics3d;

// Constructor
// TODO : Add documentation to this constructor
HeightFieldShape::HeightFieldShape(int width, int length, int minHeight, int maxHeight,
                                   const void* heightFieldData, HeightDataType dataType, int upAxis)
                 : ConcaveShape(CONCAVE_MESH), mWidth(width), mLength(length), mMinHeight(minHeight),
                   mMaxHeight(maxHeight), mUpAxis(upAxis), mHeightDataType(dataType) {

    assert(width >= 1);
    assert(length >= 1);
    assert(minHeight <= maxHeight);
    assert(upAxis == 0 || upAxis == 1 || upAxis == 2);

    mHeightFieldData = heightFieldData;
}

// Destructor
HeightFieldShape::~HeightFieldShape() {

}

// Use a callback method on all triangles of the concave shape inside a given AABB
void HeightFieldShape::testAllTriangles(TriangleCallback& callback, const AABB& localAABB) const {

   // TODO : Implement this
}

// Raycast method with feedback information
/// Note that only the first triangle hit by the ray in the mesh will be returned, even if
/// the ray hits many triangles.
bool HeightFieldShape::raycast(const Ray& ray, RaycastInfo& raycastInfo, ProxyShape* proxyShape) const {

    PROFILE("HeightFieldShape::raycast()");

    // TODO : Implement this
}
