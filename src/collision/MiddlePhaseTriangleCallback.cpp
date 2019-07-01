/********************************************************************************
* ReactPhysics3D physics library, http://www.reactphysics3d.com                 *
* Copyright (c) 2010-2019 Daniel Chappuis                                       *
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
#include "collision/MiddlePhaseTriangleCallback.h"
#include "engine/OverlappingPair.h"
#include "collision/NarrowPhaseInfo.h"
#include "collision/shapes/TriangleShape.h"

using namespace reactphysics3d;

// Report collision between a triangle of a concave shape and the convex mesh shape (for middle-phase)
void MiddlePhaseTriangleCallback::testTriangle(const Vector3* trianglePoints, const Vector3* verticesNormals, uint shapeId) {

    // Create a triangle collision shape (the allocated memory for the TriangleShape will be released in the
	// destructor of the corresponding NarrowPhaseInfo.
    TriangleShape* triangleShape = new (mAllocator.allocate(sizeof(TriangleShape)))
                                   TriangleShape(trianglePoints, verticesNormals, shapeId, mAllocator);

#ifdef IS_PROFILING_ACTIVE

	// Set the profiler to the triangle shape
	triangleShape->setProfiler(mProfiler);

#endif

    bool isShape1Convex = mOverlappingPair->getShape1()->getCollisionShape()->isConvex();
    ProxyShape* shape1 = isShape1Convex ? mConvexProxyShape : mConcaveProxyShape;
    ProxyShape* shape2 = isShape1Convex ? mConcaveProxyShape : mConvexProxyShape;

    // Create a narrow phase info for the narrow-phase collision detection
    NarrowPhaseInfo* firstNarrowPhaseInfo = narrowPhaseInfoList;
    narrowPhaseInfoList = new (mAllocator.allocate(sizeof(NarrowPhaseInfo)))
                           NarrowPhaseInfo(mOverlappingPair,
                           isShape1Convex ? mConvexProxyShape->getCollisionShape() : triangleShape,
                           isShape1Convex ? triangleShape : mConvexProxyShape->getCollisionShape(),
                           shape1->getLocalToWorldTransform(),
                           shape2->getLocalToWorldTransform(),
                           mAllocator);
    narrowPhaseInfoList->next = firstNarrowPhaseInfo;
}
