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
#include "collision/shapes/ConcaveShape.h"
#include "collision/shapes/TriangleShape.h"
#include "ConcaveVsConvexAlgorithm.h"
#include "collision/CollisionDetection.h"
#include "engine/CollisionWorld.h"

using namespace reactphysics3d;

// Constructor
ConcaveVsConvexAlgorithm::ConcaveVsConvexAlgorithm() {

}

// Destructor
ConcaveVsConvexAlgorithm::~ConcaveVsConvexAlgorithm() {

}

// Return true and compute a contact info if the two bounding volumes collide
bool ConcaveVsConvexAlgorithm::testCollision(const CollisionShapeInfo& shape1Info,
                                             const CollisionShapeInfo& shape2Info,
                                             NarrowPhaseCallback* narrowPhaseCallback) {

    ProxyShape* convexProxyShape;
    ProxyShape* concaveProxyShape;
    const ConvexShape* convexShape;
    const ConcaveShape* concaveShape;

    // Collision shape 1 is convex, collision shape 2 is concave
    if (shape1Info.collisionShape->isConvex()) {
        convexProxyShape = shape1Info.proxyShape;
        convexShape = static_cast<const ConvexShape*>(shape1Info.collisionShape);
        concaveProxyShape = shape2Info.proxyShape;
        concaveShape = static_cast<const ConcaveShape*>(shape2Info.collisionShape);
    }
    else {  // Collision shape 2 is convex, collision shape 1 is concave
        convexProxyShape = shape2Info.proxyShape;
        convexShape = static_cast<const ConvexShape*>(shape2Info.collisionShape);
        concaveProxyShape = shape1Info.proxyShape;
        concaveShape = static_cast<const ConcaveShape*>(shape1Info.collisionShape);
    }

    // Set the parameters of the callback object
    mConvexVsTriangleCallback.setNarrowPhaseCallback(narrowPhaseCallback);
    mConvexVsTriangleCallback.setCollisionDetection(mCollisionDetection);
    mConvexVsTriangleCallback.setConvexShape(convexShape);
    mConvexVsTriangleCallback.setProxyShapes(convexProxyShape, concaveProxyShape);
    mConvexVsTriangleCallback.setOverlappingPair(shape1Info.overlappingPair);

    // Compute the convex shape AABB in the local-space of the convex shape
    AABB aabb;
    convexShape->computeAABB(aabb, convexProxyShape->getLocalToWorldTransform());

    // Call the convex vs triangle callback for each triangle of the concave shape
    concaveShape->testAllTriangles(mConvexVsTriangleCallback, aabb);

    // TODO : Handle return value here
}

// Test collision between a triangle and the convex mesh shape
void ConvexVsTriangleCallback::testTriangle(const Vector3* trianglePoints) {

    // Create a triangle collision shape
    // TODO : Do we need to use a collision margin for a triangle ?
    TriangleShape triangleShape(trianglePoints[0], trianglePoints[1], trianglePoints[2], 0.0);

    // Select the collision algorithm to use between the triangle and the convex shape
    NarrowPhaseAlgorithm* algo = mCollisionDetection->getCollisionAlgorithm(triangleShape.getType(),
                                                                            mConvexShape->getType());

    // If there is no collision algorithm between those two kinds of shapes
    if (algo == NULL) return;

    // Notify the narrow-phase algorithm about the overlapping pair we are going to test
    algo->setCurrentOverlappingPair(mOverlappingPair);

    // Create the CollisionShapeInfo objects
    CollisionShapeInfo shapeConvexInfo(mConvexProxyShape, mConvexShape, mConvexProxyShape->getLocalToWorldTransform(),
                                       mOverlappingPair, mConvexProxyShape->getCachedCollisionData());
    CollisionShapeInfo shapeConcaveInfo(mConcaveProxyShape, &triangleShape,
                                        mConcaveProxyShape->getLocalToWorldTransform(),
                                        mOverlappingPair, mConcaveProxyShape->getCachedCollisionData());

    // Use the collision algorithm to test collision between the triangle and the other convex shape
    algo->testCollision(shapeConvexInfo, shapeConcaveInfo, mNarrowPhaseCallback);
}
