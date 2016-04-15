/********************************************************************************
* ReactPhysics3D physics library, http://www.reactphysics3d.com                 *
* Copyright (c) 2010-2016 Daniel Chappuis                                       *
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
#include <algorithm>

using namespace reactphysics3d;

// Constructor
ConcaveVsConvexAlgorithm::ConcaveVsConvexAlgorithm() {

}

// Destructor
ConcaveVsConvexAlgorithm::~ConcaveVsConvexAlgorithm() {

}

// Return true and compute a contact info if the two bounding volumes collide
void ConcaveVsConvexAlgorithm::testCollision(const CollisionShapeInfo& shape1Info,
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
    ConvexVsTriangleCallback convexVsTriangleCallback;
    convexVsTriangleCallback.setCollisionDetection(mCollisionDetection);
    convexVsTriangleCallback.setConvexShape(convexShape);
    convexVsTriangleCallback.setConcaveShape(concaveShape);
    convexVsTriangleCallback.setProxyShapes(convexProxyShape, concaveProxyShape);
    convexVsTriangleCallback.setOverlappingPair(shape1Info.overlappingPair);

    // Compute the convex shape AABB in the local-space of the convex shape
    AABB aabb;
    convexShape->computeAABB(aabb, convexProxyShape->getLocalToWorldTransform());

    // If smooth mesh collision is enabled for the concave mesh
    if (concaveShape->getIsSmoothMeshCollisionEnabled()) {

        std::vector<SmoothMeshContactInfo> contactPoints;

        SmoothCollisionNarrowPhaseCallback smoothNarrowPhaseCallback(contactPoints);

        convexVsTriangleCallback.setNarrowPhaseCallback(&smoothNarrowPhaseCallback);

        // Call the convex vs triangle callback for each triangle of the concave shape
        concaveShape->testAllTriangles(convexVsTriangleCallback, aabb);

        // Run the smooth mesh collision algorithm
        processSmoothMeshCollision(shape1Info.overlappingPair, contactPoints, narrowPhaseCallback);
    }
    else {

        convexVsTriangleCallback.setNarrowPhaseCallback(narrowPhaseCallback);

        // Call the convex vs triangle callback for each triangle of the concave shape
        concaveShape->testAllTriangles(convexVsTriangleCallback, aabb);
    }
}

// Test collision between a triangle and the convex mesh shape
void ConvexVsTriangleCallback::testTriangle(const Vector3* trianglePoints) {

    // Create a triangle collision shape
    decimal margin = mConcaveShape->getTriangleMargin();
    TriangleShape triangleShape(trianglePoints[0], trianglePoints[1], trianglePoints[2], margin);

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

// Process the concave triangle mesh collision using the smooth mesh collision algorithm described
// by Pierre Terdiman (http://www.codercorner.com/MeshContacts.pdf). This is used to avoid the collision
// issue with some internal edges.
void ConcaveVsConvexAlgorithm::processSmoothMeshCollision(OverlappingPair* overlappingPair,
                                                          std::vector<SmoothMeshContactInfo> contactPoints,
                                                          NarrowPhaseCallback* narrowPhaseCallback) {

    // Set with the triangle vertices already processed to void further contacts with same triangle
    std::unordered_multimap<int, Vector3> processTriangleVertices;

    // Sort the list of narrow-phase contacts according to their penetration depth
    std::sort(contactPoints.begin(), contactPoints.end(), ContactsDepthCompare());

    // For each contact point (from smaller penetration depth to larger)
    std::vector<SmoothMeshContactInfo>::const_iterator it;
    for (it = contactPoints.begin(); it != contactPoints.end(); ++it) {

        const SmoothMeshContactInfo info = *it;
        const Vector3& contactPoint = info.isFirstShapeTriangle ? info.contactInfo.localPoint1 : info.contactInfo.localPoint2;

        // Compute the barycentric coordinates of the point in the triangle
        decimal u, v, w;
        computeBarycentricCoordinatesInTriangle(info.triangleVertices[0],
                                                info.triangleVertices[1],
                                                info.triangleVertices[2],
                                                contactPoint, u, v, w);
        int nbZeros = 0;
        bool isUZero = approxEqual(u, 0, 0.0001);
        bool isVZero = approxEqual(v, 0, 0.0001);
        bool isWZero = approxEqual(w, 0, 0.0001);
        if (isUZero) nbZeros++;
        if (isVZero) nbZeros++;
        if (isWZero) nbZeros++;

        // If it is a vertex contact
        if (nbZeros == 2) {

            Vector3 contactVertex = !isUZero ? info.triangleVertices[0] : (!isVZero ? info.triangleVertices[1] : info.triangleVertices[2]);

            // Check that this triangle vertex has not been processed yet
            if (!hasVertexBeenProcessed(processTriangleVertices, contactVertex)) {

                // Keep the contact as it is and report it
                narrowPhaseCallback->notifyContact(overlappingPair, info.contactInfo);
            }
        }
        else if (nbZeros == 1) {  // If it is an edge contact

            Vector3 contactVertex1 = isUZero ? info.triangleVertices[1] : (isVZero ? info.triangleVertices[0] : info.triangleVertices[0]);
            Vector3 contactVertex2 = isUZero ? info.triangleVertices[2] : (isVZero ? info.triangleVertices[2] : info.triangleVertices[1]);

            // Check that this triangle edge has not been processed yet
            if (!hasVertexBeenProcessed(processTriangleVertices, contactVertex1) &&
                !hasVertexBeenProcessed(processTriangleVertices, contactVertex2)) {

                // Keep the contact as it is and report it
                narrowPhaseCallback->notifyContact(overlappingPair, info.contactInfo);
            }

        }
        else {  // If it is a face contact

            ContactPointInfo newContactInfo(info.contactInfo);

            ProxyShape* firstShape;
            ProxyShape* secondShape;
            if (info.isFirstShapeTriangle) {
                firstShape = overlappingPair->getShape1();
                secondShape = overlappingPair->getShape2();
            }
            else {
                firstShape = overlappingPair->getShape2();
                secondShape = overlappingPair->getShape1();
            }

            // We use the triangle normal as the contact normal
            Vector3 a = info.triangleVertices[1] - info.triangleVertices[0];
            Vector3 b = info.triangleVertices[2] - info.triangleVertices[0];
            Vector3 localNormal = a.cross(b);
            newContactInfo.normal = firstShape->getLocalToWorldTransform().getOrientation() * localNormal;
            Vector3 firstLocalPoint = info.isFirstShapeTriangle ? info.contactInfo.localPoint1 : info.contactInfo.localPoint2;
            Vector3 firstWorldPoint = firstShape->getLocalToWorldTransform() * firstLocalPoint;
            newContactInfo.normal.normalize();
            if (newContactInfo.normal.dot(info.contactInfo.normal) < 0) {
                newContactInfo.normal = -newContactInfo.normal;
            }

            // We recompute the contact point on the second body with the new normal as described in
            // the Smooth Mesh Contacts with GJK of the Game Physics Pearls book (from Gino van Den Bergen and
            // Dirk Gregorius) to avoid adding torque
            Transform worldToLocalSecondPoint = secondShape->getLocalToWorldTransform().getInverse();
            if (info.isFirstShapeTriangle) {
                Vector3 newSecondWorldPoint = firstWorldPoint + newContactInfo.normal;
                newContactInfo.localPoint2 = worldToLocalSecondPoint * newSecondWorldPoint;
            }
            else {
                Vector3 newSecondWorldPoint = firstWorldPoint - newContactInfo.normal;
                newContactInfo.localPoint1 = worldToLocalSecondPoint * newSecondWorldPoint;
            }

            // Report the contact
            narrowPhaseCallback->notifyContact(overlappingPair, newContactInfo);
        }

        // Add the three vertices of the triangle to the set of processed
        // triangle vertices
        addProcessedVertex(processTriangleVertices, info.triangleVertices[0]);
        addProcessedVertex(processTriangleVertices, info.triangleVertices[1]);
        addProcessedVertex(processTriangleVertices, info.triangleVertices[2]);
    }
}

// Return true if the vertex is in the set of already processed vertices
bool ConcaveVsConvexAlgorithm::hasVertexBeenProcessed(const std::unordered_multimap<int, Vector3>& processTriangleVertices, const Vector3& vertex) const {

    int key = int(vertex.x * vertex.y * vertex.z);

    auto range = processTriangleVertices.equal_range(key);
    for (auto it = range.first; it != range.second; ++it) {
        if (vertex.x == it->second.x && vertex.y == it->second.y && vertex.z == it->second.z) return true;
    }

    return false;
}

// Called by a narrow-phase collision algorithm when a new contact has been found
void SmoothCollisionNarrowPhaseCallback::notifyContact(OverlappingPair* overlappingPair,
                                                       const ContactPointInfo& contactInfo) {
    Vector3 triangleVertices[3];
    bool isFirstShapeTriangle;

    // If the collision shape 1 is the triangle
    if (contactInfo.collisionShape1->getType() == TRIANGLE) {
        assert(contactInfo.collisionShape2->getType() != TRIANGLE);

        const TriangleShape* triangleShape = static_cast<const TriangleShape*>(contactInfo.collisionShape1);
        triangleVertices[0] = triangleShape->getVertex(0);
        triangleVertices[1] = triangleShape->getVertex(1);
        triangleVertices[2] = triangleShape->getVertex(2);

        isFirstShapeTriangle = true;
    }
    else {  // If the collision shape 2 is the triangle
        assert(contactInfo.collisionShape2->getType() == TRIANGLE);

        const TriangleShape* triangleShape = static_cast<const TriangleShape*>(contactInfo.collisionShape2);
        triangleVertices[0] = triangleShape->getVertex(0);
        triangleVertices[1] = triangleShape->getVertex(1);
        triangleVertices[2] = triangleShape->getVertex(2);

        isFirstShapeTriangle = false;
    }
    SmoothMeshContactInfo smoothContactInfo(contactInfo, isFirstShapeTriangle, triangleVertices[0], triangleVertices[1], triangleVertices[2]);

    // Add the narrow-phase contact into the list of contact to process for
    // smooth mesh collision
    mContactPoints.push_back(smoothContactInfo);
}
