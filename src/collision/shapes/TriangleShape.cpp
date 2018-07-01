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
#include "TriangleShape.h"
#include "collision/ProxyShape.h"
#include "mathematics/mathematics_functions.h"
#include "collision/RaycastInfo.h"
#include "utils/Profiler.h"
#include "configuration.h"
#include <cassert>

using namespace reactphysics3d;


// Constructor
/**
 * Do not use this constructor. It is supposed to be used internally only.
 * Use a ConcaveMeshShape instead.
 * @param point1 First point of the triangle
 * @param point2 Second point of the triangle
 * @param point3 Third point of the triangle
 * @param verticesNormals The three vertices normals for smooth mesh collision
 * @param margin The collision margin (in meters) around the collision shape
 */
TriangleShape::TriangleShape(const Vector3* vertices, const Vector3* verticesNormals, uint shapeId,
                             MemoryAllocator& allocator)
    : ConvexPolyhedronShape(CollisionShapeName::TRIANGLE), mFaces{HalfEdgeStructure::Face(allocator), HalfEdgeStructure::Face(allocator)} {

    mPoints[0] = vertices[0];
    mPoints[1] = vertices[1];
    mPoints[2] = vertices[2];

    // Compute the triangle normal
    mNormal = (vertices[1] - vertices[0]).cross(vertices[2] - vertices[0]);
    mNormal.normalize();

    mVerticesNormals[0] = verticesNormals[0];
    mVerticesNormals[1] = verticesNormals[1];
    mVerticesNormals[2] = verticesNormals[2];

    // Faces
    mFaces[0].faceVertices.reserve(3);
    mFaces[0].faceVertices.add(0);
    mFaces[0].faceVertices.add(1);
    mFaces[0].faceVertices.add(2);
    mFaces[0].edgeIndex = 0;

    mFaces[1].faceVertices.reserve(3);
    mFaces[1].faceVertices.add(0);
    mFaces[1].faceVertices.add(2);
    mFaces[1].faceVertices.add(1);
    mFaces[1].edgeIndex = 1;

    // Edges
    for (uint i=0; i<6; i++) {
        switch(i) {
            case 0:
                mEdges[0].vertexIndex = 0;
                mEdges[0].twinEdgeIndex = 1;
                mEdges[0].faceIndex = 0;
                mEdges[0].nextEdgeIndex = 2;
                break;
            case 1:
                mEdges[1].vertexIndex = 1;
                mEdges[1].twinEdgeIndex = 0;
                mEdges[1].faceIndex = 1;
                mEdges[1].nextEdgeIndex = 5;
                break;
            case 2:
                mEdges[2].vertexIndex = 1;
                mEdges[2].twinEdgeIndex = 3;
                mEdges[2].faceIndex = 0;
                mEdges[2].nextEdgeIndex = 4;
                break;
            case 3:
                mEdges[3].vertexIndex = 2;
                mEdges[3].twinEdgeIndex = 2;
                mEdges[3].faceIndex = 1;
                mEdges[3].nextEdgeIndex = 1;
                break;
            case 4:
                mEdges[4].vertexIndex = 2;
                mEdges[4].twinEdgeIndex = 5;
                mEdges[4].faceIndex = 0;
                mEdges[4].nextEdgeIndex = 0;
                break;
            case 5:
                mEdges[5].vertexIndex = 0;
                mEdges[5].twinEdgeIndex = 4;
                mEdges[5].faceIndex = 1;
                mEdges[5].nextEdgeIndex = 3;
                break;
        }
    }


    mRaycastTestType = TriangleRaycastSide::FRONT;

    mId = shapeId;
}

// This method compute the smooth mesh contact with a triangle in case one of the two collision
// shapes is a triangle. The idea in this case is to use a smooth vertex normal of the triangle mesh
// at the contact point instead of the triangle normal to avoid the internal edge collision issue.
// This method will return the new smooth world contact
// normal of the triangle and the the local contact point on the other shape.
void TriangleShape::computeSmoothTriangleMeshContact(const CollisionShape* shape1, const CollisionShape* shape2,
                                                     Vector3& localContactPointShape1, Vector3& localContactPointShape2,
                                                     const Transform& shape1ToWorld, const Transform& shape2ToWorld,
                                                     decimal penetrationDepth, Vector3& outSmoothVertexNormal) {

    assert(shape1->getName() != CollisionShapeName::TRIANGLE || shape2->getName() != CollisionShapeName::TRIANGLE);

    // If one the shape is a triangle
    bool isShape1Triangle = shape1->getName() == CollisionShapeName::TRIANGLE;
    if (isShape1Triangle || shape2->getName() == CollisionShapeName::TRIANGLE) {

        const TriangleShape* triangleShape = isShape1Triangle ? static_cast<const TriangleShape*>(shape1):
                                                                static_cast<const TriangleShape*>(shape2);

        // Compute the smooth triangle mesh contact normal and recompute the local contact point on the other shape
        triangleShape->computeSmoothMeshContact(isShape1Triangle ? localContactPointShape1 : localContactPointShape2,
                                                isShape1Triangle ? shape1ToWorld : shape2ToWorld,
                                                isShape1Triangle ? shape2ToWorld.getInverse() : shape1ToWorld.getInverse(),
                                                penetrationDepth, isShape1Triangle,
                                                isShape1Triangle ? localContactPointShape2 : localContactPointShape1,
                                                outSmoothVertexNormal);
    }
}


// This method implements the technique described in Game Physics Pearl book
// by Gino van der Bergen and Dirk Gregorius to get smooth triangle mesh collision. The idea is
// to replace the contact normal of the triangle shape with the precomputed normal of the triangle
// mesh at this point. Then, we need to recompute the contact point on the other shape in order to
// stay aligned with the new contact normal. This method will return the new smooth world contact
// normal of the triangle and the the local contact point on the other shape.
void TriangleShape::computeSmoothMeshContact(Vector3 localContactPointTriangle, const Transform& triangleShapeToWorldTransform,
                                             const Transform& worldToOtherShapeTransform, decimal penetrationDepth, bool isTriangleShape1,
                                             Vector3& outNewLocalContactPointOtherShape, Vector3& outSmoothWorldContactTriangleNormal) const {

    // Get the smooth contact normal of the mesh at the contact point on the triangle
    Vector3 triangleLocalNormal = computeSmoothLocalContactNormalForTriangle(localContactPointTriangle);

    // Convert the local contact normal into world-space
    Vector3 triangleWorldNormal = triangleShapeToWorldTransform.getOrientation() * triangleLocalNormal;

    // Penetration axis with direction from triangle to other shape
    Vector3 triangleToOtherShapePenAxis = isTriangleShape1 ? outSmoothWorldContactTriangleNormal : -outSmoothWorldContactTriangleNormal;

    // The triangle normal should be the one in the direction out of the current colliding face of the triangle
    if (triangleWorldNormal.dot(triangleToOtherShapePenAxis) < decimal(0.0)) {
        triangleWorldNormal = -triangleWorldNormal;
        triangleLocalNormal = -triangleLocalNormal;
    }

    // Compute the final contact normal from shape 1 to shape 2
    outSmoothWorldContactTriangleNormal = isTriangleShape1 ? triangleWorldNormal : -triangleWorldNormal;

    // Re-align the local contact point on the other shape such that it is aligned along the new contact normal
    Vector3 otherShapePointTriangleSpace = localContactPointTriangle - triangleLocalNormal * penetrationDepth;
    Vector3 otherShapePoint = worldToOtherShapeTransform * triangleShapeToWorldTransform * otherShapePointTriangleSpace;
    outNewLocalContactPointOtherShape.setAllValues(otherShapePoint.x, otherShapePoint.y, otherShapePoint.z);
}

// Get a smooth contact normal for collision for a triangle of the mesh
/// This is used to avoid the internal edges issue that occurs when a shape is colliding with
/// several triangles of a concave mesh. If the shape collide with an edge of the triangle for instance,
/// the computed contact normal from this triangle edge is not necessarily in the direction of the surface
/// normal of the mesh at this point. The idea to solve this problem is to use the real (smooth) surface
/// normal of the mesh at this point as the contact normal. This technique is described in the chapter 5
/// of the Game Physics Pearl book by Gino van der Bergen and Dirk Gregorius. The vertices normals of the
/// mesh are either provided by the user or precomputed if the user did not provide them. Note that we only
/// use the interpolated normal if the contact point is on an edge of the triangle. If the contact is in the
/// middle of the triangle, we return the true triangle normal.
Vector3 TriangleShape::computeSmoothLocalContactNormalForTriangle(const Vector3& localContactPoint) const {

    // Compute the barycentric coordinates of the point in the triangle
    decimal u, v, w;
    computeBarycentricCoordinatesInTriangle(mPoints[0], mPoints[1], mPoints[2], localContactPoint, u, v, w);

    // If the contact is in the middle of the triangle face (not on the edges)
    if (u > MACHINE_EPSILON && v > MACHINE_EPSILON && w > MACHINE_EPSILON) {

        // We return the true triangle face normal (not the interpolated one)
        return mNormal;
    }

    // We compute the contact normal as the barycentric interpolation of the three vertices normals
    return (u * mVerticesNormals[0] + v * mVerticesNormals[1] + w * mVerticesNormals[2]).getUnit();
}

// Update the AABB of a body using its collision shape
/**
 * @param[out] aabb The axis-aligned bounding box (AABB) of the collision shape
 *                  computed in world-space coordinates
 * @param transform Transform used to compute the AABB of the collision shape
 */
void TriangleShape::computeAABB(AABB& aabb, const Transform& transform) const {

    const Vector3 worldPoint1 = transform * mPoints[0];
    const Vector3 worldPoint2 = transform * mPoints[1];
    const Vector3 worldPoint3 = transform * mPoints[2];

    const Vector3 xAxis(worldPoint1.x, worldPoint2.x, worldPoint3.x);
    const Vector3 yAxis(worldPoint1.y, worldPoint2.y, worldPoint3.y);
    const Vector3 zAxis(worldPoint1.z, worldPoint2.z, worldPoint3.z);
    aabb.setMin(Vector3(xAxis.getMinValue(), yAxis.getMinValue(), zAxis.getMinValue()));
    aabb.setMax(Vector3(xAxis.getMaxValue(), yAxis.getMaxValue(), zAxis.getMaxValue()));
}

// Raycast method with feedback information
/// This method use the line vs triangle raycasting technique described in
/// Real-time Collision Detection by Christer Ericson.
bool TriangleShape::raycast(const Ray& ray, RaycastInfo& raycastInfo, ProxyShape* proxyShape, MemoryAllocator& allocator) const {

    RP3D_PROFILE("TriangleShape::raycast()", mProfiler);

    const Vector3 pq = ray.point2 - ray.point1;
    const Vector3 pa = mPoints[0] - ray.point1;
    const Vector3 pb = mPoints[1] - ray.point1;
    const Vector3 pc = mPoints[2] - ray.point1;

    // Test if the line PQ is inside the eges BC, CA and AB. We use the triple
    // product for this test.
    const Vector3 m = pq.cross(pc);
    decimal u = pb.dot(m);
    if (mRaycastTestType == TriangleRaycastSide::FRONT) {
        if (u < decimal(0.0)) return false;
    }
    else if (mRaycastTestType == TriangleRaycastSide::BACK) {
        if (u > decimal(0.0)) return false;
    }

    decimal v = -pa.dot(m);
    if (mRaycastTestType == TriangleRaycastSide::FRONT) {
        if (v < decimal(0.0)) return false;
    }
    else if (mRaycastTestType == TriangleRaycastSide::BACK) {
        if (v > decimal(0.0)) return false;
    }
    else if (mRaycastTestType == TriangleRaycastSide::FRONT_AND_BACK) {
        if (!sameSign(u, v)) return false;
    }

    decimal w = pa.dot(pq.cross(pb));
    if (mRaycastTestType == TriangleRaycastSide::FRONT) {
        if (w < decimal(0.0)) return false;
    }
    else if (mRaycastTestType == TriangleRaycastSide::BACK) {
        if (w > decimal(0.0)) return false;
    }
    else if (mRaycastTestType == TriangleRaycastSide::FRONT_AND_BACK) {
        if (!sameSign(u, w)) return false;
    }

    // If the line PQ is in the triangle plane (case where u=v=w=0)
    if (approxEqual(u, 0) && approxEqual(v, 0) && approxEqual(w, 0)) return false;

    // Compute the barycentric coordinates (u, v, w) to determine the
    // intersection point R, R = u * a + v * b + w * c
    decimal denom = decimal(1.0) / (u + v + w);
    u *= denom;
    v *= denom;
    w *= denom;

    // Compute the local hit point using the barycentric coordinates
    const Vector3 localHitPoint = u * mPoints[0] + v * mPoints[1] + w * mPoints[2];
    const decimal hitFraction = (localHitPoint - ray.point1).length() / pq.length();

    if (hitFraction < decimal(0.0) || hitFraction > ray.maxFraction) return false;

    Vector3 localHitNormal = (mPoints[1] - mPoints[0]).cross(mPoints[2] - mPoints[0]);
    if (localHitNormal.dot(pq) > decimal(0.0)) localHitNormal = -localHitNormal;

    raycastInfo.body = proxyShape->getBody();
    raycastInfo.proxyShape = proxyShape;
    raycastInfo.worldPoint = localHitPoint;
    raycastInfo.hitFraction = hitFraction;
    raycastInfo.worldNormal = localHitNormal;

    return true;
}

