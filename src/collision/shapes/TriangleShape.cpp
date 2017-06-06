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
#include "TriangleShape.h"
#include "collision/ProxyShape.h"
#include "engine/Profiler.h"
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
TriangleShape::TriangleShape(const Vector3& point1, const Vector3& point2, const Vector3& point3, decimal margin)
              : ConvexPolyhedronShape(margin) {

    mPoints[0] = point1;
    mPoints[1] = point2;
    mPoints[2] = point3;

    // Compute the triangle normal
    mNormal = (point3 - point1).cross(point2 - point1);
    mNormal.normalize();

    mRaycastTestType = TriangleRaycastSide::FRONT;
}

// Raycast method with feedback information
/// This method use the line vs triangle raycasting technique described in
/// Real-time Collision Detection by Christer Ericson.
bool TriangleShape::raycast(const Ray& ray, RaycastInfo& raycastInfo, ProxyShape* proxyShape) const {

    PROFILE("TriangleShape::raycast()");

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

// Return a given half-edge of the polyhedron
HalfEdgeStructure::Edge TriangleShape::getHalfEdge(uint edgeIndex) const {
    assert(edgeIndex < getNbHalfEdges());

    HalfEdgeStructure::Edge edge;

    switch(edgeIndex) {
        case 0:
            edge.vertexIndex = 0;
            edge.twinEdgeIndex = 1;
            edge.faceIndex = 0;
            edge.nextEdgeIndex = 2;
            break;
        case 1:
            edge.vertexIndex = 1;
            edge.twinEdgeIndex = 0;
            edge.faceIndex = 1;
            edge.nextEdgeIndex = 5;
            break;
        case 2:
            edge.vertexIndex = 1;
            edge.twinEdgeIndex = 3;
            edge.faceIndex = 0;
            edge.nextEdgeIndex = 4;
            break;
        case 3:
            edge.vertexIndex = 2;
            edge.twinEdgeIndex = 2;
            edge.faceIndex = 1;
            edge.nextEdgeIndex = 1;
            break;
        case 4:
            edge.vertexIndex = 2;
            edge.twinEdgeIndex = 5;
            edge.faceIndex = 0;
            edge.nextEdgeIndex = 0;
            break;
        case 5:
            edge.vertexIndex = 0;
            edge.twinEdgeIndex = 4;
            edge.faceIndex = 1;
            edge.nextEdgeIndex = 3;
            break;
    }

    return edge;

}
