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
#include "TriangleEPA.h"
#include "EdgeEPA.h"
#include "TrianglesStore.h"

// We use the ReactPhysics3D namespace
using namespace reactphysics3d;

// Constructor
TriangleEPA::TriangleEPA() {
    
}

// Constructor
TriangleEPA::TriangleEPA(uint indexVertex1, uint indexVertex2, uint indexVertex3)
            : mIsObsolete(false) {
    mIndicesVertices[0] = indexVertex1;
    mIndicesVertices[1] = indexVertex2;
    mIndicesVertices[2] = indexVertex3;
}

// Destructor
TriangleEPA::~TriangleEPA() {

}

// Compute the point v closest to the origin of this triangle
bool TriangleEPA::computeClosestPoint(const Vector3* vertices) {
    const Vector3& p0 = vertices[mIndicesVertices[0]];

    Vector3 v1 = vertices[mIndicesVertices[1]] - p0;
    Vector3 v2 = vertices[mIndicesVertices[2]] - p0;
    decimal v1Dotv1 = v1.dot(v1);
    decimal v1Dotv2 = v1.dot(v2);
    decimal v2Dotv2 = v2.dot(v2);
    decimal p0Dotv1 = p0.dot(v1);
    decimal p0Dotv2 = p0.dot(v2);

    // Compute determinant
    mDet = v1Dotv1 * v2Dotv2 - v1Dotv2 * v1Dotv2;

    // Compute lambda values
    mLambda1 = p0Dotv2 * v1Dotv2 - p0Dotv1 * v2Dotv2;
    mLambda2 = p0Dotv1 * v1Dotv2 - p0Dotv2 * v1Dotv1;

    // If the determinant is positive
    if (mDet > 0.0) {
        // Compute the closest point v
        mClosestPoint = p0 + decimal(1.0) / mDet * (mLambda1 * v1 + mLambda2 * v2);

        // Compute the square distance of closest point to the origin
        mDistSquare = mClosestPoint.dot(mClosestPoint);

        return true;
    }

    return false;
}

/// Link an edge with another one. It means that the current edge of a triangle will
/// be associated with the edge of another triangle in order that both triangles
/// are neighbour along both edges).
bool reactphysics3d::link(const EdgeEPA& edge0, const EdgeEPA& edge1) {
    bool isPossible = (edge0.getSourceVertexIndex() == edge1.getTargetVertexIndex() &&
                       edge0.getTargetVertexIndex() == edge1.getSourceVertexIndex());

    if (isPossible) {
        edge0.getOwnerTriangle()->mAdjacentEdges[edge0.getIndex()] = edge1;
        edge1.getOwnerTriangle()->mAdjacentEdges[edge1.getIndex()] = edge0;
    }

    return isPossible;
}

/// Make an half link of an edge with another one from another triangle. An half-link
/// between an edge "edge0" and an edge "edge1" represents the fact that "edge1" is an
/// adjacent edge of "edge0" but not the opposite. The opposite edge connection will
/// be made later.
void reactphysics3d::halfLink(const EdgeEPA& edge0, const EdgeEPA& edge1) {
    assert(edge0.getSourceVertexIndex() == edge1.getTargetVertexIndex() &&
           edge0.getTargetVertexIndex() == edge1.getSourceVertexIndex());

    // Link
    edge0.getOwnerTriangle()->mAdjacentEdges[edge0.getIndex()] = edge1;
}

// Execute the recursive silhouette algorithm from this triangle face.
/// The parameter "vertices" is an array that contains the vertices of the current polytope and the
/// parameter "indexNewVertex" is the index of the new vertex in this array. The goal of the
/// silhouette algorithm is to add the new vertex in the polytope by keeping it convex. Therefore,
/// the triangle faces that are visible from the new vertex must be removed from the polytope and we
/// need to add triangle faces where each face contains the new vertex and an edge of the silhouette.
/// The silhouette is the connected set of edges that are part of the border between faces that
/// are seen and faces that are not seen from the new vertex. This method starts from the nearest
/// face from the new vertex, computes the silhouette and create the new faces from the new vertex in
/// order that we always have a convex polytope. The faces visible from the new vertex are set
/// obselete and will not be considered as being a candidate face in the future.
bool TriangleEPA::computeSilhouette(const Vector3* vertices, uint indexNewVertex,
                                    TrianglesStore& triangleStore) {
    
    uint first = triangleStore.getNbTriangles();

    // Mark the current triangle as obsolete because it
    setIsObsolete(true);

    // Execute recursively the silhouette algorithm for the adjacent edges of neighboring
    // triangles of the current triangle
    bool result = mAdjacentEdges[0].computeSilhouette(vertices, indexNewVertex, triangleStore) &&
                  mAdjacentEdges[1].computeSilhouette(vertices, indexNewVertex, triangleStore) &&
                  mAdjacentEdges[2].computeSilhouette(vertices, indexNewVertex, triangleStore);

    if (result) {
        int i,j;

        // For each triangle face that contains the new vertex and an edge of the silhouette
        for (i=first, j=triangleStore.getNbTriangles()-1;
             i != triangleStore.getNbTriangles(); j = i++) {
            TriangleEPA* triangle = &triangleStore[i];
            halfLink(triangle->getAdjacentEdge(1), EdgeEPA(triangle, 1));

            if (!link(EdgeEPA(triangle, 0), EdgeEPA(&triangleStore[j], 2))) {
                return false;
            }
        }

    }

    return result;
}
