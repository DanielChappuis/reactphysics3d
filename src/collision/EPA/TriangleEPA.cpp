/********************************************************************************
* ReactPhysics3D physics library, http://code.google.com/p/reactphysics3d/      *
* Copyright (c) 2011 Daniel Chappuis                                            *
*********************************************************************************
*                                                                               *
* Permission is hereby granted, free of charge, to any person obtaining a copy  *
* of this software and associated documentation files (the "Software"), to deal *
* in the Software without restriction, including without limitation the rights  *
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell     *
* copies of the Software, and to permit persons to whom the Software is         *
* furnished to do so, subject to the following conditions:                      *
*                                                                               *
* The above copyright notice and this permission notice shall be included in    *
* all copies or substantial portions of the Software.                           *
*                                                                               *
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR    *
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,      *
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE   *
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER        *
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, *
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN     *
* THE SOFTWARE.                                                                 *
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
            : isObsolete(false) {
    indicesVertices[0] = indexVertex1;
    indicesVertices[1] = indexVertex2;
    indicesVertices[2] = indexVertex3;
}

// Destructor
TriangleEPA::~TriangleEPA() {

}

// Compute the point v closest to the origin of this triangle
bool TriangleEPA::computeClosestPoint(const Vector3D* vertices) {
    const Vector3D& p0 = vertices[indicesVertices[0]];

    Vector3D v1 = vertices[indicesVertices[1]] - p0;
    Vector3D v2 = vertices[indicesVertices[2]] - p0;
    double v1Dotv1 = v1.dot(v1);
    double v1Dotv2 = v1.dot(v2);
    double v2Dotv2 = v2.dot(v2);
    double p0Dotv1 = p0.dot(v1);
    double p0Dotv2 = p0.dot(v2);

    // Compute determinant
    det = v1Dotv1 * v2Dotv2 - v1Dotv2 * v1Dotv2;

    // Compute lambda values
    lambda1 = p0Dotv2 * v1Dotv2 - p0Dotv1 * v2Dotv2;
    lambda2 = p0Dotv1 * v1Dotv2 - p0Dotv2 * v1Dotv1;

    // If the determinant is positive
    if (det > 0.0) {
        // Compute the closest point v
        closestPoint = p0 + 1.0 / det * (lambda1 * v1 + lambda2 * v2);

        // Compute the square distance of closest point to the origin
        distSquare = closestPoint.dot(closestPoint);

        return true;
    }

    return false;
}

// Compute recursive silhouette algorithm for that triangle
bool TriangleEPA::computeSilhouette(const Vector3D* vertices, uint index, TrianglesStore& triangleStore) {
    
    uint first = triangleStore.getNbTriangles();

    // Mark the current triangle as obsolete
    setIsObsolete(true);

    // Execute recursively the silhouette algorithm for the ajdacent edges of neighbouring
    // triangles of the current triangle
    bool result = adjacentEdges[0].computeSilhouette(vertices, index, triangleStore) &&
                  adjacentEdges[1].computeSilhouette(vertices, index, triangleStore) &&
                  adjacentEdges[2].computeSilhouette(vertices, index, triangleStore);

}
