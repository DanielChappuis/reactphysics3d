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
#include "EdgeEPA.h"
#include "TriangleEPA.h"
#include "TrianglesStore.h"
#include <cassert>

// We want to use the ReactPhysics3D namespace
using namespace reactphysics3d;


// Constructor
EdgeEPA::EdgeEPA() {
    
}

// Constructor
EdgeEPA::EdgeEPA(TriangleEPA* ownerTriangle, int index)
        : ownerTriangle(ownerTriangle), index(index) {
    assert(index >= 0 && index < 3);
}

// Destructor
EdgeEPA::~EdgeEPA() {

}

// Return the index of the source vertex of the edge (vertex starting the edge)
uint EdgeEPA::getSourceVertexIndex() const {
    return (*ownerTriangle)[index];
}

// Return the index of the target vertex of the edge (vertex ending the edge)
uint EdgeEPA::getTargetVertexIndex() const {
    return (*ownerTriangle)[indexOfNextCounterClockwiseEdge(index)];
}

// Execute the recursive silhouette algorithm from this edge
bool EdgeEPA::computeSilhouette(const Vector3* vertices, uint indexNewVertex, TrianglesStore& triangleStore) {
    // If the edge has not already been visited
    if (!ownerTriangle->getIsObsolete()) {
        // If the triangle of this edge is not visible from the given point
        if (!ownerTriangle->isVisibleFromVertex(vertices, indexNewVertex)) {
            TriangleEPA* triangle = triangleStore.newTriangle(vertices, indexNewVertex, getTargetVertexIndex(), getSourceVertexIndex());

            // If the triangle has been created
            if (triangle) {
                halfLink(EdgeEPA(triangle, 1), *this);
                return true;
            }

            return false;
        }
        else {
            // The current triangle is visible and therefore obsolete
            ownerTriangle->setIsObsolete(true);

            int backup = triangleStore.getNbTriangles();

            if(!ownerTriangle->getAdjacentEdge(indexOfNextCounterClockwiseEdge(this->index)).computeSilhouette(vertices, indexNewVertex, triangleStore)) {
                ownerTriangle->setIsObsolete(false);

                TriangleEPA* triangle = triangleStore.newTriangle(vertices, indexNewVertex, getTargetVertexIndex(), getSourceVertexIndex());

                // If the triangle has been created
                if (triangle) {
                    halfLink(EdgeEPA(triangle, 1), *this);
                    return true;
                }

                return false;
            }
            else if (!ownerTriangle->getAdjacentEdge(indexOfPreviousCounterClockwiseEdge(this->index)).computeSilhouette(vertices, indexNewVertex, triangleStore)) {
                ownerTriangle->setIsObsolete(false);

                triangleStore.setNbTriangles(backup);

                TriangleEPA* triangle = triangleStore.newTriangle(vertices, indexNewVertex, getTargetVertexIndex(), getSourceVertexIndex());

                if (triangle) {
                    halfLink(EdgeEPA(triangle, 1), *this);
                    return true;
                }

                return false;
            }
        }
    }

    return true;
}
