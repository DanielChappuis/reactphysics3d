/********************************************************************************
* ReactPhysics3D physics library, http://code.google.com/p/reactphysics3d/      *
* Copyright (c) 2010-2012 Daniel Chappuis                                       *
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
