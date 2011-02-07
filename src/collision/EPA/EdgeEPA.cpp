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
uint EdgeEPA::getSource() const {
    return (*ownerTriangle)[index];
}

// Return the index of the target vertex of the edge (vertex ending the edge)
uint EdgeEPA::getTarget() const {
    return (*ownerTriangle)[IndexOfNextCounterClockwiseEdge(index)];
}

// Link the edge with another one (meaning that the current edge of a triangle will
// is associated with the edge of another triangle in order that both triangles
// are neighbour along both edges)
bool EdgeEPA::link(EdgeEPA& edge) {
    bool isPossible = (this->getSource() == edge.getTarget() &&
                       this->getTarget() == edge.getSource());

    // If the link is possible
    if (isPossible) {
        this->getOwnerTriangle()->setAdjacentEdge(index, edge);
        edge.getOwnerTriangle()->setAdjacentEdge(edge.getIndex(), *this);
    }

    // Return if the link has been made
    return isPossible;
}

// Half link the edge with another one
void EdgeEPA::halfLink(EdgeEPA& edge) const {

}


// Compute the silhouette
bool EdgeEPA::computeSilhouette(const Vector3D* vertices, uint index, TrianglesStore trianglesStore) {
    // TODO : Implement this
}
