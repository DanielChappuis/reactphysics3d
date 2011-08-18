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

#ifndef EDGE_EPA_H
#define EDGE_EPA_H


// Libraries
#include "../../mathematics/mathematics.h"

// ReactPhysics3D namespace
namespace reactphysics3d {

// Class declarations
class TriangleEPA;
class TrianglesStore;

/*  -------------------------------------------------------------------
    Class EdgeEPA :
        This class represents an edge of the current polytope in the EPA
        algorithm.
    -------------------------------------------------------------------
*/
class EdgeEPA {
    private:
        TriangleEPA* ownerTriangle;         // Pointer to the triangle that contains this edge
        int index;                          // Index of the edge in the triangle (between 0 and 2). 
                                            // The edge with index i connect triangle vertices i and (i+1 % 3)

    public:
        EdgeEPA();                                                      // Constructor
        EdgeEPA(TriangleEPA* ownerTriangle, int index);                 // Constructor
        ~EdgeEPA();                                                     // Destructor
        TriangleEPA* getOwnerTriangle() const;                          // Return the pointer to the owner triangle
        int getIndex() const;                                           // Return the index of the edge in the triangle
        uint getSourceVertexIndex() const;                              // Return index of the source vertex of the edge
        uint getTargetVertexIndex() const;                              // Return the index of the target vertex of the edge
        bool computeSilhouette(const Vector3* vertices, uint index,
                               TrianglesStore& triangleStore);          // Execute the recursive silhouette algorithm from this edge
};

// Return the pointer to the owner triangle
inline TriangleEPA* EdgeEPA::getOwnerTriangle() const {
    return ownerTriangle;
}

// Return the edge index
inline int EdgeEPA::getIndex() const {
    return index;
}

// Return the index of the next counter-clockwise edge of the ownver triangle
inline int indexOfNextCounterClockwiseEdge(int i) {
    return (i + 1) % 3;
}

// Return the index of the previous counter-clockwise edge of the ownver triangle
inline int indexOfPreviousCounterClockwiseEdge(int i) {
    return (i + 2) % 3;
}

}   // End of ReactPhysics3D namespace

#endif

