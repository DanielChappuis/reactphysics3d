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

#ifndef EDGE_EPA_H
#define EDGE_EPA_H


// Libraries
#include "../../../mathematics/mathematics.h"

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

