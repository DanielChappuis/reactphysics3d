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

#ifndef REACTPHYSICS3D_EDGE_EPA_H
#define REACTPHYSICS3D_EDGE_EPA_H


// Libraries
#include "mathematics/mathematics.h"

/// ReactPhysics3D namespace
namespace reactphysics3d {

// Class declarations
class TriangleEPA;
class TrianglesStore;

// Class EdgeEPA
/**
 * This class represents an edge of the current polytope in the EPA algorithm.
 */
class EdgeEPA {

    private:

        // -------------------- Attributes -------------------- //

        /// Pointer to the triangle that contains this edge
        TriangleEPA* mOwnerTriangle;

        /// Index of the edge in the triangle (between 0 and 2).
        /// The edge with index i connect triangle vertices i and (i+1 % 3)
        int mIndex;

    public:

        // -------------------- Methods -------------------- //

        /// Constructor
        EdgeEPA();

        /// Constructor
        EdgeEPA(TriangleEPA* ownerTriangle, int index);

        /// Copy-constructor
        EdgeEPA(const EdgeEPA& edge);

        /// Destructor
        ~EdgeEPA();

        /// Return the pointer to the owner triangle
        TriangleEPA* getOwnerTriangle() const;

        /// Return the index of the edge in the triangle
        int getIndex() const;

        /// Return index of the source vertex of the edge
        uint getSourceVertexIndex() const;

        /// Return the index of the target vertex of the edge
        uint getTargetVertexIndex() const;

        /// Execute the recursive silhouette algorithm from this edge
        bool computeSilhouette(const Vector3* vertices, uint index, TrianglesStore& triangleStore);

        /// Assignment operator
        EdgeEPA& operator=(const EdgeEPA& edge);
};

// Return the pointer to the owner triangle
inline TriangleEPA* EdgeEPA::getOwnerTriangle() const {
    return mOwnerTriangle;
}

// Return the edge index
inline int EdgeEPA::getIndex() const {
    return mIndex;
}

// Assignment operator
inline EdgeEPA& EdgeEPA::operator=(const EdgeEPA& edge) {
    mOwnerTriangle = edge.mOwnerTriangle;
    mIndex = edge.mIndex;
    return *this;
}

// Return the index of the next counter-clockwise edge of the ownver triangle
inline int indexOfNextCounterClockwiseEdge(int i) {
    return (i + 1) % 3;
}

// Return the index of the previous counter-clockwise edge of the ownver triangle
inline int indexOfPreviousCounterClockwiseEdge(int i) {
    return (i + 2) % 3;
}

}

#endif

