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

#ifndef REACTPHYSICS3D_TRIANGLE_EPA_H
#define REACTPHYSICS3D_TRIANGLE_EPA_H

// Libraries
#include "mathematics/mathematics.h"
#include "configuration.h"
#include "EdgeEPA.h"
#include <cassert>

/// ReactPhysics3D namespace
namespace reactphysics3d {

// Prototypes
bool link(const EdgeEPA& edge0, const EdgeEPA& edge1);
void halfLink(const EdgeEPA& edge0, const EdgeEPA& edge1);


// Class TriangleEPA
/**
 * This class represents a triangle face of the current polytope in the EPA algorithm.
 */
class TriangleEPA {

    private:

        // -------------------- Attributes -------------------- //

        /// Indices of the vertices y_i of the triangle
        uint mIndicesVertices[3];

        /// Three adjacent edges of the triangle (edges of other triangles)
        EdgeEPA mAdjacentEdges[3];

        /// True if the triangle face is visible from the new support point
        bool mIsObsolete;

        /// Determinant
        decimal mDet;

        /// Point v closest to the origin on the affine hull of the triangle
        Vector3 mClosestPoint;

        /// Lambda1 value such that v = lambda0 * y_0 + lambda1 * y_1 + lambda2 * y_2
        decimal mLambda1;

        /// Lambda1 value such that v = lambda0 * y_0 + lambda1 * y_1 + lambda2 * y_2
        decimal mLambda2;

        /// Square distance of the point closest point v to the origin
        decimal mDistSquare;

        // -------------------- Methods -------------------- //

        /// Private copy-constructor
        TriangleEPA(const TriangleEPA& triangle);

        /// Private assignment operator
        TriangleEPA& operator=(const TriangleEPA& triangle);

    public:

        // -------------------- Methods -------------------- //

        /// Constructor
        TriangleEPA();

        /// Constructor
        TriangleEPA(uint v1, uint v2, uint v3);

        /// Destructor
        ~TriangleEPA();

        /// Return an adjacent edge of the triangle
        EdgeEPA& getAdjacentEdge(int index);

        /// Set an adjacent edge of the triangle
        void setAdjacentEdge(int index, EdgeEPA& edge);

        /// Return the square distance of the closest point to origin
        decimal getDistSquare() const;

        /// Set the isObsolete value
        void setIsObsolete(bool isObsolete);

        /// Return true if the triangle face is obsolete
        bool getIsObsolete() const;

        /// Return the point closest to the origin
        const Vector3& getClosestPoint() const;

        // Return true if the closest point on affine hull is inside the triangle
        bool isClosestPointInternalToTriangle() const;

        /// Return true if the triangle is visible from a given vertex
        bool isVisibleFromVertex(const Vector3* vertices, uint index) const;

        /// Compute the point v closest to the origin of this triangle
        bool computeClosestPoint(const Vector3* vertices);

        /// Compute the point of an object closest to the origin
        Vector3 computeClosestPointOfObject(const Vector3* supportPointsOfObject) const;

        /// Execute the recursive silhouette algorithm from this triangle face.
        bool computeSilhouette(const Vector3* vertices, uint index, TrianglesStore& triangleStore);

        /// Access operator
        uint operator[](int i) const;

        /// Associate two edges
        friend bool link(const EdgeEPA& edge0, const EdgeEPA& edge1);

        /// Make a half-link between two edges
        friend void halfLink(const EdgeEPA& edge0, const EdgeEPA& edge1);
};

// Return an edge of the triangle
inline EdgeEPA& TriangleEPA::getAdjacentEdge(int index) {
    assert(index >= 0 && index < 3);
    return mAdjacentEdges[index];
}

// Set an adjacent edge of the triangle
inline void TriangleEPA::setAdjacentEdge(int index, EdgeEPA& edge) {
    assert(index >=0 && index < 3);
    mAdjacentEdges[index] = edge;
}

// Return the square distance  of the closest point to origin
inline decimal TriangleEPA::getDistSquare() const {
    return mDistSquare;
}

// Set the isObsolete value
inline void TriangleEPA::setIsObsolete(bool isObsolete) {
    mIsObsolete = isObsolete;
}

// Return true if the triangle face is obsolete
inline bool TriangleEPA::getIsObsolete() const {
    return mIsObsolete;
}

// Return the point closest to the origin
inline const Vector3& TriangleEPA::getClosestPoint() const {
    return mClosestPoint;
}

// Return true if the closest point on affine hull is inside the triangle
inline bool TriangleEPA::isClosestPointInternalToTriangle() const {
    return (mLambda1 >= 0.0 && mLambda2 >= 0.0 && (mLambda1 + mLambda2) <= mDet);
}

// Return true if the triangle is visible from a given vertex
inline bool TriangleEPA::isVisibleFromVertex(const Vector3* vertices, uint index) const {
    Vector3 closestToVert = vertices[index] - mClosestPoint;
    return (mClosestPoint.dot(closestToVert) > 0.0);
}

// Compute the point of an object closest to the origin
inline Vector3 TriangleEPA::computeClosestPointOfObject(const Vector3* supportPointsOfObject) const{
    const Vector3& p0 = supportPointsOfObject[mIndicesVertices[0]];
    return p0 + decimal(1.0)/mDet * (mLambda1 * (supportPointsOfObject[mIndicesVertices[1]] - p0) +
                           mLambda2 * (supportPointsOfObject[mIndicesVertices[2]] - p0));
}

// Access operator
inline uint TriangleEPA::operator[](int i) const {
    assert(i >= 0 && i <3);
    return mIndicesVertices[i];
}

}

#endif
