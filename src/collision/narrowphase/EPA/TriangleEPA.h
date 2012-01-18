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

#ifndef TRIANGLE_EPA_H
#define TRIANGLE_EPA_H

// Libraries
#include "../../../mathematics/mathematics.h"
#include "../../../configuration.h"
#include "EdgeEPA.h"
#include <cassert>

// ReactPhysics3D namespace
namespace reactphysics3d {

// Prototypes
bool link(const EdgeEPA& edge0, const EdgeEPA& edge1);
void halfLink(const EdgeEPA& edge0, const EdgeEPA& edge1);


/*  -------------------------------------------------------------------
    Class TriangleEPA :
        This class represents a triangle face of the current polytope in the EPA
        algorithm.
    -------------------------------------------------------------------
*/
class TriangleEPA {
    private:
        uint indicesVertices[3];    // Indices of the vertices y_i of the triangle
        EdgeEPA adjacentEdges[3];   // Three adjacent edges of the triangle (edges of other triangles)
        bool isObsolete;            // True if the triangle face is visible from the new support point
        decimal det;                // Determinant
        Vector3 closestPoint;       // Point v closest to the origin on the affine hull of the triangle
        decimal lambda1;            // Lambda1 value such that v = lambda0 * y_0 + lambda1 * y_1 + lambda2 * y_2
        decimal lambda2;            // Lambda1 value such that v = lambda0 * y_0 + lambda1 * y_1 + lambda2 * y_2
        decimal distSquare;         // Square distance of the point closest point v to the origin

    public:
        TriangleEPA();                              // Constructor
        TriangleEPA(uint v1, uint v2, uint v3);     // Constructor
        ~TriangleEPA();                             // Destructor

        EdgeEPA& getAdjacentEdge(int index);                                                // Return an adjacent edge of the triangle
        void setAdjacentEdge(int index, EdgeEPA& edge);                                     // Set an adjacent edge of the triangle
        decimal getDistSquare() const;                                                       // Return the square distance  of the closest point to origin
        void setIsObsolete(bool isObsolete);                                                // Set the isObsolete value
        bool getIsObsolete() const;                                                         // Return true if the triangle face is obsolete
        const Vector3& getClosestPoint() const;                                            // Return the point closest to the origin
        bool isClosestPointInternalToTriangle() const;                                      // Return true if the closest point on affine hull is inside the triangle
        bool isVisibleFromVertex(const Vector3* vertices, uint index) const;               // Return true if the triangle is visible from a given vertex
        bool computeClosestPoint(const Vector3* vertices);                                 // Compute the point v closest to the origin of this triangle
        Vector3 computeClosestPointOfObject(const Vector3* supportPointsOfObject) const;  // Compute the point of an object closest to the origin
        bool computeSilhouette(const Vector3* vertices, uint index,
                               TrianglesStore& triangleStore);                              // Execute the recursive silhouette algorithm from this triangle face

        uint operator[](int i) const;                                                       // Access operator
        friend bool link(const EdgeEPA& edge0, const EdgeEPA& edge1);                       // Associate two edges
        friend void halfLink(const EdgeEPA& edge0, const EdgeEPA& edge1);                   // Make a half-link between two edges
};

// Return an edge of the triangle
inline EdgeEPA& TriangleEPA::getAdjacentEdge(int index) {
    assert(index >= 0 && index < 3);
    return adjacentEdges[index];
}

// Set an adjacent edge of the triangle
inline void TriangleEPA::setAdjacentEdge(int index, EdgeEPA& edge) {
    assert(index >=0 && index < 3);
    adjacentEdges[index] = edge;
}

// Return the square distance  of the closest point to origin
inline decimal TriangleEPA::getDistSquare() const {
    return distSquare;
}

// Set the isObsolete value
inline void TriangleEPA::setIsObsolete(bool isObsolete) {
    this->isObsolete = isObsolete;
}

// Return true if the triangle face is obsolete
inline bool TriangleEPA::getIsObsolete() const {
    return isObsolete;
}

// Return the point closest to the origin
inline const Vector3& TriangleEPA::getClosestPoint() const {
    return closestPoint;
}

// Return true if the closest point on affine hull is inside the triangle
inline bool TriangleEPA::isClosestPointInternalToTriangle() const {
    return (lambda1 >= 0.0 && lambda2 >= 0.0 && (lambda1 + lambda2) <= det);
}

// Return true if the triangle is visible from a given vertex
inline bool TriangleEPA::isVisibleFromVertex(const Vector3* vertices, uint index) const {
    Vector3 closestToVert = vertices[index] - closestPoint;
    return (closestPoint.dot(closestToVert) > 0.0);
}

// Compute the point of an object closest to the origin
inline Vector3 TriangleEPA::computeClosestPointOfObject(const Vector3* supportPointsOfObject) const {
    const Vector3& p0 = supportPointsOfObject[indicesVertices[0]];
    return p0 + 1.0/det * (lambda1 * (supportPointsOfObject[indicesVertices[1]] - p0) +
                           lambda2 * (supportPointsOfObject[indicesVertices[2]] - p0));
}

// Access operator
inline uint TriangleEPA::operator[](int i) const {
    assert(i >= 0 && i <3);
    return indicesVertices[i];
}

}   // End of ReactPhysics3D namespace

#endif