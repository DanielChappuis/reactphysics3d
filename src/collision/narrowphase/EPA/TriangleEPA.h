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

#ifndef TRIANGLE_EPA_H
#define TRIANGLE_EPA_H

// Libraries
#include "../../../mathematics/mathematics.h"
#include "../../../constants.h"
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
        double det;                 // Determinant
        Vector3 closestPoint;      // Point v closest to the origin on the affine hull of the triangle
        double lambda1;             // Lambda1 value such that v = lambda0 * y_0 + lambda1 * y_1 + lambda2 * y_2
        double lambda2;             // Lambda1 value such that v = lambda0 * y_0 + lambda1 * y_1 + lambda2 * y_2
        double distSquare;          // Square distance of the point closest point v to the origin

    public:
        TriangleEPA();                              // Constructor
        TriangleEPA(uint v1, uint v2, uint v3);     // Constructor
        ~TriangleEPA();                             // Destructor

        EdgeEPA& getAdjacentEdge(int index);                                                // Return an adjacent edge of the triangle
        void setAdjacentEdge(int index, EdgeEPA& edge);                                     // Set an adjacent edge of the triangle
        double getDistSquare() const;                                                       // Return the square distance  of the closest point to origin
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
inline double TriangleEPA::getDistSquare() const {
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