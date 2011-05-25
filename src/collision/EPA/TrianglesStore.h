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

#ifndef TRIANGLES_STORE_H
#define TRIANGLES_STORE_H

#include "TriangleEPA.h"


// Libraries
#include <cassert>

// ReactPhysics3D namespace
namespace reactphysics3d {

// Constants
const unsigned int MAX_TRIANGLES = 200;     // Maximum number of triangles


/*  -------------------------------------------------------------------
    Class TrianglesStore :
        This class stores several triangles of the polytope in the EPA
        algorithm.
    -------------------------------------------------------------------
*/
class TrianglesStore {
    private:
        TriangleEPA triangles[MAX_TRIANGLES];       // Triangles
        int nbTriangles;                            // Number of triangles
        
    public:
        TrianglesStore();               // Constructor
        ~TrianglesStore();              // Destructor

        void clear();                       // Clear all the storage
        int getNbTriangles() const;         // Return the number of triangles
        void setNbTriangles(int backup);    // Set the number of triangles
        TriangleEPA& last();                // Return the last triangle

        TriangleEPA* newTriangle(const Vector3D* vertices, uint v0, uint v1, uint v2);  // Create a new triangle

        TriangleEPA& operator[](int i);     // Access operator
};

// Clear all the storage
inline void TrianglesStore::clear() {
    nbTriangles = 0;
}

// Return the number of triangles
inline int TrianglesStore::getNbTriangles() const {
    return nbTriangles;
}


inline void TrianglesStore::setNbTriangles(int backup) {
    nbTriangles = backup;
}

// Return the last triangle
inline TriangleEPA& TrianglesStore::last() {
    assert(nbTriangles > 0);
    return triangles[nbTriangles - 1];
}

// Create a new triangle
inline TriangleEPA* TrianglesStore::newTriangle(const Vector3D* vertices, uint v0, uint v1, uint v2) {
    TriangleEPA* newTriangle = 0;

    // If we have not reach the maximum number of triangles
    if (nbTriangles != MAX_TRIANGLES) {
        newTriangle = &triangles[nbTriangles++];
        new (newTriangle) TriangleEPA(v0, v1, v2);
        if (!newTriangle->computeClosestPoint(vertices)) {
            nbTriangles--;
            // TODO : DO WE HAVE TO add "delete newTriangle;" here ??
            newTriangle = 0;
        }
    }

    // Return the new triangle
    return newTriangle;
}

// Access operator
inline TriangleEPA& TrianglesStore::operator[](int i) {
    return triangles[i];
}

}   // End of ReactPhysics3D namespace

#endif