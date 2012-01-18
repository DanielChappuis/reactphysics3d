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
        TrianglesStore();                           // Constructor
        ~TrianglesStore();                          // Destructor

        void clear();                               // Clear all the storage
        int getNbTriangles() const;                 // Return the number of triangles
        void setNbTriangles(int backup);            // Set the number of triangles
        TriangleEPA& last();                        // Return the last triangle

        TriangleEPA* newTriangle(const Vector3* vertices, uint v0, uint v1, uint v2);  // Create a new triangle

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
inline TriangleEPA* TrianglesStore::newTriangle(const Vector3* vertices, uint v0, uint v1, uint v2) {
    TriangleEPA* newTriangle = 0;

    // If we have not reached the maximum number of triangles
    if (nbTriangles != MAX_TRIANGLES) {
        newTriangle = &triangles[nbTriangles++];
        new (newTriangle) TriangleEPA(v0, v1, v2);
        if (!newTriangle->computeClosestPoint(vertices)) {
            nbTriangles--;
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