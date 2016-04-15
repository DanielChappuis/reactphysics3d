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

#ifndef REACTPHYSICS3D_TRIANGLES_STORE_H
#define REACTPHYSICS3D_TRIANGLES_STORE_H

#include "TriangleEPA.h"


// Libraries
#include <cassert>

/// ReactPhysics3D namespace
namespace reactphysics3d {

// Constants
const unsigned int MAX_TRIANGLES = 200;     // Maximum number of triangles

// Class TriangleStore
/**
 * This class stores several triangles of the polytope in the EPA algorithm.
 */
class TrianglesStore {

    private:

        // -------------------- Attributes -------------------- //

        /// Triangles
        TriangleEPA mTriangles[MAX_TRIANGLES];

        /// Number of triangles
        int mNbTriangles;

        // -------------------- Methods -------------------- //

        /// Private copy-constructor
        TrianglesStore(const TrianglesStore& triangleStore);

        /// Private assignment operator
        TrianglesStore& operator=(const TrianglesStore& triangleStore);
        
    public:

        // -------------------- Methods -------------------- //

        /// Constructor
        TrianglesStore();

        /// Destructor
        ~TrianglesStore();

        /// Clear all the storage
        void clear();

        /// Return the number of triangles
        int getNbTriangles() const;

        /// Set the number of triangles
        void setNbTriangles(int backup);

        /// Return the last triangle
        TriangleEPA& last();

        /// Create a new triangle
        TriangleEPA* newTriangle(const Vector3* vertices, uint v0, uint v1, uint v2);

        /// Access operator
        TriangleEPA& operator[](int i);
};

// Clear all the storage
inline void TrianglesStore::clear() {
    mNbTriangles = 0;
}

// Return the number of triangles
inline int TrianglesStore::getNbTriangles() const {
    return mNbTriangles;
}


inline void TrianglesStore::setNbTriangles(int backup) {
    mNbTriangles = backup;
}

// Return the last triangle
inline TriangleEPA& TrianglesStore::last() {
    assert(mNbTriangles > 0);
    return mTriangles[mNbTriangles - 1];
}

// Create a new triangle
inline TriangleEPA* TrianglesStore::newTriangle(const Vector3* vertices,
                                                uint v0,uint v1, uint v2) {
    TriangleEPA* newTriangle = NULL;

    // If we have not reached the maximum number of triangles
    if (mNbTriangles != MAX_TRIANGLES) {
        newTriangle = &mTriangles[mNbTriangles++];
        new (newTriangle) TriangleEPA(v0, v1, v2);
        if (!newTriangle->computeClosestPoint(vertices)) {
            mNbTriangles--;
            newTriangle = NULL;
        }
    }

    // Return the new triangle
    return newTriangle;
}

// Access operator
inline TriangleEPA& TrianglesStore::operator[](int i) {
    return mTriangles[i];
}

}

#endif
