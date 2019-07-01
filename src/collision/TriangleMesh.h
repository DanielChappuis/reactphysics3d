/********************************************************************************
* ReactPhysics3D physics library, http://www.reactphysics3d.com                 *
* Copyright (c) 2010-2019 Daniel Chappuis                                       *
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

#ifndef REACTPHYSICS3D_TRIANGLE_MESH_H
#define REACTPHYSICS3D_TRIANGLE_MESH_H

// Libraries
#include <cassert>
#include "containers/List.h"

namespace reactphysics3d {

// Declarations
class TriangleVertexArray;
class MemoryManager;

// Class TriangleMesh
/**
 * This class represents a mesh made of triangles. A TriangleMesh contains
 * one or several parts. Each part is a set of triangles represented in a
 * TriangleVertexArray object describing all the triangles vertices of the part.
 * A TriangleMesh object can be used to create a ConcaveMeshShape from a triangle
 * mesh for instance.
 */
class TriangleMesh {

    protected:

        /// All the triangle arrays of the mesh (one triangle array per part)
        List<TriangleVertexArray*> mTriangleArrays;

    public:

        /// Constructor
        TriangleMesh();

        /// Destructor
        ~TriangleMesh() = default;

        /// Add a subpart of the mesh
        void addSubpart(TriangleVertexArray* triangleVertexArray);

        /// Return a pointer to a given subpart (triangle vertex array) of the mesh
        TriangleVertexArray* getSubpart(uint indexSubpart) const;

        /// Return the number of subparts of the mesh
        uint getNbSubparts() const;
};

// Add a subpart of the mesh
/**
 * @param triangleVertexArray Pointer to the TriangleVertexArray to add into the mesh
 */
inline void TriangleMesh::addSubpart(TriangleVertexArray* triangleVertexArray) {
    mTriangleArrays.add(triangleVertexArray );
}

// Return a pointer to a given subpart (triangle vertex array) of the mesh
/**
 * @param indexSubpart The index of the sub-part of the mesh
 * @return A pointer to the triangle vertex array of a given sub-part of the mesh
 */
inline TriangleVertexArray* TriangleMesh::getSubpart(uint indexSubpart) const {
   assert(indexSubpart < mTriangleArrays.size());
   return mTriangleArrays[indexSubpart];
}

// Return the number of sub-parts of the mesh
/**
 * @return The number of sub-parts of the mesh
 */
inline uint TriangleMesh::getNbSubparts() const {
    return mTriangleArrays.size();
}

}

#endif

