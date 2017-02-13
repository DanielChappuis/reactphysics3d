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

#ifndef REACTPHYSICS3D_POLYHEDRON_MESH_H
#define REACTPHYSICS3D_POLYHEDRON_MESH_H

// Libraries
#include "mathematics/mathematics.h"
#include "HalfEdgeStructure.h"
#include <vector>

namespace reactphysics3d {

// Class PolyhedronMesh
/**
 * This class describes a polyhedron mesh made of faces and vertices.
 * The faces do not have to be triangle
 */
class PolyhedronMesh {

    private:

        /// Half-edge structure of the mesh
        HalfEdgeStructure mHalfEdgeStructure;

        /// True if the half-edge structure has been generated
        bool mIsFinalized;

        /// All the vertices
        std::vector<Vector3> mVertices;

        /// All the indexes of the face vertices
        std::vector<std::vector<uint>> mFaces;

    public:

        /// Constructor
        PolyhedronMesh();

        /// Destructor
        ~PolyhedronMesh() = default;

        /// Add a vertex into the polyhedron
        uint addVertex(const Vector3& vertex);

        /// Add a face into the polyhedron
        void addFace(std::vector<uint> faceVertices);

        /// Call this method when you are done adding vertices and faces
        void finalize();

        /// Return the half-edge structure of the mesh
        const HalfEdgeStructure& getHalfEdgeStructure() const;
};

// Return the half-edge structure of the mesh
inline const HalfEdgeStructure& PolyhedronMesh::getHalfEdgeStructure() const {
    return mHalfEdgeStructure;
}

}

#endif

