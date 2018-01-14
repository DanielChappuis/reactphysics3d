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
#include "collision/PolygonVertexArray.h"
#include "memory/DefaultAllocator.h"
#include <vector>

namespace reactphysics3d {

// Class PolyhedronMesh
/**
 * This class describes a polyhedron mesh made of faces and vertices.
 * The faces do not have to be triangles.
 */
class PolyhedronMesh {

    private:

        // -------------------- Attributes -------------------- //

        /// Pointer the the polygon vertex array with vertices and faces
        /// of the mesh
        PolygonVertexArray* mPolygonVertexArray;

        /// Half-edge structure of the mesh
        HalfEdgeStructure mHalfEdgeStructure;

        /// Array with the face normals
        Vector3* mFacesNormals;

        /// Centroid of the polyhedron
        Vector3 mCentroid;

        // -------------------- Methods -------------------- //

        /// Create the half-edge structure of the mesh
        void createHalfEdgeStructure();

        /// Compute the faces normals
        void computeFacesNormals();

        /// Compute the centroid of the polyhedron
        void computeCentroid() ;

    public:

        // -------------------- Methods -------------------- //

        /// Constructor
        PolyhedronMesh(PolygonVertexArray* polygonVertexArray);

        /// Destructor
        ~PolyhedronMesh();

        /// Return the number of vertices
        uint getNbVertices() const;

        /// Return a vertex
        Vector3 getVertex(uint index) const;

        /// Return a face normal
        Vector3 getFaceNormal(uint faceIndex) const;

        /// Return the half-edge structure of the mesh
        const HalfEdgeStructure& getHalfEdgeStructure() const;

        /// Return the centroid of the polyhedron
        Vector3 getCentroid() const;
};

// Return the number of vertices
inline uint PolyhedronMesh::getNbVertices() const {
    return mHalfEdgeStructure.getNbVertices();
}

// Return a face normal
inline Vector3 PolyhedronMesh::getFaceNormal(uint faceIndex) const {
    assert(faceIndex < mHalfEdgeStructure.getNbFaces());
    return mFacesNormals[faceIndex];
}

// Return the half-edge structure of the mesh
inline const HalfEdgeStructure& PolyhedronMesh::getHalfEdgeStructure() const {
    return mHalfEdgeStructure;
}

// Return the centroid of the polyhedron
inline Vector3 PolyhedronMesh::getCentroid() const {
    return mCentroid;
}

}

#endif

