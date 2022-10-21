/********************************************************************************
* ReactPhysics3D physics library, http://www.reactphysics3d.com                 *
* Copyright (c) 2010-2022 Daniel Chappuis                                       *
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

#ifndef REACTPHYSICS3D_CONVEX_MESH_H
#define REACTPHYSICS3D_CONVEX_MESH_H

// Libraries
#include <reactphysics3d/mathematics/mathematics.h>
#include <reactphysics3d/containers/Array.h>
#include "HalfEdgeStructure.h"

namespace reactphysics3d {

// Declarations
class DefaultAllocator;
class PolygonVertexArray;
struct Error;

// Class ConvexMesh
/**
 * This class describes a convex mesh made of faces and vertices.
 * The faces are made of polygons (not only triangles).
 */
class ConvexMesh {

    private:

        // -------------------- Attributes -------------------- //

        /// Reference to the memory allocator
        MemoryAllocator& mMemoryAllocator;

        /// Half-edge structure of the mesh
        HalfEdgeStructure mHalfEdgeStructure;

        // All the vertices of the mesh
        Array<Vector3> mVertices;

        /// Array with the face normals
        Array<Vector3> mFacesNormals;

        /// Centroid of the mesh
        Vector3 mCentroid;

        // -------------------- Methods -------------------- //

        /// Constructor
        ConvexMesh(MemoryAllocator& allocator, uint32 nbVertices, uint32 nbFaces);

        /// Initialize a mesh and returns errors if any
        bool init(PolygonVertexArray* polygonVertexArray, std::vector<Error>& errors);

        /// Copy the vertices into the mesh
        bool copyVertices(PolygonVertexArray* polygonVertexArray, std::vector<Error>& errors);

        /// Create the half-edge structure of the mesh
        bool createHalfEdgeStructure(PolygonVertexArray* polygonVertexArray, std::vector<Error> &errors);

        /// Compute the faces normals
        bool computeFacesNormals(std::vector<Error>& errors);

        /// Compute and return the area of a face
        decimal getFaceArea(uint32 faceIndex) const;

        /// Static factory method to create a convex mesh
        static ConvexMesh* create(PolygonVertexArray* polygonVertexArray, MemoryAllocator& allocator, std::vector<reactphysics3d::Error>& errors);

    public:

        // -------------------- Methods -------------------- //

        /// Return the number of vertices
        uint32 getNbVertices() const;

        /// Return a vertex
        const Vector3& getVertex(uint32 index) const;

        /// Return the number of faces
        uint32 getNbFaces() const;

        /// Return a face normal
        const Vector3& getFaceNormal(uint32 faceIndex) const;

        /// Return the half-edge structure of the mesh
        const HalfEdgeStructure& getHalfEdgeStructure() const;

        /// Return the centroid of the mesh
        Vector3 getCentroid() const;

        /// Compute and return the volume of the mesh
        decimal getVolume() const;

        // ---------- Friendship ---------- //

        friend class PhysicsCommon;
};

// Return the number of vertices
/**
 * @return The number of vertices in the mesh
 */
RP3D_FORCE_INLINE uint32 ConvexMesh::getNbVertices() const {
    return mVertices.size();
}

/// Return a vertex
/**
 * @param index Index of a given vertex in the mesh
 * @return The coordinates of a given vertex in the mesh
 */
RP3D_FORCE_INLINE const Vector3& ConvexMesh::getVertex(uint32 index) const {
    assert(index < mVertices.size());
    return mVertices[index];
}

// Return the number of faces
/**
 * @return The number of faces in the mesh
 */
RP3D_FORCE_INLINE uint32 ConvexMesh::getNbFaces() const {
   return mHalfEdgeStructure.getNbFaces();
}

// Return a face normal
/**
 * @param faceIndex The index of a given face of the mesh
 * @return The normal vector of a given face of the mesh
 */
RP3D_FORCE_INLINE const Vector3& ConvexMesh::getFaceNormal(uint32 faceIndex) const {
    assert(faceIndex < mHalfEdgeStructure.getNbFaces());
    return mFacesNormals[faceIndex];
}

// Return the half-edge structure of the mesh
/**
 * @return The Half-Edge structure of the mesh
 */
RP3D_FORCE_INLINE const HalfEdgeStructure& ConvexMesh::getHalfEdgeStructure() const {
    return mHalfEdgeStructure;
}

// Return the centroid of the polyhedron
/**
 * @return The centroid of the mesh
 */
RP3D_FORCE_INLINE Vector3 ConvexMesh::getCentroid() const {
    return mCentroid;
}

}

#endif

