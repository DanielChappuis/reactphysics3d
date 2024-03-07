/********************************************************************************
* ReactPhysics3D physics library, http://www.reactphysics3d.com                 *
* Copyright (c) 2010-2024 Daniel Chappuis                                       *
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
#include <reactphysics3d/collision/shapes/AABB.h>
#include "HalfEdgeStructure.h"

namespace reactphysics3d {

// Declarations
class DefaultAllocator;
class PolygonVertexArray;
struct Message;

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

        /// All the vertices of the mesh
        Array<Vector3> mVertices;

        /// Array with the face normals
        Array<Vector3> mFacesNormals;

        /// Centroid of the mesh
        Vector3 mCentroid;

        /// Mesh minimum/maximum bounds in the three local x, y and z directions
        AABB mBounds;

        /// Volume of the mesh
        decimal mVolume;

        // -------------------- Methods -------------------- //

        /// Constructor
        ConvexMesh(MemoryAllocator& allocator);

        /// Initialize a mesh and returns errors if any
        bool init(const PolygonVertexArray& polygonVertexArray, std::vector<Message>& errors);

        /// Copy the vertices into the mesh
        bool copyVertices(const PolygonVertexArray& polygonVertexArray, std::vector<Message>& errors);

        /// Create the half-edge structure of the mesh
        bool createHalfEdgeStructure(const PolygonVertexArray& polygonVertexArray, std::vector<Message>& errors);

        /// Compute the faces normals
        bool computeFacesNormals(std::vector<Message>& errors);

        /// Compute and return the face normal (not normalized)
        Vector3 computeFaceNormal(uint32 faceIndex) const;

        /// Compute the volume of the mesh
        void computeVolume();

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
        const Vector3& getCentroid() const;

        /// Return the bounds of the mesh in the x,y,z direction
        const AABB& getBounds() const;

        /// Compute and return the volume of the mesh
        decimal getVolume() const;

        /// Return the local inertia tensor of the mesh
        Vector3 getLocalInertiaTensor(decimal mass, Vector3 scale) const;

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
RP3D_FORCE_INLINE const Vector3& ConvexMesh::getCentroid() const {
    return mCentroid;
}

// Return the volume of the convex mesh
/**
 * @return The volume of the mesh
 */
RP3D_FORCE_INLINE decimal ConvexMesh::getVolume() const {
   return mVolume;
}

// Return the local inertia tensor of the mesh
/// The local inertia tensor of the convex mesh is approximated using the inertia tensor
/// of its bounding box.
/**
* @param mass Mass to use to compute the inertia tensor of the collision shape
* @param scale Scaling factor for the mesh
*/
RP3D_FORCE_INLINE Vector3 ConvexMesh::getLocalInertiaTensor(decimal mass, Vector3 scale) const {

    // TODO: We should compute a much better inertia tensor here (not using a box)

    const decimal factor = (decimal(1.0) / decimal(3.0)) * mass;
    const Vector3 realExtent = decimal(0.5) * scale * (mBounds.getMax() - mBounds.getMin());
    assert(realExtent.x > 0 && realExtent.y > 0 && realExtent.z > 0);
    const decimal xSquare = realExtent.x * realExtent.x;
    const decimal ySquare = realExtent.y * realExtent.y;
    const decimal zSquare = realExtent.z * realExtent.z;
    return Vector3(factor * (ySquare + zSquare), factor * (xSquare + zSquare), factor * (xSquare + ySquare));
}

}

#endif

