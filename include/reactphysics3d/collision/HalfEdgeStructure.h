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

#ifndef REACTPHYSICS3D_HALF_EDGE_STRUCTURE_MESH_H
#define REACTPHYSICS3D_HALF_EDGE_STRUCTURE_MESH_H

// Libraries
#include <reactphysics3d/mathematics/mathematics.h>

namespace reactphysics3d {

// Class HalfEdgeStructure
/**
 * This class describes a polyhedron mesh made of faces and vertices.
 * The faces do not have to be triangle.
 */
class HalfEdgeStructure {

    public:

        using VerticesPair = Pair<uint32, uint32>;

        /// Edge
        struct Edge {
            uint32 vertexIndex;       // Index of the vertex at the beginning of the edge
            uint32 twinEdgeIndex;     // Index of the twin edge
            uint32 faceIndex;         // Adjacent face index of the edge
            uint32 nextEdgeIndex;     // Index of the next edge
        };

        /// Face
        struct Face {
            uint32 edgeIndex;             // Index of an half-edge of the face
            Array<uint32> faceVertices;	// Index of the vertices of the face

            /// Constructor
            Face(MemoryAllocator& allocator) : edgeIndex(0), faceVertices(allocator) {}

            /// Constructor
            Face(const Array<uint32>& vertices) : edgeIndex(0), faceVertices(vertices) {}    
        };

        /// Vertex
        struct Vertex {
            uint32 vertexPointIndex;  // Index of the vertex point in the origin vertex array
            uint32 edgeIndex;         // Index of one edge emanting from this vertex

            /// Constructor
            Vertex(uint32 vertexCoordsIndex) : vertexPointIndex(vertexCoordsIndex), edgeIndex(0) { }
        };

    private:

        /// Reference to a memory allocator
        MemoryAllocator& mAllocator;

        /// All the faces
        Array<Face> mFaces;

        /// All the vertices
        Array<Vertex> mVertices;

        /// All the half-edges
        Array<Edge> mEdges;

    public:

        /// Constructor
        HalfEdgeStructure(MemoryAllocator& allocator, uint32 facesCapacity, uint32 verticesCapacity,
                          uint32 edgesCapacity) :mAllocator(allocator), mFaces(allocator, facesCapacity),
                          mVertices(allocator, verticesCapacity), mEdges(allocator, edgesCapacity) {}

        /// Destructor
        ~HalfEdgeStructure() = default;

        /// Compute the half-edges (when all vertices and faces have been added)
        void computeHalfEdges();

        /// Add a vertex
        uint32 addVertex(uint32 vertexPointIndex);

        /// Add a face
        void addFace(const Array<uint32>& faceVertices);

        /// Return the number of faces
        uint32 getNbFaces() const;

        /// Return the number of half-edges
        uint32 getNbHalfEdges() const;

        /// Return the number of vertices
        uint32 getNbVertices() const;

        /// Return a given face
        const Face& getFace(uint32 index) const;

        /// Return a given edge
        const Edge& getHalfEdge(uint32 index) const;

        /// Return a given vertex
        const Vertex& getVertex(uint32 index) const;

        /// Reserve some memory for vertices, faces and edges
        void reserve(uint32 facesCapacity, uint32 verticesCapacity, uint32 edgesCapacity);

        /// Return a string representation of the half-edge structure
        std::string to_string() const;

};

// Add a vertex
/**
 * @param vertexPointIndex Index of the vertex in the external user vertex data array
 */
RP3D_FORCE_INLINE uint32 HalfEdgeStructure::addVertex(uint32 vertexPointIndex) {
    Vertex vertex(vertexPointIndex);
    mVertices.add(vertex);
    return static_cast<uint32>(mVertices.size()) - 1;
}

// Add a face
/**
 * @param faceVertices Array of the vertices in a face (ordered in CCW order as seen from outside
 *                     the polyhedron). The indices are the internal indices of the vertices inside the HalfEdgeStructure.
 */
RP3D_FORCE_INLINE void HalfEdgeStructure::addFace(const Array<uint32>& faceVertices) {

    // Create a new face
    mFaces.add(Face(faceVertices));
}

// Return the number of faces
/**
 * @return The number of faces in the polyhedron
 */
RP3D_FORCE_INLINE uint32 HalfEdgeStructure::getNbFaces() const {
    return static_cast<uint32>(mFaces.size());
}

// Return the number of edges
/**
 * @return The number of edges in the polyhedron
 */
RP3D_FORCE_INLINE uint32 HalfEdgeStructure::getNbHalfEdges() const {
    return static_cast<uint32>(mEdges.size());
}

// Return the number of vertices
/**
 * @return The number of vertices in the polyhedron
 */
RP3D_FORCE_INLINE uint32 HalfEdgeStructure::getNbVertices() const {
    return static_cast<uint32>(mVertices.size());
}

// Return a given face
/**
 * @return A given face of the polyhedron
 */
RP3D_FORCE_INLINE const HalfEdgeStructure::Face& HalfEdgeStructure::getFace(uint32 index) const {
    assert(index < mFaces.size());
    return mFaces[index];
}

// Return a given edge
/**
 * @return A given edge of the polyhedron
 */
RP3D_FORCE_INLINE const HalfEdgeStructure::Edge& HalfEdgeStructure::getHalfEdge(uint32 index) const {
    assert(index < mEdges.size());
    return mEdges[index];
}

// Return a given vertex
/**
 * @return A given vertex of the polyhedron
 */
RP3D_FORCE_INLINE const HalfEdgeStructure::Vertex& HalfEdgeStructure::getVertex(uint32 index) const {
    assert(index < mVertices.size());
    return mVertices[index];
}

}

#endif

