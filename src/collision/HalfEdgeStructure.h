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

#ifndef REACTPHYSICS3D_HALF_EDGE_STRUCTURE_MESH_H
#define REACTPHYSICS3D_HALF_EDGE_STRUCTURE_MESH_H

// Libraries
#include "mathematics/mathematics.h"
#include <vector>

namespace reactphysics3d {

// Class HalfEdgeStructure
/**
 * This class describes a polyhedron mesh made of faces and vertices.
 * The faces do not have to be triangle. Note that the half-edge structure
 * is only valid if the mesh is closed (each edge has two adjacent faces).
 */
class HalfEdgeStructure {

    public:

        struct Edge {
            uint vertexIndex;       // Index of the vertex at the beginning of the edge
            uint twinEdgeIndex;     // Index of the twin edge
            uint faceIndex;         // Adjacent face index of the edge
            uint nextEdgeIndex;     // Index of the next edge
        };

        struct Face {
            uint edgeIndex;                     // Index of an half-edge of the face
            std::vector<uint> faceVertices;     // Index of the vertices of the face

            /// Constructor
            Face(std::vector<uint> vertices) : faceVertices(vertices) {}
        };

        struct Vertex {
            uint vertexPointIndex;  // Index of the vertex point in the origin vertex array
            uint edgeIndex;         // Index of one edge emanting from this vertex

            /// Constructor
            Vertex(uint vertexCoordsIndex) : vertexPointIndex(vertexCoordsIndex) { }
        };

    private:

        /// All the faces
        std::vector<Face> mFaces;

        /// All the vertices
        std::vector<Vertex> mVertices;

        /// All the half-edges
        std::vector<Edge> mEdges;

    public:

        /// Constructor
        HalfEdgeStructure() = default;

        /// Destructor
        ~HalfEdgeStructure() = default;

        /// Initialize the structure (when all vertices and faces have been added)
        void init();

        /// Add a vertex
        uint addVertex(uint vertexPointIndex);

        /// Add a face
        void addFace(std::vector<uint> faceVertices);

        /// Return the number of faces
        uint getNbFaces() const;

        /// Return the number of half-edges
        uint getNbHalfEdges() const;

        /// Return the number of vertices
        uint getNbVertices() const;

        /// Return a given face
        Face getFace(uint index) const;

        /// Return a given edge
        Edge getHalfEdge(uint index) const;

        /// Return a given vertex
        Vertex getVertex(uint index) const;

};

// Add a vertex
inline uint HalfEdgeStructure::addVertex(uint vertexPointIndex) {
    Vertex vertex(vertexPointIndex);
    mVertices.push_back(vertex);
    return mVertices.size() - 1;
}

// Add a face
inline void HalfEdgeStructure::addFace(std::vector<uint> faceVertices) {

    // Create a new face
    Face face(faceVertices);
    mFaces.push_back(face);
}

// Return the number of faces
inline uint HalfEdgeStructure::getNbFaces() const {
    return mFaces.size();
}

// Return the number of edges
inline uint HalfEdgeStructure::getNbHalfEdges() const {
    return mEdges.size();
}

// Return the number of vertices
inline uint HalfEdgeStructure::getNbVertices() const {
    return mVertices.size();
}

// Return a given face
inline HalfEdgeStructure::Face HalfEdgeStructure::getFace(uint index) const {
    assert(index < mFaces.size());
    return mFaces[index];
}

// Return a given edge
inline HalfEdgeStructure::Edge HalfEdgeStructure::getHalfEdge(uint index) const {
    assert(index < mEdges.size());
    return mEdges[index];
}

// Retunr a given vertex
inline HalfEdgeStructure::Vertex HalfEdgeStructure::getVertex(uint index) const {
    assert(index < mVertices.size());
    return mVertices[index];
}

}

#endif

