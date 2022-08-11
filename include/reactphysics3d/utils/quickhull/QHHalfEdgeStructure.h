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

#ifndef REACTPHYSICS3D_QH_HALF_EDGE_STRUCTURE_MESH_H
#define REACTPHYSICS3D_QH_HALF_EDGE_STRUCTURE_MESH_H

// Libraries
#include <reactphysics3d/mathematics/mathematics.h>
#include <reactphysics3d/containers/Map.h>

namespace reactphysics3d {

// Class QHHalfEdgeStructure
/**
 * This class describes a polyhedron mesh made of faces and vertices.
 * The faces do not have to be triangles. Note that in the linked-list of
 * half-edges an edge must always follow its twin edge. This way, on closed
 * mesh, we can always iterate every two half-edges to find the unique edges.
 * This class is only used for QuickHull algorithm.
 */
class QHHalfEdgeStructure {

    public:

        // Forward declaration
        struct Vertex;
        struct Face;

        using VerticesPair = Pair<uint32, uint32>;
        using EdgeVertices = Pair<const Vertex*, const Vertex*>;

        /// Edge
        struct Edge {

            Vertex* startVertex;        // Vertex at the beginning of the edge
            Vertex* endVertex;          // Vertex at the end of the edge
            Face* face;                 // Adjacent face of the edge
            Edge* previousEdge;         // Previous edge in the linked-list of edges
            Edge* nextEdge;             // Next edge in the linked-list of edges
            Edge* previousFaceEdge;     // Previous edge around the face of the edge
            Edge* nextFaceEdge;         // Next edge around the face of the edge
            Edge* twinEdge;             // Twin edge

            Edge(Vertex* startVertex, Vertex* endVertex, Face* face)
                :startVertex(startVertex), endVertex(endVertex), face(face), previousEdge(nullptr), nextEdge(nullptr),
                 previousFaceEdge(nullptr), nextFaceEdge(nullptr), twinEdge(nullptr) {}

            // Return the next edge of the next edge (if any)
            const Edge* getNextSecondEdge() const {

               if (nextEdge != nullptr) {
                   return nextEdge->nextEdge;
               }

               return nullptr;
            }
        };

        /// Face
        struct Face {

            Face* nextFace;
            Face* previousFace;

            Edge* edge;             // One half-edge of the face

            /// Constructor
            Face(): nextFace(nullptr), previousFace(nullptr) {}

            // Return a vertex of the face
            const Vertex* getVertex() const {
                return edge->startVertex;
            }

            // Return a string with the vertices of the face
            std::string verticesString() const {

               std::string verticesString = "(";

               const Edge* firstFaceEdge = edge;
               const Edge* faceEdge = edge;
               do {

                   verticesString += std::to_string(faceEdge->startVertex->externalIndex);

                   faceEdge = faceEdge->nextFaceEdge;

                   if (faceEdge != firstFaceEdge) {
                       verticesString += ",";
                   }

               } while(faceEdge != firstFaceEdge);
               verticesString += ")";

               return verticesString;
            }
        };

        /// Vertex
        struct Vertex {

            uint32 externalIndex;       // Index of the vertex point in the user vertex array

            Vertex* previousVertex;
            Vertex* nextVertex;

            /// Constructor
            Vertex(uint32 externalIndex, Vertex* previousVertex, Vertex* nextVertex)
                : externalIndex(externalIndex), previousVertex(previousVertex),
                  nextVertex(nextVertex) { }
        };

    private:

        // ---------- Attributes ---------- //

        /// Reference to a memory allocator
        MemoryAllocator& mAllocator;

        /// Map a pair of vertice to the corresponding edge
        Map<EdgeVertices, Edge*> mMapVerticesToEdge;

        /// Number of faces
        uint32 mNbFaces = 0;

        /// Number of half-edges
        uint32 mNbHalfEdges = 0;

        /// Number of vertices
        uint32 mNbVertices = 0;

        /// Linked-list of faces
        Face* mFaces = nullptr;

        /// Linked-list of vertices
        Vertex* mVertices = nullptr;

        /// Linked-list of half-edges
        Edge* mHalfEdges = nullptr;

        // ---------- Methods ---------- //

        /// Add an edge before another one in the linked-list of edges
        void addEdgeToLinkedListBefore(Edge* newEdge, Edge* edge);

        /// Remove an half-edge from the linked-list of half-edges
        void removeEdgeFromLinkedList(Edge* edge);

        /// Add a face to the linked-list of faces
        void addFaceToLinkedList(Face* face);

        /// Remove a face from the linked-list of faces
        void removeFaceFromLinkedList(Face* face);

        /// Remove an half-edge
        void removeHalfEdge(Edge* edge);

    public:

        // ---------- Methods ---------- //

        /// Constructor
        QHHalfEdgeStructure(MemoryAllocator& allocator) :mAllocator(allocator), mMapVerticesToEdge(allocator) {}

        /// Destructor
        ~QHHalfEdgeStructure() = default;

        /// Add a vertex
        Vertex* addVertex(uint32 externalIndex);

        /// Add a face
        Face* addFace(const Array<Vertex*>& faceVertices);

        /// Remove a face
        void removeFace(Face* face);

        /// Remove a vertex
        void removeVertex(Vertex* vertex);

        /// Return the number of faces
        uint32 getNbFaces() const;

        /// Return the number of half-edges
        uint32 getNbHalfEdges() const;

        /// Return the number of vertices
        uint32 getNbVertices() const;

        /// Return a pointer to the first face in the linked-list of faces
        const Face* getFaces() const;

        /// Return a pointer to the first half-edge in the linked-list of half-edges
        const Edge* getHalfEdges() const;

        /// Return a pointer to the first vertex in the linked-list of vertices
        const Vertex* getVertices() const;

        /// Return true if the half-edge structure is valid (for debugging purpose)
        bool isValid() const;

        /// Return a string representation of the half-edge structure
        std::string to_string() const;        
};

// Return the number of faces
/**
 * @return The number of faces in the polyhedron
 */
RP3D_FORCE_INLINE uint32 QHHalfEdgeStructure::getNbFaces() const {
    return mNbFaces;
}

// Return the number of edges
/**
 * @return The number of edges in the polyhedron
 */
RP3D_FORCE_INLINE uint32 QHHalfEdgeStructure::getNbHalfEdges() const {
    return mNbHalfEdges;
}

// Return the number of vertices
/**
 * @return The number of vertices in the polyhedron
 */
RP3D_FORCE_INLINE uint32 QHHalfEdgeStructure::getNbVertices() const {
    return mNbVertices;
}

// Return a pointer to the first face in the linked-list of faces
RP3D_FORCE_INLINE const QHHalfEdgeStructure::Face* QHHalfEdgeStructure::getFaces() const {
   return mFaces;
}

// Return a pointer to the first half-edge in the linked-list of half-edges
RP3D_FORCE_INLINE const QHHalfEdgeStructure::Edge* QHHalfEdgeStructure::getHalfEdges() const {
   return mHalfEdges;
}

// Return a pointer to the first vertex in the linked-list of vertices
RP3D_FORCE_INLINE const QHHalfEdgeStructure::Vertex* QHHalfEdgeStructure::getVertices() const {
   return mVertices;
}

}

#endif

