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

#ifndef REACTPHYSICS3D_QH_HALF_EDGE_STRUCTURE_MESH_H
#define REACTPHYSICS3D_QH_HALF_EDGE_STRUCTURE_MESH_H

// Libraries
#include <reactphysics3d/mathematics/mathematics.h>
#include <reactphysics3d/containers/Map.h>
#include <reactphysics3d/containers/Set.h>

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

        // Struct Edge
        /**
         * An half-edge
         */
        struct Edge {

            /// Vertex at the beginning of the edge
            Vertex* startVertex;

            /// Vertex at the end of the edge
            Vertex* endVertex;

            /// Adjacent face of the edge
            Face* face;

            /// Previous edge in the linked-list of edges
            Edge* previousEdge;

            /// Next edge in the linked-list of edges
            Edge* nextEdge;

            /// Previous edge around the face of the edge
            Edge* previousFaceEdge;

            /// Next edge around the face of the edge
            Edge* nextFaceEdge;

            /// Twin edge
            Edge* twinEdge;

            Edge(Vertex* startVertex, Vertex* endVertex, Face* face)
                :startVertex(startVertex), endVertex(endVertex), face(face), previousEdge(nullptr), nextEdge(nullptr),
                 previousFaceEdge(nullptr), nextFaceEdge(nullptr), twinEdge(nullptr) {}

            bool isValid() const {

                bool isValid = true;

                isValid &= face != nullptr;
                isValid &= previousFaceEdge != nullptr;
                isValid &= nextFaceEdge != nullptr;
                isValid &= previousFaceEdge->nextFaceEdge == this;
                isValid &= nextFaceEdge->previousFaceEdge == this;
                isValid &= twinEdge != nullptr;
                isValid &= twinEdge->twinEdge == this;
                isValid &= startVertex == twinEdge->endVertex;
                isValid &= endVertex == twinEdge->startVertex;
                isValid &= endVertex == nextFaceEdge->startVertex;
                isValid &= startVertex == previousFaceEdge->endVertex;

                return isValid;
            }
        };

        // Struct Face
        /**
         * A face
         */
        struct Face {

            /// Pointer to the next face
            Face* nextFace;

            /// Pointer to the previous face
            Face* previousFace;

            /// One half-edge of the face
            Edge* edge;

            /// Face normal
            Vector3 normal;

            /// Center of the face (average of the face vertices)
            Vector3 centroid;

            /// Area of the face
            decimal area;

            /// Array with some remaining points visible from this face that need to be processed
            Array<uint32> conflictPoints;

            /// Constructor
            Face(MemoryAllocator& allocator)
                : nextFace(nullptr), previousFace(nullptr), edge(nullptr), normal(0, 0, 0), area(0), conflictPoints(allocator, 8) {

            }

            /// Return a vertex of the face
            const Vertex* getVertex() const {
                return edge->startVertex;
            }

            /// Recalculate the face centroid and normal to better fit its new vertices (using Newell method)
            void recalculateFace(const Array<Vector3>& points) {

                centroid.setToZero();
                normal.setToZero();
                uint32 nbVertices = 0;

                // For each vertex of the face
                const QHHalfEdgeStructure::Edge* firstFaceEdge = edge;
                const QHHalfEdgeStructure::Edge* faceEdge = firstFaceEdge;
                do {

                    const Vector3 v1 = points[faceEdge->startVertex->externalIndex];
                    const Vector3 v2 = points[faceEdge->endVertex->externalIndex];
                    centroid += v1;
                    normal += Vector3((v1.y - v2.y) * (v1.z + v2.z),
                                      (v1.z - v2.z) * (v1.x + v2.x),
                                      (v1.x - v2.x) * (v1.y + v2.y));

                    nbVertices++;

                   faceEdge = faceEdge->nextFaceEdge;

                } while(faceEdge != firstFaceEdge);

                assert(nbVertices > 0);

                centroid = centroid / nbVertices;
                const decimal normalLength = normal.length();
                assert(normalLength > 0);
                normal = normal / normalLength;
                area = normalLength * decimal(0.5);
            }

            /// Return a string with the vertices of the face
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

            /// Return true if the face is a triangle
            bool isTriangle() {

                return edge->nextFaceEdge->nextFaceEdge->nextFaceEdge == edge;
            }

            /// Return true if the face structure is valid (for debugging purpose)
            bool isValid() {
               bool isValid = true;

               isValid &= approxEqual(normal.lengthSquare(), 1.0, 0.01);
               isValid &= edge->face == this;

                const QHHalfEdgeStructure::Edge* firstFaceEdge = edge;
                const QHHalfEdgeStructure::Edge* faceEdge = firstFaceEdge;
                do {

                   if (faceEdge->face != this) {
                       return false;
                   }

                   faceEdge = faceEdge->nextFaceEdge;

                } while(faceEdge != firstFaceEdge);


               return isValid;
            }

        };

        // Struct Vertex
        /**
         * A vertex
         */
        struct Vertex {

            /// Index of the vertex point in the user vertex array
            uint32 externalIndex;

            /// Pointer to the previous vertex
            Vertex* previousVertex;

            /// Pointer to the next vertex
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

        static const size_t mVertexAllocatedSize;
        static const size_t mEdgeAllocatedSize;
        static const size_t mFaceAllocatedSize;

        // ---------- Methods ---------- //

        /// Add an edge before another one in the linked-list of edges
        void addEdgeToLinkedListBefore(Edge* newEdge, Edge* edge);

        /// Remove an half-edge from the linked-list of half-edges
        void removeEdgeFromLinkedList(Edge* edge);

        /// Add a face to the linked-list of faces
        void addFaceToLinkedList(Face* face);

        /// Remove a face from the linked-list of faces
        void removeFaceFromLinkedList(Face* face);

    public:

        // ---------- Methods ---------- //

        /// Constructor
        QHHalfEdgeStructure(MemoryAllocator& allocator) :mAllocator(allocator), mMapVerticesToEdge(allocator) {}

        /// Destructor
        ~QHHalfEdgeStructure();

        /// Add a vertex
        Vertex* addVertex(uint32 externalIndex);

        /// Add a face
        Face* addFace(const Array<Vertex*>& faceVertices, const Array<Vector3>& points, MemoryAllocator& allocator);

        /// Remove a face
        void removeFace(Face* face);

        /// Delete the face
        void deleteFace(Face* face);

        /// Remove an half-edge
        void removeHalfEdge(Edge* edge);

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

