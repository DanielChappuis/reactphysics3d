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

// Libraries
#include <reactphysics3d/utils/quickhull/QHHalfEdgeStructure.h>
#include <reactphysics3d/containers/Map.h>
#include <reactphysics3d/containers/Pair.h>
#include <reactphysics3d/containers/containers_common.h>

using namespace reactphysics3d;

// Make sure capacity is an integral multiple of alignment
const size_t QHHalfEdgeStructure::mVertexAllocatedSize = std::ceil(sizeof(Vertex) / float(GLOBAL_ALIGNMENT)) * GLOBAL_ALIGNMENT;
const size_t QHHalfEdgeStructure::mEdgeAllocatedSize = std::ceil(sizeof(Edge) / float(GLOBAL_ALIGNMENT)) * GLOBAL_ALIGNMENT;
const size_t QHHalfEdgeStructure::mFaceAllocatedSize = std::ceil(sizeof(Face) / float(GLOBAL_ALIGNMENT)) * GLOBAL_ALIGNMENT;


// Destructor
QHHalfEdgeStructure::~QHHalfEdgeStructure() {

    // Delete faces
    Face* face = mFaces;
    while (face != nullptr) {

        Face* nextFace = face->nextFace;

        face->~Face();
        mAllocator.release(face, mFaceAllocatedSize);

        face = nextFace;
    }

    // Delete edges
    Edge* edge = mHalfEdges;
    while (edge != nullptr) {

        Edge* nextEdge = edge->nextEdge;

        edge->~Edge();
        mAllocator.release(edge, mEdgeAllocatedSize);

        edge = nextEdge;
    }

    // Delete vertices
    Vertex* vertex = mVertices;
    while (vertex != nullptr) {

        Vertex* nextVertex = vertex->nextVertex;

        vertex->~Vertex();
        mAllocator.release(vertex, mVertexAllocatedSize);

        vertex = nextVertex;
    }
}

// Add a vertex
/**
 * @param externalIndex Index of the vertex in the external user vertex data array
 * @return A pointer to the new vertex
 */
QHHalfEdgeStructure::Vertex* QHHalfEdgeStructure::addVertex(uint32 externalIndex) {

    // Create a new vertex
    Vertex* vertex = new (mAllocator.allocate(mVertexAllocatedSize)) Vertex(externalIndex, nullptr, mVertices);

    if (mVertices != nullptr) {
       mVertices->previousVertex = vertex;
    }

    mVertices = vertex;

    mNbVertices++;

    return vertex;
}

// Add a face
/**
 * @param faceVertices Array of the vertices in a face (ordered in CCW order as seen from outside
 *                     the polyhedron). The indices are the internal indices of the vertices inside the HalfEdgeStructure.
 * @param points Array with the points (coordinates) of the face vertices
 * @param allocator Reference to a memory allocator
 * @return A Pointer to the new face
 */
QHHalfEdgeStructure::Face* QHHalfEdgeStructure::addFace(const Array<Vertex*>& faceVertices, const Array<Vector3>& points,
                                                        MemoryAllocator& allocator) {

    assert(faceVertices.size() >= 3);

    // Create a new face
    Face* face = new (mAllocator.allocate(mFaceAllocatedSize)) Face(allocator);

    Edge* prevFaceEdge = nullptr;
    Edge* firstFaceEdge = nullptr;

    // For each edge of the face
    for (uint32 i=0; i < faceVertices.size(); i++) {

        Vertex* v1 = faceVertices[i];
        Vertex* v2 = faceVertices[i == faceVertices.size() - 1 ? 0 : i+1];

        assert(!mMapVerticesToEdge.containsKey(EdgeVertices(v1, v2)));

        // Create an edge
        Edge* edge = new (mAllocator.allocate(mEdgeAllocatedSize)) Edge(v1, v2, face);
        edge->previousFaceEdge = prevFaceEdge;

        // Get twin edge (if any)
        Edge* twinEdge = nullptr;
        auto it = mMapVerticesToEdge.find(EdgeVertices(v2, v1));
        if (it != mMapVerticesToEdge.end()) {
           twinEdge = it->second;
           assert(twinEdge->twinEdge == nullptr);
           twinEdge->twinEdge = edge;
        }
        edge->twinEdge = twinEdge;

        if (prevFaceEdge != nullptr) {
            prevFaceEdge->nextFaceEdge = edge;
        }

        if (i == 0) {
            firstFaceEdge = edge;
        }

        // Add the new edge before the corresponding twin edge in the linked-list of edges (or at the beginning if there is no twin edge)
        addEdgeToLinkedListBefore(edge, twinEdge);

        mNbHalfEdges++;

        // Add the edge into the map
        assert(!mMapVerticesToEdge.containsKey(EdgeVertices(v1, v2)));
        mMapVerticesToEdge.add(Pair<EdgeVertices, Edge*>(EdgeVertices(v1, v2), edge));

        assert(twinEdge != nullptr || mHalfEdges == edge);
        assert(edge->face == face);

        prevFaceEdge = edge;
    }

    prevFaceEdge->nextFaceEdge = firstFaceEdge;
    firstFaceEdge->previousFaceEdge = prevFaceEdge;

    face->edge = firstFaceEdge;

    addFaceToLinkedList(face);

    mNbFaces++;

    // Compute the normal, area and centroid
    face->recalculateFace(points);

    return face;
}

// Add an edge before another one in the linked-list of edges
void QHHalfEdgeStructure::addEdgeToLinkedListBefore(Edge* newEdge, Edge* edge) {

    assert(newEdge != nullptr);

    // If we need to add the edge at the beginning instead
    if (edge == nullptr) {
       newEdge->nextEdge = mHalfEdges;
       newEdge->previousEdge = nullptr;
       if (mHalfEdges != nullptr) {
           mHalfEdges->previousEdge = newEdge;
       }
       mHalfEdges = newEdge;
    }
    else {  // If we need to add the newEdge before the edge

        if (edge->previousEdge != nullptr) {
            assert(edge->previousEdge->nextEdge == edge);
            edge->previousEdge->nextEdge = newEdge;
        }
        else { // If the new edge must be the first one in the linked-list
           assert(mHalfEdges == edge);
           mHalfEdges = newEdge;
        }

        newEdge->previousEdge = edge->previousEdge;
        edge->previousEdge = newEdge;
        newEdge->nextEdge = edge;
    }

    assert(edge == nullptr || edge->previousEdge == newEdge);
    assert(mHalfEdges == newEdge || newEdge->previousEdge != nullptr);
    assert(mHalfEdges == newEdge || newEdge->previousEdge->nextEdge == newEdge);
    assert(newEdge->nextEdge == nullptr || newEdge->nextEdge->previousEdge == newEdge);
}

// Remove an half-edge from the linked-list of half-edges
void QHHalfEdgeStructure::removeEdgeFromLinkedList(Edge* edge) {

    if (edge->nextEdge != nullptr) {
        edge->nextEdge->previousEdge = edge->previousEdge;
    }
    if (edge->previousEdge != nullptr) {
        edge->previousEdge->nextEdge = edge->nextEdge;
    }
    else {
        assert(mHalfEdges == edge);
        mHalfEdges = edge->nextEdge;
        assert(mHalfEdges == nullptr || mHalfEdges->previousEdge == nullptr);
    }
}

// Add a face to the linked-list of faces
void QHHalfEdgeStructure::addFaceToLinkedList(Face* face) {

    if (mFaces != nullptr) {
        mFaces->previousFace = face;
    }
    face->nextFace = mFaces;

    mFaces = face;
}

// Remove a face from the linked-list of faces
void QHHalfEdgeStructure::removeFaceFromLinkedList(Face* face) {

    if (face->nextFace != nullptr) {
        face->nextFace->previousFace = face->previousFace;
    }
    if (face->previousFace != nullptr) {
        face->previousFace->nextFace = face->nextFace;
    }
    else {
        assert(mFaces == face);
        mFaces = face->nextFace;
    }
}

// Remove an half-edge
void QHHalfEdgeStructure::removeHalfEdge(Edge* edge) {

    assert(mMapVerticesToEdge.containsKey(EdgeVertices(edge->startVertex, edge->endVertex)));
    mMapVerticesToEdge.remove(EdgeVertices(edge->startVertex, edge->endVertex));

    if (edge->twinEdge != nullptr) {
        edge->twinEdge->twinEdge = nullptr;
    }

    removeEdgeFromLinkedList(edge);

    edge->~Edge();
    mAllocator.release(edge, mEdgeAllocatedSize);

    mNbHalfEdges--;
}

// Remove a face and all its edges
void QHHalfEdgeStructure::removeFace(Face* face) {

    // Remove all half-edges of the face
    Edge* firstEdge = face->edge;
    Edge* edge = firstEdge;
    do {

        Edge* nextFaceEdge = edge->nextFaceEdge;

        removeHalfEdge(edge);

        edge = nextFaceEdge;

    } while(edge != firstEdge);

    deleteFace(face);
}

// Delete the face
void QHHalfEdgeStructure::deleteFace(Face* face) {

    // Remove the face
    removeFaceFromLinkedList(face);

    face->~Face();
    mAllocator.release(face, mFaceAllocatedSize);

    mNbFaces--;
}

// Remove a vertex
void QHHalfEdgeStructure::removeVertex(Vertex* vertex) {

    if (vertex->previousVertex != nullptr) {
       vertex->previousVertex->nextVertex = vertex->nextVertex;
    }
    if (vertex->nextVertex != nullptr) {
       vertex->nextVertex->previousVertex = vertex->previousVertex;
    }

    if (mVertices == vertex) {
        mVertices = vertex->nextVertex;
    }

    vertex->~Vertex();
    mAllocator.release(vertex, mVertexAllocatedSize);

    mNbVertices--;
}

// Return true if the half-edge structure is valid (for debugging purpose)
bool QHHalfEdgeStructure::isValid() const {

    bool isValid = true;

    // For each face
    uint32 nbFaces = 0;
    Face* face;
    Face* previousFace;
    for (face = mFaces, previousFace = nullptr; face != nullptr; previousFace = face, face = face->nextFace) {

       isValid &= face->edge != nullptr;
       isValid &= (face->previousFace != nullptr || nbFaces == 0);
       isValid &= (face->previousFace == nullptr || face->previousFace == previousFace);
       isValid &= (previousFace == nullptr || previousFace->nextFace == face);
       isValid &= face->isValid();
       isValid &= face->area > 0.00001;
       nbFaces++;
    };
    isValid &= nbFaces > 0 || mFaces == nullptr;

    // For each vertex
    uint32 nbVertices = 0;
    Vertex* vertex;
    Vertex* previousVertex;
    for (vertex = mVertices, previousVertex = nullptr; vertex != nullptr; previousVertex = vertex, vertex = vertex->nextVertex) {

       isValid &= (vertex->previousVertex != nullptr || nbVertices == 0);
       isValid &= (vertex->previousVertex == nullptr || vertex->previousVertex == previousVertex);
       isValid &= (previousVertex == nullptr || previousVertex->nextVertex == vertex);
       nbVertices++;
    };
    isValid &= nbVertices == mNbVertices;
    isValid &= (nbVertices > 0 || mVertices == nullptr);

    // For each half-edge
    uint32 nbEdges = 0;
    Edge* edge;
    Edge* previousEdge;
    for (edge = mHalfEdges, previousEdge = nullptr; edge != nullptr; previousEdge = edge, edge = edge->nextEdge) {

       isValid &= (edge->previousEdge != nullptr || nbEdges == 0);
       isValid &= (edge->previousEdge == nullptr || edge->previousEdge == previousEdge);
       isValid &= (previousEdge == nullptr || previousEdge->nextEdge == edge);
       isValid &= edge->isValid();
       nbEdges++;
    };

    isValid &= nbEdges == mNbHalfEdges;
    isValid &= mNbHalfEdges % 2 == 0;
    isValid &= (nbEdges > 0 || mHalfEdges == nullptr);

    return isValid;
}

// Return a string representation of the half-edge structure
std::string QHHalfEdgeStructure::to_string() const {

    std::string faces = "Faces{";
    for (const Face* face = mFaces; face != nullptr; face = face->nextFace) {

       faces += "Face =" + face->verticesString();

       if (face->nextFace != nullptr) {
           faces += ",";
       }
    }

    faces += "}";

    std::string edges = "Edges{";
    for (const Edge* edge = mHalfEdges; edge != nullptr; edge = edge->nextEdge) {

       edges += "Edge(";

       edges += std::to_string(edge->startVertex->externalIndex) + "," + std::to_string(edge->endVertex->externalIndex) + ")";

       if (edge->nextEdge != nullptr) {
           edges += ",";
       }
    }

    std::string vertices = "Vertices{";
    for (const Vertex* vertex = mVertices; vertex != nullptr; vertex = vertex->nextVertex) {

       vertices += std::to_string(vertex->externalIndex);

       if (vertex->nextVertex != nullptr) {
           vertices += ",";
       }
    }
    return "HalfEdgeStructure(" + faces + ",\n"  + edges + ",\n" + vertices + ")";
}
