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
#include <reactphysics3d/collision/HalfEdgeStructure.h>
#include <reactphysics3d/containers/Map.h>
#include <reactphysics3d/containers/Pair.h>
#include <reactphysics3d/containers/containers_common.h>

using namespace reactphysics3d;

// Initialize the structure (when all vertices and faces have been added)
void HalfEdgeStructure::computeHalfEdges() {

    Map<VerticesPair, Edge> edges(mAllocator);
    Map<VerticesPair, VerticesPair> nextEdges(mAllocator);
    Map<VerticesPair, uint32> mapEdgeToStartVertex(mAllocator);
    Map<VerticesPair, uint32> mapEdgeToIndex(mAllocator);
    Map<uint32, VerticesPair> mapEdgeIndexToKey(mAllocator);
    Map<uint32, VerticesPair> mapFaceIndexToEdgeKey(mAllocator);

    Array<VerticesPair> currentFaceEdges(mAllocator, mFaces[0].faceVertices.size());

    // For each face
    const uint32 nbFaces = static_cast<uint32>(mFaces.size());
    for (uint32 f=0; f < nbFaces; f++) {

        Face& face = mFaces[f];

        VerticesPair firstEdgeKey(0, 0);

        // For each vertex of the face
        const uint32 nbFaceVertices = static_cast<uint32>(face.faceVertices.size());
        for (uint32 v=0; v < nbFaceVertices; v++) {
            uint32 v1Index = face.faceVertices[v];
            uint32 v2Index = face.faceVertices[v == (face.faceVertices.size() - 1) ? 0 : v + 1];

            const VerticesPair pairV1V2 = VerticesPair(v1Index, v2Index);

            // Create a new half-edge
            Edge edge;
            edge.faceIndex = f;
            edge.vertexIndex = v1Index;
            if (v == 0) {
                firstEdgeKey = pairV1V2;
            }
            else if (v >= 1) {
                nextEdges.add(Pair<VerticesPair, VerticesPair>(currentFaceEdges[currentFaceEdges.size() - 1], pairV1V2));
            }
            if (v == (face.faceVertices.size() - 1)) {
                nextEdges.add(Pair<VerticesPair, VerticesPair>(pairV1V2, firstEdgeKey));
            }
            edges.add(Pair<VerticesPair, Edge>(pairV1V2, edge));

            const VerticesPair pairV2V1(v2Index, v1Index);

            mapEdgeToStartVertex.add(Pair<VerticesPair, uint32>(pairV1V2, v1Index), true);
            mapEdgeToStartVertex.add(Pair<VerticesPair, uint32>(pairV2V1, v2Index), true);

            mapFaceIndexToEdgeKey.add(Pair<uint32, VerticesPair>(f, pairV1V2), true);

            auto itEdge = edges.find(pairV2V1);
            if (itEdge != edges.end()) {

                const uint32 edgeIndex = static_cast<uint32>(mEdges.size());

                itEdge->second.twinEdgeIndex = edgeIndex + 1;
                edge.twinEdgeIndex = edgeIndex;

                mapEdgeIndexToKey.add(Pair<uint32, VerticesPair>(edgeIndex, pairV2V1));
                mapEdgeIndexToKey.add(Pair<uint32, VerticesPair>(edgeIndex + 1, pairV1V2));

                mVertices[v1Index].edgeIndex = edgeIndex + 1;
                mVertices[v2Index].edgeIndex = edgeIndex;

                mapEdgeToIndex.add(Pair<VerticesPair, uint32>(pairV1V2, edgeIndex + 1));
                mapEdgeToIndex.add(Pair<VerticesPair, uint32>(pairV2V1, edgeIndex));

                mEdges.add(itEdge->second);
                mEdges.add(edge);
            }

            currentFaceEdges.add(pairV1V2);
        }

        currentFaceEdges.clear();
    }

    // Set next edges
    const uint32 nbEdges = static_cast<uint32>(mEdges.size());
    for (uint32 i=0; i < nbEdges; i++) {
        mEdges[i].nextEdgeIndex = mapEdgeToIndex[nextEdges[mapEdgeIndexToKey[i]]];
    }

    // Set face edge
    for (uint32 f=0; f < nbFaces; f++) {
        mFaces[f].edgeIndex = mapEdgeToIndex[mapFaceIndexToEdgeKey[f]];
    }
}

// Reserve some memory for vertices, faces and edges
void HalfEdgeStructure::reserve(uint32 facesCapacity, uint32 verticesCapacity, uint32 edgesCapacity) {

   mFaces.reserve(facesCapacity);
   mVertices.reserve(verticesCapacity);
   mEdges.reserve(edgesCapacity);

}

// Return a string representation of the half-edge structure
std::string HalfEdgeStructure::to_string() const {

    std::string faces = "Faces{";
    for (uint32 i=0; i < mFaces.size(); i++) {

       faces += "Face" + std::to_string(i) + "(";

       const Face& face = mFaces[i];

       for (uint32 v=0; v < face.faceVertices.size(); v++) {

           faces += std::to_string(mVertices[face.faceVertices[v]].vertexPointIndex);

           if (v < face.faceVertices.size() - 1) {
               faces += ",";
           }
       }

       faces += ")";

       if (i < mFaces.size() - 1) {
           faces += ",";
       }
    }

    faces += "}";

    std::string edges = "Edges{";
    for (uint32 i=0; i < mEdges.size(); i++) {

       edges += "Edge" + std::to_string(i) + "(";

       const Edge& edge = mEdges[i];
       const Edge& twinEdge = mEdges[edge.twinEdgeIndex];

       edges += std::to_string(mVertices[edge.vertexIndex].vertexPointIndex) + "," + std::to_string(mVertices[twinEdge.vertexIndex].vertexPointIndex) + ")";

       if (i < mEdges.size() - 1) {
           edges += ",";
       }
    }

    std::string vertices = "Vertices{";
    for (uint32 i=0; i < mVertices.size(); i++) {

       vertices += "Vertex" + std::to_string(i) + "(";

       const Vertex& vertex = mVertices[i];

       vertices += std::to_string(vertex.vertexPointIndex) + ")";

       if (i < mVertices.size() - 1) {
           vertices += ",";
       }
    }
    return "HalfEdgeStructure(" + faces + ",\n"  + edges + ",\n" + vertices + ")";
}
