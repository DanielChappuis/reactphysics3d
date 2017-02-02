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

// Libraries
#include "HalfEdgeStructure.h"
#include <map>

using namespace reactphysics3d;

// Initialize the structure
void HalfEdgeStructure::init(std::vector<const Vector3> vertices, std::vector<std::vector<uint>> faces) {

    using edgeKey = std::pair<uint, uint>;

    std::map<edgeKey, Edge> edges;
    std::map<edgeKey, edgeKey> nextEdges;
    std::map<edgeKey, uint> mapEdgeToStartVertex;

    // For each vertices
    for (uint v=0; v<vertices.size(); v++) {
        Vertex vertex(vertices[v]);
        mVertices.push_back(vertex);
    }

    // For each face
    for (uint f=0; f<faces.size(); f++) {

        // Create a new face
        Face face;
        mFaces.push_back(face);

        // Vertices of the current face
        std::vector<uint>& faceVertices = faces[f];

        std::vector<edgeKey> currentFaceEdges;

        edgeKey firstEdgeKey;

        // For each edge of the face
        for (uint e=0; e < faceVertices.size(); e++) {
            uint v1Index = faceVertices[e];
            uint v2Index = faceVertices[e == (faceVertices.size() - 1) ? 0 : e + 1];

            const edgeKey pairV1V2 = std::make_pair(v1Index, v2Index);

            // Create a new half-edge
            Edge edge;
            edge.faceIndex = f;
            edge.vertexIndex = v1Index;
            if (e == 0) {
                firstEdgeKey = pairV1V2;
            }
            else if (e >= 1) {
                nextEdges.insert(currentFaceEdges[currentFaceEdges.size() - 1], pairV1V2);
            }
            if (e == (faceVertices.size() - 1)) {
                nextEdges.insert(pairV1V2, firstEdgeKey);
            }
            edges.insert(pairV1V2, edge);

            const edgeKey pairV2V1 = std::make_pair(v2Index, v1Index);

            mapEdgeToStartVertex.insert(pairV1V2, v1Index);
            mapEdgeToStartVertex.insert(pairV2V1, v2Index);

            auto itEdge = edges.find(pairV2V1);
            if (itEdge != edges.end()) {

                const uint edgeIndex = mEdges.size();
                mEdges.push_back(itEdge->second);
                mEdges.push_back(edge);

                itEdge->second.twinEdgeIndex = edgeIndex + 1;
                itEdge->second.nextEdgeIndex = nextEdges[pairV2V1];

                edge.twinEdgeIndex = edgeIndex;
                edge.nextEdgeIndex = edges[nextEdges[pairV1V2]].;

                mVertices[v1Index].edgeIndex = edgeIndex;
                mVertices[v2Index].edgeIndex = edgeIndex + 1;

                face.edgeIndex = edgeIndex + 1;
            }

            currentFaceEdges.push_back(pairV1V2);
        }

        // For each edge of the face
        for (uint e=0; e < currentFaceEdges.size(); e++) {
            Edge& edge = currentFaceEdges[e];
            edge.nextEdgeIndex =
        }
    }
}
