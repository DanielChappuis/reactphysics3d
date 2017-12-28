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

// Initialize the structure (when all vertices and faces have been added)
void HalfEdgeStructure::init() {

    using edgeKey = std::pair<uint, uint>;

    std::map<edgeKey, Edge> edges;
    std::map<edgeKey, edgeKey> nextEdges;
    std::map<edgeKey, uint> mapEdgeToStartVertex;
    std::map<edgeKey, uint> mapEdgeToIndex;
    std::map<uint, edgeKey> mapEdgeIndexToKey;
    std::map<uint, edgeKey> mapFaceIndexToEdgeKey;

    // For each face
    for (uint f=0; f<mFaces.size(); f++) {

        Face face = mFaces[f];

        std::vector<edgeKey> currentFaceEdges;

        edgeKey firstEdgeKey;

        // For each vertex of the face
        for (uint v=0; v < face.faceVertices.size(); v++) {
            uint v1Index = face.faceVertices[v];
            uint v2Index = face.faceVertices[v == (face.faceVertices.size() - 1) ? 0 : v + 1];

            const edgeKey pairV1V2 = std::make_pair(v1Index, v2Index);

            // Create a new half-edge
            Edge edge;
            edge.faceIndex = f;
            edge.vertexIndex = v1Index;
            if (v == 0) {
                firstEdgeKey = pairV1V2;
            }
            else if (v >= 1) {
                nextEdges.insert(std::make_pair(currentFaceEdges[currentFaceEdges.size() - 1], pairV1V2));
            }
            if (v == (face.faceVertices.size() - 1)) {
                nextEdges.insert(std::make_pair(pairV1V2, firstEdgeKey));
            }
            edges.insert(std::make_pair(pairV1V2, edge));

            const edgeKey pairV2V1 = std::make_pair(v2Index, v1Index);

            mapEdgeToStartVertex.insert(std::make_pair(pairV1V2, v1Index));
            mapEdgeToStartVertex.insert(std::make_pair(pairV2V1, v2Index));

            mapFaceIndexToEdgeKey.insert(std::make_pair(f, pairV1V2));

            auto itEdge = edges.find(pairV2V1);
            if (itEdge != edges.end()) {

                const uint edgeIndex = mEdges.size();

                itEdge->second.twinEdgeIndex = edgeIndex + 1;
                edge.twinEdgeIndex = edgeIndex;

                mapEdgeIndexToKey[edgeIndex] = pairV2V1;
                mapEdgeIndexToKey[edgeIndex + 1] = pairV1V2;

                mVertices[v1Index].edgeIndex = edgeIndex + 1;
                mVertices[v2Index].edgeIndex = edgeIndex;

                mapEdgeToIndex.insert(std::make_pair(pairV1V2, edgeIndex + 1));
                mapEdgeToIndex.insert(std::make_pair(pairV2V1, edgeIndex));

                mEdges.add(itEdge->second);
                mEdges.add(edge);
            }

            currentFaceEdges.push_back(pairV1V2);
        }
    }

    // Set next edges
    for (uint i=0; i < mEdges.size(); i++) {
        mEdges[i].nextEdgeIndex = mapEdgeToIndex[nextEdges[mapEdgeIndexToKey[i]]];
    }

    // Set face edge
    for (uint f=0; f < mFaces.size(); f++) {
        mFaces[f].edgeIndex = mapEdgeToIndex[mapFaceIndexToEdgeKey[f]];
    }
}
