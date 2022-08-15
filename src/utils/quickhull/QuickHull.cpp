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

// Libraries
#include <reactphysics3d/utils/quickhull/QuickHull.h>
#include <reactphysics3d/collision/PolyhedronMesh.h>
#include <reactphysics3d/collision/PolygonVertexArray.h>
#include <reactphysics3d/containers/Map.h>
#include <reactphysics3d/containers/Array.h>
#include <reactphysics3d/containers/Stack.h>
#include <reactphysics3d/containers/Set.h>
#include <iostream>

// Namespace
using namespace reactphysics3d;

// Compute the convex hull of a set of points and return the resulting polyhedron mesh
PolygonVertexArray* QuickHull::computeConvexHull(uint32 nbPoints, const void* pointsStart,
                                                 uint32 pointsStride,
                                                 PolygonVertexArray::VertexDataType pointDataType,
                                                 MemoryAllocator& allocator) {

    // TODO : Maybe create a PointsArray type to pass the array of points in parameter

    // Extract the points from the array
    Array<Vector3> points(allocator);
    extractPoints(nbPoints, pointsStart, pointsStride, pointDataType, points);
    Array<uint32> orphanPointsIndices(allocator, nbPoints);
    for (uint32 i=0 ; i < nbPoints; i++) {
        orphanPointsIndices.add(i);
    }


    QHHalfEdgeStructure convexHullHalfEdgeStructure(allocator);

    Array<QHHalfEdgeStructure::Face*> initialFaces(allocator);

    // Compute the initial convex hull
    computeInitialHull(points, convexHullHalfEdgeStructure, initialFaces, orphanPointsIndices, allocator);

    assert(convexHullHalfEdgeStructure.getNbVertices() == 4);
    assert(convexHullHalfEdgeStructure.getNbFaces() == 4);

    std::cout << " Initial hull: " << std::endl << convexHullHalfEdgeStructure.to_string() << std::endl;

    assert(convexHullHalfEdgeStructure.getNbVertices() == 4);

    // Associate all the remaining points with the closest faces of the initial hull
    associateOrphanPointsToNewFaces(orphanPointsIndices, initialFaces, points);

    // Get The next vertex candidate
    uint32 nextVertexIndex;
    QHHalfEdgeStructure::Face* nextFace;
    findNextVertexCandidate(points, nextVertexIndex, convexHullHalfEdgeStructure, nextFace);
    while (nextVertexIndex != INVALID_VERTEX_INDEX) {

        assert(nextFace != nullptr);

        std::cout << "Adding vertex " << nextVertexIndex << " to the hull" << std::endl;

        // Add the vertex to the hull
        addVertexToHull(nextVertexIndex, nextFace, points, convexHullHalfEdgeStructure, allocator);

        std::cout << " New hull: " << std::endl << convexHullHalfEdgeStructure.to_string() << std::endl;

        // Get the next vertex candidate
        findNextVertexCandidate(points, nextVertexIndex, convexHullHalfEdgeStructure, nextFace);
    }

    return nullptr;
}

// Add a vertex to the current convex hull to expand it
void QuickHull::addVertexToHull(uint32 vertexIndex,
                                QHHalfEdgeStructure::Face* face,
                                Array<Vector3>& points,
                                QHHalfEdgeStructure& convexHullHalfEdgeStructure,
                                MemoryAllocator& allocator) {

    Array<QHHalfEdgeStructure::Vertex*> horizonVertices(allocator);
    Array<QHHalfEdgeStructure::Face*> visibleFaces(allocator);
    Array<uint32> orphanPointsIndices(allocator);

    // Find the horizon edges
    findHorizon(points[vertexIndex], face, points, allocator, horizonVertices, visibleFaces);

    assert(visibleFaces.size() >= 1);
    assert(horizonVertices.size() >= 6);

    // TODO : Delete this
    std::cout << "---- Horizon Edges ----" << std::endl;
    for (uint32 i=0; i < horizonVertices.size(); i += 2) {
        const QHHalfEdgeStructure::Vertex* vertex1 = horizonVertices[i];
        const QHHalfEdgeStructure::Vertex* vertex2 = horizonVertices[i+1];
        const uint32 v1 = vertex1->externalIndex;
        const uint32 v2 = vertex2->externalIndex;
        std::cout << "Edge(" << v1 << ", " << v2 << ")" << std::endl;
    }

    assert(horizonVertices[0]->externalIndex == horizonVertices[horizonVertices.size()-1]->externalIndex);

    // Delete all the faces visible from the vertex
    deleteVisibleFaces(visibleFaces, convexHullHalfEdgeStructure, orphanPointsIndices, horizonVertices, allocator);

    // Build the new faces
    Array<QHHalfEdgeStructure::Face*> newFaces(allocator);
    buildNewFaces(vertexIndex, horizonVertices, convexHullHalfEdgeStructure, points, newFaces, allocator);

    associateOrphanPointsToNewFaces(orphanPointsIndices, newFaces, points);
}

// Build the new faces that contain the new vertex and the horizon edges
void QuickHull::buildNewFaces(uint32 newVertexIndex,
                              Array<QHHalfEdgeStructure::Vertex*>& horizonVertices,
                              QHHalfEdgeStructure& convexHullHalfEdgeStructure,
                              Array<Vector3>& points,
                              Array<QHHalfEdgeStructure::Face*>& newFaces,
                              MemoryAllocator& allocator) {

    // Add the new vertex to the convex hull
    QHHalfEdgeStructure::Vertex* v = convexHullHalfEdgeStructure.addVertex(newVertexIndex);

    Array<QHHalfEdgeStructure::Vertex*> faceVertices(allocator, 8);

    // For each horizon edge
    for (uint32 i=0; i < horizonVertices.size(); i += 2) {

        QHHalfEdgeStructure::Vertex* v2 = horizonVertices[i];
        QHHalfEdgeStructure::Vertex* v3 = horizonVertices[i+1];

        // Create the vertices of a triangular face
        faceVertices.add(v2);
        faceVertices.add(v3);
        faceVertices.add(v);

        // Compute the face normal
        Vector3 normal = (points[v3->externalIndex] - points[v2->externalIndex]).cross(points[v->externalIndex] - points[v3->externalIndex]);
        normal.normalize();

        // Create a new face
        QHHalfEdgeStructure::Face* face = convexHullHalfEdgeStructure.addFace(faceVertices, normal, allocator);

        newFaces.add(face);

        faceVertices.clear();
    }
}

// Delete all the faces visible from the vertex to be added
void QuickHull::deleteVisibleFaces(const Array<QHHalfEdgeStructure::Face*>& visibleFaces,
                                   QHHalfEdgeStructure& convexHullHalfEdgeStructure,
                                   Array<uint32>& orphanPoints,
                                   const Array<QHHalfEdgeStructure::Vertex*>& horizonVertices,
                                   MemoryAllocator& allocator) {

    const uint32 nbFaces = visibleFaces.size();

    Array<QHHalfEdgeStructure::Vertex*> verticesToRemove(allocator);

    // For each visible face
    for (uint32 i=0; i < nbFaces; i++) {

        // TODO : DELETE THIS
        std::cout << "Removing visible face with vertices " << visibleFaces[i]->verticesString() << std::endl;

        // Add all the remaining points associated with this face to the array of orphan points
        orphanPoints.addRange(visibleFaces[i]->remainingClosestPoints);

        // For each vertex of the face to remove
        const QHHalfEdgeStructure::Edge* firstFaceEdge = visibleFaces[i]->edge;
        const QHHalfEdgeStructure::Edge* faceEdge = firstFaceEdge;
        do {

           QHHalfEdgeStructure::Vertex* vertex = faceEdge->startVertex;

           // If the vertex is not part of the horizon
           if (!checkVertexInHorizon(vertex, horizonVertices)) {

               // Add the vertex to the array of vertices to be removed
               verticesToRemove.add(vertex);
           }

           faceEdge = faceEdge->nextFaceEdge;

        } while(faceEdge != firstFaceEdge);

        // Remove the face from the current convex hull
        convexHullHalfEdgeStructure.removeFace(visibleFaces[i]);
    }

    // Remove the vertices to be removed
    for (uint32 i=0; i < verticesToRemove.size(); i++) {
        convexHullHalfEdgeStructure.removeVertex(verticesToRemove[i]);
    }
}

// Return true if the vertex is part of horizon edges
bool QuickHull::checkVertexInHorizon(QHHalfEdgeStructure::Vertex* vertex, const Array<QHHalfEdgeStructure::Vertex*>& horizonVertices) {

    // For each edge of the horizon
    for (uint32 i=0; i < horizonVertices.size(); i += 2) {

        if (horizonVertices[i] == vertex) return true;
    }

    return false;
}

// Find the horizon (edges) forming the separation between the faces that are visible from the new vertex and the faces that are not visible
void QuickHull::findHorizon(const Vector3& vertex, QHHalfEdgeStructure::Face* face,
                            const Array<Vector3>& points, MemoryAllocator& allocator,
                            Array<QHHalfEdgeStructure::Vertex*>& outHorizonVertices,
                            Array<QHHalfEdgeStructure::Face*>& outVisibleFaces) {

    Stack<CandidateFace> facesToVisit(allocator);
    Set<const QHHalfEdgeStructure::Face*> visitedFaces(allocator);

    const QHHalfEdgeStructure::Edge* startEdge = face->edge;
    facesToVisit.push(CandidateFace(face, startEdge, startEdge));

    outVisibleFaces.add(face);

    // TODO : DELETE THIS
    std::cout << "Face visible from candidate vertex has vertices : " << face->verticesString() << std::endl;

    // While there still are faces to visit
    while (facesToVisit.size() > 0) {

        // Get the next face to process
        CandidateFace& candidateFace = facesToVisit.top();

        // Mark the current face as visited
        visitedFaces.add(candidateFace.face);

        // For each edge of the current face
        do {

            // Get the current edge to cross of the current face
            const QHHalfEdgeStructure::Edge* edge = candidateFace.currentEdge;
            assert(edge->face == candidateFace.face);

            const QHHalfEdgeStructure::Edge* twinEdge = edge->twinEdge;

            // Get the next face
            QHHalfEdgeStructure::Face* nextFace = twinEdge->face;

            // If the next face is not visited yet
            if (!visitedFaces.contains(nextFace)) {

                const Vector3& faceVertex = points[nextFace->getVertex()->externalIndex];

                // If the next face is visible from the vertex
                if (nextFace->normal.dot(vertex - faceVertex) > 0) {

                    // Add the next face to the stack of faces to visit
                    facesToVisit.push(CandidateFace(nextFace, nextFace->edge, nextFace->edge));

                    // TODO : DELETE THIS
                    std::cout << "Face visible from candidate vertex has vertices : " << nextFace->verticesString() << std::endl;

                    outVisibleFaces.add(nextFace);

                    // If the face is visible we move to visit this new visible face (Depth First Search)
                    break;
                }
                else {    // We have found a face that is not visible from the vertex

                    // We add the edge between current face and next face to the array of horizon edges
                    outHorizonVertices.add(candidateFace.currentEdge->startVertex);
                    outHorizonVertices.add(candidateFace.currentEdge->endVertex);
                }
            }

            // Move to the next edge
            candidateFace.currentEdge = edge->nextFaceEdge;

        } while (candidateFace.currentEdge != candidateFace.startEdge);

        // If we have visited all the edges of the current face
        if (candidateFace.currentEdge == candidateFace.startEdge) {

            // Remove the current face from the stack of faces to visit
            facesToVisit.pop();
        }
    }
}

// Return the index of the next vertex candidate to be added to the hull
// This method returns INVALID_VERTEX_INDEX if there is no more vertex candidate
void QuickHull::findNextVertexCandidate(Array<Vector3>& points, uint32& outNextVertexIndex,
                                        QHHalfEdgeStructure& convexHullHalfEdgeStructure,
                                        QHHalfEdgeStructure::Face*& outNextFace) {

    decimal maxDistance = 0;
    uint32 maxVertexI = INVALID_VERTEX_INDEX;
    outNextFace = nullptr;
    outNextVertexIndex = INVALID_VERTEX_INDEX;

    // For each face
    for (auto face = convexHullHalfEdgeStructure.getFaces(); face != nullptr; face = face->nextFace) {

        // If the face has remaining candidates points
        if (face->remainingClosestPoints.size() > 0) {

            std::cout << "Face with vertices " << face->verticesString() << " has candidates points" << std::endl;

            const Vector3& faceNormal = face->normal;
            Vector3& faceVertex = points[face->getVertex()->externalIndex];

            // For each remaining candidate point of the face
            for (uint32 i=0; i < face->remainingClosestPoints.size(); i++) {

                uint32 vertexIndex = face->remainingClosestPoints[i];

                const decimal distance = (points[vertexIndex] - faceVertex).dot(faceNormal);
                assert(distance > 0);

                if (distance > maxDistance) {
                    maxDistance = distance;
                    outNextVertexIndex = vertexIndex;
                    maxVertexI = i;
                    outNextFace = const_cast<QHHalfEdgeStructure::Face*>(face);
                }
            }
        }
    }

    std::cout << "Next vertex candidate: " << std::to_string(outNextVertexIndex) << " is furthest" << std::endl;

    // Remove the vertex from the array of remaining vertices for that face
    if (outNextFace != nullptr) {
        outNextFace->remainingClosestPoints.removeAt(maxVertexI);
    }

}

std::string QuickHull::showMap(Map<const QHHalfEdgeStructure::Face*, Array<uint32>>& mapFaceToRemainingClosestPoints) {

    std::string str = "Map: \n";
    for (auto it = mapFaceToRemainingClosestPoints.begin(); it != mapFaceToRemainingClosestPoints.end(); ++it) {
       str += "Face with vertices " + it->first->verticesString() + " => vertices(";

       // For each point associated to this face
       for (uint32 v = 0; v < it->second.size(); v++) {
          str += std::to_string(it->second[v]);

          if (v != it->second.size() - 1) {
            str += ',';
          }
       }

       str += ")\n";
    }

    return str;
}

// Take all the orphan points to the closest faces among the new faces
void QuickHull::associateOrphanPointsToNewFaces(Array<uint32>& orphanPointsIndices,
                                                Array<QHHalfEdgeStructure::Face*>& newFaces,
                                                Array<Vector3>& points) {

    // For each candidate points of the old faces
    for (uint32 i=0; i < orphanPointsIndices.size(); i++) {

        findClosestFaceForVertex(orphanPointsIndices[i], newFaces, points);
    }
}

// Find the closest face for a given vertex and add this vertex to the remaining closest points for this face
void QuickHull::findClosestFaceForVertex(uint32 vertexIndex, Array<QHHalfEdgeStructure::Face*>& faces, Array<Vector3>& points) {

    decimal minDistanceToFace = DECIMAL_LARGEST;
    QHHalfEdgeStructure::Face* closestFace = nullptr;

    // For each new face
    for (uint32 f=0; f < faces.size(); f++) {

        QHHalfEdgeStructure::Face* face = faces[f];
        const QHHalfEdgeStructure::Vertex* v1 = face->getVertex();

        const decimal distanceToFace = face->normal.dot(points[vertexIndex] - points[v1->externalIndex]);

        // If the point is in front the face and with a smaller distance from the face
        if (distanceToFace > 0 && distanceToFace < minDistanceToFace) {
            minDistanceToFace = distanceToFace;
            closestFace = face;
        }
    }

    // If we have found a closest face (where the point is in front of the face)
    if (closestFace != nullptr) {

        // Add the vertex to the remaining closest points of the face
        closestFace->remainingClosestPoints.add(vertexIndex);
    }
}

// Compute the initial tetrahedron convex hull
void QuickHull::computeInitialHull(Array<Vector3>& points, QHHalfEdgeStructure& convexHullHalfEdgeStructure,
                                   Array<QHHalfEdgeStructure::Face*>& initialFaces,
                                   Array<uint32>& orphanPointsIndices,
                                   MemoryAllocator& allocator) {

    // Find the extreme points on each X, Y and Z axes

    uint32 extremePointsIndices[6] = {0, 0, 0, 0, 0, 0};

    const uint32 nbPoints = points.size();
    for (uint32 i=0; i < nbPoints; i++) {

        if (points[i].x < points[extremePointsIndices[0]].x) {
            extremePointsIndices[0] = i;
        }
        else if (points[i].x > points[extremePointsIndices[1]].x) {
            extremePointsIndices[1] = i;
        }

        if (points[i].y < points[extremePointsIndices[2]].y) {
            extremePointsIndices[2] = i;
        }
        else if (points[i].y > points[extremePointsIndices[3]].y) {
            extremePointsIndices[3] = i;
        }

        if (points[i].z < points[extremePointsIndices[4]].z) {
            extremePointsIndices[4] = i;
        }
        else if (points[i].z > points[extremePointsIndices[5]].z) {
            extremePointsIndices[5] = i;
        }
    }

    // Find the two extreme points with largest distance

    decimal maxLargestDistSquare = 0;
    uint32 iMax = 0;
    for (uint32 i=0; i < 3; i++) {
        const decimal distSquare = (points[extremePointsIndices[i*2]] - points[extremePointsIndices[i*2+1]]).lengthSquare();
        if (distSquare > maxLargestDistSquare) {
            iMax = i;
            maxLargestDistSquare = distSquare;
        }
    }

    // The pair of points that have the largest distance between them
    uint32 i1 = extremePointsIndices[iMax * 2];
    uint32 i2 = extremePointsIndices[iMax * 2 + 1];

    // Find a third point that is the furthest from line (v1, v2)
    uint32 i3 = 0;
    maxLargestDistSquare = 0;
    Vector3 lineDirection = points[i2] - points[i1];
    lineDirection.normalize();
    for (uint32 i=0; i < nbPoints; i++) {

        Vector3 vec = points[i] - points[i1];
        const decimal distSquare = vec.cross(lineDirection).lengthSquare();

        if (distSquare > maxLargestDistSquare) {
           i3 = i;
           maxLargestDistSquare = distSquare;
        }
    }

    // Find a fourth point that has the largest distance with the v1, v2, v3 plane

    uint32 i4 = 0;
    maxLargestDistSquare = 0;
    Vector3 planeNormal = (points[i3] - points[i1]).cross(points[i2] - points[i1]);
    for (uint32 i=0; i < nbPoints; i++) {

        Vector3 vec = points[i] - points[i1];
        const decimal distSquare = std::abs(vec.dot(planeNormal));

        if (distSquare > maxLargestDistSquare) {
           i4 = i;
           maxLargestDistSquare = distSquare;
        }
    }

    // Test in which side of the triangle face v1,v2,v3 is the point v4
    // to know the orientation of the tetrahedron

    QHHalfEdgeStructure::Vertex* v0 = convexHullHalfEdgeStructure.addVertex(i1);
    QHHalfEdgeStructure::Vertex* v1 = convexHullHalfEdgeStructure.addVertex(i2);
    QHHalfEdgeStructure::Vertex* v2 = convexHullHalfEdgeStructure.addVertex(i3);
    QHHalfEdgeStructure::Vertex* v3 = convexHullHalfEdgeStructure.addVertex(i4);

    Array<QHHalfEdgeStructure::Vertex*> face1Vertices(allocator);
    Array<QHHalfEdgeStructure::Vertex*> face2Vertices(allocator);
    Array<QHHalfEdgeStructure::Vertex*> face3Vertices(allocator);
    Array<QHHalfEdgeStructure::Vertex*> face4Vertices(allocator);

    const Vector3 v1V4 = points[i4] - points[i1];

    QHHalfEdgeStructure::Face* face0 = nullptr;
    QHHalfEdgeStructure::Face* face1 = nullptr;
    QHHalfEdgeStructure::Face* face2 = nullptr;
    QHHalfEdgeStructure::Face* face3 = nullptr;

    // Create the tetrahedron faces (according to the orientation of the tetrahedron)
    if (planeNormal.dot(v1V4) < 0) {

        face1Vertices.add(v0); face1Vertices.add(v2); face1Vertices.add(v1);
        face2Vertices.add(v1); face2Vertices.add(v2); face2Vertices.add(v3);
        face3Vertices.add(v0); face3Vertices.add(v1); face3Vertices.add(v3);
        face4Vertices.add(v0); face4Vertices.add(v3); face4Vertices.add(v2);

        // Compute the face normal
        const Vector3 face0Normal = planeNormal.getUnit();
        const Vector3 face1Normal = (points[i3] - points[i2]).cross(points[i4] - points[i2]).getUnit();
        const Vector3 face2Normal = (points[i2] - points[i1]).cross(points[i4] - points[i1]).getUnit();
        const Vector3 face3Normal = (points[i4] - points[i1]).cross(points[i3] - points[i1]).getUnit();

        // Add vertices for each face
        face0 = convexHullHalfEdgeStructure.addFace(face1Vertices, face0Normal, allocator);
        face1 = convexHullHalfEdgeStructure.addFace(face2Vertices, face1Normal, allocator);
        face2 = convexHullHalfEdgeStructure.addFace(face3Vertices, face2Normal, allocator);
        face3 = convexHullHalfEdgeStructure.addFace(face4Vertices, face3Normal, allocator);
    }
    else {

        face1Vertices.add(v0); face1Vertices.add(v1); face1Vertices.add(v2);
        face2Vertices.add(v1); face2Vertices.add(v3); face2Vertices.add(v2);
        face3Vertices.add(v0); face3Vertices.add(v3); face3Vertices.add(v1);
        face4Vertices.add(v0); face4Vertices.add(v2); face4Vertices.add(v3);

        // Compute the face normal
        const Vector3 face0Normal = -planeNormal.getUnit();
        const Vector3 face1Normal = (points[i4] - points[i2]).cross(points[i3] - points[i2]).getUnit();
        const Vector3 face2Normal = (points[i4] - points[i1]).cross(points[i2] - points[i1]).getUnit();
        const Vector3 face3Normal = (points[i3] - points[i1]).cross(points[i4] - points[i1]).getUnit();

        // Add vertices for each face
        face0 = convexHullHalfEdgeStructure.addFace(face1Vertices, face0Normal, allocator);
        face1 = convexHullHalfEdgeStructure.addFace(face2Vertices, face1Normal, allocator);
        face2 = convexHullHalfEdgeStructure.addFace(face3Vertices, face2Normal, allocator);
        face3 = convexHullHalfEdgeStructure.addFace(face4Vertices, face3Normal, allocator);
    }

    initialFaces.add(face0);
    initialFaces.add(face1);
    initialFaces.add(face2);
    initialFaces.add(face3);

    orphanPointsIndices.remove(v0->externalIndex);
    orphanPointsIndices.remove(v1->externalIndex);
    orphanPointsIndices.remove(v2->externalIndex);
    orphanPointsIndices.remove(v3->externalIndex);
}

// Extract the points from the array
void QuickHull::extractPoints(uint32 nbPoints, const void* pointsStart, uint32 pointsStride,
                              PolygonVertexArray::VertexDataType pointDataType, Array<Vector3>& outArray)  {

    const unsigned char* pointsStartPointer = reinterpret_cast<const unsigned char*>(pointsStart);

    if (pointDataType == PolygonVertexArray::VertexDataType::VERTEX_FLOAT_TYPE) {
        for (uint32 p=0; p < nbPoints; p++) {
            const float* points = (float*)(pointsStartPointer + p * pointsStride);
            outArray.add(Vector3(points[0], points[1], points[2]));
        }
    }
    else if (pointDataType == PolygonVertexArray::VertexDataType::VERTEX_DOUBLE_TYPE) {
        for (uint32 p=0; p < nbPoints; p++) {
            const double* points = (double*)(pointsStartPointer + p * pointsStride);
            outArray.add(Vector3(points[0], points[1], points[2]));
        }
    }
    else {
        assert(false);
    }
}
