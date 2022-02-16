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
#include <reactphysics3d/utils/QuickHull.h>
#include <reactphysics3d/collision/PolyhedronMesh.h>
#include <reactphysics3d/collision/PolygonVertexArray.h>
#include <reactphysics3d/containers/Map.h>
#include <reactphysics3d/containers/Array.h>
#include <iostream>

// Namespace
using namespace reactphysics3d;

// Compute the convex hull of a set of points and return the resulting polyhedron mesh
PolygonVertexArray* QuickHull::computeConvexHull(uint32 nbPoints, const void* pointsStart, uint32 pointsStride,
                                                 PolygonVertexArray::VertexDataType pointDataType, MemoryAllocator& allocator) {

    // TODO : Maybe create a PointsArray type to pass the array of points in parameter

    // Extract the points from the array
    Array<Vector3> points(allocator);
    extractPoints(nbPoints, pointsStart, pointsStride, pointDataType, points);

    // Indices of the points that still need to be processed
    Map<uint32, Array<uint32>> mapFaceIndexToRemainingClosestPoints(allocator);

    HalfEdgeStructure convexHullHalfEdgeStructure(allocator, nbPoints, nbPoints, nbPoints);

    Map<uint32, Vector3> mapFaceIndexToNormal(allocator);

    // Compute the initial convex hull
    computeInitialHull(points, convexHullHalfEdgeStructure, mapFaceIndexToNormal, allocator);

    assert(convexHullHalfEdgeStructure.getNbVertices() == 4);
    assert(convexHullHalfEdgeStructure.getNbFaces() == 4);

    std::cout << " Initial hull: " << std::endl << convexHullHalfEdgeStructure.to_string() << std::endl;

    mapFaceIndexToRemainingClosestPoints.add(Pair<uint32, Array<uint32>>(0, Array<uint32>(allocator)));
    mapFaceIndexToRemainingClosestPoints.add(Pair<uint32, Array<uint32>>(1, Array<uint32>(allocator)));
    mapFaceIndexToRemainingClosestPoints.add(Pair<uint32, Array<uint32>>(2, Array<uint32>(allocator)));

    // Associate all the remaining points with the closest faces of the initial hull
    Array<uint32> initialFacesIndices(allocator);
    initialFacesIndices.add(0);
    initialFacesIndices.add(1);
    initialFacesIndices.add(2);
    associateOldFacePointsToNewFaces(INVALID_FACE_INDEX, initialFacesIndices, points, convexHullHalfEdgeStructure, mapFaceIndexToNormal,
                                     mapFaceIndexToRemainingClosestPoints);

    // TODO : DELETE THIS
    std::cout << showMap(mapFaceIndexToRemainingClosestPoints) << std::endl;

    // Get The next vertex candidate
    uint32 vertexIndex = findNextVertexCandidate(mapFaceIndexToRemainingClosestPoints, mapFaceIndexToNormal, convexHullHalfEdgeStructure, points);
    while (vertexIndex != INVALID_VERTEX_INDEX) {

        std::cout << "Adding vertex " << vertexIndex << " to the hull" << std::endl;

        // Add the vertex to the hull
        addVertexToHull(vertexIndex, points, convexHullHalfEdgeStructure, allocator);

        std::cout << " New hull: " << std::endl << convexHullHalfEdgeStructure.to_string() << std::endl;

        // Get the next vertex candidate
        vertexIndex = findNextVertexCandidate(mapFaceIndexToRemainingClosestPoints, mapFaceIndexToNormal, convexHullHalfEdgeStructure, points);
    }

    return nullptr;
}

// Add a vertex to the current convex hull to expand it
void QuickHull::addVertexToHull(uint32 vertexIndex, Array<Vector3>& points, HalfEdgeStructure& convexHullHalfEdgeStructure, MemoryAllocator& allocator) {

    /*
    associateOldFacePointsToNewFaces(oldFaceIndex, newFacesIndices, points, convexHullHalfEdgeStructure,
                                     mapFaceIndexToRemainingClosestPoints, allocator);
    */
}

// Return the index of the next vertex candidate to be added to the hull
// This method returns INVALID_VERTEX_INDEX if there is no more vertex candidate
uint32 QuickHull::findNextVertexCandidate(Map<uint32, Array<uint32>>& mapFaceIndexToRemainingClosestPoints, Map<uint32, Vector3>& mapFaceIndexToNormal,
                                          HalfEdgeStructure& convexHullHalfEdgeStructure, Array<Vector3>& points) {

    // Find a face with remaining candidates points
    for (auto it = mapFaceIndexToRemainingClosestPoints.begin(); it != mapFaceIndexToRemainingClosestPoints.end(); ++it) {

        // If the face has remaining candidates points
        if (it->second.size() > 0) {

            std::cout << "Face " << std::to_string(it->first) << " has candidates points" << std::endl;

            Vector3& faceNormal = mapFaceIndexToNormal[it->first];
            Vector3& faceVertex = points[convexHullHalfEdgeStructure.getVertex(convexHullHalfEdgeStructure.getFace(it->first).faceVertices[0]).vertexPointIndex];

            decimal maxDistance = 0;
            uint32 maxVertexIndex = INVALID_VERTEX_INDEX;

            // For each remaining candidate point of the face
            for (uint32 i=0; i < it->second.size(); i++) {

                uint32 vertexIndex = it->second[i];

                const decimal distance = (points[vertexIndex] - faceVertex).dot(faceNormal);
                assert(distance >= 0);

                if (distance > maxDistance) {
                    maxDistance = distance;
                    maxVertexIndex = vertexIndex;
                }
            }

            assert(maxVertexIndex != INVALID_FACE_INDEX);

            std::cout << "Next vertex candidate: " << std::to_string(maxVertexIndex) << " is furthest" << std::endl;

            return maxVertexIndex;
        }
    }

    return INVALID_VERTEX_INDEX;
}

std::string QuickHull::showMap(Map<uint32, Array<uint32>>& mapFaceIndexToRemainingClosestPoints) {

    std::string str = "Map: \n";
    for (auto it = mapFaceIndexToRemainingClosestPoints.begin(); it != mapFaceIndexToRemainingClosestPoints.end(); ++it) {
       str += "Face " + std::to_string(it->first) + " => vertices(";

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

// Take all the points closest to the old face and add them to the closest faces among the
// new faces that replace the old face
void QuickHull::associateOldFacePointsToNewFaces(uint32 oldFaceIndex, Array<uint32>& newFacesIndices, Array<Vector3>& points,
                                          HalfEdgeStructure& convexHullHalfEdgeStructure,
                                          Map<uint32, Vector3> &mapFaceIndexToNormal,
                                          Map<uint32, Array<uint32>>& mapFaceIndexToRemainingClosestPoints) {

    // Only first time for initial hull
    if (oldFaceIndex == INVALID_FACE_INDEX) {

        // For each point
        const uint32 nbPoints = points.size();
        for (uint32 i=0; i < nbPoints; i++) {

            // Except the points that are already part of the initial convex hull
            if (i != convexHullHalfEdgeStructure.getVertex(0).vertexPointIndex &&
                i != convexHullHalfEdgeStructure.getVertex(1).vertexPointIndex &&
                i != convexHullHalfEdgeStructure.getVertex(2).vertexPointIndex &&
                i != convexHullHalfEdgeStructure.getVertex(3).vertexPointIndex) {

                findClosestFaceForVertex(i, newFacesIndices, points, convexHullHalfEdgeStructure, mapFaceIndexToNormal, mapFaceIndexToRemainingClosestPoints);
            }
        }
    }
    else {

        // For each candidate points of the old face
        for (uint32 i=0; i < mapFaceIndexToRemainingClosestPoints[oldFaceIndex].size(); i++) {

            const uint32 vertexIndex = mapFaceIndexToRemainingClosestPoints[oldFaceIndex][i];
            findClosestFaceForVertex(vertexIndex, newFacesIndices, points, convexHullHalfEdgeStructure, mapFaceIndexToNormal, mapFaceIndexToRemainingClosestPoints);
        }
    }
}

// Find the closest face for a given vertex and add this vertex to the remaining closest points for this face
void QuickHull::findClosestFaceForVertex(uint32 vertexIndex, Array<uint32>& facesIndices, Array<Vector3>& points,
                                                HalfEdgeStructure& convexHullHalfEdgeStructure,
                                                Map<uint32, Vector3>& mapFaceIndexToNormal,
                                                Map<uint32, Array<uint32>>& mapFaceIndexToRemainingClosestPoints) {

    decimal minDistanceToFace = DECIMAL_LARGEST;
    uint32 closestFaceIndex = INVALID_FACE_INDEX;

    // For each new face
    for (uint32 f=0; f < facesIndices.size(); f++) {

        const uint32 faceIndex = facesIndices[f];
        const HalfEdgeStructure::Face& face = convexHullHalfEdgeStructure.getFace(faceIndex);
        const uint32 v1Index = face.faceVertices[0];

        const HalfEdgeStructure::Vertex& v1 = convexHullHalfEdgeStructure.getVertex(v1Index);
        const Vector3& normal = mapFaceIndexToNormal[faceIndex];

        const decimal distanceToFace = normal.dot(points[vertexIndex] - points[v1.vertexPointIndex]);

        // If the point is in front the face and with a smaller distance from the face
        if (distanceToFace >= 0 && distanceToFace < minDistanceToFace) {
            minDistanceToFace = distanceToFace;
            closestFaceIndex = faceIndex;
        }
    }

    // If we have found a closest face (where the point is in front of the face)
    if (closestFaceIndex != INVALID_FACE_INDEX) {

        // Add the vertex to the remaining closest points of the face
        mapFaceIndexToRemainingClosestPoints[closestFaceIndex].add(vertexIndex);
    }
}

// Compute the initial tetrahedron convex hull
void QuickHull::computeInitialHull(Array<Vector3>& points, HalfEdgeStructure& convexHullHalfEdgeStructure,
                                   Map<uint32, Vector3>& mapFaceIndexToNormal, MemoryAllocator& allocator) {

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

    convexHullHalfEdgeStructure.addVertex(i1);
    convexHullHalfEdgeStructure.addVertex(i2);
    convexHullHalfEdgeStructure.addVertex(i3);
    convexHullHalfEdgeStructure.addVertex(i4);

    Array<uint32> face1Vertices(allocator);
    Array<uint32> face2Vertices(allocator);
    Array<uint32> face3Vertices(allocator);
    Array<uint32> face4Vertices(allocator);

    const Vector3 v1V4 = points[i4] - points[i1];

    // Create the tetrahedron faces (according to the orientation of the tetrahedron)
    if (planeNormal.dot(v1V4) < 0) {

        face1Vertices.add(0); face1Vertices.add(2); face1Vertices.add(1);
        face2Vertices.add(1); face2Vertices.add(2); face2Vertices.add(3);
        face3Vertices.add(0); face3Vertices.add(1); face3Vertices.add(3);
        face4Vertices.add(0); face4Vertices.add(3); face4Vertices.add(2);

        mapFaceIndexToNormal[0] = planeNormal;
        mapFaceIndexToNormal[1] = (points[i3] - points[i2]).cross(points[i4] - points[i2]);
        mapFaceIndexToNormal[2] = (points[i2] - points[i1]).cross(points[i4] - points[i1]);
        mapFaceIndexToNormal[3] = (points[i4] - points[i1]).cross(points[i3] - points[i1]);
    }
    else {

        face1Vertices.add(0); face1Vertices.add(1); face1Vertices.add(2);
        face2Vertices.add(1); face2Vertices.add(3); face2Vertices.add(2);
        face3Vertices.add(0); face3Vertices.add(3); face3Vertices.add(1);
        face4Vertices.add(0); face4Vertices.add(2); face4Vertices.add(3);

        mapFaceIndexToNormal[0] = -planeNormal;
        mapFaceIndexToNormal[1] = (points[i4] - points[i2]).cross(points[i3] - points[i2]);
        mapFaceIndexToNormal[2] = (points[i4] - points[i1]).cross(points[i2] - points[i1]);
        mapFaceIndexToNormal[3] = (points[i3] - points[i1]).cross(points[i4] - points[i1]);
    }

    // Add vertices for each face
    convexHullHalfEdgeStructure.addFace(face1Vertices);
    convexHullHalfEdgeStructure.addFace(face2Vertices);
    convexHullHalfEdgeStructure.addFace(face3Vertices);
    convexHullHalfEdgeStructure.addFace(face4Vertices);

    convexHullHalfEdgeStructure.computeHalfEdges();
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
