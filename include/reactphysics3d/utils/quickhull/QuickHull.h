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

#ifndef REACTPHYSICS3D_QUICKHULL_H
#define REACTPHYSICS3D_QUICKHULL_H

// Libraries
#include <reactphysics3d/configuration.h>
#include <reactphysics3d/collision/PolygonVertexArray.h>
#include <reactphysics3d/containers/Map.h>
#include <reactphysics3d/utils/quickhull/QHHalfEdgeStructure.h>

/// ReactPhysics3D namespace
namespace reactphysics3d {

// Declarations
class PolyhedronMesh;
template<typename T>
class Array;

// Class QuickHull
// This algorithm is based on 'Implementing Quickhull' presentation at GDC from Dirk Gregorius
class QuickHull {

    private:

        // Structure Candidate
        // This is used during the process to find the horizon
        struct CandidateFace {

            const QHHalfEdgeStructure::Face* face;
            const QHHalfEdgeStructure::Edge* startEdge;
            const QHHalfEdgeStructure::Edge* currentEdge;

            // Constructor
            CandidateFace(const QHHalfEdgeStructure::Face* face, const QHHalfEdgeStructure::Edge* startEdge,
                          const QHHalfEdgeStructure::Edge* currentEdge)
                :face(face), startEdge(startEdge), currentEdge(currentEdge) {}
        };

        // -------------------- Constants -------------------- //

        static const uint32 INVALID_VERTEX_INDEX = -1;
        static const uint32 INVALID_FACE_INDEX = -1;

        // -------------------- Attributes -------------------- //

        // -------------------- Methods -------------------- //

        // Compute the initial tetrahedron convex hull
        static void computeInitialHull(Array<Vector3>& points, QHHalfEdgeStructure& convexHullHalfEdgeStructure,
                                       Map<const QHHalfEdgeStructure::Face*, Vector3>& mapFaceToNormal,
                                       Map<const QHHalfEdgeStructure::Face *, Array<uint32>>& mapFaceToRemainingClosestPoints,
                                       Array<const QHHalfEdgeStructure::Face*>& initialFaces,
                                       Array<uint32>& orphanPointsIndices,
                                       MemoryAllocator& allocator);

        /// Extract the points from the array
        static void extractPoints(uint32 nbPoints, const void* pointsStart, uint32 pointsStride,
                                  PolygonVertexArray::VertexDataType pointDataType, Array<Vector3>& outArray);

        /// Add a vertex to the current convex hull to expand it
        static void addVertexToHull(uint32 vertexIndex, QHHalfEdgeStructure::Face* face, Array<Vector3>& points,
                                    Map<const QHHalfEdgeStructure::Face*, Vector3>& mapFaceToNormal,
                                    QHHalfEdgeStructure& convexHullHalfEdgeStructure,
                                    Map<const QHHalfEdgeStructure::Face*, Array<uint32>>& mapFaceToRemainingClosestPoints,
                                    MemoryAllocator& allocator);

        /// Build the new faces that contain the new vertex and the horizon edges
        static void buildNewFaces(uint32 newVertexIndex, Array<QHHalfEdgeStructure::Vertex*>& horizonVertices,
                                  QHHalfEdgeStructure& convexHullHalfEdgeStructure,
                                  Map<const QHHalfEdgeStructure::Face*, Vector3>& mapFaceToNormal,
                                  Map<const QHHalfEdgeStructure::Face*, Array<uint32>>& mapFaceToRemainingClosestPoints,
                                  Array<Vector3>& points,
                                  Array<const QHHalfEdgeStructure::Face*>& newFaces,
                                  MemoryAllocator& allocator);

        /// Delete all the faces visible from the vertex to be added
        static void deleteVisibleFaces(const Array<QHHalfEdgeStructure::Face*>& visibleFaces,
                                       QHHalfEdgeStructure& convexHullHalfEdgeStructure,
                                       Map<const QHHalfEdgeStructure::Face *, Array<uint32>>& mapFaceToRemainingClosestPoints,
                                       Map<const QHHalfEdgeStructure::Face *, Vector3>& mapFaceToNormal,
                                       Array<uint32>& orphanPoints,
                                       const Array<QHHalfEdgeStructure::Vertex*>& horizonVertices,
                                       MemoryAllocator& allocator);

        /// Find the horizon (edges) forming the separation between the faces that are visible from the vertex and the faces that are not visible
        static void findHorizon(const Vector3& vertex, QHHalfEdgeStructure::Face *face,
                                Map<const QHHalfEdgeStructure::Face*, Vector3>& mapFaceToNormal, const Array<Vector3>& points,
                                MemoryAllocator& allocator,
                                Array<QHHalfEdgeStructure::Vertex*>& outHorizonVertices,
                                Array<QHHalfEdgeStructure::Face*>& outVisibleFaces);

        /// Return the index of the next vertex candidate to be added to the hull
        static void findNextVertexCandidate(Map<const QHHalfEdgeStructure::Face*, Array<uint32>>& mapFacesToRemainingClosestPoints,
                                            Map<const QHHalfEdgeStructure::Face*, Vector3>& mapFacesToNormal,
                                            Array<Vector3>& points, uint32& outNextVertexIndex,
                                            QHHalfEdgeStructure::Face*& outNextFace);

        /// Find the closest face for a given vertex and add this vertex to the remaining closest points for this face
        static void findClosestFaceForVertex(uint32 vertexIndex, Array<const QHHalfEdgeStructure::Face*>& faces,
                                             Array<Vector3>& points, Map<const QHHalfEdgeStructure::Face*, Vector3>& mapFaceToNormal,
                                             Map<const QHHalfEdgeStructure::Face*, Array<uint32>>& mapFaceToRemainingClosestPoints);

        /// Take all the points closest to the old face and add them to the closest faces among the new faces that replace the old face
        static void associateOrphanPointsToNewFaces(Array<uint32>& orphanPointsIndices,
                                                    Array<const QHHalfEdgeStructure::Face*>& newFaces,
                                                    Array<Vector3>& points,
                                                    Map<const QHHalfEdgeStructure::Face*,
                                                    Vector3>& mapFaceToNormal,
                                                    Map<const QHHalfEdgeStructure::Face*, Array<uint32>>& mapFaceToRemainingClosestPoints);

        /// Return true if the vertex is part of horizon edges
        static bool checkVertexInHorizon(QHHalfEdgeStructure::Vertex* vertex, const Array<QHHalfEdgeStructure::Vertex*>& horizonVertices);

        // TODO : Remove this
        static std::string showMap(Map<const QHHalfEdgeStructure::Face*, Array<uint32>>& mapFaceIndexToRemainingClosestPoints);


    public:

        // -------------------- Methods -------------------- //

        /// Compute the convex hull of a set of points and return the resulting polyhedron mesh
        static PolygonVertexArray* computeConvexHull(uint32 nbPoints, const void* points, uint32 pointsStride,
                                                 PolygonVertexArray::VertexDataType pointDataType,
                                                 MemoryAllocator& allocator);



};

}

#endif
