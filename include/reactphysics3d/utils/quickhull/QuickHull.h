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
        static void computeInitialHull(Array<Vector3>& points, QHHalfEdgeStructure& convexHull,
                                       Array<QHHalfEdgeStructure::Face*>& initialFaces,
                                       Array<uint32>& orphanPointsIndices,
                                       MemoryAllocator& allocator);

        /// Extract the points from the array
        static void extractPoints(uint32 nbPoints, const void* pointsStart, uint32 pointsStride,
                                  PolygonVertexArray::VertexDataType pointDataType, Array<Vector3>& outArray);

        /// Add a vertex to the current convex hull to expand it
        static void addVertexToHull(uint32 vertexIndex, QHHalfEdgeStructure::Face* face, Array<Vector3>& points,
                                    QHHalfEdgeStructure& convexHull, decimal epsilon,
                                    MemoryAllocator& allocator);

        /// Build the new faces that contain the new vertex and the horizon edges
        static void buildNewFaces(uint32 newVertexIndex, Array<QHHalfEdgeStructure::Vertex*>& horizonVertices,
                                  QHHalfEdgeStructure& convexHull,
                                  Array<Vector3>& points,
                                  Array<QHHalfEdgeStructure::Face*>& newFaces,
                                  MemoryAllocator& allocator);

        /// Delete all the faces visible from the vertex to be added
        static void deleteVisibleFaces(const Array<QHHalfEdgeStructure::Face*>& visibleFaces,
                                       QHHalfEdgeStructure& convexHull,
                                       Array<uint32>& orphanPoints,
                                       const Array<QHHalfEdgeStructure::Vertex*>& horizonVertices,
                                       MemoryAllocator& allocator);

        /// Find the horizon (edges) forming the separation between the faces that are visible from the vertex and the faces that are not visible
        static void findHorizon(const Vector3& vertex, QHHalfEdgeStructure::Face *face,
                                MemoryAllocator& allocator,
                                Array<QHHalfEdgeStructure::Vertex*>& outHorizonVertices,
                                Array<QHHalfEdgeStructure::Face*>& outVisibleFaces,
                                decimal epsilon);

        /// Iterate over all new faces and fix faces that are forming a concave shape in order to always keep the hull convex
        static void mergeConcaveFaces(QHHalfEdgeStructure& convexHull,
                                      Array<QHHalfEdgeStructure::Face*>& newFaces, const Array<Vector3>& points, decimal epsilon);

        /// Merge two faces that are concave at a given edge
        static void mergeConcaveFacesAtEdge(QHHalfEdgeStructure::Edge* edge, QHHalfEdgeStructure& convexHull,
                                            const Array<Vector3>& points);

        /// Fix topological issues (if any) that might have been created during faces merge
        static void fixTopologicalIssues(QHHalfEdgeStructure& convexHull, QHHalfEdgeStructure::Face* face, const Array<Vector3>& points);

        /// Fix topological issue at a given edge
        static void fixTopologicalIssueAtEdge(QHHalfEdgeStructure& convexHull, QHHalfEdgeStructure::Face* face,
                                              QHHalfEdgeStructure::Edge* inEdge, const Array<Vector3>& points);

        /// Recalculate the face centroid and normal to better fit its new vertices (using Newell method)
        static void recalculateFace(QHHalfEdgeStructure::Face* face, const Array<Vector3>& points);

        /// Return the index of the next vertex candidate to be added to the hull
        static void findNextVertexCandidate(Array<Vector3>& points, uint32& outNextVertexIndex,
                                            QHHalfEdgeStructure& convexHull,
                                            QHHalfEdgeStructure::Face*& outNextFace, decimal epsilon);

        /// Find the closest face for a given vertex and add this vertex to the remaining closest points for this face
        static void findClosestFaceForVertex(uint32 vertexIndex, Array<QHHalfEdgeStructure::Face*>& faces, Array<Vector3>& points, decimal epsilon);

        /// Take all the points closest to the old face and add them to the closest faces among the new faces that replace the old face
        static void associateOrphanPointsToNewFaces(Array<uint32>& orphanPointsIndices,
                                                    Array<QHHalfEdgeStructure::Face*>& newFaces,
                                                    Array<Vector3>& points, decimal epsilon);

        /// Return true if the vertex is part of horizon edges
        static bool testIsVertexInHorizon(QHHalfEdgeStructure::Vertex* vertex, const Array<QHHalfEdgeStructure::Vertex*>& horizonVertices);

        /// Return true if a given edge is convex and false otherwise
        static bool testIsConvexEdge(const QHHalfEdgeStructure::Edge* edge, decimal epsilon);

        /// Compute the center of a face (the average of face vertices)
        static Vector3 computeFaceCenter(QHHalfEdgeStructure::Face* face, const Array<Vector3>& points);

        // TODO : Remove this
        static std::string showMap(Map<const QHHalfEdgeStructure::Face*, Array<uint32>>& mapFaceIndexToRemainingClosestPoints);


    public:

        // -------------------- Methods -------------------- //

        /// Compute the convex hull of a set of points and return the resulting polyhedron mesh
        static PolyhedronMesh* computeConvexHull(uint32 nbPoints, const void* points, uint32 pointsStride,
                                                 PolygonVertexArray::VertexDataType pointDataType,
                                                 MemoryAllocator& allocator);



};

}

#endif
