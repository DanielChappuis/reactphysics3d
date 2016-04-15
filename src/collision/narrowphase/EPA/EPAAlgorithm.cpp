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
#include "EPAAlgorithm.h"
#include "engine/Profiler.h"
#include "collision/narrowphase//GJK/GJKAlgorithm.h"
#include "TrianglesStore.h"

// We want to use the ReactPhysics3D namespace
using namespace reactphysics3d;

// Constructor
EPAAlgorithm::EPAAlgorithm() {

}

// Destructor
EPAAlgorithm::~EPAAlgorithm() {
    
}

// Decide if the origin is in the tetrahedron.
/// Return 0 if the origin is in the tetrahedron and return the number (1,2,3 or 4) of
/// the vertex that is wrong if the origin is not in the tetrahedron
int EPAAlgorithm::isOriginInTetrahedron(const Vector3& p1, const Vector3& p2,
                                        const Vector3& p3, const Vector3& p4) const {

    // Check vertex 1
    Vector3 normal1 = (p2-p1).cross(p3-p1);
    if ((normal1.dot(p1) > 0.0) == (normal1.dot(p4) > 0.0)) {
        return 4;
    }

    // Check vertex 2
    Vector3 normal2 = (p4-p2).cross(p3-p2);
    if ((normal2.dot(p2) > 0.0) == (normal2.dot(p1) > 0.0)) {
        return 1;
    }

    // Check vertex 3
    Vector3 normal3 = (p4-p3).cross(p1-p3);
    if ((normal3.dot(p3) > 0.0) == (normal3.dot(p2) > 0.0)) {
        return 2;
    }

    // Check vertex 4
    Vector3 normal4 = (p2-p4).cross(p1-p4);
    if ((normal4.dot(p4) > 0.0) == (normal4.dot(p3) > 0.0)) {
        return 3;
    }

    // The origin is in the tetrahedron, we return 0
    return 0;
}

// Compute the penetration depth with the EPA algorithm.
/// This method computes the penetration depth and contact points between two
/// enlarged objects (with margin) where the original objects (without margin)
/// intersect. An initial simplex that contains origin has been computed with
/// GJK algorithm. The EPA Algorithm will extend this simplex polytope to find
/// the correct penetration depth
void EPAAlgorithm::computePenetrationDepthAndContactPoints(const Simplex& simplex,
                                                           CollisionShapeInfo shape1Info,
                                                           const Transform& transform1,
                                                           CollisionShapeInfo shape2Info,
                                                           const Transform& transform2,
                                                           Vector3& v,
                                                           NarrowPhaseCallback* narrowPhaseCallback) {

    PROFILE("EPAAlgorithm::computePenetrationDepthAndContactPoints()");

    assert(shape1Info.collisionShape->isConvex());
    assert(shape2Info.collisionShape->isConvex());

    const ConvexShape* shape1 = static_cast<const ConvexShape*>(shape1Info.collisionShape);
    const ConvexShape* shape2 = static_cast<const ConvexShape*>(shape2Info.collisionShape);

    void** shape1CachedCollisionData = shape1Info.cachedCollisionData;
    void** shape2CachedCollisionData = shape2Info.cachedCollisionData;

    Vector3 suppPointsA[MAX_SUPPORT_POINTS];  // Support points of object A in local coordinates
    Vector3 suppPointsB[MAX_SUPPORT_POINTS];  // Support points of object B in local coordinates
    Vector3 points[MAX_SUPPORT_POINTS];       // Current points
    TrianglesStore triangleStore;             // Store the triangles
    TriangleEPA* triangleHeap[MAX_FACETS];    // Heap that contains the face
                                              // candidate of the EPA algorithm

    // Transform a point from local space of body 2 to local
    // space of body 1 (the GJK algorithm is done in local space of body 1)
    Transform body2Tobody1 = transform1.getInverse() * transform2;

    // Matrix that transform a direction from local
    // space of body 1 into local space of body 2
    Quaternion rotateToBody2 = transform2.getOrientation().getInverse() *
                              transform1.getOrientation();

    // Get the simplex computed previously by the GJK algorithm
    unsigned int nbVertices = simplex.getSimplex(suppPointsA, suppPointsB, points);

    // Compute the tolerance
    decimal tolerance = MACHINE_EPSILON * simplex.getMaxLengthSquareOfAPoint();

    // Number of triangles in the polytope
    unsigned int nbTriangles = 0;

    // Clear the storing of triangles
    triangleStore.clear();

    // Select an action according to the number of points in the simplex
    // computed with GJK algorithm in order to obtain an initial polytope for
    // The EPA algorithm.
    switch(nbVertices) {
        case 1:
            // Only one point in the simplex (which should be the origin).
            // We have a touching contact with zero penetration depth.
            // We drop that kind of contact. Therefore, we return false
            return;

        case 2: {
            // The simplex returned by GJK is a line segment d containing the origin.
            // We add two additional support points to construct a hexahedron (two tetrahedron
            // glued together with triangle faces. The idea is to compute three different vectors
            // v1, v2 and v3 that are orthogonal to the segment d. The three vectors are relatively
            // rotated of 120 degree around the d segment. The the three new points to
            // construct the polytope are the three support points in those three directions
            // v1, v2 and v3.

            // Direction of the segment
            Vector3 d = (points[1] - points[0]).getUnit();

            // Choose the coordinate axis from the minimal absolute component of the vector d
            int minAxis = d.getAbsoluteVector().getMinAxis();

            // Compute sin(60)
            const decimal sin60 = decimal(sqrt(3.0)) * decimal(0.5);

            // Create a rotation quaternion to rotate the vector v1 to get the vectors
            // v2 and v3
            Quaternion rotationQuat(d.x * sin60, d.y * sin60, d.z * sin60, 0.5);

            // Compute the vector v1, v2, v3
            Vector3 v1 = d.cross(Vector3(minAxis == 0, minAxis == 1, minAxis == 2));
            Vector3 v2 = rotationQuat * v1;
            Vector3 v3 = rotationQuat * v2;

            // Compute the support point in the direction of v1
            suppPointsA[2] = shape1->getLocalSupportPointWithMargin(v1, shape1CachedCollisionData);
            suppPointsB[2] = body2Tobody1 *
                       shape2->getLocalSupportPointWithMargin(rotateToBody2 * (-v1), shape2CachedCollisionData);
            points[2] = suppPointsA[2] - suppPointsB[2];

            // Compute the support point in the direction of v2
            suppPointsA[3] = shape1->getLocalSupportPointWithMargin(v2, shape1CachedCollisionData);
            suppPointsB[3] = body2Tobody1 *
                     shape2->getLocalSupportPointWithMargin(rotateToBody2 * (-v2), shape2CachedCollisionData);
            points[3] = suppPointsA[3] - suppPointsB[3];

            // Compute the support point in the direction of v3
            suppPointsA[4] = shape1->getLocalSupportPointWithMargin(v3, shape1CachedCollisionData);
            suppPointsB[4] = body2Tobody1 *
                            shape2->getLocalSupportPointWithMargin(rotateToBody2 * (-v3), shape2CachedCollisionData);
            points[4] = suppPointsA[4] - suppPointsB[4];

            // Now we have an hexahedron (two tetrahedron glued together). We can simply keep the
            // tetrahedron that contains the origin in order that the initial polytope of the
            // EPA algorithm is a tetrahedron, which is simpler to deal with.

            // If the origin is in the tetrahedron of points 0, 2, 3, 4
            if (isOriginInTetrahedron(points[0], points[2], points[3], points[4]) == 0) {
                // We use the point 4 instead of point 1 for the initial tetrahedron
                suppPointsA[1] = suppPointsA[4];
                suppPointsB[1] = suppPointsB[4];
                points[1] = points[4];
            }
            // If the origin is in the tetrahedron of points 1, 2, 3, 4
            else if (isOriginInTetrahedron(points[1], points[2], points[3], points[4]) == 0) {
                // We use the point 4 instead of point 0 for the initial tetrahedron
                suppPointsA[0] = suppPointsA[4];
                suppPointsB[0] = suppPointsB[4];
                points[0] = points[4];
            }
            else {
                // The origin is not in the initial polytope
                return;
            }

            // The polytope contains now 4 vertices
            nbVertices = 4;
        }
        case 4: {
            // The simplex computed by the GJK algorithm is a tetrahedron. Here we check
            // if this tetrahedron contains the origin. If it is the case, we keep it and
            // otherwise we remove the wrong vertex of the tetrahedron and go in the case
            // where the GJK algorithm compute a simplex of three vertices.

            // Check if the tetrahedron contains the origin (or wich is the wrong vertex otherwise)
            int badVertex = isOriginInTetrahedron(points[0], points[1], points[2], points[3]);

            // If the origin is in the tetrahedron
            if (badVertex == 0) {
                // The tetrahedron is a correct initial polytope for the EPA algorithm.
                // Therefore, we construct the tetrahedron.

                // Comstruct the 4 triangle faces of the tetrahedron
                TriangleEPA* face0 = triangleStore.newTriangle(points, 0, 1, 2);
                TriangleEPA* face1 = triangleStore.newTriangle(points, 0, 3, 1);
                TriangleEPA* face2 = triangleStore.newTriangle(points, 0, 2, 3);
                TriangleEPA* face3 = triangleStore.newTriangle(points, 1, 3, 2);

                // If the constructed tetrahedron is not correct
                if (!((face0 != NULL) && (face1 != NULL) && (face2 != NULL) && (face3 != NULL)
                   && face0->getDistSquare() > 0.0 && face1->getDistSquare() > 0.0
                   && face2->getDistSquare() > 0.0 && face3->getDistSquare() > 0.0)) {
                    return;
                }

                // Associate the edges of neighbouring triangle faces
                link(EdgeEPA(face0, 0), EdgeEPA(face1, 2));
                link(EdgeEPA(face0, 1), EdgeEPA(face3, 2));
                link(EdgeEPA(face0, 2), EdgeEPA(face2, 0));
                link(EdgeEPA(face1, 0), EdgeEPA(face2, 2));
                link(EdgeEPA(face1, 1), EdgeEPA(face3, 0));
                link(EdgeEPA(face2, 1), EdgeEPA(face3, 1));

                // Add the triangle faces in the candidate heap
                addFaceCandidate(face0, triangleHeap, nbTriangles, DECIMAL_LARGEST);
                addFaceCandidate(face1, triangleHeap, nbTriangles, DECIMAL_LARGEST);
                addFaceCandidate(face2, triangleHeap, nbTriangles, DECIMAL_LARGEST);
                addFaceCandidate(face3, triangleHeap, nbTriangles, DECIMAL_LARGEST);

                break;
            }

            // The tetrahedron contains a wrong vertex (the origin is not inside the tetrahedron)
            // Remove the wrong vertex and continue to the next case with the
            // three remaining vertices
            if (badVertex < 4) {

                suppPointsA[badVertex-1] = suppPointsA[3];
                suppPointsB[badVertex-1] = suppPointsB[3];
                points[badVertex-1] = points[3];
            }

            // We have removed the wrong vertex
            nbVertices = 3;
        }
        case 3: {
            // The GJK algorithm returned a triangle that contains the origin.
            // We need two new vertices to create two tetrahedron. The two new
            // vertices are the support points in the "n" and "-n" direction
            // where "n" is the normal of the triangle. Then, we use only the
            // tetrahedron that contains the origin.

            // Compute the normal of the triangle
            Vector3 v1 = points[1] - points[0];
            Vector3 v2 = points[2] - points[0];
            Vector3 n = v1.cross(v2);

            // Compute the two new vertices to obtain a hexahedron
            suppPointsA[3] = shape1->getLocalSupportPointWithMargin(n, shape1CachedCollisionData);
            suppPointsB[3] = body2Tobody1 *
                     shape2->getLocalSupportPointWithMargin(rotateToBody2 * (-n), shape2CachedCollisionData);
            points[3] = suppPointsA[3] - suppPointsB[3];
            suppPointsA[4] = shape1->getLocalSupportPointWithMargin(-n, shape1CachedCollisionData);
            suppPointsB[4] = body2Tobody1 *
                     shape2->getLocalSupportPointWithMargin(rotateToBody2 * n, shape2CachedCollisionData);
            points[4] = suppPointsA[4] - suppPointsB[4];

            TriangleEPA* face0 = NULL;
            TriangleEPA* face1 = NULL;
            TriangleEPA* face2 = NULL;
            TriangleEPA* face3 = NULL;

            // If the origin is in the first tetrahedron
            if (isOriginInTetrahedron(points[0], points[1],
                                      points[2], points[3]) == 0) {
                // The tetrahedron is a correct initial polytope for the EPA algorithm.
                // Therefore, we construct the tetrahedron.

                // Comstruct the 4 triangle faces of the tetrahedron
                face0 = triangleStore.newTriangle(points, 0, 1, 2);
                face1 = triangleStore.newTriangle(points, 0, 3, 1);
                face2 = triangleStore.newTriangle(points, 0, 2, 3);
                face3 = triangleStore.newTriangle(points, 1, 3, 2);
            }
            else if (isOriginInTetrahedron(points[0], points[1],
                                           points[2], points[4]) == 0) {

                // The tetrahedron is a correct initial polytope for the EPA algorithm.
                // Therefore, we construct the tetrahedron.

                // Comstruct the 4 triangle faces of the tetrahedron
                face0 = triangleStore.newTriangle(points, 0, 1, 2);
                face1 = triangleStore.newTriangle(points, 0, 4, 1);
                face2 = triangleStore.newTriangle(points, 0, 2, 4);
                face3 = triangleStore.newTriangle(points, 1, 4, 2);
            }
            else {
                return;
            }

            // If the constructed tetrahedron is not correct
            if (!((face0 != NULL) && (face1 != NULL) && (face2 != NULL) && (face3 != NULL)
               && face0->getDistSquare() > 0.0 && face1->getDistSquare() > 0.0
               && face2->getDistSquare() > 0.0 && face3->getDistSquare() > 0.0)) {
                return;
            }

            // Associate the edges of neighbouring triangle faces
            link(EdgeEPA(face0, 0), EdgeEPA(face1, 2));
            link(EdgeEPA(face0, 1), EdgeEPA(face3, 2));
            link(EdgeEPA(face0, 2), EdgeEPA(face2, 0));
            link(EdgeEPA(face1, 0), EdgeEPA(face2, 2));
            link(EdgeEPA(face1, 1), EdgeEPA(face3, 0));
            link(EdgeEPA(face2, 1), EdgeEPA(face3, 1));

            // Add the triangle faces in the candidate heap
            addFaceCandidate(face0, triangleHeap, nbTriangles, DECIMAL_LARGEST);
            addFaceCandidate(face1, triangleHeap, nbTriangles, DECIMAL_LARGEST);
            addFaceCandidate(face2, triangleHeap, nbTriangles, DECIMAL_LARGEST);
            addFaceCandidate(face3, triangleHeap, nbTriangles, DECIMAL_LARGEST);

            nbVertices = 4;

        }
        break;
    }

    // At this point, we have a polytope that contains the origin. Therefore, we
    // can run the EPA algorithm.

    if (nbTriangles == 0) {
        return;
    }

    TriangleEPA* triangle = 0;
    decimal upperBoundSquarePenDepth = DECIMAL_LARGEST;

    do {
        triangle = triangleHeap[0];

        // Get the next candidate face (the face closest to the origin)
        std::pop_heap(&triangleHeap[0], &triangleHeap[nbTriangles], mTriangleComparison);
        nbTriangles--;

        // If the candidate face in the heap is not obsolete
        if (!triangle->getIsObsolete()) {
            // If we have reached the maximum number of support points
            if (nbVertices == MAX_SUPPORT_POINTS) {
                assert(false);
                break;
            }

            // Compute the support point of the Minkowski
            // difference (A-B) in the closest point direction
            suppPointsA[nbVertices] = shape1->getLocalSupportPointWithMargin(
                                        triangle->getClosestPoint(), shape1CachedCollisionData);
            suppPointsB[nbVertices] = body2Tobody1 *
                               shape2->getLocalSupportPointWithMargin(rotateToBody2 *
                                                                (-triangle->getClosestPoint()), shape2CachedCollisionData);
            points[nbVertices] = suppPointsA[nbVertices] - suppPointsB[nbVertices];

            int indexNewVertex = nbVertices;
            nbVertices++;

            // Update the upper bound of the penetration depth
            decimal wDotv = points[indexNewVertex].dot(triangle->getClosestPoint());
            assert(wDotv > 0.0);
            decimal wDotVSquare = wDotv * wDotv / triangle->getDistSquare();
            if (wDotVSquare < upperBoundSquarePenDepth) {
                upperBoundSquarePenDepth = wDotVSquare;
            }

            // Compute the error
            decimal error = wDotv - triangle->getDistSquare();
            if (error <= std::max(tolerance, REL_ERROR_SQUARE * wDotv) ||
                points[indexNewVertex] == points[(*triangle)[0]] ||
                points[indexNewVertex] == points[(*triangle)[1]] ||
                points[indexNewVertex] == points[(*triangle)[2]]) {
                break;
            }

            // Now, we compute the silhouette cast by the new vertex. The current triangle
            // face will not be in the convex hull. We start the local recursive silhouette
            // algorithm from the current triangle face.
            int i = triangleStore.getNbTriangles();
            if (!triangle->computeSilhouette(points, indexNewVertex, triangleStore)) {
                break;
            }

            // Add all the new triangle faces computed with the silhouette algorithm
            // to the candidates list of faces of the current polytope
            while(i != triangleStore.getNbTriangles()) {
                TriangleEPA* newTriangle = &triangleStore[i];
                addFaceCandidate(newTriangle, triangleHeap, nbTriangles, upperBoundSquarePenDepth);
                i++;
            }
        }
    } while(nbTriangles > 0 && triangleHeap[0]->getDistSquare() <= upperBoundSquarePenDepth);

    // Compute the contact info
    v = transform1.getOrientation() * triangle->getClosestPoint();
    Vector3 pALocal = triangle->computeClosestPointOfObject(suppPointsA);
    Vector3 pBLocal = body2Tobody1.getInverse() * triangle->computeClosestPointOfObject(suppPointsB);
    Vector3 normal = v.getUnit();
    decimal penetrationDepth = v.length();
    assert(penetrationDepth > 0.0);

    if (normal.lengthSquare() < MACHINE_EPSILON) return;
    
    // Create the contact info object
    ContactPointInfo contactInfo(shape1Info.proxyShape, shape2Info.proxyShape, shape1Info.collisionShape,
                                 shape2Info.collisionShape, normal, penetrationDepth, pALocal, pBLocal);

    narrowPhaseCallback->notifyContact(shape1Info.overlappingPair, contactInfo);
}
