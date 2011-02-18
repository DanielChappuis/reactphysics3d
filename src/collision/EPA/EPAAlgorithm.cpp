/********************************************************************************
* ReactPhysics3D physics library, http://code.google.com/p/reactphysics3d/      *
* Copyright (c) 2011 Daniel Chappuis                                            *
*********************************************************************************
*                                                                               *
* Permission is hereby granted, free of charge, to any person obtaining a copy  *
* of this software and associated documentation files (the "Software"), to deal *
* in the Software without restriction, including without limitation the rights  *
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell     *
* copies of the Software, and to permit persons to whom the Software is         *
* furnished to do so, subject to the following conditions:                      *
*                                                                               *
* The above copyright notice and this permission notice shall be included in    *
* all copies or substantial portions of the Software.                           *
*                                                                               *
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR    *
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,      *
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE   *
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER        *
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, *
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN     *
* THE SOFTWARE.                                                                 *
********************************************************************************/

// Libraries
#include "EPAAlgorithm.h"
#include "../GJK/GJKAlgorithm.h"
#include "TrianglesStore.h"

// We want to use the ReactPhysics3D namespace
using namespace reactphysics3d;

// TODO : Check that allocated memory is correctly deleted

// Constructor
EPAAlgorithm::EPAAlgorithm() {

}

// Destructor
EPAAlgorithm::~EPAAlgorithm() {
    
}

// Decide if the origin is in the tetrahedron
// Return 0 if the origin is in the tetrahedron and return the number (1,2,3 or 4) of
// the vertex that is wrong if the origin is not in the tetrahedron
int EPAAlgorithm::isOriginInTetrahedron(const Vector3D& p1, const Vector3D& p2, const Vector3D& p3, const Vector3D& p4) const {

    // Check vertex 1
    Vector3D normal1 = (p2-p1).cross(p3-p1);
    if (normal1.dot(p1) > 0.0 == normal1.dot(p4) > 0.0) {
        return 4;
    }

    // Check vertex 2
    Vector3D normal2 = (p4-p2).cross(p3-p2);
    if (normal2.dot(p2) > 0.0 == normal2.dot(p1) > 0.0) {
        return 1;
    }

    // Check vertex 3
    Vector3D normal3 = (p4-p3).cross(p1-p3);
    if (normal3.dot(p3) > 0.0 == normal3.dot(p2) > 0.0) {
        return 2;
    }

    // Check vertex 4
    Vector3D normal4 = (p2-p4).cross(p1-p4);
    if (normal4.dot(p4) > 0.0 == normal4.dot(p3) > 0.0) {
        return 3;
    }

    // The origin is in the tetrahedron, we return 0
    return 0;
}

// Compute the penetration depth with the EPA algorithms
// This method computes the penetration depth and contact points between two
// enlarged objects (with margin) where the original objects (without margin)
// intersect. An initial simplex that contains origin has been computed with
// GJK algorithm. The EPA Algorithm will extend this simplex polytope to find
// the correct penetration depth
bool EPAAlgorithm::computePenetrationDepthAndContactPoints(Simplex simplex, const NarrowBoundingVolume* const boundingVolume1,
                                                           const NarrowBoundingVolume* const boundingVolume2,
                                                           Vector3D& v, ContactInfo*& contactInfo) {
    Vector3D suppPointsA[MAX_SUPPORT_POINTS];       // Support points of object A in local coordinates
    Vector3D suppPointsB[MAX_SUPPORT_POINTS];       // Support points of object B in local coordinates
    Vector3D points[MAX_SUPPORT_POINTS];            // Current points
    TrianglesStore triangleStore;                   // Store the triangles
    TriangleEPA* triangleHeap[MAX_FACETS];          // Heap that contains the face candidate of the EPA algorithm
    
    // TODO : Check that we call all the supportPoint() function with a margin

    // Get the simplex computed previously by the GJK algorithm
    unsigned int nbVertices = simplex.getSimplex(suppPointsA, suppPointsB, points);

    // Compute the tolerance
    double tolerance = MACHINE_EPSILON * simplex.getMaxLengthSquareOfAPoint();

    // Number of triangles in the polytope
    unsigned int nbTriangles = 0;

    // Clear the storing of triangles
    triangleStore.clear();

    // Select an action according to the number of points in the simplex
    // computed with GJK algorithm in order to obtain an initial polytope for
    // The EPA algorithm.
    switch(nbVertices) {
        case 1:
            // Only one point in the simplex (which should be the origin). We have a touching contact
            // with zero penetration depth. We drop that kind of contact. Therefore, we return false
            return false;

        case 2: {
            // The simplex returned by GJK is a line segment d containing the origin.
            // We add two additional support points to construct a hexahedron (two tetrahedron
            // glued together with triangle faces. The idea is to compute three different vectors
            // v1, v2 and v3 that are orthogonal to the segment d. The three vectors are relatively
            // rotated of 120 degree around the d segment. The the three new points to
            // construct the polytope are the three support points in those three directions
            // v1, v2 and v3.

            // Direction of the segment
            Vector3D d = (points[1] - points[0]).getUnit();

            // Choose the coordinate axis from the minimal absolute component of the vector d
            int minAxis = d.getAbsoluteVector().getMinAxis();

            // Compute sin(60)
            const double sin60 = sqrt(3.0) * 0.5;

            // Create a rotation quaternion to rotate the vector v1 to get the vectors
            // v2 and v3
            Quaternion rotationQuat(d.getX() * sin60, d.getY() * sin60, d.getZ() * sin60, 0.5);

            // Construct the corresponding rotation matrix
            Matrix3x3 rotationMat = rotationQuat.getMatrix();

            // Compute the vector v1, v2, v3
            Vector3D v1 = d.cross(Vector3D(minAxis == 0, minAxis == 1, minAxis == 2));
            Vector3D v2 = rotationMat * v1;
            Vector3D v3 = rotationMat * v2;

            // Compute the support point in the direction of v1
            suppPointsA[2] = boundingVolume1->getSupportPoint(v1, OBJECT_MARGIN);
            suppPointsB[2] = boundingVolume2->getSupportPoint(v1.getOpposite(), OBJECT_MARGIN);
            points[2] = suppPointsA[2] - suppPointsB[2];

            // Compute the support point in the direction of v2
            suppPointsA[3] = boundingVolume1->getSupportPoint(v2, OBJECT_MARGIN);
            suppPointsB[3] = boundingVolume2->getSupportPoint(v2.getOpposite(), OBJECT_MARGIN);
            points[3] = suppPointsA[3] - suppPointsB[3];

            // Compute the support point in the direction of v3
            suppPointsA[4] = boundingVolume1->getSupportPoint(v3, OBJECT_MARGIN);
            suppPointsB[4] = boundingVolume2->getSupportPoint(v3.getOpposite(), OBJECT_MARGIN);
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
            else if (isOriginInTetrahedron(points[1], points[2], points[3], points[4]) == 0) {  // If the origin is in the tetrahedron of points 1, 2, 3, 4
                // We use the point 4 instead of point 0 for the initial tetrahedron
                suppPointsA[0] = suppPointsA[0];
                suppPointsB[0] = suppPointsB[0];
                points[0] = points[0];
            }
            else {
                // The origin is not in the initial polytope
                return false;
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
                if (!(face0 && face1 && face2 && face3 && face0->getDistSquare() > 0.0 &&
                      face1->getDistSquare() > 0.0 && face2->getDistSquare() > 0.0 && face3->getDistSquare() > 0.0)) {
                    return false;
                }

                // Associate the edges of neighbouring triangle faces
                EdgeEPA(face0, 0).link(EdgeEPA(face1, 2));
                EdgeEPA(face0, 1).link(EdgeEPA(face3, 2));
                EdgeEPA(face0, 2).link(EdgeEPA(face2, 0));
                EdgeEPA(face1, 0).link(EdgeEPA(face2, 2));
                EdgeEPA(face1, 1).link(EdgeEPA(face3, 0));
                EdgeEPA(face2, 1).link(EdgeEPA(face3, 1));

                // Add the triangle faces in the candidate heap
                addFaceCandidate(face0, triangleHeap, nbTriangles, DBL_MAX);
                addFaceCandidate(face1, triangleHeap, nbTriangles, DBL_MAX);
                addFaceCandidate(face2, triangleHeap, nbTriangles, DBL_MAX);
                addFaceCandidate(face3, triangleHeap, nbTriangles, DBL_MAX);

                break;
            }

            // If the tetrahedron contains a wrong vertex (the origin is not inside the tetrahedron)
            if (badVertex < 4) {
                // Replace the wrong vertex with the point 5 (if it exists)
                suppPointsA[badVertex-1] = suppPointsA[4];
                suppPointsB[badVertex-1] = suppPointsB[4];
                points[badVertex-1] = points[4];
            }

            // We have removed the wrong vertex
            nbVertices = 3;
        }
        case 3: {
            // The GJK algorithm returned a triangle that contains the origin.
            // We need two new vertices to obtain a hexahedron. The two new vertices
            // are the support points in the "n" and "-n" direction where "n" is the
            // normal of the triangle.

            // Compute the normal of the triangle
            Vector3D v1 = points[1] - points[0];
            Vector3D v2 = points[2] - points[0];
            Vector3D n = v1.cross(v2);

            // Compute the two new vertices to obtain a hexahedron
            suppPointsA[3] = boundingVolume1->getSupportPoint(n, OBJECT_MARGIN);
            suppPointsB[3] = boundingVolume2->getSupportPoint(n.getOpposite(), OBJECT_MARGIN);
            points[3] = suppPointsA[3] - suppPointsB[3];
            suppPointsA[4] = boundingVolume1->getSupportPoint(n.getOpposite(), OBJECT_MARGIN);
            suppPointsB[4] = boundingVolume2->getSupportPoint(n, OBJECT_MARGIN);
            points[4] = suppPointsA[4] - suppPointsB[4];

            // Construct the triangle faces
            TriangleEPA* face0 = triangleStore.newTriangle(points, 0, 1, 3);
            TriangleEPA* face1 = triangleStore.newTriangle(points, 1, 2, 3);
            TriangleEPA* face2 = triangleStore.newTriangle(points, 2, 0, 3);
            TriangleEPA* face3 = triangleStore.newTriangle(points, 0, 2, 4);
            TriangleEPA* face4 = triangleStore.newTriangle(points, 2, 1, 4);
            TriangleEPA* face5 = triangleStore.newTriangle(points, 1, 0, 4);

            // If the polytope hasn't been correctly constructed
            if (!(face0 && face1 && face2 && face3 && face4 && face5 &&
                  face0->getDistSquare() > 0.0 && face1->getDistSquare() > 0.0 &&
                  face2->getDistSquare() > 0.0 && face3->getDistSquare() > 0.0 &&
                  face4->getDistSquare() > 0.0 && face5->getDistSquare() > 0.0)) {
                return false;
            }

            // Associate the edges of neighbouring faces
            EdgeEPA(face0, 1).link(EdgeEPA(face1, 2));
            EdgeEPA(face1, 1).link(EdgeEPA(face2, 2));
            EdgeEPA(face2, 1).link(EdgeEPA(face0, 2));
            EdgeEPA(face0, 0).link(EdgeEPA(face5, 0));
            EdgeEPA(face1, 0).link(EdgeEPA(face4, 0));
            EdgeEPA(face2, 0).link(EdgeEPA(face3, 0));
            EdgeEPA(face3, 1).link(EdgeEPA(face4, 2));
            EdgeEPA(face4, 1).link(EdgeEPA(face5, 2));
            EdgeEPA(face5, 1).link(EdgeEPA(face3, 2));

            // Add the candidate faces in the heap
            addFaceCandidate(face0, triangleHeap, nbTriangles, DBL_MAX);
            addFaceCandidate(face1, triangleHeap, nbTriangles, DBL_MAX);
            addFaceCandidate(face2, triangleHeap, nbTriangles, DBL_MAX);
            addFaceCandidate(face3, triangleHeap, nbTriangles, DBL_MAX);
            addFaceCandidate(face4, triangleHeap, nbTriangles, DBL_MAX);
            addFaceCandidate(face5, triangleHeap, nbTriangles, DBL_MAX);

            nbVertices = 5;
        }
        break;
    }

    // At this point, we have a polytope that contains the origin. Therefore, we
    // can run the EPA algorithm.

    if (nbTriangles == 0) {
        return false;
    }

    TriangleEPA* triangle = 0;
    double upperBoundSquarePenDepth = DBL_MAX;

    do {
        triangle = triangleHeap[0];

        // Get the next candidate face (the face closest to the origin)
        std::pop_heap(&triangleHeap[0], &triangleHeap[nbTriangles], triangleComparison);
        nbTriangles--;

        // If the candidate face in the heap is not obsolete
        if (!triangle->getIsObsolete()) {
            // If we have reached the maximum number of support points
            if (nbVertices == MAX_SUPPORT_POINTS) {
                assert(false);
                break;
            }

            // Compute the support point of the Minkowski difference (A-B) in the closest point direction
            suppPointsA[nbVertices] = boundingVolume1->getSupportPoint(triangle->getClosestPoint(), OBJECT_MARGIN);
            suppPointsB[nbVertices] = boundingVolume2->getSupportPoint(triangle->getClosestPoint().getOpposite());
            points[nbVertices] = suppPointsA[nbVertices] - suppPointsB[nbVertices];

            int indexNewVertex = nbVertices;
            nbVertices++;

            // Update the upper bound of the penetration depth
            double wDotv = points[indexNewVertex].dot(triangle->getClosestPoint());
            assert(wDotv > 0.0);
            double wDotVSquare = wDotv * wDotv / triangle->getDistSquare();
            if (wDotVSquare < upperBoundSquarePenDepth) {
                upperBoundSquarePenDepth = wDotVSquare;
            }

            // Compute the error
            double error = wDotv - triangle->getDistSquare();
            if (error <= std::max(tolerance, REL_ERROR_SQUARE * wDotv) ||
                points[indexNewVertex] == points[(*triangle)[0]] ||
                points[indexNewVertex] == points[(*triangle)[1]] ||
                points[indexNewVertex] == points[(*triangle)[2]]) {
                break;
            }

            // Now, we compute the silhouette cast by the new vertex.
            // The current triangle face will not be in the convex hull.
            // We start the local recursive silhouette algorithm from
            // the current triangle face.
            int i = triangleStore.getNbTriangles();
            if (!triangle->computeSilhouette(points, indexNewVertex, triangleStore)) {
                break;
            }

            // Construct the new polytope by constructing triangle faces from the
            // silhouette to the new vertex of the polytope in order that the new
            // polytope is always convex
            while(i != triangleStore.getNbTriangles()) {
                TriangleEPA* newTriangle = &triangleStore[i];

                addFaceCandidate(newTriangle, triangleHeap, nbTriangles, upperBoundSquarePenDepth);
                i++;
            }
        }

    } while(nbTriangles > 0 && triangleHeap[0]->getDistSquare() <= upperBoundSquarePenDepth);

    // Compute the contact info
    v = triangle->getClosestPoint();
    Vector3D pA = triangle->computeClosestPointOfObject(suppPointsA);
    Vector3D pB = triangle->computeClosestPointOfObject(suppPointsB);
    Vector3D diff = pB - pA;
    Vector3D normal = diff.getUnit();
    double penetrationDepth = diff.length();
    assert(penetrationDepth > 0.0);
    contactInfo = new ContactInfo(boundingVolume1->getBodyPointer(), boundingVolume2->getBodyPointer(),
                                  normal, penetrationDepth, pA, pB);
    
    return true;
}
