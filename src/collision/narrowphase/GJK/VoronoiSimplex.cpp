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
#include <reactphysics3d/collision/narrowphase/GJK/VoronoiSimplex.h>
#include <reactphysics3d/mathematics/Vector2.h>

// We want to use the ReactPhysics3D namespace
using namespace reactphysics3d;

decimal VoronoiSimplex::epsilon = decimal(0.0001);

// Constructor
VoronoiSimplex::VoronoiSimplex() : mNbPoints(0), mRecomputeClosestPoint(false), mIsClosestPointValid(false) {

}

// Destructor
VoronoiSimplex::~VoronoiSimplex() {

}

// Add a new support point of (A-B) into the simplex
/// suppPointA : support point of object A in a direction -v
/// suppPointB : support point of object B in a direction v
/// point      : support point of object (A-B) => point = suppPointA - suppPointB
void VoronoiSimplex::addPoint(const Vector3& point, const Vector3& suppPointA, const Vector3& suppPointB) {
    assert(!isFull());

    mPoints[mNbPoints] = point;
    mSuppPointsA[mNbPoints] = suppPointA;
    mSuppPointsB[mNbPoints] = suppPointB;

    mNbPoints++;
    mRecomputeClosestPoint = true;
}

// Remove a point from the simplex
void VoronoiSimplex::removePoint(int index) {
    assert(mNbPoints > 0);
    mNbPoints--;
    mPoints[index] = mPoints[mNbPoints];
    mSuppPointsA[index] = mSuppPointsA[mNbPoints];
    mSuppPointsB[index] = mSuppPointsB[mNbPoints];
}

// Reduce the simplex (only keep vertices that participate to the point closest to the origin)
/// bitsUsedPoints is seen as a sequence of bits representing whether the four points of
/// the simplex are used or not to represent the current closest point to the origin.
/// - The most right bit is set to one if the first point is used
/// - The second most right bit is set to one if the second point is used
/// - The third most right bit is set to one if the third point is used
/// - The fourth most right bit is set to one if the fourth point is used
void VoronoiSimplex::reduceSimplex(int bitsUsedPoints) {

    if ((mNbPoints >= 4) && (bitsUsedPoints & 8) == 0)
        removePoint(3);

    if ((mNbPoints >= 3) && (bitsUsedPoints & 4) == 0)
        removePoint(2);

    if ((mNbPoints >= 2) && (bitsUsedPoints & 2) == 0)
        removePoint(1);

    if ((mNbPoints >= 1) && (bitsUsedPoints & 1) == 0)
        removePoint(0);
}

// Return true if the point is in the simplex
bool VoronoiSimplex::isPointInSimplex(const Vector3& point) const {

    // For each four possible points in the simplex
    for (int i=0; i<mNbPoints; i++) {

        // Compute the distance between the points
        decimal distanceSquare = (mPoints[i] - point).lengthSquare();

        // If the point is very close
        if (distanceSquare <= epsilon) {
            return true;
        }
    }

    return false;
}

// Return the points of the simplex
int VoronoiSimplex::getSimplex(Vector3* suppPointsA, Vector3* suppPointsB,
                                 Vector3* points) const {

    for (int i=0; i<mNbPoints; i++) {
        points[i] = mPoints[i];
        suppPointsA[i] = mSuppPointsA[i];
        suppPointsB[i] = mSuppPointsB[i];
    }

    // Return the number of points in the simplex
    return mNbPoints;
}

// Return true if the set is affinely dependent.
/// A set if affinely dependent if a point of the set
/// is an affine combination of other points in the set
bool VoronoiSimplex::isAffinelyDependent() const {

    assert(mNbPoints >= 0 && mNbPoints <= 4);

    switch(mNbPoints) {
        case 0: return false;
        case 1: return false;

        // Two points are independent if there distance is larger than zero
        case 2: return (mPoints[1] - mPoints[0]).lengthSquare() <= epsilon;

        // Three points are independent if the triangle area is larger than zero
        case 3: return (mPoints[1] - mPoints[0]).cross(mPoints[2] - mPoints[0]).lengthSquare() <= epsilon;

        // Four points are independent if the tetrahedron volume is larger than zero
        // Test in three different ways (for more robustness)
        case 4: return std::abs((mPoints[1] - mPoints[0]).dot((mPoints[2] - mPoints[0]).cross(mPoints[3] - mPoints[0]))) <= epsilon;
    }

    return false;
}

// Compute the closest points "pA" and "pB" of object A and B.
/// The points are computed as follows :
///      pA = sum(lambda_i * a_i)    where "a_i" are the support points of object A
///      pB = sum(lambda_i * b_i)    where "b_i" are the support points of object B
///      with lambda_i = deltaX_i / deltaX
void VoronoiSimplex::computeClosestPointsOfAandB(Vector3& pA, Vector3& pB) const {

    pA = mClosestSuppPointA;
    pB = mClosestSuppPointB;
}

// Recompute the closest point if the simplex has been modified
/// This method computes the point "v" of simplex that is closest to the origin.
/// The method returns true if a closest point has been found.
bool VoronoiSimplex::recomputeClosestPoint() {

    assert(mNbPoints <= 4);

    // If we need to recompute the closest point
    if (mRecomputeClosestPoint) {

        mRecomputeClosestPoint = false;

        switch(mNbPoints) {

            case 0:

                // Cannot compute closest point when the simplex is empty
                mIsClosestPointValid = false;
                break;

            case 1:

                {
                    // There is a single point in the simplex, therefore, this point is
                    // the one closest to the origin
                    mClosestPoint = mPoints[0];
                    mClosestSuppPointA = mSuppPointsA[0];
                    mClosestSuppPointB = mSuppPointsB[0];
                    setBarycentricCoords(1, 0, 0, 0);
                    mIsClosestPointValid = checkClosestPointValid();
                }
                break;

            case 2:

                {
                    int bitsUsedPoints = 0;
                    float t;

                    // The simplex is a line AB (where A=mPoints[0] and B=mPoints[1].
                    // We need to find the point of that line closest to the origin
                    computeClosestPointOnSegment(mPoints[0], mPoints[1], bitsUsedPoints, t);

                    // Compute the closest point
                    mClosestSuppPointA = mSuppPointsA[0] + t * (mSuppPointsA[1] - mSuppPointsA[0]);
                    mClosestSuppPointB = mSuppPointsB[0] + t * (mSuppPointsB[1] - mSuppPointsB[0]);
                    mClosestPoint = mClosestSuppPointA - mClosestSuppPointB;
                    setBarycentricCoords(decimal(1.0) - t, t, 0, 0);
                    mIsClosestPointValid = checkClosestPointValid();

                    // Reduce the simplex (remove vertices that are not participating to
                    // the closest point
                    reduceSimplex(bitsUsedPoints);
                }
                break;

            case 3:
                {
                    // The simplex is a triangle. We need to find the point of that
                    // triangle that is closest to the origin

                    int bitsUsedVertices = 0;
                    Vector3 baryCoords;

                    // Compute the point of the triangle closest to the origin
                    computeClosestPointOnTriangle(mPoints[0], mPoints[1], mPoints[2], bitsUsedVertices, baryCoords);
                    mClosestSuppPointA = baryCoords[0] * mSuppPointsA[0] + baryCoords[1] * mSuppPointsA[1] +
                                         baryCoords[2] * mSuppPointsA[2];
                    mClosestSuppPointB = baryCoords[0] * mSuppPointsB[0] + baryCoords[1] * mSuppPointsB[1] +
                                         baryCoords[2] * mSuppPointsB[2];
                    mClosestPoint = mClosestSuppPointA - mClosestSuppPointB;

                    setBarycentricCoords(baryCoords.x, baryCoords.y, baryCoords.z, 0.0);
                    mIsClosestPointValid = checkClosestPointValid();

                    // Reduce the simplex (remove vertices that are not participating to
                    // the closest point
                    reduceSimplex(bitsUsedVertices);
                }
                break;

            case 4:

                {
                    // The simplex is a tetrahedron. We need to find the point of that
                    // tetrahedron that is closest to the origin

                    int bitsUsedVertices = 0;
                    Vector2 baryCoordsAB;
                    Vector2 baryCoordsCD;
                    bool isDegenerate;

                    // Compute the point closest to the origin on the tetrahedron
                    bool isOutside = computeClosestPointOnTetrahedron(mPoints[0], mPoints[1], mPoints[2], mPoints[3],
                                                                      bitsUsedVertices, baryCoordsAB, baryCoordsCD, isDegenerate);

                    // If the origin is outside the tetrahedron
                    if (isOutside) {

                        // Compute the point of the tetrahedron closest to the origin
                        mClosestSuppPointA = baryCoordsAB.x * mSuppPointsA[0] + baryCoordsAB.y * mSuppPointsA[1] +
                                             baryCoordsCD.x * mSuppPointsA[2] + baryCoordsCD.y * mSuppPointsA[3];
                        mClosestSuppPointB = baryCoordsAB.x * mSuppPointsB[0] + baryCoordsAB.y * mSuppPointsB[1] +
                                             baryCoordsCD.x * mSuppPointsB[2] + baryCoordsCD.y * mSuppPointsB[3];
                        mClosestPoint = mClosestSuppPointA - mClosestSuppPointB;

                        setBarycentricCoords(baryCoordsAB.x, baryCoordsAB.y, baryCoordsCD.x, baryCoordsCD.y);

                        // Reduce the simplex (remove vertices that are not participating to
                        // the closest point
                        reduceSimplex(bitsUsedVertices);
                    }
                    else {

                        // If it is a degenerate case
                        if (isDegenerate) {
                            mIsClosestPointValid = false;
                        }
                        else {

                            // The origin is inside the tetrahedron, therefore, the closest point
                            // is the origin

                            setBarycentricCoords(0.0, 0.0, 0.0, 0.0);

                            mClosestSuppPointA.setToZero();
                            mClosestSuppPointB.setToZero();
                            mClosestPoint.setToZero();

                            mIsClosestPointValid = true;
                        }
                        break;
                    }

                    mIsClosestPointValid = checkClosestPointValid();
                }
                break;
        }
    }

    return mIsClosestPointValid;
}

// Compute point of a line segment that is closest to the origin
void VoronoiSimplex::computeClosestPointOnSegment(const Vector3& a, const Vector3& b, int& bitUsedVertices,
                                                  float& t) const {

    Vector3 AP = -a;
    Vector3 AB = b - a;
    decimal APDotAB = AP.dot(AB);

    // If the closest point is on the side of A in the direction of B
    if (APDotAB > decimal(0.0)) {
        decimal lengthABSquare = AB.lengthSquare();

        // If the closest point is on the segment AB
        if (APDotAB < lengthABSquare) {
            t = APDotAB / lengthABSquare;

            bitUsedVertices = 3; // 0011 (both A and B are used)

        }
        else {  // If the origin is on the side of B that is not in the direction of A
            // Therefore, the closest point is B
            t = decimal(1.0);

            bitUsedVertices = 2; // 0010 (only B is used)
        }
    }
    else {  // If the origin is on the side of A that is not in the direction of B
        // Therefore, the closest point of the line is A
        t = decimal(0.0);

        bitUsedVertices = 1; // 0001 (only A is used)
    }
}

// Compute point on a triangle that is closest to the origin
/// This implementation is based on the one in the book
/// "Real-Time Collision Detection" by Christer Ericson.
void VoronoiSimplex::computeClosestPointOnTriangle(const Vector3& a, const Vector3& b, const Vector3& c,
                                                   int& bitsUsedVertices, Vector3& baryCoordsABC) const {

    // Check if the origin is in the Voronoi region of vertex A
    Vector3 ab = b - a;
    Vector3 ac = c - a;
    Vector3 ap = -a;
    decimal d1 = ab.dot(ap);
    decimal d2 = ac.dot(ap);
    if (d1 <= decimal(0.0) && d2 <= decimal(0.0)) {

        // The origin is in the Voronoi region of vertex A

        // Set the barycentric coords of the closest point on the triangle
        baryCoordsABC.setAllValues(1.0, 0, 0);

        bitsUsedVertices = 1;    // 0001 (only A is used)
        return;
    }

    // Check if the origin is in the Voronoi region of vertex B
    Vector3 bp = -b;
    decimal d3 = ab.dot(bp);
    decimal d4 = ac.dot(bp);
    if (d3 >= decimal(0.0) && d4 <= d3) {

        // The origin is in the Voronoi region of vertex B

        // Set the barycentric coords of the closest point on the triangle
        baryCoordsABC.setAllValues(0.0, 1.0, 0);

        bitsUsedVertices = 2;    // 0010 (only B is used)
        return;
    }

    // Check if the origin is in the Voronoi region of edge AB
    decimal vc = d1 * d4 - d3 * d2;
    if (vc <= decimal(0.0) && d1 >= decimal(0.0) && d3 <= decimal(0.0)) {

        // The origin is in the Voronoi region of edge AB
        // We return the projection of the origin on the edge AB
        assert(std::abs(d1 - d3) > MACHINE_EPSILON);
        decimal v = d1 / (d1 - d3);

        // Set the barycentric coords of the closest point on the triangle
        baryCoordsABC.setAllValues(decimal(1.0) - v, v, 0);

        bitsUsedVertices = 3;    // 0011 (A and B are used)
        return;
    }

    // Check if the origin is in the Voronoi region of vertex C
    Vector3 cp = -c;
    decimal d5 = ab.dot(cp);
    decimal d6 = ac.dot(cp);
    if (d6 >= decimal(0.0) && d5 <= d6) {

        // The origin is in the Voronoi region of vertex C

        // Set the barycentric coords of the closest point on the triangle
        baryCoordsABC.setAllValues(0.0, 0.0, 1.0);

        bitsUsedVertices = 4;    // 0100 (only C is used)
        return;
    }

    // Check if the origin is in the Voronoi region of edge AC
    decimal vb = d5 * d2 - d1 * d6;
    if (vb <= decimal(0.0) && d2 >= decimal(0.0) && d6 <= decimal(0.0)) {

        // The origin is in the Voronoi region of edge AC
        // We return the projection of the origin on the edge AC
        assert(std::abs(d2 - d6) > MACHINE_EPSILON);
        decimal w = d2 / (d2 - d6);

        // Set the barycentric coords of the closest point on the triangle
        baryCoordsABC.setAllValues(decimal(1.0) - w, 0, w);

        bitsUsedVertices = 5;    // 0101 (A and C are used)
        return;
    }

    // Check if the origin is in the Voronoi region of edge BC
    decimal va = d3 * d6 - d5 * d4;
    if (va <= decimal(0.0) && (d4 - d3) >= decimal(0.0) && (d5 - d6) >= decimal(0.0)) {

        // The origin is in the Voronoi region of edge BC
        // We return the projection of the origin on the edge BC
        assert(std::abs((d4 - d3) + (d5 - d6)) > MACHINE_EPSILON);
        decimal w = (d4 - d3) / ((d4 - d3) + (d5 - d6));

        // Set the barycentric coords of the closest point on the triangle
        baryCoordsABC.setAllValues(0.0, decimal(1.0) - w, w);

        bitsUsedVertices = 6;    // 0110 (B and C are used)
        return;
    }

    // The origin is in the Voronoi region of the face ABC
    decimal denom = decimal(1.0) / (va + vb + vc);
    decimal v = vb * denom;
    decimal w = vc * denom;

    // Set the barycentric coords of the closest point on the triangle
    baryCoordsABC.setAllValues(1 - v - w, v, w);

    bitsUsedVertices = 7;    // 0111 (A, B and C are used)
    return;
}

// Compute point of a tetrahedron that is closest to the origin
/// This implementation is based on the one in the book
/// "Real-Time Collision Detection" by Christer Ericson.
/// This method returns true if the origin is outside the tetrahedron.
bool VoronoiSimplex::computeClosestPointOnTetrahedron(const Vector3& a, const Vector3& b, const Vector3& c,
                                                      const Vector3& d, int& bitsUsedPoints, Vector2& baryCoordsAB,
                                                      Vector2& baryCoordsCD, bool& isDegenerate) const {

    isDegenerate = false;

    // Start as if the origin was inside the tetrahedron
    bitsUsedPoints = 15;        // 1111 (A, B, C and D are used)
    baryCoordsAB.setToZero();
    baryCoordsCD.setToZero();

    // Check if the origin is outside each tetrahedron face
    int isOriginOutsideFaceABC = testOriginOutsideOfPlane(a, b, c, d);
    int isOriginOutsideFaceACD = testOriginOutsideOfPlane(a, c, d, b);
    int isOriginOutsideFaceADB = testOriginOutsideOfPlane(a, d, b, c);
    int isOriginOutsideFaceBDC = testOriginOutsideOfPlane(b, d, c, a);

    // If we have a degenerate tetrahedron
    if (isOriginOutsideFaceABC < 0 || isOriginOutsideFaceACD < 0 ||
        isOriginOutsideFaceADB < 0 || isOriginOutsideFaceBDC < 0) {

        // The tetrahedron is degenerate
        isDegenerate = true;
        return false;
    }

    if (isOriginOutsideFaceABC == 0 && isOriginOutsideFaceACD == 0 &&
        isOriginOutsideFaceADB == 0 && isOriginOutsideFaceBDC == 0) {

        // The origin is inside the tetrahedron
        return true;
    }

    // We know that the origin is outside the tetrahedron, we now need to find
    // which of the four triangle faces is closest to it.

    decimal closestSquareDistance = DECIMAL_LARGEST;
    int tempUsedVertices;
    Vector3 triangleBaryCoords;

    // If the origin is outside face ABC
    if (isOriginOutsideFaceABC) {

        // Compute the closest point on this triangle face
        computeClosestPointOnTriangle(a, b, c, tempUsedVertices, triangleBaryCoords);
        Vector3 closestPoint = triangleBaryCoords[0] * a + triangleBaryCoords[1] * b +
                               triangleBaryCoords[2] * c;
        decimal squareDist = closestPoint.lengthSquare();

        // If the point on that face is the closest to the origin so far
        if (squareDist < closestSquareDistance) {

            // Use it as the current closest point
            closestSquareDistance = squareDist;
            baryCoordsAB.setAllValues(triangleBaryCoords[0], triangleBaryCoords[1]);
            baryCoordsCD.setAllValues(triangleBaryCoords[2], 0.0);
            bitsUsedPoints = tempUsedVertices;
        }
    }

    // If the origin is outside face ACD
    if (isOriginOutsideFaceACD) {

        // Compute the closest point on this triangle face
        computeClosestPointOnTriangle(a, c, d, tempUsedVertices, triangleBaryCoords);
        Vector3 closestPoint = triangleBaryCoords[0] * a + triangleBaryCoords[1] * c +
                               triangleBaryCoords[2] * d;
        decimal squareDist = closestPoint.lengthSquare();

        // If the point on that face is the closest to the origin so far
        if (squareDist < closestSquareDistance) {

            // Use it as the current closest point
            closestSquareDistance = squareDist;
            baryCoordsAB.setAllValues(triangleBaryCoords[0], 0.0);
            baryCoordsCD.setAllValues(triangleBaryCoords[1], triangleBaryCoords[2]);
            bitsUsedPoints = mapTriangleUsedVerticesToTetrahedron(tempUsedVertices, 0, 2, 3);
        }
    }

    // If the origin is outside face
    if (isOriginOutsideFaceADB) {

        // Compute the closest point on this triangle face
        computeClosestPointOnTriangle(a, d, b, tempUsedVertices, triangleBaryCoords);
        Vector3 closestPoint = triangleBaryCoords[0] * a + triangleBaryCoords[1] * d +
                               triangleBaryCoords[2] * b;
        decimal squareDist = closestPoint.lengthSquare();

        // If the point on that face is the closest to the origin so far
        if (squareDist < closestSquareDistance) {

            // Use it as the current closest point
            closestSquareDistance = squareDist;
            baryCoordsAB.setAllValues(triangleBaryCoords[0], triangleBaryCoords[2]);
            baryCoordsCD.setAllValues(0.0, triangleBaryCoords[1]);
            bitsUsedPoints = mapTriangleUsedVerticesToTetrahedron(tempUsedVertices, 0, 3, 1);
        }
    }

    // If the origin is outside face
    if (isOriginOutsideFaceBDC) {

        // Compute the closest point on this triangle face
        computeClosestPointOnTriangle(b, d, c, tempUsedVertices, triangleBaryCoords);
        Vector3 closestPoint = triangleBaryCoords[0] * b + triangleBaryCoords[1] * d +
                               triangleBaryCoords[2] * c;
        decimal squareDist = closestPoint.lengthSquare();

        // If the point on that face is the closest to the origin so far
        if (squareDist < closestSquareDistance) {

            // Use it as the current closest point
            baryCoordsAB.setAllValues(0.0, triangleBaryCoords[0]);
            baryCoordsCD.setAllValues(triangleBaryCoords[2], triangleBaryCoords[1]);
            bitsUsedPoints = mapTriangleUsedVerticesToTetrahedron(tempUsedVertices, 1, 3, 2);
        }
    }

    return true;
}

// 0111 (1,2,3) => 0111
// 0011 (2,1,3) =>
int VoronoiSimplex::mapTriangleUsedVerticesToTetrahedron(int triangleUsedVertices, int first, int second, int third) const {

    assert(triangleUsedVertices <= 7);
    int tetrahedronUsedVertices = (((1 & triangleUsedVertices) != 0) << first) |
                                  (((2 & triangleUsedVertices) != 0) << second) |
                                  (((4 & triangleUsedVertices) != 0) << third);

    assert(tetrahedronUsedVertices <= 14);
    return tetrahedronUsedVertices;
}

// Test if a point is outside of the plane given by the points of the triangle (a, b, c)
/// This method returns 1 if the point d and the origin are on opposite sides of
/// the triangle face (a, b, c). It returns 0 if they are on the same side.
/// This implementation is based on the one in the book
/// "Real-Time Collision Detection" by Christer Ericson.
int VoronoiSimplex::testOriginOutsideOfPlane(const Vector3& a, const Vector3& b, const Vector3& c, const Vector3& d) const {

    // Normal of (a,b,c) triangle
    const Vector3 n = (b-a).cross(c-a);

    decimal signp = (-a).dot(n);
    decimal signd = (d-a).dot(n);

    // If the tetrahedron is degenerate (all points on the same plane)
    // This should never happen because after a point is added into the simplex, the user
    // of this class must check that the simplex is not affinely dependent using the
    // isAffinelyDependent() method
    if(signd * signd < epsilon * epsilon) {
        return -1;
    }

    return signp * signd < decimal(0.0);
}

// Backup the closest point
void VoronoiSimplex::backupClosestPointInSimplex(Vector3& v) {
    v = mClosestPoint;
}

// Return the maximum squared length of a point
decimal VoronoiSimplex::getMaxLengthSquareOfAPoint() const {

    decimal maxLengthSquare = decimal(0.0);

    for (int i=0; i<mNbPoints; i++) {
        decimal lengthSquare = mPoints[i].lengthSquare();
        if (lengthSquare > maxLengthSquare) {
            maxLengthSquare = lengthSquare;
        }
    }

    // Return the maximum squared length
    return maxLengthSquare;
}
