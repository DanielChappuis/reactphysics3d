/********************************************************************************
* ReactPhysics3D physics library, http://code.google.com/p/reactphysics3d/      *
* Copyright (c) 2010 Daniel Chappuis                                            *
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

#ifndef MATHEMATICS_H
#define MATHEMATICS_H

// Libraries
#include "Matrix3x3.h"
#include "Quaternion.h"
#include "Vector3.h"
#include "Transform.h"
#include "../constants.h"
#include "exceptions.h"
#include "mathematics_functions.h"
#include <vector>
#include <cstdio>
#include <cassert>
#include <cmath>

// ReactPhysics3D namespace
namespace reactphysics3d {

// ---------- Mathematics functions ---------- //

// Rotate a vector according to a rotation quaternion.
// The function returns the vector rotated according to the quaternion in argument
inline reactphysics3d::Vector3 rotateVectorWithQuaternion(const reactphysics3d::Vector3& vector, const reactphysics3d::Quaternion& quaternion) {
    // Convert the vector into a quaternion
    reactphysics3d::Quaternion vectorQuaternion(0, vector);

    // Compute the quaternion rotation result
    reactphysics3d::Quaternion quaternionResult = (quaternion * vectorQuaternion) * quaternion.getInverse();

    // Convert the result quaternion into a vector
    return quaternionResult.vectorV();
}

// Given two lines (given by the points "point1", "point2" and the vectors "d1" and "d2" that are not parallel, this method returns the values
// "alpha" and "beta" such that the two points P1 and P2 are the two closest point between the two lines and such that
// P1 = point1 + alpha * d1
// P2 = point2 + beta * d2
inline void closestPointsBetweenTwoLines(const reactphysics3d::Vector3& point1, const reactphysics3d::Vector3& d1, const reactphysics3d::Vector3& point2,
                                         const reactphysics3d::Vector3& d2, double* alpha, double* beta) {

    reactphysics3d::Vector3 r = point1 - point2;
    double a = d1.dot(d1);
    double b = d1.dot(d2);
    double c = d1.dot(r);
    double e = d2.dot(d2);
    double f = d2.dot(r);
    double d = a*e-b*b;

    // The two lines must not be parallel
    assert(!reactphysics3d::approxEqual(d, 0.0));

    // Compute the "alpha" and "beta" values
    *alpha = (b*f -c*e)/d;
    *beta = (a*f-b*c)/d;
}

// This method returns true if the point "P" is on the segment between "segPointA" and "segPointB" and return false otherwise
inline bool isPointOnSegment(const reactphysics3d::Vector3& segPointA, const reactphysics3d::Vector3& segPointB, const reactphysics3d::Vector3& P) {

    // Check if the point P is on the line between "segPointA" and "segPointB"
    reactphysics3d::Vector3 d = segPointB - segPointA;
    reactphysics3d::Vector3 dP = P - segPointA;
    if (!d.isParallelWith(dP)) {
        return false;
    }

    // Compute the length of the segment
    double segmentLength = d.length();

    // Compute the distance from point "P" to points "segPointA" and "segPointB"
    double distA = dP.length();
    double distB = (P - segPointB).length();

    // If one of the "distA" and "distB" is greather than the length of the segment, then P is not on the segment
    if (distA > segmentLength || distB > segmentLength) {
        return false;
    }

    // Otherwise, the point P is on the segment
    return true;
}


// Given two lines in 3D that intersect, this method returns the intersection point between the two lines.
// The first line is given by the point "p1" and the vector "d1", the second line is given by the point "p2" and the vector "d2".
inline reactphysics3d::Vector3 computeLinesIntersection(const reactphysics3d::Vector3& p1, const reactphysics3d::Vector3& d1,
                                                         const reactphysics3d::Vector3& p2, const reactphysics3d::Vector3& d2) {
    // Computes the two closest points on the lines
    double alpha, beta;
    closestPointsBetweenTwoLines(p1, d1, p2, d2, &alpha, &beta);
    reactphysics3d::Vector3 point1 = p1 + alpha * d1;
    reactphysics3d::Vector3 point2 = p2 + beta * d2;

    // The two points must be very close
    //assert((point1-point2).length() <= 0.1);

    // Return the intersection point (halfway between "point1" and "point2")
    return 0.5 * (point1 + point2);
}


// Given two segments in 3D that are not parallel and that intersect, this method computes the intersection point between the two segments.
// This method returns the intersection point.
inline reactphysics3d::Vector3 computeNonParallelSegmentsIntersection(const reactphysics3d::Vector3& seg1PointA, const reactphysics3d::Vector3& seg1PointB,
                                                                       const reactphysics3d::Vector3& seg2PointA, const reactphysics3d::Vector3& seg2PointB) {
    // Determine the lines of both segments
    reactphysics3d::Vector3 d1 = seg1PointB - seg1PointA;
    reactphysics3d::Vector3 d2 = seg2PointB - seg2PointA;

    // The segments must not be parallel
    assert(!d1.isParallelWith(d2));

    // Compute the closet points between the two lines
    double alpha, beta;
    closestPointsBetweenTwoLines(seg1PointA, d1, seg2PointA, d2, &alpha, &beta);
    reactphysics3d::Vector3 point1 = seg1PointA + alpha * d1;
    reactphysics3d::Vector3 point2 = seg2PointA + beta * d2;

    // The closest points have to be on the segments, otherwise there is no intersection between the segments
    assert(isPointOnSegment(seg1PointA, seg1PointB, point1));
    assert(isPointOnSegment(seg2PointA, seg2PointB, point2));

    // If the two closest point aren't very close, there is no intersection between the segments
    reactphysics3d::Vector3 d = point2 - point1;
    assert(d.length() <= EPSILON_TEST);

    // They are very close so we return the intersection point (halfway between "point1" and "point2"
    return 0.5 * (point1 + point2);
}


// Move a set of points by a given vector.
// The method returns a set of points moved by the given vector.
inline std::vector<reactphysics3d::Vector3> movePoints(const std::vector<reactphysics3d::Vector3>& points, const reactphysics3d::Vector3& vector) {
    std::vector<reactphysics3d::Vector3> result;

    // For each point of the set
    for (unsigned int i=0; i<points.size(); ++i) {
        // Move the point
        result.push_back(points[i] + vector);
    }

    // Return the result set of points
    return result;
}


// Compute the projection of a set of 3D points onto a 3D plane. The set of points is given by "points" and the plane is given by
// a point "A" and a normal vector "normal". This method returns the initial set of points projected onto the plane.
inline std::vector<reactphysics3d::Vector3> projectPointsOntoPlane(const std::vector<reactphysics3d::Vector3>& points, const reactphysics3d::Vector3& A,
                                                                    const reactphysics3d::Vector3& normal) {
    assert(normal.length() != 0.0);

    std::vector<Vector3> projectedPoints;
    reactphysics3d::Vector3 n = normal.getUnit();

    // For each point of the set
    for (unsigned int i=0; i<points.size(); ++i) {
        // Compute the projection of the point onto the plane
        projectedPoints.push_back(points[i] - (n * (points[i] - A).dot(n)));

    }

    // Return the projected set of points
    return projectedPoints;
}


// Compute the distance between a point "P" and a line (given by a point "A" and a vector "v")
inline double computeDistanceBetweenPointAndLine(const reactphysics3d::Vector3& P, const reactphysics3d::Vector3& A, const reactphysics3d::Vector3& v) {
    assert(v.length() != 0);
    return ((P-A).cross(v).length() / (v.length()));
}


// Compute the orthogonal projection of a point "P" on a line (given by a point "A" and a vector "v")
inline reactphysics3d::Vector3 computeOrthogonalProjectionOfPointOntoALine(const reactphysics3d::Vector3& P, const reactphysics3d::Vector3& A, const reactphysics3d::Vector3& v) {
    return (A + ((P-A).dot(v) / (v.dot(v))) * v);
}

// Given a point P and 4 points that form a rectangle (point P and the 4 points have to be on the same plane) this method computes
// the point Q that is the nearest point to P that is inside (on a border of) the rectangle. The point P should be outside the rectangle.
// The result point Q will be in a segment of the rectangle
inline reactphysics3d::Vector3 computeNearestPointOnRectangle(const reactphysics3d::Vector3& P, const std::vector<reactphysics3d::Vector3> rectangle) {
    assert(rectangle.size() == 4);
    double distPSegment1 = computeDistanceBetweenPointAndLine(P, rectangle[0], rectangle[1] - rectangle[0]);
    double distPSegment2 = computeDistanceBetweenPointAndLine(P, rectangle[1], rectangle[2] - rectangle[1]);
    double distPSegment3 = computeDistanceBetweenPointAndLine(P, rectangle[2], rectangle[3] - rectangle[2]);
    double distPSegment4 = computeDistanceBetweenPointAndLine(P, rectangle[3], rectangle[0] - rectangle[3]);
    double distSegment1Segment3 = computeDistanceBetweenPointAndLine(rectangle[0], rectangle[3], rectangle[3] - rectangle[2]);
    double distSegment2Segment4 = computeDistanceBetweenPointAndLine(rectangle[1], rectangle[3], rectangle[0] - rectangle[3]);
    Vector3 resultPoint;
    
    // Check if P is between the lines of the first pair of parallel segments of the rectangle
    if (distPSegment1 <= distSegment1Segment3 && distPSegment3 <= distSegment1Segment3) {
        // Find among segments 2 and 4 which one is the nearest
        if (distPSegment2 <= distPSegment4) { // Segment 2 is the nearest
            // We compute the projection of the point P onto the segment 2
            resultPoint = computeOrthogonalProjectionOfPointOntoALine(P, rectangle[1], rectangle[2] - rectangle[1]);
        }
        else {  // Segment 4 is the nearest
            // We compute the projection of the point P onto the segment 4
            resultPoint = computeOrthogonalProjectionOfPointOntoALine(P, rectangle[3], rectangle[0] - rectangle[3]);
        }
    }
    // Check if P is between the lines of the second pair of parallel segments of the rectangle
    else if (distPSegment2 <= distSegment2Segment4 && distPSegment4 <= distSegment2Segment4) {
        // Find among segments 1 and 3 which one is the nearest
        if (distPSegment1 <= distPSegment3) { // Segment 1 is the nearest
            // We compute the projection of the point P onto the segment 1
            resultPoint = computeOrthogonalProjectionOfPointOntoALine(P, rectangle[0], rectangle[1] - rectangle[0]);
        }
        else {  // Segment 3 is the nearest
            // We compute the projection of the point P onto the segment 3
            resultPoint = computeOrthogonalProjectionOfPointOntoALine(P, rectangle[2], rectangle[3] - rectangle[2]);
        }
    }
    else if (distPSegment4 <= distPSegment2) {
        if (distPSegment1 <= distPSegment3) { // The point P is in the corner of point rectangle[0]
            // Return the corner of the rectangle
            return rectangle[0];
        }
        else { // The point P is in the corner of point rectangle[3]
            // Return the corner of the rectangle
            return rectangle[3];
        }
    }
    else {
        if (distPSegment1 <= distPSegment3) { // The point P is in the corner of point rectangle[1]
            // Return the corner of the rectangle
            return rectangle[1];
        }
        else { // The point P is in the corner of point rectangle[2]
            // Return the corner of the rectangle
            return rectangle[2];
        }
    }

    // Return the result point
    return resultPoint;
}

// Compute the intersection between two parallel segments (the first segment is between the points "seg1PointA" and "seg1PointB" and the second
// segment is between the points "seg2PointA" and "seg2PointB"). The result is the segment intersection (represented by the points "resultPointA"
// and "resultPointB". Because the two given segments don't have to be on the same exact line, the result intersection segment will a segment
// halway between the first and the second given segments.
inline void computeParallelSegmentsIntersection(const reactphysics3d::Vector3& seg1PointA, const reactphysics3d::Vector3& seg1PointB,
                                                const reactphysics3d::Vector3& seg2PointA, const reactphysics3d::Vector3& seg2PointB,
                                                reactphysics3d::Vector3& resultPointA, reactphysics3d::Vector3& resultPointB) {
    // Compute the segment vectors
    reactphysics3d::Vector3 d1 = seg1PointB - seg1PointA;
    reactphysics3d::Vector3 d2 = seg2PointB - seg2PointA;

    // The two segments should be parallel
    assert(d1.isParallelWith(d2));

    // Compute the projection of the two points of the second segment onto the vector of segment 1
    double projSeg2PointA = d1.getUnit().dot(seg2PointA - seg1PointA);
    double projSeg2PointB = d1.getUnit().dot(seg2PointB - seg1PointA);

    // The projections intervals should intersect
    assert(!(projSeg2PointA < 0.0 && projSeg2PointB < 0.0));
    assert(!(projSeg2PointA > d1.length() && projSeg2PointB > d1.length()));

    // Compute the vector "v" from a point on the line 1 to the orthogonal point of the line 2
    reactphysics3d::Vector3 point = computeOrthogonalProjectionOfPointOntoALine(seg2PointA, seg1PointA, d1);
    reactphysics3d::Vector3 v = seg2PointA - point;

    // Return the segment intersection according to the configuration of two projection intervals
    if (projSeg2PointA >= 0 && projSeg2PointA <= d1.length() && projSeg2PointB >= d1.length()) {
        // Move the contact points halfway between the two segments
        resultPointA = seg2PointA - 0.5 * v;
        resultPointB = seg1PointB + 0.5 * v;
    }
    else if (projSeg2PointA <= 0 && projSeg2PointB >= 0 && projSeg2PointB <= d1.length()) {
        // Move the contact points halfway between the two segments
        resultPointA = seg1PointA + 0.5 * v;
        resultPointB = seg2PointB - 0.5 * v;
    }
    else if (projSeg2PointA <= 0 && projSeg2PointB >= d1.length()) {
        // Move the contact points halfway between the two segments
        resultPointA = seg1PointA + 0.5 * v;
        resultPointB = seg1PointB + 0.5 * v;
    }
    else if (projSeg2PointA <= d1.length() && projSeg2PointB <= d1.length()) {
        // Move the contact points halfway between the two segments
        resultPointA = seg2PointA - 0.5 * v;
        resultPointB = seg2PointB - 0.5 * v;
    }
}

// This method clip a 3D segment with 3D rectangle polygon. The segment and the rectangle are asssumed to be on the same plane. We
// also assume that the segment is not completely outside the clipping rectangle.
// The segment is given by the two vertices in "segment" and the rectangle is given by the ordered vertices in "clipRectangle".
// This method returns the clipped segment.
inline std::vector<reactphysics3d::Vector3> clipSegmentWithRectangleInPlane(const std::vector<reactphysics3d::Vector3>& segment, const std::vector<reactphysics3d::Vector3> clipRectangle) {
    double const epsilon = 0.01;

    assert(segment.size() == 2);
    assert(clipRectangle.size() == 4);

    std::vector<reactphysics3d::Vector3> inputSegment = segment;
    std::vector<reactphysics3d::Vector3> outputSegment;
    
    // For each edge of the clip rectangle
    for (unsigned int i=0; i<4; ++i) {
        outputSegment.clear();

        // Current clipped segment
        //assert(inputSegment.size() == 2);
        reactphysics3d::Vector3 S = inputSegment[0];
        reactphysics3d::Vector3 P = inputSegment[1];

        // Edge of the clip rectangle
        reactphysics3d::Vector3 A = clipRectangle[i];
        reactphysics3d::Vector3 B = clipRectangle[ (i+1) % 4];
        reactphysics3d::Vector3 planeNormal = clipRectangle[(i+2) % 4] - clipRectangle[(i+1) % 4];

        // If the point P is inside the clip plane
        if (planeNormal.dot(P-A) >= 0.0 - epsilon) {
            // If the point S is inside the clip plane
            if (planeNormal.dot(S-A) >= 0.0 - epsilon) {
                outputSegment.push_back(P);
                outputSegment.push_back(S);
            }
            else {  // P is inside and S is outside the clip plane
                // Compute the intersection point between the segment SP and the clip plane
                reactphysics3d::Vector3 intersectPoint = computeLinesIntersection(S, P-S, A, B-A);

                outputSegment.push_back(P);
                outputSegment.push_back(intersectPoint);
            }
        }
        else if (planeNormal.dot(S-A) > 0.0 - epsilon) {    // P is outside and S is inside the clip plane
                // Compute the intersection point between the segment SP and the clip plane
                reactphysics3d::Vector3 intersectPoint = computeLinesIntersection(S, P-S, A, B-A);

                outputSegment.push_back(S);
                outputSegment.push_back(intersectPoint);
        }

        inputSegment = outputSegment;
    }

    // Return the clipped segment
    return outputSegment;
}

// This method uses the Sutherland-Hodgman clipping algorithm to clip a subject polygon (given by the ordered 3D vertices in "subjectPolygon") using
// a rectangle polygon (given by the ordered 3D vertices in "clipRectangle"). The subject polygon and the clip rectangle are in 3D but we assumed that
// they are on a same plane in 3D. The method returns the ordered 3D vertices of the subject polygon clipped using the rectangle polygon.
inline std::vector<reactphysics3d::Vector3> clipPolygonWithRectangleInPlane(const std::vector<reactphysics3d::Vector3>& subjectPolygon, const std::vector<reactphysics3d::Vector3>& clipRectangle) {
    double const epsilon = 0.1;
    assert(clipRectangle.size() == 4);

    std::vector<reactphysics3d::Vector3> outputPolygon;
    std::vector<reactphysics3d::Vector3> inputPolygon = subjectPolygon;

    // For each edge of the clip rectangle
    for (unsigned int i=0; i<4; ++i) {
        outputPolygon.clear();

        // Each edge defines a clip plane. The clip plane is define by a point on this plane (a vertice of the current edge) and
        // a plane normal (because we are using a clip rectangle, the plane normal is the next edge of the clip rectangle).
        reactphysics3d::Vector3 planeNormal = clipRectangle[(i+2) % 4] - clipRectangle[(i+1) % 4];
        reactphysics3d::Vector3 A = clipRectangle[i];          // Segment AB is the current segment of the "clipRectangle"
        reactphysics3d::Vector3 B = clipRectangle[(i+1) % 4];
        reactphysics3d::Vector3 S = inputPolygon[0];

        // For each vertex of the subject polygon
        for (unsigned int j=0; j<inputPolygon.size(); ++j) {
            reactphysics3d::Vector3 P = inputPolygon[(j+1) % inputPolygon.size()];

            // If the point P is inside the clip plane
            double test = planeNormal.dot(P-A);
            if (planeNormal.dot(P-A) >= 0.0 - epsilon) {
                // If the point S is also inside the clip plane
                if (planeNormal.dot(S-A) >= 0.0 - epsilon) {
                    outputPolygon.push_back(P);
                }
                else {  // If the point S is outside the clip plane
                    // Compute the intersection point between the segment SP and the clip plane
                    reactphysics3d::Vector3 intersectPoint = computeLinesIntersection(S, P-S, A, B-A);

                    outputPolygon.push_back(intersectPoint);
                    outputPolygon.push_back(P);
                }
            }
            else if (planeNormal.dot(S-A) > 0.0) {
                // Compute the intersection point between the segment SP and the clip plane
                reactphysics3d::Vector3 intersectPoint = computeLinesIntersection(S, P-S, A, B-A);

                outputPolygon.push_back(intersectPoint);
            }
            S = P;
        }
        inputPolygon = outputPolygon;
    }

    // Return the clipped polygon
    return outputPolygon;
}

// Compute the intersection point between a line and a plane in 3D space. There must be an intersection, therefore the
// the lineVector must not be orthogonal to the planeNormal.
inline reactphysics3d::Vector3 intersectLineWithPlane(const reactphysics3d::Vector3& linePoint, const reactphysics3d::Vector3& lineVector,
                                                       const reactphysics3d::Vector3& planePoint, const reactphysics3d::Vector3& planeNormal) {
    assert(!approxEqual(lineVector.dot(planeNormal), 0.0));

    // The plane is represented by the equation planeNormal dot X = d where X is a point of the plane
    double d  = planeNormal.dot(planePoint);

    // Compute the parameter t
    double t = (d - planeNormal.dot(linePoint)) / planeNormal.dot(lineVector);

    // Compute the intersection point
    return linePoint + lineVector * t;
}


} // End of the ReactPhysics3D namespace


#endif
