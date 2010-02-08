/****************************************************************************
* Copyright (C) 2009      Daniel Chappuis                                  *
****************************************************************************
* This file is part of ReactPhysics3D.                                     *
*                                                                          *
* ReactPhysics3D is free software: you can redistribute it and/or modify   *
* it under the terms of the GNU Lesser General Public License as published *
* by the Free Software Foundation, either version 3 of the License, or     *
* (at your option) any later version.                                      *
*                                                                          *
* ReactPhysics3D is distributed in the hope that it will be useful,        *
* but WITHOUT ANY WARRANTY; without even the implied warranty of           *
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the             *
* GNU Lesser General Public License for more details.                      *
*                                                                          *
* You should have received a copy of the GNU Lesser General Public License *
* along with ReactPhysics3D. If not, see <http://www.gnu.org/licenses/>.   *
***************************************************************************/

// TODO : Mathematics library : Check everywhere that in member methods we use attributes access instead of getter and setter.

#ifndef MATHEMATICS_H
#define MATHEMATICS_H

// Libraries
#include "Matrix.h"
#include "Matrix3x3.h"
#include "Quaternion.h"
#include "Vector.h"
#include "Vector3D.h"
#include "constants.h"
#include "exceptions.h"
#include "mathematics_functions.h"
#include <vector>
#include <cstdio>
#include <cassert>
#include <cmath>

// ReactPhysics3D namespace
namespace reactphysics3d {

// ---------- Mathematics functions ---------- //

// TODO : Test this method
// Rotate a vector according to a rotation quaternion.
// The function returns the vector rotated according to the quaternion in argument
inline reactphysics3d::Vector3D rotateVectorWithQuaternion(const reactphysics3d::Vector3D& vector, const reactphysics3d::Quaternion& quaternion) {
    // Convert the vector into a quaternion
    reactphysics3d::Quaternion vectorQuaternion(0, vector);

    // Compute the quaternion rotation result
    reactphysics3d::Quaternion quaternionResult = (quaternion * vectorQuaternion) * quaternion.getInverse();

    // Convert the result quaternion into a vector
    return quaternionResult.vectorV();
}

// TODO : Test this method
// Given two lines (given by the points "point1", "point2" and the vectors "d1" and "d2" that are not parallel, this method returns the values
// "alpha" and "beta" such that the two points P1 and P2 are the two closest point between the two lines and such that
// P1 = point1 + alpha * d1
// P2 = point2 + beta * d2
inline void closestPointsBetweenTwoLines(const reactphysics3d::Vector3D& point1, const reactphysics3d::Vector3D& d1, const reactphysics3d::Vector3D& point2,
                                         const reactphysics3d::Vector3D& d2, double* alpha, double* beta) {

    reactphysics3d::Vector3D r = point1 - point2;
    double a = d1.scalarProduct(d1);
    double b = d1.scalarProduct(d2);
    double c = d1.scalarProduct(r);
    double e = d2.scalarProduct(d2);
    double f = d2.scalarProduct(r);
    double d = a*e-b*b;

    // The two lines must not be parallel
    assert(!reactphysics3d::approxEqual(d, 0.0));

    // Compute the "alpha" and "beta" values
    *alpha = (b*f -c*e)/d;
    *beta = (a*f-b*c)/d;
}

// TODO : Test this method
// This method returns true if the point "P" is on the segment between "segPointA" and "segPointB" and return false otherwise
inline bool isPointOnSegment(const reactphysics3d::Vector3D& segPointA, const reactphysics3d::Vector3D& segPointB, const reactphysics3d::Vector3D& P) {

    // Check if the point P is on the line between "segPointA" and "segPointB"
    reactphysics3d::Vector3D d = segPointB - segPointA;
    reactphysics3d::Vector3D dP = P - segPointA;
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

// TODO : Test this method
// Given two lines in 3D that intersect, this method returns the intersection point between the two lines.
// The first line is given by the point "p1" and the vector "d1", the second line is given by the point "p2" and the vector "d2".
inline reactphysics3d::Vector3D computeLinesIntersection(const reactphysics3d::Vector3D& p1, const reactphysics3d::Vector3D& d1,
                                                         const reactphysics3d::Vector3D& p2, const reactphysics3d::Vector3D& d2) {
    // Computes the two closest points on the lines
    double alpha, beta;
    closestPointsBetweenTwoLines(p1, d1, p2, d2, &alpha, &beta);
    reactphysics3d::Vector3D point1 = p1 + alpha * d1;
    reactphysics3d::Vector3D point2 = p2 + beta * d2;

    // The two points must be very close
    assert((point1-point2).length() <= EPSILON);

    // Return the intersection point (halfway between "point1" and "point2")
    return 0.5 * (point1 + point2);
}

// TODO : Test this method
// Given two segments in 3D that are not parallel and that intersect, this method computes the intersection point between the two segments.
// This method returns the intersection point.
inline reactphysics3d::Vector3D computeNonParallelSegmentsIntersection(const reactphysics3d::Vector3D& seg1PointA, const reactphysics3d::Vector3D& seg1PointB,
                                                                       const reactphysics3d::Vector3D& seg2PointA, const reactphysics3d::Vector3D& seg2PointB) {
    // Determine the lines of both segments
    reactphysics3d::Vector3D d1 = seg1PointB - seg1PointA;
    reactphysics3d::Vector3D d2 = seg2PointB - seg2PointA;

    // The segments must not be parallel
    assert(!d1.isParallelWith(d2));

    // Compute the closet points between the two lines
    double alpha, beta;
    closestPointsBetweenTwoLines(seg1PointA, d1, seg2PointA, d2, &alpha, &beta);
    reactphysics3d::Vector3D point1 = seg1PointA + alpha * d1;
    reactphysics3d::Vector3D point2 = seg2PointA + beta * d2;

    // The closest points have to be on the segments, otherwise there is no intersection between the segments
    assert(isPointOnSegment(seg1PointA, seg1PointB, point1));
    assert(isPointOnSegment(seg2PointA, seg2PointB, point2));

    // If the two closest point aren't very close, there is no intersection between the segments
    reactphysics3d::Vector3D d = point2 - point1;
    assert(d.length() <= EPSILON);

    // They are very close so we return the intersection point (halfway between "point1" and "point2"
    return 0.5 * (point1 + point2);
}


// TODO : Test this method
// Move a set of points by a given vector.
// The method returns a set of points moved by the given vector.
inline std::vector<reactphysics3d::Vector3D> movePoints(const std::vector<reactphysics3d::Vector3D>& points, const reactphysics3d::Vector3D& vector) {
    std::vector<reactphysics3d::Vector3D> result;

    // For each point of the set
    for (unsigned int i=0; i<points.size(); ++i) {
        // Move the point
        result.push_back(points[i] + vector);
    }

    // Return the result set of points
    return result;
}


// TODO : Test this method
// Compute the projection of a set of 3D points onto a 3D plane. The set of points is given by "points" and the plane is given by
// a point "A" and a normal vector "normal". This method returns the initial set of points projected onto the plane.
inline std::vector<reactphysics3d::Vector3D> projectPointsOntoPlane(const std::vector<reactphysics3d::Vector3D>& points, const reactphysics3d::Vector3D& A,
                                                                    const reactphysics3d::Vector3D& normal) {
    assert(normal.length() != 0.0);

    std::vector<Vector3D> projectedPoints;
    reactphysics3d::Vector3D n = normal.getUnit();

    // For each point of the set
    for (unsigned int i=0; i<points.size(); ++i) {
        // Compute the projection of the point onto the plane
        projectedPoints.push_back(points[i] - ((points[i] - A).scalarProduct(n)) * n);
    }

    // Return the projected set of points
    return projectedPoints;
}

// TODO : Test this method
// Compute the distance between a point "P" and a line (given by a point "A" and a vector "v")
inline double computeDistanceBetweenPointAndLine(const reactphysics3d::Vector3D& P, const reactphysics3d::Vector3D& A, const reactphysics3d::Vector3D& v) {
    assert(v.length() != 0);
    return ((P-A).crossProduct(v).length() / (v.length()));
}

// TODO : Test this method
// Compute the orthogonal projection of a point "P" on a line (given by a point "A" and a vector "v")
inline reactphysics3d::Vector3D computeOrthogonalProjectionOfPointOntoALine(const reactphysics3d::Vector3D& P, const reactphysics3d::Vector3D& A, const reactphysics3d::Vector3D& v) {
    return (A + ((P-A).scalarProduct(v) / (v.scalarProduct(v))) * v);
}

// TODO : Test this method
// Compute the intersection between two parallel segments (the first segment is between the points "seg1PointA" and "seg1PointB" and the second
// segment is between the points "seg2PointA" and "seg2PointB"). The result is the segment intersection (represented by the points "resultPointA"
// and "resultPointB". Because the two given segments don't have to be on the same exact line, the result intersection segment will a segment
// halway between the first and the second given segments.
inline void computeParallelSegmentsIntersection(const reactphysics3d::Vector3D& seg1PointA, const reactphysics3d::Vector3D& seg1PointB,
                                                const reactphysics3d::Vector3D& seg2PointA, const reactphysics3d::Vector3D& seg2PointB,
                                                reactphysics3d::Vector3D& resultPointA, reactphysics3d::Vector3D& resultPointB) {
    // Compute the segment vectors
    reactphysics3d::Vector3D d1 = seg1PointB - seg1PointA;
    reactphysics3d::Vector3D d2 = seg2PointB - seg2PointA;

    // The two segments should be parallel
    assert(d1.isParallelWith(d2));

    // Compute the projection of the two points of the second segment onto the vector of segment 1
    double projSeg2PointA = d1.getUnit().scalarProduct(seg2PointA - seg1PointA);
    double projSeg2PointB = d1.getUnit().scalarProduct(seg2PointB - seg1PointA);

    // The projections intervals should intersect
    assert(!(projSeg2PointA < 0.0 && projSeg2PointB < 0.0));
    assert(!(projSeg2PointA > d1.length() && projSeg2PointB > d1.length()));

    // Compute the vector "v" from a point on the line 1 to the orthogonal point of the line 2
    reactphysics3d::Vector3D point = computeOrthogonalProjectionOfPointOntoALine(seg2PointA, seg1PointA, d1);
    reactphysics3d::Vector3D v = seg2PointA - point;

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

// TODO : Test this method
// This method clip a 3D segment with 3D rectangle polygon. The segment and the rectangle are asssumed to be on the same plane. We
// also assume that the segment is not completely outside the clipping rectangle.
// The segment is given by the two vertices in "segment" and the rectangle is given by the ordered vertices in "clipRectangle".
// This method returns the clipped segment.
inline std::vector<reactphysics3d::Vector3D> clipSegmentWithRectangleInPlane(const std::vector<reactphysics3d::Vector3D>& segment, const std::vector<reactphysics3d::Vector3D> clipRectangle) {
    assert(segment.size() == 2);
    assert(clipRectangle.size() == 4);

    std::vector<reactphysics3d::Vector3D> inputSegment = segment;
    std::vector<reactphysics3d::Vector3D> outputSegment;

    // For each edge of the clip rectangle
    for (unsigned int i=0; i<4; ++i) {
        outputSegment.clear();

        // Current clipped segment
        assert(inputSegment.size() == 2);
        reactphysics3d::Vector3D S = inputSegment[0];
        reactphysics3d::Vector3D P = inputSegment[1];

        // Edge of the clip rectangle
        reactphysics3d::Vector3D A = clipRectangle[i];
        reactphysics3d::Vector3D B = clipRectangle[ (i+1) % 4];
        reactphysics3d::Vector3D planeNormal = clipRectangle[(i+2) % 4] - clipRectangle[(i+1) % 4];

        // If the point P is inside the clip plane
        if (planeNormal.scalarProduct(P-A) >= 0.0) {
            // If the point S is inside the clip plane
            if (planeNormal.scalarProduct(S-A) >= 0.0) {
                outputSegment.push_back(P);
                outputSegment.push_back(S);
            }
            else {  // P is inside and S is outside the clip plane
                // Compute the intersection point between the segment SP and the clip plane
                reactphysics3d::Vector3D intersectPoint = computeLinesIntersection(S, P-S, A, B-A);

                outputSegment.push_back(P);
                outputSegment.push_back(intersectPoint);
            }
        }
        else if (planeNormal.scalarProduct(S-A) > 0.0) {    // P is outside and S is inside the clip plane
                // Compute the intersection point between the segment SP and the clip plane
                reactphysics3d::Vector3D intersectPoint = computeLinesIntersection(S, P-S, A, B-A);

                outputSegment.push_back(S);
                outputSegment.push_back(intersectPoint);
        }

        inputSegment = outputSegment;
    }

    // Return the clipped segment
    return outputSegment;
}

// TODO : Test this method
// This method uses the Sutherland-Hodgman clipping algorithm to clip a subject polygon (given by the ordered 3D vertices in "subjectPolygon") using
// a rectangle polygon (given by the ordered 3D vertices in "clipRectangle"). The subject polygon and the clip rectangle are in 3D but we assumed that
// they are on a same plane in 3D. The method returns the ordered 3D vertices of the subject polygon clipped using the rectangle polygon.
inline std::vector<reactphysics3d::Vector3D> clipPolygonWithRectangleInPlane(const std::vector<reactphysics3d::Vector3D>& subjectPolygon, const std::vector<reactphysics3d::Vector3D>& clipRectangle) {
    assert(clipRectangle.size() == 4);

    std::vector<reactphysics3d::Vector3D> outputPolygon;
    std::vector<reactphysics3d::Vector3D> inputPolygon = subjectPolygon;

    // For each edge of the clip rectangle
    for (unsigned int i=0; i<4; ++i) {
        outputPolygon.clear();

        // Each edge defines a clip plane. The clip plane is define by a point on this plane (a vertice of the current edge) and
        // a plane normal (because we are using a clip rectangle, the plane normal is the next edge of the clip rectangle).
        reactphysics3d::Vector3D planeNormal = clipRectangle[(i+2) % 4] - clipRectangle[(i+1) % 4];
        reactphysics3d::Vector3D A = clipRectangle[i];          // Segment AB is the current segment of the "clipRectangle"
        reactphysics3d::Vector3D B = clipRectangle[(i+1) % 4];
        reactphysics3d::Vector3D S = inputPolygon[0];

        // For each vertex of the subject polygon
        for (unsigned int j=0; j<inputPolygon.size(); ++j) {
            reactphysics3d::Vector3D P = inputPolygon[(j+1) % inputPolygon.size()];

            // If the point P is inside the clip plane
            if (planeNormal.scalarProduct(P-A) >= 0.0) {
                // If the point S is also inside the clip plane
                if (planeNormal.scalarProduct(S-A) >= 0.0) {
                    outputPolygon.push_back(P);
                }
                else {  // If the point S is outside the clip plane
                    // Compute the intersection point between the segment SP and the clip plane
                    reactphysics3d::Vector3D intersectPoint = computeLinesIntersection(S, P-S, A, B-A);

                    outputPolygon.push_back(intersectPoint);
                    outputPolygon.push_back(P);
                }
            }
            else if (planeNormal.scalarProduct(S-A) > 0.0) {
                    // Compute the intersection point between the segment SP and the clip plane
                    reactphysics3d::Vector3D intersectPoint = computeLinesIntersection(S, P-S, A, B-A);

                    outputPolygon.push_back(intersectPoint);
            }
            S = P;
        }
        inputPolygon = outputPolygon;
    }

    // Return the clipped polygon
    return outputPolygon;
}

} // End of the ReactPhysics3D namespace


#endif
