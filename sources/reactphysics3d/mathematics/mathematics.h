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
#include "Polygon3D.h"
#include "Quaternion.h"
#include "Segment3D.h"
#include "Vector.h"
#include "Vector3D.h"
#include "constants.h"
#include "exceptions.h"
#include "mathematics_functions.h"
#include <cstdio>
#include <cassert>
#include <cmath>

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

/*
// TODO : Test this method
// Move a set of points by a given vector.
// The method returns a set of points moved by the given vector.
inline std::vector<reactphysics3d::Vector3D> movePoints(const std::vector<reactphysics3d::Vector3D>& points, const reactphysics3d::Vector3D& vector) {
    std::vector<reactphysics3d::Vector3D> result;

    // For each point of the set
    for (unsigned int i=0; i<points.size(); ++i) {
        // Move the point
        result.push_back(points.at(i) + vector);
    }

    // Return the result set of points
    return result;
}
*/

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

#endif
