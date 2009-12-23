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

// TODO : Test this method
// Return the intersection between two Segment3D that are on the same line. The result of the intersection
// will be another Segment3D.
inline reactphysics3d::Segment3D computeParallelSegmentsIntersection(const reactphysics3d::Segment3D& segment1, const reactphysics3d::Segment3D& segment2) {
    // The two segments should be colinear
    assert(approxEqual(std::abs(segment1.getSegmentVector().scalarProduct(segment2.getSegmentVector())), segment1.getSegmentVector().length() * segment2.getSegmentVector().length()));

    // Result segment
    reactphysics3d::Segment3D resultSegment;

    // Compute the vector of the line where both segments are
    reactphysics3d::Vector3D lineVector = segment1.getSegmentVector();

    // Compute the projection of the two points of the second segment on the line
    double projSegment2PointA = lineVector.getUnit().scalarProduct(segment2.getPointA() - segment1.getPointA());
    double projSegment2PointB = lineVector.getUnit().scalarProduct(segment2.getPointB() - segment1.getPointA());

    std::cout << "Segment 2 - Proj A : " << projSegment2PointA << std::endl;
    std::cout << "Segment 2 - Proj B : " << projSegment2PointB << std::endl;
    std::cout << "Segment 1 - length : " << segment1.getLength() << std::endl;

    // The projections intervals should intersect
    assert(!(projSegment2PointA < 0.0 && projSegment2PointB < 0.0));
    assert(!(projSegment2PointA > segment1.getLength() && projSegment2PointB > segment1.getLength()));

    // Return the segment intersection according to the configuration of two projection intervals
    if (projSegment2PointA >= 0 && projSegment2PointA <= segment1.getLength() && projSegment2PointB >= segment1.getLength()) {
        resultSegment.setPointA(segment2.getPointA());
        resultSegment.setPointB(segment1.getPointB());
        return resultSegment;
    }
    else if (projSegment2PointA <= 0 && projSegment2PointB >= 0 && projSegment2PointB <= segment1.getLength()) {
        resultSegment.setPointA(segment1.getPointA());
        resultSegment.setPointB(segment2.getPointB());
        return resultSegment;
    }
    else if (projSegment2PointA <= 0 && projSegment2PointB >= segment1.getLength()) {
        return segment1;
    }
    else if (projSegment2PointA <= segment1.getLength() && projSegment2PointB <= segment1.getLength()) {
        return segment2;
    }

    // We should never go here
    assert(false);
}
*/

#endif
