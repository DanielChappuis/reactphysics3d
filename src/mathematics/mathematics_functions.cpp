/********************************************************************************
* ReactPhysics3D physics library, http://www.reactphysics3d.com                 *
* Copyright (c) 2010-2019 Daniel Chappuis                                       *
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
#include "mathematics_functions.h"
#include "Vector3.h"
#include "Vector2.h"
#include <cassert>

using namespace reactphysics3d;


// Function to test if two vectors are (almost) equal
bool reactphysics3d::approxEqual(const Vector3& vec1, const Vector3& vec2, decimal epsilon) {
    return approxEqual(vec1.x, vec2.x, epsilon) && approxEqual(vec1.y, vec2.y, epsilon) &&
           approxEqual(vec1.z, vec2.z, epsilon);
}

// Function to test if two vectors are (almost) equal
bool reactphysics3d::approxEqual(const Vector2& vec1, const Vector2& vec2, decimal epsilon) {
    return approxEqual(vec1.x, vec2.x, epsilon) && approxEqual(vec1.y, vec2.y, epsilon);
}

// Compute the barycentric coordinates u, v, w of a point p inside the triangle (a, b, c)
// This method uses the technique described in the book Real-Time collision detection by
// Christer Ericson.
void reactphysics3d::computeBarycentricCoordinatesInTriangle(const Vector3& a, const Vector3& b, const Vector3& c,
                                             const Vector3& p, decimal& u, decimal& v, decimal& w) {
    const Vector3 v0 = b - a;
    const Vector3 v1 = c - a;
    const Vector3 v2 = p - a;

    decimal d00 = v0.dot(v0);
    decimal d01 = v0.dot(v1);
    decimal d11 = v1.dot(v1);
    decimal d20 = v2.dot(v0);
    decimal d21 = v2.dot(v1);

    decimal denom = d00 * d11 - d01 * d01;
    v = (d11 * d20 - d01 * d21) / denom;
    w = (d00 * d21 - d01 * d20) / denom;
    u = decimal(1.0) - v - w;
}

// Clamp a vector such that it is no longer than a given maximum length
Vector3 reactphysics3d::clamp(const Vector3& vector, decimal maxLength) {
    if (vector.lengthSquare() > maxLength * maxLength) {
        return vector.getUnit() * maxLength;
    }
    return vector;
}

// Return true if two vectors are parallel
bool reactphysics3d::areParallelVectors(const Vector3& vector1, const Vector3& vector2) {
    return vector1.cross(vector2).lengthSquare() < decimal(0.00001);
}

// Return true if two vectors are orthogonal
bool reactphysics3d::areOrthogonalVectors(const Vector3& vector1, const Vector3& vector2) {
    return std::abs(vector1.dot(vector2)) < decimal(0.001);
}

// Compute and return a point on segment from "segPointA" and "segPointB" that is closest to point "pointC"
Vector3 reactphysics3d::computeClosestPointOnSegment(const Vector3& segPointA, const Vector3& segPointB, const Vector3& pointC) {

	const Vector3 ab = segPointB - segPointA;

	decimal abLengthSquare = ab.lengthSquare();

	// If the segment has almost zero length
	if (abLengthSquare < MACHINE_EPSILON) {

		// Return one end-point of the segment as the closest point
		return segPointA;
	}

	// Project point C onto "AB" line
	decimal t = (pointC - segPointA).dot(ab) / abLengthSquare;

	// If projected point onto the line is outside the segment, clamp it to the segment
	if (t < decimal(0.0)) t = decimal(0.0);
	if (t > decimal(1.0)) t = decimal(1.0);

	// Return the closest point on the segment
	return segPointA + t * ab;
}

// Compute the closest points between two segments
// This method uses the technique described in the book Real-Time
// collision detection by Christer Ericson.
void reactphysics3d::computeClosestPointBetweenTwoSegments(const Vector3& seg1PointA, const Vector3& seg1PointB,
										   const Vector3& seg2PointA, const Vector3& seg2PointB,
										   Vector3& closestPointSeg1, Vector3& closestPointSeg2) {

	const Vector3 d1 = seg1PointB - seg1PointA;
	const Vector3 d2 = seg2PointB - seg2PointA;
	const Vector3 r = seg1PointA - seg2PointA;
	decimal a = d1.lengthSquare();
	decimal e = d2.lengthSquare();
	decimal f = d2.dot(r);
	decimal s, t;

	// If both segments degenerate into points
	if (a <= MACHINE_EPSILON && e <= MACHINE_EPSILON) {

		closestPointSeg1 = seg1PointA;
		closestPointSeg2 = seg2PointA;
		return;
	}
	if (a <= MACHINE_EPSILON) {   // If first segment degenerates into a point
		
		s = decimal(0.0);

		// Compute the closest point on second segment
		t = clamp(f / e, decimal(0.0), decimal(1.0));
	}
	else {

		decimal c = d1.dot(r);

		// If the second segment degenerates into a point
		if (e <= MACHINE_EPSILON) {

			t = decimal(0.0);
			s = clamp(-c / a, decimal(0.0), decimal(1.0));
		}
		else {

			decimal b = d1.dot(d2);
			decimal denom = a * e - b * b;

			// If the segments are not parallel
			if (denom != decimal(0.0)) {

				// Compute the closest point on line 1 to line 2 and
				// clamp to first segment.
				s = clamp((b * f - c * e) / denom, decimal(0.0), decimal(1.0));
			}
			else {

				// Pick an arbitrary point on first segment
				s = decimal(0.0);
			}

			// Compute the point on line 2 closest to the closest point
			// we have just found
			t = (b * s + f) / e;

			// If this closest point is inside second segment (t in [0, 1]), we are done.
			// Otherwise, we clamp the point to the second segment and compute again the
			// closest point on segment 1
			if (t < decimal(0.0)) {
				t = decimal(0.0);
				s = clamp(-c / a, decimal(0.0), decimal(1.0));
			}
			else if (t > decimal(1.0)) {
				t = decimal(1.0);
				s = clamp((b - c) / a, decimal(0.0), decimal(1.0));
			}
		}
	}

	// Compute the closest points on both segments
	closestPointSeg1 = seg1PointA + d1 * s;
	closestPointSeg2 = seg2PointA + d2 * t;
}

// Compute the intersection between a plane and a segment
// Let the plane define by the equation planeNormal.dot(X) = planeD with X a point on the plane and "planeNormal" the plane normal. This method
// computes the intersection P between the plane and the segment (segA, segB). The method returns the value "t" such
// that P = segA + t * (segB - segA). Note that it only returns a value in [0, 1] if there is an intersection. Otherwise,
// there is no intersection between the plane and the segment.
decimal reactphysics3d::computePlaneSegmentIntersection(const Vector3& segA, const Vector3& segB, const decimal planeD, const Vector3& planeNormal) {

    const decimal parallelEpsilon = decimal(0.0001);
	decimal t = decimal(-1);

    decimal nDotAB = planeNormal.dot(segB - segA);

	// If the segment is not parallel to the plane
    if (std::abs(nDotAB) > parallelEpsilon) {
		t = (planeD - planeNormal.dot(segA)) / nDotAB;
	}

	return t;
}

// Compute the distance between a point "point" and a line given by the points "linePointA" and "linePointB"
decimal reactphysics3d::computePointToLineDistance(const Vector3& linePointA, const Vector3& linePointB, const Vector3& point) {
	
	decimal distAB = (linePointB - linePointA).length();

	if (distAB < MACHINE_EPSILON) {
		return (point - linePointA).length();
	}

	return ((point - linePointA).cross(point - linePointB)).length() / distAB;
}

// Clip a segment against multiple planes and return the clipped segment vertices
// This method implements the Sutherland–Hodgman clipping algorithm
List<Vector3> reactphysics3d::clipSegmentWithPlanes(const Vector3& segA, const Vector3& segB,
                                                           const List<Vector3>& planesPoints,
                                                           const List<Vector3>& planesNormals,
                                                           MemoryAllocator& allocator) {
    assert(planesPoints.size() == planesNormals.size());

    List<Vector3> inputVertices(allocator, 2);
    List<Vector3> outputVertices(allocator, 2);

    inputVertices.add(segA);
    inputVertices.add(segB);

    // For each clipping plane
    for (uint p=0; p<planesPoints.size(); p++) {

        // If there is no more vertices, stop
        if (inputVertices.size() == 0) return inputVertices;

        assert(inputVertices.size() == 2);

        outputVertices.clear();

        Vector3& v1 = inputVertices[0];
        Vector3& v2 = inputVertices[1];

        decimal v1DotN = (v1 - planesPoints[p]).dot(planesNormals[p]);
        decimal v2DotN = (v2 - planesPoints[p]).dot(planesNormals[p]);

        // If the second vertex is in front of the clippling plane
        if (v2DotN >= decimal(0.0)) {

            // If the first vertex is not in front of the clippling plane
            if (v1DotN < decimal(0.0)) {

                // The second point we keep is the intersection between the segment v1, v2 and the clipping plane
                decimal t = computePlaneSegmentIntersection(v1, v2, planesNormals[p].dot(planesPoints[p]), planesNormals[p]);

                if (t >= decimal(0) && t <= decimal(1.0)) {
                    outputVertices.add(v1 + t * (v2 - v1));
                }
                else {
                    outputVertices.add(v2);
                }
            }
            else {
                outputVertices.add(v1);
            }

            // Add the second vertex
            outputVertices.add(v2);
        }
        else {  // If the second vertex is behind the clipping plane

            // If the first vertex is in front of the clippling plane
            if (v1DotN >= decimal(0.0)) {

                outputVertices.add(v1);

                // The first point we keep is the intersection between the segment v1, v2 and the clipping plane
                decimal t = computePlaneSegmentIntersection(v1, v2, -planesNormals[p].dot(planesPoints[p]), -planesNormals[p]);

                if (t >= decimal(0.0) && t <= decimal(1.0)) {
                    outputVertices.add(v1 + t * (v2 - v1));
                }
            }
        }

        inputVertices = outputVertices;
    }

    return outputVertices;
}

// Clip a polygon against multiple planes and return the clipped polygon vertices
// This method implements the Sutherland–Hodgman clipping algorithm
List<Vector3> reactphysics3d::clipPolygonWithPlanes(const List<Vector3>& polygonVertices, const List<Vector3>& planesPoints,
                                                    const List<Vector3>& planesNormals, MemoryAllocator& allocator) {

    assert(planesPoints.size() == planesNormals.size());

        uint nbMaxElements = polygonVertices.size() + planesPoints.size();
        List<Vector3> inputVertices(allocator, nbMaxElements);
        List<Vector3> outputVertices(allocator, nbMaxElements);

        inputVertices.addRange(polygonVertices);

        // For each clipping plane
        for (uint p=0; p<planesPoints.size(); p++) {

            outputVertices.clear();

            uint nbInputVertices = inputVertices.size();
            uint vStart = nbInputVertices - 1;

            // For each edge of the polygon
            for (uint vEnd = 0; vEnd<nbInputVertices; vEnd++) {

                Vector3& v1 = inputVertices[vStart];
                Vector3& v2 = inputVertices[vEnd];

                decimal v1DotN = (v1 - planesPoints[p]).dot(planesNormals[p]);
                decimal v2DotN = (v2 - planesPoints[p]).dot(planesNormals[p]);

                // If the second vertex is in front of the clippling plane
                if (v2DotN >= decimal(0.0)) {

                    // If the first vertex is not in front of the clippling plane
                    if (v1DotN < decimal(0.0)) {

                        // The second point we keep is the intersection between the segment v1, v2 and the clipping plane
                        decimal t = computePlaneSegmentIntersection(v1, v2, planesNormals[p].dot(planesPoints[p]), planesNormals[p]);

                        if (t >= decimal(0) && t <= decimal(1.0)) {
                            outputVertices.add(v1 + t * (v2 - v1));
                        }
                        else {
                            outputVertices.add(v2);
                        }
                    }

                    // Add the second vertex
                    outputVertices.add(v2);
                }
                else {  // If the second vertex is behind the clipping plane

                    // If the first vertex is in front of the clippling plane
                    if (v1DotN >= decimal(0.0)) {

                        // The first point we keep is the intersection between the segment v1, v2 and the clipping plane
                        decimal t = computePlaneSegmentIntersection(v1, v2, -planesNormals[p].dot(planesPoints[p]), -planesNormals[p]);

                        if (t >= decimal(0.0) && t <= decimal(1.0)) {
                            outputVertices.add(v1 + t * (v2 - v1));
                        }
                        else {
                            outputVertices.add(v1);
                        }
                    }
                }

                vStart = vEnd;
            }

            inputVertices = outputVertices;
        }

        return outputVertices;
}

// Project a point onto a plane that is given by a point and its unit length normal
Vector3 reactphysics3d::projectPointOntoPlane(const Vector3& point, const Vector3& unitPlaneNormal, const Vector3& planePoint) {
	return point - unitPlaneNormal.dot(point - planePoint) * unitPlaneNormal;
}

// Return the distance between a point and a plane (the plane normal must be normalized)
decimal reactphysics3d::computePointToPlaneDistance(const Vector3& point, const Vector3& planeNormal, const Vector3& planePoint) {
    return planeNormal.dot(point - planePoint);
}

// Return true if the given number is prime
bool reactphysics3d::isPrimeNumber(int number) {

    // If it's a odd number
    if ((number & 1) != 0) {

        int limit = static_cast<int>(std::sqrt(number));

        for (int divisor = 3; divisor <= limit; divisor += 2) {

            // If we have found a divisor
            if ((number % divisor) == 0) {

                // It is not a prime number
                return false;
            }
        }

        return true;
    }

    return number == 2;
}


