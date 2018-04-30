/********************************************************************************
* ReactPhysics3D physics library, http://www.reactphysics3d.com                 *
* Copyright (c) 2010-2018 Daniel Chappuis                                       *
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

#ifndef REACTPHYSICS3D_MATHEMATICS_FUNCTIONS_H
#define REACTPHYSICS3D_MATHEMATICS_FUNCTIONS_H

// Libraries
#include "configuration.h"
#include "decimal.h"
#include <algorithm>
#include <cassert>
#include <cmath>
#include "containers/List.h"

/// ReactPhysics3D namespace
namespace reactphysics3d {

struct Vector3;
struct Vector2;

// ---------- Mathematics functions ---------- //

/// Function to test if two real numbers are (almost) equal
/// We test if two numbers a and b are such that (a-b) are in [-EPSILON; EPSILON]
inline bool approxEqual(decimal a, decimal b, decimal epsilon = MACHINE_EPSILON) {
    return (std::fabs(a - b) < epsilon);
}

/// Function to test if two vectors are (almost) equal
bool approxEqual(const Vector3& vec1, const Vector3& vec2, decimal epsilon = MACHINE_EPSILON);

/// Function to test if two vectors are (almost) equal
bool approxEqual(const Vector2& vec1, const Vector2& vec2, decimal epsilon = MACHINE_EPSILON);

/// Function that returns the result of the "value" clamped by
/// two others values "lowerLimit" and "upperLimit"
inline int clamp(int value, int lowerLimit, int upperLimit) {
    assert(lowerLimit <= upperLimit);
    return std::min(std::max(value, lowerLimit), upperLimit);
}

/// Function that returns the result of the "value" clamped by
/// two others values "lowerLimit" and "upperLimit"
inline decimal clamp(decimal value, decimal lowerLimit, decimal upperLimit) {
    assert(lowerLimit <= upperLimit);
    return std::min(std::max(value, lowerLimit), upperLimit);
}

/// Return the minimum value among three values
inline decimal min3(decimal a, decimal b, decimal c) {
    return std::min(std::min(a, b), c);
}

/// Return the maximum value among three values
inline decimal max3(decimal a, decimal b, decimal c) {
    return std::max(std::max(a, b), c);
}

/// Return true if two values have the same sign
inline bool sameSign(decimal a, decimal b) {
    return a * b >= decimal(0.0);
}

/// Return true if two vectors are parallel
bool areParallelVectors(const Vector3& vector1, const Vector3& vector2);

/// Return true if two vectors are orthogonal
bool areOrthogonalVectors(const Vector3& vector1, const Vector3& vector2);

/// Clamp a vector such that it is no longer than a given maximum length
Vector3 clamp(const Vector3& vector, decimal maxLength);

// Compute and return a point on segment from "segPointA" and "segPointB" that is closest to point "pointC"
Vector3 computeClosestPointOnSegment(const Vector3& segPointA, const Vector3& segPointB, const Vector3& pointC);

// Compute the closest points between two segments
void computeClosestPointBetweenTwoSegments(const Vector3& seg1PointA, const Vector3& seg1PointB,
										   const Vector3& seg2PointA, const Vector3& seg2PointB,
										   Vector3& closestPointSeg1, Vector3& closestPointSeg2);

/// Compute the barycentric coordinates u, v, w of a point p inside the triangle (a, b, c)
void computeBarycentricCoordinatesInTriangle(const Vector3& a, const Vector3& b, const Vector3& c,
                                             const Vector3& p, decimal& u, decimal& v, decimal& w);

/// Compute the intersection between a plane and a segment
decimal computePlaneSegmentIntersection(const Vector3& segA, const Vector3& segB, const decimal planeD, const Vector3& planeNormal);

/// Compute the distance between a point and a line
decimal computePointToLineDistance(const Vector3& linePointA, const Vector3& linePointB, const Vector3& point);

/// Clip a segment against multiple planes and return the clipped segment vertices
List<Vector3> clipSegmentWithPlanes(const Vector3& segA, const Vector3& segB,
                                                           const List<Vector3>& planesPoints,
                                                           const List<Vector3>& planesNormals,
                                                           MemoryAllocator& allocator);

/// Clip a polygon against multiple planes and return the clipped polygon vertices
List<Vector3> clipPolygonWithPlanes(const List<Vector3>& polygonVertices, const List<Vector3>& planesPoints,
                                    const List<Vector3>& planesNormals, MemoryAllocator& allocator);

/// Project a point onto a plane that is given by a point and its unit length normal
Vector3 projectPointOntoPlane(const Vector3& point, const Vector3& planeNormal, const Vector3& planePoint);

/// Return the distance between a point and a plane (the plane normal must be normalized)
decimal computePointToPlaneDistance(const Vector3& point, const Vector3& planeNormal, const Vector3& planePoint);

/// Return true if the given number is prime
bool isPrimeNumber(int number);

}


#endif
