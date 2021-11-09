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

#ifndef TEST_MATHEMATICS_FUNCTIONS_H
#define TEST_MATHEMATICS_FUNCTIONS_H

// Libraries
#include <reactphysics3d/containers/Array.h>
#include <reactphysics3d/memory/DefaultAllocator.h>
#include <reactphysics3d/mathematics/mathematics.h>

/// Reactphysics3D namespace
namespace reactphysics3d {

// Class TestMathematicsFunctions
/**
 * Unit test for mathematics functions
 */
class TestMathematicsFunctions : public Test {

    private :

        // ---------- Atributes ---------- //

        DefaultAllocator mAllocator;

    public :

        // ---------- Methods ---------- //

        /// Constructor
        TestMathematicsFunctions(const std::string& name): Test(name)  {}

        /// Run the tests
        void run() {

            // Test approxEqual()
            rp3d_test(approxEqual(2, 7, 5.2));
            rp3d_test(approxEqual(7, 2, 5.2));
            rp3d_test(approxEqual(6, 6));
            rp3d_test(!approxEqual(1, 5));
            rp3d_test(!approxEqual(1, 5, 3));
            rp3d_test(approxEqual(-2, -2));
            rp3d_test(approxEqual(-2, -7, 6));
            rp3d_test(!approxEqual(-2, 7, 2));
            rp3d_test(approxEqual(-3, 8, 12));
            rp3d_test(!approxEqual(-3, 8, 6));

            // Test clamp()
            rp3d_test(clamp(4, -3, 5) == 4);
            rp3d_test(clamp(-3, 1, 8) == 1);
            rp3d_test(clamp(45, -6, 7) == 7);
            rp3d_test(clamp(-5, -2, -1) == -2);
            rp3d_test(clamp(-5, -9, -1) == -5);
            rp3d_test(clamp(6, 6, 9) == 6);
            rp3d_test(clamp(9, 6, 9) == 9);
            rp3d_test(clamp(decimal(4), decimal(-3), decimal(5)) == decimal(4));
            rp3d_test(clamp(decimal(-3), decimal(1), decimal(8)) == decimal(1));
            rp3d_test(clamp(decimal(45), decimal(-6), decimal(7)) == decimal(7));
            rp3d_test(clamp(decimal(-5), decimal(-2), decimal(-1)) == decimal(-2));
            rp3d_test(clamp(decimal(-5), decimal(-9), decimal(-1)) == decimal(-5));
            rp3d_test(clamp(decimal(6), decimal(6), decimal(9)) == decimal(6));
            rp3d_test(clamp(decimal(9), decimal(6), decimal(9)) == decimal(9));

            // Test min3()
            rp3d_test(min3(1, 5, 7) == 1);
            rp3d_test(min3(-4, 2, 4) == -4);
            rp3d_test(min3(-1, -5, -7) == -7);
            rp3d_test(min3(13, 5, 47) == 5);
            rp3d_test(min3(4, 4, 4) == 4);

            // Test max3()
            rp3d_test(max3(1, 5, 7) == 7);
            rp3d_test(max3(-4, 2, 4) == 4);
            rp3d_test(max3(-1, -5, -7) == -1);
            rp3d_test(max3(13, 5, 47) == 47);
            rp3d_test(max3(4, 4, 4) == 4);

            // Test sameSign()
            rp3d_test(sameSign(4, 53));
            rp3d_test(sameSign(-4, -8));
            rp3d_test(!sameSign(4, -7));
            rp3d_test(!sameSign(-4, 53));

            // Test computePointToPlaneDistance()
            Vector3 p(8, 4, 0);
            Vector3 n1(1, 0, 0);
            Vector3 n2(-1, 0, 0);
            Vector3 q1(1, 54, 0);
            Vector3 q2(8, 17, 0);
            rp3d_test(approxEqual(computePointToPlaneDistance(q1, n1, p), decimal(-7)));
            rp3d_test(approxEqual(computePointToPlaneDistance(q1, n2, p), decimal(7)));
            rp3d_test(approxEqual(computePointToPlaneDistance(q2, n2, p), decimal(0.0)));

            // Test computeBarycentricCoordinatesInTriangle()
            Vector3 a(0, 0, 0);
            Vector3 b(5, 0, 0);
            Vector3 c(0, 0, 5);
            Vector3 testPoint(4, 0, 1);
            decimal u,v,w;
            computeBarycentricCoordinatesInTriangle(a, b, c, a, u, v, w);
            rp3d_test(approxEqual(u, 1.0, 0.000001));
            rp3d_test(approxEqual(v, 0.0, 0.000001));
            rp3d_test(approxEqual(w, 0.0, 0.000001));
            computeBarycentricCoordinatesInTriangle(a, b, c, b, u, v, w);
            rp3d_test(approxEqual(u, 0.0, 0.000001));
            rp3d_test(approxEqual(v, 1.0, 0.000001));
            rp3d_test(approxEqual(w, 0.0, 0.000001));
            computeBarycentricCoordinatesInTriangle(a, b, c, c, u, v, w);
            rp3d_test(approxEqual(u, 0.0, 0.000001));
            rp3d_test(approxEqual(v, 0.0, 0.000001));
            rp3d_test(approxEqual(w, 1.0, 0.000001));

            computeBarycentricCoordinatesInTriangle(a, b, c, testPoint, u, v, w);
            rp3d_test(approxEqual(u + v + w, 1.0, 0.000001));

			// Test computeClosestPointBetweenTwoSegments()
			Vector3 closestSeg1, closestSeg2;
			computeClosestPointBetweenTwoSegments(Vector3(4, 0, 0), Vector3(6, 0, 0), Vector3(8, 0, 0), Vector3(8, 6, 0), closestSeg1, closestSeg2);
            rp3d_test(approxEqual(closestSeg1.x, 6.0, 0.000001));
            rp3d_test(approxEqual(closestSeg1.y, 0.0, 0.000001));
            rp3d_test(approxEqual(closestSeg1.z, 0.0, 0.000001));
            rp3d_test(approxEqual(closestSeg2.x, 8.0, 0.000001));
            rp3d_test(approxEqual(closestSeg2.y, 0.0, 0.000001));
            rp3d_test(approxEqual(closestSeg2.z, 0.0, 0.000001));
			computeClosestPointBetweenTwoSegments(Vector3(4, 6, 5), Vector3(4, 6, 5), Vector3(8, 3, -9), Vector3(8, 3, -9), closestSeg1, closestSeg2);
            rp3d_test(approxEqual(closestSeg1.x, 4.0, 0.000001));
            rp3d_test(approxEqual(closestSeg1.y, 6.0, 0.000001));
            rp3d_test(approxEqual(closestSeg1.z, 5.0, 0.000001));
            rp3d_test(approxEqual(closestSeg2.x, 8.0, 0.000001));
            rp3d_test(approxEqual(closestSeg2.y, 3.0, 0.000001));
            rp3d_test(approxEqual(closestSeg2.z, -9.0, 0.000001));
			computeClosestPointBetweenTwoSegments(Vector3(0, -5, 0), Vector3(0, 8, 0), Vector3(6, 3, 0), Vector3(10, -3, 0), closestSeg1, closestSeg2);
            rp3d_test(approxEqual(closestSeg1.x, 0.0, 0.000001));
            rp3d_test(approxEqual(closestSeg1.y, 3.0, 0.000001));
            rp3d_test(approxEqual(closestSeg1.z, 0.0, 0.000001));
            rp3d_test(approxEqual(closestSeg2.x, 6.0, 0.000001));
            rp3d_test(approxEqual(closestSeg2.y, 3.0, 0.000001));
            rp3d_test(approxEqual(closestSeg2.z, 0.0, 0.000001));
			computeClosestPointBetweenTwoSegments(Vector3(1, -4, -5), Vector3(1, 4, -5), Vector3(-6, 5, -5), Vector3(6, 5, -5), closestSeg1, closestSeg2);
            rp3d_test(approxEqual(closestSeg1.x, 1.0, 0.000001));
            rp3d_test(approxEqual(closestSeg1.y, 4.0, 0.000001));
            rp3d_test(approxEqual(closestSeg1.z, -5.0, 0.000001));
            rp3d_test(approxEqual(closestSeg2.x, 1.0, 0.000001));
            rp3d_test(approxEqual(closestSeg2.y, 5.0, 0.000001));
            rp3d_test(approxEqual(closestSeg2.z, -5.0, 0.000001));

			// Test computePlaneSegmentIntersection();
            rp3d_test(approxEqual(computePlaneSegmentIntersection(Vector3(-6, 3, 0), Vector3(6, 3, 0), 0.0, Vector3(-1, 0, 0)), 0.5, 0.000001));
            rp3d_test(approxEqual(computePlaneSegmentIntersection(Vector3(-6, 3, 0), Vector3(6, 3, 0), 0.0, Vector3(1, 0, 0)), 0.5, 0.000001));
            rp3d_test(approxEqual(computePlaneSegmentIntersection(Vector3(5, 12, 0), Vector3(5, 4, 0), 6, Vector3(0, 1, 0)), 0.75, 0.000001));
            rp3d_test(approxEqual(computePlaneSegmentIntersection(Vector3(5, 4, 8), Vector3(9, 14, 8), 4, Vector3(0, 1, 0)), 0.0, 0.000001));
			decimal tIntersect = computePlaneSegmentIntersection(Vector3(5, 4, 0), Vector3(9, 4, 0), 4, Vector3(0, 1, 0));
            rp3d_test(tIntersect < 0.0 || tIntersect > 1.0);

            // Test computePointToLineDistance()
            rp3d_test(approxEqual(computePointToLineDistance(Vector3(6, 0, 0), Vector3(14, 0, 0), Vector3(5, 3, 0)), 3.0, 0.000001));
            rp3d_test(approxEqual(computePointToLineDistance(Vector3(6, -5, 0), Vector3(10, -5, 0), Vector3(4, 3, 0)), 8.0, 0.000001));
            rp3d_test(approxEqual(computePointToLineDistance(Vector3(6, -5, 0), Vector3(10, -5, 0), Vector3(-43, 254, 0)), 259.0, 0.000001));
            rp3d_test(approxEqual(computePointToLineDistance(Vector3(6, -5, 8), Vector3(10, -5, -5), Vector3(6, -5, 8)), 0.0, 0.000001));
            rp3d_test(approxEqual(computePointToLineDistance(Vector3(6, -5, 8), Vector3(10, -5, -5), Vector3(10, -5, -5)), 0.0, 0.000001));

            // Test clipSegmentWithPlanes()
            std::vector<Vector3> segmentVertices;
            segmentVertices.push_back(Vector3(-6, 3, 0));
            segmentVertices.push_back(Vector3(8, 3, 0));

            Array<Vector3> planesNormals(mAllocator, 2);
            Array<Vector3> planesPoints(mAllocator, 2);
            planesNormals.add(Vector3(-1, 0, 0));
            planesPoints.add(Vector3(4, 0, 0));

            Array<Vector3> clipSegmentVertices = clipSegmentWithPlanes(segmentVertices[0], segmentVertices[1],
                                                                             planesPoints, planesNormals, mAllocator);
            rp3d_test(clipSegmentVertices.size() == 2);
            rp3d_test(approxEqual(clipSegmentVertices[0].x, -6, 0.000001));
            rp3d_test(approxEqual(clipSegmentVertices[0].y, 3, 0.000001));
            rp3d_test(approxEqual(clipSegmentVertices[0].z, 0, 0.000001));
            rp3d_test(approxEqual(clipSegmentVertices[1].x, 4, 0.000001));
            rp3d_test(approxEqual(clipSegmentVertices[1].y, 3, 0.000001));
            rp3d_test(approxEqual(clipSegmentVertices[1].z, 0, 0.000001));

            segmentVertices.clear();
            segmentVertices.push_back(Vector3(8, 3, 0));
            segmentVertices.push_back(Vector3(-6, 3, 0));

            clipSegmentVertices = clipSegmentWithPlanes(segmentVertices[0], segmentVertices[1], planesPoints, planesNormals, mAllocator);
            rp3d_test(clipSegmentVertices.size() == 2);
            rp3d_test(approxEqual(clipSegmentVertices[0].x, 4, 0.000001));
            rp3d_test(approxEqual(clipSegmentVertices[0].y, 3, 0.000001));
            rp3d_test(approxEqual(clipSegmentVertices[0].z, 0, 0.000001));
            rp3d_test(approxEqual(clipSegmentVertices[1].x, -6, 0.000001));
            rp3d_test(approxEqual(clipSegmentVertices[1].y, 3, 0.000001));
            rp3d_test(approxEqual(clipSegmentVertices[1].z, 0, 0.000001));

            segmentVertices.clear();
            segmentVertices.push_back(Vector3(-6, 3, 0));
            segmentVertices.push_back(Vector3(3, 3, 0));

            clipSegmentVertices = clipSegmentWithPlanes(segmentVertices[0], segmentVertices[1], planesPoints, planesNormals, mAllocator);
            rp3d_test(clipSegmentVertices.size() == 2);
            rp3d_test(approxEqual(clipSegmentVertices[0].x, -6, 0.000001));
            rp3d_test(approxEqual(clipSegmentVertices[0].y, 3, 0.000001));
            rp3d_test(approxEqual(clipSegmentVertices[0].z, 0, 0.000001));
            rp3d_test(approxEqual(clipSegmentVertices[1].x, 3, 0.000001));
            rp3d_test(approxEqual(clipSegmentVertices[1].y, 3, 0.000001));
            rp3d_test(approxEqual(clipSegmentVertices[1].z, 0, 0.000001));

            segmentVertices.clear();
            segmentVertices.push_back(Vector3(5, 3, 0));
            segmentVertices.push_back(Vector3(8, 3, 0));

            clipSegmentVertices = clipSegmentWithPlanes(segmentVertices[0], segmentVertices[1], planesPoints, planesNormals, mAllocator);
            rp3d_test(clipSegmentVertices.size() == 0);

            // Test clipPolygonWithPlanes()
            Array<Vector3> polygonVertices(mAllocator);
            polygonVertices.add(Vector3(-4, 2, 0));
            polygonVertices.add(Vector3(7, 2, 0));
            polygonVertices.add(Vector3(7, 4, 0));
            polygonVertices.add(Vector3(-4, 4, 0));

            Array<Vector3> polygonPlanesNormals(mAllocator);
            Array<Vector3> polygonPlanesPoints(mAllocator);
            polygonPlanesNormals.add(Vector3(1, 0, 0));
            polygonPlanesPoints.add(Vector3(0, 0, 0));
            polygonPlanesNormals.add(Vector3(0, 1, 0));
            polygonPlanesPoints.add(Vector3(0, 0, 0));
            polygonPlanesNormals.add(Vector3(-1, 0, 0));
            polygonPlanesPoints.add(Vector3(10, 0, 0));
            polygonPlanesNormals.add(Vector3(0, -1, 0));
            polygonPlanesPoints.add(Vector3(10, 5, 0));

            Array<Vector3> clipPolygonVertices(mAllocator);
            for (size_t i=0; i < polygonPlanesPoints.size(); i++) {

                clipPolygonVertices.clear();
                clipPolygonWithPlane(polygonVertices, polygonPlanesPoints[i], polygonPlanesNormals[i], clipPolygonVertices);
                polygonVertices = clipPolygonVertices;
            }
            rp3d_test(clipPolygonVertices.size() == 4);
            rp3d_test(approxEqual(clipPolygonVertices[0].x, 0, 0.000001));
            rp3d_test(approxEqual(clipPolygonVertices[0].y, 2, 0.000001));
            rp3d_test(approxEqual(clipPolygonVertices[0].z, 0, 0.000001));
            rp3d_test(approxEqual(clipPolygonVertices[1].x, 7, 0.000001));
            rp3d_test(approxEqual(clipPolygonVertices[1].y, 2, 0.000001));
            rp3d_test(approxEqual(clipPolygonVertices[1].z, 0, 0.000001));
            rp3d_test(approxEqual(clipPolygonVertices[2].x, 7, 0.000001));
            rp3d_test(approxEqual(clipPolygonVertices[2].y, 4, 0.000001));
            rp3d_test(approxEqual(clipPolygonVertices[2].z, 0, 0.000001));
            rp3d_test(approxEqual(clipPolygonVertices[3].x, 0, 0.000001));
            rp3d_test(approxEqual(clipPolygonVertices[3].y, 4, 0.000001));
            rp3d_test(approxEqual(clipPolygonVertices[3].z, 0, 0.000001));

            // Test isPowerOfTwo()
            rp3d_test(!isPowerOfTwo(0));
            rp3d_test(!isPowerOfTwo(3));
            rp3d_test(!isPowerOfTwo(144));
            rp3d_test(!isPowerOfTwo(13));
            rp3d_test(!isPowerOfTwo(18));
            rp3d_test(!isPowerOfTwo(1000));

            rp3d_test(isPowerOfTwo(1));
            rp3d_test(isPowerOfTwo(2));
            rp3d_test(isPowerOfTwo(4));
            rp3d_test(isPowerOfTwo(8));
            rp3d_test(isPowerOfTwo(256));
            rp3d_test(isPowerOfTwo(1024));
            rp3d_test(isPowerOfTwo(2048));

            // Test nextPowerOfTwo32Bits()
            rp3d_test(nextPowerOfTwo64Bits(0) == 1);
            rp3d_test(nextPowerOfTwo64Bits(1) == 1);
            rp3d_test(nextPowerOfTwo64Bits(2) == 2);
            rp3d_test(nextPowerOfTwo64Bits(3) == 4);
            rp3d_test(nextPowerOfTwo64Bits(5) == 8);
            rp3d_test(nextPowerOfTwo64Bits(6) == 8);
            rp3d_test(nextPowerOfTwo64Bits(7) == 8);
            rp3d_test(nextPowerOfTwo64Bits(1000) == 1024);
            rp3d_test(nextPowerOfTwo64Bits(129) == 256);
            rp3d_test(nextPowerOfTwo64Bits(260) == 512);
        }

 };

}

#endif
