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
#include "TestSuite.h"
#include "tests/mathematics/TestVector2.h"
#include "tests/mathematics/TestVector3.h"
#include "tests/mathematics/TestTransform.h"
#include "tests/mathematics/TestQuaternion.h"
#include "tests/mathematics/TestMatrix2x2.h"
#include "tests/mathematics/TestMatrix3x3.h"
#include "tests/mathematics/TestMathematicsFunctions.h"
#include "tests/collision/TestPointInside.h"
#include "tests/collision/TestRaycast.h"
#include "tests/collision/TestCollisionWorld.h"
#include "tests/collision/TestAABB.h"
#include "tests/collision/TestDynamicAABBTree.h"
#include "tests/collision/TestHalfEdgeStructure.h"
#include "tests/collision/TestTriangleVertexArray.h"
#include "tests/containers/TestList.h"
#include "tests/containers/TestMap.h"
#include "tests/containers/TestSet.h"
#include "tests/containers/TestDeque.h"
#include "tests/containers/TestStack.h"

using namespace reactphysics3d;

int main() {

    TestSuite testSuite("ReactPhysics3D Tests");

    // ---------- Containers tests ---------- //

    testSuite.addTest(new TestList("List"));
    testSuite.addTest(new TestMap("Map"));
    testSuite.addTest(new TestSet("Set"));
    testSuite.addTest(new TestDeque("Deque"));
    testSuite.addTest(new TestStack("Stack"));

    // ---------- Mathematics tests ---------- //

    testSuite.addTest(new TestVector2("Vector2"));
    testSuite.addTest(new TestVector3("Vector3"));
    testSuite.addTest(new TestTransform("Transform"));
    testSuite.addTest(new TestQuaternion("Quaternion"));
    testSuite.addTest(new TestMatrix3x3("Matrix3x3"));
    testSuite.addTest(new TestMatrix2x2("Matrix2x2"));
    testSuite.addTest(new TestMathematicsFunctions("Maths Functions"));

    // ---------- Collision Detection tests ---------- //

    testSuite.addTest(new TestAABB("AABB"));
    testSuite.addTest(new TestPointInside("IsPointInside"));
    testSuite.addTest(new TestTriangleVertexArray("TriangleVertexArray"));
    testSuite.addTest(new TestRaycast("Raycasting"));
    testSuite.addTest(new TestCollisionWorld("CollisionWorld"));
    testSuite.addTest(new TestDynamicAABBTree("DynamicAABBTree"));
    testSuite.addTest(new TestHalfEdgeStructure("HalfEdgeStructure"));

    // Run the tests
    testSuite.run();

    // Display the report
    long nbFailedTests = testSuite.report();

    // Clear the tests from the test suite
    testSuite.clear();

    return nbFailedTests;
}
