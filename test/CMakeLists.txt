# Minimum cmake version required
CMAKE_MINIMUM_REQUIRED(VERSION 3.2 FATAL_ERROR)

# Project configuration
PROJECT(TESTS)

# Header files
SET (RP3D_TESTS_HEADERS
    "Test.h"
    "TestSuite.h"
    "tests/collision/TestAABB.h"
    "tests/collision/TestCollisionWorld.h"
    "tests/collision/TestDynamicAABBTree.h"
    "tests/collision/TestHalfEdgeStructure.h"
    "tests/collision/TestPointInside.h"
    "tests/collision/TestRaycast.h"
    "tests/collision/TestTriangleVertexArray.h"
    "tests/containers/TestList.h"
    "tests/containers/TestMap.h"
    "tests/containers/TestSet.h"
    "tests/mathematics/TestMathematicsFunctions.h"
    "tests/mathematics/TestMatrix2x2.h"
    "tests/mathematics/TestMatrix3x3.h"
    "tests/mathematics/TestQuaternion.h"
    "tests/mathematics/TestTransform.h"
    "tests/mathematics/TestVector2.h"
    "tests/mathematics/TestVector3.h"
)

# Source files
SET (RP3D_TESTS_SOURCES
    "main.cpp"
    "Test.cpp"
    "TestSuite.cpp"
)

# Create the tests executable
ADD_EXECUTABLE(tests ${RP3D_TESTS_HEADERS} ${RP3D_TESTS_SOURCES})

# Headers
TARGET_INCLUDE_DIRECTORIES(reactphysics3d PUBLIC
              $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
)

TARGET_LINK_LIBRARIES(tests reactphysics3d)

ADD_TEST(Test tests)
