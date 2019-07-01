# Minimum cmake version required
CMAKE_MINIMUM_REQUIRED(VERSION 3.2 FATAL_ERROR)

# Project configuration
PROJECT(REACTPHYSICS3D LANGUAGES CXX)

# In order to install libraries into correct locations on all platforms.
include(GNUInstallDirs)

# Build type
IF (NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE "Release")
ENDIF()

# Where to build the library
SET(LIBRARY_OUTPUT_PATH "lib")

# Where to build the executables
SET(OUR_EXECUTABLE_OUTPUT_PATH "${PROJECT_BINARY_DIR}/bin")

# CMake modules path
SET(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/CMakeModules)

# Enable testing
ENABLE_TESTING()

# Options
OPTION(RP3D_COMPILE_TESTBED "Select this if you want to build the testbed application" OFF)
OPTION(RP3D_COMPILE_TESTS "Select this if you want to build the tests" OFF)
OPTION(RP3D_PROFILING_ENABLED "Select this if you want to compile with enabled profiling" OFF)
OPTION(RP3D_LOGS_ENABLED "Select this if you want to compile with logs enabled during execution" OFF)
OPTION(RP3D_CODE_COVERAGE_ENABLED "Select this if you need to build for code coverage calculation" OFF)
OPTION(RP3D_DOUBLE_PRECISION_ENABLED "Select this if you want to compile using double precision floating
                                 values" OFF)

# Warning Compiler flags
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")

IF(RP3D_CODE_COVERAGE_ENABLED)
    IF(CMAKE_COMPILER_IS_GNUCXX)
        INCLUDE(CodeCoverage)
        SETUP_TARGET_FOR_COVERAGE(${PROJECT_NAME}_coverage tests coverage)
    ENDIF()
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O0 -fprofile-arcs -ftest-coverage")
ENDIF()

# C++11 flags
IF(CMAKE_CXX_COMPILER_ID MATCHES "Clang" OR CMAKE_CXX_COMPILER_ID MATCHES "GNU")
	include(CheckCXXCompilerFlag)
	CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
	IF(COMPILER_SUPPORTS_CXX11)
		set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
	ELSE()
		message("The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
	ENDIF()
ENDIF()

# Use libc++ with Clang
#IF(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -stdlib=libc++")
#ENDIF()

IF(RP3D_PROFILING_ENABLED)
    ADD_DEFINITIONS(-DIS_PROFILING_ACTIVE)
ENDIF()

IF(RP3D_LOGS_ENABLED)
    ADD_DEFINITIONS(-DIS_LOGGING_ACTIVE)
ENDIF()

IF(RP3D_DOUBLE_PRECISION_ENABLED)
    ADD_DEFINITIONS(-DIS_DOUBLE_PRECISION_ENABLED)
ENDIF()

# Headers files
SET (REACTPHYSICS3D_HEADERS
    "src/configuration.h"
    "src/decimal.h"
    "src/reactphysics3d.h"
    "src/body/Body.h"
    "src/body/CollisionBody.h"
    "src/body/RigidBody.h"
    "src/collision/ContactPointInfo.h"
    "src/collision/ContactManifoldInfo.h"
    "src/collision/broadphase/BroadPhaseAlgorithm.h"
    "src/collision/broadphase/DynamicAABBTree.h"
    "src/collision/narrowphase/CollisionDispatch.h"
    "src/collision/narrowphase/DefaultCollisionDispatch.h"
    "src/collision/narrowphase/GJK/VoronoiSimplex.h"
    "src/collision/narrowphase/GJK/GJKAlgorithm.h"
    "src/collision/narrowphase/SAT/SATAlgorithm.h"
    "src/collision/narrowphase/NarrowPhaseAlgorithm.h"
    "src/collision/narrowphase/SphereVsSphereAlgorithm.h"
    "src/collision/narrowphase/CapsuleVsCapsuleAlgorithm.h"
    "src/collision/narrowphase/SphereVsCapsuleAlgorithm.h"
    "src/collision/narrowphase/SphereVsConvexPolyhedronAlgorithm.h"
    "src/collision/narrowphase/CapsuleVsConvexPolyhedronAlgorithm.h"
    "src/collision/narrowphase/ConvexPolyhedronVsConvexPolyhedronAlgorithm.h"
    "src/collision/shapes/AABB.h"
    "src/collision/shapes/ConvexShape.h"
    "src/collision/shapes/ConvexPolyhedronShape.h"
    "src/collision/shapes/ConcaveShape.h"
    "src/collision/shapes/BoxShape.h"
    "src/collision/shapes/CapsuleShape.h"
    "src/collision/shapes/CollisionShape.h"
    "src/collision/shapes/ConvexMeshShape.h"
    "src/collision/shapes/SphereShape.h"
    "src/collision/shapes/TriangleShape.h"
    "src/collision/shapes/ConcaveMeshShape.h"
    "src/collision/shapes/HeightFieldShape.h"
    "src/collision/RaycastInfo.h"
    "src/collision/ProxyShape.h"
    "src/collision/TriangleVertexArray.h"
    "src/collision/PolygonVertexArray.h"
    "src/collision/TriangleMesh.h"
    "src/collision/PolyhedronMesh.h"
    "src/collision/HalfEdgeStructure.h"
    "src/collision/CollisionDetection.h"
    "src/collision/NarrowPhaseInfo.h"
    "src/collision/ContactManifold.h"
    "src/collision/ContactManifoldSet.h"
    "src/collision/MiddlePhaseTriangleCallback.h"
    "src/constraint/BallAndSocketJoint.h"
    "src/constraint/ContactPoint.h"
    "src/constraint/FixedJoint.h"
    "src/constraint/HingeJoint.h"
    "src/constraint/Joint.h"
    "src/constraint/SliderJoint.h"
    "src/engine/CollisionWorld.h"
    "src/engine/ConstraintSolver.h"
    "src/engine/ContactSolver.h"
    "src/engine/DynamicsWorld.h"
    "src/engine/EventListener.h"
    "src/engine/Island.h"
    "src/engine/Material.h"
    "src/engine/OverlappingPair.h"
    "src/engine/Timer.h"
    "src/engine/Timer.cpp"
    "src/collision/CollisionCallback.h"
    "src/collision/OverlapCallback.h"
    "src/mathematics/mathematics.h"
    "src/mathematics/mathematics_functions.h"
    "src/mathematics/Matrix2x2.h"
    "src/mathematics/Matrix3x3.h"
    "src/mathematics/Quaternion.h"
    "src/mathematics/Transform.h"
    "src/mathematics/Vector2.h"
    "src/mathematics/Vector3.h"
    "src/mathematics/Ray.h"
    "src/memory/MemoryAllocator.h"
    "src/memory/DefaultPoolAllocator.h"
    "src/memory/DefaultSingleFrameAllocator.h"
    "src/memory/DefaultAllocator.h"
    "src/memory/MemoryManager.h"
    "src/containers/Stack.h"
    "src/containers/LinkedList.h"
    "src/containers/List.h"
    "src/containers/Map.h"
    "src/containers/Set.h"
    "src/containers/Pair.h"
    "src/utils/Profiler.h"
    "src/utils/Logger.h"
)

# Source files
SET (REACTPHYSICS3D_SOURCES
    "src/body/Body.cpp"
    "src/body/CollisionBody.cpp"
    "src/body/RigidBody.cpp"
    "src/collision/ContactManifoldInfo.cpp"
    "src/collision/broadphase/BroadPhaseAlgorithm.cpp"
    "src/collision/broadphase/DynamicAABBTree.cpp"
    "src/collision/narrowphase/DefaultCollisionDispatch.cpp"
    "src/collision/narrowphase/GJK/VoronoiSimplex.cpp"
    "src/collision/narrowphase/GJK/GJKAlgorithm.cpp"
    "src/collision/narrowphase/SAT/SATAlgorithm.cpp"
    "src/collision/narrowphase/SphereVsSphereAlgorithm.cpp"
    "src/collision/narrowphase/CapsuleVsCapsuleAlgorithm.cpp"
    "src/collision/narrowphase/SphereVsCapsuleAlgorithm.cpp"
    "src/collision/narrowphase/SphereVsConvexPolyhedronAlgorithm.cpp"
    "src/collision/narrowphase/CapsuleVsConvexPolyhedronAlgorithm.cpp"
    "src/collision/narrowphase/ConvexPolyhedronVsConvexPolyhedronAlgorithm.cpp"
    "src/collision/shapes/AABB.cpp"
    "src/collision/shapes/ConvexShape.cpp"
    "src/collision/shapes/ConvexPolyhedronShape.cpp"
    "src/collision/shapes/ConcaveShape.cpp"
    "src/collision/shapes/BoxShape.cpp"
    "src/collision/shapes/CapsuleShape.cpp"
    "src/collision/shapes/CollisionShape.cpp"
    "src/collision/shapes/ConvexMeshShape.cpp"
    "src/collision/shapes/SphereShape.cpp"
    "src/collision/shapes/TriangleShape.cpp"
    "src/collision/shapes/ConcaveMeshShape.cpp"
    "src/collision/shapes/HeightFieldShape.cpp"
    "src/collision/RaycastInfo.cpp"
    "src/collision/ProxyShape.cpp"
    "src/collision/TriangleVertexArray.cpp"
    "src/collision/PolygonVertexArray.cpp"
    "src/collision/TriangleMesh.cpp"
    "src/collision/PolyhedronMesh.cpp"
    "src/collision/HalfEdgeStructure.cpp"
    "src/collision/CollisionDetection.cpp"
    "src/collision/NarrowPhaseInfo.cpp"
    "src/collision/ContactManifold.cpp"
    "src/collision/ContactManifoldSet.cpp"
    "src/collision/MiddlePhaseTriangleCallback.cpp"
    "src/constraint/BallAndSocketJoint.cpp"
    "src/constraint/ContactPoint.cpp"
    "src/constraint/FixedJoint.cpp"
    "src/constraint/HingeJoint.cpp"
    "src/constraint/Joint.cpp"
    "src/constraint/SliderJoint.cpp"
    "src/engine/CollisionWorld.cpp"
    "src/engine/ConstraintSolver.cpp"
    "src/engine/ContactSolver.cpp"
    "src/engine/DynamicsWorld.cpp"
    "src/engine/Island.cpp"
    "src/engine/Material.cpp"
    "src/engine/OverlappingPair.cpp"
    "src/engine/Timer.cpp"
    "src/collision/CollisionCallback.cpp"
    "src/mathematics/mathematics_functions.cpp"
    "src/mathematics/Matrix2x2.cpp"
    "src/mathematics/Matrix3x3.cpp"
    "src/mathematics/Quaternion.cpp"
    "src/mathematics/Transform.cpp"
    "src/mathematics/Vector2.cpp"
    "src/mathematics/Vector3.cpp"
    "src/memory/DefaultPoolAllocator.cpp"
    "src/memory/DefaultSingleFrameAllocator.cpp"
    "src/memory/MemoryManager.cpp"
    "src/utils/Profiler.cpp"
    "src/utils/Logger.cpp"
)

# Create the library
ADD_LIBRARY(reactphysics3d ${REACTPHYSICS3D_HEADERS} ${REACTPHYSICS3D_SOURCES})

# Headers
TARGET_INCLUDE_DIRECTORIES(reactphysics3d PUBLIC
              $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
              $<INSTALL_INTERFACE:include>
)

# If we need to compile the testbed application
IF(RP3D_COMPILE_TESTBED)
   add_subdirectory(testbed/)
ENDIF()

# If we need to compile the tests
IF(RP3D_COMPILE_TESTS)
   add_subdirectory(test/)
ENDIF()

SET_TARGET_PROPERTIES(reactphysics3d PROPERTIES PUBLIC_HEADER "${REACTPHYSICS3D_HEADERS}")

# Version number and soname for the library
SET_TARGET_PROPERTIES(reactphysics3d  PROPERTIES
          VERSION "0.7.1" 
          SOVERSION "0.7" 
)

# Install target
INSTALL(TARGETS reactphysics3d
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
        PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/reactphysics3d
)
