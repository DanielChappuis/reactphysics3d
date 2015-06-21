# Minimum cmake version required
CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

# Project configuration
PROJECT(REACTPHYSICS3D)

# Build type
IF (NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE "Release")
ENDIF (NOT CMAKE_BUILD_TYPE)

IF (CMAKE_BUILD_TYPE MATCHES "Debug")
    IF (CMAKE_COMPILER_IS_GNUCXX)
        SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wno-unused-parameter")
    ENDIF (CMAKE_COMPILER_IS_GNUCXX)
ENDIF (CMAKE_BUILD_TYPE MATCHES "Debug")

# Where to build the library
SET(LIBRARY_OUTPUT_PATH "lib")

# Where to build the executables
SET(OUR_EXECUTABLE_OUTPUT_PATH "${PROJECT_BINARY_DIR}/bin")

# Options
OPTION(COMPILE_TESTBED "Select this if you want to build the testbed application" OFF)
OPTION(COMPILE_TESTS "Select this if you want to build the tests" OFF)
OPTION(PROFILING_ENABLED "Select this if you want to compile with enabled profiling" OFF)
OPTION(DOUBLE_PRECISION_ENABLED "Select this if you want to compile using double precision floating
                                 values" OFF)
# Headers
INCLUDE_DIRECTORIES("${PROJECT_SOURCE_DIR}/include")

IF(PROFILING_ENABLED)
    ADD_DEFINITIONS(-DIS_PROFILING_ACTIVE)
ENDIF(PROFILING_ENABLED)

IF(DOUBLE_PRECISION_ENABLED)
    ADD_DEFINITIONS(-DIS_DOUBLE_PRECISION_ENABLED)
ENDIF(DOUBLE_PRECISION_ENABLED)

SET(SRCDIR "${PROJECT_SOURCE_DIR}/src/reactphysics3d")
set(INCDIR "${PROJECT_SOURCE_DIR}/include/reactphysics3d")

# Source files
SET (REACTPHYSICS3D_SOURCES
    "${INCDIR}/configuration.h"
    "${INCDIR}/decimal.h"
    "${INCDIR}/reactphysics3d.h"
    "${INCDIR}/body/Body.h"
    "${INCDIR}/body/CollisionBody.h"
    "${INCDIR}/body/RigidBody.h"
    "${INCDIR}/collision/broadphase/BroadPhaseAlgorithm.h"
    "${INCDIR}/collision/broadphase/DynamicAABBTree.h"
    "${INCDIR}/collision/narrowphase/EPA/EdgeEPA.h"
    "${INCDIR}/collision/narrowphase/EPA/EPAAlgorithm.h"
    "${INCDIR}/collision/narrowphase/EPA/TriangleEPA.h"
    "${INCDIR}/collision/narrowphase/EPA/TrianglesStore.h"
    "${INCDIR}/collision/narrowphase/GJK/Simplex.h"
    "${INCDIR}/collision/narrowphase/GJK/GJKAlgorithm.h"
    "${INCDIR}/collision/narrowphase/NarrowPhaseAlgorithm.h"
    "${INCDIR}/collision/narrowphase/SphereVsSphereAlgorithm.h"
    "${INCDIR}/collision/shapes/AABB.h"
    "${INCDIR}/collision/shapes/BoxShape.h"
    "${INCDIR}/collision/shapes/CapsuleShape.h"
    "${INCDIR}/collision/shapes/CollisionShape.h"
    "${INCDIR}/collision/shapes/ConeShape.h"
    "${INCDIR}/collision/shapes/ConvexMeshShape.h"
    "${INCDIR}/collision/shapes/CylinderShape.h"
    "${INCDIR}/collision/shapes/SphereShape.h"
    "${INCDIR}/collision/RaycastInfo.h"
    "${INCDIR}/collision/ProxyShape.h"
    "${INCDIR}/collision/CollisionDetection.h"
    "${INCDIR}/constraint/BallAndSocketJoint.h"
    "${INCDIR}/constraint/ContactPoint.h"
    "${INCDIR}/constraint/FixedJoint.h"
    "${INCDIR}/constraint/HingeJoint.h"
    "${INCDIR}/constraint/Joint.h"
    "${INCDIR}/constraint/SliderJoint.h"
    "${INCDIR}/engine/CollisionWorld.h"
    "${INCDIR}/engine/ConstraintSolver.h"
    "${INCDIR}/engine/ContactManifold.h"
    "${INCDIR}/engine/ContactSolver.h"
    "${INCDIR}/engine/DynamicsWorld.h"
    "${INCDIR}/engine/EventListener.h"
    "${INCDIR}/engine/Impulse.h"
    "${INCDIR}/engine/Island.h"
    "${INCDIR}/engine/Material.h"
    "${INCDIR}/engine/OverlappingPair.h"
    "${INCDIR}/engine/Profiler.h"
    "${INCDIR}/mathematics/mathematics.h"
    "${INCDIR}/mathematics/mathematics_functions.h"
    "${INCDIR}/mathematics/Matrix2x2.h"
    "${INCDIR}/mathematics/Matrix3x3.h"
    "${INCDIR}/mathematics/Quaternion.h"
    "${INCDIR}/mathematics/Transform.h"
    "${INCDIR}/mathematics/Vector2.h"
    "${INCDIR}/mathematics/Vector3.h"
    "${INCDIR}/mathematics/Ray.h"
    "${INCDIR}/memory/MemoryAllocator.h"
    "${INCDIR}/memory/Stack.h"

    "${SRCDIR}/body/Body.cpp"
    "${SRCDIR}/body/CollisionBody.cpp"
    "${SRCDIR}/body/RigidBody.cpp"
    "${SRCDIR}/collision/broadphase/BroadPhaseAlgorithm.cpp"
    "${SRCDIR}/collision/broadphase/DynamicAABBTree.cpp"
    "${SRCDIR}/collision/narrowphase/EPA/EdgeEPA.cpp"
    "${SRCDIR}/collision/narrowphase/EPA/EPAAlgorithm.cpp"
    "${SRCDIR}/collision/narrowphase/EPA/TriangleEPA.cpp"
    "${SRCDIR}/collision/narrowphase/EPA/TrianglesStore.cpp"
    "${SRCDIR}/collision/narrowphase/GJK/Simplex.cpp"
    "${SRCDIR}/collision/narrowphase/GJK/GJKAlgorithm.cpp"
    "${SRCDIR}/collision/narrowphase/NarrowPhaseAlgorithm.cpp"
    "${SRCDIR}/collision/narrowphase/SphereVsSphereAlgorithm.cpp"
    "${SRCDIR}/collision/shapes/AABB.cpp"
    "${SRCDIR}/collision/shapes/BoxShape.cpp"
    "${SRCDIR}/collision/shapes/CapsuleShape.cpp"
    "${SRCDIR}/collision/shapes/CollisionShape.cpp"
    "${SRCDIR}/collision/shapes/ConeShape.cpp"
    "${SRCDIR}/collision/shapes/ConvexMeshShape.cpp"
    "${SRCDIR}/collision/shapes/CylinderShape.cpp"
    "${SRCDIR}/collision/shapes/SphereShape.cpp"
    "${SRCDIR}/collision/RaycastInfo.cpp"
    "${SRCDIR}/collision/ProxyShape.cpp"
    "${SRCDIR}/collision/CollisionDetection.cpp"
    "${SRCDIR}/constraint/BallAndSocketJoint.cpp"
    "${SRCDIR}/constraint/ContactPoint.cpp"
    "${SRCDIR}/constraint/FixedJoint.cpp"
    "${SRCDIR}/constraint/HingeJoint.cpp"
    "${SRCDIR}/constraint/Joint.cpp"
    "${SRCDIR}/constraint/SliderJoint.cpp"
    "${SRCDIR}/engine/CollisionWorld.cpp"
    "${SRCDIR}/engine/ConstraintSolver.cpp"
    "${SRCDIR}/engine/ContactManifold.cpp"
    "${SRCDIR}/engine/ContactSolver.cpp"
    "${SRCDIR}/engine/DynamicsWorld.cpp"
    "${SRCDIR}/engine/Island.cpp"
    "${SRCDIR}/engine/Material.cpp"
    "${SRCDIR}/engine/OverlappingPair.cpp"
    "${SRCDIR}/engine/Profiler.cpp"
    "${SRCDIR}/mathematics/Matrix2x2.cpp"
    "${SRCDIR}/mathematics/Matrix3x3.cpp"
    "${SRCDIR}/mathematics/Quaternion.cpp"
    "${SRCDIR}/mathematics/Transform.cpp"
    "${SRCDIR}/mathematics/Vector2.cpp"
    "${SRCDIR}/mathematics/Vector3.cpp"
    "${SRCDIR}/memory/MemoryAllocator.cpp"
)

# Create the library
ADD_LIBRARY (reactphysics3d STATIC ${REACTPHYSICS3D_SOURCES})

# If we need to compile the testbed application
IF(COMPILE_TESTBED)
   add_subdirectory(testbed/)
ENDIF(COMPILE_TESTBED)

# If we need to compile the tests
IF(COMPILE_TESTS)
   add_subdirectory(test/)
ENDIF(COMPILE_TESTS)
