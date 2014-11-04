# Minimum cmake version required
CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

# Project configuration
PROJECT(REACTPHYSICS3D)

# Build type
IF (NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE "Release")
ENDIF (NOT CMAKE_BUILD_TYPE)

# Where to build the library
SET(LIBRARY_OUTPUT_PATH "lib")

# Where to build the executables
SET(OUR_EXECUTABLE_OUTPUT_PATH "${PROJECT_BINARY_DIR}/bin")

# Options
OPTION(COMPILE_EXAMPLES "Select this if you want to build the examples" OFF)
OPTION(COMPILE_TESTS "Select this if you want to build the tests" OFF)
OPTION(PROFILING_ENABLED "Select this if you want to compile with enabled profiling" OFF)
OPTION(DOUBLE_PRECISION_ENABLED "Select this if you want to compile using double precision floating
                                 values" OFF)
# Headers
INCLUDE_DIRECTORIES(src)

IF(PROFILING_ENABLED)
    ADD_DEFINITIONS(-DIS_PROFILING_ACTIVE)
ENDIF(PROFILING_ENABLED)

IF(DOUBLE_PRECISION_ENABLED)
    ADD_DEFINITIONS(-DIS_DOUBLE_PRECISION_ENABLED)
ENDIF(DOUBLE_PRECISION_ENABLED)

# Source files
SET (REACTPHYSICS3D_SOURCES
    "src/configuration.h"
    "src/decimal.h"
    "src/reactphysics3d.h"
    "src/body/Body.h"
    "src/body/Body.cpp"
    "src/body/CollisionBody.h"
    "src/body/CollisionBody.cpp"
    "src/body/RigidBody.h"
    "src/body/RigidBody.cpp"
    "src/collision/broadphase/BroadPhaseAlgorithm.h"
    "src/collision/broadphase/BroadPhaseAlgorithm.cpp"
    "src/collision/broadphase/DynamicAABBTree.h"
    "src/collision/broadphase/DynamicAABBTree.cpp"
    "src/collision/narrowphase/EPA/EdgeEPA.h"
    "src/collision/narrowphase/EPA/EdgeEPA.cpp"
    "src/collision/narrowphase/EPA/EPAAlgorithm.h"
    "src/collision/narrowphase/EPA/EPAAlgorithm.cpp"
    "src/collision/narrowphase/EPA/TriangleEPA.h"
    "src/collision/narrowphase/EPA/TriangleEPA.cpp"
    "src/collision/narrowphase/EPA/TrianglesStore.h"
    "src/collision/narrowphase/EPA/TrianglesStore.cpp"
    "src/collision/narrowphase/GJK/Simplex.h"
    "src/collision/narrowphase/GJK/Simplex.cpp"
    "src/collision/narrowphase/GJK/GJKAlgorithm.h"
    "src/collision/narrowphase/GJK/GJKAlgorithm.cpp"
    "src/collision/narrowphase/NarrowPhaseAlgorithm.h"
    "src/collision/narrowphase/NarrowPhaseAlgorithm.cpp"
    "src/collision/narrowphase/SphereVsSphereAlgorithm.h"
    "src/collision/narrowphase/SphereVsSphereAlgorithm.cpp"
    "src/collision/shapes/AABB.h"
    "src/collision/shapes/AABB.cpp"
    "src/collision/shapes/BoxShape.h"
    "src/collision/shapes/BoxShape.cpp"
    "src/collision/shapes/CapsuleShape.h"
    "src/collision/shapes/CapsuleShape.cpp"
    "src/collision/shapes/CollisionShape.h"
    "src/collision/shapes/CollisionShape.cpp"
    "src/collision/shapes/ConeShape.h"
    "src/collision/shapes/ConeShape.cpp"
    "src/collision/shapes/ConvexMeshShape.h"
    "src/collision/shapes/ConvexMeshShape.cpp"
    "src/collision/shapes/CylinderShape.h"
    "src/collision/shapes/CylinderShape.cpp"
    "src/collision/shapes/SphereShape.h"
    "src/collision/shapes/SphereShape.cpp"
    "src/collision/BroadPhasePair.h"
    "src/collision/BroadPhasePair.cpp"
    "src/collision/RaycastInfo.h"
    "src/collision/RaycastInfo.cpp"
    "src/collision/ProxyShape.h"
    "src/collision/ProxyShape.cpp"
    "src/collision/CollisionDetection.h"
    "src/collision/CollisionDetection.cpp"
    "src/constraint/BallAndSocketJoint.h"
    "src/constraint/BallAndSocketJoint.cpp"
    "src/constraint/ContactPoint.h"
    "src/constraint/ContactPoint.cpp"
    "src/constraint/FixedJoint.h"
    "src/constraint/FixedJoint.cpp"
    "src/constraint/HingeJoint.h"
    "src/constraint/HingeJoint.cpp"
    "src/constraint/Joint.h"
    "src/constraint/Joint.cpp"
    "src/constraint/SliderJoint.h"
    "src/constraint/SliderJoint.cpp"
    "src/engine/CollisionWorld.h"
    "src/engine/CollisionWorld.cpp"
    "src/engine/ConstraintSolver.h"
    "src/engine/ConstraintSolver.cpp"
    "src/engine/ContactManifold.h"
    "src/engine/ContactManifold.cpp"
    "src/engine/ContactSolver.h"
    "src/engine/ContactSolver.cpp"
    "src/engine/DynamicsWorld.h"
    "src/engine/DynamicsWorld.cpp"
    "src/engine/EventListener.h"
    "src/engine/Impulse.h"
    "src/engine/Island.h"
    "src/engine/Island.cpp"
    "src/engine/Material.h"
    "src/engine/Material.cpp"
    "src/engine/OverlappingPair.h"
    "src/engine/OverlappingPair.cpp"
    "src/engine/Profiler.h"
    "src/engine/Profiler.cpp"
    "src/engine/Timer.h"
    "src/engine/Timer.cpp"
    "src/mathematics/mathematics.h"
    "src/mathematics/mathematics_functions.h"
    "src/mathematics/Matrix2x2.h"
    "src/mathematics/Matrix2x2.cpp"
    "src/mathematics/Matrix3x3.h"
    "src/mathematics/Matrix3x3.cpp"
    "src/mathematics/Quaternion.h"
    "src/mathematics/Quaternion.cpp"
    "src/mathematics/Transform.h"
    "src/mathematics/Transform.cpp"
    "src/mathematics/Vector2.h"
    "src/mathematics/Vector2.cpp"
    "src/mathematics/Vector3.h"
    "src/mathematics/Ray.h"
    "src/mathematics/Vector3.cpp"
    "src/memory/MemoryAllocator.h"
    "src/memory/MemoryAllocator.cpp"
    "src/memory/Stack.h"
)

# Create the library
ADD_LIBRARY (reactphysics3d STATIC ${REACTPHYSICS3D_SOURCES})

# If we need to compile the examples
IF(COMPILE_EXAMPLES)
   add_subdirectory(examples/)
ENDIF(COMPILE_EXAMPLES)

# If we need to compile the tests
IF(COMPILE_TESTS)
   add_subdirectory(test/)
ENDIF(COMPILE_TESTS)
