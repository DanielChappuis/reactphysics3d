# Minimum cmake version required
cmake_minimum_required(VERSION 3.10)

# Project configuration
project(ReactPhysics3D VERSION 0.10.0 LANGUAGES CXX)
 
# In order to install libraries into correct locations on all platforms.
include(GNUInstallDirs)

# Set default build type
set(default_build_type "Release")
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  message(STATUS "Setting build type to '${default_build_type}' as none was specified.")
  set(CMAKE_BUILD_TYPE "${default_build_type}" CACHE
      STRING "Choose the type of build." FORCE)
  # Set the possible values of build type for cmake-gui
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS
    "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

# CMake modules path
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/CMakeModules)

# Add INSTALL target in Visual Studio solution
set(CMAKE_VS_INCLUDE_INSTALL_TO_DEFAULT_BUILD 1)

# Enable testing
enable_testing()

# Options
option(RP3D_COMPILE_TESTBED "Select this if you want to build the testbed application with demos" OFF)
option(RP3D_COMPILE_TESTS "Select this if you want to build the unit tests" OFF)
option(RP3D_PROFILING_ENABLED "Select this if you want to compile for performanace profiling" OFF)
option(RP3D_CODE_COVERAGE_ENABLED "Select this if you need to build for code coverage calculation" OFF)
option(RP3D_DOUBLE_PRECISION_ENABLED "Select this if you want to compile using double precision floating values" OFF)

# Code Coverage
if(RP3D_CODE_COVERAGE_ENABLED)
    INCLUDE(CodeCoverage)
    SETUP_TARGET_FOR_COVERAGE_LCOV(NAME coverage EXECUTABLE test/tests)
    APPEND_COVERAGE_COMPILER_FLAGS()
    # set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O0 -fprofile-arcs -ftest-coverage")
endif()

# Headers files
set (REACTPHYSICS3D_HEADERS
    "include/reactphysics3d/configuration.h"
    "include/reactphysics3d/decimal.h"
    "include/reactphysics3d/reactphysics3d.h"
    "include/reactphysics3d/body/Body.h"
    "include/reactphysics3d/body/RigidBody.h"
    "include/reactphysics3d/collision/ContactPointInfo.h"
    "include/reactphysics3d/collision/ContactManifoldInfo.h"
    "include/reactphysics3d/collision/ContactPair.h"
    "include/reactphysics3d/collision/broadphase/DynamicAABBTree.h"
    "include/reactphysics3d/collision/narrowphase/CollisionDispatch.h"
    "include/reactphysics3d/collision/narrowphase/GJK/VoronoiSimplex.h"
    "include/reactphysics3d/collision/narrowphase/GJK/GJKAlgorithm.h"
    "include/reactphysics3d/collision/narrowphase/SAT/SATAlgorithm.h"
    "include/reactphysics3d/collision/narrowphase/NarrowPhaseAlgorithm.h"
    "include/reactphysics3d/collision/narrowphase/SphereVsSphereAlgorithm.h"
    "include/reactphysics3d/collision/narrowphase/CapsuleVsCapsuleAlgorithm.h"
    "include/reactphysics3d/collision/narrowphase/SphereVsCapsuleAlgorithm.h"
    "include/reactphysics3d/collision/narrowphase/SphereVsConvexPolyhedronAlgorithm.h"
    "include/reactphysics3d/collision/narrowphase/CapsuleVsConvexPolyhedronAlgorithm.h"
    "include/reactphysics3d/collision/narrowphase/ConvexPolyhedronVsConvexPolyhedronAlgorithm.h"
    "include/reactphysics3d/collision/narrowphase/NarrowPhaseInput.h"
    "include/reactphysics3d/collision/narrowphase/NarrowPhaseInfoBatch.h"
    "include/reactphysics3d/collision/shapes/AABB.h"
    "include/reactphysics3d/collision/shapes/ConvexShape.h"
    "include/reactphysics3d/collision/shapes/ConvexPolyhedronShape.h"
    "include/reactphysics3d/collision/shapes/ConcaveShape.h"
    "include/reactphysics3d/collision/shapes/BoxShape.h"
    "include/reactphysics3d/collision/shapes/CapsuleShape.h"
    "include/reactphysics3d/collision/shapes/CollisionShape.h"
    "include/reactphysics3d/collision/shapes/ConvexMeshShape.h"
    "include/reactphysics3d/collision/shapes/SphereShape.h"
    "include/reactphysics3d/collision/shapes/TriangleShape.h"
    "include/reactphysics3d/collision/shapes/ConcaveMeshShape.h"
    "include/reactphysics3d/collision/shapes/HeightFieldShape.h"
    "include/reactphysics3d/collision/RaycastInfo.h"
    "include/reactphysics3d/collision/Collider.h"
    "include/reactphysics3d/collision/TriangleVertexArray.h"
    "include/reactphysics3d/collision/PolygonVertexArray.h"
    "include/reactphysics3d/collision/VertexArray.h"
    "include/reactphysics3d/collision/TriangleMesh.h"
    "include/reactphysics3d/collision/HeightField.h"
    "include/reactphysics3d/collision/ConvexMesh.h"
    "include/reactphysics3d/collision/HalfEdgeStructure.h"
    "include/reactphysics3d/collision/ContactManifold.h"
    "include/reactphysics3d/constraint/BallAndSocketJoint.h"
    "include/reactphysics3d/constraint/ContactPoint.h"
    "include/reactphysics3d/constraint/FixedJoint.h"
    "include/reactphysics3d/constraint/HingeJoint.h"
    "include/reactphysics3d/constraint/Joint.h"
    "include/reactphysics3d/constraint/SliderJoint.h"
    "include/reactphysics3d/engine/Entity.h"
    "include/reactphysics3d/engine/EntityManager.h"
    "include/reactphysics3d/engine/PhysicsCommon.h"
    "include/reactphysics3d/systems/ConstraintSolverSystem.h"
    "include/reactphysics3d/systems/ContactSolverSystem.h"
    "include/reactphysics3d/systems/DynamicsSystem.h"
    "include/reactphysics3d/systems/CollisionDetectionSystem.h"
    "include/reactphysics3d/systems/SolveBallAndSocketJointSystem.h"
    "include/reactphysics3d/systems/SolveFixedJointSystem.h"
    "include/reactphysics3d/systems/SolveHingeJointSystem.h"
    "include/reactphysics3d/systems/SolveSliderJointSystem.h"
    "include/reactphysics3d/engine/PhysicsWorld.h"
    "include/reactphysics3d/engine/EventListener.h"
    "include/reactphysics3d/engine/Island.h"
    "include/reactphysics3d/engine/Islands.h"
    "include/reactphysics3d/engine/Material.h"
    "include/reactphysics3d/engine/OverlappingPairs.h"
    "include/reactphysics3d/systems/BroadPhaseSystem.h"
    "include/reactphysics3d/components/Components.h"
    "include/reactphysics3d/components/BodyComponents.h"
    "include/reactphysics3d/components/RigidBodyComponents.h"
    "include/reactphysics3d/components/TransformComponents.h"
    "include/reactphysics3d/components/ColliderComponents.h"
    "include/reactphysics3d/components/JointComponents.h"
    "include/reactphysics3d/components/BallAndSocketJointComponents.h"
    "include/reactphysics3d/components/FixedJointComponents.h"
    "include/reactphysics3d/components/HingeJointComponents.h"
    "include/reactphysics3d/components/SliderJointComponents.h"
    "include/reactphysics3d/collision/CollisionCallback.h"
    "include/reactphysics3d/collision/OverlapCallback.h"
    "include/reactphysics3d/mathematics/mathematics.h"
    "include/reactphysics3d/mathematics/mathematics_common.h"
    "include/reactphysics3d/mathematics/mathematics_functions.h"
    "include/reactphysics3d/mathematics/Matrix2x2.h"
    "include/reactphysics3d/mathematics/Matrix3x3.h"
    "include/reactphysics3d/mathematics/Quaternion.h"
    "include/reactphysics3d/mathematics/Transform.h"
    "include/reactphysics3d/mathematics/Vector2.h"
    "include/reactphysics3d/mathematics/Vector3.h"
    "include/reactphysics3d/mathematics/Ray.h"
    "include/reactphysics3d/memory/MemoryAllocator.h"
    "include/reactphysics3d/memory/PoolAllocator.h"
    "include/reactphysics3d/memory/SingleFrameAllocator.h"
    "include/reactphysics3d/memory/HeapAllocator.h"
    "include/reactphysics3d/memory/DefaultAllocator.h"
    "include/reactphysics3d/memory/MemoryManager.h"
    "include/reactphysics3d/containers/Stack.h"
    "include/reactphysics3d/containers/LinkedList.h"
    "include/reactphysics3d/containers/Array.h"
    "include/reactphysics3d/containers/Map.h"
    "include/reactphysics3d/containers/Set.h"
    "include/reactphysics3d/containers/Pair.h"
    "include/reactphysics3d/containers/Deque.h"
    "include/reactphysics3d/utils/Profiler.h"
    "include/reactphysics3d/utils/Logger.h"
    "include/reactphysics3d/utils/Message.h"
    "include/reactphysics3d/utils/DefaultLogger.h"
    "include/reactphysics3d/utils/DebugRenderer.h"
    "include/reactphysics3d/utils/quickhull/QuickHull.h"
    "include/reactphysics3d/utils/quickhull/QHHalfEdgeStructure.h"
)

# Source files
set (REACTPHYSICS3D_SOURCES
    "src/body/Body.cpp"
    "src/body/RigidBody.cpp"
    "src/collision/broadphase/DynamicAABBTree.cpp"
    "src/collision/narrowphase/CollisionDispatch.cpp"
    "src/collision/narrowphase/GJK/VoronoiSimplex.cpp"
    "src/collision/narrowphase/GJK/GJKAlgorithm.cpp"
    "src/collision/narrowphase/SAT/SATAlgorithm.cpp"
    "src/collision/narrowphase/SphereVsSphereAlgorithm.cpp"
    "src/collision/narrowphase/CapsuleVsCapsuleAlgorithm.cpp"
    "src/collision/narrowphase/SphereVsCapsuleAlgorithm.cpp"
    "src/collision/narrowphase/SphereVsConvexPolyhedronAlgorithm.cpp"
    "src/collision/narrowphase/CapsuleVsConvexPolyhedronAlgorithm.cpp"
    "src/collision/narrowphase/ConvexPolyhedronVsConvexPolyhedronAlgorithm.cpp"
    "src/collision/narrowphase/NarrowPhaseInput.cpp"
    "src/collision/narrowphase/NarrowPhaseInfoBatch.cpp"
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
    "src/collision/Collider.cpp"
    "src/collision/TriangleVertexArray.cpp"
    "src/collision/PolygonVertexArray.cpp"
    "src/collision/VertexArray.cpp"
    "src/collision/TriangleMesh.cpp"
    "src/collision/HeightField.cpp"
    "src/collision/ConvexMesh.cpp"
    "src/collision/HalfEdgeStructure.cpp"
    "src/collision/ContactManifold.cpp"
    "src/constraint/BallAndSocketJoint.cpp"
    "src/constraint/ContactPoint.cpp"
    "src/constraint/FixedJoint.cpp"
    "src/constraint/HingeJoint.cpp"
    "src/constraint/Joint.cpp"
    "src/constraint/SliderJoint.cpp"
    "src/engine/PhysicsCommon.cpp"
    "src/systems/ConstraintSolverSystem.cpp"
    "src/systems/ContactSolverSystem.cpp"
    "src/systems/DynamicsSystem.cpp"
    "src/systems/CollisionDetectionSystem.cpp"
    "src/systems/SolveBallAndSocketJointSystem.cpp"
    "src/systems/SolveFixedJointSystem.cpp"
    "src/systems/SolveHingeJointSystem.cpp"
    "src/systems/SolveSliderJointSystem.cpp"
    "src/engine/PhysicsWorld.cpp"
    "src/engine/Island.cpp"
    "src/engine/Material.cpp"
    "src/engine/OverlappingPairs.cpp"
    "src/engine/Entity.cpp"
    "src/engine/EntityManager.cpp"
    "src/systems/BroadPhaseSystem.cpp"
    "src/components/Components.cpp"
    "src/components/BodyComponents.cpp"
    "src/components/RigidBodyComponents.cpp"
    "src/components/TransformComponents.cpp"
    "src/components/ColliderComponents.cpp"
    "src/components/JointComponents.cpp"
    "src/components/BallAndSocketJointComponents.cpp"
    "src/components/FixedJointComponents.cpp"
    "src/components/HingeJointComponents.cpp"
    "src/components/SliderJointComponents.cpp"
    "src/collision/CollisionCallback.cpp"
    "src/collision/OverlapCallback.cpp"
    "src/mathematics/Matrix2x2.cpp"
    "src/mathematics/Matrix3x3.cpp"
    "src/mathematics/Quaternion.cpp"
    "src/mathematics/Transform.cpp"
    "src/mathematics/Vector2.cpp"
    "src/mathematics/Vector3.cpp"
    "src/memory/PoolAllocator.cpp"
    "src/memory/SingleFrameAllocator.cpp"
    "src/memory/HeapAllocator.cpp"
    "src/memory/MemoryManager.cpp"
    "src/memory/MemoryAllocator.cpp"
    "src/utils/Profiler.cpp"
    "src/utils/DefaultLogger.cpp"
    "src/utils/DebugRenderer.cpp"
    "src/utils/quickhull/QuickHull.cpp"
    "src/utils/quickhull/QHHalfEdgeStructure.cpp"
)

# Create the library
add_library(reactphysics3d ${REACTPHYSICS3D_HEADERS} ${REACTPHYSICS3D_SOURCES})

# Creates an Alias Target, such that "ReactPhysics3D::reactphysics3d" can be used
# to refer to "reactphysics3d" in subsequent commands
add_library(ReactPhysics3D::reactphysics3d ALIAS reactphysics3d)

# C++11 compiler features
target_compile_features(reactphysics3d PUBLIC cxx_std_17)
set_target_properties(reactphysics3d PROPERTIES CXX_EXTENSIONS OFF)

# Library headers
target_include_directories(reactphysics3d PUBLIC
              $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
              $<INSTALL_INTERFACE:include>
)

# If we need to compile the testbed application
if(RP3D_COMPILE_TESTBED)
   add_subdirectory(testbed/)
endif()

# If we need to compile the tests
if(RP3D_COMPILE_TESTS)
   add_subdirectory(test/)
endif()

# Enable profiling if necessary
if(RP3D_PROFILING_ENABLED)
    target_compile_definitions(reactphysics3d PUBLIC IS_RP3D_PROFILING_ENABLED)
endif()

# Enable double precision if necessary
if(RP3D_DOUBLE_PRECISION_ENABLED)
    target_compile_definitions(reactphysics3d PUBLIC IS_RP3D_DOUBLE_PRECISION_ENABLED)
endif()

# Version number and soname for the library
set_target_properties(reactphysics3d  PROPERTIES
          VERSION "0.10.0" 
          SOVERSION "0.10"
)

# Install target (install library only, not headers)
install(TARGETS reactphysics3d
        EXPORT reactphysics3d-targets
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

# Install the headers separately (because INSTALL(TARGETS ... PUBLIC_HEADER DESTINATION ...) does
# not support subfolders
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

# This is required so that the exported target has the name ReactPhysics3D and not reactphysics3d
set_target_properties(reactphysics3d PROPERTIES EXPORT_NAME ReactPhysics3D)

# Give CMake access to the version number of the library so that the user is able to use the find_package() function
# with a given version number to select the version of the library he/she wants to use. This will create a
# "ReactPhysics3DConfigVersion.cmake" file that will later be copied into the install destination folder (see below)
include(CMakePackageConfigHelpers)
write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/ReactPhysics3DConfigVersion.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY AnyNewerVersion
)

# When the user runs "make install", this will export the targets into
# a "ReactPhysics3DConfig.cmake" file in the install destination
install(EXPORT reactphysics3d-targets
    FILE
        ReactPhysics3DConfig.cmake
    NAMESPACE
        ReactPhysics3D::
    DESTINATION
        ${CMAKE_INSTALL_LIBDIR}/cmake/ReactPhysics3D
)

# When the user runs "make install", this will copy the "ReactPhysics3DConfigVersion.cmake" file
# we have previously created into the install destination
install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/ReactPhysics3DConfigVersion.cmake
    DESTINATION lib/cmake/ReactPhysics3D
)

