# Minimum cmake version required
cmake_minimum_required(VERSION 2.6)

# Project configuration
PROJECT(CollisionShapes)

# Where to build the executable
SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin/collisionshapes/)

# Copy the shaders used for the demo into the build directory
FILE(COPY "../common/opengl-framework/src/shaders/" DESTINATION "${EXECUTABLE_OUTPUT_PATH}/shaders/")

# Copy the meshes used for the demo into the build directory
FILE(COPY "../common/meshes/" DESTINATION "${EXECUTABLE_OUTPUT_PATH}/meshes/")

# Headers
INCLUDE_DIRECTORIES("../common/opengl-framework/src/" "../common/")

# Create the example executable using the
# compiled reactphysics3d static library
ADD_EXECUTABLE(collisionshapes CollisionShapes.cpp Scene.cpp Scene.h "../common/VisualContactPoint.cpp" "../common/ConvexMesh.cpp" "../common/Capsule.cpp" "../common/Sphere.cpp" "../common/Cylinder.cpp" "../common/Cone.cpp" "../common/Box.cpp" "../common/Viewer.cpp")

TARGET_LINK_LIBRARIES(collisionshapes reactphysics3d openglframework)
