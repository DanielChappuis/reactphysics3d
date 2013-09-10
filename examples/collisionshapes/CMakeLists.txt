# Minimum cmake version required
cmake_minimum_required(VERSION 2.6)

# Project configuration
PROJECT(CollisionShapes)

# Copy the shaders used for the demo into the build directory
FILE(COPY "../common/opengl-framework/src/shaders/" DESTINATION "${CMAKE_CURRENT_BINARY_DIR}/shaders/")

# Copy the meshes used for the demo into the build directory
FILE(COPY "../common/meshes/" DESTINATION "${CMAKE_CURRENT_BINARY_DIR}/meshes/")

# Headers
INCLUDE_DIRECTORIES("../common/opengl-framework/src/" "../common/")

# Create the example executable using the
# compiled reactphysics3d static library
ADD_EXECUTABLE(collisionshapes CollisionShapes.cpp Scene.cpp Scene.h "../common/VisualContactPoint.cpp" "../common/ConvexMesh.cpp" "../common/Capsule.cpp" "../common/Sphere.cpp" "../common/Cylinder.cpp" "../common/Cone.cpp" "../common/Box.cpp" "../common/Viewer.cpp")

TARGET_LINK_LIBRARIES(collisionshapes reactphysics3d openglframework)
