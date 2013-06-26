# Minimum cmake version required
cmake_minimum_required(VERSION 2.6)

# Project configuration
PROJECT(Joints)

# Copy the shaders used for the demo into the build directory
FILE(COPY "../common/opengl-framework/src/shaders/" DESTINATION "${CMAKE_CURRENT_BINARY_DIR}/shaders/")

# Headers
INCLUDE_DIRECTORIES("../common/opengl-framework/src/" "../common/")

# Create the example executable using the
# compiled reactphysics3d static library
ADD_EXECUTABLE(joints Joints.cpp Scene.cpp Scene.h "../common/Box.cpp" "../common/Box.h" "../common/Viewer.cpp" "../common/Viewer.h")

TARGET_LINK_LIBRARIES(joints reactphysics3d openglframework)
