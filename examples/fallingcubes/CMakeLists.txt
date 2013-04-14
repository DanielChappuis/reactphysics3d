# Minimum cmake version required
cmake_minimum_required(VERSION 2.6)

# Project configuration
PROJECT(FallingCubes)

# Copy the shaders used for the demo into the build directory
FILE(COPY "../opengl-framework/src/shaders/" DESTINATION "${CMAKE_CURRENT_BINARY_DIR}/shaders/")

# Headers
INCLUDE_DIRECTORIES("../opengl-framework/src/")

# Create the example executable using the
# compiled reactphysics3d static library
ADD_EXECUTABLE(fallingcubes main.cpp Scene.cpp Scene.h Box.cpp Box.h)

TARGET_LINK_LIBRARIES(fallingcubes reactphysics3d openglframework)
