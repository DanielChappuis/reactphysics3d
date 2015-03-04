# Minimum cmake version required
CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

# Set a variable for the directory of the opengl-framework
SET(OPENGLFRAMEWORK_DIR "${CMAKE_CURRENT_SOURCE_DIR}/common/opengl-framework")

ADD_SUBDIRECTORY(common/)
ADD_SUBDIRECTORY(cubes/)
ADD_SUBDIRECTORY(joints/)
ADD_SUBDIRECTORY(collisionshapes/)
ADD_SUBDIRECTORY(raycast/)
