# Minimum cmake version required
cmake_minimum_required(VERSION 2.6)

# Project configuration
PROJECT(Testbed)

# Where to build the executables
SET(EXECUTABLE_OUTPUT_PATH "${OUR_EXECUTABLE_OUTPUT_PATH}/testbed")
SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${EXECUTABLE_OUTPUT_PATH})
SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE ${EXECUTABLE_OUTPUT_PATH})
SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG ${EXECUTABLE_OUTPUT_PATH})

ADD_SUBDIRECTORY(opengl-framework/)
ADD_SUBDIRECTORY(glfw/)

# Copy the shaders used for the demo into the build directory
FILE(COPY "shaders/" DESTINATION "${EXECUTABLE_OUTPUT_PATH}/shaders/")

# Headers
MESSAGE("Test" "${OPENGLFRAMEWORK_DIR}")
INCLUDE_DIRECTORIES("opengl-framework/src/" "glfw/include/" "common/")

# Source files
SET(TESTBED_SOURCES
    Main.cpp
    TestbedApplication.h
    TestbedApplication.cpp
    Gui.h
    Gui.cpp
    Scene.h
    Scene.cpp
)

# Create the executable
ADD_EXECUTABLE(testbed ${TESTBED_SOURCES})

# Link with libraries
TARGET_LINK_LIBRARIES(testbed reactphysics3d openglframework glfw ${GLFW_LIBRARIES})
