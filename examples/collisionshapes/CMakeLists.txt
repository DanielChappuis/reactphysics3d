# Minimum cmake version required
cmake_minimum_required(VERSION 2.6)

# Project configuration
PROJECT(CollisionShapes)

# Where to build the executables
SET(EXECUTABLE_OUTPUT_PATH "${OUR_EXECUTABLE_OUTPUT_PATH}/collisionshapes")
SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${EXECUTABLE_OUTPUT_PATH})
SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE ${EXECUTABLE_OUTPUT_PATH})
SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG ${EXECUTABLE_OUTPUT_PATH})

# Copy the shaders used for the demo into the build directory
FILE(COPY "${OPENGLFRAMEWORK_DIR}/src/shaders/" DESTINATION "${EXECUTABLE_OUTPUT_PATH}/shaders/")

# Copy the meshes used for the demo into the build directory
FILE(COPY "../common/meshes/" DESTINATION "${EXECUTABLE_OUTPUT_PATH}/meshes/")

# Headers
INCLUDE_DIRECTORIES("${OPENGLFRAMEWORK_DIR}/src/" "../common/glfw/include/" "../common/")

# Source files
SET(COLLISION_SHAPES_SOURCES
    CollisionShapes.cpp
    Scene.cpp
    Scene.h
    "../common/VisualContactPoint.cpp"
    "../common/ConvexMesh.cpp"
    "../common/Capsule.cpp"
    "../common/Sphere.cpp"
    "../common/Cylinder.cpp"
    "../common/Cone.cpp"
    "../common/Box.cpp"
    "../common/Viewer.cpp"
)

# Create the executable
ADD_EXECUTABLE(collisionshapes ${COLLISION_SHAPES_SOURCES})

# Link with libraries
TARGET_LINK_LIBRARIES(collisionshapes reactphysics3d openglframework glfw ${GLFW_LIBRARIES})
