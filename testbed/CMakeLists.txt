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

# Copy the meshes used for the demo into the build directory
FILE(COPY "meshes/" DESTINATION "${EXECUTABLE_OUTPUT_PATH}/meshes/")

# Copy the fonts used for the GUI into the build directory
FILE(COPY "imgui/DroidSans.ttf" DESTINATION "${EXECUTABLE_OUTPUT_PATH}")

# Enable C++11 features
SET(CMAKE_CXX_FLAGS "-std=c++0x")

#ADD_DEFINITIONS(-DGL3)

# Headers
INCLUDE_DIRECTORIES(${GLEW_INCLUDE_PATH} "src/" "opengl-framework/src/" "glfw/include/" "common/" "scenes/" "imgui/")

# Testbed source files
SET(TESTBED_SOURCES
    src/Main.cpp
    src/TestbedApplication.h
    src/TestbedApplication.cpp
    src/Gui.h
    src/Gui.cpp
    src/Scene.h
    src/Scene.cpp
    src/SceneDemo.h
    src/SceneDemo.cpp
    src/Timer.h
    src/Timer.cpp
)

# IMGUI source files
SET(IMGUI_SOURCES
    imgui/imgui.cpp
    imgui/imgui.h
    imgui/imguiRenderGL3.h
    imgui/imguiRenderGL3.cpp
    imgui/stb_truetype.h
)

# Common source files
SET(COMMON_SOURCES
    common/Box.h
    common/Box.cpp
    common/Cone.h
    common/Cone.cpp
    common/Sphere.h
    common/Sphere.cpp
    common/Line.h
    common/Line.cpp
    common/Capsule.h
    common/Capsule.cpp
    common/ConvexMesh.h
    common/ConvexMesh.cpp
    common/Cylinder.h
    common/Cylinder.cpp
    common/Dumbbell.h
    common/Dumbbell.cpp
    common/PhysicsObject.h
    common/PhysicsObject.cpp
    common/VisualContactPoint.h
    common/VisualContactPoint.cpp
)

# Examples scenes source files
SET(SCENES_SOURCES
    scenes/cubes/CubesScene.h
    scenes/cubes/CubesScene.cpp
    scenes/joints/JointsScene.h
    scenes/joints/JointsScene.cpp
    scenes/raycast/RaycastScene.h
    scenes/raycast/RaycastScene.cpp
    scenes/collisionshapes/CollisionShapesScene.h
    scenes/collisionshapes/CollisionShapesScene.cpp
)

# Create the executable
ADD_EXECUTABLE(testbed ${TESTBED_SOURCES} ${SCENES_SOURCES} ${COMMON_SOURCES} ${IMGUI_SOURCES})

# Link with libraries
TARGET_LINK_LIBRARIES(testbed reactphysics3d openglframework glfw ${GLFW_LIBRARIES})
