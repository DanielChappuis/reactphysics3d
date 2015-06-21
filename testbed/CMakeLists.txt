# Minimum cmake version required
cmake_minimum_required(VERSION 2.6)

# Project configuration
PROJECT(Testbed)

IF (CMAKE_BUILD_TYPE MATCHES "Debug")
    IF (CMAKE_COMPILER_IS_GNUCXX)
        SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wno-unused-parameter")
    ENDIF (CMAKE_COMPILER_IS_GNUCXX)
ENDIF (CMAKE_BUILD_TYPE MATCHES "Debug")

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

# Headers
INCLUDE_DIRECTORIES("../include/reactphysics3d" "src" "opengl-framework/src/"
                    "glfw/include/" "common/" "scenes/" "imgui/")

# Testbed source files
SET(TESTBED_SOURCES
    src/Main.cpp
    src/TestbedApplication.h
    src/TestbedApplication.cpp
    src/Gui.h
    src/Gui.cpp
    src/Scene.h
    src/Scene.cpp
    src/Timer.h
    src/Timer.cpp
)

# IMGUI source files
SET(IMGUI_SOURCES
    imgui/imgui.h
    imgui/imgui.cpp
    imgui/imconfig.h
    imgui/stb_rect_pack.h
    imgui/stb_textedit.h
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
