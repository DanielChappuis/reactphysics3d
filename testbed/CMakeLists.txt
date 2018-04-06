# Minimum cmake version required
CMAKE_MINIMUM_REQUIRED(VERSION 3.2 FATAL_ERROR)

# Project configuration
PROJECT(Testbed)

# Where to build the executables
SET(EXECUTABLE_OUTPUT_PATH "${OUR_EXECUTABLE_OUTPUT_PATH}/testbed")
SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${EXECUTABLE_OUTPUT_PATH})
SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE ${EXECUTABLE_OUTPUT_PATH})
SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG ${EXECUTABLE_OUTPUT_PATH})

ADD_SUBDIRECTORY(nanogui/)

# Copy the shaders used for the demo into the build directory
FILE(COPY "shaders/" DESTINATION "${EXECUTABLE_OUTPUT_PATH}/shaders/")

# Copy the meshes used for the demo into the build directory
FILE(COPY "meshes/" DESTINATION "${EXECUTABLE_OUTPUT_PATH}/meshes/")

# Headers
INCLUDE_DIRECTORIES("src/" "nanogui/include/" "opengl-framework/src/" "common/" "scenes/" ${NANOGUI_EXTRA_INCS})

# OpenGLFramework source files
SET(OPENGLFRAMEWORK_SOURCES
    opengl-framework/src/maths/Color.h
    opengl-framework/src/maths/Matrix3.h
    opengl-framework/src/maths/Matrix4.h
    opengl-framework/src/maths/Vector2.h
    opengl-framework/src/maths/Vector3.h
    opengl-framework/src/maths/Vector4.h
    opengl-framework/src/Camera.cpp
    opengl-framework/src/Camera.h
    opengl-framework/src/definitions.h
    opengl-framework/src/FrameBufferObject.cpp
    opengl-framework/src/FrameBufferObject.h
    opengl-framework/src/Light.h
    opengl-framework/src/Light.cpp
    opengl-framework/src/Mesh.h
    opengl-framework/src/Mesh.cpp
    opengl-framework/src/MeshReaderWriter.h
    opengl-framework/src/MeshReaderWriter.cpp
    opengl-framework/src/Object3D.h
    opengl-framework/src/Object3D.cpp
    opengl-framework/src/openglframework.h
    opengl-framework/src/Shader.h
    opengl-framework/src/Shader.cpp
    opengl-framework/src/Texture2D.h
    opengl-framework/src/Texture2D.cpp
    opengl-framework/src/TextureReaderWriter.h
    opengl-framework/src/TextureReaderWriter.cpp
    opengl-framework/src/VertexBufferObject.h
    opengl-framework/src/VertexBufferObject.cpp
    opengl-framework/src/VertexArrayObject.h
    opengl-framework/src/VertexArrayObject.cpp
)

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

# Common source files
SET(COMMON_SOURCES
    common/Box.h
    common/Box.cpp
    common/Sphere.h
    common/Sphere.cpp
    common/Line.h
    common/Line.cpp
    common/Capsule.h
    common/Capsule.cpp
    common/ConvexMesh.h
    common/ConvexMesh.cpp
    common/ConcaveMesh.h
    common/ConcaveMesh.cpp
    common/Dumbbell.h
    common/Dumbbell.cpp
    common/HeightField.h
    common/HeightField.cpp
    common/PhysicsObject.h
    common/PhysicsObject.cpp
    common/VisualContactPoint.h
    common/VisualContactPoint.cpp
    common/PerlinNoise.h
    common/PerlinNoise.cpp
    common/AABB.h
    common/AABB.cpp
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
    scenes/collisiondetection/CollisionDetectionScene.h
    scenes/collisiondetection/CollisionDetectionScene.cpp
    scenes/concavemesh/ConcaveMeshScene.h
    scenes/concavemesh/ConcaveMeshScene.cpp
    scenes/heightfield/HeightFieldScene.h
    scenes/heightfield/HeightFieldScene.cpp
    scenes/cubestack/CubeStackScene.h
    scenes/cubestack/CubeStackScene.cpp
)

# Add .user file to set debug path in Visual Studio
SET(USER_FILE testbed.vcxproj.user)
SET(VS_USERFILE_WORKING_DIRECTORY_DEBUG ${EXECUTABLE_OUTPUT_PATH})
SET(OUTPUT_PATH ${CMAKE_CURRENT_BINARY_DIR}/${USER_FILE})
CONFIGURE_FILE(VisualStudioUserTemplate.user ${USER_FILE} @ONLY)

# Compile definitions
ADD_DEFINITIONS(${NANOGUI_EXTRA_DEFS})

# Create the executable
ADD_EXECUTABLE(testbed ${TESTBED_SOURCES} ${SCENES_SOURCES} ${COMMON_SOURCES} ${OPENGLFRAMEWORK_SOURCES})

# Enable C++11 features
set_property(TARGET testbed PROPERTY CXX_STANDARD 11)
set_property(TARGET testbed PROPERTY CXX_STANDARD_REQUIRED ON)

# Link with libraries
TARGET_LINK_LIBRARIES(testbed reactphysics3d nanogui ${NANOGUI_EXTRA_LIBS})
