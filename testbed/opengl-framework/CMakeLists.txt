# Minimum cmake version required
CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

# Project configuration
PROJECT(OPENGLFRAMEWORK)

# Options
OPTION(USE_JPEG_TEXTURES "Select this if you want to use jpeg textures (libjpeg required)" OFF)

# Where to find the module to find special packages/libraries
SET(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/")

# Find OpenGL
FIND_PACKAGE(OpenGL REQUIRED)
IF(OPENGL_FOUND)
   MESSAGE("OpenGL found")
ELSE()
   MESSAGE("OpenGL not found")
ENDIF()

# Find the GLEW library
FIND_PACKAGE(GLEW REQUIRED)
IF(GLEW_FOUND)
   MESSAGE("GLEW found")
ELSE()
   MESSAGE("GLEW not found")
ENDIF()

# If the user wants to use JPEG textures
IF(USE_JPEG_TEXTURES)

    # Find the LIBJPEG library
    FIND_PACKAGE(JPEG REQUIRED)
    IF(JPEG_FOUND)
        MESSAGE("LIBJPEG found")
    ELSE()
        MESSAGE("LIBJPEG not found")
    ENDIF()
ENDIF()

# Headers
INCLUDE_DIRECTORIES(src ${OPENGL_INCLUDE_DIR} ${GLEW_INCLUDE_PATH} ${JPEG_INCLUDE_DIR})

# Source files
SET(OPENGL_FRAMEWORK_SOURCES
    "src/maths/Color.h"
    "src/maths/Matrix3.h"
    "src/maths/Matrix4.h"
    "src/maths/Vector2.h"
    "src/maths/Vector3.h"
    "src/maths/Vector4.h"
    "src/Camera.cpp"
    "src/Camera.h"
    "src/definitions.h"
    "src/FrameBufferObject.cpp"
    "src/FrameBufferObject.h"
    "src/Light.h"
    "src/Light.cpp"
    "src/Mesh.h"
    "src/Mesh.cpp"
    "src/MeshReaderWriter.h"
    "src/MeshReaderWriter.cpp"
    "src/Object3D.h"
    "src/Object3D.cpp"
    "src/openglframework.h"
    "src/Shader.h"
    "src/Shader.cpp"
    "src/Texture2D.h"
    "src/Texture2D.cpp"
    "src/TextureReaderWriter.h"
    "src/TextureReaderWriter.cpp"
    "src/VertexBufferObject.h"
    "src/VertexBufferObject.cpp"
)

IF(USE_JPEG_TEXTURES)
    ADD_DEFINITIONS(-DUSE_JPEG_TEXTURE)
ENDIF()

# Create the library
ADD_LIBRARY (openglframework STATIC ${OPENGL_FRAMEWORK_SOURCES})

# Link with others libraries
TARGET_LINK_LIBRARIES(openglframework ${GLEW_LIBRARIES} ${OPENGL_LIBRARY})
