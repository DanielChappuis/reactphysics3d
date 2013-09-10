# Minimum cmake version required
CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

# Project configuration
PROJECT(OPENGLFRAMEWORK)

# Options
OPTION(USE_JPEG_TEXTURES "Select this if you want to use jpeg textures (libjpeg required)" OFF)

# Where to find the module to find special packages/libraries
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/")

# Find OpenGL
FIND_PACKAGE(OpenGL REQUIRED)
if(OPENGL_FOUND)
   MESSAGE("OpenGL found")
else()
   MESSAGE("OpenGL not found")
endif()

# Find the GLEW library
FIND_PACKAGE(GLEW REQUIRED)
if(GLEW_FOUND)
   MESSAGE("GLEW found")
else()
   MESSAGE("GLEW not found")
endif()

# If the user wants to use JPEG textures
if(USE_JPEG_TEXTURES)

    # Find the LIBJPEG library
    FIND_PACKAGE(JPEG REQUIRED)
    if(JPEG_FOUND)
        MESSAGE("LIBJPEG found")
    else()
        MESSAGE("LIBJPEG not found")
    endif()
endif()

# Freeglut
add_subdirectory(freeglut)

# Headers
INCLUDE_DIRECTORIES(src freeglut ${JPEG_INCLUDE_DIR})

if(USE_JPEG_TEXTURES)
    add_definitions(-DUSE_JPEG_TEXTURE)
endif()

# Library configuration
file (
  GLOB_RECURSE
  OPENGLFRAMEWORK_SOURCES_FILES
  src/*
)

# Require the opengl-framework code to be compiled in a static library
ADD_LIBRARY (
   openglframework
   STATIC
   ${OPENGLFRAMEWORK_SOURCES_FILES}
)

TARGET_LINK_LIBRARIES(openglframework ${GLEW_LIBRARIES} ${OPENGL_LIBRARY} freeglut_static)
