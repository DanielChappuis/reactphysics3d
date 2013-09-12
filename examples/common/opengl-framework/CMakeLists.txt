# Minimum cmake version required
CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

# Project configuration
PROJECT(OPENGLFRAMEWORK)

# Options
OPTION(USE_JPEG_TEXTURES "Select this if you want to use jpeg textures (libjpeg required)" OFF)

# Where to find the module to find special packages/libraries
set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/")

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

# Find the GLUT/FREEGLUT library
IF(APPLE)
	
	# Find the GLUT library
	FIND_PACKAGE(GLUT REQUIRED)
	IF(GLUT_FOUND)
   		MESSAGE("GLUT found")
	ELSE(GLUT_FOUND)
   		MESSAGE(SEND_ERROR "GLUT not found")
    ENDIF(GLUT_FOUND)

ELSE(APPLE)

 	# Find the FREEGLUT library
	FIND_PACKAGE(FREEGLUT REQUIRED)
	IF(FREEGLUT_FOUND)
   		MESSAGE("FREEGLUT found")
		ADD_DEFINITIONS(-DUSE_FREEGLUT)
	ELSE(FREEGLUT_FOUND)
   		MESSAGE(SEND_ERROR "FREEGLUT not found")
    ENDIF(FREEGLUT_FOUND)

ENDIF(APPLE)

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

# Headers
INCLUDE_DIRECTORIES(src ${OPENGL_INCLUDE_DIR} ${GLEW_INCLUDE_PATH} ${FREEGLUT_INCLUDE_DIR} ${GLUT_INCLUDE_DIR} ${JPEG_INCLUDE_DIR})

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

TARGET_LINK_LIBRARIES(openglframework ${GLEW_LIBRARIES} ${OPENGL_LIBRARY} ${FREEGLUT_LIBRARY} ${GLUT_LIBRARY})
