# Minimum cmake version required
cmake_minimum_required(VERSION 2.6)

# Project configuration
PROJECT(FallingCubes)

# Headers
INCLUDE_DIRECTORIES(${REACTPHYSICS3D_SOURCE_DIR}/src)

# Find Glut or Freeglut
if(WIN32 OR APPLE)
   # Find the necessary libraries
   FIND_PACKAGE(GLUT)
else()
   # Find the Freeglut library
   FIND_PATH(FREEGLUT_INCLUDE_DIR NAMES GL/freeglut.h
             /usr/include/GL
             )
   FIND_LIBRARY(FREEGLUT_LIBRARY NAMES freeglut glut)
   INCLUDE(FindPackageHandleStandardArgs)
   FIND_PACKAGE_HANDLE_STANDARD_ARGS(FREEGLUT DEFAULT_MSG FREEGLUT_LIBRARY FREEGLUT_INCLUDE_DIR)
   IF(FREEGLUT_FOUND)
      MESSAGE("Freeglut found")
      SET(FREEGLUT_LIBRARIES ${FREEGLUT_LIBRARY})
      SET(FREEGLUT_INCLUDE_DIRS ${FREEGLUT_INCLUDE_DIR})
   else()
      MESSAGE("Freeglut not found (required to build the examples)")
      SET(FREEGLUT_LIBRARIES)
      SET(FREEGLUT_INCLUDE_DIRS)
   endif()
endif()

# Find OpenGL
FIND_PACKAGE(OpenGL)
if(OPENGL_FOUND)
   MESSAGE("OpenGL found")
else()
   MESSAGE("OpenGL not found")
endif()


# Create the example executable using the
# compiled reactphysics3d static library
ADD_EXECUTABLE(fallingcubes main.cpp Box.cpp Box.h)

if(WIN32 OR APPLE)
   TARGET_LINK_LIBRARIES(fallingcubes reactphysics3d ${GLUT_LIBRARY} ${OPENGL_LIBRARY})
elseif(FREEGLUT_FOUND)
   TARGET_LINK_LIBRARIES(fallingcubes reactphysics3d ${FREEGLUT_LIBRARY} ${OPENGL_LIBRARY})
endif()
