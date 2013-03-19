# Minimum cmake version required
cmake_minimum_required(VERSION 2.6)

# Project configuration
PROJECT(FallingCubes)

# Find Glut or Freeglut
FIND_PACKAGE(GLUT)
if (NOT GLUT_FOUND)
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

# Headers
if(GLUT_FOUND)
    INCLUDE_DIRECTORIES(${REACTPHYSICS3D_SOURCE_DIR}/src ${GLUT_INCLUDE_DIR})
elseif(FREEGLUT_FOUND)
    INCLUDE_DIRECTORIES(${REACTPHYSICS3D_SOURCE_DIR}/src ${FREEGLUT_INCLUDE_DIRS})
endif()

# Definitions
if (FREEGLUT_FOUND)
    ADD_DEFINITIONS(-DUSE_FREEGLUT)
endif()

# Create the example executable using the
# compiled reactphysics3d static library
ADD_EXECUTABLE(fallingcubes main.cpp Box.cpp Box.h)

if(GLUT_FOUND)
   TARGET_LINK_LIBRARIES(fallingcubes reactphysics3d ${GLUT_LIBRARY} ${OPENGL_LIBRARY})
elseif(FREEGLUT_FOUND)
   TARGET_LINK_LIBRARIES(fallingcubes reactphysics3d ${FREEGLUT_LIBRARY} ${OPENGL_LIBRARY})
endif()
