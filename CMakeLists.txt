# Minimum cmake version required
CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

# Project configuration
PROJECT(REACTPHYSICS3D)

# Where to build the library
SET(LIBRARY_OUTPUT_PATH lib/)

# Options
OPTION(COMPILE_EXAMPLES "Select this if you want to build the examples" OFF)

# Headers
INCLUDE_DIRECTORIES(src)

# Library configuration
file (
  GLOB_RECURSE
  source_files
  src/*
)

# Require the reactphysics3d code to be compiled in a static library
ADD_LIBRARY (
   reactphysics3d
   STATIC
   ${source_files}
)

# If we need to compile the examples
IF (COMPILE_EXAMPLES)
   add_subdirectory(examples/fallingcubes)
ENDIF (COMPILE_EXAMPLES)
