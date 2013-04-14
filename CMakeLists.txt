# Minimum cmake version required
CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

# Project configuration
PROJECT(REACTPHYSICS3D)

# Where to build the library
SET(LIBRARY_OUTPUT_PATH lib/)

# Options
OPTION(COMPILE_EXAMPLES "Select this if you want to build the examples" OFF)
OPTION(COMPILE_TESTS "Select this if you want to build the tests" OFF)
OPTION(PROFILING_ENABLED "Select this if you want to compile with enabled profiling" OFF)

# Headers
INCLUDE_DIRECTORIES(src)

IF (PROFILING_ENABLED)
	ADD_DEFINITIONS(-DIS_PROFILING_ACTIVE)
ENDIF (PROFILING_ENABLED)

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
   add_subdirectory(examples/)
ENDIF (COMPILE_EXAMPLES)

# If we need to compile the tests
IF (COMPILE_TESTS)
   add_subdirectory(test/)
ENDIF (COMPILE_TESTS)
