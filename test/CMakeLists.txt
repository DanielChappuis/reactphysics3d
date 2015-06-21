# Minimum cmake version required
cmake_minimum_required(VERSION 2.6)

# Project configuration
PROJECT(TESTS)

IF (CMAKE_BUILD_TYPE MATCHES "Debug")
    IF (CMAKE_COMPILER_IS_GNUCXX)
        SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wno-unused-parameter")
    ENDIF (CMAKE_COMPILER_IS_GNUCXX)
ENDIF (CMAKE_BUILD_TYPE MATCHES "Debug")

# Headers
INCLUDE_DIRECTORIES(${REACTPHYSICS3D_SOURCE_DIR}/test)

# Sources files of tests
file (
  GLOB_RECURSE
  TESTS_SOURCE_FILES
  ${REACTPHYSICS3D_SOURCE_DIR}/test/*
)

# Create the tests executable
ADD_EXECUTABLE(tests ${TESTS_SOURCE_FILES})

TARGET_LINK_LIBRARIES(tests reactphysics3d)
