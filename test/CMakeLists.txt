# Minimum cmake version required
cmake_minimum_required(VERSION 2.6)

# Project configuration
PROJECT(TESTS)

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

ADD_TEST(Test tests)
