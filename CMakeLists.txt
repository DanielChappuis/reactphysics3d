# Minimum cmake version required
cmake_minimum_required(VERSION 2.6)

# Project configuration
project(ReactPhysics3D)
set(LIBRARY_OUTPUT_PATH lib/)

# Headers
include_directories(src)

# Library configuration
file (
  GLOB_RECURSE
  source_files
  src/*
)

add_library (
   reactphysics3d
   STATIC
   ${source_files}
)
