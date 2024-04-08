
 This file describes how the documentation of the project is generated.

 Reference: https://devblogs.microsoft.com/cppblog/clear-functional-c-documentation-with-sphinx-breathe-doxygen-cmake

 # Dependencies

  - Doxygen
  - Sphinx
  - Breathe (bridge between Doxygen and Sphinx)
  - Exhale (used to better display the API documentation from Doxygen)
  - CMake

 # How to generate the documentation

  1. First, we need to configure CMake to generate the documentation by enabling the RP3D_GENERATE_DOCUMENTATION option
  2. The simply run 'make' to generate the documentation


