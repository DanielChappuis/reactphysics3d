# This is the CMakeLists.txt file of the "hello world" project
# in order to show a simple executable that uses the ReactPhysics3D library

# Minimum cmake version required
cmake_minimum_required(VERSION 3.8)

# Import the ReactPhysics3D library that you have installed on your computer using
# the "make install" command
find_package(ReactPhysics3D REQUIRED)

# Project
project(HelloWorld)

# Create the executable
add_executable(helloworld Main.cpp)

# Link with the ReactPhysics3D library
target_link_libraries(helloworld ReactPhysics3D::ReactPhysics3D)
