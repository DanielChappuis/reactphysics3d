# User Documentation {#mainpage}

## Introduction {#introduction}

ReactPhysics3D is an open source C++ physics engine library that can be used
in 3D simulations and games. The library is released under the ZLib license.

## Features {#features}

   The ReactPhysics3D library has the following features:

   - Rigid body dynamics
   - Discrete collision detection
   - Collision shapes (Sphere, Box, Capsule, Convex Mesh, Static Concave Mesh, Height Field)
   - Multiple collision shapes per body
   - Broadphase collision detection (Dynamic AABB tree)
   - Narrowphase collision detection (SAT/GJK)
   - Collision response and friction (Sequential Impulses Solver)
   - Joints (Ball and Socket, Hinge, Slider, Fixed)
   - Collision filtering with categories
   - Ray casting
   - Sleeping technique for inactive bodies
   - Multi-platform (Windows, Linux, Mac OS X)
   - No external libraries (do not use STL containers)
   - Documentation (user manual and Doxygen API)
   - Testbed application with demos
   - Integrated profiler
   - Debug renderer
   - Logs
   - Unit tests

## License {#license}

The ReactPhysics3D library is released under the open-source ZLib license. For more information, read the "LICENSE" file.

## Building and installing the library {#building}

In order to build the library on your system, you first need to clone the code repository with the following command: 

    git clone https://github.com/DanielChappuis/reactphysics3d.git

Note that the *git* versioning software needs to be installed on your system. 

Then, you will need to build (compile) the library and install it on your system in order to use it in your project.
The best way is to use CMake for that. CMake will generate the necessary files on your platform (Windows, OS X or Linux) to build
the library.

CMake can be downloaded at [http://www.cmake.org](http://www.cmake.org) or using your package-management program (apt, yum, \dots) on Linux.
If you have never used CMake before, you should read [this page](http://www.cmake.org/cmake/help/runningcmake.html) as
it contains a lot of useful information. 

The remaining of this section will describe how to build and install the library with CMake. 

