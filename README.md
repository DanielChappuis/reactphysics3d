[![Travis Build Status](https://travis-ci.org/DanielChappuis/reactphysics3d.svg?branch=master)](https://travis-ci.org/DanielChappuis/reactphysics3d)

## ReactPhysics3D

ReactPhysics3D is an open source C++ physics engine library that can be used in 3D simulations and games.

Website : [http://www.reactphysics3d.com](http://www.reactphysics3d.com)

Author : Daniel Chappuis

<img src="https://raw.githubusercontent.com/DanielChappuis/reactphysics3d/develop/documentation/UserManual/images/testbed.png" alt="Drawing" height="400" />

## Features

ReactPhysics3D has the following features :

- Rigid body dynamics
- Discrete collision detection
- Collision shapes (Sphere, Box, Cone, Cylinder, Capsule, Convex Mesh, static Concave Mesh, Height Field)
- Multiple collision shapes per body
- Broadphase collision detection (Dynamic AABB tree)
- Narrowphase collision detection (GJK/EPA)
- Collision response and friction (Sequential Impulses Solver)
- Joints (Ball and Socket, Hinge, Slider, Fixed)
- Collision filtering with categories
- Ray casting
- Sleeping technique for inactive bodies
- Integrated Profiler
- Multi-platform (Windows, Linux, Mac OS X)
- No dependencies (only OpenGL for the testbed application)
- Documentation (User manual and Doxygen API)
- Testbed application with demo scenes
- Unit tests

## License

The ReactPhysics3D library is released under the open-source [ZLib license](http://opensource.org/licenses/zlib).

## Documentation

You can find the User Manual and the Doxygen API Documentation [here](http://www.reactphysics3d.com/documentation.html)

## Branches

The "master" branch always contains the last released version of the library and some possible bug fixes. This is the most stable version. On the other side,
the "develop" branch is used for development. This branch is frequently updated and can be quite unstable. Therefore, if you want to use the library in
your application, it is recommended to checkout the "master" branch.

## Issues

If you find any issue with the library, you can report it on the issue tracker [here](https://github.com/DanielChappuis/reactphysics3d/issues).
