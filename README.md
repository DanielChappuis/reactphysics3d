[![Travis Build Status](https://travis-ci.org/DanielChappuis/reactphysics3d.svg?branch=master)](https://travis-ci.org/DanielChappuis/reactphysics3d)
[![Codacy Badge](https://api.codacy.com/project/badge/Grade/3ae24e998e304e4da78ec848eade9e3a)](https://www.codacy.com/app/chappuis.daniel/reactphysics3d?utm_source=github.com&amp;utm_medium=referral&amp;utm_content=DanielChappuis/reactphysics3d&amp;utm_campaign=Badge_Grade)
[![codecov.io](https://codecov.io/github/DanielChappuis/reactphysics3d/coverage.svg?branch=master)](https://codecov.io/github/DanielChappuis/reactphysics3d?branch=master)

## ReactPhysics3D

ReactPhysics3D is an open source C++ physics engine library that can be used in 3D simulations and games.

Website : [https://www.reactphysics3d.com](https://www.reactphysics3d.com)

Author : Daniel Chappuis

<img src="https://raw.githubusercontent.com/DanielChappuis/reactphysics3d/master/documentation/UserManual/images/testbed.png" alt="Drawing" height="400" />

## Features

ReactPhysics3D has the following features:

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
 - Debugging renderer
 - Logs
 - Unit tests

## License

The ReactPhysics3D library is released under the open-source [ZLib license](http://opensource.org/licenses/zlib).

## Documentation

You can find the user manual and the Doxygen API documentation [here](https://www.reactphysics3d.com/documentation.html)

## Branches

The "master" branch always contains the last released version of the library and some possible bug fixes. This is the most stable version. On the other side,
the "develop" branch is used for development. This branch is frequently updated and can be quite unstable. Therefore, if you want to use the library in
your application, it is recommended to checkout the "master" branch.

## Issues

If you find any issue with the library, you can report it on the issue tracker [here](https://github.com/DanielChappuis/reactphysics3d/issues).

## Credits

Thanks a lot to Erin Catto, Dirk Gregorius, Erwin Coumans, Pierre Terdiman and Christer Ericson for their amazing GDC presentations,
their physics engines, their books or articles and their contributions on many physics engine forums.

