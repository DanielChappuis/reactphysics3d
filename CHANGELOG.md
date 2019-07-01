# Changelog

## Version 0.7.1 (July 01, 2019)

### Added

 - Make possible for the user to get vertices, normals and triangle indices of a ConcaveMeshShape
 - Make possible for the user to get vertices and height values of the HeightFieldShape
 - Make possible for the user to use a custom single frame and pool memory allocator

### Fixed

 - Bug [#45](https://github.com/DanielChappuis/reactphysics3d/issues/45) has been fixed.
 - Bug [#50](https://github.com/DanielChappuis/reactphysics3d/issues/50) has been fixed.
 - Bug [#52](https://github.com/DanielChappuis/reactphysics3d/issues/52) has been fixed.
 - Bug [#53](https://github.com/DanielChappuis/reactphysics3d/issues/53) has been fixed.
 - Bug [#54](https://github.com/DanielChappuis/reactphysics3d/issues/54) has been fixed.
 - Bug [#55](https://github.com/DanielChappuis/reactphysics3d/issues/55) has been fixed.
 - Bug [#51](https://github.com/DanielChappuis/reactphysics3d/issues/51) has been fixed.
 - Bug [#60](https://github.com/DanielChappuis/reactphysics3d/issues/60) has been fixed.
 - Bug [#57](https://github.com/DanielChappuis/reactphysics3d/issues/57) has been fixed.
 - Bug [#37](https://github.com/DanielChappuis/reactphysics3d/issues/37) has been fixed.
 - Bug [#62](https://github.com/DanielChappuis/reactphysics3d/issues/62) has been fixed.
 - Bug [#63](https://github.com/DanielChappuis/reactphysics3d/issues/63) has been fixed.
 - Bug [#82](https://github.com/DanielChappuis/reactphysics3d/issues/82) has been fixed.
 - Bug [#85](https://github.com/DanielChappuis/reactphysics3d/issues/85) has been fixed.
 - Bug [#79](https://github.com/DanielChappuis/reactphysics3d/issues/79) has been fixed.
 - Bug: the free() method was called in PoolAllocator instead of release() method of base allocator.

## Version 0.7.0 (May 1, 2018)

### Added
 
 - Dedicated Sphere vs Capsule collision detection algorithm.
 - Dedicated Capsule vs Capsule collision detection algorithm.
 - Allow a Rigid Body to have zero mass (for static bodies for instance).
 - Add single frame memory allocator for faster memory allocation in a frame.
 - Make possible to have a profiler per DynamicsWorld instead of a unique one.
 - Make possible to use your own allocation/deallocation methods instead of default malloc/free
 - Make possible to display the AABB of the bodies in the testbed application.
 - Add RigidBody::setInverseLocalInertiaTensor() method to directly set the inverse inertia tensor of a rigid body.
 - More unit tests have been added

### Changed

 - Use single-shot contact manifold computation instead of incremental across several frames.
 - Replace the EPA narrow-phase collision detection with SAT algorithm.
 - The collision detection is now faster and more robust.
 - Code has been refactored such that we do not use STL containers anymore.
 - The way to create a ConvexMeshShape has changed (see the user manual)
 - The vertex indices stride has changed in the TriangleVertexArray for ConvexMeshShape (see the API documentation)
 - The raycasting of a ConvexMeshShape does not use GJK algorithm anymore.
 - The test do detect if a point is inside of a ConvexMeshShape does not use GJK algorithm anymore.
 - A lot of optimizations have been performed and the library is now faster.
 - Release code is now compiled with -O2 compiler optimization level (instead of none)
 - Documentation has been updated

### Removed
 
 - Quaternion constructor with Euler angles has been removed. The Quaternion::fromEulerAngles() method should be used instead.
 - Cylinder and Cone collision shapes have been removed. The ConvexMeshShape collision shape should be used instead.
 - The ProxyShape::setLocalScaling() method has been removed. The ConvexMeshShape, ConcaveMeshShape and HeightFieldShape
   collision shapes can be scaled directly.

### Fixed

 - Issues with checkboxes in testbed application were fixed.
 - Fix issue with wrong AABB computation that can cause missing collision in broad-phase
 - Bug [#26](https://github.com/DanielChappuis/reactphysics3d/issues/26) has been fixed.
 - Bug [#30](https://github.com/DanielChappuis/reactphysics3d/issues/30) has been fixed.
 - Bug [#32](https://github.com/DanielChappuis/reactphysics3d/issues/32) has been fixed.
 - Issue [#31](https://github.com/DanielChappuis/reactphysics3d/issues/31) has been fixed.
 - Issue [#34](https://github.com/DanielChappuis/reactphysics3d/issues/34) has been fixed.
 - Issue [#37](https://github.com/DanielChappuis/reactphysics3d/issues/37) has been fixed.

## Version 0.6.0 (April 15, 2016)

### Added

 - Support for static concave triangular mesh collision shape.
 - Support for height field collision shape.
 - Support for rolling resistance.
 - It is now possible to change the local scaling of a collision shape.
 - Add new constructor of ConvexMeshShape that accepts a triangular mesh.
 - It is now easier to use your own narrow-phase collision detection algorithm.
 - The CollisionWorld and DynamicsWorld now automatically destroys bodies and joints that have not been destroyed at the end.
 - New testbed application with demo scenes.

### Changed

 - The DynamicsWorld::update() method now takes the time step for the next simulation step in parameter.

## Version 0.5.0 (March 4, 2015)

### Added

 - It is now possible to use multiple collision shapes per body.
 - Ray casting support.
 - Add methods to check if a point is inside a body or a proxy shape.
 - Add collision filtering using collision categories (with bit masks).
 - It is possible to attach user data to a body or a proxy shape.
 - It is now possible to create a quaternion using Euler angles.
 - It now possible to activate of deactivate a body.
 - Differentiation between dynamic, kinematic and static bodies.
 - Gravity can now be changed after the creation of the world.
 - The center of mass and inertia tensor of a body can be set manually (center of mass can be different from body origin).
 - Add a simulation step callback in the EventListener class that is called at each internal physics tick.
 - Add methods to transform points/vectors from/into local-space/world-space coordinates of a body.
 - Add CollisionWorld::testAABBOverlap() method to test overlap between two bodies or two proxy shapes.
 - Add a ray casting example.
 - Add unit tests for ray casting and collision detection.

### Changed

 - Replace the Sweep-And-Prune algorithm by a Dynamic AABB Tree for the broad-phase collision detection.
 - The center of mass of a body is now automatically computed from its collision shapes.
 - Use GLFW instead of GLUT/Freeglut for the examples.

### Fixed

 - Fix two issues in the EPA algorithm.

## Version 0.4.0 (October 7, 2013)

### Added

 - Add collision shapes (Capsule, Convex Mesh).
 - Add joints (Ball and Socket, Hinge, Slider, Fixed).
 - Add sleeping technique for inactive bodies.
 - Add velocity damping.
 - It is now easier to apply force and torque to a rigid body.
 - Add the EventListener class to allow the user to be notified when some events occur (contacts, …).
 - Add examples for the joints and collision shapes.
 - Make possible to modify the collision margin of some collision shapes.
 - Add a Material class to keep track of the properties of a rigid body (friction coefficient, bounciness, …).
 - Add a hierarchical real-time profiler.

### Changed

 - Collision shapes now use the internal memory allocator.
 - New internal memory allocator.

### Removed

 - Remove the world gravity force from the external force of rigid bodies and allow the user to disable the gravity on a given body.
 - Reduce the allocated memory of the broad-phase when several bodies are removed from the world.

### Fixed

 - Fix issue in the Sweep-And-Prune broad-phase collision detection (thanks to Aleksi Sapon).
 - Fix issue in the contact solver resulting in less jittering.


## Version 0.3.0

### Added

 - Implementation of a dedicated collision detection algorithm for spheres against spheres instead of using GJK/EPA algorithm.
 - Make possible to use a unique instance of a collision shape for multiple rigid bodies.
 - Create the API documentation using Doxygen.
 - Add Unit tests

### Changed

 - The Sweep-and-Prune broad-phase collision detection algorithm has been rewritten according to the technique described by Pierre Terdiman at http://www.codercorner.com/SAP.pdf to be much more efficient than the previous naive implementation.
 - The contact solver has been rewritten to use the Sequential Impulses technique from Erin Catto which is mathematically equivalent to the Projected Gauss Seidel technique that was used before. The Sequential Impulses technique is more intuitive.
 - Make GJK/EPA algorithm more robust for spheres.
 - Change the structure of the code for a better separation between the collision detection and the dynamics simulation code.

## Version 0.2.0

### Added

 - Add the GJK/EPA algorithm for convex shapes collision detection
 - Persistent contacts storing between frames
 - Add Sphere, Cylinder and Cone collision shapes
 - Add the Transform class for better handling of position and orientation of rigid bodies
 - It is now possible to approximate the inertia tensor of rigid bodies using the collision shape inertia tensor.
 - The AABB is now computed automatically using the collision shape
 - Add the MemoryPool class to avoid intense dynamic allocation during the simulation

### Changed

 - Simplification of the mathematics library
 - Optimization of the constraint solver
 - ReactPhysics3D is now under the zlib license

