# Changelog

## Develop

### Added

 - It is now possible to change the size of a BoxShape using the BoxShape::setHalfExtents() method
 - It is now possible to change the radius of a SphereShape using the SphereShape::setRadius() method
 - It is now possible to change the height and radius of a CapsuleShape using the CapsuleShape::setHeight() and CapsuleShape::setRadius methods
 - It is now possible to change the scale of a ConvexMeshShape using the ConvexMeshShape::setScale() method
 - It is now possible to change the scale of a ConcaveMeshShape using the ConcaveMeshShape::setScale() method
 - It is now possible to change the scale of a HeightFieldShape using the HeightFieldShape::setScale() method
 - A method PhysicsWorld::getCollisionBody(uint index) has been added on a physics world to retrieve a given CollisionBody
 - A method PhysicsWorld::getRigidBody(uint index) has been added on a physics world to retrieve a given RigidBody
 - The Material of a Collider now has a mass density parameter that is used to compute its mass
 - A RigidBody::getLocalCenterOfMass() method has been added to retrieve the current center of mass of a rigid body
 - Add PhysicsCommon class that needs to be instanciated at the beginning and is used as a factory for other objects of the library
 - The RigidBody::updateLocalCenterOfMassFromColliders() method has been added to compute and set the center of mass of a body using its colliders
 - The RigidBody::updateLocalInertiaTensorFromColliders() method has been added to compute and set the local inertia tensor of a body using its colliders
 - The RigidBody::getLocalInertiaTensor() method has been added to retrieve the local-space inertia tensor of a rigid body.
 - The RigidBody::updateMassFromColliders() method has been added to compute and set the mass of a body using its colliders
 - A Material nows has a mass density parameter that can be set using the Material::setMassDensity() method. The mass density is used to compute the mass of a collider when computing the mass of a rigid body

### Changed

 - The CollisionWorld::testCollision() methods do not have the 'categoryMaskBits' parameter anymore.
 - The CollisionWorld::testOverlap() methods do not have the 'categoryMaskBits' parameter anymore.
 - Many methods in the EventListener class have changed. Check the user manual for more information.
 - The way to retrieve contacts from a CollisionCallbackInfo object has changed. Check the user manual for more information.
 - Now, the collision shapes need be be created with the PhysicsWorld::createXXXShape() methods instead of using the constructor of the shape as before. For instance, you need to use the PhysicsWorld::createBoxShape() method to create a BoxShape.
 - DynamicsWorld and CollisionWorld classes have been merged into a single class called PhysicsWorld.
 - ProxyShape class has been renamed into Collider.
 - The Material is now part of the Collider instead of the RigidBody. Therefore, it is now possible to have a RigidBody with multiple
   colliders and a different material per Collider.
 - The Logger has to be instanciated using the PhysicsCommon::createLogger() method instead of using its constructor.
 - The Profiler has to be instanciated using the PhysicsCommon::createProfiler() method instead of using its constructor.
 - There is now a single MemoryManager (with memory allocators) per PhysicsWorld. The memory allocators are no longer shared between worlds. 
 - The Box::getExtent() method has been renamed to Box::getHalfExtents()
 - An instance of the BoxShape class cannot be instanciated directly anymore. You need to use the PhysicsCommon::createBoxShape() method.
 - An instance of the SphereShape class cannot be instanciated directly anymore. You need to use the PhysicsCommon::createSphereShape() method.
 - An instance of the CapsuleShape class cannot be instanciated directly anymore. You need to use the PhysicsCommon::createCapsuleShape() method.
 - An instance of the ConvexMeshShape class cannot be instanciated directly anymore. You need to use the PhysicsCommon::createConvexMeshShape() method.
 - An instance of the HeightFieldShape class cannot be instanciated directly anymore. You need to use the PhysicsCommon::createHeightFieldShape() method.
 - An instance of the ConcaveMeshShape class cannot be instanciated directly anymore. You need to use the PhysicsCommon::createConcaveMeshShape() method.
 - An instance of the PolyhedronMesh class cannot be instanciated directly anymore. You need to use the PhysicsCommon::createPolyhedronMesh() method.
 - An instance of the TriangleMesh class cannot be instanciated directly anymore. You need to use the PhysicsCommon::createTriangleMesh() method.
 - The ProxyShape class has been renamed to Collider. The CollisionBody::addCollider(), RigidBody::addCollider() methods have to be used to create and add a collider to a body. Then methods CollisionBody::removeCollider(), RigidBody::removeCollider() need to be used to remove a collider.
 - The RigidBody::addCollider() method (previously addProxyShape() method) does not take a "mass" parameter anymore.
 - The RigidBody::setCenterOfMassLocal() method has been renamed to RigidBody::setLocalCenterOfMass()
 - The RigidBody::setInertiaTensorLocal() method has been renamed to RigidBody::setLocalInertiaTensor()
 - Now, the local inertia tensor of a rigid body has to be set using a Vector3 instead of a Matrix3x3. You only need to provide the diagonal values of the matrix
 - The RigidBody::recomputeMassInformation() method has been renamed to RigidBody::updateMassPropertiesFromColliders.
 - Now, you need to manually call the RigidBody::recomputeMassInformation() method after adding colliders to a rigid body to recompute its inertia tensor, center of mass and mass
 - The rendering in the testbed application has been improved

### Removed

 - DynamicsWorld::getContactsList(). You need to use the EventListener class to retrieve contacts now.
 - The DynamicsWorld::getNbJoints() method has been removed.
 - The DynamicsWorld::getNbRigidBodies() method has been removed.
 - The EventListener::beginInternalTick() method has been removed (because internal ticks do not exist anymore).
 - The EventListener::endInternalTick() method has been removed (because internal ticks do not exist anymore).
 - The RigidBody::getJointsList() method has been removed.
 - It is not possible anymore to set custom pool and stack frame allocators. Only the base allocator can be customized when creating a PhysicsCommon instance.
 - The RigidBody::setInverseInertiaTensorLocal() method has been removed. The RigidBody::setInertiaTensorLocal() has to be used instead.
 - The RigidBody::getInverseInertiaTensorWorld() method has been removed.
 - The Collider::getMass() method has been removed.

## Version 0.7.1 (July 01, 2019)

### Added

 - Make possible for the user to get vertices, normals and triangle indices of a ConcaveMeshShape
 - Make possible for the user to get vertices and height values of the HeightFieldShape

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

### Removed

 - The CollisionWorld::setCollisionDispatch() method has been removed. In order to use a custom collision
   algorithm, you must not get the collision dispatch object with the
   CollisionWorld::getCollisionDispatch() method and set a collision algorithm to this object.
 - The methods CollisionBody::getProxyShapesList() has been remove. You can now use the
   CollisionBody::getNbProxyShapes() method to know the number of proxy-shapes of a body and the
   CollisionBody::getProxyShape(uint proxyShapeIndex) method to get a given proxy-shape of the body.
 - The CollisionWorld::testAABBOverlap() methods have been removed.

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

