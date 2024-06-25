# Changelog

## [0.10.1] - 2024-06-25

### Fixed

- Issue [#377](https://github.com/DanielChappuis/reactphysics3d/issues/377) Assert when destroying an empty PhysicsWorld
- Issue [#378](https://github.com/DanielChappuis/reactphysics3d/issues/378) Wrong raycasting result against the HeightFieldShape
- Issue [#381](https://github.com/DanielChappuis/reactphysics3d/issues/381) Fix robustness issue with CapsuleShape vs CapsuleShape collision
- Issue [#387](https://github.com/DanielChappuis/reactphysics3d/issues/387) Fix memory allocation issue when destroying a ConvexMesh
- Issue [#388](https://github.com/DanielChappuis/reactphysics3d/issues/388) Fix bodies without simulation collider not taken into account during islands creation
- Fix crash within testbed application in raycasting scene

## [0.10.0] - 2024-03-10

### Changed

- The library must now be compiled with a C++ 17 compiler
- The internal allocators now allocates memory that is 16-bytes aligned
- If the user sets its own custom allocator, the return allocated memory must now be 16 bytes aligned
- The PolyhedronMesh class has been renamed to ConvexMesh
- The PhysicsCommon::createPolyhedronMesh() method has been renamed to PhysicsCommon::createConvexMesh()
- The PhysicsCommon::destroyPolyhedronMesh() method has been renamed to PhysicsCommon::destroyConvexMesh()
- The PhysicsCommon::createConvexMesh() nows outputs a list of errors that might have happened during the mesh creation
- The PhysicsCommon::createConvexMesh() method now takes a reference to PolygonVertexArray
- When creating a ConvexMesh with PhysicsCommon::createConvexMesh(), the user data (vertices, faces) is now copied into the ConvexMesh and not shared anymore
- The PhysicsCommon::createTriangleMesh() method now directly takes a TriangleVertexArray
- The PhysicsCommon::createTriangleMesh() nows outputs a list of errors that might have happened during the mesh creation
- When creating a TriangleMesh with PhysicsCommon::createTriangleMesh(), the user data (vertices, faces) is now copied into the TriangleMesh and not shared anymore
- The PhysicsCommon::createHeightField() must be used to create a HeightField object 
- The PhysicsCommon::createHeightFieldShape() method now takes a HeightField object
- It is not necessary anymore to specify the min/max height when creating a HeightFieldShape
- It is not possible anymore to specify the up axis when creating a HeightFieldShape
- When creating a HeightField with PhysicsCommon::createHeightField(), the user data (heights values) is now copied into the HeightField and not shared anymore
- The signature of the TriangleVertexArray::getTriangleVerticesIndices() method has changed
- The signature of the TriangleVertexArray::getNormal() method has changed
- The getLocalBounds() methods of the collision shapes now returns an AABB
- It is now necessary to enable debug rendering for each body that you want to debug using the Body::setIsDebugEnabled() method

### Added

- The library will now return errors found in input data during the creation of ConvexMesh, TriangularMesh and HeighField
- It is now possible to create a ConvexMeshShape by specifying only a list of vertices (automatic computation of convex hull using internal
QuickHull algorithm)
- The performance of static bodies has been improved 
- The reporting of contact state is now correct even if the body goes to sleep
- The DebugRenderer can now display the normals of the collider faces for debugging purpose
- It is now possible to select for which bodies the debug information from the DebugRenderer is displayed

### Removed

- The TriangleMesh does not support adding multiple parts of a mesh anymore. 
- The TriangleMesh::addSubpart() method has been removed. The PhysicsCommon::createTriangleMesh() method should be used instead
- The TriangleMesh::getSubpart() method has been removed. 
- The TriangleMesh::getNbSubparts() method has been removed.
- When creating a HeightField, it is not possible to specify the up axis anymore (changing the Transform of the Collider must be used instead)
- No need to specify the min/max height when creating a HeightField anymore (this is now automatically computed)
- The HeightFiedShape::getNbColumns() method has been removed (HeightFieldShape::getHeightField()->getNbColumns() must be used instead)
- The HeightFiedShape::getNbRows() method has been removed (HeightFieldShape::getHeightField()->getNbRows() must be used instead)
- The HeightFiedShape::getHeightAt() method has been removed (HeightFieldShape::getHeightField()->getHeightAt() must be used instead)
- The CollisionBody class has been removed (RigidBody class must be used instead with a Collider where isSimulationCollider is disabled)
- The PhysicsWorld::createCollisionBody() method has been removed 
- The PhysicsWorld::destroyCollisionBody() method has been removed 
- The PhysicsWorld::getCollisionBody() method has been removed 
- The PhysicsWorld::getNbCollisionBodies() method has been removed 

### Fixed

- Issue [#206](https://github.com/DanielChappuis/reactphysics3d/issues/206) Collision issue and scaling of collider normals
- Issue [#235](https://github.com/DanielChappuis/reactphysics3d/issues/235) Removing a body should wake up its neighbors
- Issue [#237](https://github.com/DanielChappuis/reactphysics3d/issues/237) Wrong assert has been removed 
- Issue [#239](https://github.com/DanielChappuis/reactphysics3d/issues/239) Memory allocation alignment
- Issue [#240](https://github.com/DanielChappuis/reactphysics3d/issues/240) Uninitialized variable
- Issue [#347](https://github.com/DanielChappuis/reactphysics3d/issues/347) Missing collision between capsule and triangle edge in some case
- Issue [#362](https://github.com/DanielChappuis/reactphysics3d/issues/362) Bug in Collider::setLocalToBodyTransform()
- Issue [#275](https://github.com/DanielChappuis/reactphysics3d/issues/275) Compilation warning
- Issue [#286](https://github.com/DanielChappuis/reactphysics3d/issues/286) Compilation error on Android
- Issue [#323](https://github.com/DanielChappuis/reactphysics3d/issues/323) Avoid conflict with X11 library
- Issue [#362](https://github.com/DanielChappuis/reactphysics3d/issues/362) Crash
- Issue [#364](https://github.com/DanielChappuis/reactphysics3d/issues/364) Assert in createContacts() method
- Issue [#366](https://github.com/DanielChappuis/reactphysics3d/issues/366) Crash when creating islands
- Issue [#370](https://github.com/DanielChappuis/reactphysics3d/issues/370) Compilation error on recent compiler
- Issue with edge vs edge collision detection for BoxShape, ConvexMeshShape, ConcaveMeshShape and HeightFieldShape (SAT algorithm)
- Compilation error on Clang 19

## [0.9.0] - 2022-01-04

### Changed

- The PhysicsWorld::setGravity() method now takes a const parameter
- Rolling resistance constraint is not solved anymore in the solver. Angular damping needs to be used instead to simulate it.
- The List class has been renamed to Array
- The default number of iterations for the velocity solver is now 6 instead of 10
- The default number of iterations for the position solver is now 3 instead of 5
- Rename method RigidBody::applyForceAtWorldPosition() into RigidBody::applyWorldForceAtWorldPosition()
- Rename method RigidBody::applyForceAtLocalPosition() into RigidBody::applyWorldForceAtLocalPosition()
- Rename method RigidBody::applyForceToCenterOfMass() into RigidBody::applyWorldForceAtCenterOfMass()
- Rename method RigidBody::applyTorque() into RigidBody::applyWorldTorque()
- The raycasting broad-phase performance has been improved
- The raycasting performance against HeighFieldShape has been improved (better middle-phase algorithm)
- Robustness of polyhedron vs polyhedron collision detection has been improved in SAT algorithm (face contacts are favored over edge-edge contacts for better stability)

### Added

- The performance of the collision detection and rigid bodies simulation (PhysicsWorld::update() method) has been improved significantly (1.7x speedup on average measured in [PEEL](https://github.com/Pierre-Terdiman/PEEL) scenes) 
- Method RigidBody::resetForce() to reset the accumulated external force on a rigid body has been added
- Method RigidBody::resetTorque() to reset the accumulated external torque on a rigid body has been added
- Constructors with local-space anchor/axis have been added to BallAndSocketJointInfo, HingeJointInfo, FixedJointInfo and SliderJointInfo classes
- Method HingeJoint::getAngle() to get the current angle of the hinge joint has been added 
- Method Joint::getReactionForce() has been added to retrieve the current reaction force of a joint
- Method Joint::getReactionTorque() has been added to retrieve the current reaction torque of a joint
- Method RigidBody::setLinearLockAxisFactor() to lock the translational movement of a body along the world-space x, y and z axes
- Method RigidBody::setAngularLockAxisFactor() to lock the rotational movement of a body around the world-space x, y and z axes
- Method RigidBody::applyLocalForceAtWorldPosition() to manually apply a force to a rigid body
- Method RigidBody::applyLocalForceAtLocalPosition() to manually apply a force to a rigid body
- Method RigidBody::applyLocalForceToCenterOfMass() to manually apply a force to a rigid body
- Method RigidBody::applyLocalTorque() to apply a local-space torque to a rigid body
- Method RigidBody::getForce() to get the total manually applied force on a rigid body
- Method RigidBody::getTorque() to get the total manually applied torque on a rigid body
- Method RigidBody::setIsSleeping() is now public in order to wake up or put to sleep a rigid body
- A cone limit can now be set to the ball-and-socket joint (this is useful for ragdolls)
- New scenes have been added to the testbed application (Box Tower, Ragdoll, Rope, Ball And Socket Joint, Bridge, Hinge Joint, Hinge Joint chain, Ball and
Socket Joint chain, Ball and Socket Joint net, ...)
- It is now possible to move bodies using the mouse (CTRL + click and drag) in the testbed application

### Removed

- Method Material::getRollingResistance() has been removed (angular damping has to be used instead of rolling resistance)
- Method Material::setRollingResistance() has been removed (angular damping has to be used instead of rolling resistance)

### Fixed

- Issue [#165](https://github.com/DanielChappuis/reactphysics3d/issues/165) with order of contact manifolds in islands creation has been fixed
- Issue [#179](https://github.com/DanielChappuis/reactphysics3d/issues/179) with FixedJoint constraint 
- Issue [#195](https://github.com/DanielChappuis/reactphysics3d/issues/195) in RigidBodyComponents
- Issue with concave vs convex shape collision detection has been fixed
- Issue with edge vs edge collision has been fixed in SAT algorithm (wrong contact normal was computed)
- Issue with sphere radius in DebugRenderer
- Issue where changing the transform of a Collider attached to a sleeping RigidBody caused the body to remain asleep
- Issue with wrong calculation performed in the ContactSolverSystem
- Issue with joints when center of mass is not at the center of the rigid body local-space
- Issue [#157](https://github.com/DanielChappuis/reactphysics3d/issues/157) with matrix to quaternion conversion has been fixed
- Issue [#184](https://github.com/DanielChappuis/reactphysics3d/issues/184) with update of mass/inertia properties of static bodies
- Issue with the computation of the two friction vectors in the contact solver
- Issue with the rendering of the capsule collision shape in the Debug Renderer (missing triangle faces)
- Issue with wrong linear velocity update computed in RigidBody::setLocalCenterOfMass() method
- Issue with wrong linear velocity update computed in RigidBody::updateLocalCenterOfMassFromColliders() method
- Issue with wrong linear velocity update computed in RigidBody::updateMassPropertiesFromColliders() method
- Issue in copy-constructors in Map and Set classes
- A lot of code warnings have been fixed [#221](https://github.com/DanielChappuis/reactphysics3d/issues/221), [#222](https://github.com/DanielChappuis/reactphysics3d/issues/222), [#223](https://github.com/DanielChappuis/reactphysics3d/issues/223) and [#224](https://github.com/DanielChappuis/reactphysics3d/issues/224)
- The default warning level is not set anymore in CMakeLists.txt file (Issue [#220](https://github.com/DanielChappuis/reactphysics3d/issues/220)) 
- Issue [#225](https://github.com/DanielChappuis/reactphysics3d/issues/225) with collision not working when setting a body to be static before calling updateMassPropertiesFromColliders() 

## [0.8.0] - 2020-05-31

Note that this release contains some public API changes. Please read carefully the following changes before upgrading to this new version and
do not hesitate to take a look at the user manual.

### Changed

- The CollisionWorld::testCollision() methods do not have the 'categoryMaskBits' parameter anymore
- The CollisionWorld::testOverlap() methods do not have the 'categoryMaskBits' parameter anymore
- Many methods in the EventListener class have changed. Check the user manual for more information
- The way to retrieve contacts has changed. Check the user manual for more information
- DynamicsWorld and CollisionWorld classes have been merged into a single class called PhysicsWorld
- The ProxyShape class has been renamed into Collider
- The Material is now part of the Collider instead of the RigidBody. Therefore, it is now possible to have a RigidBody with multiple
colliders and a different material for each Collider
- The Logger has to be set using the PhysicsCommon::setLogger() method
- The Box::getExtent() method has been renamed to Box::getHalfExtents()
- An instance of the BoxShape class cannot be instanciated directly anymore. You need to use the PhysicsCommon::createBoxShape() method
- An instance of the SphereShape class cannot be instanciated directly anymore. You need to use the PhysicsCommon::createSphereShape() method
- An instance of the CapsuleShape class cannot be instanciated directly anymore. You need to use the PhysicsCommon::createCapsuleShape() method
- An instance of the ConvexMeshShape class cannot be instanciated directly anymore. You need to use the PhysicsCommon::createConvexMeshShape() method
- An instance of the HeightFieldShape class cannot be instanciated directly anymore. You need to use the PhysicsCommon::createHeightFieldShape() method
- An instance of the ConcaveMeshShape class cannot be instanciated directly anymore. You need to use the PhysicsCommon::createConcaveMeshShape() method
- An instance of the PolyhedronMesh class cannot be instanciated directly anymore. You need to use the PhysicsCommon::createPolyhedronMesh() method
- An instance of the TriangleMesh class cannot be instanciated directly anymore. You need to use the PhysicsCommon::createTriangleMesh() method
- The ProxyShape class has been renamed to Collider. The CollisionBody::addCollider(), RigidBody::addCollider() methods have to be used to create and add a collider to a body. Then methods CollisionBody::removeCollider(), RigidBody::removeCollider() need to be used to remove a collider from a body.
- The RigidBody::addCollider() method (previously addProxyShape() method) does not take a "mass" parameter anymore
- The RigidBody::setCenterOfMassLocal() method has been renamed to RigidBody::setLocalCenterOfMass()
- The RigidBody::setInertiaTensorLocal() method has been renamed to RigidBody::setLocalInertiaTensor()
- Now, the local inertia tensor of a rigid body has to be set using a Vector3 instead of a Matrix3x3. You only need to provide the three diagonal values of the matrix
- The RigidBody::recomputeMassInformation() method has been renamed to RigidBody::updateMassPropertiesFromColliders.
- Now, you need to manually call the RigidBody::updateMassPropertiesFromColliders() method after adding colliders to a rigid body to recompute its inertia tensor, center of mass and mass. There are other methods that you can use form that (see the user manual)
- The RigidBody::applyForce() method has been renamed to RigidBody::applyForceAtWorldPosition()
- The linear and angular damping function of the rigid bodies has been changed
- The rendering in the testbed application has been improved
- Many of the data inside the library have been refactored for better caching and easier parallelization in the future
- The old Logger class has been renamed to DefaultLogger
- The Logger class is now an abstract class that you can inherit from in order to receive log events from the library
- User manual and API documentation have been updated

### Added

- It is now possible to change the size of a BoxShape using the BoxShape::setHalfExtents() method
- It is now possible to change the radius of a SphereShape using the SphereShape::setRadius() method
- It is now possible to change the height and radius of a CapsuleShape using the CapsuleShape::setHeight() and CapsuleShape::setRadius() methods
- It is now possible to change the scale of a ConvexMeshShape using the ConvexMeshShape::setScale() method
- It is now possible to change the scale of a ConcaveMeshShape using the ConcaveMeshShape::setScale() method
- It is now possible to change the scale of a HeightFieldShape using the HeightFieldShape::setScale() method
- A method PhysicsWorld::getCollisionBody(uint index) has been added on a physics world to retrieve a given CollisionBody
- A method PhysicsWorld::getRigidBody(uint index) has been added on a physics world to retrieve a given RigidBody
- A RigidBody::getLocalCenterOfMass() method has been added to retrieve the current local-space center of mass of a rigid body
- Add PhysicsCommon class that needs to be instanciated at the beginning and is used as a factory for other objects of the library (see the user manual)
- The RigidBody::updateLocalCenterOfMassFromColliders() method has been added to compute and set the center of mass of a body using its colliders
- The RigidBody::updateLocalInertiaTensorFromColliders() method has been added to compute and set the local inertia tensor of a body using its colliders
- The RigidBody::getLocalInertiaTensor() method has been added to retrieve the local-space inertia tensor of a rigid body
- The RigidBody::updateMassFromColliders() method has been added to compute and set the mass of a body using its colliders
- A Material nows has a mass density parameter that can be set using the Material::setMassDensity() method. The mass density is used to compute the mass of a collider when computing the mass of a rigid body
- A Collider can now be a trigger. This collider will be used to only report collisions with another collider but no collision response will be applied. You can use the Collider::setIsTrigger() method for this.
- The EventListener class now has a onTrigger() method that is called when a trigger collider is colling with another collider
- In the EventListener, the onContact() and onTrigger() method now reports the type of event (start, stay, exit) for each contact. This way the user can know whether it's a new contact or not or when two colliders are not in contact anymore 
- A DebugRenderer class has been added in order to display debug info (colliders, AABBs, contacts, ...) in your simulation using graphics primitives (lines, triangles).
- A RigidBody::applyForceAtLocalPosition() method has been added to apply a force at a given position of the rigid body in local-space
- A default logger can be instanciated using the PhysicsCommon::createDefaultLogger() method
- The CMakeLists.txt file of the library has been refactored to use modern CMake. The targets are now exported when you install the library so that you can import the library with the find_package(ReactPhysics3D) function in your own CMakeLists.txt file
- A Hello World project has been added to show a very simple project that shows how to compile and use the ReactPhysics3D library

### Removed

- The method DynamicsWorld::getContactsList() has been removed. You need to use the EventListener class to retrieve contacts now (see the user manual).
- The DynamicsWorld::getNbJoints() method has been removed.
- The EventListener::beginInternalTick() method has been removed (because internal ticks do not exist anymore).
- The EventListener::endInternalTick() method has been removed (because internal ticks do not exist anymore).
- The RigidBody::getJointsList() method has been removed.
- It is not possible anymore to set custom pool and stack frame allocators. Only the base allocator can be customized when creating a PhysicsCommon instance.
- The RigidBody::setInverseInertiaTensorLocal() method has been removed. The RigidBody::setInertiaTensorLocal() has to be used instead.
- The RigidBody::getInverseInertiaTensorWorld() method has been removed.
- The Collider::getMass() method has been removed.

### Fixed

- Issues [#125](https://github.com/DanielChappuis/reactphysics3d/issues/125) and [#106](https://github.com/DanielChappuis/reactphysics3d/issues/106) with CMake install of the library have been fixed
- Issue [#141](https://github.com/DanielChappuis/reactphysics3d/issues/141) with limits of hinge and slider joints has been fixed
- Issue [#117](https://github.com/DanielChappuis/reactphysics3d/issues/117) in documentation has been fixed
- Issue [#131](https://github.com/DanielChappuis/reactphysics3d/issues/131) in documentation has been fixed
- Issue [#139](https://github.com/DanielChappuis/reactphysics3d/issues/139) in API documentation has been fixed
- Issue [#122](https://github.com/DanielChappuis/reactphysics3d/issues/122) in logger has been fixed

## [0.7.1] - 2019-07-01

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

## [0.7.0] - 2018-05-01

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

## [0.6.0] - 2016-04-15

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

## [0.5.0] - 2015-03-04

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

## [0.4.0] - 2013-10-07

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

## [0.3.0]  - 2013-03-20

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

## [0.2.0] - 2013-01-01

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

[0.10.1]: https://github.com/DanielChappuis/reactphysics3d/compare/v0.10.0...v0.10.1
[0.10.0]: https://github.com/DanielChappuis/reactphysics3d/compare/v0.9.0...v0.10.0
[0.9.0]: https://github.com/DanielChappuis/reactphysics3d/compare/v0.8.0...v0.9.0
[0.8.0]: https://github.com/DanielChappuis/reactphysics3d/compare/v0.7.1...v0.8.0
[0.7.1]: https://github.com/DanielChappuis/reactphysics3d/compare/v0.7.0...v0.7.1
[0.7.0]: https://github.com/DanielChappuis/reactphysics3d/compare/v0.6.0...v0.7.0
[0.6.0]: https://github.com/DanielChappuis/reactphysics3d/compare/v0.5.0...v0.6.0
[0.5.0]: https://github.com/DanielChappuis/reactphysics3d/compare/v0.4.0...v0.5.0
[0.4.0]: https://github.com/DanielChappuis/reactphysics3d/compare/v0.3.0...v0.4.0
[0.3.0]: https://github.com/DanielChappuis/reactphysics3d/releases/tag/v0.3.0
