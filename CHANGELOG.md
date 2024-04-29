## [0.10.1] - 2024-04-22
### Added
- Glob pattern support
- Unit Tests
- Log version

### Fixed
- Exception on margins larger than context of changelog
- Nil pointer exception in 'release' package

### Changed
- Refactor JavaScript wrapper

## [3.3.0] - 2020-06-27
### Added
- Wrapper script: allow execution on Windows runners

### Changed
- Action execution through Git: from Docker to NodeJS

## [0.10.0] - 2024-03-10
### Added

 - The library will now return errors found in input data during the creation of ConvexMesh, TriangularMesh and HeighField
 - It is now possible to create a ConvexMeshShape by specifying only a list of vertices (automatic computation of convex hull using internal
   QuickHull algorithm)
 - The performance of static bodies has been improved 
 - The reporting of contact state is now correct even if the body goes to sleep
 - The DebugRenderer can now display the normals of the collider faces for debugging purpose
 - It is now possible to select for which bodies the debug information from the DebugRenderer is displayed

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
[0.10.1]: https://github.com/DanielChappuis/reactphysics3d/compare/v0.10.0...v0.10.1
[0.10.0]: https://github.com/DanielChappuis/reactphysics3d/compare/v0.9.0...v0.10.0
