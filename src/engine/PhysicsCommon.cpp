/********************************************************************************
* ReactPhysics3D physics library, http://www.reactphysics3d.com                 *
* Copyright (c) 2010-2019 Daniel Chappuis                                       *
*********************************************************************************
*                                                                               *
* This software is provided 'as-is', without any express or implied warranty.   *
* In no event will the authors be held liable for any damages arising from the  *
* use of this software.                                                         *
*                                                                               *
* Permission is granted to anyone to use this software for any purpose,         *
* including commercial applications, and to alter it and redistribute it        *
* freely, subject to the following restrictions:                                *
*                                                                               *
* 1. The origin of this software must not be misrepresented; you must not claim *
*    that you wrote the original software. If you use this software in a        *
*    product, an acknowledgment in the product documentation would be           *
*    appreciated but is not required.                                           *
*                                                                               *
* 2. Altered source versions must be plainly marked as such, and must not be    *
*    misrepresented as being the original software.                             *
*                                                                               *
* 3. This notice may not be removed or altered from any source distribution.    *
*                                                                               *
********************************************************************************/

// Libraries
#include <reactphysics3d/engine/PhysicsCommon.h>

using namespace reactphysics3d;

/// Constructor
PhysicsCommon::PhysicsCommon(MemoryAllocator* baseMemoryAllocator)
              : mMemoryManager(baseMemoryAllocator),
                mPhysicsWorlds(mMemoryManager.getHeapAllocator()), mSphereShapes(mMemoryManager.getHeapAllocator()),
                mBoxShapes(mMemoryManager.getHeapAllocator()), mCapsuleShapes(mMemoryManager.getHeapAllocator()),
                mConvexMeshShapes(mMemoryManager.getHeapAllocator()), mConcaveMeshShapes(mMemoryManager.getHeapAllocator()),
                mHeightFieldShapes(mMemoryManager.getHeapAllocator()), mPolyhedronMeshes(mMemoryManager.getHeapAllocator()),
                mTriangleMeshes(mMemoryManager.getHeapAllocator()),
#ifdef IS_LOGGING_ACTIVE
                mLoggers(mMemoryManager.getHeapAllocator()),
#endif
                mProfilers(mMemoryManager.getHeapAllocator()) {

}

// Destructor
PhysicsCommon::~PhysicsCommon() {

    // Release the allocated memory
    release();
}

// Destroy and release everything that has been allocated
void PhysicsCommon::release() {

    // Destroy the physics worlds
    for (auto it = mPhysicsWorlds.begin(); it != mPhysicsWorlds.end(); ++it) {
        destroyPhysicsWorld(*it);
    }

    // Destroy the sphere shapes
    for (auto it = mSphereShapes.begin(); it != mSphereShapes.end(); ++it) {
        destroySphereShape(*it);
    }

    // Destroy the box shapes
    for (auto it = mBoxShapes.begin(); it != mBoxShapes.end(); ++it) {
        destroyBoxShape(*it);
    }

    // Destroy the capsule shapes
    for (auto it = mCapsuleShapes.begin(); it != mCapsuleShapes.end(); ++it) {
        destroyCapsuleShape(*it);
    }

    // Destroy the convex mesh shapes
    for (auto it = mConvexMeshShapes.begin(); it != mConvexMeshShapes.end(); ++it) {
        destroyConvexMeshShape(*it);
    }

    // Destroy the heigh-field shapes
    for (auto it = mHeightFieldShapes.begin(); it != mHeightFieldShapes.end(); ++it) {
        destroyHeightFieldShape(*it);
    }

    // Destroy the concave mesh shapes
    for (auto it = mConcaveMeshShapes.begin(); it != mConcaveMeshShapes.end(); ++it) {
        destroyConcaveMeshShape(*it);
    }

    // Destroy the polyhedron mesh
    for (auto it = mPolyhedronMeshes.begin(); it != mPolyhedronMeshes.end(); ++it) {
        destroyPolyhedronMesh(*it);
    }

    // Destroy the triangle mesh
    for (auto it = mTriangleMeshes.begin(); it != mTriangleMeshes.end(); ++it) {
        destroyTriangleMesh(*it);
    }

// If logging is enabled
#ifdef IS_LOGGING_ACTIVE

    // Destroy the loggers
    for (auto it = mLoggers.begin(); it != mLoggers.end(); ++it) {
        destroyDefaultLogger(*it);
    }

#endif

// If profiling is enabled
#ifdef IS_PROFILING_ACTIVE

    // Destroy the profilers
    for (auto it = mProfilers.begin(); it != mProfilers.end(); ++it) {
        destroyProfiler(*it);
    }

#endif

}

// Create and return an instance of PhysicsWorld
PhysicsWorld* PhysicsCommon::createPhysicsWorld(const PhysicsWorld::WorldSettings& worldSettings, Logger* logger, Profiler* profiler) {

#ifdef IS_PROFILING_ACTIVE

    // If the user has not provided its own profiler, we create one
    if (profiler == nullptr) {

        profiler = createProfiler();

        // Add a destination file for the profiling data
        profiler->addFileDestination("rp3d_profiling_" + worldSettings.worldName + ".txt", Profiler::Format::Text);
    }

#endif

#ifdef IS_LOGGING_ACTIVE

    // If the user has not provided its own logger, we create one
    if (logger == nullptr) {

       DefaultLogger* defaultLogger = createDefaultLogger();

        // Add a log destination file
        uint logLevel = static_cast<uint>(Logger::Level::Information) | static_cast<uint>(Logger::Level::Warning) |
                static_cast<uint>(Logger::Level::Error);
        defaultLogger->addFileDestination("rp3d_log_" + worldSettings.worldName + ".html", logLevel, DefaultLogger::Format::HTML);

        logger = defaultLogger;
    }

#endif

    PhysicsWorld* world = new(mMemoryManager.allocate(MemoryManager::AllocationType::Heap, sizeof(PhysicsWorld))) PhysicsWorld(mMemoryManager, worldSettings, logger, profiler);

    mPhysicsWorlds.add(world);

    return world;
}

// Destroy an instance of PhysicsWorld
void PhysicsCommon::destroyPhysicsWorld(PhysicsWorld* world) {

   // Call the destructor of the world
   world->~PhysicsWorld();

   // Release allocated memory
   mMemoryManager.release(MemoryManager::AllocationType::Heap, world, sizeof(PhysicsWorld));

   mPhysicsWorlds.remove(world);
}

// Create and return a sphere collision shape
SphereShape* PhysicsCommon::createSphereShape(const decimal radius) {

    SphereShape* shape = new (mMemoryManager.allocate(MemoryManager::AllocationType::Pool, sizeof(SphereShape))) SphereShape(radius, mMemoryManager.getHeapAllocator());
    mSphereShapes.add(shape);

    return shape;
}

// Destroy a sphere collision shape
void PhysicsCommon::destroySphereShape(SphereShape* sphereShape) {

    // TODO Test if collision shape is still part of some colliders, if so throw error

    assert(sphereShape->mColliders.size() == 0);

   // Call the destructor of the shape
   sphereShape->~SphereShape();

   // Release allocated memory
   mMemoryManager.release(MemoryManager::AllocationType::Pool, sphereShape, sizeof(SphereShape));

   mSphereShapes.remove(sphereShape);
}

// Create and return a box collision shape
BoxShape* PhysicsCommon::createBoxShape(const Vector3& extent) {

    BoxShape* shape = new (mMemoryManager.allocate(MemoryManager::AllocationType::Pool, sizeof(BoxShape))) BoxShape(extent, mMemoryManager.getHeapAllocator());

    mBoxShapes.add(shape);

    return shape;
}

// Destroy a box collision shape
void PhysicsCommon::destroyBoxShape(BoxShape* boxShape) {

    // TODO Test if collision shape is still part of some colliders, if so throw error

    assert(boxShape->mColliders.size() == 0);

   // Call the destructor of the shape
   boxShape->~BoxShape();

   // Release allocated memory
   mMemoryManager.release(MemoryManager::AllocationType::Pool, boxShape, sizeof(BoxShape));

   mBoxShapes.remove(boxShape);
}

// Create and return a capsule shape
CapsuleShape* PhysicsCommon::createCapsuleShape(decimal radius, decimal height) {

    CapsuleShape* shape = new (mMemoryManager.allocate(MemoryManager::AllocationType::Pool, sizeof(CapsuleShape))) CapsuleShape(radius, height, mMemoryManager.getHeapAllocator());

    mCapsuleShapes.add(shape);

    return shape;
}

// Destroy a capsule collision shape
void PhysicsCommon::destroyCapsuleShape(CapsuleShape* capsuleShape) {

    // TODO Test if collision shape is still part of some colliders, if so throw error

    assert(capsuleShape->mColliders.size() == 0);

   // Call the destructor of the shape
   capsuleShape->~CapsuleShape();

   // Release allocated memory
   mMemoryManager.release(MemoryManager::AllocationType::Pool, capsuleShape, sizeof(CapsuleShape));

   mCapsuleShapes.remove(capsuleShape);
}

// Create and return a convex mesh shape
ConvexMeshShape* PhysicsCommon::createConvexMeshShape(PolyhedronMesh* polyhedronMesh, const Vector3& scaling) {

    ConvexMeshShape* shape = new (mMemoryManager.allocate(MemoryManager::AllocationType::Pool, sizeof(ConvexMeshShape))) ConvexMeshShape(polyhedronMesh, mMemoryManager.getHeapAllocator(), scaling);

    mConvexMeshShapes.add(shape);

    return shape;
}

// Destroy a convex mesh shape
void PhysicsCommon::destroyConvexMeshShape(ConvexMeshShape* convexMeshShape) {

    // TODO Test if collision shape is still part of some colliders, if so throw error

    assert(convexMeshShape->mColliders.size() == 0);

   // Call the destructor of the shape
   convexMeshShape->~ConvexMeshShape();

   // Release allocated memory
   mMemoryManager.release(MemoryManager::AllocationType::Pool, convexMeshShape, sizeof(ConvexMeshShape));

   mConvexMeshShapes.remove(convexMeshShape);
}

// Create and return a height-field shape
HeightFieldShape* PhysicsCommon::createHeightFieldShape(int nbGridColumns, int nbGridRows, decimal minHeight, decimal maxHeight,
                                         const void* heightFieldData, HeightFieldShape::HeightDataType dataType,
                                         int upAxis, decimal integerHeightScale, const Vector3& scaling) {

    HeightFieldShape* shape = new (mMemoryManager.allocate(MemoryManager::AllocationType::Pool, sizeof(HeightFieldShape))) HeightFieldShape(nbGridColumns, nbGridRows, minHeight, maxHeight,
                                         heightFieldData, dataType, mMemoryManager.getHeapAllocator(), upAxis, integerHeightScale, scaling);

    mHeightFieldShapes.add(shape);

    return shape;
}

// Destroy a height-field shape
void PhysicsCommon::destroyHeightFieldShape(HeightFieldShape* heightFieldShape) {

   // TODO Test if collision shape is still part of some colliders, if so throw error

   assert(heightFieldShape->mColliders.size() == 0);

   // Call the destructor of the shape
   heightFieldShape->~HeightFieldShape();

   // Release allocated memory
   mMemoryManager.release(MemoryManager::AllocationType::Pool, heightFieldShape, sizeof(HeightFieldShape));

   mHeightFieldShapes.remove(heightFieldShape);
}

// Create and return a concave mesh shape
ConcaveMeshShape* PhysicsCommon::createConcaveMeshShape(TriangleMesh* triangleMesh, const Vector3& scaling) {

    ConcaveMeshShape* shape = new (mMemoryManager.allocate(MemoryManager::AllocationType::Pool, sizeof(ConcaveMeshShape))) ConcaveMeshShape(triangleMesh, mMemoryManager.getHeapAllocator(), scaling);

    mConcaveMeshShapes.add(shape);

    return shape;
}

// Destroy a concave mesh shape
void PhysicsCommon::destroyConcaveMeshShape(ConcaveMeshShape* concaveMeshShape) {

   // TODO Test if collision shape is still part of some colliders, if so throw error

   assert(concaveMeshShape->mColliders.size() == 0);

   // Call the destructor of the shape
   concaveMeshShape->~ConcaveMeshShape();

   // Release allocated memory
   mMemoryManager.release(MemoryManager::AllocationType::Pool, concaveMeshShape, sizeof(ConcaveMeshShape));

   mConcaveMeshShapes.remove(concaveMeshShape);
}

// Create a polyhedron mesh
PolyhedronMesh* PhysicsCommon::createPolyhedronMesh(PolygonVertexArray* polygonVertexArray) {

    PolyhedronMesh* mesh = new (mMemoryManager.allocate(MemoryManager::AllocationType::Pool, sizeof(PolyhedronMesh))) PolyhedronMesh(polygonVertexArray, mMemoryManager.getHeapAllocator());

    mPolyhedronMeshes.add(mesh);

    return mesh;
}

// Destroy a polyhedron mesh
void PhysicsCommon::destroyPolyhedronMesh(PolyhedronMesh* polyhedronMesh) {

   // Call the destructor of the shape
   polyhedronMesh->~PolyhedronMesh();

   // Release allocated memory
   mMemoryManager.release(MemoryManager::AllocationType::Pool, polyhedronMesh, sizeof(PolyhedronMesh));

   mPolyhedronMeshes.remove(polyhedronMesh);
}

// Create a triangle mesh
TriangleMesh* PhysicsCommon::createTriangleMesh() {

    TriangleMesh* mesh = new (mMemoryManager.allocate(MemoryManager::AllocationType::Pool, sizeof(TriangleMesh))) TriangleMesh(mMemoryManager.getHeapAllocator());

    mTriangleMeshes.add(mesh);

    return mesh;
}

// Destroy a triangle mesh
void PhysicsCommon::destroyTriangleMesh(TriangleMesh* triangleMesh) {

   // Call the destructor of the shape
   triangleMesh->~TriangleMesh();

   // Release allocated memory
   mMemoryManager.release(MemoryManager::AllocationType::Pool, triangleMesh, sizeof(TriangleMesh));

   mTriangleMeshes.remove(triangleMesh);
}

// If logging is enabled
#ifdef IS_LOGGING_ACTIVE

// Create and return a new logger
DefaultLogger* PhysicsCommon::createDefaultLogger() {

    DefaultLogger* logger = new (mMemoryManager.allocate(MemoryManager::AllocationType::Pool, sizeof(DefaultLogger))) DefaultLogger(mMemoryManager.getHeapAllocator());

    mLoggers.add(logger);

    return logger;
}

// Destroy a logger
void PhysicsCommon::destroyDefaultLogger(DefaultLogger* logger) {

   // Call the destructor of the logger
   logger->~DefaultLogger();

   // Release allocated memory
   mMemoryManager.release(MemoryManager::AllocationType::Pool, logger, sizeof(DefaultLogger));

   mLoggers.remove(logger);
}

#endif

// If profiling is enabled
#ifdef IS_PROFILING_ACTIVE

// Create and return a new profiler
/// Note that you need to use a different profiler for each PhysicsWorld.
Profiler* PhysicsCommon::createProfiler() {

    Profiler* profiler = new (mMemoryManager.allocate(MemoryManager::AllocationType::Pool, sizeof(Profiler))) Profiler();

    mProfilers.add(profiler);

    return profiler;
}

// Destroy a profiler
void PhysicsCommon::destroyProfiler(Profiler* profiler) {

   // Call the destructor of the profiler
   profiler->~Profiler();

   // Release allocated memory
   mMemoryManager.release(MemoryManager::AllocationType::Pool, profiler, sizeof(Profiler));

   mProfilers.remove(profiler);
}

#endif
