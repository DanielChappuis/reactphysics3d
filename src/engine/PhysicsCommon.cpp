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
#include "PhysicsCommon.h"

using namespace reactphysics3d;

/// Constructor
PhysicsCommon::PhysicsCommon(MemoryAllocator* baseMemoryAllocator)
              : mMemoryManager(baseMemoryAllocator), mCollisionWorlds(mMemoryManager.getHeapAllocator()),
                mDynamicsWorlds(mMemoryManager.getHeapAllocator()), mSphereShapes(mMemoryManager.getHeapAllocator()),
                mBoxShapes(mMemoryManager.getHeapAllocator()), mCapsuleShapes(mMemoryManager.getHeapAllocator()),
                mConvexMeshShapes(mMemoryManager.getHeapAllocator()), mConcaveMeshShapes(mMemoryManager.getHeapAllocator()),
                mHeightFieldShapes(mMemoryManager.getHeapAllocator()), mPolyhedronMeshes(mMemoryManager.getHeapAllocator()),
                mTriangleMeshes(mMemoryManager.getHeapAllocator()) {

}

// Destructor
PhysicsCommon::~PhysicsCommon() {

    // Release the allocated memory
    release();
}

// Destroy and release everything that has been allocated
void PhysicsCommon::release() {

    // Destroy the collision worlds
    for (auto it = mCollisionWorlds.begin(); it != mCollisionWorlds.end(); ++it) {
        destroyCollisionWorld(*it);
    }

    // Destroy the dynamics worlds
    for (auto it = mDynamicsWorlds.begin(); it != mDynamicsWorlds.end(); ++it) {
        destroyDynamicsWorld(*it);
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
}

// Create and return an instance of CollisionWorld
CollisionWorld* PhysicsCommon::createCollisionWorld(const WorldSettings& worldSettings, Logger* logger, Profiler* profiler) {

    CollisionWorld* world = new(mMemoryManager.allocate(MemoryManager::AllocationType::Heap, sizeof(CollisionWorld))) CollisionWorld(mMemoryManager, worldSettings, logger, profiler);
    mCollisionWorlds.add(world);

    return world;
}

// Destroy an instance of CollisionWorld
void PhysicsCommon::destroyCollisionWorld(CollisionWorld* world) {

   // Call the destructor of the world
   world->~CollisionWorld();

   // Release allocated memory
   mMemoryManager.release(MemoryManager::AllocationType::Heap, world, sizeof(CollisionWorld));

   mCollisionWorlds.remove(world);
}

// Create and return an instance of DynamicsWorld
DynamicsWorld* PhysicsCommon::createDynamicsWorld(const Vector3& gravity, const WorldSettings& worldSettings,
                                                  Logger* logger, Profiler* profiler) {

    DynamicsWorld* world = new(mMemoryManager.allocate(MemoryManager::AllocationType::Heap, sizeof(DynamicsWorld))) DynamicsWorld(gravity, mMemoryManager, worldSettings, logger, profiler);

    mDynamicsWorlds.add(world);

    return world;
}

// Destroy an instance of DynamicsWorld
DynamicsWorld* PhysicsCommon::destroyDynamicsWorld(DynamicsWorld* world) {

   // Call the destructor of the world
   world->~DynamicsWorld();

   // Release allocated memory
   mMemoryManager.release(MemoryManager::AllocationType::Heap, world, sizeof(DynamicsWorld));

   mDynamicsWorlds.remove(world);
}

// Create and return a sphere collision shape
SphereShape* PhysicsCommon::createSphereShape(const decimal radius) {

    SphereShape* shape = new (mMemoryManager.allocate(MemoryManager::AllocationType::Pool, sizeof(SphereShape))) SphereShape(radius);
    mSphereShapes.add(shape);

    return shape;
}

// Destroy a sphere collision shape
void PhysicsCommon::destroySphereShape(SphereShape* sphereShape) {

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

   // Call the destructor of the shape
   boxShape->~BoxShape();

   // Release allocated memory
   mMemoryManager.release(MemoryManager::AllocationType::Pool, boxShape, sizeof(BoxShape));

   mBoxShapes.remove(boxShape);
}

// Create and return a capsule shape
CapsuleShape* PhysicsCommon::createCapsuleShape(decimal radius, decimal height) {

    CapsuleShape* shape = new (mMemoryManager.allocate(MemoryManager::AllocationType::Pool, sizeof(CapsuleShape))) CapsuleShape(radius, height);

    mCapsuleShapes.add(shape);

    return shape;
}

// Destroy a capsule collision shape
void PhysicsCommon::destroyCapsuleShape(CapsuleShape* capsuleShape) {

   // Call the destructor of the shape
   capsuleShape->~CapsuleShape();

   // Release allocated memory
   mMemoryManager.release(MemoryManager::AllocationType::Pool, capsuleShape, sizeof(CapsuleShape));

   mCapsuleShapes.remove(capsuleShape);
}

// Create and return a convex mesh shape
ConvexMeshShape* PhysicsCommon::createConvexMeshShape(PolyhedronMesh* polyhedronMesh, const Vector3& scaling) {

    ConvexMeshShape* shape = new (mMemoryManager.allocate(MemoryManager::AllocationType::Pool, sizeof(ConvexMeshShape))) ConvexMeshShape(polyhedronMesh, scaling);

    mConvexMeshShapes.add(shape);

    return shape;
}

// Destroy a convex mesh shape
void PhysicsCommon::destroyConvexMeshShape(ConvexMeshShape* convexMeshShape) {

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
                                         heightFieldData, dataType, upAxis, integerHeightScale, scaling);

    mHeightFieldShapes.add(shape);

    return shape;
}

// Destroy a height-field shape
void PhysicsCommon::destroyHeightFieldShape(HeightFieldShape* heightFieldShape) {

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
