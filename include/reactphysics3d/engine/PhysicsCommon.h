/********************************************************************************
* ReactPhysics3D physics library, http://www.reactphysics3d.com                 *
* Copyright (c) 2010-2024 Daniel Chappuis                                       *
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

#ifndef REACTPHYSICS3D_PHYSICS_COMMON_H
#define REACTPHYSICS3D_PHYSICS_COMMON_H

// Libraries
#include <reactphysics3d/memory/MemoryManager.h>
#include <reactphysics3d/engine/PhysicsWorld.h>
#include <reactphysics3d/collision/shapes/SphereShape.h>
#include <reactphysics3d/collision/shapes/BoxShape.h>
#include <reactphysics3d/collision/shapes/CapsuleShape.h>
#include <reactphysics3d/collision/shapes/HeightFieldShape.h>
#include <reactphysics3d/collision/shapes/ConvexMeshShape.h>
#include <reactphysics3d/collision/shapes/ConcaveMeshShape.h>
#include <reactphysics3d/collision/TriangleMesh.h>
#include <reactphysics3d/collision/ConvexMesh.h>
#include <reactphysics3d/collision/HeightField.h>
#include <reactphysics3d/utils/DefaultLogger.h>
#include <reactphysics3d/collision/PolygonVertexArray.h>
#include <reactphysics3d/collision/VertexArray.h>

/// ReactPhysics3D namespace
namespace reactphysics3d {

class VertexArray;

// Class PhysicsCommon
/**
 * This class is a singleton that needs to be instanciated once at the beginning.
 * Then this class is used by the user as a factory to create the physics world and
 * other objects.
 */
class PhysicsCommon {

    private :

        // -------------------- Attributes -------------------- //

        /// Memory manager
        MemoryManager mMemoryManager;

        /// Set of physics worlds
        Set<PhysicsWorld*> mPhysicsWorlds;

        /// Set of sphere shapes
        Set<SphereShape*> mSphereShapes;

        /// Set of box shapes
        Set<BoxShape*> mBoxShapes;

        /// Set of capsule shapes
        Set<CapsuleShape*> mCapsuleShapes;

        /// Set of convex mesh shapes
        Set<ConvexMeshShape*> mConvexMeshShapes;

        /// Set of concave mesh shapes
        Set<ConcaveMeshShape*> mConcaveMeshShapes;

        /// Set of height field shapes
        Set<HeightFieldShape*> mHeightFieldShapes;

        /// Set of convex meshes
        Set<ConvexMesh*> mConvexMeshes;

        /// Set of triangle meshes
        Set<TriangleMesh*> mTriangleMeshes;

        /// Set of height-fields
        Set<HeightField*> mHeightFields;

        /// Pointer to the current logger
        static Logger* mLogger;

        /// Set of profilers
        Set<Profiler*> mProfilers;

        /// Set of default loggers
        Set<DefaultLogger*> mDefaultLoggers;

        /// Half-edge structure of a box polyhedron
        HalfEdgeStructure mBoxShapeHalfEdgeStructure;

        /// Half-edge structure of a triangle shape
        HalfEdgeStructure mTriangleShapeHalfEdgeStructure;

        // -------------------- Methods -------------------- //

        /// Initialization
        void init();

        /// Destroy and release everything that has been allocated
        void release();

        /// Delete an instance of PhysicsWorld
        void deletePhysicsWorld(PhysicsWorld* world);

        /// Delete a sphere collision shape
        void deleteSphereShape(SphereShape* sphereShape);

        /// Delete a box collision shape
        void deleteBoxShape(BoxShape* boxShape);

        /// Delete a capsule collision shape
        void deleteCapsuleShape(CapsuleShape* capsuleShape);

        /// Delete a convex mesh shape
        void deleteConvexMeshShape(ConvexMeshShape* convexMeshShape);

        /// Delete a height-field shape
        void deleteHeightFieldShape(HeightFieldShape* heightFieldShape);

        /// Delete a concave mesh shape
        void deleteConcaveMeshShape(ConcaveMeshShape* concaveMeshShape);

        /// Delete a convex mesh
        void deleteConvexMesh(ConvexMesh* convexMesh);

        /// Delete a triangle mesh
        void deleteTriangleMesh(TriangleMesh* triangleMesh);

        /// Delete a height-field
        void deleteHeightField(HeightField* heightField);

        /// Delete a default logger
        void deleteDefaultLogger(DefaultLogger* logger);

        /// Initialize the half-edge structure of a BoxShape
        void initBoxShapeHalfEdgeStructure();

        /// Initialize the static half-edge structure of a TriangleShape
        void initTriangleShapeHalfEdgeStructure();

// If profiling is enabled
#ifdef IS_RP3D_PROFILING_ENABLED

        /// Create and return a new profiler
        Profiler* createProfiler();

        /// Destroy a profiler
        void destroyProfiler(Profiler* profiler);

        /// Delete a profiler
        void deleteProfiler(Profiler* profiler);

#endif

    public :

        // -------------------- Methods -------------------- //

        /// Constructor
        PhysicsCommon(MemoryAllocator* baseMemoryAllocator = nullptr);

        /// Destructor
        ~PhysicsCommon();

        /// Create and return an instance of PhysicsWorld
        PhysicsWorld* createPhysicsWorld(const PhysicsWorld::WorldSettings& worldSettings = PhysicsWorld::WorldSettings());

        /// Destroy an instance of PhysicsWorld
        void destroyPhysicsWorld(PhysicsWorld* world);

        /// Create and return a sphere collision shape
        SphereShape* createSphereShape(const decimal radius);

        /// Destroy a sphere collision shape
        void destroySphereShape(SphereShape* sphereShape);

        /// Create and return a box collision shape
        BoxShape* createBoxShape(const Vector3& extent);

        /// Destroy a box collision shape
        void destroyBoxShape(BoxShape* boxShape);

        /// Create and return a capsule shape
        CapsuleShape* createCapsuleShape(decimal radius, decimal height);

        /// Destroy a capsule collision shape
        void destroyCapsuleShape(CapsuleShape* capsuleShape);

        /// Create and return a convex mesh shape
        ConvexMeshShape* createConvexMeshShape(ConvexMesh* convexMesh, const Vector3& scaling = Vector3(1,1,1));

        /// Destroy a convex mesh shape
        void destroyConvexMeshShape(ConvexMeshShape* convexMeshShape);

        /// Create and return a height-field
        HeightField* createHeightField(int nbGridColumns, int nbGridRows, const void* heightFieldData,
                                       HeightField::HeightDataType dataType, std::vector<Message>& messages,
                                       decimal integerHeightScale = 1.0f);

        /// Create and return a height-field shape
        HeightFieldShape* createHeightFieldShape(HeightField* heightField,
                                                 const Vector3& scaling = Vector3(1,1,1));

        /// Destroy a height-field shape
        void destroyHeightFieldShape(HeightFieldShape* heightFieldShape);

        /// Create and return a concave mesh shape
        ConcaveMeshShape* createConcaveMeshShape(TriangleMesh* triangleMesh, const Vector3& scaling = Vector3(1, 1, 1));

        /// Destroy a concave mesh shape
        void destroyConcaveMeshShape(ConcaveMeshShape* concaveMeshShape);

        /// Create a convex mesh from a PolygonVertexArray describing vertices and faces
        ConvexMesh* createConvexMesh(const PolygonVertexArray& polygonVertexArray, std::vector<Message>& messages);

        /// Create a convex mesh from an array of vertices (automatically computing the convex hull using QuickHull)
        ConvexMesh* createConvexMesh(const VertexArray& vertexArray, std::vector<Message>& messages);

        /// Destroy a convex mesh
        void destroyConvexMesh(ConvexMesh* convexMesh);

        /// Create a triangle mesh
        TriangleMesh* createTriangleMesh(const TriangleVertexArray& triangleVertexArray, std::vector<Message>& messages);

        /// Destroy a triangle mesh
        void destroyTriangleMesh(TriangleMesh* triangleMesh);

        /// Destroy a height-field
        void destroyHeightField(HeightField* heightField);

        /// Create and return a new default logger
        DefaultLogger* createDefaultLogger();

        /// Destroy a default logger
        void destroyDefaultLogger(DefaultLogger* logger);

        /// Return the current logger
        static Logger* getLogger();

        /// Set the logger
        static void setLogger(Logger* logger);


        // ---------- Friendship ---------- //

        friend class BoxShape;
        friend class TriangleShape;
        friend class PhysicsWorld;
};

// Return the current logger
/**
 * @return A pointer to the current logger
 */
RP3D_FORCE_INLINE Logger* PhysicsCommon::getLogger() {
    return mLogger;
}

// Set the logger
/**
 * @param logger A pointer to the logger to use
 */
RP3D_FORCE_INLINE void PhysicsCommon::setLogger(Logger* logger) {
    mLogger = logger;
}

// Use this macro to log something
#define RP3D_LOG(physicsWorldName, level, category, message, filename, lineNumber) if (reactphysics3d::PhysicsCommon::getLogger() != nullptr) PhysicsCommon::getLogger()->log(level, physicsWorldName, category, message, filename, lineNumber)

}

#endif
