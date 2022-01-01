/********************************************************************************
* ReactPhysics3D physics library, http://www.reactphysics3d.com                 *
* Copyright (c) 2010-2022 Daniel Chappuis                                       *
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
#include <reactphysics3d/collision/shapes/ConcaveMeshShape.h>
#include <reactphysics3d/memory/MemoryManager.h>
#include <reactphysics3d/collision/RaycastInfo.h>
#include <reactphysics3d/collision/TriangleMesh.h>
#include <reactphysics3d/utils/Profiler.h>
#include <reactphysics3d/collision/TriangleVertexArray.h>

using namespace reactphysics3d;

// Constructor
ConcaveMeshShape::ConcaveMeshShape(TriangleMesh* triangleMesh, MemoryAllocator& allocator, HalfEdgeStructure& triangleHalfEdgeStructure, const Vector3& scaling)
                 : ConcaveShape(CollisionShapeName::TRIANGLE_MESH, allocator, scaling), mDynamicAABBTree(allocator), mTriangleHalfEdgeStructure(triangleHalfEdgeStructure) {

    mTriangleMesh = triangleMesh;
    mRaycastTestType = TriangleRaycastSide::FRONT;

    // Insert all the triangles into the dynamic AABB tree
    initBVHTree();
}

// Insert all the triangles into the dynamic AABB tree
void ConcaveMeshShape::initBVHTree() {

    // TODO : Try to randomly add the triangles into the tree to obtain a better tree

    // For each sub-part of the mesh
    for (uint32 subPart=0; subPart<mTriangleMesh->getNbSubparts(); subPart++) {

        // Get the triangle vertex array of the current sub-part
        TriangleVertexArray* triangleVertexArray = mTriangleMesh->getSubpart(subPart);

        // For each triangle of the concave mesh
        for (uint32 triangleIndex=0; triangleIndex<triangleVertexArray->getNbTriangles(); triangleIndex++) {

            Vector3 trianglePoints[3];

            // Get the triangle vertices
            triangleVertexArray->getTriangleVertices(triangleIndex, trianglePoints);

            // Create the AABB for the triangle
            AABB aabb = AABB::createAABBForTriangle(trianglePoints);

            // Add the AABB with the index of the triangle into the dynamic AABB tree
            mDynamicAABBTree.addObject(aabb, subPart, triangleIndex);
        }
    }
}

// Return the three vertices coordinates (in the array outTriangleVertices) of a triangle
void ConcaveMeshShape::getTriangleVertices(uint32 subPart, uint32 triangleIndex, Vector3* outTriangleVertices) const {

    // Get the triangle vertex array of the current sub-part
    TriangleVertexArray* triangleVertexArray = mTriangleMesh->getSubpart(subPart);

    // Get the vertices coordinates of the triangle
    triangleVertexArray->getTriangleVertices(triangleIndex, outTriangleVertices);

    // Apply the scaling factor to the vertices
    outTriangleVertices[0].x *= mScale.x;
    outTriangleVertices[0].y *= mScale.y;
    outTriangleVertices[0].z *= mScale.z;
    outTriangleVertices[1].x *= mScale.x;
    outTriangleVertices[1].y *= mScale.y;
    outTriangleVertices[1].z *= mScale.z;
    outTriangleVertices[2].x *= mScale.x;
    outTriangleVertices[2].y *= mScale.y;
    outTriangleVertices[2].z *= mScale.z;
}

// Return the three vertex normals (in the array outVerticesNormals) of a triangle
void ConcaveMeshShape::getTriangleVerticesNormals(uint32 subPart, uint32 triangleIndex, Vector3* outVerticesNormals) const {

    // Get the triangle vertex array of the current sub-part
    TriangleVertexArray* triangleVertexArray = mTriangleMesh->getSubpart(subPart);

    // Get the vertices normals of the triangle
    triangleVertexArray->getTriangleVerticesNormals(triangleIndex, outVerticesNormals);
}

// Return the indices of the three vertices of a given triangle in the array
void ConcaveMeshShape::getTriangleVerticesIndices(uint32 subPart, uint32 triangleIndex, uint32* outVerticesIndices) const {

    // Get the triangle vertex array of the current sub-part
    TriangleVertexArray* triangleVertexArray = mTriangleMesh->getSubpart(subPart);

    // Get the vertices normals of the triangle
    triangleVertexArray->getTriangleVerticesIndices(triangleIndex, outVerticesIndices);
}

// Return the number of sub parts contained in this mesh
uint32 ConcaveMeshShape::getNbSubparts() const
{
	return mTriangleMesh->getNbSubparts();
}
		
// Return the number of triangles in a sub part of the mesh
uint32 ConcaveMeshShape::getNbTriangles(uint32 subPart) const
{
	assert(mTriangleMesh->getSubpart(subPart));
	return mTriangleMesh->getSubpart(subPart)->getNbTriangles();
}

// Compute all the triangles of the mesh that are overlapping with the AABB in parameter
void ConcaveMeshShape::computeOverlappingTriangles(const AABB& localAABB, Array<Vector3>& triangleVertices,
                                                   Array<Vector3>& triangleVerticesNormals, Array<uint32>& shapeIds,
                                                   MemoryAllocator& allocator) const {

    RP3D_PROFILE("ConcaveMeshShape::computeOverlappingTriangles()", mProfiler);

    // Scale the input AABB with the inverse scale of the concave mesh (because
    // we store the vertices without scale inside the dynamic AABB tree
    AABB aabb(localAABB);
    aabb.applyScale(Vector3(decimal(1.0) / mScale.x, decimal(1.0) / mScale.y, decimal(1.0) / mScale.z));

    // Compute the nodes of the internal AABB tree that are overlapping with the AABB
    Array<int> overlappingNodes(allocator, 64);
    mDynamicAABBTree.reportAllShapesOverlappingWithAABB(aabb, overlappingNodes);

    const uint32 nbOverlappingNodes = static_cast<uint32>(overlappingNodes.size());

    // Add space in the array of triangles vertices/normals for the new triangles
    triangleVertices.addWithoutInit(nbOverlappingNodes * 3);
    triangleVerticesNormals.addWithoutInit(nbOverlappingNodes * 3);

    // For each overlapping node
    for (uint32 i=0; i < nbOverlappingNodes; i++) {

        int nodeId = overlappingNodes[i];

        // Get the node data (triangle index and mesh subpart index)
        int32* data = mDynamicAABBTree.getNodeDataInt(nodeId);

        // Get the triangle vertices for this node from the concave mesh shape
        getTriangleVertices(data[0], data[1], &(triangleVertices[i * 3]));

        // Get the vertices normals of the triangle
        getTriangleVerticesNormals(data[0], data[1], &(triangleVerticesNormals[i * 3]));

        // Compute the triangle shape ID
        shapeIds.add(computeTriangleShapeId(data[0], data[1]));
    }
}

// Raycast method with feedback information
/// Note that only the first triangle hit by the ray in the mesh will be returned, even if
/// the ray hits many triangles.
bool ConcaveMeshShape::raycast(const Ray& ray, RaycastInfo& raycastInfo, Collider* collider, MemoryAllocator& allocator) const {

    RP3D_PROFILE("ConcaveMeshShape::raycast()", mProfiler);

    // Apply the concave mesh inverse scale factor because the mesh is stored without scaling
    // inside the dynamic AABB tree
    const Vector3 inverseScale(decimal(1.0) / mScale.x, decimal(1.0) / mScale.y, decimal(1.0) / mScale.z);
    Ray scaledRay(ray.point1 * inverseScale, ray.point2 * inverseScale, ray.maxFraction);

    // Create the callback object that will compute ray casting against triangles
    ConcaveMeshRaycastCallback raycastCallback(mDynamicAABBTree, *this, collider, raycastInfo, scaledRay, mScale, allocator);

#ifdef IS_RP3D_PROFILING_ENABLED


	// Set the profiler
	raycastCallback.setProfiler(mProfiler);

#endif

    // Ask the Dynamic AABB Tree to report all AABB nodes that are hit by the ray.
    // The raycastCallback object will then compute ray casting against the triangles
    // in the hit AABBs.
    mDynamicAABBTree.raycast(scaledRay, raycastCallback);

    raycastCallback.raycastTriangles();

    return raycastCallback.getIsHit();
}

// Compute the shape Id for a given triangle of the mesh
uint32 ConcaveMeshShape::computeTriangleShapeId(uint32 subPart, uint32 triangleIndex) const {

    RP3D_PROFILE("ConcaveMeshShape::computeTriangleShapeId()", mProfiler);

    uint32 shapeId = 0;

    uint32 i=0;
    while (i < subPart) {

        shapeId += mTriangleMesh->getSubpart(i)->getNbTriangles();

        i++;
    }

    return shapeId + triangleIndex;
}

// Collect all the AABB nodes that are hit by the ray in the Dynamic AABB Tree
decimal ConcaveMeshRaycastCallback::raycastBroadPhaseShape(int32 nodeId, const Ray& ray) {

    // Add the id of the hit AABB node into
    mHitAABBNodes.add(nodeId);

    return ray.maxFraction;
}

// Raycast all collision shapes that have been collected
void ConcaveMeshRaycastCallback::raycastTriangles() {

    Array<int>::Iterator it;
    decimal smallestHitFraction = mRay.maxFraction;

    for (it = mHitAABBNodes.begin(); it != mHitAABBNodes.end(); ++it) {

        // Get the node data (triangle index and mesh subpart index)
        int32* data = mDynamicAABBTree.getNodeDataInt(*it);

        // Get the triangle vertices for this node from the concave mesh shape
        Vector3 trianglePoints[3];
        mConcaveMeshShape.getTriangleVertices(data[0], data[1], trianglePoints);

        // Get the vertices normals of the triangle
        Vector3 verticesNormals[3];
        mConcaveMeshShape.getTriangleVerticesNormals(data[0], data[1], verticesNormals);

        // Create a triangle collision shape
        TriangleShape triangleShape(trianglePoints, verticesNormals, mConcaveMeshShape.computeTriangleShapeId(data[0], data[1]), mConcaveMeshShape.mTriangleHalfEdgeStructure, mAllocator);
        triangleShape.setRaycastTestType(mConcaveMeshShape.getRaycastTestType());
		
#ifdef IS_RP3D_PROFILING_ENABLED


		// Set the profiler to the triangle shape
		triangleShape.setProfiler(mProfiler);

#endif

        // Ray casting test against the collision shape
        RaycastInfo raycastInfo;
        bool isTriangleHit = triangleShape.raycast(mRay, raycastInfo, mCollider, mAllocator);

        // If the ray hit the collision shape
        if (isTriangleHit && raycastInfo.hitFraction <= smallestHitFraction) {

            assert(raycastInfo.hitFraction >= decimal(0.0));

            mRaycastInfo.body = raycastInfo.body;
            mRaycastInfo.collider = raycastInfo.collider;
            mRaycastInfo.hitFraction = raycastInfo.hitFraction;
            mRaycastInfo.worldPoint = raycastInfo.worldPoint * mMeshScale;
            mRaycastInfo.worldNormal = raycastInfo.worldNormal;
            mRaycastInfo.meshSubpart = data[0];
            mRaycastInfo.triangleIndex = data[1];

            smallestHitFraction = raycastInfo.hitFraction;
            mIsHit = true;
        }
    }
}

// Return the string representation of the shape
std::string ConcaveMeshShape::to_string() const {

    std::stringstream ss;

    ss << "ConcaveMeshShape{" << std::endl;
    ss << "nbSubparts=" << mTriangleMesh->getNbSubparts() << std::endl;

    // Vertices array
    for (uint32 subPart=0; subPart<mTriangleMesh->getNbSubparts(); subPart++) {

        // Get the triangle vertex array of the current sub-part
        TriangleVertexArray* triangleVertexArray = mTriangleMesh->getSubpart(subPart);

        ss << "subpart" << subPart << "={" << std::endl;
        ss << "nbVertices=" << triangleVertexArray->getNbVertices() << std::endl;
        ss << "nbTriangles=" << triangleVertexArray->getNbTriangles() << std::endl;

        ss << "vertices=[";

        // For each triangle of the concave mesh
        for (uint32 v=0; v<triangleVertexArray->getNbVertices(); v++) {

            Vector3 vertex;
            triangleVertexArray->getVertex(v, &vertex);

            ss << vertex.to_string() << ", ";
        }

        ss << "], " << std::endl;

        ss << "normals=[";

        // For each triangle of the concave mesh
        for (uint32 v=0; v<triangleVertexArray->getNbVertices(); v++) {

            Vector3 normal;
            triangleVertexArray->getNormal(v, &normal);

            ss << normal.to_string() << ", ";
        }

        ss << "], " << std::endl;

        ss << "triangles=[";

        // For each triangle of the concave mesh
        // For each triangle of the concave mesh
        for (uint32 triangleIndex=0; triangleIndex<triangleVertexArray->getNbTriangles(); triangleIndex++) {

            uint32 indices[3];

            triangleVertexArray->getTriangleVerticesIndices(triangleIndex, indices);

            ss << "(" << indices[0] << "," << indices[1] << "," << indices[2] << "), ";
        }

        ss << "], " << std::endl;

        ss << "}" << std::endl;
    }

    return ss.str();
}
