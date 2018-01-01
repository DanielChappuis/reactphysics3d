/********************************************************************************
* ReactPhysics3D physics library, http://www.reactphysics3d.com                 *
* Copyright (c) 2010-2016 Daniel Chappuis                                       *
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
#include <complex>
#include "configuration.h"
#include "ConvexMeshShape.h"

using namespace reactphysics3d;

// TODO : Check in every collision shape that localScalling is used correctly and even with SAT
//        algorithm (not only in getLocalSupportPoint***() methods)

// Constructor to initialize with an array of 3D vertices.
/// This method creates an internal copy of the input vertices.
/**
 * @param arrayVertices Array with the vertices of the convex mesh
 * @param nbVertices Number of vertices in the convex mesh
 * @param stride Stride between the beginning of two elements in the vertices array
 * @param margin Collision margin (in meters) around the collision shape
 */
ConvexMeshShape::ConvexMeshShape(PolyhedronMesh* polyhedronMesh)
                : ConvexPolyhedronShape(CollisionShapeName::CONVEX_MESH), mPolyhedronMesh(polyhedronMesh), mMinBounds(0, 0, 0), mMaxBounds(0, 0, 0) {

    // Recalculate the bounds of the mesh
    recalculateBounds();
}

// Return a local support point in a given direction without the object margin.
/// If the edges information is not used for collision detection, this method will go through
/// the whole vertices list and pick up the vertex with the largest dot product in the support
/// direction. This is an O(n) process with "n" being the number of vertices in the mesh.
/// However, if the edges information is used, we can cache the previous support vertex and use
/// it as a start in a hill-climbing (local search) process to find the new support vertex which
/// will be in most of the cases very close to the previous one. Using hill-climbing, this method
/// runs in almost constant time.
Vector3 ConvexMeshShape::getLocalSupportPointWithoutMargin(const Vector3& direction) const {

    double maxDotProduct = DECIMAL_SMALLEST;
    uint indexMaxDotProduct = 0;

    // For each vertex of the mesh
    for (uint i=0; i<mPolyhedronMesh->getNbVertices(); i++) {

        // Compute the dot product of the current vertex
        double dotProduct = direction.dot(mPolyhedronMesh->getVertex(i));

        // If the current dot product is larger than the maximum one
        if (dotProduct > maxDotProduct) {
            indexMaxDotProduct = i;
            maxDotProduct = dotProduct;
        }
    }

    assert(maxDotProduct >= decimal(0.0));

    // Return the vertex with the largest dot product in the support direction
    return mPolyhedronMesh->getVertex(indexMaxDotProduct) * mScaling;
}

// Recompute the bounds of the mesh
void ConvexMeshShape::recalculateBounds() {

    // TODO : Only works if the local origin is inside the mesh
    //        => Make it more robust (init with first vertex of mesh instead)

    mMinBounds.setToZero();
    mMaxBounds.setToZero();

    // For each vertex of the mesh
    for (uint i=0; i<mPolyhedronMesh->getNbVertices(); i++) {

        if (mPolyhedronMesh->getVertex(i).x > mMaxBounds.x) mMaxBounds.x = mPolyhedronMesh->getVertex(i).x;
        if (mPolyhedronMesh->getVertex(i).x < mMinBounds.x) mMinBounds.x = mPolyhedronMesh->getVertex(i).x;

        if (mPolyhedronMesh->getVertex(i).y > mMaxBounds.y) mMaxBounds.y = mPolyhedronMesh->getVertex(i).y;
        if (mPolyhedronMesh->getVertex(i).y < mMinBounds.y) mMinBounds.y = mPolyhedronMesh->getVertex(i).y;

        if (mPolyhedronMesh->getVertex(i).z > mMaxBounds.z) mMaxBounds.z = mPolyhedronMesh->getVertex(i).z;
        if (mPolyhedronMesh->getVertex(i).z < mMinBounds.z) mMinBounds.z = mPolyhedronMesh->getVertex(i).z;
    }

    // Apply the local scaling factor
    mMaxBounds = mMaxBounds * mScaling;
    mMinBounds = mMinBounds * mScaling;

    // Add the object margin to the bounds
    mMaxBounds += Vector3(mMargin, mMargin, mMargin);
    mMinBounds -= Vector3(mMargin, mMargin, mMargin);
}

// Raycast method with feedback information
bool ConvexMeshShape::raycast(const Ray& ray, RaycastInfo& raycastInfo, ProxyShape* proxyShape, MemoryAllocator& allocator) const {
    return proxyShape->mBody->mWorld.mCollisionDetection.mNarrowPhaseGJKAlgorithm.raycast(
                                     ray, proxyShape, raycastInfo);
}
