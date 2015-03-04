/********************************************************************************
* ReactPhysics3D physics library, http://www.reactphysics3d.com                 *
* Copyright (c) 2010-2015 Daniel Chappuis                                       *
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

// Constructor to initialize with a array of 3D vertices.
/// This method creates an internal copy of the input vertices.
/**
 * @param arrayVertices Array with the vertices of the convex mesh
 * @param nbVertices Number of vertices in the convex mesh
 * @param stride Stride between the beginning of two elements in the vertices array
 * @param margin Collision margin (in meters) around the collision shape
 */
ConvexMeshShape::ConvexMeshShape(const decimal* arrayVertices, uint nbVertices, int stride,
                                 decimal margin)
                : CollisionShape(CONVEX_MESH, margin), mNbVertices(nbVertices), mMinBounds(0, 0, 0),
                  mMaxBounds(0, 0, 0), mIsEdgesInformationUsed(false) {
    assert(nbVertices > 0);
    assert(stride > 0);

    const unsigned char* vertexPointer = (const unsigned char*) arrayVertices;

    // Copy all the vertices into the internal array
    for (uint i=0; i<mNbVertices; i++) {
        const decimal* newPoint = (const decimal*) vertexPointer;
        mVertices.push_back(Vector3(newPoint[0], newPoint[1], newPoint[2]));
        vertexPointer += stride;
    }

    // Recalculate the bounds of the mesh
    recalculateBounds();
}

// Constructor.
/// If you use this constructor, you will need to set the vertices manually one by one using
/// the addVertex() method.
ConvexMeshShape::ConvexMeshShape(decimal margin)
                : CollisionShape(CONVEX_MESH, margin), mNbVertices(0), mMinBounds(0, 0, 0),
                  mMaxBounds(0, 0, 0), mIsEdgesInformationUsed(false) {

}

// Private copy-constructor
ConvexMeshShape::ConvexMeshShape(const ConvexMeshShape& shape)
    : CollisionShape(shape), mVertices(shape.mVertices), mNbVertices(shape.mNbVertices),
      mMinBounds(shape.mMinBounds), mMaxBounds(shape.mMaxBounds),
      mIsEdgesInformationUsed(shape.mIsEdgesInformationUsed),
      mEdgesAdjacencyList(shape.mEdgesAdjacencyList) {

    assert(mNbVertices == mVertices.size());
}

// Destructor
ConvexMeshShape::~ConvexMeshShape() {

}

// Return a local support point in a given direction with the object margin
Vector3 ConvexMeshShape::getLocalSupportPointWithMargin(const Vector3& direction,
                                                        void** cachedCollisionData) const {

    // Get the support point without the margin
    Vector3 supportPoint = getLocalSupportPointWithoutMargin(direction, cachedCollisionData);

    // Get the unit direction vector
    Vector3 unitDirection = direction;
    if (direction.lengthSquare() < MACHINE_EPSILON * MACHINE_EPSILON) {
        unitDirection.setAllValues(1.0, 1.0, 1.0);
    }
    unitDirection.normalize();

    // Add the margin to the support point and return it
    return supportPoint + unitDirection * mMargin;
}

// Return a local support point in a given direction without the object margin.
/// If the edges information is not used for collision detection, this method will go through
/// the whole vertices list and pick up the vertex with the largest dot product in the support
/// direction. This is an O(n) process with "n" being the number of vertices in the mesh.
/// However, if the edges information is used, we can cache the previous support vertex and use
/// it as a start in a hill-climbing (local search) process to find the new support vertex which
/// will be in most of the cases very close to the previous one. Using hill-climbing, this method
/// runs in almost constant time.
Vector3 ConvexMeshShape::getLocalSupportPointWithoutMargin(const Vector3& direction,
                                                           void** cachedCollisionData) const {

    assert(mNbVertices == mVertices.size());
    assert(cachedCollisionData != NULL);

    // Allocate memory for the cached collision data if not allocated yet
    if ((*cachedCollisionData) == NULL) {
        *cachedCollisionData = (int*) malloc(sizeof(int));
        *((int*)(*cachedCollisionData)) = 0;
    }

    // If the edges information is used to speed up the collision detection
    if (mIsEdgesInformationUsed) {

        assert(mEdgesAdjacencyList.size() == mNbVertices);

        uint maxVertex = *((int*)(*cachedCollisionData));
        decimal maxDotProduct = direction.dot(mVertices[maxVertex]);
        bool isOptimal;

        // Perform hill-climbing (local search)
        do {
            isOptimal = true;

            assert(mEdgesAdjacencyList.at(maxVertex).size() > 0);

            // For all neighbors of the current vertex
            std::set<uint>::const_iterator it;
            std::set<uint>::const_iterator itBegin = mEdgesAdjacencyList.at(maxVertex).begin();
            std::set<uint>::const_iterator itEnd = mEdgesAdjacencyList.at(maxVertex).end();
            for (it = itBegin; it != itEnd; ++it) {

                // Compute the dot product
                decimal dotProduct = direction.dot(mVertices[*it]);

                // If the current vertex is a better vertex (larger dot product)
                if (dotProduct > maxDotProduct) {
                    maxVertex = *it;
                    maxDotProduct = dotProduct;
                    isOptimal = false;
                }
            }

        } while(!isOptimal);

        // Cache the support vertex
        *((int*)(*cachedCollisionData)) = maxVertex;

        // Return the support vertex
        return mVertices[maxVertex];
    }
    else {  // If the edges information is not used

        double maxDotProduct = DECIMAL_SMALLEST;
        uint indexMaxDotProduct = 0;

        // For each vertex of the mesh
        for (uint i=0; i<mNbVertices; i++) {

            // Compute the dot product of the current vertex
            double dotProduct = direction.dot(mVertices[i]);

            // If the current dot product is larger than the maximum one
            if (dotProduct > maxDotProduct) {
                indexMaxDotProduct = i;
                maxDotProduct = dotProduct;
            }
        }

        assert(maxDotProduct >= decimal(0.0));

        // Return the vertex with the largest dot product in the support direction
        return mVertices[indexMaxDotProduct];
    }
}

// Recompute the bounds of the mesh
void ConvexMeshShape::recalculateBounds() {

    mMinBounds.setToZero();
    mMaxBounds.setToZero();

    // For each vertex of the mesh
    for (uint i=0; i<mNbVertices; i++) {

        if (mVertices[i].x > mMaxBounds.x) mMaxBounds.x = mVertices[i].x;
        if (mVertices[i].x < mMinBounds.x) mMinBounds.x = mVertices[i].x;

        if (mVertices[i].y > mMaxBounds.y) mMaxBounds.y = mVertices[i].y;
        if (mVertices[i].y < mMinBounds.y) mMinBounds.y = mVertices[i].y;

        if (mVertices[i].z > mMaxBounds.z) mMaxBounds.z = mVertices[i].z;
        if (mVertices[i].z < mMinBounds.z) mMinBounds.z = mVertices[i].z;
    }

    // Add the object margin to the bounds
    mMaxBounds += Vector3(mMargin, mMargin, mMargin);
    mMinBounds -= Vector3(mMargin, mMargin, mMargin);
}

// Test equality between two cone shapes
bool ConvexMeshShape::isEqualTo(const CollisionShape& otherCollisionShape) const {
    const ConvexMeshShape& otherShape = dynamic_cast<const ConvexMeshShape&>(otherCollisionShape);

    assert(mNbVertices == mVertices.size());

    if (mNbVertices != otherShape.mNbVertices) return false;

    if (mIsEdgesInformationUsed != otherShape.mIsEdgesInformationUsed) return false;

    if (mEdgesAdjacencyList.size() != otherShape.mEdgesAdjacencyList.size()) return false;

    // Check that the vertices are the same
    for (uint i=0; i<mNbVertices; i++) {
        if (mVertices[i] != otherShape.mVertices[i]) return false;
    }

    // Check that the edges are the same
    for (uint i=0; i<mEdgesAdjacencyList.size(); i++) {

        assert(otherShape.mEdgesAdjacencyList.count(i) == 1);
        if (mEdgesAdjacencyList.at(i) != otherShape.mEdgesAdjacencyList.at(i)) return false;
    }

    return true;
}

// Raycast method with feedback information
bool ConvexMeshShape::raycast(const Ray& ray, RaycastInfo& raycastInfo, ProxyShape* proxyShape) const {
    return proxyShape->mBody->mWorld.mCollisionDetection.mNarrowPhaseGJKAlgorithm.raycast(
                                     ray, proxyShape, raycastInfo);
}
