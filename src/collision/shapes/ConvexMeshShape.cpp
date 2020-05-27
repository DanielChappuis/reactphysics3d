/********************************************************************************
* ReactPhysics3D physics library, http://www.reactphysics3d.com                 *
* Copyright (c) 2010-2020 Daniel Chappuis                                       *
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
#include <reactphysics3d/configuration.h>
#include <reactphysics3d/collision/shapes/ConvexMeshShape.h>
#include <reactphysics3d/engine/PhysicsWorld.h>
#include <reactphysics3d/collision/RaycastInfo.h>

using namespace reactphysics3d;

// Constructor to initialize with an array of 3D vertices.
/// This method creates an internal copy of the input vertices.
/**
 * @param arrayVertices Array with the vertices of the convex mesh
 * @param nbVertices Number of vertices in the convex mesh
 * @param stride Stride between the beginning of two elements in the vertices array
 * @param margin Collision margin (in meters) around the collision shape
 */
ConvexMeshShape::ConvexMeshShape(PolyhedronMesh* polyhedronMesh, MemoryAllocator& allocator, const Vector3& scale)
                : ConvexPolyhedronShape(CollisionShapeName::CONVEX_MESH, allocator), mPolyhedronMesh(polyhedronMesh),
                  mMinBounds(0, 0, 0), mMaxBounds(0, 0, 0), mScale(scale) {

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

    decimal maxDotProduct = DECIMAL_SMALLEST;
    uint indexMaxDotProduct = 0;

    // For each vertex of the mesh
    for (uint i=0; i<mPolyhedronMesh->getNbVertices(); i++) {

        // Compute the dot product of the current vertex
        decimal dotProduct = direction.dot(mPolyhedronMesh->getVertex(i));

        // If the current dot product is larger than the maximum one
        if (dotProduct > maxDotProduct) {
            indexMaxDotProduct = i;
            maxDotProduct = dotProduct;
        }
    }

    assert(maxDotProduct >= decimal(0.0));

    // Return the vertex with the largest dot product in the support direction
    return mPolyhedronMesh->getVertex(indexMaxDotProduct) * mScale;
}

// Recompute the bounds of the mesh
void ConvexMeshShape::recalculateBounds() {

    mMinBounds = mPolyhedronMesh->getVertex(0);
    mMaxBounds = mPolyhedronMesh->getVertex(0);

    // For each vertex of the mesh
    for (uint i=1; i<mPolyhedronMesh->getNbVertices(); i++) {

        if (mPolyhedronMesh->getVertex(i).x > mMaxBounds.x) mMaxBounds.x = mPolyhedronMesh->getVertex(i).x;
        if (mPolyhedronMesh->getVertex(i).x < mMinBounds.x) mMinBounds.x = mPolyhedronMesh->getVertex(i).x;

        if (mPolyhedronMesh->getVertex(i).y > mMaxBounds.y) mMaxBounds.y = mPolyhedronMesh->getVertex(i).y;
        if (mPolyhedronMesh->getVertex(i).y < mMinBounds.y) mMinBounds.y = mPolyhedronMesh->getVertex(i).y;

        if (mPolyhedronMesh->getVertex(i).z > mMaxBounds.z) mMaxBounds.z = mPolyhedronMesh->getVertex(i).z;
        if (mPolyhedronMesh->getVertex(i).z < mMinBounds.z) mMinBounds.z = mPolyhedronMesh->getVertex(i).z;
    }

    // Apply the local scaling factor
    mMaxBounds = mMaxBounds * mScale;
    mMinBounds = mMinBounds * mScale;
}

// Raycast method with feedback information
/// This method implements the technique in the book "Real-time Collision Detection" by
/// Christer Ericson.
bool ConvexMeshShape::raycast(const Ray& ray, RaycastInfo& raycastInfo, Collider* collider, MemoryAllocator& allocator) const {

    // Ray direction
    Vector3 direction = ray.point2 - ray.point1;

    decimal tMin = decimal(0.0);
    decimal tMax = ray.maxFraction;
    Vector3 currentFaceNormal;
    bool isIntersectionFound = false;

    const HalfEdgeStructure& halfEdgeStructure = mPolyhedronMesh->getHalfEdgeStructure();

    // For each face of the convex mesh
    for (uint f=0; f < mPolyhedronMesh->getNbFaces(); f++) {

        const HalfEdgeStructure::Face& face = halfEdgeStructure.getFace(f);
        const Vector3 faceNormal = mPolyhedronMesh->getFaceNormal(f);
        const HalfEdgeStructure::Vertex& faceVertex = halfEdgeStructure.getVertex(face.faceVertices[0]);
        const Vector3 facePoint = mPolyhedronMesh->getVertex(faceVertex.vertexPointIndex);
        decimal denom = faceNormal.dot(direction);
        decimal planeD = faceNormal.dot(facePoint);
        decimal dist = planeD -  faceNormal.dot(ray.point1);

        // If ray is parallel to the face
        if (denom == decimal(0.0)) {

            // If ray is outside the clipping face, we return no intersection
            if (dist < decimal(0.0)) return false;
        }
        else {

            // Compute the intersection between the ray and the current face plane
            decimal t = dist / denom;

            // Update the current ray intersection by clipping it with the current face plane
            // If the place faces the ray
            if (denom < decimal(0.0)) {
                // Clip the current ray intersection as it enters the convex mesh
                if (t > tMin) {
                    tMin = t;
                    currentFaceNormal = faceNormal;
                    isIntersectionFound = true;
                }
            }
            else {
                // Clip the current ray intersection as it exits the convex mesh
                if (t < tMax) tMax = t;
            }

            // If the ray intersection with the convex mesh becomes empty, report no intersection
            if (tMin > tMax) return false;
        }
    }

    if (isIntersectionFound) {

        // The ray intersects with the convex mesh
        assert(tMin >= decimal(0.0));
        assert(tMax <= ray.maxFraction);
        assert(tMin <= tMax);
        assert(currentFaceNormal.lengthSquare() > decimal(0.0));

        // The ray intersects the three slabs, we compute the hit point
        Vector3 localHitPoint = ray.point1 + tMin * direction;

        raycastInfo.hitFraction = tMin;
        raycastInfo.body = collider->getBody();
        raycastInfo.collider = collider;
        raycastInfo.worldPoint = localHitPoint;
        raycastInfo.worldNormal = currentFaceNormal;

        return true;
    }

    return false;
}

// Return true if a point is inside the collision shape
bool ConvexMeshShape::testPointInside(const Vector3& localPoint, Collider* collider) const {

    const HalfEdgeStructure& halfEdgeStructure = mPolyhedronMesh->getHalfEdgeStructure();

    // For each face plane of the convex mesh
    for (uint f=0; f < mPolyhedronMesh->getNbFaces(); f++) {

        const HalfEdgeStructure::Face& face = halfEdgeStructure.getFace(f);
        const Vector3 faceNormal = mPolyhedronMesh->getFaceNormal(f);
        const HalfEdgeStructure::Vertex& faceVertex = halfEdgeStructure.getVertex(face.faceVertices[0]);
        const Vector3 facePoint = mPolyhedronMesh->getVertex(faceVertex.vertexPointIndex);

        // If the point is out of the face plane, it is outside of the convex mesh
        if (computePointToPlaneDistance(localPoint, faceNormal, facePoint) > decimal(0.0)) return false;
    }

    return true;
}

// Return the string representation of the shape
std::string ConvexMeshShape::to_string() const {

    std::stringstream ss;
    ss << "ConvexMeshShape{" << std::endl;
    ss << "nbVertices=" << mPolyhedronMesh->getNbVertices() << std::endl;
    ss << "nbFaces=" << mPolyhedronMesh->getNbFaces() << std::endl;

    ss << "vertices=[";

    for (uint v=0; v < mPolyhedronMesh->getNbVertices(); v++) {

        Vector3 vertex = mPolyhedronMesh->getVertex(v);
        ss << vertex.to_string();
        if (v != mPolyhedronMesh->getNbVertices() - 1) {
            ss << ", ";
        }
    }

    ss << "], faces=[";

    HalfEdgeStructure halfEdgeStruct = mPolyhedronMesh->getHalfEdgeStructure();
    for (uint f=0; f < mPolyhedronMesh->getNbFaces(); f++) {

        const HalfEdgeStructure::Face& face = halfEdgeStruct.getFace(f);

        ss << "[";

        for (uint v=0; v < face.faceVertices.size(); v++) {

            ss << face.faceVertices[v];
            if (v != face.faceVertices.size() - 1) {
               ss << ",";
            }
        }

        ss << "]";
    }

    ss << "]}";

    return ss.str();
}

