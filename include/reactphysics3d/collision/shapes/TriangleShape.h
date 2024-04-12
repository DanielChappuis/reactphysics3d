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

#ifndef REACTPHYSICS3D_TRIANGLE_SHAPE_H
#define REACTPHYSICS3D_TRIANGLE_SHAPE_H

// Libraries
#include <reactphysics3d/mathematics/mathematics.h>
#include <reactphysics3d/collision/shapes/AABB.h>
#include <reactphysics3d/collision/shapes/ConvexPolyhedronShape.h>

/// ReactPhysics3D namespace
namespace reactphysics3d {

// Forward declarations
class PhysicsCommon;

/// Raycast test side for the triangle
enum class TriangleRaycastSide {

    /// Raycast against front triangle
    FRONT,

    /// Raycast against back triangle
    BACK,

    /// Raycast against front and back triangle
    FRONT_AND_BACK
};

// Class TriangleShape
/**
 * This class represents a triangle collision shape that is centered
 * at the origin and defined three points. A user cannot instanciate
 * an object of this class. This class is for internal use only. Instances
 * of this class are created when the user creates an HeightFieldShape and
 * a ConcaveMeshShape
 */
class TriangleShape : public ConvexPolyhedronShape {

    protected:

        // -------------------- Attribute -------------------- //


        /// Three points of the triangle
        Vector3 mPoints[3];

        /// Normal of the triangle
        Vector3 mNormal;

        /// Three vertices normals for smooth collision with triangle mesh
        Vector3 mVerticesNormals[3];

        /// Raycast test type for the triangle (front, back, front-back)
        TriangleRaycastSide mRaycastTestType;

        /// Reference to triangle half-edge structure
        HalfEdgeStructure& mTriangleHalfEdgeStructure;

        // -------------------- Methods -------------------- //

        /// Return a local support point in a given direction without the object margin
        virtual Vector3 getLocalSupportPointWithoutMargin(const Vector3& direction) const override;

        /// Get a smooth contact normal for collision for a triangle of the mesh
        Vector3 computeSmoothLocalContactNormalForTriangle(const Vector3& localContactPoint) const;

        /// Return true if a point is inside the collider
        virtual bool testPointInside(const Vector3& localPoint, Collider* collider) const override;

        /// Raycast method with feedback information
        virtual bool raycast(const Ray& ray, RaycastInfo& raycastInfo, Collider* collider, MemoryAllocator& allocator) const override;

        /// Return the number of bytes used by the collision shape
        virtual size_t getSizeInBytes() const override;

        /// Generate the id of the shape (used for temporal coherence)
        void generateId();

        // -------------------- Methods -------------------- //

        /// This method implements the technique described in Game Physics Pearl book
        void computeSmoothMeshContact(Vector3 localContactPointTriangle, const Transform& triangleShapeToWorldTransform,
                                      const Transform& worldToOtherShapeTransform, decimal penetrationDepth, bool isTriangleShape1,
                                      Vector3& outNewLocalContactPointOtherShape, Vector3& outSmoothWorldContactTriangleNormal) const;

        /// Constructor
        TriangleShape(const Vector3* vertices, const Vector3* verticesNormals, uint32 shapeId, HalfEdgeStructure& triangleHalfEdgeStructure,
                      MemoryAllocator& allocator);

        /// Constructor
        TriangleShape(const Vector3* vertices, uint32 shapeId, HalfEdgeStructure& triangleHalfEdgeStructure, MemoryAllocator& allocator);

        /// Destructor
        virtual ~TriangleShape() override = default;

    public:

        // -------------------- Methods -------------------- //

        /// Deleted copy-constructor
        TriangleShape(const TriangleShape& shape) = delete;

        /// Deleted assignment operator
        TriangleShape& operator=(const TriangleShape& shape) = delete;

        /// Return the local bounds of the shape in x, y and z directions.
        virtual AABB getLocalBounds() const override;

        /// Return the local inertia tensor of the collision shape
        virtual Vector3 getLocalInertiaTensor(decimal mass) const override;

        /// Compute the transformed AABB of the collision shape given a transform
        virtual AABB computeTransformedAABB(const Transform& transform) const override;

        /// Return the raycast test type (front, back, front-back)
        TriangleRaycastSide getRaycastTestType() const;

        // Set the raycast test type (front, back, front-back)
        void setRaycastTestType(TriangleRaycastSide testType);

        /// Return the number of faces of the polyhedron
        virtual uint32 getNbFaces() const override;

        /// Return a given face of the polyhedron
        virtual const HalfEdgeStructure::Face& getFace(uint32 faceIndex) const override;

        /// Return the number of vertices of the polyhedron
        virtual uint32 getNbVertices() const override;

        /// Return a given vertex of the polyhedron
        virtual const HalfEdgeStructure::Vertex& getVertex(uint32 vertexIndex) const override;

        /// Return the position of a given vertex
        virtual Vector3 getVertexPosition(uint32 vertexIndex) const override;

        /// Return the normal vector of a given face of the polyhedron
        virtual Vector3 getFaceNormal(uint32 faceIndex) const override;

        /// Return the number of half-edges of the polyhedron
        virtual uint32 getNbHalfEdges() const override;

        /// Return a given half-edge of the polyhedron
        virtual const HalfEdgeStructure::Edge& getHalfEdge(uint32 edgeIndex) const override;

        /// Return the centroid of the polyhedron
        virtual Vector3 getCentroid() const override;

        /// Compute and return the volume of the collision shape
        virtual decimal getVolume() const override;

        /// This method compute the smooth mesh contact with a triangle in case one of the two collision shapes is a triangle. The idea in this case is to use a smooth vertex normal of the triangle mesh
        static void computeSmoothTriangleMeshContact(const CollisionShape* shape1, const CollisionShape* shape2,
                                                     Vector3& localContactPointShape1, Vector3& localContactPointShape2,
                                                     const Transform& shape1ToWorld, const Transform& shape2ToWorld,
                                                     decimal penetrationDepth, Vector3& outSmoothVertexNormal);

        /// Return the string representation of the shape
        virtual std::string to_string() const override;

        // ---------- Friendship ---------- //

        friend class ConcaveMeshRaycastCallback;
        friend class TriangleOverlapCallback;
        friend class MiddlePhaseTriangleCallback;
        friend class HeightField;
        friend class CollisionDetectionSystem;
};

// Return the number of bytes used by the collision shape
RP3D_FORCE_INLINE size_t TriangleShape::getSizeInBytes() const {
    return sizeof(TriangleShape);
}

// Return a local support point in a given direction without the object margin
RP3D_FORCE_INLINE Vector3 TriangleShape::getLocalSupportPointWithoutMargin(const Vector3& direction) const {
    Vector3 dotProducts(direction.dot(mPoints[0]), direction.dot(mPoints[1]), direction.dot(mPoints[2]));
    return mPoints[dotProducts.getMaxAxis()];
}

// Return the local bounds of the shape in x, y and z directions.
/**
 * @return The AABB with the min/max bounds of the shape
 */
RP3D_FORCE_INLINE AABB TriangleShape::getLocalBounds() const {

    const Vector3 xAxis(mPoints[0].x, mPoints[1].x, mPoints[2].x);
    const Vector3 yAxis(mPoints[0].y, mPoints[1].y, mPoints[2].y);
    const Vector3 zAxis(mPoints[0].z, mPoints[1].z, mPoints[2].z);
    Vector3 min(xAxis.getMinValue(), yAxis.getMinValue(), zAxis.getMinValue());
    Vector3 max(xAxis.getMaxValue(), yAxis.getMaxValue(), zAxis.getMaxValue());

    min -= Vector3(mMargin, mMargin, mMargin);
    max += Vector3(mMargin, mMargin, mMargin);

    return AABB(min, max);
}

// Return the local inertia tensor of the triangle shape
/**
 * @param mass Mass to use to compute the inertia tensor of the collision shape
 * @return A vector with the three diagonal values of the local inertia tensor
 */
RP3D_FORCE_INLINE Vector3 TriangleShape::getLocalInertiaTensor(decimal /*mass*/) const {
    return Vector3(0, 0, 0);
}

// Return true if a point is inside the collision shape
RP3D_FORCE_INLINE bool TriangleShape::testPointInside(const Vector3& /*localPoint*/, Collider* /*collider*/) const {
    return false;
}

// Return the number of faces of the polyhedron
RP3D_FORCE_INLINE uint32 TriangleShape::getNbFaces() const {
    return 2;
}

// Return the number of vertices of the polyhedron
RP3D_FORCE_INLINE uint32 TriangleShape::getNbVertices() const {
    return 3;
}

// Return a given vertex of the polyhedron
RP3D_FORCE_INLINE const HalfEdgeStructure::Vertex& TriangleShape::getVertex(uint32 vertexIndex) const {
    assert(vertexIndex < 3);
    return mTriangleHalfEdgeStructure.getVertex(vertexIndex);
}

// Return the position of a given vertex
RP3D_FORCE_INLINE Vector3 TriangleShape::getVertexPosition(uint32 vertexIndex) const {
    assert(vertexIndex < 3);
    return mPoints[vertexIndex];
}

// Return the normal vector of a given face of the polyhedron
RP3D_FORCE_INLINE Vector3 TriangleShape::getFaceNormal(uint32 faceIndex) const {
    assert(faceIndex < 2);
    assert(mNormal.length() > decimal(0.0));
    return faceIndex == 0 ? mNormal : -mNormal;
}

// Return the centroid of the box
RP3D_FORCE_INLINE Vector3 TriangleShape::getCentroid() const {
    return (mPoints[0] + mPoints[1] + mPoints[2]) / decimal(3.0);
}

// Return the number of half-edges of the polyhedron
RP3D_FORCE_INLINE uint32 TriangleShape::getNbHalfEdges() const {
    return 6;
}

// Return the raycast test type (front, back, front-back)
RP3D_FORCE_INLINE TriangleRaycastSide TriangleShape::getRaycastTestType() const {
    return mRaycastTestType;
}

// Set the raycast test type (front, back, front-back)
/**
 * @param testType Raycast test type for the triangle (front, back, front-back)
 */
RP3D_FORCE_INLINE void TriangleShape::setRaycastTestType(TriangleRaycastSide testType) {
    mRaycastTestType = testType;
}

// Return the string representation of the shape
RP3D_FORCE_INLINE std::string TriangleShape::to_string() const {
    return "TriangleShape{v1=" + mPoints[0].to_string() + ", v2=" + mPoints[1].to_string() + "," +
            "v3=" + mPoints[2].to_string() + "}";
}

// Compute and return the volume of the collision shape
RP3D_FORCE_INLINE decimal TriangleShape::getVolume() const {
    return decimal(0.0);
}

// Get a smooth contact normal for collision for a triangle of the mesh
/// This is used to avoid the internal edges issue that occurs when a shape is colliding with
/// several triangles of a concave mesh. If the shape collide with an edge of the triangle for instance,
/// the computed contact normal from this triangle edge is not necessarily in the direction of the surface
/// normal of the mesh at this point. The idea to solve this problem is to use the real (smooth) surface
/// normal of the mesh at this point as the contact normal. This technique is described in the chapter 5
/// of the Game Physics Pearl book by Gino van der Bergen and Dirk Gregorius. The vertices normals of the
/// mesh are either provided by the user or precomputed if the user did not provide them. Note that we only
/// use the interpolated normal if the contact point is on an edge of the triangle. If the contact is in the
/// middle of the triangle, we return the true triangle normal.
RP3D_FORCE_INLINE Vector3 TriangleShape::computeSmoothLocalContactNormalForTriangle(const Vector3& localContactPoint) const {

    assert(mNormal.length() > decimal(0.0));

    // Compute the barycentric coordinates of the point in the triangle
    decimal u, v, w;
    computeBarycentricCoordinatesInTriangle(mPoints[0], mPoints[1], mPoints[2], localContactPoint, u, v, w);

    // If the contact is in the middle of the triangle face (not on the edges)
    if (u > MACHINE_EPSILON && v > MACHINE_EPSILON && w > MACHINE_EPSILON) {

        // We return the true triangle face normal (not the interpolated one)
        return mNormal;
    }

    // We compute the contact normal as the barycentric interpolation of the three vertices normals
    const Vector3 interpolatedNormal = u * mVerticesNormals[0] + v * mVerticesNormals[1] + w * mVerticesNormals[2];

    // If the interpolated normal is degenerated
    if (interpolatedNormal.lengthSquare() < MACHINE_EPSILON) {

        // Return the original normal
        return mNormal;
    }

    return interpolatedNormal.getUnit();
}

// This method compute the smooth mesh contact with a triangle in case one of the two collision
// shapes is a triangle. The idea in this case is to use a smooth vertex normal of the triangle mesh
// at the contact point instead of the triangle normal to avoid the internal edge collision issue.
// This method will return the new smooth world contact
// normal of the triangle and the the local contact point on the other shape.
RP3D_FORCE_INLINE void TriangleShape::computeSmoothTriangleMeshContact(const CollisionShape* shape1, const CollisionShape* shape2,
                                                     Vector3& localContactPointShape1, Vector3& localContactPointShape2,
                                                     const Transform& shape1ToWorld, const Transform& shape2ToWorld,
                                                     decimal penetrationDepth, Vector3& outSmoothVertexNormal) {

    assert(shape1->getName() != CollisionShapeName::TRIANGLE || shape2->getName() != CollisionShapeName::TRIANGLE);

    // If one the shape is a triangle
    bool isShape1Triangle = shape1->getName() == CollisionShapeName::TRIANGLE;
    if (isShape1Triangle || shape2->getName() == CollisionShapeName::TRIANGLE) {

        const TriangleShape* triangleShape = isShape1Triangle ? static_cast<const TriangleShape*>(shape1):
                                                                static_cast<const TriangleShape*>(shape2);

        // Compute the smooth triangle mesh contact normal and recompute the local contact point on the other shape
        triangleShape->computeSmoothMeshContact(isShape1Triangle ? localContactPointShape1 : localContactPointShape2,
                                                isShape1Triangle ? shape1ToWorld : shape2ToWorld,
                                                isShape1Triangle ? shape2ToWorld.getInverse() : shape1ToWorld.getInverse(),
                                                penetrationDepth, isShape1Triangle,
                                                isShape1Triangle ? localContactPointShape2 : localContactPointShape1,
                                                outSmoothVertexNormal);
    }
}

// Return a given face of the polyhedron
RP3D_FORCE_INLINE const HalfEdgeStructure::Face& TriangleShape::getFace(uint32 faceIndex) const {
    assert(faceIndex < 2);
    return mTriangleHalfEdgeStructure.getFace(faceIndex);
}

// Return a given half-edge of the polyhedron
RP3D_FORCE_INLINE const HalfEdgeStructure::Edge& TriangleShape::getHalfEdge(uint32 edgeIndex) const {
    assert(edgeIndex < getNbHalfEdges());
    return mTriangleHalfEdgeStructure.getHalfEdge(edgeIndex);
}

}

#endif

