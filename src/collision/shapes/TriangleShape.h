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

#ifndef REACTPHYSICS3D_TRIANGLE_SHAPE_H
#define REACTPHYSICS3D_TRIANGLE_SHAPE_H

// Libraries
#include "mathematics/mathematics.h"
#include "ConvexPolyhedronShape.h"

/// ReactPhysics3D namespace
namespace reactphysics3d {

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

        /// Faces information for the two faces of the triangle
        HalfEdgeStructure::Face mFaces[2];

        /// Edges information for the six edges of the triangle
        HalfEdgeStructure::Edge mEdges[6];

        // -------------------- Methods -------------------- //

        /// Return a local support point in a given direction without the object margin
        virtual Vector3 getLocalSupportPointWithoutMargin(const Vector3& direction) const override;

        /// Get a smooth contact normal for collision for a triangle of the mesh
        Vector3 computeSmoothLocalContactNormalForTriangle(const Vector3& localContactPoint) const;

        /// Return true if a point is inside the collision shape
        virtual bool testPointInside(const Vector3& localPoint, ProxyShape* proxyShape) const override;

        /// Raycast method with feedback information
        virtual bool raycast(const Ray& ray, RaycastInfo& raycastInfo, ProxyShape* proxyShape,
                             MemoryAllocator& allocator) const override;

        /// Return the number of bytes used by the collision shape
        virtual size_t getSizeInBytes() const override;

        /// Generate the id of the shape (used for temporal coherence)
        void generateId();

        // -------------------- Methods -------------------- //

        /// This method implements the technique described in Game Physics Pearl book
        void computeSmoothMeshContact(Vector3 localContactPointTriangle, const Transform& triangleShapeToWorldTransform,
                                      const Transform& worldToOtherShapeTransform, decimal penetrationDepth, bool isTriangleShape1,
                                      Vector3& outNewLocalContactPointOtherShape, Vector3& outSmoothWorldContactTriangleNormal) const;

    public:

        // -------------------- Methods -------------------- //

        /// Constructor
        TriangleShape(const Vector3* vertices, const Vector3* verticesNormals,
                      uint shapeId, MemoryAllocator& allocator);

        /// Destructor
        virtual ~TriangleShape() override = default;

        /// Deleted copy-constructor
        TriangleShape(const TriangleShape& shape) = delete;

        /// Deleted assignment operator
        TriangleShape& operator=(const TriangleShape& shape) = delete;

        /// Return the local bounds of the shape in x, y and z directions.
        virtual void getLocalBounds(Vector3& min, Vector3& max) const override;

        /// Return the local inertia tensor of the collision shape
        virtual void computeLocalInertiaTensor(Matrix3x3& tensor, decimal mass) const override;

        /// Update the AABB of a body using its collision shape
        virtual void computeAABB(AABB& aabb, const Transform& transform) const override;

        /// Return the raycast test type (front, back, front-back)
        TriangleRaycastSide getRaycastTestType() const;

        // Set the raycast test type (front, back, front-back)
        void setRaycastTestType(TriangleRaycastSide testType);

        /// Return the number of faces of the polyhedron
        virtual uint getNbFaces() const override;

        /// Return a given face of the polyhedron
        virtual const HalfEdgeStructure::Face& getFace(uint faceIndex) const override;

        /// Return the number of vertices of the polyhedron
        virtual uint getNbVertices() const override;

        /// Return a given vertex of the polyhedron
        virtual HalfEdgeStructure::Vertex getVertex(uint vertexIndex) const override;

        /// Return the position of a given vertex
        virtual Vector3 getVertexPosition(uint vertexIndex) const override;

        /// Return the normal vector of a given face of the polyhedron
        virtual Vector3 getFaceNormal(uint faceIndex) const override;

        /// Return the number of half-edges of the polyhedron
        virtual uint getNbHalfEdges() const override;

        /// Return a given half-edge of the polyhedron
        virtual const HalfEdgeStructure::Edge& getHalfEdge(uint edgeIndex) const override;

        /// Return the centroid of the polyhedron
        virtual Vector3 getCentroid() const override;

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
};

// Return the number of bytes used by the collision shape
inline size_t TriangleShape::getSizeInBytes() const {
    return sizeof(TriangleShape);
}

// Return a local support point in a given direction without the object margin
inline Vector3 TriangleShape::getLocalSupportPointWithoutMargin(const Vector3& direction) const {
    Vector3 dotProducts(direction.dot(mPoints[0]), direction.dot(mPoints[1]), direction.dot(mPoints[2]));
    return mPoints[dotProducts.getMaxAxis()];
}

// Return the local bounds of the shape in x, y and z directions.
// This method is used to compute the AABB of the box
/**
 * @param min The minimum bounds of the shape in local-space coordinates
 * @param max The maximum bounds of the shape in local-space coordinates
 */
inline void TriangleShape::getLocalBounds(Vector3& min, Vector3& max) const {

    const Vector3 xAxis(mPoints[0].x, mPoints[1].x, mPoints[2].x);
    const Vector3 yAxis(mPoints[0].y, mPoints[1].y, mPoints[2].y);
    const Vector3 zAxis(mPoints[0].z, mPoints[1].z, mPoints[2].z);
    min.setAllValues(xAxis.getMinValue(), yAxis.getMinValue(), zAxis.getMinValue());
    max.setAllValues(xAxis.getMaxValue(), yAxis.getMaxValue(), zAxis.getMaxValue());

    min -= Vector3(mMargin, mMargin, mMargin);
    max += Vector3(mMargin, mMargin, mMargin);
}

// Return the local inertia tensor of the triangle shape
/**
 * @param[out] tensor The 3x3 inertia tensor matrix of the shape in local-space
 *                    coordinates
 * @param mass Mass to use to compute the inertia tensor of the collision shape
 */
inline void TriangleShape::computeLocalInertiaTensor(Matrix3x3& tensor, decimal mass) const {
    tensor.setToZero();
}

// Return true if a point is inside the collision shape
inline bool TriangleShape::testPointInside(const Vector3& localPoint, ProxyShape* proxyShape) const {
    return false;
}

// Return the number of faces of the polyhedron
inline uint TriangleShape::getNbFaces() const {
    return 2;
}

// Return a given face of the polyhedron
inline const HalfEdgeStructure::Face& TriangleShape::getFace(uint faceIndex) const {
    assert(faceIndex < 2);
    return mFaces[faceIndex];
}

// Return the number of vertices of the polyhedron
inline uint TriangleShape::getNbVertices() const {
    return 3;
}

// Return a given vertex of the polyhedron
inline HalfEdgeStructure::Vertex TriangleShape::getVertex(uint vertexIndex) const {
    assert(vertexIndex < 3);

    HalfEdgeStructure::Vertex vertex(vertexIndex);
    switch (vertexIndex) {
        case 0: vertex.edgeIndex = 0; break;
        case 1: vertex.edgeIndex = 2; break;
        case 2: vertex.edgeIndex = 4; break;
    }
    return vertex;
}

// Return a given half-edge of the polyhedron
inline const HalfEdgeStructure::Edge& TriangleShape::getHalfEdge(uint edgeIndex) const {
    assert(edgeIndex < getNbHalfEdges());
    return mEdges[edgeIndex];
}

// Return the position of a given vertex
inline Vector3 TriangleShape::getVertexPosition(uint vertexIndex) const {
    assert(vertexIndex < 3);
    return mPoints[vertexIndex];
}

// Return the normal vector of a given face of the polyhedron
inline Vector3 TriangleShape::getFaceNormal(uint faceIndex) const {
    assert(faceIndex < 2);
    return faceIndex == 0 ? mNormal : -mNormal;
}

// Return the centroid of the box
inline Vector3 TriangleShape::getCentroid() const {
    return (mPoints[0] + mPoints[1] + mPoints[2]) / decimal(3.0);
}

// Return the number of half-edges of the polyhedron
inline uint TriangleShape::getNbHalfEdges() const {
    return 6;
}

// Return the raycast test type (front, back, front-back)
inline TriangleRaycastSide TriangleShape::getRaycastTestType() const {
    return mRaycastTestType;
}

// Set the raycast test type (front, back, front-back)
/**
 * @param testType Raycast test type for the triangle (front, back, front-back)
 */
inline void TriangleShape::setRaycastTestType(TriangleRaycastSide testType) {
    mRaycastTestType = testType;
}

// Return the string representation of the shape
inline std::string TriangleShape::to_string() const {
    return "TriangleShape{v1=" + mPoints[0].to_string() + ", v2=" + mPoints[1].to_string() + "," +
            "v3=" + mPoints[2].to_string() + "}";
}

}

#endif

