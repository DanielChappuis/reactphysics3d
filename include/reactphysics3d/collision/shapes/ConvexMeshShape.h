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

#ifndef REACTPHYSICS3D_CONVEX_MESH_SHAPE_H
#define REACTPHYSICS3D_CONVEX_MESH_SHAPE_H

// Libraries
#include <reactphysics3d/collision/shapes/ConvexPolyhedronShape.h>
#include <reactphysics3d/mathematics/mathematics.h>
#include <reactphysics3d/collision/PolyhedronMesh.h>

/// ReactPhysics3D namespace
namespace reactphysics3d {

// Declaration
class GJKAlgorithm;
class PhysicsWorld;

// Class ConvexMeshShape
/**
 * This class represents a convex mesh shape. In order to create a convex mesh shape, you
 * need to indicate the local-space position of the mesh vertices. The center of mass
 * of the shape will be at the origin of the local-space geometry that you use to create
 * the mesh.
 */
class ConvexMeshShape : public ConvexPolyhedronShape {

    protected :

        // -------------------- Attributes -------------------- //

        /// Polyhedron structure of the mesh
        PolyhedronMesh* mPolyhedronMesh;

        /// Mesh minimum bounds in the three local x, y and z directions
        Vector3 mMinBounds;

        /// Mesh maximum bounds in the three local x, y and z directions
        Vector3 mMaxBounds;

        /// Scale of the mesh
        Vector3 mScale;

        // -------------------- Methods -------------------- //

        /// Constructor
        ConvexMeshShape(PolyhedronMesh* polyhedronMesh,  MemoryAllocator& allocator, const Vector3& scale = Vector3(1,1,1));

        /// Recompute the bounds of the mesh
        void recalculateBounds();

        /// Return a local support point in a given direction without the object margin.
        virtual Vector3 getLocalSupportPointWithoutMargin(const Vector3& direction) const override;

        /// Return true if a point is inside the collision shape
        virtual bool testPointInside(const Vector3& localPoint, Collider* collider) const override;

        /// Raycast method with feedback information
        virtual bool raycast(const Ray& ray, RaycastInfo& raycastInfo, Collider* collider, MemoryAllocator& allocator) const override;

        /// Return the number of bytes used by the collision shape
        virtual size_t getSizeInBytes() const override;

        /// Destructor
        virtual ~ConvexMeshShape() override = default;

    public :

        // -------------------- Methods -------------------- //

        /// Deleted copy-constructor
        ConvexMeshShape(const ConvexMeshShape& shape) = delete;

        /// Deleted assignment operator
        ConvexMeshShape& operator=(const ConvexMeshShape& shape) = delete;

        /// Return the scale
        const Vector3& getScale() const;

        /// Set the scale
        void setScale(const Vector3& scale);

        /// Return the local bounds of the shape in x, y and z directions
        virtual void getLocalBounds(Vector3& min, Vector3& max) const override;

        /// Return the local inertia tensor of the collision shape.
        virtual Vector3 getLocalInertiaTensor(decimal mass) const override;

        /// Return the number of faces of the polyhedron
        virtual uint32 getNbFaces() const override;

        /// Return a given face of the polyhedron
        virtual const HalfEdgeStructure::Face& getFace(uint32 faceIndex) const override;

        /// Return the number of vertices of the polyhedron
        virtual uint32 getNbVertices() const override;

        /// Return a given vertex of the polyhedron
        virtual const HalfEdgeStructure::Vertex& getVertex(uint32 vertexIndex) const override;

        /// Return the number of half-edges of the polyhedron
        virtual uint32 getNbHalfEdges() const override;

        /// Return a given half-edge of the polyhedron
        virtual const HalfEdgeStructure::Edge& getHalfEdge(uint32 edgeIndex) const override;

        /// Return the position of a given vertex
        virtual Vector3 getVertexPosition(uint32 vertexIndex) const override;

        /// Return the normal vector of a given face of the polyhedron
        virtual Vector3 getFaceNormal(uint32 faceIndex) const override;

        /// Return the centroid of the polyhedron
        virtual Vector3 getCentroid() const override;

        /// Compute and return the volume of the collision shape
        virtual decimal getVolume() const override;

        /// Return the string representation of the shape
        virtual std::string to_string() const override;

        // ----- Friendship ----- //

        friend class PhysicsCommon;
};

// Return the number of bytes used by the collision shape
RP3D_FORCE_INLINE size_t ConvexMeshShape::getSizeInBytes() const {
    return sizeof(ConvexMeshShape);
}

// Return the scaling vector
RP3D_FORCE_INLINE const Vector3& ConvexMeshShape::getScale() const {
    return mScale;
}

// Set the scale
/// Note that you might want to recompute the inertia tensor and center of mass of the body
/// after changing the scale of a collision shape
RP3D_FORCE_INLINE void ConvexMeshShape::setScale(const Vector3& scale) {
    mScale = scale;
    recalculateBounds();
    notifyColliderAboutChangedSize();
}

// Return the local bounds of the shape in x, y and z directions
/**
 * @param min The minimum bounds of the shape in local-space coordinates
 * @param max The maximum bounds of the shape in local-space coordinates
 */
RP3D_FORCE_INLINE void ConvexMeshShape::getLocalBounds(Vector3& min, Vector3& max) const {
    min = mMinBounds;
    max = mMaxBounds;
}

// Return the local inertia tensor of the collision shape.
/// The local inertia tensor of the convex mesh is approximated using the inertia tensor
/// of its bounding box.
/**
* @param mass Mass to use to compute the inertia tensor of the collision shape
*/
RP3D_FORCE_INLINE Vector3 ConvexMeshShape::getLocalInertiaTensor(decimal mass) const {
    const decimal factor = (decimal(1.0) / decimal(3.0)) * mass;
    const Vector3 realExtent = decimal(0.5) * (mMaxBounds - mMinBounds);
    assert(realExtent.x > 0 && realExtent.y > 0 && realExtent.z > 0);
    const decimal xSquare = realExtent.x * realExtent.x;
    const decimal ySquare = realExtent.y * realExtent.y;
    const decimal zSquare = realExtent.z * realExtent.z;
    return Vector3(factor * (ySquare + zSquare), factor * (xSquare + zSquare), factor * (xSquare + ySquare));
}

// Return the number of faces of the polyhedron
RP3D_FORCE_INLINE uint32 ConvexMeshShape::getNbFaces() const {
    return mPolyhedronMesh->getHalfEdgeStructure().getNbFaces();
}

// Return a given face of the polyhedron
RP3D_FORCE_INLINE const HalfEdgeStructure::Face& ConvexMeshShape::getFace(uint32 faceIndex) const {
    assert(faceIndex < getNbFaces());
    return mPolyhedronMesh->getHalfEdgeStructure().getFace(faceIndex);
}

// Return the number of vertices of the polyhedron
RP3D_FORCE_INLINE uint32 ConvexMeshShape::getNbVertices() const {
    return mPolyhedronMesh->getHalfEdgeStructure().getNbVertices();
}

// Return a given vertex of the polyhedron
RP3D_FORCE_INLINE const HalfEdgeStructure::Vertex& ConvexMeshShape::getVertex(uint32 vertexIndex) const {
    assert(vertexIndex < getNbVertices());
    return mPolyhedronMesh->getHalfEdgeStructure().getVertex(vertexIndex);
}

// Return the number of half-edges of the polyhedron
RP3D_FORCE_INLINE uint32 ConvexMeshShape::getNbHalfEdges() const {
    return mPolyhedronMesh->getHalfEdgeStructure().getNbHalfEdges();
}

// Return a given half-edge of the polyhedron
RP3D_FORCE_INLINE const HalfEdgeStructure::Edge& ConvexMeshShape::getHalfEdge(uint32 edgeIndex) const {
    assert(edgeIndex < getNbHalfEdges());
    return mPolyhedronMesh->getHalfEdgeStructure().getHalfEdge(edgeIndex);
}

// Return the position of a given vertex
RP3D_FORCE_INLINE Vector3 ConvexMeshShape::getVertexPosition(uint32 vertexIndex) const {
    assert(vertexIndex < getNbVertices());
    return mPolyhedronMesh->getVertex(vertexIndex) * mScale;
}

// Return the normal vector of a given face of the polyhedron
RP3D_FORCE_INLINE Vector3 ConvexMeshShape::getFaceNormal(uint32 faceIndex) const {
    assert(faceIndex < getNbFaces());
    return mPolyhedronMesh->getFaceNormal(faceIndex);
}

// Return the centroid of the polyhedron
RP3D_FORCE_INLINE Vector3 ConvexMeshShape::getCentroid() const {
    return mPolyhedronMesh->getCentroid() * mScale;
}


// Compute and return the volume of the collision shape
RP3D_FORCE_INLINE decimal ConvexMeshShape::getVolume() const {
    return mPolyhedronMesh->getVolume();
}

}

#endif
