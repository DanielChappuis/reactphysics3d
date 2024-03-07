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

#ifndef REACTPHYSICS3D_BOX_SHAPE_H
#define REACTPHYSICS3D_BOX_SHAPE_H

// Libraries
#include <reactphysics3d/collision/shapes/ConvexPolyhedronShape.h>
#include <reactphysics3d/mathematics/mathematics.h>

/// ReactPhysics3D namespace
namespace reactphysics3d {

// Declarations
class Body;
class DefaultAllocator;
class PhysicsCommon;
class AABB;

// Class BoxShape
/**
 * This class represents a 3D box shape. Those axis are unit length.
 * The three extents are half-widths of the box along the three
 * axis x, y, z local axis. The "transform" of the corresponding
 * body will give an orientation and a position to the box.
 */
class BoxShape : public ConvexPolyhedronShape {

    protected :

        // -------------------- Attributes -------------------- //

        /// Half-extents of the box in the x, y and z direction
        Vector3 mHalfExtents;

        /// Reference to the physics common object
        PhysicsCommon& mPhysicsCommon;

        // -------------------- Methods -------------------- //

        /// Constructor
        BoxShape(const Vector3& halfExtents, MemoryAllocator& allocator, PhysicsCommon& physicsCommon);

        /// Return a local support point in a given direction without the object margin
        virtual Vector3 getLocalSupportPointWithoutMargin(const Vector3& direction) const override;

        /// Return true if a point is inside the collision shape
        virtual bool testPointInside(const Vector3& localPoint, Collider* collider) const override;

        /// Raycast method with feedback information
        virtual bool raycast(const Ray& ray, RaycastInfo& raycastInfo, Collider* collider, MemoryAllocator& allocator) const override;

        /// Return the number of bytes used by the collision shape
        virtual size_t getSizeInBytes() const override;

        /// Destructor
        virtual ~BoxShape() override = default;

    public :

        // -------------------- Methods -------------------- //

        /// Deleted copy-constructor
        BoxShape(const BoxShape& shape) = delete;

        /// Deleted assignment operator
        BoxShape& operator=(const BoxShape& shape) = delete;

        /// Return the half-extents of the box
        const Vector3& getHalfExtents() const;

        /// Set the half-extents of the box
        void setHalfExtents(const Vector3& halfExtents);

        /// Return the local bounds of the shape in x, y and z directions
        virtual AABB getLocalBounds() const override;

        /// Return the local inertia tensor of the collision shape
        virtual Vector3 getLocalInertiaTensor(decimal mass) const override;

        /// Compute and return the volume of the collision shape
        virtual decimal getVolume() const override;

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

        /// Return the string representation of the shape
        virtual std::string to_string() const override;

        // ----- Friendship ----- //

        friend class PhysicsCommon;
};

// Return the extents of the box
/**
 * @return The vector with the three half-extents of the box shape
 */
RP3D_FORCE_INLINE const Vector3& BoxShape::getHalfExtents() const {
    return mHalfExtents;
}

// Set the half-extents of the box
/// Note that you might want to recompute the inertia tensor and center of mass of the body
/// after changing the size of the collision shape
/**
 * @param halfExtents The vector with the three half-extents of the box
 */
RP3D_FORCE_INLINE void BoxShape::setHalfExtents(const Vector3& halfExtents) {
    mHalfExtents = halfExtents;

    notifyColliderAboutChangedSize();
}

// Return the number of bytes used by the collision shape
RP3D_FORCE_INLINE size_t BoxShape::getSizeInBytes() const {
    return sizeof(BoxShape);
}

// Return a local support point in a given direction without the object margin
RP3D_FORCE_INLINE Vector3 BoxShape::getLocalSupportPointWithoutMargin(const Vector3& direction) const {

    return Vector3(direction.x < decimal(0.0) ? -mHalfExtents.x : mHalfExtents.x,
                   direction.y < decimal(0.0) ? -mHalfExtents.y : mHalfExtents.y,
                   direction.z < decimal(0.0) ? -mHalfExtents.z : mHalfExtents.z);
}

// Return true if a point is inside the collision shape
RP3D_FORCE_INLINE bool BoxShape::testPointInside(const Vector3& localPoint, Collider* /*collider*/) const {
    return (localPoint.x < mHalfExtents[0] && localPoint.x > -mHalfExtents[0] &&
            localPoint.y < mHalfExtents[1] && localPoint.y > -mHalfExtents[1] &&
            localPoint.z < mHalfExtents[2] && localPoint.z > -mHalfExtents[2]);
}

// Return the number of faces of the polyhedron
RP3D_FORCE_INLINE uint32 BoxShape::getNbFaces() const {
    return 6;
}

// Return the number of vertices of the polyhedron
RP3D_FORCE_INLINE uint32 BoxShape::getNbVertices() const {
    return 8;
}

// Return the position of a given vertex
RP3D_FORCE_INLINE Vector3 BoxShape::getVertexPosition(uint32 vertexIndex) const {
    assert(vertexIndex < getNbVertices());

    switch(vertexIndex) {
        case 0: return Vector3(-mHalfExtents.x, -mHalfExtents.y, mHalfExtents.z);
        case 1: return Vector3(mHalfExtents.x, -mHalfExtents.y, mHalfExtents.z);
        case 2: return Vector3(mHalfExtents.x, mHalfExtents.y, mHalfExtents.z);
        case 3: return Vector3(-mHalfExtents.x, mHalfExtents.y, mHalfExtents.z);
        case 4: return Vector3(-mHalfExtents.x, -mHalfExtents.y, -mHalfExtents.z);
        case 5: return Vector3(mHalfExtents.x, -mHalfExtents.y, -mHalfExtents.z);
        case 6: return Vector3(mHalfExtents.x, mHalfExtents.y, -mHalfExtents.z);
        case 7: return Vector3(-mHalfExtents.x, mHalfExtents.y, -mHalfExtents.z);
    }

    assert(false);
    return Vector3::zero();
}

// Return the normal vector of a given face of the polyhedron
RP3D_FORCE_INLINE Vector3 BoxShape::getFaceNormal(uint32 faceIndex) const {
    assert(faceIndex < getNbFaces());

    switch(faceIndex) {
        case 0: return Vector3(0, 0, 1);
        case 1: return Vector3(1, 0, 0);
        case 2: return Vector3(0, 0, -1);
        case 3: return Vector3(-1, 0, 0);
        case 4: return Vector3(0, -1, 0);
        case 5: return Vector3(0, 1, 0);
    }

    assert(false);
    return Vector3::zero();
}

// Return the centroid of the box
RP3D_FORCE_INLINE Vector3 BoxShape::getCentroid() const {
    return Vector3::zero();
}

// Compute and return the volume of the collision shape
RP3D_FORCE_INLINE decimal BoxShape::getVolume() const {
    return 8 * mHalfExtents.x * mHalfExtents.y * mHalfExtents.z;
}

// Return the string representation of the shape
RP3D_FORCE_INLINE std::string BoxShape::to_string() const {
    return "BoxShape{extents=" + mHalfExtents.to_string() + "}";
}

// Return the number of half-edges of the polyhedron
RP3D_FORCE_INLINE uint32 BoxShape::getNbHalfEdges() const {
    return 24;
}

}

#endif
