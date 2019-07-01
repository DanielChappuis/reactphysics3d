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

#ifndef REACTPHYSICS3D_BOX_SHAPE_H
#define REACTPHYSICS3D_BOX_SHAPE_H

// Libraries
#include "ConvexPolyhedronShape.h"
#include "mathematics/mathematics.h"

/// ReactPhysics3D namespace
namespace reactphysics3d {

// Declarations
class CollisionBody;
class DefaultAllocator;

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

        /// Extent sizes of the box in the x, y and z direction
        Vector3 mExtent;

        /// Half-edge structure of the polyhedron
        HalfEdgeStructure mHalfEdgeStructure;

        // -------------------- Methods -------------------- //

        /// Return a local support point in a given direction without the object margin
        virtual Vector3 getLocalSupportPointWithoutMargin(const Vector3& direction) const override;

        /// Return true if a point is inside the collision shape
        virtual bool testPointInside(const Vector3& localPoint, ProxyShape* proxyShape) const override;

        /// Raycast method with feedback information
        virtual bool raycast(const Ray& ray, RaycastInfo& raycastInfo, ProxyShape* proxyShape, MemoryAllocator& allocator) const override;

        /// Return the number of bytes used by the collision shape
        virtual size_t getSizeInBytes() const override;

    public :

        // -------------------- Methods -------------------- //

        /// Constructor
        BoxShape(const Vector3& extent);

        /// Destructor
        virtual ~BoxShape() override = default;

        /// Deleted copy-constructor
        BoxShape(const BoxShape& shape) = delete;

        /// Deleted assignment operator
        BoxShape& operator=(const BoxShape& shape) = delete;

        /// Return the extents of the box
        Vector3 getExtent() const;

        /// Return the local bounds of the shape in x, y and z directions
        virtual void getLocalBounds(Vector3& min, Vector3& max) const override;

        /// Return the local inertia tensor of the collision shape
        virtual void computeLocalInertiaTensor(Matrix3x3& tensor, decimal mass) const override;

        /// Return the number of faces of the polyhedron
        virtual uint getNbFaces() const override;

        /// Return a given face of the polyhedron
        virtual const HalfEdgeStructure::Face& getFace(uint faceIndex) const override;

        /// Return the number of vertices of the polyhedron
        virtual uint getNbVertices() const override;

        /// Return a given vertex of the polyhedron
        virtual HalfEdgeStructure::Vertex getVertex(uint vertexIndex) const override;

        /// Return the number of half-edges of the polyhedron
        virtual uint getNbHalfEdges() const override;

        /// Return a given half-edge of the polyhedron
        virtual const HalfEdgeStructure::Edge& getHalfEdge(uint edgeIndex) const override;

        /// Return the position of a given vertex
        virtual Vector3 getVertexPosition(uint vertexIndex) const override;

        /// Return the normal vector of a given face of the polyhedron
        virtual Vector3 getFaceNormal(uint faceIndex) const override;

        /// Return the centroid of the polyhedron
        virtual Vector3 getCentroid() const override;

        /// Return the string representation of the shape
        virtual std::string to_string() const override;
};

// Return the extents of the box
/**
 * @return The vector with the three extents of the box shape (in meters)
 */
inline Vector3 BoxShape::getExtent() const {
    return mExtent;
}

// Return the local bounds of the shape in x, y and z directions
/// This method is used to compute the AABB of the box
/**
 * @param min The minimum bounds of the shape in local-space coordinates
 * @param max The maximum bounds of the shape in local-space coordinates
 */
inline void BoxShape::getLocalBounds(Vector3& min, Vector3& max) const {

    // Maximum bounds
    max = mExtent;

    // Minimum bounds
    min = -max;
}

// Return the number of bytes used by the collision shape
inline size_t BoxShape::getSizeInBytes() const {
    return sizeof(BoxShape);
}

// Return a local support point in a given direction without the object margin
inline Vector3 BoxShape::getLocalSupportPointWithoutMargin(const Vector3& direction) const {

    return Vector3(direction.x < decimal(0.0) ? -mExtent.x : mExtent.x,
                   direction.y < decimal(0.0) ? -mExtent.y : mExtent.y,
                   direction.z < decimal(0.0) ? -mExtent.z : mExtent.z);
}

// Return true if a point is inside the collision shape
inline bool BoxShape::testPointInside(const Vector3& localPoint, ProxyShape* proxyShape) const {
    return (localPoint.x < mExtent[0] && localPoint.x > -mExtent[0] &&
            localPoint.y < mExtent[1] && localPoint.y > -mExtent[1] &&
            localPoint.z < mExtent[2] && localPoint.z > -mExtent[2]);
}

// Return the number of faces of the polyhedron
inline uint BoxShape::getNbFaces() const {
    return 6;
}

// Return a given face of the polyhedron
inline const HalfEdgeStructure::Face& BoxShape::getFace(uint faceIndex) const {
    assert(faceIndex < mHalfEdgeStructure.getNbFaces());
    return mHalfEdgeStructure.getFace(faceIndex);
}

// Return the number of vertices of the polyhedron
inline uint BoxShape::getNbVertices() const {
    return 8;
}

// Return a given vertex of the polyhedron
inline HalfEdgeStructure::Vertex BoxShape::getVertex(uint vertexIndex) const {
    assert(vertexIndex < getNbVertices());
    return mHalfEdgeStructure.getVertex(vertexIndex);
}

// Return the position of a given vertex
inline Vector3 BoxShape::getVertexPosition(uint vertexIndex) const {
    assert(vertexIndex < getNbVertices());

    Vector3 extent = getExtent();

    switch(vertexIndex) {
        case 0: return Vector3(-extent.x, -extent.y, extent.z);
        case 1: return Vector3(extent.x, -extent.y, extent.z);
        case 2: return Vector3(extent.x, extent.y, extent.z);
        case 3: return Vector3(-extent.x, extent.y, extent.z);
        case 4: return Vector3(-extent.x, -extent.y, -extent.z);
        case 5: return Vector3(extent.x, -extent.y, -extent.z);
        case 6: return Vector3(extent.x, extent.y, -extent.z);
        case 7: return Vector3(-extent.x, extent.y, -extent.z);
    }

    assert(false);
    return Vector3::zero();
}

// Return the normal vector of a given face of the polyhedron
inline Vector3 BoxShape::getFaceNormal(uint faceIndex) const {
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
inline Vector3 BoxShape::getCentroid() const {
    return Vector3::zero();
}

// Return the string representation of the shape
inline std::string BoxShape::to_string() const {
    return "BoxShape{extents=" + mExtent.to_string() + "}";
}

// Return the number of half-edges of the polyhedron
inline uint BoxShape::getNbHalfEdges() const {
    return 24;
}

// Return a given half-edge of the polyhedron
inline const HalfEdgeStructure::Edge& BoxShape::getHalfEdge(uint edgeIndex) const {
    assert(edgeIndex < getNbHalfEdges());
    return mHalfEdgeStructure.getHalfEdge(edgeIndex);
}

}

#endif
