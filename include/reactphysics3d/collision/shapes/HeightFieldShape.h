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

#ifndef REACTPHYSICS3D_HEIGHTFIELD_SHAPE_H
#define REACTPHYSICS3D_HEIGHTFIELD_SHAPE_H

// Libraries
#include <reactphysics3d/collision/shapes/ConcaveShape.h>
#include <reactphysics3d/collision/HeightField.h>
#include <reactphysics3d/collision/shapes/AABB.h>

namespace reactphysics3d {

class HeightFieldShape;
class Profiler;
class TriangleShape;

// Class HeightFieldShape
/**
 * This class represents a static height field that can be used to represent
 * a terrain. The height field is made of a grid with rows and columns with a
 * height value at each grid point. Note that the height values are not copied into the shape
 * but are shared instead. The height values can be of type integer, float or double.
 * When creating a HeightFieldShape, you need to specify the minimum and maximum height value of
 * your height field. Note that the HeightFieldShape will be re-centered based on its AABB. It means
 * that for instance, if the minimum height value is -200 and the maximum value is 400, the final
 * minimum height of the field in the simulation will be -300 and the maximum height will be 300.
 */
class HeightFieldShape : public ConcaveShape {

    public:

    protected:

        // -------------------- Attributes -------------------- //

        /// Height-field
        HeightField* mHeightField;

        // -------------------- Methods -------------------- //

        /// Constructor
        HeightFieldShape(HeightField* heightfield, MemoryAllocator& allocator, const Vector3& scaling = Vector3(1,1,1));

        /// Raycast method with feedback information
        virtual bool raycast(const Ray& ray, RaycastInfo& raycastInfo, Collider* collider, MemoryAllocator& allocator) const override;

        /// Return the number of bytes used by the collision shape
        virtual size_t getSizeInBytes() const override;

        /// Destructor
        virtual ~HeightFieldShape() override = default;

    public:

        /// Deleted copy-constructor
        HeightFieldShape(const HeightFieldShape& shape) = delete;

        /// Deleted assignment operator
        HeightFieldShape& operator=(const HeightFieldShape& shape) = delete;

        /// Return a pointer to the internal height-field
        HeightField* getHeightField() const;

        /// Return the vertex (local-coordinates) of the height field at a given (x,y) position
        Vector3 getVertexAt(uint32 x, uint32 y) const;

        /// Return the local bounds of the shape in x, y and z directions.
        virtual AABB getLocalBounds() const override;

        /// Use a callback method on all triangles of the concave shape inside a given AABB
        virtual void computeOverlappingTriangles(const AABB& localAABB, Array<Vector3>& triangleVertices,
                                                   Array<Vector3>& triangleVerticesNormals, Array<uint32>& shapeIds,
                                                   MemoryAllocator& allocator) const override;

        /// Return the string representation of the shape
        virtual std::string to_string() const override;

        // ---------- Friendship ----------- //

        friend class ConvexTriangleAABBOverlapCallback;
        friend class ConcaveMeshRaycastCallback;
        friend class PhysicsCommon;
};

// Return a pointer to the internal height-field
RP3D_FORCE_INLINE HeightField* HeightFieldShape::getHeightField() const {
    return mHeightField;
}

// Return the number of bytes used by the collision shape
RP3D_FORCE_INLINE size_t HeightFieldShape::getSizeInBytes() const {
    return sizeof(HeightFieldShape);
}

// Return the vertex (local-coordinates) of the height field at a given (x,y) position
RP3D_FORCE_INLINE Vector3 HeightFieldShape::getVertexAt(uint32 x, uint32 y) const {
    return mHeightField->getVertexAt(x, y) * mScale;
}

}
#endif

