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

#ifndef REACTPHYSICS3D_CONCAVE_SHAPE_H
#define REACTPHYSICS3D_CONCAVE_SHAPE_H

// Libraries
#include "CollisionShape.h"
#include "TriangleShape.h"

// ReactPhysics3D namespace
namespace reactphysics3d {

// Class TriangleCallback
/**
 * This class is used to encapsulate a callback method for
 * a single triangle of a ConcaveMesh.
 */
class TriangleCallback {

    public:

        /// Report a triangle
        virtual void testTriangle(const Vector3* trianglePoints)=0;

};


// Class ConcaveShape
/**
 * This abstract class represents a concave collision shape associated with a
 * body that is used during the narrow-phase collision detection.
 */
class ConcaveShape : public CollisionShape {

    protected :

        // -------------------- Attributes -------------------- //

        /// True if the smooth mesh collision algorithm is enabled
        bool mIsSmoothMeshCollisionEnabled;

        // Margin use for collision detection for each triangle
        decimal mTriangleMargin;

        /// Raycast test type for the triangle (front, back, front-back)
        TriangleRaycastSide mRaycastTestType;

        // -------------------- Methods -------------------- //

        /// Private copy-constructor
        ConcaveShape(const ConcaveShape& shape);

        /// Private assignment operator
        ConcaveShape& operator=(const ConcaveShape& shape);

        /// Return true if a point is inside the collision shape
        virtual bool testPointInside(const Vector3& localPoint, ProxyShape* proxyShape) const;

    public :

        // -------------------- Methods -------------------- //

        /// Constructor
        ConcaveShape(CollisionShapeType type);

        /// Destructor
        virtual ~ConcaveShape();

        /// Return the triangle margin
        decimal getTriangleMargin() const;

        /// Return the raycast test type (front, back, front-back)
        TriangleRaycastSide getRaycastTestType() const;

        // Set the raycast test type (front, back, front-back)
        void setRaycastTestType(TriangleRaycastSide testType);

        /// Return true if the collision shape is convex, false if it is concave
        virtual bool isConvex() const;

        /// Use a callback method on all triangles of the concave shape inside a given AABB
        virtual void testAllTriangles(TriangleCallback& callback, const AABB& localAABB) const=0;

        /// Return true if the smooth mesh collision is enabled
        bool getIsSmoothMeshCollisionEnabled() const;

        /// Enable/disable the smooth mesh collision algorithm
        void setIsSmoothMeshCollisionEnabled(bool isEnabled);
};

// Return the triangle margin
inline decimal ConcaveShape::getTriangleMargin() const {
    return mTriangleMargin;
}

/// Return true if the collision shape is convex, false if it is concave
inline bool ConcaveShape::isConvex() const {
    return false;
}

// Return true if a point is inside the collision shape
inline bool ConcaveShape::testPointInside(const Vector3& localPoint, ProxyShape* proxyShape) const {
    return false;
}

// Return true if the smooth mesh collision is enabled
inline bool ConcaveShape::getIsSmoothMeshCollisionEnabled() const {
    return mIsSmoothMeshCollisionEnabled;
}

// Enable/disable the smooth mesh collision algorithm
/// Smooth mesh collision is used to avoid collisions against some internal edges
/// of the triangle mesh. If it is enabled, collsions with the mesh will be smoother
/// but collisions computation is a bit more expensive.
inline void ConcaveShape::setIsSmoothMeshCollisionEnabled(bool isEnabled) {
    mIsSmoothMeshCollisionEnabled = isEnabled;
}

// Return the raycast test type (front, back, front-back)
inline TriangleRaycastSide ConcaveShape::getRaycastTestType() const {
    return mRaycastTestType;
}

// Set the raycast test type (front, back, front-back)
/**
 * @param testType Raycast test type for the triangle (front, back, front-back)
 */
inline void ConcaveShape::setRaycastTestType(TriangleRaycastSide testType) {
    mRaycastTestType = testType;
}

}

#endif

