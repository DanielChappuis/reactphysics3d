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

#ifndef REACTPHYSICS3D_CONVEX_SHAPE_H
#define REACTPHYSICS3D_CONVEX_SHAPE_H

// Libraries
#include "CollisionShape.h"

/// ReactPhysics3D namespace
namespace reactphysics3d {

// Class ConvexShape
/**
 * This abstract class represents a convex collision shape associated with a
 * body that is used during the narrow-phase collision detection.
 */
class ConvexShape : public CollisionShape {

    protected :

        // -------------------- Attributes -------------------- //

        /// Margin used for the GJK collision detection algorithm
        decimal mMargin;

        // -------------------- Methods -------------------- //

        /// Private copy-constructor
        ConvexShape(const ConvexShape& shape);

        /// Private assignment operator
        ConvexShape& operator=(const ConvexShape& shape);

        // Return a local support point in a given direction with the object margin
        virtual Vector3 getLocalSupportPointWithMargin(const Vector3& direction,
                                                       void** cachedCollisionData) const=0;

        /// Return a local support point in a given direction without the object margin
        virtual Vector3 getLocalSupportPointWithoutMargin(const Vector3& direction,
                                                          void** cachedCollisionData) const=0;

        /// Return true if a point is inside the collision shape
        virtual bool testPointInside(const Vector3& worldPoint, ProxyShape* proxyShape) const=0;

    public :

        // -------------------- Methods -------------------- //

        /// Constructor
        ConvexShape(CollisionShapeType type, decimal margin);

        /// Destructor
        virtual ~ConvexShape();

        /// Return the current object margin
        decimal getMargin() const;

        /// Return true if the collision shape is convex, false if it is concave
        virtual bool isConvex() const;

        /// Test equality between two shapes
        virtual bool isEqualTo(const CollisionShape& otherCollisionShape) const;
};

/// Return true if the collision shape is convex, false if it is concave
inline bool ConvexShape::isConvex() const {
    return true;
}

// Return the current collision shape margin
/**
 * @return The margin (in meters) around the collision shape
 */
inline decimal CollisionShape::getMargin() const {
    return mMargin;
}

// Return the type of the collision shape
/**
 * @return The type of the collision shape (box, sphere, cylinder, ...)
 */
inline CollisionShapeType CollisionShape::getType() const {
    return mType;
}

// Return the number of similar created shapes
inline uint CollisionShape::getNbSimilarCreatedShapes() const {
    return mNbSimilarCreatedShapes;
}

// Return the current collision shape margin
/**
 * @return The margin (in meters) around the collision shape
 */
inline decimal CollisionShape::getMargin() const {
    return mMargin;
}

// Increment the number of similar allocated collision shapes
inline void CollisionShape::incrementNbSimilarCreatedShapes() {
    mNbSimilarCreatedShapes++;
}

// Decrement the number of similar allocated collision shapes
inline void CollisionShape::decrementNbSimilarCreatedShapes() {
    mNbSimilarCreatedShapes--;
}

// Test equality between two shapes
inline bool ConvexShape::isEqualTo(const CollisionShape& otherCollisionShape) const {

    const ConvexShape& otherShape = static_cast<const ConvexShape&>(otherCollisionShape);
    return (mMargin == otherShape.mMargin);
}

}

#endif

