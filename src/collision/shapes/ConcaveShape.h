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

#ifndef REACTPHYSICS3D_CONCAVE_SHAPE_H
#define REACTPHYSICS3D_CONCAVE_SHAPE_H

// Libraries
#include "CollisionShape.h"

/// ReactPhysics3D namespace
namespace reactphysics3d {

// Class ConcaveShape
/**
 * This abstract class represents a concave collision shape associated with a
 * body that is used during the narrow-phase collision detection.
 */
class ConcaveShape : public CollisionShape {

    protected :

        // -------------------- Attributes -------------------- //



        // -------------------- Methods -------------------- //

        /// Private copy-constructor
        ConcaveShape(const ConcaveShape& shape);

        /// Private assignment operator
        ConcaveShape& operator=(const ConcaveShape& shape);

    public :

        // -------------------- Methods -------------------- //

        /// Constructor
        ConcaveShape(CollisionShapeType type);

        /// Destructor
        virtual ~ConcaveShape();

        /// Return true if the collision shape is convex, false if it is concave
        virtual bool isConvex() const;

        /// Test equality between two shapes
        virtual bool isEqualTo(const CollisionShape& otherCollisionShape) const;
};

/// Return true if the collision shape is convex, false if it is concave
inline bool ConcaveShape::isConvex() const {
    return false;
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

// Equality operator between two collision shapes.
/// This methods returns true only if the two collision shapes are of the same type and
/// of the same dimensions.
inline bool CollisionShape::operator==(const CollisionShape& otherCollisionShape) const {

    // If the two collisions shapes are not of the same type (same derived classes)
    // we return false
    if (mType != otherCollisionShape.mType) return false;

    assert(typeid(*this) == typeid(otherCollisionShape));

    if (mMargin != otherCollisionShape.mMargin) return false;

    // Check if the two shapes are equal
    return otherCollisionShape.isEqualTo(*this);
}

// Test equality between two shapes
inline bool ConcaveShape::isEqualTo(const CollisionShape& otherCollisionShape) const {
    return true;
}

}

#endif

