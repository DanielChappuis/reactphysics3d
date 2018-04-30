/********************************************************************************
* ReactPhysics3D physics library, http://www.reactphysics3d.com                 *
* Copyright (c) 2010-2018 Daniel Chappuis                                       *
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

        // Return a local support point in a given direction with the object margin
        Vector3 getLocalSupportPointWithMargin(const Vector3& direction) const;

        /// Return a local support point in a given direction without the object margin
        virtual Vector3 getLocalSupportPointWithoutMargin(const Vector3& direction) const=0;

    public :

        // -------------------- Methods -------------------- //

        /// Constructor
        ConvexShape(CollisionShapeName name, CollisionShapeType type, decimal margin = decimal(0.0));

        /// Destructor
        virtual ~ConvexShape() override = default;

        /// Deleted copy-constructor
        ConvexShape(const ConvexShape& shape) = delete;

        /// Deleted assignment operator
        ConvexShape& operator=(const ConvexShape& shape) = delete;

        /// Return the current object margin
        decimal getMargin() const;

        /// Return true if the collision shape is convex, false if it is concave
        virtual bool isConvex() const override;

        // -------------------- Friendship -------------------- //

        friend class GJKAlgorithm;
        friend class SATAlgorithm;
};

// Return true if the collision shape is convex, false if it is concave
inline bool ConvexShape::isConvex() const {
    return true;
}

// Return the current collision shape margin
/**
 * @return The margin (in meters) around the collision shape
 */
inline decimal ConvexShape::getMargin() const {
    return mMargin;
}

}

#endif

