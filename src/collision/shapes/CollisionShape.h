/********************************************************************************
* ReactPhysics3D physics library, http://code.google.com/p/reactphysics3d/      *
* Copyright (c) 2010-2013 Daniel Chappuis                                       *
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

#ifndef COLLISION_SHAPE_H
#define COLLISION_SHAPE_H

// Libraries
#include <cassert>
#include "../../mathematics/Vector3.h"
#include "../../mathematics/Matrix3x3.h"

/// ReactPhysics3D namespace
namespace reactphysics3d {
    
/// Type of the collision shape
enum CollisionShapeType {BOX, SPHERE, CONE, CYLINDER};

// Declarations
class Body;

// Class CollisionShape
/**
 * This abstract class represents the collision shape associated with a
 * body that is used during the narrow-phase collision detection.
 */
class CollisionShape {
        
    protected :

        // -------------------- Attributes -------------------- //

        /// Type of the collision shape
        CollisionShapeType mType;
        
        // -------------------- Methods -------------------- //

        /// Private copy-constructor
        CollisionShape(const CollisionShape& shape);

        /// Private assignment operator
        CollisionShape& operator=(const CollisionShape& shape);

    public :

        // -------------------- Methods -------------------- //

        /// Constructor
        CollisionShape(CollisionShapeType type);

        /// Destructor
        virtual ~CollisionShape();

        /// Return the type of the collision shapes
        CollisionShapeType getType() const;

        /// Return a local support point in a given direction with the object margin
        virtual Vector3 getLocalSupportPointWithMargin(const Vector3& direction) const=0;

        /// Return a local support point in a given direction without the object margin
        virtual Vector3 getLocalSupportPointWithoutMargin(const Vector3& direction) const=0;

        /// Return the local extents in x,y and z direction
        virtual Vector3 getLocalExtents(decimal margin=0.0) const=0;

        /// Return the margin distance around the shape
        virtual decimal getMargin() const=0;

        /// Return the local inertia tensor of the collision shapes
        virtual void computeLocalInertiaTensor(Matrix3x3& tensor, decimal mass) const=0;
};

// Return the type of the collision shape
inline CollisionShapeType CollisionShape::getType() const {
    return mType;
}                                                                  

}

#endif
