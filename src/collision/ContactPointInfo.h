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

#ifndef REACTPHYSICS3D_CONTACT_POINT_INFO_H
#define REACTPHYSICS3D_CONTACT_POINT_INFO_H

// Libraries
#include "mathematics/mathematics.h"
#include "configuration.h"

/// ReactPhysics3D namespace
namespace reactphysics3d {

// Declarations
class CollisionBody;

// Structure ContactPointInfo
/**
 * This structure contains informations about a collision contact
 * computed during the narrow-phase collision detection. Those
 * informations are used to compute the contact set for a contact
 * between two bodies.
 */
struct ContactPointInfo {

    private:

        // -------------------- Methods -------------------- //

    public:

        // -------------------- Attributes -------------------- //

        /// Normalized normal vector of the collision contact in world space
        Vector3 normal;

        /// Penetration depth of the contact
        decimal penetrationDepth;

        /// Contact point of body 1 in local space of body 1
        Vector3 localPoint1;

        /// Contact point of body 2 in local space of body 2
        Vector3 localPoint2;

        /// Pointer to the next contact point info
        ContactPointInfo* next;

        /// True if the contact point has already been inserted into a manifold
        bool isUsed;

        // -------------------- Methods -------------------- //

        /// Constructor
        ContactPointInfo(const Vector3& contactNormal, decimal penDepth,
                         const Vector3& localPt1, const Vector3& localPt2)
                         : normal(contactNormal), penetrationDepth(penDepth),
                           localPoint1(localPt1), localPoint2(localPt2), next(nullptr), isUsed(false) {

            assert(contactNormal.lengthSquare() > decimal(0.8));
            assert(penDepth > decimal(0.0));
        }

        /// Destructor
        ~ContactPointInfo() = default;
};

}

#endif
