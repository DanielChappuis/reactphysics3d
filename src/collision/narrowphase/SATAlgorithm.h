/********************************************************************************
* ReactPhysics3D physics library, http://code.google.com/p/reactphysics3d/      *
* Copyright (c) 2010-2012 Daniel Chappuis                                       *
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

#ifndef SAT_ALGORITHM_H
#define SAT_ALGORITHM_H

// Libraries
#include "NarrowPhaseAlgorithm.h"
#include "../../constraint/Contact.h"
#include "../../shapes/BoxShape.h"
#include "../../mathematics/Transform.h"

// ReactPhysics3D namespace
namespace reactphysics3d {

/*  -------------------------------------------------------------------
    Class SATAlgorithm :
        This class implements a narrow-phase algorithm. This algorithm
        uses a separating axis theorem (SAT) to check if two bounding
        volumes collide or not. If the
        two bounding volumes collide we have to create a contact object
        to describe the collision contact. The idea is to check if there
        exists an axis where, if we project the two bounding volumes on
        this axis, the two projections are separated. If we find at
        least an axis where the projections of the two bounding volumes
        are separated then we know that the two bounding volumes don't
        intersect.
    -------------------------------------------------------------------
*/
class SATAlgorithm : public NarrowPhaseAlgorithm {
    private :
        bool computeCollisionTest(const BoxShape* obb1, const Transform& transform1,
                                  const BoxShape* obb2, const Transform& transform2,
                                  ContactInfo*& contactInfo) const;     // Return true and compute a contact info if the two OBB collide
        double computePenetrationDepth(double min1, double max1, double min2, double max2) const;                     // Compute the penetration depth of two projection intervals                             
        Vector3 computeContactNormal(const Vector3& axis, const Vector3& distanceOfOBBs) const;                    // Compute a contact normal

    public :
        SATAlgorithm();           // Constructor
        ~SATAlgorithm();          // Destructor

        virtual bool testCollision(const Body* body1, const Body* body2, ContactInfo*& contactInfo);                   // Return true and compute a contact info if the two bounding volume collide
};

// Return the contact normal with the correct sign (from obb1 toward obb2). "axis" is the axis vector direction where the
// collision occurs and "distanceOfOBBs" is the vector (obb2.center - obb1.center).
inline Vector3 SATAlgorithm::computeContactNormal(const Vector3& axis, const Vector3& distanceOfOBBs) const {
    if (distanceOfOBBs.dot(axis) >= 0.0) {
        return axis;
    }
    else {
        return -axis;
    }
}

} // End of the ReactPhysics3D namespace

#endif
