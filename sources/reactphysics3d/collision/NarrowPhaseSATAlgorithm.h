/***************************************************************************
* Copyright (C) 2009      Daniel Chappuis                                  *
****************************************************************************
* This file is part of ReactPhysics3D.                                     *
*                                                                          *
* ReactPhysics3D is free software: you can redistribute it and/or modify   *
* it under the terms of the GNU Lesser General Public License as published *
* by the Free Software Foundation, either version 3 of the License, or     *
* (at your option) any later version.                                      *
*                                                                          *
* ReactPhysics3D is distributed in the hope that it will be useful,        *
* but WITHOUT ANY WARRANTY; without even the implied warranty of           *
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the             *
* GNU Lesser General Public License for more details.                      *
*                                                                          *
* You should have received a copy of the GNU Lesser General Public License *
* along with ReactPhysics3D. If not, see <http://www.gnu.org/licenses/>.   *
***************************************************************************/

#ifndef NARROWPHASESATALGORITHM_H
#define NARROWPHASESATALGORITHM_H

// Libraries
#include "NarrowPhaseAlgorithm.h"
#include "../constraint/Contact.h"
#include "../body/OBB.h"

// ReactPhysics3D namespace
namespace reactphysics3d {

/*  -------------------------------------------------------------------
    Class NarrowPhaseSATAlgorithm :
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
class NarrowPhaseSATAlgorithm : public NarrowPhaseAlgorithm {
    private :
        bool computeCollisionTest(const OBB* const obb1, const OBB* const obb2, Contact** contact) const;                                               // Return true and compute a collision contact if the two OBB collide
        double computePenetrationDepth(double min1, double max1, double min2, double max2, bool& side);                                                 // Compute the penetration depth of two projection intervals
        void computeContact(const OBB* const obb1, const OBB* const obb2, const Vector3D normal, double penetrationDepth,
                            const std::vector<Vector3D>& obb1Extremepoints, const std::vector<Vector3D>& obb2ExtremePoints, Contact** contact) const;   // Compute a new contact                                                   // Compute a new collision contact between two projection intervals
        Vector3D computeContactNormal(const Vector3D& axis, const Vector3D& distanceOfOBBs) const;                                                      // Compute a contact normal
    public :
        NarrowPhaseSATAlgorithm();           // Constructor
        ~NarrowPhaseSATAlgorithm();          // Destructor

        virtual bool testCollision(const BoundingVolume* const boundingVolume1, const BoundingVolume* const boundingVolume2, Contact** contact);      // Return true and compute a collision contact if the two bounding volume collide
};

// --- Inlines function --- //

// Return the contact normal with the correct sign (from obb1 toward obb2). "axis" is the axis vector direction where the
// collision occur and "distanceOfOBBs" is the vector (obb2.center - obb1.center).
inline Vector3D NarrowPhaseAlgorithm::computeContactNormal(const Vector3D& axis, const Vector3D& distanceOfOBBs) const {
    if (distanceOfOBBs.scalarProduct(axis) >= 0) {
        return axis;
    }
    else {
        return axis.getOpposite();
    }
}

} // End of the ReactPhysics3D namespace

#endif
