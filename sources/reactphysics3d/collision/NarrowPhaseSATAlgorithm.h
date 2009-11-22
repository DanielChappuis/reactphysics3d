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

// Enumeration for the contact type
enum ContactType {
    EDGE_EDGE,              // Contact between an edge of OBB1 and an edge of OBB2
    FACEOBB1_SOMETHING,     // Contact between a face of OBB1 and eiter a vertex, an edge or a face of OBB2
    FACEOBB2_SOMETHING      // Contact between a face of OBB2 and either a vertex, an edge or a face of OBB1
}

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
        bool computeCollisionTest(const OBB* const obb1, const OBB* const obb2, Contact** contact);                         // Return true and compute a collision contact if the two OBB collide
        double computePenetrationDepth(double min1, double max1, double min2, double max2, double& minPenetrationDepth);    // Compute the penetration depth of two projection intervals
        void computeContact(const OBB* const obb1, const OBB* const obb2, const Vector3D normal, double penetrationDepth,
                            ContactType contactType,Contact** contact);                                                     // Compute a new collision contact between two projection intervals

    public :
        NarrowPhaseSATAlgorithm();           // Constructor
        ~NarrowPhaseSATAlgorithm();          // Destructor

        virtual bool testCollision(const BoundingVolume* const boundingVolume1, const BoundingVolume* const boundingVolume2, Contact** contact);      // Return true and compute a collision contact if the two bounding volume collide
};

} // End of the ReactPhysics3D namespace

#endif
