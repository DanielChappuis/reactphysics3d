/****************************************************************************
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

#ifndef NARROWPHASEALGORITHM_H
#define NARROWPHASEALGORITHM_H

// Libraries
#include "../body/BoundingVolume.h"
#include "../constraint/Contact.h"

// Namespace ReactPhysics3D
namespace reactphysics3d {

/*  -------------------------------------------------------------------
    Class NarrowPhaseAlgorithm :
        This class is an abstract class that represents an algorithm
        used to perform the narrow phase of a collision detection. The
        goal of the narrow phase algorithm is to compute contact
        informations of a collision between two bodies.
    -------------------------------------------------------------------
*/
class NarrowPhaseAlgorithm {
    private :

    public :
        NarrowPhaseAlgorithm();              // Constructor
        virtual ~NarrowPhaseAlgorithm();     // Destructor

        virtual bool testCollision(const BoundingVolume* const boundingVolume1, const BoundingVolume* const boundingVolume2, Contact** contact)=0;     // Return true and compute a collision contact if the two bounding volume collide
};

} // End of reactphysics3d namespace

#endif


