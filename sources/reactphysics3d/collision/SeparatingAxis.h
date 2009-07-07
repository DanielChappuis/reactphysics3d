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

#ifndef SEPARATINGAXIS_H
#define SEPARATINGAXIS_H

// Libraries
#include "NarrowPhaseAlgorithm.h"
#include "../Constraint/Contact.h"
#include "../body/OBB.h"

// ReactPhysics3D namespace
namespace reactphysics3d {

class SeparatingAxis : public NarrowPhaseAlgorithm {
    private :

    public :
        SeparatingAxis();           // Constructor
        ~SeparatingAxis();          // Destructor

        virtual bool testCollision(const BoundingVolume& boundingVolume1, const BoundingVolume& boundingVolume2, Contact* const contact);    // Return true and compute a collision contact if the two bounding volume collide

};

} // End of the ReactPhysics3D namespace

#endif
