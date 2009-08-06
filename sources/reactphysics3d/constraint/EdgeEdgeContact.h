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

#ifndef EDGEEDGECONTACT_H
#define EDGEEDGECONTACT_H

// Libraries
#include "Contact.h"
#include "../mathematics/mathematics.h"

// ReactPhysics3D namespace
namespace reactphysics3d {

/*  -------------------------------------------------------------------
    Class EdgeEdgeContact :
        This class represents a edge-edge collision contact between two
        bodies in the physics engine.
    -------------------------------------------------------------------
*/
class EdgeEdgeContact : public Contact {
    private :
        Segment3D contactSegment;               // Contact segment

    public :
        EdgeEdgeContact(Body* const body1, Body* const body2, const Vector3D normalVector, const Time& time, const Segment3D& contactSegment);  // Constructor
        virtual ~EdgeEdgeContact();                                                                                                             // Destructor

        Segment3D getContactSegment() const;        // Return the contact segment
};

// Return the contact segment
inline Segment3D EdgeEdgeContact::getContactSegment() const {
    return contactSegment;
}

} // End of the ReactPhysics3D namespace

#endif

