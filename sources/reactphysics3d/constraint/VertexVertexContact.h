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

#ifndef VERTEXVERTEXCONTACT_H
#define VERTEXVERTEXCONTACT_H

// Libraries
#include "Contact.h"
#include "../mathematics/mathematics.h"

// ReactPhysics3D namespace
namespace reactphysics3d {

/*  -------------------------------------------------------------------
    Class VertexVertexContact :
        This class represents a vertex-vertex collision contact between two
        bodies in the physics engine.
    -------------------------------------------------------------------
*/
class VertexVertexContact : public Contact {
    private :
        Vector3D contactVertex;         // Contact vertex

    public :
        VertexVertexContact(Body* const body1, Body* const body2, const Vector3D& normalVector, const Vector3D& contactVertex);     // Constructor
        virtual ~VertexVertexContact();                                                                                 // Destructor

        Vector3D getContactVertex() const;      // Return the contact vertex

};

// Return the contact vertex
inline Vector3D VertexVertexContact::getContactVertex() const {
    return contactVertex;
}


} // End of the ReactPhysics3D namespace
#endif
