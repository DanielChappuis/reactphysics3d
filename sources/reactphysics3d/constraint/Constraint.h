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

#ifndef CONSTRAINT_H
#define CONSTRAINT_H

// Libraries
#include "../body/Body.h"

// ReactPhysics3D namespace
namespace reactphysics3d {

/*  -------------------------------------------------------------------
    Class Constraint :
        This class represents a constraint in the physics engine.
        A constraint can be a collision contact or a joint for
        instance.
    -------------------------------------------------------------------
*/
class Constraint {
    private :
        Body& body1;            // Reference to the first body of the constraint
        Body& body2;            // Reference to the second body of the constraint

    public :
        Constraint(Body& body1, Body& body2);   // Constructor
        virtual ~Constraint();                  // Destructor

        Body& getBody1() const;                 // Return the reference to the body 1
        Body& getBody2() const;                 // Return the reference to the body 2
};

// Return the reference to the body 1
inline Body& Constraint::getBody1() const {
    return body1;
}

// Return the reference to the body 2
inline Body& Constraint::getBody2() const {
    return body2;
}

} // End of the ReactPhysics3D namespace

#endif
