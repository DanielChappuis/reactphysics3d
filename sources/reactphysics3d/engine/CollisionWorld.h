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

#ifndef PHYSICSWORLD_H
#define PHYSICSWORLD_H

// Libraries
#include <vector>
#include <stdexcept>
#include "../constraint/Constraint.h"
#include "../mathematics/mathematics.h"

// ReactPhysics3D namespace
namespace reactphysics3d {

/*  -------------------------------------------------------------------
    Class PhysicsWorld :
        This class represent a physics world. The physics world contains
        some bodies.
    -------------------------------------------------------------------
*/
class PhysicsWorld {
    private :
        Vector3D gravity;                           // Gravity vector of the physics world
        std::vector<Constraint*> constraintList;    // List that contains all the current constraints

    public :
        PhysicsWorld(const Vector3D& gravity);        // Constructor
        virtual ~PhysicsWorld();                              // Destructor

        void addConstraint(Constraint* constraint) throw(std::invalid_argument);            // Add a constraint
        void removeConstraint(Constraint* constraint) throw(std::invalid_argument);         // Remove a constraint
        void removeAllContactConstraints();                                                 // Remove all collision contacts constraints
        std::vector<Constraint*>::const_iterator getConstraintListStartIterator() const;    // Return a start iterator on the constraint list
        std::vector<Constraint*>::const_iterator getConstraintListEndIterator() const;      // Return a end iterator on the constraint list
};

// Return a start iterator on the constraint list
inline std::vector<Constraint*>::const_iterator PhysicsWorld::getConstraintListStartIterator() const {
    return constraintList.begin();
}

// Return a end iterator on the constraint list
inline std::vector<Constraint*>::const_iterator PhysicsWorld::getConstraintListEndIterator() const {
    return constraintList.end();
}


} // End of the ReactPhysics3D namespace

#endif
