/****************************************************************************
* Copyright (C) 2009      Daniel Chappuis                                  *
****************************************************************************
* This file is part of ReactPhysics3D.                                     *
*                                                                          *
* ReactPhysics3D is free software: you can redistribute it and/or modify   *
* it under the terms of the GNU General Public License as published by     *
* the Free Software Foundation, either version 3 of the License, or        *
* (at your option) any later version.                                      *
*                                                                          *
* ReactPhysics3D is distributed in the hope that it will be useful,        *
* but WITHOUT ANY WARRANTY; without even the implied warranty of           *
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the             *
* GNU General Public License for more details.                             *
*                                                                          *
* You should have received a copy of the GNU General Public License        *
* along with ReactPhysics3D. If not, see <http://www.gnu.org/licenses/>.   *
***************************************************************************/

#ifndef WORLD_H
#define WORLD_H

// Libraries
#include <vector>
#include <stdexcept>
#include "../mathematics/mathematics.h"
#include "../body/Body.h"

// Namespace reactphysics3d
namespace reactphysics3d {

/*  -------------------------------------------------------------------
    Class PhysicsWorld :
        This is an abstract class that represents the world of the
        physics engine. A physics world contains all the bodies of the physics
        engine.
    -------------------------------------------------------------------
*/
class PhysicsWorld {
    private :
        PhysicsWorld(const PhysicsWorld&);          // Copy-constructor is private because we don't want it to be used

    protected :
        std::vector<Body*> bodyList;                // list that contains all bodies of the physics world
        Vector3D gravity;                           // Gravity vector of the world

    public :
        PhysicsWorld(const Vector3D& gravity);      // Constructor
        virtual ~PhysicsWorld();                    // Destructor

        void addBody(Body* body) throw(std::invalid_argument);                  // Add a body to the physics world
        void removeBody(Body const* const body) throw(std::invalid_argument);   // Remove a body from the physics world
};

}   // End of the ReactPhysics3D namespace

 #endif
