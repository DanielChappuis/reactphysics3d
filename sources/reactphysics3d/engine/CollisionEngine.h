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

#ifndef COLLISIONENGINE_H
#define COLLISIONENGINE_H

// Libraries
#include "DynamicEngine.h"
#include "../collision/CollisionDetection.h"

// ReactPhysics3D namespace
namespace reactphysics3d {

/*  -------------------------------------------------------------------
    Class CollisionEngine :
        This class implements the physics engine where bodies can
        collide. This class inherits from the class DynamicEngine.
    -------------------------------------------------------------------
*/
class CollisionEngine : public DynamicEngine {
    private :
        CollisionDetection collisionDetection;          // Collision detection
        // ConstraintSolver constraintSolver;              // Constraint solver

    public :
        CollisionEngine(CollisionWorld* world, const Time& timeStep);       // Constructor
        virtual ~CollisionEngine();                                         // Destructor

        virtual void update();                           // Update the physics simulation
};

} // End of the ReactPhysics3D namespace

#endif
