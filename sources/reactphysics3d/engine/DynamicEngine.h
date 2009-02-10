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

#ifndef DYNAMICENGINE_H
#define DYNAMICENGINE_H

// Libraries
#include "PhysicsEngine.h"
#include "NumericalIntegrator.h"
#include "../body/Body.h"
#include "../body/RigidBody.h"
#include "../body/BodyState.h"

// Namespace ReactPhysics3D
namespace reactphysics3d {

/*  -------------------------------------------------------------------
    Class DynamicEngine :
        This class that represents a physics engine where we use
        the dynamics to simulate the movement of bodies. The class
        DynamicEngine inherits from the class PhysicsEngine.
    -------------------------------------------------------------------
*/
class DynamicEngine : public PhysicsEngine {
    protected :
        NumericalIntegrator numericalIntegrator;        // Numerical integrator used to solve differential equations of movement

        BodyState interpolateState(const BodyState& previousBodyState, const BodyState& currentBodyState) const;    // Compute the interpolation state
        void updateBodyState(RigidBody* const rigidBody);                                                           // Update the state of a rigid body

    public :
        DynamicEngine(DynamicsWorld& world, const Time& timeStep);      // Constructor
        DynamicEngine(const DynamicEngine& engine);                     // Copy-constructor
        virtual ~DynamicEngine();                                       // Destructor

        void update();                                  // Update the physics simulation
};

}

#endif
