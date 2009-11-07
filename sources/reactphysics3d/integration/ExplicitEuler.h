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

#ifndef EXPLICITEULER_H
#define EXPLICITEULER_H

// Libraries
#include "IntegrationAlgorithm.h"
#include "../body/BodyState.h"

// TODO : Test explicit-Euler integrator (change has been made : computeForce(time) and computeTorque(time)
//        replaced by getForce() and getTorque().

// Namespace ReactPhysics3D
namespace reactphysics3d {

/*  -------------------------------------------------------------------
    Class ExplicitEuler :
        This class will be used to solve the differential equation of
        movement by integrating a body state. This class implements
        the explicit Euler algorithm. It's important to undersand that this
        algorithm should be used only for testing purpose because the
        explicit Euler algorithm can be unstable.
    -------------------------------------------------------------------
*/
class ExplicitEuler : public IntegrationAlgorithm {
    private :

    public :
        ExplicitEuler();                                        // Constructor
        ExplicitEuler(const ExplicitEuler& euler);              // Copy-constructor
        virtual ~ExplicitEuler();                               // Destructor

        void integrate(BodyState& bodyState, const Time& t, const Time& dt);    // Integrate a body state over time
};

}

#endif
