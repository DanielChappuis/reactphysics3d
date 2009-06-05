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

#ifndef BODY_H
#define BODY_H

 // Libraries
#include <stdexcept>
#include "../physics/physics.h"

// Namespace reactphysics3d
namespace reactphysics3d {

/*  -------------------------------------------------------------------
    Class Body :
        This class is an abstract class to represent body of the physics
        engine.
    -------------------------------------------------------------------
*/
class Body {
    private :
        Kilogram mass;                      // Mass of the body

    public :
        Body(Kilogram mass) throw(std::invalid_argument);   // Constructor
        Body(const Body& body);                             // Copy-constructor
        virtual ~Body();                                    // Destructor

        Kilogram getMass();                 // Return the mass of the body
        void setMass(Kilogram mass);        // Set the mass of the body
};

// --- Inlines function --- //

// Method that return the mass of the body
inline Kilogram Body::getMass() {
    return mass;
};

// Method that set the mass of the body
inline void Body::setMass(Kilogram mass) {
    this->mass = mass;
}

}

 #endif
