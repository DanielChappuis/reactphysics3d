/********************************************************************************
* ReactPhysics3D physics library, http://www.reactphysics3d.com                 *
* Copyright (c) 2010-2015 Daniel Chappuis                                       *
*********************************************************************************
*                                                                               *
* This software is provided 'as-is', without any express or implied warranty.   *
* In no event will the authors be held liable for any damages arising from the  *
* use of this software.                                                         *
*                                                                               *
* Permission is granted to anyone to use this software for any purpose,         *
* including commercial applications, and to alter it and redistribute it        *
* freely, subject to the following restrictions:                                *
*                                                                               *
* 1. The origin of this software must not be misrepresented; you must not claim *
*    that you wrote the original software. If you use this software in a        *
*    product, an acknowledgment in the product documentation would be           *
*    appreciated but is not required.                                           *
*                                                                               *
* 2. Altered source versions must be plainly marked as such, and must not be    *
*    misrepresented as being the original software.                             *
*                                                                               *
* 3. This notice may not be removed or altered from any source distribution.    *
*                                                                               *
********************************************************************************/

#ifndef REACTPHYSICS3D_MATERIAL_H
#define REACTPHYSICS3D_MATERIAL_H

// Libraries
#include <cassert>
#include "configuration.h"

namespace reactphysics3d {

// Class Material
/**
 * This class contains the material properties of a rigid body that will be use for
 * the dynamics simulation like the friction coefficient or the bounciness of the rigid
 * body.
 */
class Material {

    private :

        // -------------------- Attributes -------------------- //

        /// Friction coefficient (positive value)
        decimal mFrictionCoefficient;

        /// Bounciness during collisions (between 0 and 1) where 1 is for a very bouncy body
        decimal mBounciness;

    public :

        // -------------------- Methods -------------------- //

        /// Constructor
        Material();

        /// Copy-constructor
        Material(const Material& material);

        /// Destructor
        ~Material();

        /// Return the bounciness
        decimal getBounciness() const;

        /// Set the bounciness.
        void setBounciness(decimal bounciness);

        /// Return the friction coefficient
        decimal getFrictionCoefficient() const;

        /// Set the friction coefficient.
        void setFrictionCoefficient(decimal frictionCoefficient);

        /// Overloaded assignment operator
        Material& operator=(const Material& material);
};

// Return the bounciness
/**
 * @return Bounciness factor (between 0 and 1) where 1 is very bouncy
 */
inline decimal Material::getBounciness() const {
    return mBounciness;
}

// Set the bounciness.
/// The bounciness should be a value between 0 and 1. The value 1 is used for a
/// very bouncy body and zero is used for a body that is not bouncy at all.
/**
 * @param bounciness Bounciness factor (between 0 and 1) where 1 is very bouncy
 */
inline void Material::setBounciness(decimal bounciness) {
    assert(bounciness >= decimal(0.0) && bounciness <= decimal(1.0));
    mBounciness = bounciness;
}

// Return the friction coefficient
/**
 * @return Friction coefficient (positive value)
 */
inline decimal Material::getFrictionCoefficient() const {
    return mFrictionCoefficient;
}

// Set the friction coefficient.
/// The friction coefficient has to be a positive value. The value zero is used for no
/// friction at all.
/**
 * @param frictionCoefficient Friction coefficient (positive value)
 */
inline void Material::setFrictionCoefficient(decimal frictionCoefficient) {
    assert(frictionCoefficient >= decimal(0.0));
    mFrictionCoefficient = frictionCoefficient;
}

// Overloaded assignment operator
inline Material& Material::operator=(const Material& material) {

    // Check for self-assignment
    if (this != &material) {
        mFrictionCoefficient = material.mFrictionCoefficient;
        mBounciness = material.mBounciness;
    }

    // Return this material
    return *this;
}

}

#endif
