/********************************************************************************
* ReactPhysics3D physics library, http://www.reactphysics3d.com                 *
* Copyright (c) 2010-2024 Daniel Chappuis                                       *
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
#include <cmath>
#include <reactphysics3d/configuration.h>

namespace reactphysics3d {

// Class Material
/**
 * This class contains the material properties of a collider that will be use for
 * the dynamics simulation like the friction coefficient or the bounciness of the rigid
 * body.
 */
class Material {

    private :

        // -------------------- Attributes -------------------- //

        /// Square root of the friction coefficient
        decimal mFrictionCoefficientSqrt;

        /// Bounciness during collisions (between 0 and 1) where 1 is for a very bouncy body
        decimal mBounciness;

        /// Density of mass used to compute the mass of the collider
        decimal mMassDensity;

        // -------------------- Methods -------------------- //

        /// Constructor
        Material(decimal frictionCoefficient, decimal bounciness, decimal massDensity = decimal(1.0));

    public :

        // -------------------- Methods -------------------- //

        /// Return the bounciness
        decimal getBounciness() const;

        /// Set the bounciness.
        void setBounciness(decimal bounciness);

        /// Return the friction coefficient
        decimal getFrictionCoefficient() const;

        /// Set the friction coefficient.
        void setFrictionCoefficient(decimal frictionCoefficient);

        /// Return the square root friction coefficient
        decimal getFrictionCoefficientSqrt() const;

        /// Return the mass density of the collider
        decimal getMassDensity() const;

        /// Set the mass density of the collider
        void setMassDensity(decimal massDensity);

        /// Return a string representation for the material
        std::string to_string() const;

        // ---------- Friendship ---------- //

        friend class Collider;
        friend class Body;
        friend class RigidBody;
};

// Return the bounciness
/**
 * @return Bounciness factor (between 0 and 1) where 1 is very bouncy
 */
RP3D_FORCE_INLINE decimal Material::getBounciness() const {
    return mBounciness;
}

// Set the bounciness.
/// The bounciness should be a value between 0 and 1. The value 1 is used for a
/// very bouncy body and zero is used for a body that is not bouncy at all.
/**
 * @param bounciness Bounciness factor (between 0 and 1) where 1 is very bouncy
 */
RP3D_FORCE_INLINE void Material::setBounciness(decimal bounciness) {
    assert(bounciness >= decimal(0.0) && bounciness <= decimal(1.0));
    mBounciness = bounciness;
}

// Return the friction coefficient
/**
 * @return Friction coefficient (positive value)
 */
RP3D_FORCE_INLINE decimal Material::getFrictionCoefficient() const {
    return mFrictionCoefficientSqrt * mFrictionCoefficientSqrt;
}

// Set the friction coefficient.
/// The friction coefficient has to be a positive value. The value zero is used for no
/// friction at all.
/**
 * @param frictionCoefficient Friction coefficient (positive value)
 */
RP3D_FORCE_INLINE void Material::setFrictionCoefficient(decimal frictionCoefficient) {
    assert(frictionCoefficient >= decimal(0.0));
    mFrictionCoefficientSqrt = std::sqrt(frictionCoefficient);
}

// Return the square root friction coefficient
RP3D_FORCE_INLINE decimal Material::getFrictionCoefficientSqrt() const {
    return mFrictionCoefficientSqrt;
}

// Return the mass density of the collider
RP3D_FORCE_INLINE decimal Material::getMassDensity() const {
   return mMassDensity;
}

// Set the mass density of the collider
/**
 * @param massDensity The mass density of the collider
 */
RP3D_FORCE_INLINE void Material::setMassDensity(decimal massDensity) {
   mMassDensity = massDensity;
}

// Return a string representation for the material
RP3D_FORCE_INLINE std::string Material::to_string() const {

    std::stringstream ss;

    ss << "frictionCoefficient=" << (mFrictionCoefficientSqrt * mFrictionCoefficientSqrt) << std::endl;
    ss << "bounciness=" << mBounciness << std::endl;

    return ss.str();
}

}

#endif
