/********************************************************************************
* ReactPhysics3D physics library, http://code.google.com/p/reactphysics3d/      *
* Copyright (c) 2010-2014 Daniel Chappuis                                       *
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

#ifndef REACTPHYSICS3D_RAY_H
#define REACTPHYSICS3D_RAY_H

// Libraries
#include "Vector3.h"

/// ReactPhysics3D namespace
namespace reactphysics3d {

// Class Ray
/**
 * This structure represents a 3D ray with a origin point and a direction.
 */
struct Ray {

    public:

        // -------------------- Attributes -------------------- //

        /// Origin point of the ray
        Vector3 origin;

        /// Direction vector of the ray
        Vector3 direction;

        // -------------------- Methods -------------------- //

        /// Constructor with arguments
        Ray(const Vector3& originPoint, const Vector3& directionVector);

        /// Copy-constructor
        Ray(const Ray& ray);

        /// Destructor
        ~Ray();

        /// Overloaded assignment operator
        Ray& operator=(const Ray& ray);
};

// Assignment operator
inline Ray& Ray::operator=(const Ray& ray) {
    if (&ray != this) {
        origin = ray.origin;
        direction = ray.direction;
    }
    return *this;
}

}

#endif
