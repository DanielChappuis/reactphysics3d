/********************************************************************************
* ReactPhysics3D physics library, http://www.reactphysics3d.com                 *
* Copyright (c) 2010-2020 Daniel Chappuis                                       *
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
#include <reactphysics3d/mathematics/Vector3.h>

/// ReactPhysics3D namespace
namespace reactphysics3d {

// Class Ray
/**
 * This structure represents a 3D ray represented by two points.
 * The ray goes from point1 to point1 + maxFraction * (point2 - point1).
 * The points are specified in world-space coordinates.
 */
struct Ray {

    public:

        // -------------------- Attributes -------------------- //

        /// First point of the ray (origin) in world-space
        Vector3 point1;

        /// Second point of the ray in world-space
        Vector3 point2;

        /// Maximum fraction value
        decimal maxFraction;

        // -------------------- Methods -------------------- //

        /// Constructor with arguments
        Ray(const Vector3& p1, const Vector3& p2, decimal maxFrac = decimal(1.0))
           : point1(p1), point2(p2), maxFraction(maxFrac) {

        }

        /// Copy-constructor
        Ray(const Ray& ray) : point1(ray.point1), point2(ray.point2), maxFraction(ray.maxFraction) {

        }

        /// Destructor
        ~Ray() = default;

        /// Overloaded assignment operator
        Ray& operator=(const Ray& ray) {
            if (&ray != this) {
                point1 = ray.point1;
                point2 = ray.point2;
                maxFraction = ray.maxFraction;
            }
            return *this;
        }
};

}

#endif
