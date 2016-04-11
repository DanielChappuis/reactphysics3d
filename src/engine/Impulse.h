/********************************************************************************
* ReactPhysics3D physics library, http://www.reactphysics3d.com                 *
* Copyright (c) 2010-2016 Daniel Chappuis                                       *
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

#ifndef REACTPHYSICS3D_IMPULSE_H
#define REACTPHYSICS3D_IMPULSE_H

// Libraries
#include "mathematics/mathematics.h"

namespace reactphysics3d {

// Structure Impulse
/**
 * Represents an impulse that we can apply to bodies in the contact or constraint solver.
 */
struct Impulse {

    private:

        // -------------------- Methods -------------------- //

        /// Private assignment operator
        Impulse& operator=(const Impulse& impulse);

    public:

        // -------------------- Attributes -------------------- //

        /// Linear impulse applied to the first body
        const Vector3 linearImpulseBody1;

        /// Angular impulse applied to the first body
        const Vector3 angularImpulseBody1;

        /// Linear impulse applied to the second body
        const Vector3 linearImpulseBody2;

        /// Angular impulse applied to the second body
        const Vector3 angularImpulseBody2;

        // -------------------- Methods -------------------- //

        /// Constructor
        Impulse(const Vector3& initLinearImpulseBody1, const Vector3& initAngularImpulseBody1,
                const Vector3& initLinearImpulseBody2, const Vector3& initAngularImpulseBody2)
            : linearImpulseBody1(initLinearImpulseBody1),
              angularImpulseBody1(initAngularImpulseBody1),
              linearImpulseBody2(initLinearImpulseBody2),
              angularImpulseBody2(initAngularImpulseBody2) {

        }

        /// Copy-constructor
        Impulse(const Impulse& impulse)
              : linearImpulseBody1(impulse.linearImpulseBody1),
                angularImpulseBody1(impulse.angularImpulseBody1),
                linearImpulseBody2(impulse.linearImpulseBody2),
                angularImpulseBody2(impulse.angularImpulseBody2) {
;
        }
};

}

#endif
