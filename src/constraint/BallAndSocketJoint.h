/********************************************************************************
* ReactPhysics3D physics library, http://code.google.com/p/reactphysics3d/      *
* Copyright (c) 2010-2013 Daniel Chappuis                                       *
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

#ifndef REACTPHYSICS3D_BALL_AND_SOCKET_JOINT_H
#define REACTPHYSICS3D_BALL_AND_SOCKET_JOINT_H

// Libraries
#include "Constraint.h"

namespace reactphysics3d {

// Class BallAndSocketJoint
/**
 * This class represents a ball-and-socket joint that allows arbitrary rotation
 * between two bodies.
 */
class BallAndSocketJoint : public Constraint {

    private :

        // -------------------- Attributes -------------------- //

    public :

        // -------------------- Methods -------------------- //

        /// Constructor
        BallAndSocketJoint(RigidBody* const body1, RigidBody* const body2,
                           bool active, ConstraintType type);

        /// Destructor
        virtual ~BallAndSocketJoint();
};

}

#endif
