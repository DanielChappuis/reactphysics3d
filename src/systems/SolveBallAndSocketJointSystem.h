/********************************************************************************
* ReactPhysics3D physics library, http://www.reactphysics3d.com                 *
* Copyright (c) 2010-2018 Daniel Chappuis                                       *
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

#ifndef REACTPHYSICS3D_SOLVE_BALL_SOCKET_JOINT_SYSTEM_H
#define REACTPHYSICS3D_SOLVE_BALL_SOCKET_JOINT_SYSTEM_H

// Libraries
#include "utils/Profiler.h"
#include "components/RigidBodyComponents.h"
#include "components/TransformComponents.h"

namespace reactphysics3d {

// Class SolveBallAndSocketJointSystem
/**
 * This class is responsible to solve the BallAndSocketJoint constraints
 */
class SolveBallAndSocketJointSystem {

    private :

        // -------------------- Attributes -------------------- //

        /// Reference to the rigid body components
        RigidBodyComponents& mRigidBodyComponents;

#ifdef IS_PROFILING_ACTIVE

        /// Pointer to the profiler
        Profiler* mProfiler;
#endif

    public :

        // -------------------- Methods -------------------- //

        /// Constructor
        SolveBallAndSocketJointSystem(RigidBodyComponents& rigidBodyComponents);

        /// Destructor
        ~SolveBallAndSocketJointSystem() = default;

#ifdef IS_PROFILING_ACTIVE

        /// Set the profiler
        void setProfiler(Profiler* profiler);

#endif

};

#ifdef IS_PROFILING_ACTIVE

// Set the profiler
inline void SolveBallAndSocketJointSystem::setProfiler(Profiler* profiler) {
    mProfiler = profiler;
}

#endif

}

#endif
