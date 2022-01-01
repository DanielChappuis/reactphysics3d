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

#ifndef BALL_AND_SOCKET_JOINTS_NET_SCENE_H
#define BALL_AND_SOCKET_JOINTS_NET_SCENE_H

// Libraries
#include "openglframework.h"
#include <reactphysics3d/reactphysics3d.h>
#include "Sphere.h"
#include "SceneDemo.h"

namespace ballandsocketjointsnetscene {

// Constants
const float SCENE_RADIUS = 40.0f;
const float SPHERE_RADIUS = 0.5f;
const int NB_ROWS_NET_SPHERES = 20;

// Class JointsScene
class BallAndSocketJointsNetScene : public SceneDemo {

    protected :

        // -------------------- Attributes -------------------- //

        /// Spheres in the Ball-And-Socket joint net
        Sphere* mNetSpheres[NB_ROWS_NET_SPHERES][NB_ROWS_NET_SPHERES];

        /// Main sphere
        Sphere* mMainSphere;

        /// Ball-And-Socket joints of the chain
        std::vector<rp3d::BallAndSocketJoint*> mBallAndSocketJoints;

        // -------------------- Methods -------------------- //

        /// Create the joints
        void createJoints();

    public:

        // -------------------- Methods -------------------- //

        /// Constructor
        BallAndSocketJointsNetScene(const std::string& name, EngineSettings& settings, reactphysics3d::PhysicsCommon& physicsCommon);

        /// Destructor
        virtual ~BallAndSocketJointsNetScene() override ;

        /// Reset the scene
        virtual void reset() override;
};

}

#endif
