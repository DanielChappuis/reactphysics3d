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

#ifndef BALL_AND_SOCKET_JOINTS_CHAIN_SCENE_H
#define BALL_AND_SOCKET_JOINTS_CHAIN_SCENE_H

// Libraries
#include "openglframework.h"
#include <reactphysics3d/reactphysics3d.h>
#include "Sphere.h"
#include "SceneDemo.h"

namespace ballandsocketjointschainscene {

// Constants
const float SCENE_RADIUS = 60.0f;
const float SPHERE_RADIUS = 1.0f;
const int NB_SPHERES = 20;

// Class BallAndSocketJointsChain scene
class BallAndSocketJointsChainScene : public SceneDemo {

    protected :

        // -------------------- Attributes -------------------- //

        /// Spheres in the Ball-And-Socket joint net
        Sphere* mSpheres[NB_SPHERES];

        /// Ball-And-Socket joints of the chain
        std::vector<rp3d::BallAndSocketJoint*> mBallAndSocketJoints;

        /// World settings
        rp3d::PhysicsWorld::WorldSettings mWorldSettings;

        // -------------------- Methods -------------------- //

        /// Create the joints
        void createJoints();

    public:

        // -------------------- Methods -------------------- //

        /// Constructor
        BallAndSocketJointsChainScene(const std::string& name, EngineSettings& settings, reactphysics3d::PhysicsCommon& physicsCommon);

        /// Destructor
        virtual ~BallAndSocketJointsChainScene() override ;

        /// Reset the scene
        virtual void reset() override;

        /// Create the physics world
        void createPhysicsWorld();

        /// Destroy the physics world
        void destroyPhysicsWorld();

        /// Initialize the bodies positions
        void initBodiesPositions();
};

}

#endif
