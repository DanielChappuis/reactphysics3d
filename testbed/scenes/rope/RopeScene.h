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

#ifndef ROPE_SCENE_H
#define ROPE_SCENE_H

// Libraries
#include "openglframework.h"
#include <reactphysics3d/reactphysics3d.h>
#include "Capsule.h"
#include "Box.h"
#include "SceneDemo.h"

namespace ropescene {

// Constants
const float SCENE_RADIUS = 60.0f;
const float CAPSULE_RADIUS = 0.2f;
const float CAPSULE_HEIGHT = 1.5f;
const float BOX_SIZE = 5.0f;
const int NB_ROPES = 5;
const int NB_CAPSULES_PER_ROPE = 20;

// Class Rope scene
class RopeScene : public SceneDemo {

    protected :

        // -------------------- Attributes -------------------- //

        /// Capsules for the rope
        Capsule* mCapsules[NB_ROPES * NB_CAPSULES_PER_ROPE];

        /// Box attached to the single rope
        Box* mBox1;

        /// Box attached to four ropes
        Box* mBox2;

        /// Plank box
        Box* mPlank;

        /// Ball-And-Socket joints of the rope
        std::vector<rp3d::BallAndSocketJoint*> mBallAndSocketJoints;

        int nbIterations;
        int nbTorqueIterations;

        /// World settings
        rp3d::PhysicsWorld::WorldSettings mWorldSettings;

        // -------------------- Methods -------------------- //

        /// Create the joints
        void createJoints();

    public:

        // -------------------- Methods -------------------- //

        /// Constructor
        RopeScene(const std::string& name, EngineSettings& settings, reactphysics3d::PhysicsCommon& physicsCommon);

        /// Destructor
        virtual ~RopeScene() override ;

        /// Reset the scene
        virtual void reset() override;

        /// Update the physics world (take a simulation step)
        /// Can be called several times per frame
        virtual void updatePhysics() override;

        /// Create the physics world
        void createPhysicsWorld();

        /// Destroy the physics world
        void destroyPhysicsWorld();

        /// Initialize the bodies positions
        void initBodiesPositions();

        /// Move the first rope to an horizontal position
        void moveFirstRopeToHorizontalPosition();
};

}

#endif
