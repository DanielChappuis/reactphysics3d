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

#ifndef RAGDOLL_SCENE_H
#define RAGDOLL_SCENE_H

// Libraries
#include "openglframework.h"
#include <reactphysics3d/reactphysics3d.h>
#include "Box.h"
#include "SceneDemo.h"

namespace ragdollscene {

// Constants
const float SCENE_RADIUS = 45.0f;
const int NB_RAGDOLLS_ROWS = 3;
const int NB_RAGDOLLS_COLS = 2;
const int NB_RAGDOLLS = NB_RAGDOLLS_ROWS * NB_RAGDOLLS_COLS;
const openglframework::Vector3 BOX_SIZE(2, 2, 2);           // Box dimensions in meters
const openglframework::Vector3 FLOOR_1_SIZE(52, 0.5f, 52);  // Floor dimensions in meters
const openglframework::Vector3 FLOOR_2_SIZE(60, 0.5f, 82);  // Floor dimensions in meters
const int NB_BALLSOCKETJOINT_BOXES = 7;                     // Number of Ball-And-Socket chain boxes
const int NB_HINGE_BOXES = 7;                               // Number of Hinge chain boxes

// Class RagdollScene
class RagdollScene : public SceneDemo {

    protected :

        // -------------------- Attributes -------------------- //

        /// Head box
        Box* mHeadBox[NB_RAGDOLLS];

        /// Torso box
        Box* mTorsoBox[NB_RAGDOLLS];

        /// Left upper arm box
        Box* mLeftUpperArmBox[NB_RAGDOLLS];

        /// Left lower arm box
        Box* mLeftLowerArmBox[NB_RAGDOLLS];

        /// Left upper leg box
        Box* mLeftUpperLegBox[NB_RAGDOLLS];

        /// Left lower leg box
        Box* mLeftLowerLegBox[NB_RAGDOLLS];

        /// Right upper arm box
        Box* mRightUpperArmBox[NB_RAGDOLLS];

        /// Right lower arm box
        Box* mRightLowerArmBox[NB_RAGDOLLS];

        /// Right upper leg box
        Box* mRightUpperLegBox[NB_RAGDOLLS];

        /// Right lower leg box
        Box* mRightLowerLegBox[NB_RAGDOLLS];

        /// Box for the floor 1
        Box* mFloor1;

        /// Box for the floor 2
        Box* mFloor2;

        /// Large box
        Box* mLargeBox;

        /// Inclined plane box
        Box* mInclinedPlaneBox;

        /// Ball-And-Socket joint between head and torso
        rp3d::BallAndSocketJoint* mHeadTorsoJoint[NB_RAGDOLLS];

        /// Ball-And-Socket joint between torso and left upper arm
        rp3d::BallAndSocketJoint* mTorsoLeftUpperArmJoint[NB_RAGDOLLS];

        /// Hinge joint between left upper and left lower arm
        rp3d::HingeJoint* mLeftUpperLeftLowerArmJoint[NB_RAGDOLLS];

        /// Ball-And-Socket joint between torso and left upper leg
        rp3d::BallAndSocketJoint* mTorsoLeftUpperLegJoint[NB_RAGDOLLS];

        /// Hinge joint between left upper and left lower leg
        rp3d::HingeJoint* mLeftUpperLeftLowerLegJoint[NB_RAGDOLLS];

        /// Ball-And-Socket joint between torso and right upper arm
        rp3d::BallAndSocketJoint* mTorsoRightUpperArmJoint[NB_RAGDOLLS];

        /// Hinge joint between left upper and right lower arm
        rp3d::HingeJoint* mRightUpperRightLowerArmJoint[NB_RAGDOLLS];

        /// Ball-And-Socket joint between torso and right upper leg
        rp3d::BallAndSocketJoint* mTorsoRightUpperLegJoint[NB_RAGDOLLS];

        /// Hinge joint between left upper and left lower leg
        rp3d::HingeJoint* mRightUpperRightLowerLegJoint[NB_RAGDOLLS];

        rp3d::Vector3 mHeadPos[NB_RAGDOLLS];
        rp3d::Vector3 mTorsoPos[NB_RAGDOLLS];
        rp3d::Vector3 mLeftUpperArmPos[NB_RAGDOLLS];
        rp3d::Vector3 mLeftLowerArmPos[NB_RAGDOLLS];
        rp3d::Vector3 mLeftUpperLegPos[NB_RAGDOLLS];
        rp3d::Vector3 mLeftLowerLegPos[NB_RAGDOLLS];
        rp3d::Vector3 mRightUpperArmPos[NB_RAGDOLLS];
        rp3d::Vector3 mRightLowerArmPos[NB_RAGDOLLS];
        rp3d::Vector3 mRightUpperLegPos[NB_RAGDOLLS];
        rp3d::Vector3 mRightLowerLegPos[NB_RAGDOLLS];

        /// World settings
        rp3d::PhysicsWorld::WorldSettings mWorldSettings;

        // -------------------- Methods -------------------- //

        /// Create the bodies and joints for the ragdoll
        void createRagdolls();

    public:

        // -------------------- Methods -------------------- //

        /// Constructor
        RagdollScene(const std::string& name, EngineSettings& settings, reactphysics3d::PhysicsCommon& physicsCommon);

        /// Destructor
        virtual ~RagdollScene() override ;

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
