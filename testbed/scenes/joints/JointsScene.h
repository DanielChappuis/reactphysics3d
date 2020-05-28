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

#ifndef JOINTS_SCENE_H
#define JOINTS_SCENE_H

// Libraries
#include "openglframework.h"
#include <reactphysics3d/reactphysics3d.h>
#include "Box.h"
#include "SceneDemo.h"

namespace jointsscene {

// Constants
const float SCENE_RADIUS = 30.0f;
const openglframework::Vector3 BOX_SIZE(2, 2, 2);           // Box dimensions in meters
const openglframework::Vector3 FLOOR_SIZE(50, 0.5f, 50);    // Floor dimensions in meters
const int NB_BALLSOCKETJOINT_BOXES = 7;                     // Number of Ball-And-Socket chain boxes
const int NB_HINGE_BOXES = 7;                               // Number of Hinge chain boxes

// Class JointsScene
class JointsScene : public SceneDemo {

    protected :

        // -------------------- Attributes -------------------- //

        /// Boxes of Ball-And-Socket joint chain
        Box* mBallAndSocketJointChainBoxes[NB_BALLSOCKETJOINT_BOXES];

        /// Boxes of the Hinge joint chain
        Box* mHingeJointChainBoxes[NB_HINGE_BOXES];

        /// Ball-And-Socket joints of the chain
        rp3d::BallAndSocketJoint* mBallAndSocketJoints[NB_BALLSOCKETJOINT_BOXES-1];

        /// Hinge joints of the chain
        rp3d::HingeJoint* mHingeJoints[NB_HINGE_BOXES-1];

        /// Bottom box of the Slider joint
        Box* mSliderJointBottomBox;

        /// Top box of the Slider joint
        Box* mSliderJointTopBox;

        /// Slider joint
        rp3d::SliderJoint* mSliderJoint;

        /// Propeller box
        Box* mPropellerBox;

        /// Box 1 of Fixed joint
        Box* mFixedJointBox1;

        /// Box 2 of Fixed joint
        Box* mFixedJointBox2;

        /// Hinge joint
        rp3d::HingeJoint* mPropellerHingeJoint;

        /// First Fixed joint
        rp3d::FixedJoint* mFixedJoint1;

        /// Second Fixed joint
        rp3d::FixedJoint* mFixedJoint2;

        /// Box for the floor
        Box* mFloor;

        // -------------------- Methods -------------------- //

        /// Create the boxes and joints for the Ball-and-Socket joint example
        void createBallAndSocketJoints();

        /// Create the boxes and joint for the Slider joint example
        void createSliderJoint();

        /// Create the boxes and joint for the Hinge joint example
        void createPropellerHingeJoint();

        /// Create the boxes and joint for the Fixed joint example
        void createFixedJoints();

        /// Create the floor
        void createFloor();

    public:

        // -------------------- Methods -------------------- //

        /// Constructor
        JointsScene(const std::string& name, EngineSettings& settings);

        /// Destructor
        virtual ~JointsScene() override ;

        /// Update the physics world (take a simulation step)
        /// Can be called several times per frame
        virtual void updatePhysics() override;

        /// Reset the scene
        virtual void reset() override;
};

}

#endif
