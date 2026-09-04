/********************************************************************************
* ReactPhysics3D physics library, http://www.reactphysics3d.com                 *
* Copyright (c) 2010-2024 Daniel Chappuis                                       *
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

#ifndef CRANE_SCENE_H
#define CRANE_SCENE_H

// Libraries
#include "openglframework.h"
#include <reactphysics3d/reactphysics3d.h>
#include "Box.h"
#include "SceneDemo.h"

namespace nanogui { class Slider; class Label; }

namespace cranescene {

// Constants
const float SCENE_RADIUS = 12.0f;

// Class CraneScene
/**
 * A Crane build from a bunch of joints and sliders.
 * Explains it's self in the GUI
 */
class CraneScene : public SceneDemo {

    protected :

        // -------------------- Constants -------------------- //

        static constexpr float BOOM_LENGTH = 4.0f;
        static constexpr float BOOM_ELEVATION_DEG = 45.0f;    // creation pose, = hinge angle 0
        static constexpr float BOOM_MIN_ANGLE_DEG = -35.0f;   // hinge angle, negative = raised
        static constexpr float BOOM_MAX_ANGLE_DEG = 35.0f;
        static constexpr float BOOM_MAX_RATE = 0.5f;          // rad/s at full slider
        static constexpr float BOOM_MAX_TORQUE = 3000.0f;     // N.m
        static constexpr float BOOM_LIMIT_MARGIN_DEG = 5.0f;

        static constexpr float EXT_LENGTH = 3.0f;
        static constexpr float EXT_RETRACTED_CENTRE = 0.7f;   // boom-local z of the extension's centre when retracted
        static constexpr float EXT_MAX = 2.5f;                // m
        static constexpr float EXT_MAX_RATE = 0.6f;           // m/s
        static constexpr float EXT_MAX_FORCE = 3000.0f;       // N
        static constexpr float EXT_LIMIT_MARGIN = 0.15f;

        static constexpr float CABLE_MIN_LENGTH = 1.5f;       // pivot to hook centre, reeled in
        static constexpr float HOOK_MAX_PAYOUT = 3.0f;        // m
        static constexpr float HOOK_MAX_RATE = 1.0f;          // m/s
        static constexpr float HOOK_MAX_FORCE = 300.0f;       // N - a rigid cable can push; keep the winch weak
        static constexpr float HOOK_LIMIT_MARGIN = 0.15f;

        static constexpr float SLEW_MAX_RATE = 0.5f;          // rad/s at full slider, base about the up axis

        // -------------------- Attributes -------------------- //

        Box* mFloor;
        Box* mBase;
        Box* mBoom;
        Box* mExtension;
        Box* mSwivel;
        Box* mHook;
        Box* mPiston;   // cosmetic
        Box* mCable;    // cosmetic

        rp3d::HingeJoint* mBoomHinge;
        rp3d::SliderJoint* mExtensionSlider;
        rp3d::BallAndSocketJoint* mHookPivot;
        rp3d::SliderJoint* mHookSlider;

        /// Velocity commands in [-1, 1] from the Scene panel sliders
        float mBoomCommand;       // positive raises
        float mExtensionCommand;  // positive extends
        float mHookCommand;       // positive lowers (pays out cable)
        float mSlewCommand;       // positive turns the base counter-clockwise seen from above

        nanogui::Slider* mBoomSlider;
        nanogui::Slider* mExtensionSlider_ui;
        nanogui::Slider* mHookSlider_ui;
        nanogui::Slider* mSlewSlider;
        nanogui::Label* mStatusLabel;

        /// World settings
        rp3d::PhysicsWorld::WorldSettings mWorldSettings;

        // -------------------- Methods -------------------- //

        /// Create a box body with a given mass (via collider density), position and orientation
        Box* createBox(rp3d::BodyType type, bool isSimulated, const openglframework::Vector3& size,
                       const rp3d::Vector3& position, const rp3d::Quaternion& orientation,
                       float mass, const openglframework::Color& color);

        /// Create all the bodies and joints
        void createCrane();

        /// Re-stretch the cosmetic piston and cable between their anchors
        void updateVisuals();

        /// Stretch a cosmetic box between two world points
        static void spanBox(Box* box, const rp3d::Vector3& from, const rp3d::Vector3& to, float thickness);

    public:

        // -------------------- Methods -------------------- //

        /// Constructor
        CraneScene(const std::string& name, EngineSettings& settings, reactphysics3d::PhysicsCommon& physicsCommon);

        /// Destructor
        virtual ~CraneScene() override;

        /// Reset the scene
        virtual void reset() override;

        /// Create the physics world
        void createPhysicsWorld();

        /// Destroy the physics world
        void destroyPhysicsWorld();

        /// One physics step: drive the motors from the commands, step, refresh the visuals
        virtual void updatePhysics() override;

        /// Per frame: refresh the status label
        virtual void update() override;

        /// Scene panel controls
        virtual void createGuiWidgets(nanogui::Widget* parent) override;

        /// Hold-to-move keys: W/S boom, A/D slew, T/G extension, Q/E hook
        virtual bool keyboardEvent(int key, int scancode, int action, int mods) override;
};

}

#endif
