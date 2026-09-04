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

// Libraries
#include "CraneScene.h"
#include <nanogui/nanogui.h>
#include <GLFW/glfw3.h>
#include <cmath>
#include <sstream>
#include <iomanip>

// Namespaces
using namespace openglframework;
using namespace cranescene;

static const float DEG = rp3d::PI_RP3D / 180.0f;

// Direction of a box's local +Z (its long axis, for the boom/extension) once tilted up by
// `elevation` about the X axis. fromEulerAngles(x,0,0) is a right-handed rotation about X, which
// sends +Z to (0, -sin x, cos x) - so the rotation for an upward elevation is by -elevation.
static rp3d::Quaternion elevationRotation(float elevation) {
    return rp3d::Quaternion::fromEulerAngles(-elevation, 0, 0);
}
static rp3d::Vector3 alongElevation(float elevation, float distance) {
    return rp3d::Vector3(0, distance * std::sin(elevation), distance * std::cos(elevation));
}

// Rotation that points a box's local +Z along `dir`
static rp3d::Quaternion rotationWithZAlong(const rp3d::Vector3& dir) {
    rp3d::Vector3 z = dir.getUnit();
    rp3d::Vector3 x = z.getOneUnitOrthogonalVector();
    rp3d::Vector3 y = z.cross(x);
    // Matrix3x3 is row-major; the columns are the rotated basis vectors
    return rp3d::Quaternion(rp3d::Matrix3x3(x.x, y.x, z.x,
                                            x.y, y.y, z.y,
                                            x.z, y.z, z.z));
}

static float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

// Constructor
CraneScene::CraneScene(const std::string& name, EngineSettings& settings, reactphysics3d::PhysicsCommon& physicsCommon)
      : SceneDemo(name, settings, physicsCommon, true),
        mFloor(nullptr), mBase(nullptr), mBoom(nullptr), mExtension(nullptr), mSwivel(nullptr), mHook(nullptr),
        mPiston(nullptr), mCable(nullptr),
        mBoomHinge(nullptr), mExtensionSlider(nullptr), mHookPivot(nullptr), mHookSlider(nullptr),
        mBoomCommand(0), mExtensionCommand(0), mHookCommand(0), mSlewCommand(0),
        mBoomSlider(nullptr), mExtensionSlider_ui(nullptr), mHookSlider_ui(nullptr), mSlewSlider(nullptr), mStatusLabel(nullptr) {

    // Compute the radius and the center of the scene
    openglframework::Vector3 center(0, 3, 1);

    // Set the center of the scene
    setScenePosition(center, SCENE_RADIUS);
    setInitZoom(1.4);
    resetCameraToViewAll();

    mWorldSettings.worldName = name;
}

// Destructor
CraneScene::~CraneScene() {
    destroyPhysicsWorld();
}

// Create the physics world
void CraneScene::createPhysicsWorld() {
    // Gravity vector in the physics world
    mWorldSettings.gravity = rp3d::Vector3(mEngineSettings.gravity.x, mEngineSettings.gravity.y, mEngineSettings.gravity.z);

    // Create the physics world for the physics simulation
    mPhysicsWorld = mPhysicsCommon.createPhysicsWorld(mWorldSettings);
    mPhysicsWorld->setEventListener(this);

    createCrane();
}

// Destroy the physics world
void CraneScene::destroyPhysicsWorld() {
    if (mPhysicsWorld != nullptr) {
        for (PhysicsObject* object : mPhysicsObjects) {
            delete object;
        }
        mPhysicsObjects.clear();
        mFloor = mBase = mBoom = mExtension = mSwivel = mHook = mPiston = mCable = nullptr;
        mBoomHinge = nullptr; mExtensionSlider = nullptr; mHookPivot = nullptr; mHookSlider = nullptr;

        mPhysicsCommon.destroyPhysicsWorld(mPhysicsWorld);
        mPhysicsWorld = nullptr;
    }
}

// Reset the scene
void CraneScene::reset() {
    SceneDemo::reset();

    destroyPhysicsWorld();
    createPhysicsWorld();

    // Back to "hold" on every motor, sliders included
    mBoomCommand = mExtensionCommand = mHookCommand = mSlewCommand = 0.0f;
    if (mBoomSlider) mBoomSlider->set_value(0.0f);
    if (mExtensionSlider_ui) mExtensionSlider_ui->set_value(0.0f);
    if (mHookSlider_ui) mHookSlider_ui->set_value(0.0f);
    if (mSlewSlider) mSlewSlider->set_value(0.0f);
}

Box* CraneScene::createBox(rp3d::BodyType type, bool isSimulated, const openglframework::Vector3& size,
                           const rp3d::Vector3& position, const rp3d::Quaternion& orientation,
                           float mass, const openglframework::Color& color) {

    Box* box = new Box(type, isSimulated, size, mPhysicsCommon, mPhysicsWorld, mMeshFolderPath);
    box->setTransform(rp3d::Transform(position, orientation));
    box->setColor(color);
    box->setSleepingColor(color);

    if (type == rp3d::BodyType::DYNAMIC) {
        // Mass via density so the inertia tensor is consistent with it (setMass() alone would
        // leave the tensor at whatever the default density produced)
        const float volume = size.x * size.y * size.z;
        box->getCollider()->getMaterial().setMassDensity(mass / volume);
        box->getRigidBody()->updateMassPropertiesFromColliders();
        box->getRigidBody()->setIsAllowedToSleep(false);
    }
    box->getCollider()->getMaterial().setBounciness(0.0f);
    box->getCollider()->getMaterial().setFrictionCoefficient(0.8f);

    mPhysicsObjects.push_back(box);
    return box;
}

// Create all the bodies and joints
void CraneScene::createCrane() {
    // The whole jig is turned so we can see it from the side at startup.
    const rp3d::Quaternion rig = rp3d::Quaternion::fromEulerAngles(0, 90.0f * DEG, 0);
    auto P = [&](const rp3d::Vector3& v) { return rig * v; };
    auto R = [&](const rp3d::Quaternion& q) { return rig * q; };

    const rp3d::Quaternion identity = rp3d::Quaternion::identity();
    const float elevation = BOOM_ELEVATION_DEG * DEG;
    const rp3d::Vector3 boomDir = alongElevation(elevation, 1.0f);

    // Floor, top sitting at y = 0
    mFloor = createBox(rp3d::BodyType::STATIC, true, Vector3(20, 0.5f, 20), rp3d::Vector3(0, -0.25f, 0), identity, 0, mFloorColorDemo);

    // Base slab is KINEMATIC so it can be rotated by the user.
    mBase = createBox(rp3d::BodyType::KINEMATIC, true, Vector3(2, 0.6f, 2), P(rp3d::Vector3(0, 0.3f, 0)), R(identity), 0, mObjectColorDemo);

    // Boom: Pinned at the base, elevated by a piston.
    const rp3d::Vector3 hingeAnchor(0, 0.9f, -0.6f);
    const rp3d::Vector3 boomCentre = hingeAnchor + boomDir * (BOOM_LENGTH * 0.5f);
    mBoom = createBox(rp3d::BodyType::DYNAMIC, true, Vector3(0.3f, 0.3f, BOOM_LENGTH), P(boomCentre), R(elevationRotation(elevation)), 40.0f, mObjectColorDemo);

    {
        rp3d::HingeJointInfo info(mBase->getRigidBody(), mBoom->getRigidBody(), P(hingeAnchor), P(rp3d::Vector3(1, 0, 0)));
        info.isCollisionEnabled = false;   // the boom's root overlaps the base at the hinge by design
        info.isLimitEnabled = true;
        info.minAngleLimit = BOOM_MIN_ANGLE_DEG * DEG;
        info.maxAngleLimit = BOOM_MAX_ANGLE_DEG * DEG;
        info.isMotorEnabled = true;        // the motor IS the hydraulic piston: velocity in, force rating, speed 0 holds
        info.motorSpeed = 0.0;
        info.maxMotorTorque = BOOM_MAX_TORQUE;
        mBoomHinge = dynamic_cast<rp3d::HingeJoint*>(mPhysicsWorld->createJoint(info));
    }

    // Extension: thinner box nested inside the boom, sliding along the boom
    const rp3d::Vector3 extCentre = boomCentre + boomDir * EXT_RETRACTED_CENTRE;
    mExtension = createBox(rp3d::BodyType::DYNAMIC, true, Vector3(0.22f, 0.22f, EXT_LENGTH), P(extCentre), R(elevationRotation(elevation)), 20.0f, mObjectColorDemo);

    {
        // body1 = boom, body2 = extension: translation grows outward, positive motor speed extends
        rp3d::SliderJointInfo info(mBoom->getRigidBody(), mExtension->getRigidBody(), P(extCentre), P(boomDir));
        info.isCollisionEnabled = false;
        info.isLimitEnabled = true;
        info.minTranslationLimit = 0.0;
        info.maxTranslationLimit = EXT_MAX;
        info.isMotorEnabled = true;
        info.motorSpeed = 0.0;
        info.maxMotorForce = EXT_MAX_FORCE;
        mExtensionSlider = dynamic_cast<rp3d::SliderJoint*>(mPhysicsWorld->createJoint(info));
    }

    // Swivel: small cube where the crane hook hangs from.
    const rp3d::Vector3 pivot = extCentre + boomDir * (EXT_LENGTH * 0.5f + 0.25f);
    mSwivel = createBox(rp3d::BodyType::DYNAMIC, true, Vector3(0.2f, 0.2f, 0.2f), P(pivot), R(identity), 2.0f, mObjectColorDemo);

    {
        rp3d::BallAndSocketJointInfo info(mExtension->getRigidBody(), mSwivel->getRigidBody(), P(pivot));
        info.isCollisionEnabled = false;
        mHookPivot = dynamic_cast<rp3d::BallAndSocketJoint*>(mPhysicsWorld->createJoint(info));
    }

    // Hook: hangs below the swivel on a slider along the swivel's down axis. It's more a rod than a cable.
    mHook = createBox(rp3d::BodyType::DYNAMIC, true, Vector3(0.2f, 0.3f, 0.2f), P(pivot - rp3d::Vector3(0, CABLE_MIN_LENGTH, 0)), R(identity), 10.0f, mObjectColorDemo);

    {
        rp3d::SliderJointInfo info(mSwivel->getRigidBody(), mHook->getRigidBody(), P(pivot), rp3d::Vector3(0, -1, 0));
        info.isCollisionEnabled = false;
        info.isLimitEnabled = true;
        info.minTranslationLimit = 0.0;
        info.maxTranslationLimit = HOOK_MAX_PAYOUT;
        info.isMotorEnabled = true;
        info.motorSpeed = 0.0;
        info.maxMotorForce = HOOK_MAX_FORCE;
        mHookSlider = dynamic_cast<rp3d::SliderJoint*>(mPhysicsWorld->createJoint(info));
    }

    // No colliders for the cosmetic piston and cable. They strech between the anchor points.
    mPiston = createBox(rp3d::BodyType::STATIC, false, Vector3(0.16f, 0.16f, 1.0f), rp3d::Vector3(0, 0, 0), identity, 0, mSleepingColorDemo);
    mCable = createBox(rp3d::BodyType::STATIC, false, Vector3(0.06f, 0.06f, 1.0f), rp3d::Vector3(0, 0, 0), identity, 0, mSleepingColorDemo);
    updateVisuals();
}

// Stretch a box between two world points
void CraneScene::spanBox(Box* box, const rp3d::Vector3& from, const rp3d::Vector3& to, float thickness) {
    rp3d::Vector3 delta = to - from;
    const float span = delta.length();
    if (span < 0.001f) return;

    box->setSize(Vector3(thickness, thickness, span));
    box->setTransform(rp3d::Transform(from + delta * 0.5f, rotationWithZAlong(delta)));
}

// Re-stretch the piston and cable between their anchors
void CraneScene::updateVisuals() {
    if (mPiston && mBase && mBoom) {
        const rp3d::Vector3 baseAnchor = mBase->getRigidBody()->getTransform() * rp3d::Vector3(0, 0.4f, -0.2f);
        const rp3d::Vector3 boomAnchor = mBoom->getRigidBody()->getTransform() * rp3d::Vector3(0, 0, -BOOM_LENGTH * 0.5f + 1.2f);
        spanBox(mPiston, baseAnchor, boomAnchor, 0.16f);
    }
    if (mCable && mSwivel && mHook) {
        const rp3d::Vector3 swivelPos = mSwivel->getRigidBody()->getTransform().getPosition();
        const rp3d::Vector3 hookTop = mHook->getRigidBody()->getTransform() * rp3d::Vector3(0, 0.15f, 0);
        spanBox(mCable, swivelPos, hookTop, 0.06f);
    }
}

// Gracefully stop the motor at it's limit instead of banging against it. The motor speed is tapered to zero as the limit is approached, over a distance of `margin`.
static float taperedSpeed(float command, float value, float minLimit, float maxLimit, float margin, float maxRate) {
    const float remaining = command > 0.0f ? maxLimit - value : value - minLimit;
    const float taper = clampf(remaining / margin, 0.0f, 1.0f);
    return command * maxRate * taper;
}

// One physics step
void CraneScene::updatePhysics() {
    if (mBoomHinge) {
        const float angle = mBoomHinge->getAngle();
        mBoomHinge->setMotorSpeed(-taperedSpeed(mBoomCommand, -angle, -BOOM_MAX_ANGLE_DEG * DEG, -BOOM_MIN_ANGLE_DEG * DEG, BOOM_LIMIT_MARGIN_DEG * DEG, BOOM_MAX_RATE));
    }
    if (mExtensionSlider) {
        mExtensionSlider->setMotorSpeed(taperedSpeed(mExtensionCommand, mExtensionSlider->getTranslation(), 0.0f, EXT_MAX, EXT_LIMIT_MARGIN, EXT_MAX_RATE));
    }
    if (mHookSlider) {
        mHookSlider->setMotorSpeed(taperedSpeed(mHookCommand, mHookSlider->getTranslation(), 0.0f, HOOK_MAX_PAYOUT, HOOK_LIMIT_MARGIN, HOOK_MAX_RATE));
    }
    if (mBase) {
        // Slewrate: the kinematic base just gets the angular velocity
        mBase->getRigidBody()->setAngularVelocity(rp3d::Vector3(0, mSlewCommand * SLEW_MAX_RATE, 0));
    }

    SceneDemo::updatePhysics();

    updateVisuals();
}

// Per frame
void CraneScene::update() {
    SceneDemo::update();

    if (mStatusLabel && mBoomHinge && mExtensionSlider && mHookSlider) {
        std::ostringstream out;
        // Base yaw from its forward axis (the rig's local +Z turned by the base's orientation)
        const rp3d::Vector3 fwd = mBase->getRigidBody()->getTransform().getOrientation() * rp3d::Vector3(0, 0, 1);
        const float yaw = std::atan2(fwd.x, fwd.z) / DEG;
        out << std::fixed << std::setprecision(1)
            << "Boom " << (mBoomHinge->getAngle() / DEG) << " deg, base " << yaw << " deg, "
            << std::setprecision(2)
            << "ext " << mExtensionSlider->getTranslation() << " m, "
            << "cable " << mHookSlider->getTranslation() << " m";
        mStatusLabel->set_caption(out.str());
    }
}

bool CraneScene::keyboardEvent(int key, int scancode, int action, int mods) {
    if (SceneDemo::keyboardEvent(key, scancode, action, mods)) return true;
    if (action == GLFW_REPEAT) return false;

    const float value = (action == GLFW_PRESS) ? 1.0f : 0.0f;
    auto drive = [&](float& command, nanogui::Slider* slider, float sign) {
        command = sign * value;
        if (slider) slider->set_value(command);
        return true;
    };
    switch (key) {
        case GLFW_KEY_W: return drive(mBoomCommand, mBoomSlider, 1.0f);
        case GLFW_KEY_S: return drive(mBoomCommand, mBoomSlider, -1.0f);
        case GLFW_KEY_A: return drive(mSlewCommand, mSlewSlider, 1.0f);
        case GLFW_KEY_D: return drive(mSlewCommand, mSlewSlider, -1.0f);
        case GLFW_KEY_T: return drive(mExtensionCommand, mExtensionSlider_ui, 1.0f);
        case GLFW_KEY_G: return drive(mExtensionCommand, mExtensionSlider_ui, -1.0f);
        case GLFW_KEY_E: return drive(mHookCommand, mHookSlider_ui, 1.0f);
        case GLFW_KEY_Q: return drive(mHookCommand, mHookSlider_ui, -1.0f);
        default: return false;
    }
}

// Scene panel controls for this scene
void CraneScene::createGuiWidgets(nanogui::Widget* parent) {
    using namespace nanogui;

    new Label(parent, "Crane Scene Demonstrates:", "sans-bold");
    auto addText = [&](const std::string& text) {
        Label* label = new Label(parent, text);
        label->set_fixed_width(230);   // wraps
        return label;
    };
    addText("Boom (DYNAMIC): HingeJoint to the base, motor + angle limits.");
    addText("Extension (DYNAMIC): SliderJoint along the boom, motor + limits.");
    addText("Hook (DYNAMIC): BallAndSocketJoint swivel at the tip, then a SliderJoint 'cable' with the winch as its motor.");
    addText("Base (KINEMATIC): No motor. Rotatable by angular velocity. All joints should follow.");
    addText("Each motor's velocity can be set: 0 tries to hold it with the motor torque. Velocity tapers off near a limit.");

    new Label(parent, "Keys (hold)", "sans-bold");
    addText("W / S : boom up / down");
    addText("A / D : slew left / right");
    addText("T / G : extension out / in");
    addText("Q / E : hook up / down");

    new Label(parent, "Velocity commands (0 holds)", "sans-bold");

    auto addSlider = [&](const std::string& title, float& command) -> Slider* {
        Label* label = new Label(parent, title + " : 0.00");
        Slider* slider = new Slider(parent);
        slider->set_range(std::make_pair(-1.0f, 1.0f));
        slider->set_value(0.0f);
        slider->set_fixed_width(230);
        slider->set_callback([&command, label, title](float value) {
            command = value;
            std::ostringstream out;
            out << title << " : " << std::fixed << std::setprecision(2) << value;
            label->set_caption(out.str());
        });
        return slider;
    };

    mBoomSlider = addSlider("Boom (+ raises)", mBoomCommand);
    mExtensionSlider_ui = addSlider("Extension (+ extends)", mExtensionCommand);
    mHookSlider_ui = addSlider("Hook winch (+ lowers)", mHookCommand);
    mSlewSlider = addSlider("Base slew (+ left)", mSlewCommand);

    Button* stop = new Button(parent, "Stop all");
    stop->set_callback([this] {
        mBoomCommand = mExtensionCommand = mHookCommand = mSlewCommand = 0.0f;
        mBoomSlider->set_value(0.0f);
        mExtensionSlider_ui->set_value(0.0f);
        mHookSlider_ui->set_value(0.0f);
        mSlewSlider->set_value(0.0f);
    });

    mStatusLabel = new Label(parent, "");
}
