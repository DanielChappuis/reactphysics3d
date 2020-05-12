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

// Libraries
#include "Gui.h"
#include <GLFW/glfw3.h>
#include "TestbedApplication.h"

using namespace nanogui;

//GLFWwindow* Gui::mWindow = NULL;
double Gui::mScrollX = 0.0;
double Gui::mScrollY = 0.0;
double Gui::mTimeSinceLastProfilingDisplay = 0;
double Gui::mCachedFPS = 0;
double Gui::mCachedUpdateTime = 0;
double Gui::mCachedTotalPhysicsUpdateTime = 0;
double Gui::mCachedPhysicsStepTime = 0;

// Constructor
Gui::Gui(TestbedApplication* app)
    : mApp(app), mSimulationPanel(nullptr), mSettingsPanel(nullptr), mPhysicsPanel(nullptr),
      mRenderingPanel(nullptr), mFPSLabel(nullptr), mFrameTimeLabel(nullptr), mTotalPhysicsTimeLabel(nullptr),
      mPhysicsStepTimeLabel(nullptr)
{

}

// Destructor
Gui::~Gui() {


}

/// Initialize the GUI
void Gui::init() {

    // Create the Simulation panel
    createSimulationPanel();

    // Create the Settings panel
    createSettingsPanel();

    // Create the Profiling panel
    createProfilingPanel();

    mApp->set_visible(true);
    mApp->perform_layout();

    mTimeSinceLastProfilingDisplay = glfwGetTime();
}

// Update the GUI
void Gui::update() {

    // Update Profiling GUI every seconds
    if ((mApp->mCurrentTime - mTimeSinceLastProfilingDisplay)  > TIME_INTERVAL_DISPLAY_PROFILING_INFO) {
        mTimeSinceLastProfilingDisplay = mApp->mCurrentTime;
        mCachedFPS = mApp->mFPS;
        mCachedUpdateTime = mApp->mFrameTime;
        mCachedTotalPhysicsUpdateTime = mApp->mTotalPhysicsTime;
        mCachedPhysicsStepTime = mApp->mPhysicsStepTime;
    }

    // Framerate (FPS)
    mFPSLabel->set_caption(std::string("FPS : ") + floatToString(mCachedFPS, 0));

    // Frame time
    mFrameTimeLabel->set_caption(std::string("Frame time : ") + floatToString(mCachedUpdateTime * 1000.0, 1) + std::string(" ms"));

    // Total Physics time
    mTotalPhysicsTimeLabel->set_caption(std::string("Total physics time : ") + floatToString(mCachedTotalPhysicsUpdateTime * 1000.0, 1) + std::string(" ms"));

    // Physics step time
    mPhysicsStepTimeLabel->set_caption(std::string("Physics step time : ") + floatToString(mCachedPhysicsStepTime * 1000.0, 1) + std::string(" ms"));
}

void Gui::createSimulationPanel() {

    mSimulationPanel = new Window(mApp, "Simulation");
    mSimulationPanel->set_position(Vector2i(15, 15));
    mSimulationPanel->set_layout(new GroupLayout(10, 5, 10 , 20));
    //mSimulationPanel->setId("SimulationPanel");
    mSimulationPanel->set_fixed_width(220);

    // Scenes/Physics/Rendering buttons
    new Label(mSimulationPanel, "Controls","sans-bold");
    Widget* panelControls = new Widget(mSimulationPanel);
    panelControls->set_layout(new BoxLayout(Orientation::Horizontal, Alignment::Middle, 0, 5));
    ToolButton* buttonPlay = new ToolButton(panelControls, FA_PLAY);
    buttonPlay->set_flags(Button::NormalButton);
    buttonPlay->set_callback([&] {
        mApp->playSimulation();
    });
    ToolButton* buttonPause = new ToolButton(panelControls, FA_PAUSE);
    buttonPause->set_flags(Button::NormalButton);
    buttonPause->set_callback([&] {
        mApp->pauseSimulation();
    });
    ToolButton* buttonPlayStep = new ToolButton(panelControls, FA_STEP_FORWARD);
    buttonPlayStep->set_flags(Button::NormalButton);
    buttonPlayStep->set_callback([&] {
        mApp->toggleTakeSinglePhysicsStep();
    });
    ToolButton* buttonRestart = new ToolButton(panelControls, FA_REDO);
    buttonRestart->set_flags(Button::NormalButton);
    buttonRestart->set_callback([&] {
        mApp->restartSimulation();
    });

    // Scenes
    std::vector<Scene*> scenes = mApp->getScenes();
    std::vector<std::string> scenesNames;
    for (uint i=0; i<scenes.size(); i++) {
        scenesNames.push_back(scenes[i]->getName().c_str());
    }
    new Label(mSimulationPanel, "Scene","sans-bold");
    ComboBox* comboBoxScenes = new ComboBox(mSimulationPanel, scenesNames);
    comboBoxScenes->set_fixed_width(150);
    comboBoxScenes->set_callback([&, scenes](int index) {
        mApp->switchScene(scenes[index]);
    });
}

void Gui::createSettingsPanel() {

    mSettingsPanel = new Window(mApp, "Settings");
    mSettingsPanel->set_position(Vector2i(15, 180));
    mSettingsPanel->set_layout(new BoxLayout(Orientation::Vertical, Alignment::Middle, 10, 5));
    //mSettingsPanel->setId("SettingsPanel");
    mSettingsPanel->set_fixed_width(220);

    // Scenes/Physics/Rendering buttons
    Widget* buttonsPanel = new Widget(mSettingsPanel);
    buttonsPanel->set_layout(new BoxLayout(Orientation::Horizontal, Alignment::Middle, 5, 5));
    Button* buttonPhysics = new Button(buttonsPanel, "Physics");
    buttonPhysics->set_flags(Button::RadioButton);
    buttonPhysics->set_pushed(true);
    buttonPhysics->set_change_callback([&](bool state) {
        mPhysicsPanel->set_visible(true);
        mRenderingPanel->set_visible(false);
        mApp->perform_layout();
    });
    Button* buttonRendering = new Button(buttonsPanel, "Rendering");
    buttonRendering->set_flags(Button::RadioButton);
    buttonRendering->set_change_callback([&](bool state) {
        mRenderingPanel->set_visible(true);
        mPhysicsPanel->set_visible(false);
        mApp->perform_layout();
    });

    // ---------- Physics Panel ----------
    mPhysicsPanel = new Widget(mSettingsPanel);
    mPhysicsPanel->set_layout(new BoxLayout(Orientation::Vertical, Alignment::Fill, 0, 5));

    // Enable/Disable sleeping
    CheckBox* checkboxSleeping = new CheckBox(mPhysicsPanel, "Sleeping enabled");
    checkboxSleeping->set_checked(mApp->mEngineSettings.isSleepingEnabled);
    checkboxSleeping->set_callback([&](bool value) {
        mApp->mEngineSettings.isSleepingEnabled = value;
        mApp->notifyEngineSetttingsChanged();
    });

    // Enabled/Disable Gravity
    CheckBox* checkboxGravity = new CheckBox(mPhysicsPanel, "Gravity enabled");
    checkboxGravity->set_checked(mApp->mEngineSettings.isGravityEnabled);
    checkboxGravity->set_callback([&](bool value) {
        mApp->mEngineSettings.isGravityEnabled = value;
        mApp->notifyEngineSetttingsChanged();
    });

    // Timestep
    Widget* panelTimeStep = new Widget(mPhysicsPanel);
    panelTimeStep->set_layout(new BoxLayout(Orientation::Horizontal, Alignment::Middle, 0, 5));
    Label* labelTimeStep = new Label(panelTimeStep, "Time step","sans-bold");
    labelTimeStep->set_fixed_width(120);
    TextBox* textboxTimeStep = new TextBox(panelTimeStep);
    textboxTimeStep->set_fixed_size(Vector2i(70, 25));
    textboxTimeStep->set_editable(true);
    std::ostringstream out;
    out << std::setprecision(1) << std::fixed << (mApp->mEngineSettings.timeStep * 1000);
    textboxTimeStep->set_value(out.str());
    textboxTimeStep->set_units("ms");
    textboxTimeStep->set_callback([&, textboxTimeStep](const std::string &str) {

        try {
            float value = std::stof(str);
            std::ostringstream out;
            out << std::setprecision(1) << std::fixed << std::showpoint << value;
            float finalValue = std::stof(out.str());

            if (finalValue < 1 || finalValue > 1000) return false;

            mApp->mEngineSettings.timeStep = finalValue / 1000.0f;
            mApp->notifyEngineSetttingsChanged();
            textboxTimeStep->set_value(out.str());
        }
        catch (...) {
            return false;
        }

        return true;
    });
    textboxTimeStep->set_font_size(16);
    textboxTimeStep->set_alignment(TextBox::Alignment::Right);

    // Velocity solver iterations
    Widget* panelVelocityIterations = new Widget(mPhysicsPanel);
    panelVelocityIterations->set_layout(new BoxLayout(Orientation::Horizontal, Alignment::Middle, 0, 5));
    Label* labelVelocityIterations = new Label(panelVelocityIterations, "Velocity solver","sans-bold");
    labelVelocityIterations->set_fixed_width(120);
    TextBox* textboxVelocityIterations = new TextBox(panelVelocityIterations);
    textboxVelocityIterations->set_fixed_size(Vector2i(70, 25));
    textboxVelocityIterations->set_editable(true);
    textboxVelocityIterations->set_value(std::to_string(mApp->mEngineSettings.nbVelocitySolverIterations));
    textboxVelocityIterations->set_units("iter");
    textboxVelocityIterations->set_callback([&, textboxVelocityIterations](const std::string &str) {

        try {
            float value = std::stof(str);
            std::ostringstream out;
            out << std::setprecision(0) << std::fixed << value;

            if (value < 1 || value > 1000) return false;

            mApp->mEngineSettings.nbVelocitySolverIterations = value;
            mApp->notifyEngineSetttingsChanged();
            textboxVelocityIterations->set_value(out.str());
        }
        catch (...) {
            return false;
        }

        return true;
    });
    textboxVelocityIterations->set_font_size(16);
    textboxVelocityIterations->set_alignment(TextBox::Alignment::Right);

    // Position solver iterations
    Widget* panelPositionIterations = new Widget(mPhysicsPanel);
    panelPositionIterations->set_layout(new BoxLayout(Orientation::Horizontal, Alignment::Middle, 0, 5));
    Label* labelPositionIterations = new Label(panelPositionIterations, "Position solver","sans-bold");
    labelPositionIterations->set_fixed_width(120);
    TextBox* textboxPositionIterations = new TextBox(panelPositionIterations);
    textboxPositionIterations->set_fixed_size(Vector2i(70, 25));
    textboxPositionIterations->set_editable(true);
    textboxPositionIterations->set_value(std::to_string(mApp->mEngineSettings.nbPositionSolverIterations));
    textboxPositionIterations->set_units("iter");
    textboxPositionIterations->set_callback([&, textboxPositionIterations](const std::string &str) {

        try {
            float value = std::stof(str);
            std::ostringstream out;
            out << std::setprecision(0) << std::fixed << value;

            if (value < 1 || value > 1000) return false;

            mApp->mEngineSettings.nbPositionSolverIterations = value;
            mApp->notifyEngineSetttingsChanged();
            textboxPositionIterations->set_value(out.str());
        }
        catch (...) {
            return false;
        }

        return true;
    });
    textboxPositionIterations->set_font_size(16);
    textboxPositionIterations->set_alignment(TextBox::Alignment::Right);

    // Time before sleep
    Widget* panelTimeSleep = new Widget(mPhysicsPanel);
    panelTimeSleep->set_layout(new BoxLayout(Orientation::Horizontal, Alignment::Middle, 0, 5));
    Label* labelTimeSleep = new Label(panelTimeSleep, "Time before sleep","sans-bold");
    labelTimeSleep->set_fixed_width(120);
    out.str("");
    out << std::setprecision(0) << std::fixed << (mApp->mEngineSettings.timeBeforeSleep * 1000);
    TextBox* textboxTimeSleep = new TextBox(panelTimeSleep);
    textboxTimeSleep->set_fixed_size(Vector2i(70, 25));
    textboxTimeSleep->set_editable(true);
    textboxTimeSleep->set_value(out.str());
    textboxTimeSleep->set_units("ms");
    textboxTimeSleep->set_callback([&, textboxTimeSleep](const std::string &str) {

        try {
            float value = std::stof(str);
            std::ostringstream out;
            out << std::setprecision(0) << std::fixed << std::showpoint << value;
            float finalValue = std::stof(out.str());

            if (finalValue < 1 || finalValue > 100000) return false;

            mApp->mEngineSettings.timeBeforeSleep = finalValue / 1000.0f;
            mApp->notifyEngineSetttingsChanged();
            textboxTimeSleep->set_value(out.str());
        }
        catch (...) {
            return false;
        }

        return true;
    });
    textboxTimeSleep->set_font_size(16);
    textboxTimeSleep->set_alignment(TextBox::Alignment::Right);

    // Sleep linear velocity
    Widget* panelSleepLinearVel = new Widget(mPhysicsPanel);
    panelSleepLinearVel->set_layout(new GridLayout(Orientation::Horizontal, 2, Alignment::Middle, 0, 5));
    Label* labelSleepLinearVel = new Label(panelSleepLinearVel, "Sleep linear velocity","sans-bold");
    labelSleepLinearVel->set_fixed_width(120);
    out.str("");
    out << std::setprecision(2) << std::fixed << (mApp->mEngineSettings.sleepLinearVelocity);
    TextBox* textboxSleepLinearVel = new TextBox(panelSleepLinearVel);
    textboxSleepLinearVel->set_fixed_size(Vector2i(70, 25));
    textboxSleepLinearVel->set_editable(true);
    textboxSleepLinearVel->set_value(out.str());
    textboxSleepLinearVel->set_units("m/s");
    textboxSleepLinearVel->set_callback([&, textboxSleepLinearVel](const std::string &str) {

        try {
            float value = std::stof(str);
            std::ostringstream out;
            out << std::setprecision(2) << std::fixed << std::showpoint << value;
            float finalValue = std::stof(out.str());

            if (finalValue < 0 || finalValue > 10000) return false;

            mApp->mEngineSettings.sleepLinearVelocity = finalValue;
            mApp->notifyEngineSetttingsChanged();
            textboxSleepLinearVel->set_value(out.str());
        }
        catch (...) {
            return false;
        }

        return true;
    });
    textboxSleepLinearVel->set_font_size(16);
    textboxSleepLinearVel->set_alignment(TextBox::Alignment::Right);

    // Sleep angular velocity
    Widget* panelSleepAngularVel = new Widget(mPhysicsPanel);
    panelSleepAngularVel->set_layout(new GridLayout(Orientation::Horizontal, 2, Alignment::Middle, 0, 5));
    Label* labelSleepAngularVel = new Label(panelSleepAngularVel, "Sleep angular velocity","sans-bold");
    labelSleepAngularVel->set_fixed_width(120);
    out.str("");
    out << std::setprecision(2) << std::fixed << (mApp->mEngineSettings.sleepAngularVelocity);
    TextBox* textboxSleepAngularVel = new TextBox(panelSleepAngularVel);
    textboxSleepAngularVel->set_fixed_size(Vector2i(70, 25));
    textboxSleepAngularVel->set_editable(true);
    textboxSleepAngularVel->set_value(out.str());
    textboxSleepAngularVel->set_units("rad/s");
    textboxSleepAngularVel->set_callback([&, textboxSleepAngularVel](const std::string &str) {

        try {
            float value = std::stof(str);
            std::ostringstream out;
            out << std::setprecision(2) << std::fixed << std::showpoint << value;
            float finalValue = std::stof(out.str());

            if (finalValue < 0 || finalValue > 10000) return false;

            mApp->mEngineSettings.sleepAngularVelocity = finalValue;
            mApp->notifyEngineSetttingsChanged();
            textboxSleepAngularVel->set_value(out.str());
        }
        catch (...) {
            return false;
        }

        return true;
    });
    textboxSleepAngularVel->set_font_size(16);
    textboxSleepAngularVel->set_alignment(TextBox::Alignment::Right);

    // ---------- Rendering Panel ----------
    mRenderingPanel = new Widget(mSettingsPanel);
    mRenderingPanel->set_layout(new BoxLayout(Orientation::Vertical, Alignment::Fill, 0, 5));

    // Display/Hide contact points
    CheckBox* checkboxDebugRendererEnabled = new CheckBox(mRenderingPanel, "Debug rendering");
    checkboxDebugRendererEnabled->set_checked(mApp->mIsDebugRendererEnabled);

    // Display/Hide contact points
    CheckBox* checkboxContactPoints = new CheckBox(mRenderingPanel, "Contact points");
    checkboxContactPoints->set_checked(mApp->mAreContactPointsDisplayed);
    checkboxContactPoints->set_enabled(false);
    checkboxContactPoints->set_callback([&](bool value) {
        mApp->mAreContactPointsDisplayed = value;
    });

    // Display/Hide contact normals
    CheckBox* checkboxContactNormals = new CheckBox(mRenderingPanel, "Contact normals");
    checkboxContactNormals->set_checked(mApp->mAreContactNormalsDisplayed);
    checkboxContactNormals->set_enabled(false);
    checkboxContactNormals->set_callback([&](bool value) {
        mApp->mAreContactNormalsDisplayed = value;
    });

    // Display/Hide the Broad-phase AABBs
    CheckBox* checkboxBroadPhaseAABBs = new CheckBox(mRenderingPanel, "Broad phase AABBs");
    checkboxBroadPhaseAABBs->set_checked(mApp->mAreBroadPhaseAABBsDisplayed);
    checkboxBroadPhaseAABBs->set_enabled(false);
    checkboxBroadPhaseAABBs->set_callback([&](bool value) {
        mApp->mAreBroadPhaseAABBsDisplayed = value;
    });

    // Display/Hide the colliders AABBs
    CheckBox* checkboxColliderAABBs = new CheckBox(mRenderingPanel, "Colliders AABBs");
    checkboxColliderAABBs->set_checked(mApp->mAreCollidersAABBsDisplayed);
    checkboxColliderAABBs->set_enabled(false);
    checkboxColliderAABBs->set_callback([&](bool value) {
        mApp->mAreCollidersAABBsDisplayed = value;
    });

    // Display/Hide the collision shapes
    CheckBox* checkboxCollisionShapes = new CheckBox(mRenderingPanel, "Collision shapes");
    checkboxCollisionShapes->set_checked(mApp->mAreCollisionShapesDisplayed);
    checkboxCollisionShapes->set_enabled(false);
    checkboxCollisionShapes->set_callback([&](bool value) {
        mApp->mAreCollisionShapesDisplayed = value;
    });

    // Enable/Disable wireframe mode
    CheckBox* checkboxWireframe = new CheckBox(mRenderingPanel, "Objects Wireframe");
    checkboxWireframe->set_checked(mApp->mAreObjectsWireframeEnabled);
    checkboxWireframe->set_callback([&](bool value) {
        mApp->mAreObjectsWireframeEnabled = value;
    });

    // Enabled/Disable VSync
    CheckBox* checkboxVSync = new CheckBox(mRenderingPanel, "V-Sync");
    checkboxVSync->set_checked(mApp->mIsVSyncEnabled);
    checkboxVSync->set_callback([&](bool value) {
        mApp->enableVSync(value);
    });

    // Enabled/Disable Shadows
    CheckBox* checkboxShadows = new CheckBox(mRenderingPanel, "Shadows");
    checkboxShadows->set_checked(mApp->mIsShadowMappingEnabled);
    checkboxShadows->set_callback([&](bool value) {
        mApp->mIsShadowMappingEnabled = value;
    });

    checkboxDebugRendererEnabled->set_callback([&, checkboxContactPoints, checkboxContactNormals,
                                               checkboxBroadPhaseAABBs, checkboxColliderAABBs,
                                               checkboxCollisionShapes](bool value) {
        mApp->mIsDebugRendererEnabled = value;
        checkboxContactPoints->set_enabled(value);
        checkboxContactNormals->set_enabled(value);
        checkboxBroadPhaseAABBs->set_enabled(value);
        checkboxColliderAABBs->set_enabled(value);
        checkboxCollisionShapes->set_enabled(value);
    });

    mPhysicsPanel->set_visible(true);
    mRenderingPanel->set_visible(false);
}

void Gui::createProfilingPanel() {

    Widget* profilingPanel = new Window(mApp, "Profiling");
    profilingPanel->set_position(Vector2i(15, 525));
    profilingPanel->set_layout(new BoxLayout(Orientation::Vertical, Alignment::Fill, 10, 5));
    //profilingPanel->setId("SettingsPanel");
    profilingPanel->set_fixed_width(220);

    // Framerate (FPS)
    mFPSLabel = new Label(profilingPanel, std::string("FPS : ") + floatToString(mCachedFPS, 0),"sans-bold");

    // Update time
    mFrameTimeLabel = new Label(profilingPanel, std::string("Frame time : ") + floatToString(mCachedUpdateTime * 1000.0, 1) + std::string(" ms"),"sans-bold");

    // Total physics time
    mTotalPhysicsTimeLabel = new Label(profilingPanel, std::string("Total physics time : ") + floatToString(mCachedTotalPhysicsUpdateTime * 1000.0, 1) + std::string(" ms"),"sans-bold");

    // Physics step time
    mPhysicsStepTimeLabel = new Label(profilingPanel, std::string("Physics step time : ") + floatToString(mCachedPhysicsStepTime * 1000.0, 1) + std::string(" ms"),"sans-bold");

    profilingPanel->set_visible(true);
}

