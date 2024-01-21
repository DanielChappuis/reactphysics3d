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
      mPhysicsStepTimeLabel(nullptr), mIsDisplayed(true)
{

}

// Destructor
Gui::~Gui() {


}

/// Initialize the GUI
void Gui::init(GLFWwindow* window) {

    mWindow = window;

    mScreen = new Screen();
    mScreen->initialize(window, true);

    // Create the Simulation panel
    createSimulationPanel();

    // Create the Settings panel
    createSettingsPanel();

    // Create the Profiling panel
    createProfilingPanel();

    mScreen->set_visible(true);
    mScreen->perform_layout();

    mTimeSinceLastProfilingDisplay = glfwGetTime();
}

void Gui::drawAll() {
    mScreen->clear();
    mScreen->draw_all();
}

void Gui::draw() {

  if (mIsDisplayed) {
      mScreen->draw_setup();
      mScreen->draw_contents();
  }
}

void Gui::drawTearDown() {

  if (mIsDisplayed) {
      mScreen->draw_widgets();
  }

  mScreen->draw_teardown();
}


// Update the GUI values with the engine settings from the current scene
void Gui::resetWithValuesFromCurrentScene() {

    auto test = mApp->getCurrentSceneEngineSettings();
    mCheckboxSleeping->set_checked(mApp->getCurrentSceneEngineSettings().isSleepingEnabled);
    mCheckboxGravity->set_checked(mApp->getCurrentSceneEngineSettings().isGravityEnabled);

    std::ostringstream out;
    out << std::setprecision(1) << std::fixed << (mApp->getCurrentSceneEngineSettings().timeStep.count() * 1000);
    mTextboxTimeStep->set_value(out.str());

    mTextboxVelocityIterations->set_value(std::to_string(mApp->getCurrentSceneEngineSettings().nbVelocitySolverIterations));
    mTextboxPositionIterations->set_value(std::to_string(mApp->getCurrentSceneEngineSettings().nbPositionSolverIterations));

    out.str("");
    out << std::setprecision(0) << std::fixed << (mApp->getCurrentSceneEngineSettings().timeBeforeSleep * 1000);
    mTextboxTimeSleep->set_value(out.str());

    out.str("");
    out << std::setprecision(2) << std::fixed << (mApp->getCurrentSceneEngineSettings().sleepLinearVelocity);
    mTextboxSleepLinearVel->set_value(out.str());

    out.str("");
    out << std::setprecision(2) << std::fixed << (mApp->getCurrentSceneEngineSettings().sleepAngularVelocity);
    mTextboxSleepAngularVel->set_value(out.str());
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

    mSimulationPanel = new Window(mScreen, "Simulation");
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

    mComboBoxScenes = new ComboBox(mSimulationPanel, scenesNames);
    mComboBoxScenes->set_callback([&, scenes](int index) {
        mApp->switchScene(scenes[index]);
    });
}

void Gui::createSettingsPanel() {

    mSettingsPanel = new Window(mScreen, "Settings");
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
    buttonPhysics->set_change_callback([&](bool /*state*/) {
        mPhysicsPanel->set_visible(true);
        mRenderingPanel->set_visible(false);
        mScreen->perform_layout();
    });
    Button* buttonRendering = new Button(buttonsPanel, "Rendering");
    buttonRendering->set_flags(Button::RadioButton);
    buttonRendering->set_change_callback([&](bool /*state*/) {
        mRenderingPanel->set_visible(true);
        mPhysicsPanel->set_visible(false);
        mScreen->perform_layout();
    });

    // ---------- Physics Panel ----------
    mPhysicsPanel = new Widget(mSettingsPanel);
    mPhysicsPanel->set_layout(new BoxLayout(Orientation::Vertical, Alignment::Fill, 0, 5));

    // Enable/Disable sleeping
    mCheckboxSleeping = new CheckBox(mPhysicsPanel, "Sleeping enabled");
    mCheckboxSleeping->set_checked(true);
    mCheckboxSleeping->set_callback([&](bool value) {
        mApp->getCurrentSceneEngineSettings().isSleepingEnabled = value;
        mApp->notifyEngineSetttingsChanged();
    });

    // Enabled/Disable Gravity
    mCheckboxGravity = new CheckBox(mPhysicsPanel, "Gravity enabled");
    mCheckboxGravity->set_checked(true);
    mCheckboxGravity->set_callback([&](bool value) {
        mApp->getCurrentSceneEngineSettings().isGravityEnabled = value;
        mApp->notifyEngineSetttingsChanged();
    });

    // Timestep
    Widget* panelTimeStep = new Widget(mPhysicsPanel);
    panelTimeStep->set_layout(new BoxLayout(Orientation::Horizontal, Alignment::Middle, 0, 5));
    Label* labelTimeStep = new Label(panelTimeStep, "Time step","sans-bold");
    labelTimeStep->set_fixed_width(120);
    mTextboxTimeStep = new TextBox(panelTimeStep);
    mTextboxTimeStep->set_fixed_size(Vector2i(70, 25));
    mTextboxTimeStep->set_editable(true);
    std::ostringstream out;
    out << std::setprecision(1) << std::fixed << 0;
    mTextboxTimeStep->set_value(out.str());
    mTextboxTimeStep->set_units("ms");
    mTextboxTimeStep->set_callback([&](const std::string &str) {

        try {
            float value = std::stof(str);
            std::ostringstream out;
            out << std::setprecision(1) << std::fixed << std::showpoint << value;
            float finalValue = std::stof(out.str());

            if (finalValue < 1 || finalValue > 1000) return false;

            mApp->getCurrentSceneEngineSettings().timeStep = std::chrono::duration<double>(finalValue / 1000.0f);
            mApp->notifyEngineSetttingsChanged();
            mTextboxTimeStep->set_value(out.str());
        }
        catch (...) {
            return false;
        }

        return true;
    });
    mTextboxTimeStep->set_font_size(16);
    mTextboxTimeStep->set_alignment(TextBox::Alignment::Right);

    // Velocity solver iterations
    Widget* panelVelocityIterations = new Widget(mPhysicsPanel);
    panelVelocityIterations->set_layout(new BoxLayout(Orientation::Horizontal, Alignment::Middle, 0, 5));
    Label* labelVelocityIterations = new Label(panelVelocityIterations, "Velocity solver","sans-bold");
    labelVelocityIterations->set_fixed_width(120);
    mTextboxVelocityIterations = new TextBox(panelVelocityIterations);
    mTextboxVelocityIterations->set_fixed_size(Vector2i(70, 25));
    mTextboxVelocityIterations->set_editable(true);
    mTextboxVelocityIterations->set_value(std::to_string(0));
    mTextboxVelocityIterations->set_units("iter");
    mTextboxVelocityIterations->set_callback([&](const std::string &str) {

        try {
            float value = std::stof(str);
            std::ostringstream out;
            out << std::setprecision(0) << std::fixed << value;

            if (value < 1 || value > 1000) return false;

            mApp->getCurrentSceneEngineSettings().nbVelocitySolverIterations = value;
            mApp->notifyEngineSetttingsChanged();
            mTextboxVelocityIterations->set_value(out.str());
        }
        catch (...) {
            return false;
        }

        return true;
    });
    mTextboxVelocityIterations->set_font_size(16);
    mTextboxVelocityIterations->set_alignment(TextBox::Alignment::Right);

    // Position solver iterations
    Widget* panelPositionIterations = new Widget(mPhysicsPanel);
    panelPositionIterations->set_layout(new BoxLayout(Orientation::Horizontal, Alignment::Middle, 0, 5));
    Label* labelPositionIterations = new Label(panelPositionIterations, "Position solver","sans-bold");
    labelPositionIterations->set_fixed_width(120);
    mTextboxPositionIterations = new TextBox(panelPositionIterations);
    mTextboxPositionIterations->set_fixed_size(Vector2i(70, 25));
    mTextboxPositionIterations->set_editable(true);
    mTextboxPositionIterations->set_value(std::to_string(0));
    mTextboxPositionIterations->set_units("iter");
    mTextboxPositionIterations->set_callback([&](const std::string &str) {

        try {
            float value = std::stof(str);
            std::ostringstream out;
            out << std::setprecision(0) << std::fixed << value;

            if (value < 1 || value > 1000) return false;

            mApp->getCurrentSceneEngineSettings().nbPositionSolverIterations = value;
            mApp->notifyEngineSetttingsChanged();
            mTextboxPositionIterations->set_value(out.str());
        }
        catch (...) {
            return false;
        }

        return true;
    });
    mTextboxPositionIterations->set_font_size(16);
    mTextboxPositionIterations->set_alignment(TextBox::Alignment::Right);

    // Time before sleep
    Widget* panelTimeSleep = new Widget(mPhysicsPanel);
    panelTimeSleep->set_layout(new BoxLayout(Orientation::Horizontal, Alignment::Middle, 0, 5));
    Label* labelTimeSleep = new Label(panelTimeSleep, "Time before sleep","sans-bold");
    labelTimeSleep->set_fixed_width(120);
    out.str("");
    out << std::setprecision(0) << std::fixed << 0;
    mTextboxTimeSleep = new TextBox(panelTimeSleep);
    mTextboxTimeSleep->set_fixed_size(Vector2i(70, 25));
    mTextboxTimeSleep->set_editable(true);
    mTextboxTimeSleep->set_value(out.str());
    mTextboxTimeSleep->set_units("ms");
    mTextboxTimeSleep->set_callback([&](const std::string &str) {

        try {
            float value = std::stof(str);
            std::ostringstream out;
            out << std::setprecision(0) << std::fixed << std::showpoint << value;
            float finalValue = std::stof(out.str());

            if (finalValue < 1 || finalValue > 100000) return false;

            mApp->getCurrentSceneEngineSettings().timeBeforeSleep = finalValue / 1000.0f;
            mApp->notifyEngineSetttingsChanged();
            mTextboxTimeSleep->set_value(out.str());
        }
        catch (...) {
            return false;
        }

        return true;
    });
    mTextboxTimeSleep->set_font_size(16);
    mTextboxTimeSleep->set_alignment(TextBox::Alignment::Right);

    // Sleep linear velocity
    Widget* panelSleepLinearVel = new Widget(mPhysicsPanel);
    panelSleepLinearVel->set_layout(new GridLayout(Orientation::Horizontal, 2, Alignment::Middle, 0, 5));
    Label* labelSleepLinearVel = new Label(panelSleepLinearVel, "Sleep linear velocity","sans-bold");
    labelSleepLinearVel->set_fixed_width(120);
    out.str("");
    out << std::setprecision(2) << std::fixed << 0;
    mTextboxSleepLinearVel = new TextBox(panelSleepLinearVel);
    mTextboxSleepLinearVel->set_fixed_size(Vector2i(70, 25));
    mTextboxSleepLinearVel->set_editable(true);
    mTextboxSleepLinearVel->set_value(out.str());
    mTextboxSleepLinearVel->set_units("m/s");
    mTextboxSleepLinearVel->set_callback([&](const std::string &str) {

        try {
            float value = std::stof(str);
            std::ostringstream out;
            out << std::setprecision(2) << std::fixed << std::showpoint << value;
            float finalValue = std::stof(out.str());

            if (finalValue < 0 || finalValue > 10000) return false;

            mApp->getCurrentSceneEngineSettings().sleepLinearVelocity = finalValue;
            mApp->notifyEngineSetttingsChanged();
            mTextboxSleepLinearVel->set_value(out.str());
        }
        catch (...) {
            return false;
        }

        return true;
    });
    mTextboxSleepLinearVel->set_font_size(16);
    mTextboxSleepLinearVel->set_alignment(TextBox::Alignment::Right);

    // Sleep angular velocity
    Widget* panelSleepAngularVel = new Widget(mPhysicsPanel);
    panelSleepAngularVel->set_layout(new GridLayout(Orientation::Horizontal, 2, Alignment::Middle, 0, 5));
    Label* labelSleepAngularVel = new Label(panelSleepAngularVel, "Sleep angular velocity","sans-bold");
    labelSleepAngularVel->set_fixed_width(120);
    out.str("");
    out << std::setprecision(2) << std::fixed << 0;
    mTextboxSleepAngularVel = new TextBox(panelSleepAngularVel);
    mTextboxSleepAngularVel->set_fixed_size(Vector2i(70, 25));
    mTextboxSleepAngularVel->set_editable(true);
    mTextboxSleepAngularVel->set_value(out.str());
    mTextboxSleepAngularVel->set_units("rad/s");
    mTextboxSleepAngularVel->set_callback([&](const std::string &str) {

        try {
            float value = std::stof(str);
            std::ostringstream out;
            out << std::setprecision(2) << std::fixed << std::showpoint << value;
            float finalValue = std::stof(out.str());

            if (finalValue < 0 || finalValue > 10000) return false;

            mApp->getCurrentSceneEngineSettings().sleepAngularVelocity = finalValue;
            mApp->notifyEngineSetttingsChanged();
            mTextboxSleepAngularVel->set_value(out.str());
        }
        catch (...) {
            return false;
        }

        return true;
    });
    mTextboxSleepAngularVel->set_font_size(16);
    mTextboxSleepAngularVel->set_alignment(TextBox::Alignment::Right);

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
    CheckBox* checkboxCollisionShapes = new CheckBox(mRenderingPanel, "Colliders shapes");
    checkboxCollisionShapes->set_checked(mApp->mAreCollisionShapesDisplayed);
    checkboxCollisionShapes->set_enabled(false);
    checkboxCollisionShapes->set_callback([&](bool value) {
        mApp->mAreCollisionShapesDisplayed = value;
    });

    // Display/Hide the collision shapes normals
    CheckBox* checkboxCollisionShapesNormals = new CheckBox(mRenderingPanel, "Colliders normals");
    checkboxCollisionShapesNormals->set_checked(mApp->mAreCollisionShapesNormalsDisplayed);
    checkboxCollisionShapesNormals->set_enabled(false);
    checkboxCollisionShapesNormals->set_callback([&](bool value) {
        mApp->mAreCollisionShapesNormalsDisplayed = value;
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
                                               checkboxCollisionShapes, checkboxCollisionShapesNormals](bool value) {
        mApp->mIsDebugRendererEnabled = value;
        checkboxContactPoints->set_enabled(value);
        checkboxContactNormals->set_enabled(value);
        checkboxBroadPhaseAABBs->set_enabled(value);
        checkboxColliderAABBs->set_enabled(value);
        checkboxCollisionShapes->set_enabled(value);
        checkboxCollisionShapesNormals->set_enabled(value);
    });

    mPhysicsPanel->set_visible(true);
    mRenderingPanel->set_visible(false);
}

void Gui::createProfilingPanel() {

    Widget* profilingPanel = new Window(mScreen, "Profiling");
    profilingPanel->set_position(Vector2i(15, 505));
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

void Gui::onWindowResizeEvent(int width, int height) {
    mScreen->resize_callback_event(width, height);
}

void Gui::onMouseMotionEvent(double x, double y) {
    mScreen->cursor_pos_callback_event(x, y);
}

bool Gui::onScrollEvent(double x, double y) {

    double xMouse, yMouse;
    glfwGetCursorPos(mWindow, &xMouse, &yMouse);

    // If the mouse cursor is over the scenes choice scrolling menu
    const float pixelRatio = mScreen->pixel_ratio();
    if (mComboBoxScenes->popup()->visible() && mComboBoxScenes->popup()->contains(Vector2i(xMouse, yMouse) / pixelRatio)) {
        mScreen->scroll_callback_event(x, y);
        return true;
    }

    return false;
}

void Gui::onMouseButtonEvent(int button, int action, int modifiers) {
    mScreen->mouse_button_callback_event(button, action, modifiers);
}

void Gui::onKeyboardEvent(int key, int scancode, int action, int modifiers) {
    mScreen->key_callback_event(key, scancode, action, modifiers);
}
