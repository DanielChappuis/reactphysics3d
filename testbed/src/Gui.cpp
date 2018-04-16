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
double Gui::mCachedPhysicsUpdateTime = 0;

// Constructor
Gui::Gui(TestbedApplication* app)
    : mApp(app), mSimulationPanel(nullptr), mSettingsPanel(nullptr), mPhysicsPanel(nullptr),
      mRenderingPanel(nullptr), mFPSLabel(nullptr), mFrameTimeLabel(nullptr), mPhysicsTimeLabel(nullptr)
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

    mApp->setVisible(true);
    mApp->performLayout();

    mTimeSinceLastProfilingDisplay = glfwGetTime();
}

// Update the GUI
void Gui::update() {

    // Update Profiling GUI every seconds
    if ((mApp->mCurrentTime - mTimeSinceLastProfilingDisplay)  > TIME_INTERVAL_DISPLAY_PROFILING_INFO) {
        mTimeSinceLastProfilingDisplay = mApp->mCurrentTime;
        mCachedFPS = mApp->mFPS;
        mCachedUpdateTime = mApp->mFrameTime;
        mCachedPhysicsUpdateTime = mApp->mPhysicsTime;
    }

    // Framerate (FPS)
    mFPSLabel->setCaption(std::string("FPS : ") + floatToString(mCachedFPS, 0));

    // Frame time
    mFrameTimeLabel->setCaption(std::string("Frame time : ") + floatToString(mCachedUpdateTime * 1000.0, 1) + std::string(" ms"));

    // Physics time
    mPhysicsTimeLabel->setCaption(std::string("Physics time : ") + floatToString(mCachedPhysicsUpdateTime * 1000.0, 1) + std::string(" ms"));
}

void Gui::createSimulationPanel() {

    mSimulationPanel = new Window(mApp, "Simulation");
    mSimulationPanel->setPosition(Vector2i(15, 15));
    mSimulationPanel->setLayout(new GroupLayout(10, 5, 10 , 20));
    mSimulationPanel->setId("SimulationPanel");
    mSimulationPanel->setFixedWidth(220);

    // Scenes/Physics/Rendering buttons
    new Label(mSimulationPanel, "Controls","sans-bold");
    Widget* panelControls = new Widget(mSimulationPanel);
    panelControls->setLayout(new BoxLayout(Orientation::Horizontal, Alignment::Middle, 0, 5));
    ToolButton* buttonPlay = new ToolButton(panelControls, ENTYPO_ICON_CONTROLLER_PLAY);
    buttonPlay->setFlags(Button::NormalButton);
    buttonPlay->setCallback([&] {
        mApp->playSimulation();
    });
    ToolButton* buttonPause = new ToolButton(panelControls, ENTYPO_ICON_CONTROLLER_PAUS);
    buttonPause->setFlags(Button::NormalButton);
    buttonPause->setCallback([&] {
        mApp->pauseSimulation();
    });
    ToolButton* buttonPlayStep = new ToolButton(panelControls, ENTYPO_ICON_CONTROLLER_NEXT);
    buttonPlayStep->setFlags(Button::NormalButton);
    buttonPlayStep->setCallback([&] {
        mApp->toggleTakeSinglePhysicsStep();
    });
    ToolButton* buttonRestart = new ToolButton(panelControls, ENTYPO_ICON_CCW);
    buttonRestart->setFlags(Button::NormalButton);
    buttonRestart->setCallback([&] {
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
    comboBoxScenes->setFixedWidth(150);
    comboBoxScenes->setCallback([&, scenes](int index) {
        mApp->switchScene(scenes[index]);
    });
}

void Gui::createSettingsPanel() {

    mSettingsPanel = new Window(mApp, "Settings");
    mSettingsPanel->setPosition(Vector2i(15, 180));
    mSettingsPanel->setLayout(new BoxLayout(Orientation::Vertical, Alignment::Middle, 10, 5));
    mSettingsPanel->setId("SettingsPanel");
    mSettingsPanel->setFixedWidth(220);

    // Scenes/Physics/Rendering buttons
    Widget* buttonsPanel = new Widget(mSettingsPanel);
    buttonsPanel->setLayout(new BoxLayout(Orientation::Horizontal, Alignment::Middle, 5, 5));
    Button* buttonPhysics = new Button(buttonsPanel, "Physics");
    buttonPhysics->setFlags(Button::RadioButton);
    buttonPhysics->setPushed(true);
    buttonPhysics->setChangeCallback([&](bool state) {
        mPhysicsPanel->setVisible(true);
        mRenderingPanel->setVisible(false);
        mApp->performLayout();
    });
    Button* buttonRendering = new Button(buttonsPanel, "Rendering");
    buttonRendering->setFlags(Button::RadioButton);
    buttonRendering->setChangeCallback([&](bool state) {
        mRenderingPanel->setVisible(true);
        mPhysicsPanel->setVisible(false);
        mApp->performLayout();
    });

    // ---------- Physics Panel ----------
    mPhysicsPanel = new Widget(mSettingsPanel);
    mPhysicsPanel->setLayout(new BoxLayout(Orientation::Vertical, Alignment::Fill, 0, 5));

    // Enable/Disable sleeping
    CheckBox* checkboxSleeping = new CheckBox(mPhysicsPanel, "Sleeping enabled");
    checkboxSleeping->setChecked(mApp->mEngineSettings.isSleepingEnabled);
    checkboxSleeping->setCallback([&](bool value) {
        mApp->mEngineSettings.isSleepingEnabled = value;
        mApp->notifyEngineSetttingsChanged();
    });

    // Enabled/Disable Gravity
    CheckBox* checkboxGravity = new CheckBox(mPhysicsPanel, "Gravity enabled");
    checkboxGravity->setChecked(mApp->mEngineSettings.isGravityEnabled);
    checkboxGravity->setCallback([&](bool value) {
        mApp->mEngineSettings.isGravityEnabled = value;
        mApp->notifyEngineSetttingsChanged();
    });

    // Timestep
    Widget* panelTimeStep = new Widget(mPhysicsPanel);
    panelTimeStep->setLayout(new BoxLayout(Orientation::Horizontal, Alignment::Middle, 0, 5));
    Label* labelTimeStep = new Label(panelTimeStep, "Time step","sans-bold");
    labelTimeStep->setFixedWidth(120);
    TextBox* textboxTimeStep = new TextBox(panelTimeStep);
    textboxTimeStep->setFixedSize(Vector2i(70, 25));
    textboxTimeStep->setEditable(true);
    std::ostringstream out;
    out << std::setprecision(1) << std::fixed << (mApp->mEngineSettings.timeStep * 1000);
    textboxTimeStep->setValue(out.str());
    textboxTimeStep->setUnits("ms");
    textboxTimeStep->setCallback([&, textboxTimeStep](const std::string &str) {

        try {
            float value = std::stof(str);
            std::ostringstream out;
            out << std::setprecision(1) << std::fixed << std::showpoint << value;
            float finalValue = std::stof(out.str());

            if (finalValue < 1 || finalValue > 1000) return false;

            mApp->mEngineSettings.timeStep = finalValue / 1000.0f;
            mApp->notifyEngineSetttingsChanged();
            textboxTimeStep->setValue(out.str());
        }
        catch (...) {
            return false;
        }

        return true;
    });
    textboxTimeStep->setFontSize(16);
    textboxTimeStep->setAlignment(TextBox::Alignment::Right);

    // Velocity solver iterations
    Widget* panelVelocityIterations = new Widget(mPhysicsPanel);
    panelVelocityIterations->setLayout(new BoxLayout(Orientation::Horizontal, Alignment::Middle, 0, 5));
    Label* labelVelocityIterations = new Label(panelVelocityIterations, "Velocity solver","sans-bold");
    labelVelocityIterations->setFixedWidth(120);
    TextBox* textboxVelocityIterations = new TextBox(panelVelocityIterations);
    textboxVelocityIterations->setFixedSize(Vector2i(70, 25));
    textboxVelocityIterations->setEditable(true);
    textboxVelocityIterations->setValue(std::to_string(mApp->mEngineSettings.nbVelocitySolverIterations));
    textboxVelocityIterations->setUnits("iter");
    textboxVelocityIterations->setCallback([&, textboxVelocityIterations](const std::string &str) {

        try {
            float value = std::stof(str);
            std::ostringstream out;
            out << std::setprecision(0) << std::fixed << value;

            if (value < 1 || value > 1000) return false;

            mApp->mEngineSettings.nbVelocitySolverIterations = value;
            mApp->notifyEngineSetttingsChanged();
            textboxVelocityIterations->setValue(out.str());
        }
        catch (...) {
            return false;
        }

        return true;
    });
    textboxVelocityIterations->setFontSize(16);
    textboxVelocityIterations->setAlignment(TextBox::Alignment::Right);

    // Position solver iterations
    Widget* panelPositionIterations = new Widget(mPhysicsPanel);
    panelPositionIterations->setLayout(new BoxLayout(Orientation::Horizontal, Alignment::Middle, 0, 5));
    Label* labelPositionIterations = new Label(panelPositionIterations, "Position solver","sans-bold");
    labelPositionIterations->setFixedWidth(120);
    TextBox* textboxPositionIterations = new TextBox(panelPositionIterations);
    textboxPositionIterations->setFixedSize(Vector2i(70, 25));
    textboxPositionIterations->setEditable(true);
    textboxPositionIterations->setValue(std::to_string(mApp->mEngineSettings.nbPositionSolverIterations));
    textboxPositionIterations->setUnits("iter");
    textboxPositionIterations->setCallback([&, textboxPositionIterations](const std::string &str) {

        try {
            float value = std::stof(str);
            std::ostringstream out;
            out << std::setprecision(0) << std::fixed << value;

            if (value < 1 || value > 1000) return false;

            mApp->mEngineSettings.nbPositionSolverIterations = value;
            mApp->notifyEngineSetttingsChanged();
            textboxPositionIterations->setValue(out.str());
        }
        catch (...) {
            return false;
        }

        return true;
    });
    textboxPositionIterations->setFontSize(16);
    textboxPositionIterations->setAlignment(TextBox::Alignment::Right);

    // Time before sleep
    Widget* panelTimeSleep = new Widget(mPhysicsPanel);
    panelTimeSleep->setLayout(new BoxLayout(Orientation::Horizontal, Alignment::Middle, 0, 5));
    Label* labelTimeSleep = new Label(panelTimeSleep, "Time before sleep","sans-bold");
    labelTimeSleep->setFixedWidth(120);
    out.str("");
    out << std::setprecision(0) << std::fixed << (mApp->mEngineSettings.timeBeforeSleep * 1000);
    TextBox* textboxTimeSleep = new TextBox(panelTimeSleep);
    textboxTimeSleep->setFixedSize(Vector2i(70, 25));
    textboxTimeSleep->setEditable(true);
    textboxTimeSleep->setValue(out.str());
    textboxTimeSleep->setUnits("ms");
    textboxTimeSleep->setCallback([&, textboxTimeSleep](const std::string &str) {

        try {
            float value = std::stof(str);
            std::ostringstream out;
            out << std::setprecision(0) << std::fixed << std::showpoint << value;
            float finalValue = std::stof(out.str());

            if (finalValue < 1 || finalValue > 100000) return false;

            mApp->mEngineSettings.timeBeforeSleep = finalValue / 1000.0f;
            mApp->notifyEngineSetttingsChanged();
            textboxTimeSleep->setValue(out.str());
        }
        catch (...) {
            return false;
        }

        return true;
    });
    textboxTimeSleep->setFontSize(16);
    textboxTimeSleep->setAlignment(TextBox::Alignment::Right);

    // Sleep linear velocity
    Widget* panelSleepLinearVel = new Widget(mPhysicsPanel);
    panelSleepLinearVel->setLayout(new GridLayout(Orientation::Horizontal, 2, Alignment::Middle, 0, 5));
    Label* labelSleepLinearVel = new Label(panelSleepLinearVel, "Sleep linear velocity","sans-bold");
    labelSleepLinearVel->setFixedWidth(120);
    out.str("");
    out << std::setprecision(2) << std::fixed << (mApp->mEngineSettings.sleepLinearVelocity);
    TextBox* textboxSleepLinearVel = new TextBox(panelSleepLinearVel);
    textboxSleepLinearVel->setFixedSize(Vector2i(70, 25));
    textboxSleepLinearVel->setEditable(true);
    textboxSleepLinearVel->setValue(out.str());
    textboxSleepLinearVel->setUnits("m/s");
    textboxSleepLinearVel->setCallback([&, textboxSleepLinearVel](const std::string &str) {

        try {
            float value = std::stof(str);
            std::ostringstream out;
            out << std::setprecision(2) << std::fixed << std::showpoint << value;
            float finalValue = std::stof(out.str());

            if (finalValue < 0 || finalValue > 10000) return false;

            mApp->mEngineSettings.sleepLinearVelocity = finalValue;
            mApp->notifyEngineSetttingsChanged();
            textboxSleepLinearVel->setValue(out.str());
        }
        catch (...) {
            return false;
        }

        return true;
    });
    textboxSleepLinearVel->setFontSize(16);
    textboxSleepLinearVel->setAlignment(TextBox::Alignment::Right);

    // Sleep angular velocity
    Widget* panelSleepAngularVel = new Widget(mPhysicsPanel);
    panelSleepAngularVel->setLayout(new GridLayout(Orientation::Horizontal, 2, Alignment::Middle, 0, 5));
    Label* labelSleepAngularVel = new Label(panelSleepAngularVel, "Sleep angular velocity","sans-bold");
    labelSleepAngularVel->setFixedWidth(120);
    out.str("");
    out << std::setprecision(2) << std::fixed << (mApp->mEngineSettings.sleepAngularVelocity);
    TextBox* textboxSleepAngularVel = new TextBox(panelSleepAngularVel);
    textboxSleepAngularVel->setFixedSize(Vector2i(70, 25));
    textboxSleepAngularVel->setEditable(true);
    textboxSleepAngularVel->setValue(out.str());
    textboxSleepAngularVel->setUnits("rad/s");
    textboxSleepAngularVel->setCallback([&, textboxSleepAngularVel](const std::string &str) {

        try {
            float value = std::stof(str);
            std::ostringstream out;
            out << std::setprecision(2) << std::fixed << std::showpoint << value;
            float finalValue = std::stof(out.str());

            if (finalValue < 0 || finalValue > 10000) return false;

            mApp->mEngineSettings.sleepAngularVelocity = finalValue;
            mApp->notifyEngineSetttingsChanged();
            textboxSleepAngularVel->setValue(out.str());
        }
        catch (...) {
            return false;
        }

        return true;
    });
    textboxSleepAngularVel->setFontSize(16);
    textboxSleepAngularVel->setAlignment(TextBox::Alignment::Right);

    // ---------- Rendering Panel ----------
    mRenderingPanel = new Widget(mSettingsPanel);
    mRenderingPanel->setLayout(new BoxLayout(Orientation::Vertical, Alignment::Fill, 0, 5));

    // Display/Hide contact points
    CheckBox* checkboxContactPoints = new CheckBox(mRenderingPanel, "Contact points");
    checkboxContactPoints->setChecked(mApp->mIsContactPointsDisplayed);
    checkboxContactPoints->setCallback([&](bool value) {
        mApp->mIsContactPointsDisplayed = value;
    });


    // Display/Hide the AABBs
    CheckBox* checkboxAABBs = new CheckBox(mRenderingPanel, "AABBs");
    checkboxAABBs->setChecked(mApp->mIsAABBsDisplayed);
    checkboxAABBs->setCallback([&](bool value) {
        mApp->mIsAABBsDisplayed = value;
    });

    // Enabled/Disable VSync
    CheckBox* checkboxVSync = new CheckBox(mRenderingPanel, "V-Sync");
    checkboxVSync->setChecked(mApp->mIsVSyncEnabled);
    checkboxVSync->setCallback([&](bool value) {
        mApp->enableVSync(value);
    });

    // Enabled/Disable Shadows
    CheckBox* checkboxShadows = new CheckBox(mRenderingPanel, "Shadows");
    checkboxShadows->setChecked(mApp->mIsShadowMappingEnabled);
    checkboxShadows->setCallback([&](bool value) {
        mApp->mIsShadowMappingEnabled = value;
    });

    // Enable/Disable wireframe mode
    CheckBox* checkboxWireframe = new CheckBox(mRenderingPanel, "Wireframe");
    checkboxWireframe->setChecked(mApp->mIsWireframeEnabled);
    checkboxWireframe->setCallback([&](bool value) {
        mApp->mIsWireframeEnabled = value;
    });

    mPhysicsPanel->setVisible(true);
    mRenderingPanel->setVisible(false);
}

void Gui::createProfilingPanel() {

    Widget* profilingPanel = new Window(mApp, "Profiling");
    profilingPanel->setPosition(Vector2i(15, 525));
    profilingPanel->setLayout(new BoxLayout(Orientation::Vertical, Alignment::Fill, 10, 5));
    profilingPanel->setId("SettingsPanel");
    profilingPanel->setFixedWidth(220);

    // Framerate (FPS)
    mFPSLabel = new Label(profilingPanel, std::string("FPS : ") + floatToString(mCachedFPS, 0),"sans-bold");

    // Update time
    mFrameTimeLabel = new Label(profilingPanel, std::string("Frame time : ") + floatToString(mCachedUpdateTime * 1000.0, 1) + std::string(" ms"),"sans-bold");

    // Update time
    mPhysicsTimeLabel = new Label(profilingPanel, std::string("Physics time : ") + floatToString(mCachedPhysicsUpdateTime * 1000.0, 1) + std::string(" ms"),"sans-bold");

    profilingPanel->setVisible(true);
}

