/********************************************************************************
* ReactPhysics3D physics library, http://www.reactphysics3d.com                 *
* Copyright (c) 2010-2015 Daniel Chappuis                                       *
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
#include <sstream>
#include <iomanip>
#include "TestbedApplication.h"

using namespace nanogui;

//GLFWwindow* Gui::mWindow = NULL;
double Gui::g_Time = 0.0f;
bool Gui::g_MousePressed[3] = {false, false, false};
float Gui::g_MouseWheel = 0.0f;
GLuint Gui::g_FontTexture = 0;
size_t Gui::g_VboSize = 0;
int Gui::g_AttribLocationTex = 0, Gui::g_AttribLocationProjMtx = 0;
int Gui::g_AttribLocationPosition = 0, Gui::g_AttribLocationUV = 0, Gui::g_AttribLocationColor = 0;
Shader Gui::mShader;
openglframework::VertexBufferObject Gui::mVBO(GL_ARRAY_BUFFER);
openglframework::VertexArrayObject Gui::mVAO;
Gui::LeftPane Gui::mLeftPane = SCENES;
double Gui::mScrollX = 0.0;
double Gui::mScrollY = 0.0;
double Gui::mTimeSinceLastProfilingDisplay = 0;
double Gui::mCachedFPS = 0;
double Gui::mCachedUpdateTime = 0;
double Gui::mCachedPhysicsUpdateTime = 0;

// Constructor
Gui::Gui(Screen* screen) {

    mScreen = screen;

    g_Time = 0.0f;
    g_MousePressed[0] = false;
    g_MousePressed[1] = false;
    g_MousePressed[2] = false;
    g_MouseWheel = 0.0f;
    g_FontTexture = 0;
    g_VboSize = 0;
}

// Destructor
Gui::~Gui() {

    mVAO.destroy();
    mVBO.destroy();

    mShader.destroy();

    //imguiRenderGLDestroy();
}

/// Initialize the GUI
void Gui::init() {

    bool bvar = true;
    int ivar = 12345678;
    double dvar = 3.1415926;
    float fvar = (float)dvar;
    std::string strval = "A string";
    nanogui::Color colval(0.5f, 0.5f, 0.7f, 1.f);

    FormHelper *gui = new FormHelper(mScreen);
    ref<Window> window = gui->addWindow(Eigen::Vector2i(10, 10), "Form helper example");
    gui->addGroup("Basic types");
    gui->addVariable("bool", bvar);
    gui->addVariable("string", strval);

    gui->addGroup("Validating fields");
    gui->addVariable("int", ivar);
    gui->addVariable("float", fvar);
    gui->addVariable("double", dvar);

    gui->addGroup("Other widgets");
    gui->addButton("A button", [](){ std::cout << "Button pressed." << std::endl; });

    mScreen->setVisible(true);
    mScreen->performLayout();
    window->center();


    // Init UI
    /*if (!imguiRenderGLInit("DroidSans.ttf")) {
        fprintf(stderr, "Could not init GUI renderer.\n");
        exit(EXIT_FAILURE);
    }*/

    mTimeSinceLastProfilingDisplay = glfwGetTime();
}

void Gui::displayLeftPane() {

    /*
    TestbedApplication& app = TestbedApplication::getInstance();

    const float scalingX = app.mWindowToFramebufferRatio.x;
    const float scalingY = app.mWindowToFramebufferRatio.y;

    int windowWidth, windowHeight;
    glfwGetWindowSize(mWindow, &windowWidth, &windowHeight);

    int scrollarea = 0;
    imguiBeginScrollArea(NULL, 0, app.mWindowToFramebufferRatio.y * (windowHeight - LEFT_PANE_HEADER_HEIGHT),
                         app.mWindowToFramebufferRatio.x * LEFT_PANE_WIDTH,
                         app.mWindowToFramebufferRatio.y * LEFT_PANE_HEADER_HEIGHT, &scrollarea);

    // ------ Header ----- //

    imguiHorizontalSpace(scalingX * 2.5);
    imguiVerticalSpace(scalingY * 5);
    imguiStartLine();

    const int button_width = app.mWindowToFramebufferRatio.x * 75.0f;
    const int button_height = app.mWindowToFramebufferRatio.y * 20.0f;

    // Play/Pause
    if (imguiButton(app.mTimer.isRunning() ? "Pause" : "Play", true, button_width, button_height, scalingX, scalingY)) {
        app.togglePlayPauseSimulation();
    }
    imguiHorizontalSpace(scalingX * 2.5);

    // Step
    if (imguiButton("Step", !app.mTimer.isRunning(), button_width, button_height, scalingX, scalingY)) {
        app.toggleTakeSinglePhysicsStep();
    }
    imguiHorizontalSpace(scalingX * 2.5);

    // Restart
    if (imguiButton("Restart", true, button_width, button_height, scalingX, scalingY)) {
        app.restartSimulation();
    }

    imguiEndLine();
    imguiVerticalSpace(scalingY * 35);

    imguiSeparatorLine();
    imguiVerticalSpace(scalingY * 2.5);
    imguiStartLine();

    // ----- Left Pane Tabs ----- //

    int widthButton = app.mWindowToFramebufferRatio.x * LEFT_PANE_WIDTH / 4.3;
    if (imguiButton("Scenes", true, widthButton, button_height, scalingX, scalingY)) {
        mLeftPane = SCENES;
    }
    imguiHorizontalSpace(scalingX * 2.5);

    if (imguiButton("Physics", true, widthButton, button_height, scalingX, scalingY)) {
        mLeftPane = PHYSICS;
    }
    imguiHorizontalSpace(scalingX * 2.5);

    if (imguiButton("Rendering", true, widthButton, button_height, scalingX, scalingY)) {
        mLeftPane = RENDERING;
    }
    imguiHorizontalSpace(scalingX * 2.5);

    if (imguiButton("Profiling", true, widthButton, button_height, scalingX, scalingY)) {
        mLeftPane = PROFILING;
    }

    imguiEndLine();
    imguiVerticalSpace(scalingY * (BUTTON_HEIGHT/2 + 15));
    imguiSeparatorLine();
    imguiEndScrollArea();

    // Display the left pane content
    switch(mLeftPane) {
        case SCENES: displayScenesPane(); break;
        case PHYSICS: displayPhysicsPane(); break;
        case RENDERING: displayRenderingPane(); break;
        case PROFILING: displayProfilingPane(); break;
    }
    */
}

// Display the list of scenes
void Gui::displayScenesPane() {

    /*
    TestbedApplication& app = TestbedApplication::getInstance();

    const float scalingX = app.mWindowToFramebufferRatio.x;
    const float scalingY = app.mWindowToFramebufferRatio.y;

    static int choice = 0;
    int startChoice = choice;
    int scrollarea = 1;
    int windowWidth, windowHeight;
    glfwGetWindowSize(mWindow, &windowWidth, &windowHeight);

    std::vector<Scene*> scenes = app.getScenes();

    imguiBeginScrollArea("Scenes", 0, 0,
                         app.mWindowToFramebufferRatio.x * LEFT_PANE_WIDTH,
                         app.mWindowToFramebufferRatio.y * (windowHeight - LEFT_PANE_HEADER_HEIGHT),
                         &scrollarea);

    imguiVerticalSpace(15);

    // For each scene
    for (int i=0; i<scenes.size(); i++) {

        // Display a radio button
        if (imguiCheck(scenes[i]->getName().c_str(), choice == i, true, scalingX, scalingY)) {
            choice = i;
        }
    }
    if (choice != startChoice) {
        app.switchScene(scenes[choice]);
    }

    imguiEndScrollArea();
    */
}

void Gui::displayPhysicsPane() {

    /*
    TestbedApplication& app = TestbedApplication::getInstance();

    const float scalingX = app.mWindowToFramebufferRatio.x;
    const float scalingY = app.mWindowToFramebufferRatio.y;

    int windowWidth, windowHeight;
    glfwGetWindowSize(mWindow, &windowWidth, &windowHeight);

    int scrollarea = 2;
    imguiBeginScrollArea("Physics Engine Parameters", 0, 0,
                         app.mWindowToFramebufferRatio.x * LEFT_PANE_WIDTH,
                         app.mWindowToFramebufferRatio.y * (windowHeight - LEFT_PANE_HEADER_HEIGHT),
                         &scrollarea);

    imguiVerticalSpace(10 * scalingY);

    // Enabled/Disable Sleeping
    bool toggle = imguiCheck("Sleeping enabled", app.mEngineSettings.isSleepingEnabled, true, scalingX, scalingY);
    if (toggle) {
        app.mEngineSettings.isSleepingEnabled = !app.mEngineSettings.isSleepingEnabled;
    }

    // Enabled/Disable Gravity
    toggle = imguiCheck("Gravity enabled", app.mEngineSettings.isGravityEnabled, true, scalingX, scalingY);
    if (toggle) {
        app.mEngineSettings.isGravityEnabled = !app.mEngineSettings.isGravityEnabled;
    }

    imguiVerticalSpace(10 * scalingY);

    // Timestep
    float timeStep = app.mEngineSettings.timeStep;
    if (imguiSlider("Timestep", &timeStep, 0.001f, 1.0f, 0.001f, true, scalingX, scalingY)) {
        app.mEngineSettings.timeStep = timeStep;
    }

    // Nb velocity solver iterations
    float nbVelocityIterations = static_cast<float>(app.mEngineSettings.nbVelocitySolverIterations);
    if (imguiSlider("Velocity Solver Iterations", &nbVelocityIterations, 1.0f, 100.0f, 1.0f, true, scalingX, scalingY)) {
        app.mEngineSettings.nbVelocitySolverIterations = static_cast<int>(nbVelocityIterations);
    }

    // Nb position solver iterations
    float nbPositionIterations = static_cast<float>(app.mEngineSettings.nbPositionSolverIterations);
    if (imguiSlider("Position Solver Iterations", &nbPositionIterations, 1.0f, 100.0f, 1.0f, true, scalingX, scalingY)) {
        app.mEngineSettings.nbPositionSolverIterations = static_cast<int>(nbPositionIterations);
    }

    // Time before sleep
    float timeBeforeSleep = app.mEngineSettings.timeBeforeSleep;
    if (imguiSlider("Time before sleep", &timeBeforeSleep, 0.0f, 60.0f, 0.5f, true, scalingX, scalingY)) {
        app.mEngineSettings.timeBeforeSleep = timeBeforeSleep;
    }

    // Sleep linear velocity
    float sleepLinearVelocity = app.mEngineSettings.sleepLinearVelocity;
    if (imguiSlider("Sleep linear velocity", &sleepLinearVelocity, 0.0f, 30.0f, 0.5f, true, scalingX, scalingY)) {
        app.mEngineSettings.sleepLinearVelocity = sleepLinearVelocity;
    }

    // Sleep angular velocity
    float sleepAngularVelocity = app.mEngineSettings.sleepAngularVelocity;
    if (imguiSlider("Sleep angular velocity", &sleepAngularVelocity, 0.0f, 30.0f, 0.5f, true, scalingX, scalingY)) {
        app.mEngineSettings.sleepAngularVelocity = sleepAngularVelocity;
    }

    // Gravity vector
    openglframework::Vector3 gravity = app.mEngineSettings.gravity;
    float gravityX = gravity.x, gravityY = gravity.y, gravityZ = gravity.z;
    if (imguiSlider("Gravity X", &gravityX, -50.0f, 50.0f, 0.5f, true, scalingX, scalingY)) {
        app.mEngineSettings.gravity.x = gravityX;
    }
    if (imguiSlider("Gravity Y", &gravityY, -50.0f, 50.0f, 0.5f, true, scalingX, scalingY)) {
        app.mEngineSettings.gravity.y = gravityY;
    }
    if (imguiSlider("Gravity Z", &gravityZ, -50.0f, 50.0f, 0.5f, true, scalingX, scalingY)) {
        app.mEngineSettings.gravity.z = gravityZ;
    }

    imguiEndScrollArea();

    */
}

void Gui::displayRenderingPane() {

    /*
    TestbedApplication& app = TestbedApplication::getInstance();

    const float scalingX = app.mWindowToFramebufferRatio.x;
    const float scalingY = app.mWindowToFramebufferRatio.y;

    int windowWidth, windowHeight;
    glfwGetWindowSize(mWindow, &windowWidth, &windowHeight);

    int scrollarea = 2;
    imguiBeginScrollArea("Rendering Parameters", 0, 0,
                         app.mWindowToFramebufferRatio.x * LEFT_PANE_WIDTH,
                         app.mWindowToFramebufferRatio.y * (windowHeight - LEFT_PANE_HEADER_HEIGHT),
                         &scrollarea);

    imguiVerticalSpace(15);

    // Display/Hide contact points
    bool toggleContactPoints = imguiCheck("Contacts", app.mIsContactPointsDisplayed, true, scalingX, scalingY);
    if (toggleContactPoints) {
        app.displayContactPoints(!app.mIsContactPointsDisplayed);
    }

    // Enabled/Disable VSync
    bool toggleVSync = imguiCheck("V Sync", app.mIsVSyncEnabled, true, scalingX, scalingY);
    if (toggleVSync) {
        app.enableVSync(!app.mIsVSyncEnabled);
    }

    // Enabled/Disable Shadows
    bool toggleShadows = imguiCheck("Shadows", app.mIsShadowMappingEnabled, true, scalingX, scalingY);
    if (toggleShadows) {
        app.enableShadows(!app.mIsShadowMappingEnabled);
    }

    imguiEndScrollArea();
    */
}

void Gui::displayProfilingPane() {

    /*
    TestbedApplication& app = TestbedApplication::getInstance();

    const float scalingX = app.mWindowToFramebufferRatio.x;
    const float scalingY = app.mWindowToFramebufferRatio.y;

    double currentTime = glfwGetTime();
    if ((currentTime - mTimeSinceLastProfilingDisplay)  > TIME_INTERVAL_DISPLAY_PROFILING_INFO) {
        mTimeSinceLastProfilingDisplay = currentTime;
        mCachedFPS = app.mFPS;
        mCachedUpdateTime = app.mUpdateTime;
        mCachedPhysicsUpdateTime = app.mPhysicsUpdateTime;
    }

    int windowWidth, windowHeight;
    glfwGetWindowSize(mWindow, &windowWidth, &windowHeight);

    int scrollarea = 2;
    imguiBeginScrollArea("Profiling", 0, 0,
                         app.mWindowToFramebufferRatio.x * LEFT_PANE_WIDTH,
                         app.mWindowToFramebufferRatio.y * (windowHeight - LEFT_PANE_HEADER_HEIGHT),
                         &scrollarea);

    imguiVerticalSpace(15);

    // Framerate (FPS)
    std::stringstream ss;
    ss << std::setprecision(4) << mCachedFPS;
    std::string fps = std::string("FPS : ") + ss.str();
    imguiItem(fps.c_str(), true, scalingX, scalingY);

    // Update time
    std::stringstream ss1;
    double updateTime = mCachedUpdateTime * 1000.0;
    ss1 << std::setprecision(4) << updateTime;
    std::string updateTimeStr = std::string("Update time (ms) : ") + ss1.str();
    imguiItem(updateTimeStr.c_str(), true, scalingX, scalingY);

    // Update time (physics)
    std::stringstream ss2;
    ss2 << std::setprecision(4) << (mCachedPhysicsUpdateTime * 1000.0);
    std::string updatePhysicsTimeStr = std::string("Update physics time (ms) : ") + ss2.str();
    imguiItem(updatePhysicsTimeStr.c_str(), true, scalingX, scalingY);

    imguiEndScrollArea();
    */
}

// Display the GUI
void Gui::render() {

    /*
    TestbedApplication& app = TestbedApplication::getInstance();

    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glDisable(GL_DEPTH_TEST);

    int display_w, display_h;
    glfwGetFramebufferSize(mWindow, &display_w, &display_h);

    int windowWidth, windowHeight;
    glfwGetWindowSize(mWindow, &windowWidth, &windowHeight);

    // Mouse position
    double mouseX, mouseY;
    glfwGetCursorPos(mWindow, &mouseX, &mouseY);

    // Mouse buttons
    int leftButton = glfwGetMouseButton(mWindow, GLFW_MOUSE_BUTTON_LEFT);
    unsigned char mousebutton = 0;
    if(leftButton == GLFW_PRESS ) {
        mousebutton |= IMGUI_MBUT_LEFT;
    }

    imguiBeginFrame(app.mWindowToFramebufferRatio.x * mouseX,
                    app.mWindowToFramebufferRatio.y * (windowHeight - mouseY), mousebutton,
                    - app.mWindowToFramebufferRatio.y * mScrollY);
    resetScroll();

    //displayHeader();
    displayLeftPane();

    imguiEndFrame();

    imguiRenderGLDraw(display_w, display_h);

    */
}
