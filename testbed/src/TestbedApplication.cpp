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
#include "TestbedApplication.h"
#include "openglframework.h"
#include <iostream>
#include <cstdlib>
#include <sstream>
#include "cubes/CubesScene.h"
#include "collisiondetection/CollisionDetectionScene.h"
#include "joints/JointsScene.h"
#include "collisionshapes/CollisionShapesScene.h"
#include "heightfield/HeightFieldScene.h"
#include "raycast/RaycastScene.h"
#include "concavemesh/ConcaveMeshScene.h"
#include "cubestack/CubeStackScene.h"
#include "pile/PileScene.h"
#include "boxtower/BoxTowerScene.h"
#include "ballandsocketjointsnet/BallAndSocketJointsNetScene.h"
#include "ballandsocketjointschain/BallAndSocketJointsChainScene.h"
#include "hingejointschain/HingeJointsChainScene.h"
#include "bridge/BridgeScene.h"
#include "fixedjoint/FixedJointScene.h"
#include "ballandsocketjoint/BallAndSocketJointScene.h"
#include "hingejoint/HingeJointScene.h"
#include "sliderjoint/SliderJointScene.h"
#include "ragdoll/RagdollScene.h"
#include "rope/RopeScene.h"

using namespace openglframework;
using namespace jointsscene;
using namespace cubesscene;
using namespace raycastscene;
using namespace collisionshapesscene;
using namespace trianglemeshscene;
using namespace heightfieldscene;
using namespace collisiondetectionscene;
using namespace cubestackscene;
using namespace pilescene;
using namespace boxtowerscene;
using namespace ballandsocketjointsnetscene;
using namespace ballandsocketjointschainscene;
using namespace hingejointschainscene;
using namespace bridgescene;
using namespace fixedjointscene;
using namespace ballandsocketjointscene;
using namespace hingejointscene;
using namespace sliderjointscene;
using namespace ragdollscene;
using namespace ropescene;

// Initialization of static variables
const float TestbedApplication::SCROLL_SENSITIVITY = 0.08f;

// Constructor
TestbedApplication::TestbedApplication()
                   : mIsInitialized(false), mGui(this), mCurrentScene(nullptr),
                     mDefaultEngineSettings(EngineSettings::defaultSettings()),
                     mFPS(0), mNbFrames(0), mPreviousTime(0),
                     mLastTimeComputedFPS(0), mFrameTime(0), mTotalPhysicsTime(0), mPhysicsStepTime(0),
                     mSinglePhysicsStepEnabled(false), mSinglePhysicsStepDone(false),
                     mWindowToFramebufferRatio(Vector2(1, 1)), mIsShadowMappingEnabled(true),
                     mAreContactPointsDisplayed(false), mAreContactNormalsDisplayed(false),
                     mAreBroadPhaseAABBsDisplayed(false), mAreCollidersAABBsDisplayed(false),
                     mAreCollisionShapesDisplayed(false), mAreCollisionShapesNormalsDisplayed(false),
                     mAreObjectsWireframeEnabled(false), mIsVSyncEnabled(false), mIsDebugRendererEnabled(false) {

    init();
}

// Destructor
TestbedApplication::~TestbedApplication() {

    // Destroy all the scenes
    destroyScenes();
}

// Initialize the viewer
void TestbedApplication::start() {

    glfwInit();
    glfwSetTime(0);

    // Get the primary monitor
    GLFWmonitor* monitor = glfwGetPrimaryMonitor();
    const GLFWvidmode* mode = glfwGetVideoMode(monitor);

    // Window size
    mWidth = mode->width;
    mHeight = mode->height;

#if defined(NANOGUI_USE_OPENGL)
    glfwWindowHint(GLFW_CLIENT_API, GLFW_OPENGL_API);

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
#elif defined(NANOGUI_USE_GLES)
    glfwWindowHint(GLFW_CLIENT_API, GLFW_OPENGL_API);

    glfwWindowHint(GLFW_CONTEXT_CREATION_API, GLFW_EGL_CONTEXT_API);
#elif defined(NANOGUI_USE_METAL)
    glfwWindowHint(GLFW_CLIENT_API, GLFW_NO_API);

    metal_init();
#endif

    glfwWindowHint(GLFW_SAMPLES, 0);
    glfwWindowHint(GLFW_RED_BITS, 8);
    glfwWindowHint(GLFW_GREEN_BITS, 8);
    glfwWindowHint(GLFW_BLUE_BITS, 8);
    glfwWindowHint(GLFW_ALPHA_BITS, 8);
    glfwWindowHint(GLFW_STENCIL_BITS, 8);
    glfwWindowHint(GLFW_DEPTH_BITS, 24);
    glfwWindowHint(GLFW_RESIZABLE, GL_TRUE);

    // Create a the GLFW window object
    std::string title = "Testbed - ReactPhysics3D v" + rp3d::RP3D_VERSION;
    mWindow = glfwCreateWindow(mWidth, mHeight, title.c_str(), IS_FULLSCREEN ? monitor : nullptr, nullptr);
    if (mWindow == nullptr) {
        std::cout << "Failed to create GLFW window" << std::endl;
        glfwTerminate();
    }
    glfwMakeContextCurrent(mWindow);

#if defined(NANOGUI_GLAD)
 if (!gladLoadGLLoader((GLADloadproc) glfwGetProcAddress))
     throw std::runtime_error("Could not initialize GLAD!");
 glGetError(); // pull and ignore unhandled errors like GL_INVALID_ENUM
#endif

    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);

    // Logger
    rp3d::PhysicsCommon::setLogger(&mLogger);

    // Create all the scenes
    createScenes();

    // Initialize the GUI
    mGui.init(mWindow);

#if defined(NANOGUI_USE_OPENGL) || defined(NANOGUI_USE_GLES)
    int width, height;
    glfwGetFramebufferSize(mWindow, &width, &height);
    glViewport(0, 0, width, height);
    glfwSwapInterval(0);
    glfwSwapBuffers(mWindow);
#endif

    // Select the initial scene
    const int firstSceneIndex = 0;
    switchScene(mScenes[firstSceneIndex]);

    mGui.drawAll();

    mTimer.start();

    int glMajorVersion, glMinorVersion;
    glGetIntegerv(GL_MAJOR_VERSION, &glMajorVersion);
    glGetIntegerv(GL_MINOR_VERSION, &glMinorVersion);

#ifdef GL_DEBUG_OUTPUT

    if (glMajorVersion > 4 || (glMajorVersion == 4 && glMinorVersion >= 3)) {

        // Enable OpenGL error reporting
        glEnable(GL_DEBUG_OUTPUT);
        glEnable(GL_DEBUG_OUTPUT_SYNCHRONOUS);
        glDebugMessageCallback(onOpenGLError, 0);
    }
#endif

    glfwSetWindowUserPointer(mWindow, this);

    glfwSetCursorPosCallback(mWindow, [](GLFWwindow* window, double x, double y) {
         TestbedApplication* app = static_cast<TestbedApplication*>(glfwGetWindowUserPointer(window));
         app->mouse_motion_event(x, y);
        }
    );

    glfwSetMouseButtonCallback(mWindow, [](GLFWwindow* window, int button, int action, int modifiers) {
         TestbedApplication* app = static_cast<TestbedApplication*>(glfwGetWindowUserPointer(window));
         app->mouse_button_event(button, action, modifiers);
        }
    );

    glfwSetKeyCallback(mWindow, [](GLFWwindow* window, int key, int scancode, int action, int mods) {
         TestbedApplication* app = static_cast<TestbedApplication*>(glfwGetWindowUserPointer(window));
         app->keyboard_event(key, scancode, action, mods);
        }
    );

    glfwSetScrollCallback(mWindow, [](GLFWwindow* window, double x, double y) {
         TestbedApplication* app = static_cast<TestbedApplication*>(glfwGetWindowUserPointer(window));
         app->scroll_event(x, y);
       }
    );

    glfwSetFramebufferSizeCallback(mWindow, [](GLFWwindow* window, int width, int height) {
            TestbedApplication* app = static_cast<TestbedApplication*>(glfwGetWindowUserPointer(window));
            app->onWindowResized(width, height);
        }
    );

    mCurrentScene->reshape(mWidth, mHeight);
    mCurrentScene->setWindowDimension(mWidth, mHeight);

    mIsInitialized = true;

    // Game loop
   while (!glfwWindowShouldClose(mWindow)) {

       // Check if any events have been activated (key pressed, mouse moved etc.) and call corresponding response functions
       glfwPollEvents();

       update();

       mGui.update();

       // Draw nanogui
       mGui.draw();

       render();

       mGui.drawTearDown();
   }

   // Terminate GLFW, clearing any resources allocated by GLFW.
   glfwTerminate();

#if defined(NANOGUI_USE_METAL)
    metal_shutdown();
#endif

}

// Create all the scenes
void TestbedApplication::createScenes() {

    uint logLevel = static_cast<uint>(rp3d::Logger::Level::Information);

    // Cubes scene
    std::string sceneName = "Cubes";
    mLogger.addFileDestination(sceneName, logLevel, rp3d::DefaultLogger::Format::HTML);
    CubesScene* cubeScene = new CubesScene(sceneName, mDefaultEngineSettings, mPhysicsCommon);
    mScenes.push_back(cubeScene);

    // Cube Stack scene
    sceneName = "Cube Stack";
    mLogger.addFileDestination(sceneName, logLevel, rp3d::DefaultLogger::Format::HTML);
    CubeStackScene* cubeStackScene = new CubeStackScene(sceneName, mDefaultEngineSettings, mPhysicsCommon);
    mScenes.push_back(cubeStackScene);

    // Joints scene
    sceneName = "Joints";
    mLogger.addFileDestination(sceneName, logLevel, rp3d::DefaultLogger::Format::HTML);
    JointsScene* jointsScene = new JointsScene(sceneName, mDefaultEngineSettings, mPhysicsCommon);
    mScenes.push_back(jointsScene);

    // Collision shapes scene
    sceneName = "Collision Shapes";
    mLogger.addFileDestination(sceneName, logLevel, rp3d::DefaultLogger::Format::HTML);
    CollisionShapesScene* collisionShapesScene = new CollisionShapesScene(sceneName, mDefaultEngineSettings, mPhysicsCommon);
    mScenes.push_back(collisionShapesScene);

    // Heightfield shape scene
    sceneName = "Heightfield";
    mLogger.addFileDestination(sceneName, logLevel, rp3d::DefaultLogger::Format::HTML);
    HeightFieldScene* heightFieldScene = new HeightFieldScene(sceneName, mDefaultEngineSettings, mPhysicsCommon);
    mScenes.push_back(heightFieldScene);

    // Raycast scene
    sceneName = "Raycast";
    mLogger.addFileDestination(sceneName, logLevel, rp3d::DefaultLogger::Format::HTML);
    RaycastScene* raycastScene = new RaycastScene(sceneName, mDefaultEngineSettings, mPhysicsCommon);
    mScenes.push_back(raycastScene);

    // Concave Mesh scene
    sceneName = "Concave Mesh";
    mLogger.addFileDestination(sceneName, logLevel, rp3d::DefaultLogger::Format::HTML);
    ConcaveMeshScene* concaveMeshScene = new ConcaveMeshScene(sceneName, mDefaultEngineSettings, mPhysicsCommon);
    mScenes.push_back(concaveMeshScene);

    // Pile scene
    sceneName = "Pile";
    mLogger.addFileDestination(sceneName, logLevel, rp3d::DefaultLogger::Format::HTML);
    PileScene* pileScene = new PileScene(sceneName, mDefaultEngineSettings, mPhysicsCommon);
    mScenes.push_back(pileScene);

    // Ball and Socket joint scene
    sceneName = "Ball and Socket joint";
    mLogger.addFileDestination(sceneName, logLevel, rp3d::DefaultLogger::Format::HTML);
    BallAndSocketJointScene* ballAndSocketJointScene = new BallAndSocketJointScene(sceneName, mDefaultEngineSettings, mPhysicsCommon);
    mScenes.push_back(ballAndSocketJointScene);

    // Box Tower scene
    sceneName = "Box Tower";
    mLogger.addFileDestination(sceneName, logLevel, rp3d::DefaultLogger::Format::HTML);
    BoxTowerScene* boxTowerScene = new BoxTowerScene(sceneName, mDefaultEngineSettings, mPhysicsCommon);
    mScenes.push_back(boxTowerScene);

    // Ball and Socket joints Net scene
    sceneName = "BallSocket Joints Net";
    mLogger.addFileDestination(sceneName, logLevel, rp3d::DefaultLogger::Format::HTML);
    BallAndSocketJointsNetScene* ballAndSocketJointsNetScene = new BallAndSocketJointsNetScene(sceneName, mDefaultEngineSettings, mPhysicsCommon);
    mScenes.push_back(ballAndSocketJointsNetScene);

    // Ball and Socket joints chain scene
    sceneName = "BallSocket Joints Chain";
    mLogger.addFileDestination(sceneName, logLevel, rp3d::DefaultLogger::Format::HTML);
    BallAndSocketJointsChainScene* ballAndSocketJointsChainScene = new BallAndSocketJointsChainScene(sceneName, mDefaultEngineSettings, mPhysicsCommon);
    mScenes.push_back(ballAndSocketJointsChainScene);

    // Hinge joints chain scene
    sceneName = "Hinge Joints Chain";
    mLogger.addFileDestination(sceneName, logLevel, rp3d::DefaultLogger::Format::HTML);
    HingeJointsChainScene* hingeJointsChainScene = new HingeJointsChainScene(sceneName, mDefaultEngineSettings, mPhysicsCommon);
    mScenes.push_back(hingeJointsChainScene);

    // Bridge scene
    sceneName = "Bridge";
    mLogger.addFileDestination(sceneName, logLevel, rp3d::DefaultLogger::Format::HTML);
    BridgeScene* bridgeScene = new BridgeScene(sceneName, mDefaultEngineSettings, mPhysicsCommon);
    mScenes.push_back(bridgeScene);

    // Fixed joint scene
    sceneName = "Fixed joint";
    mLogger.addFileDestination(sceneName, logLevel, rp3d::DefaultLogger::Format::HTML);
    FixedJointScene* fixedJointScene = new FixedJointScene(sceneName, mDefaultEngineSettings, mPhysicsCommon);
    mScenes.push_back(fixedJointScene);

    // Hinge joint scene
    sceneName = "Hinge joint";
    mLogger.addFileDestination(sceneName, logLevel, rp3d::DefaultLogger::Format::HTML);
    HingeJointScene* hingeJointScene = new HingeJointScene(sceneName, mDefaultEngineSettings, mPhysicsCommon);
    mScenes.push_back(hingeJointScene);

    // Slider joint scene
    sceneName = "Slider joint";
    mLogger.addFileDestination(sceneName, logLevel, rp3d::DefaultLogger::Format::HTML);
    SliderJointScene* sliderJointScene = new SliderJointScene(sceneName, mDefaultEngineSettings, mPhysicsCommon);
    mScenes.push_back(sliderJointScene);

    // Ragdoll scene
    sceneName = "Ragdoll";
    mLogger.addFileDestination(sceneName, logLevel, rp3d::DefaultLogger::Format::HTML);
    RagdollScene* ragdollScene = new RagdollScene(sceneName, mDefaultEngineSettings, mPhysicsCommon);
    mScenes.push_back(ragdollScene);

    // Rope scene
    sceneName = "Rope";
    mLogger.addFileDestination(sceneName, logLevel, rp3d::DefaultLogger::Format::HTML);
    RopeScene* ropeScene = new RopeScene(sceneName, mDefaultEngineSettings, mPhysicsCommon);
    mScenes.push_back(ropeScene);

    // Collision Detection scene
    sceneName = "Collision Detection";
    mLogger.addFileDestination(sceneName, logLevel, rp3d::DefaultLogger::Format::HTML);
    CollisionDetectionScene* collisionDetectionScene = new CollisionDetectionScene(sceneName, mDefaultEngineSettings, mPhysicsCommon);
    mScenes.push_back(collisionDetectionScene);

    assert(mScenes.size() > 0);
}

// Remove all the scenes
void TestbedApplication::destroyScenes() {

    for (uint i=0; i<mScenes.size(); i++) {
        delete mScenes[i];
    }

    mCurrentScene = NULL;
}

void TestbedApplication::updateSinglePhysicsStep() {

    assert(!mTimer.isRunning());

    mCurrentScene->updatePhysics();
}

// Update the physics of the current scene
void TestbedApplication::updatePhysics() {

    // Update the elapsed time
    mCurrentScene->getEngineSettings().elapsedTime = mTimer.getElapsedPhysicsTime();

    if (mTimer.isRunning()) {

        // Compute the time since the last update() call and update the timer
        mTimer.update();

        // While the time accumulator is not empty
        while(mTimer.isPossibleToTakeStep(mCurrentScene->getEngineSettings().timeStep)) {

            double currentTime = glfwGetTime();

            // Take a physics simulation step
            mCurrentScene->updatePhysics();

            mPhysicsStepTime = glfwGetTime() - currentTime;

            // Update the timer
            mTimer.nextStep(mCurrentScene->getEngineSettings().timeStep);
        }
    }
}

void TestbedApplication::update() {

    double currentTime = glfwGetTime();

    mCurrentScene->setIsDebugRendererEnabled(mIsDebugRendererEnabled);

    // Update the physics
    if (mSinglePhysicsStepEnabled && !mSinglePhysicsStepDone) {
        updateSinglePhysicsStep();
        mSinglePhysicsStepDone = true;
    }
    else {
        updatePhysics();
    }

    // Compute the physics update time
    mTotalPhysicsTime = glfwGetTime() - currentTime;

    // Compute the interpolation factor
    float factor = mTimer.computeInterpolationFactor(mDefaultEngineSettings.timeStep);
    assert(factor >= 0.0f && factor <= 1.0f);

    // Notify the scene about the interpolation factor
    mCurrentScene->setInterpolationFactor(factor);

    // Enable/Disable shadow mapping
    mCurrentScene->setIsShadowMappingEnabled(mIsShadowMappingEnabled);

    // Display/Hide contact points
    mCurrentScene->setAreContactPointsDisplayed(mAreContactPointsDisplayed);

    // Display/Hide contact normals
    mCurrentScene->setAreContactNormalsDisplayed(mAreContactNormalsDisplayed);

    // Display/Hide the broad phase AABBs
    mCurrentScene->setAreBroadPhaseAABBsDisplayed(mAreBroadPhaseAABBsDisplayed);

    // Display/Hide the colliders AABBs
    mCurrentScene->setAreCollidersAABBsDisplayed(mAreCollidersAABBsDisplayed);

    // Display/Hide the collision shapes
    mCurrentScene->setAreCollisionShapesDisplayed(mAreCollisionShapesDisplayed);

    // Display/Hide the collision shapes normals
    mCurrentScene->setAreCollisionShapesNormalsDisplayed(mAreCollisionShapesNormalsDisplayed);

    // Enable/Disable wireframe mode
    mCurrentScene->setIsWireframeEnabled(mAreObjectsWireframeEnabled);

    // Update the scene
    mCurrentScene->update();

    // Compute the current framerate
    computeFPS();
}

void TestbedApplication::render() {

    int bufferWidth, bufferHeight;
    glfwMakeContextCurrent(mWindow);
    glfwGetFramebufferSize(mWindow, &bufferWidth, &bufferHeight);

    // Set the viewport of the scene
    mCurrentScene->setViewport(0, 0, bufferWidth, bufferHeight);

    // Render the scene
    mCurrentScene->render();
}

/// Window resize event handler
bool TestbedApplication::onWindowResized(int width, int height) {

    if (!mIsInitialized) return false;

    mGui.onWindowResizeEvent(width, height);

    // Resize the camera viewport
    mCurrentScene->reshape(width, height);

    // Update the window size of the scene
    mCurrentScene->setWindowDimension(width, height);

    mWidth = width;
    mHeight = height;

    return true;
}

// Change the current scene
void TestbedApplication::switchScene(Scene* newScene) {

    if (newScene == mCurrentScene) return;

    mCurrentScene = newScene;

    mTimer.reset();

    // Resize the camera viewport
    mCurrentScene->reshape(mWidth, mHeight);

    // Update the window size of the scene
    mCurrentScene->setWindowDimension(mWidth, mHeight);

    // Reset the scene
    mCurrentScene->reset();

    mCurrentScene->updateEngineSettings();

    mGui.resetWithValuesFromCurrentScene();
}

// Notify that the engine settings have changed
void TestbedApplication::notifyEngineSetttingsChanged() {
   mCurrentScene->updateEngineSettings();
}

void GLAPIENTRY TestbedApplication::onOpenGLError(GLenum source, GLenum type, GLuint /*id*/, GLenum severity, GLsizei /*length*/,
                              const GLchar* message, const void* /*userParam*/ ) {

#ifdef GL_DEBUG_OUTPUT
    if (type == GL_DEBUG_TYPE_ERROR) {
        fprintf( stderr, "GL CALLBACK: %s source = 0x%x, type = 0x%x, severity = 0x%x, message = %s\n",
                   ("** GL ERROR **" ),
                    source, type, severity, message );
    }
#endif

}

// Compute the FPS
void TestbedApplication::computeFPS() {

    // Note : By default the nanogui library is using glfwWaitEvents() to process
    //        events and sleep to target a framerate of 50 ms (using a thread
    //        sleeping). However, for games we prefer to use glfwPollEvents()
    //        instead and remove the update. Therefore the file common.cpp of the
    //        nanogui library has been modified to have a faster framerate

    mNbFrames++;

    //  Get the number of seconds since start
    mCurrentTime = glfwGetTime();

    //  Calculate time passed
    mFrameTime = mCurrentTime - mPreviousTime;
    double timeInterval = (mCurrentTime - mLastTimeComputedFPS) * 1000.0;

    // Update the FPS counter each second
    if(timeInterval > 1000) {

        //  calculate the number of frames per second
        mFPS = static_cast<double>(mNbFrames) / timeInterval;
        mFPS *= 1000.0;

        //  Reset frame count
        mNbFrames = 0;

        mLastTimeComputedFPS = mCurrentTime;
    }

    //  Set time
    mPreviousTime = mCurrentTime;
}

void TestbedApplication::keyboard_event(int key, int scancode, int action, int modifiers) {

    mGui.onKeyboardEvent(key, scancode, action, modifiers);

    // Close application on escape key
    if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS) {
        glfwSetWindowShouldClose(mWindow, GL_TRUE);
        return;
    }

    // Show/hide the GUI with "i" key
    if (key == GLFW_KEY_I && action == GLFW_PRESS) {
        mGui.setIsDisplayed(!mGui.getIsDisplayed());
        return;
    }

    // Start/Stop camera rotation animation with "r" key
    if (key == GLFW_KEY_R && action == GLFW_PRESS) {
        mCurrentScene->setIsCameraRotationAnimationEnabled(!mCurrentScene->getIsCameraRotationAnimationEnabled());
        return;
    }

    // Pause the application on "p" key
    if (key == GLFW_KEY_P && action == GLFW_PRESS) {

        if (mTimer.isRunning()) {
            pauseSimulation();
        }
        else {
           playSimulation();
        }

        return;
    }

    mCurrentScene->keyboardEvent(key, scancode, action, modifiers);
}

// Handle a mouse button event (default implementation: propagate to children)
void TestbedApplication::mouse_button_event(int button, int action, int modifiers) {

    mGui.onMouseButtonEvent(button, action, modifiers);

    // Get the mouse cursor position
    double x, y;
    glfwGetCursorPos(mWindow, &x, &y);

    bool down = action == GLFW_PRESS;

    mCurrentScene->mouseButtonEvent(button, down, modifiers, x, y);
}

// Handle a mouse motion event (default implementation: propagate to children)
void TestbedApplication::mouse_motion_event(double x, double y) {

    mGui.onMouseMotionEvent(x, y);

    int leftButtonState = glfwGetMouseButton(mWindow, GLFW_MOUSE_BUTTON_LEFT);
    int rightButtonState = glfwGetMouseButton(mWindow, GLFW_MOUSE_BUTTON_RIGHT);
    int middleButtonState = glfwGetMouseButton(mWindow, GLFW_MOUSE_BUTTON_MIDDLE);
    int altKeyState = glfwGetKey(mWindow, GLFW_KEY_LEFT_ALT);

    mCurrentScene->mouseMotionEvent(x, y, leftButtonState, rightButtonState, middleButtonState, altKeyState);
}

// Handle a mouse scroll event
void TestbedApplication::scroll_event(double x, double y) {

    if (mGui.onScrollEvent(x, y)) {
        return;
    };

    mCurrentScene->scrollingEvent(x, y, SCROLL_SENSITIVITY);
}
