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

// Initialization of static variables
const float TestbedApplication::SCROLL_SENSITIVITY = 0.08f;

// Constructor
TestbedApplication::TestbedApplication(bool isFullscreen, int windowWidth, int windowHeight)
                   : Screen(Vector2i(windowWidth, windowHeight), "Testbed ReactPhysics3D", true, isFullscreen, true, true, false, 4, 1),
                     mIsInitialized(false), mGui(this), mCurrentScene(nullptr),
                     mEngineSettings(EngineSettings::defaultSettings()),
                     mFPS(0), mNbFrames(0), mPreviousTime(0),
                     mLastTimeComputedFPS(0), mFrameTime(0), mTotalPhysicsTime(0), mPhysicsStepTime(0),
                     mWidth(windowWidth), mHeight(windowHeight),
                     mSinglePhysicsStepEnabled(false), mSinglePhysicsStepDone(false),
                     mWindowToFramebufferRatio(Vector2(1, 1)), mIsShadowMappingEnabled(true),
                     mAreContactPointsDisplayed(false), mAreContactNormalsDisplayed(false),
                     mAreBroadPhaseAABBsDisplayed(false), mAreCollidersAABBsDisplayed(false),
                     mAreCollisionShapesDisplayed(false), mAreObjectsWireframeEnabled(false),
                     mIsVSyncEnabled(false), mIsDebugRendererEnabled(false) {

    init();

    resize_event(Vector2i(0, 0));
}

// Destructor
TestbedApplication::~TestbedApplication() {

    // Destroy all the scenes
    destroyScenes();
}

// Initialize the viewer
void TestbedApplication::init() {

    // Create all the scenes
    createScenes();

    // Initialize the GUI
    mGui.init();

    mTimer.start();

    // Enable OpenGL error reporting
    glEnable(GL_DEBUG_OUTPUT);
    glDebugMessageCallback(onOpenGLError, 0);

    mIsInitialized = true;
}

// Create all the scenes
void TestbedApplication::createScenes() {

    // Cubes scene
    CubesScene* cubeScene = new CubesScene("Cubes", mEngineSettings);
    mScenes.push_back(cubeScene);

    // Cube Stack scene
    CubeStackScene* cubeStackScene = new CubeStackScene("Cube Stack", mEngineSettings);
    mScenes.push_back(cubeStackScene);

    // Joints scene
    JointsScene* jointsScene = new JointsScene("Joints", mEngineSettings);
    mScenes.push_back(jointsScene);

    // Collision shapes scene
    CollisionShapesScene* collisionShapesScene = new CollisionShapesScene("Collision Shapes", mEngineSettings);
    mScenes.push_back(collisionShapesScene);

    // Heightfield shape scene
    HeightFieldScene* heightFieldScene = new HeightFieldScene("Heightfield", mEngineSettings);
    mScenes.push_back(heightFieldScene);

    // Raycast scene
    RaycastScene* raycastScene = new RaycastScene("Raycast", mEngineSettings);
    mScenes.push_back(raycastScene);

    // Collision Detection scene
    CollisionDetectionScene* collisionDetectionScene = new CollisionDetectionScene("Collision Detection", mEngineSettings);
    mScenes.push_back(collisionDetectionScene);

    // Concave Mesh scene
    ConcaveMeshScene* concaveMeshScene = new ConcaveMeshScene("Concave Mesh", mEngineSettings);
    mScenes.push_back(concaveMeshScene);

    // Pile scene
    PileScene* pileScene = new PileScene("Pile", mEngineSettings);
    mScenes.push_back(pileScene);
    assert(mScenes.size() > 0);

    const int firstSceneIndex = 0;

    switchScene(mScenes[firstSceneIndex]);
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
    mEngineSettings.elapsedTime = mTimer.getPhysicsTime();

    if (mTimer.isRunning()) {

        // Compute the time since the last update() call and update the timer
        mTimer.update();

        // While the time accumulator is not empty
        while(mTimer.isPossibleToTakeStep(mEngineSettings.timeStep)) {

            double currentTime = glfwGetTime();

            // Take a physics simulation step
            mCurrentScene->updatePhysics();

            mPhysicsStepTime = glfwGetTime() - currentTime;

            // Update the timer
            mTimer.nextStep(mEngineSettings.timeStep);
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
    float factor = mTimer.computeInterpolationFactor(mEngineSettings.timeStep);
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

    // Enable/Disable wireframe mode
    mCurrentScene->setIsWireframeEnabled(mAreObjectsWireframeEnabled);

    // Update the scene
    mCurrentScene->update();
}

void TestbedApplication::draw_contents() {

    update();

    int bufferWidth, bufferHeight;
    glfwMakeContextCurrent(m_glfw_window);
    glfwGetFramebufferSize(m_glfw_window, &bufferWidth, &bufferHeight);

    // Set the viewport of the scene
    mCurrentScene->setViewport(0, 0, bufferWidth, bufferHeight);

    // Render the scene
    mCurrentScene->render();

    mGui.update();

    // Compute the current framerate
    computeFPS();
}

/// Window resize event handler
bool TestbedApplication::resize_event(const Vector2i &size) {

    if (!mIsInitialized) return false;

    // Get the framebuffer dimension
    int width, height;
    glfwGetFramebufferSize(m_glfw_window, &width, &height);

    // Resize the camera viewport
    mCurrentScene->reshape(width, height);

    // Update the window size of the scene
    int windowWidth, windowHeight;
    glfwGetWindowSize(m_glfw_window, &windowWidth, &windowHeight);
    mCurrentScene->setWindowDimension(windowWidth, windowHeight);

    return true;
}

// Change the current scene
void TestbedApplication::switchScene(Scene* newScene) {

    if (newScene == mCurrentScene) return;

    mCurrentScene = newScene;

    // Reset the scene
    mCurrentScene->reset();

    mCurrentScene->updateEngineSettings();

    resize_event(Vector2i(0, 0));
}

// Notify that the engine settings have changed
void TestbedApplication::notifyEngineSetttingsChanged() {
   mCurrentScene->updateEngineSettings();
}

void GLAPIENTRY TestbedApplication::onOpenGLError(GLenum source, GLenum type, GLuint id, GLenum severity, GLsizei length,
                              const GLchar* message, const void* userParam ) {

    if (type == GL_DEBUG_TYPE_ERROR) {
        /*
        fprintf( stderr, "GL CALLBACK: %s type = 0x%x, severity = 0x%x, message = %s\n",
                   ("** GL ERROR **" ),
                    type, severity, message );
         */
    }
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

bool TestbedApplication::keyboard_event(int key, int scancode, int action, int modifiers) {

    if (Screen::keyboard_event(key, scancode, action, modifiers)) {
        return true;
    }

    // Close application on escape key
    if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS) {
        glfwSetWindowShouldClose(m_glfw_window, GL_TRUE);
        return true;
    }

    return mCurrentScene->keyboardEvent(key, scancode, action, modifiers);
}

// Handle a mouse button event (default implementation: propagate to children)
bool TestbedApplication::mouse_button_event(const Vector2i &p, int button, bool down, int modifiers) {

    if (Screen::mouse_button_event(p, button, down, modifiers)) {
        return true;
    }

    // Get the mouse cursor position
    double x, y;
    glfwGetCursorPos(m_glfw_window, &x, &y);

    return mCurrentScene->mouseButtonEvent(button, down, modifiers, x, y);
}

// Handle a mouse motion event (default implementation: propagate to children)
bool TestbedApplication::mouse_motion_event(const Vector2i &p, const Vector2i &rel, int button, int modifiers) {

    if (Screen::mouse_motion_event(p, rel, button, modifiers)) {
        return true;
    }

    int leftButtonState = glfwGetMouseButton(m_glfw_window, GLFW_MOUSE_BUTTON_LEFT);
    int rightButtonState = glfwGetMouseButton(m_glfw_window, GLFW_MOUSE_BUTTON_RIGHT);
    int middleButtonState = glfwGetMouseButton(m_glfw_window, GLFW_MOUSE_BUTTON_MIDDLE);
    int altKeyState = glfwGetKey(m_glfw_window, GLFW_KEY_LEFT_ALT);

    return mCurrentScene->mouseMotionEvent(p[0], p[1], leftButtonState, rightButtonState,
                                                  middleButtonState, altKeyState);
}

// Handle a mouse scroll event (default implementation: propagate to children)
bool TestbedApplication::scroll_event(const Vector2i &p, const Vector2f &rel) {

    if (Screen::scroll_event(p, rel)) {
        return true;
    }

    return mCurrentScene->scrollingEvent(rel[0], rel[1], SCROLL_SENSITIVITY);
}
