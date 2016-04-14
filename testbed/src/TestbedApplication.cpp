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
#include "joints/JointsScene.h"
#include "collisionshapes/CollisionShapesScene.h"
#include "heightfield/HeightFieldScene.h"
#include "raycast/RaycastScene.h"
#include "concavemesh/ConcaveMeshScene.h"

using namespace openglframework;
using namespace jointsscene;
using namespace cubesscene;
using namespace raycastscene;
using namespace collisionshapesscene;
using namespace trianglemeshscene;
using namespace heightfieldscene;

// Initialization of static variables
const float TestbedApplication::SCROLL_SENSITIVITY = 0.08f;

// Constructor
TestbedApplication::TestbedApplication(bool isFullscreen)
                   : Screen(Vector2i(1280, 800), "Testbed ReactPhysics3D", true, isFullscreen),
                     mIsInitialized(false), mGui(this), mFPS(0), mNbFrames(0), mPreviousTime(0),
                     mLastTimeComputedFPS(0), mFrameTime(0), mPhysicsTime(0) {

    mCurrentScene = NULL;
    mIsMultisamplingActive = true;
    mWidth = 1280;
    mHeight = 720;
    mSinglePhysicsStepEnabled = false;
    mSinglePhysicsStepDone = false;
    mWindowToFramebufferRatio = Vector2(1, 1);
    mIsShadowMappingEnabled = true;
    mIsVSyncEnabled = false;
    mIsContactPointsDisplayed = false;

    init();

    resizeEvent(Vector2i(0, 0));
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

    mIsInitialized = true;
}

// Create all the scenes
void TestbedApplication::createScenes() {

    // Cubes scene
    CubesScene* cubeScene = new CubesScene("Cubes");
    mScenes.push_back(cubeScene);

    // Joints scene
    JointsScene* jointsScene = new JointsScene("Joints");
    mScenes.push_back(jointsScene);

    // Collision shapes scene
    CollisionShapesScene* collisionShapesScene = new CollisionShapesScene("Collision Shapes");
    mScenes.push_back(collisionShapesScene);

    // Heightfield shape scene
    HeightFieldScene* heightFieldScene = new HeightFieldScene("Heightfield");
    mScenes.push_back(heightFieldScene);

    // Raycast scene
    RaycastScene* raycastScene = new RaycastScene("Raycast");
    mScenes.push_back(raycastScene);

    // Raycast scene
    ConcaveMeshScene* concaveMeshScene = new ConcaveMeshScene("Concave Mesh");
    mScenes.push_back(concaveMeshScene);

    assert(mScenes.size() > 0);
    mCurrentScene = mScenes[0];

    // Get the engine settings from the scene
    mEngineSettings = mCurrentScene->getEngineSettings();
    mEngineSettings.timeStep = DEFAULT_TIMESTEP;
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

    // Set the engine settings
    mEngineSettings.elapsedTime = mTimer.getPhysicsTime();
    mCurrentScene->setEngineSettings(mEngineSettings);

    if (mTimer.isRunning()) {

        // Compute the time since the last update() call and update the timer
        mTimer.update();

        // While the time accumulator is not empty
        while(mTimer.isPossibleToTakeStep(mEngineSettings.timeStep)) {

            // Take a physics simulation step
            mCurrentScene->updatePhysics();

            // Update the timer
            mTimer.nextStep(mEngineSettings.timeStep);
        }
    }
}

void TestbedApplication::update() {

    double currentTime = glfwGetTime();

    // Update the physics
    if (mSinglePhysicsStepEnabled && !mSinglePhysicsStepDone) {
        updateSinglePhysicsStep();
        mSinglePhysicsStepDone = true;
    }
    else {
        updatePhysics();
    }

    // Compute the physics update time
    mPhysicsTime = glfwGetTime() - currentTime;

    // Compute the interpolation factor
    float factor = mTimer.computeInterpolationFactor(mEngineSettings.timeStep);
    assert(factor >= 0.0f && factor <= 1.0f);

    // Notify the scene about the interpolation factor
    mCurrentScene->setInterpolationFactor(factor);

    // Enable/Disable shadow mapping
    mCurrentScene->setIsShadowMappingEnabled(mIsShadowMappingEnabled);

    // Display/Hide contact points
    mCurrentScene->setIsContactPointsDisplayed(mIsContactPointsDisplayed);

    // Update the scene
    mCurrentScene->update();
}

void TestbedApplication::drawContents() {

    update();

    int bufferWidth, bufferHeight;
    glfwMakeContextCurrent(mGLFWWindow);
    glfwGetFramebufferSize(mGLFWWindow, &bufferWidth, &bufferHeight);

    // Set the viewport of the scene
    mCurrentScene->setViewport(0, 0, bufferWidth, bufferHeight);

    // Render the scene
    mCurrentScene->render();

    // Check the OpenGL errors
    checkOpenGLErrors();

    mGui.update();

    // Compute the current framerate
    computeFPS();
}

/// Window resize event handler
bool TestbedApplication::resizeEvent(const Vector2i& size) {

    if (!mIsInitialized) return false;

    // Get the framebuffer dimension
    int width, height;
    glfwGetFramebufferSize(mGLFWWindow, &width, &height);

    // Resize the camera viewport
    mCurrentScene->reshape(width, height);

    // Update the window size of the scene
    int windowWidth, windowHeight;
    glfwGetWindowSize(mGLFWWindow, &windowWidth, &windowHeight);
    mCurrentScene->setWindowDimension(windowWidth, windowHeight);

    return true;
}

// Change the current scene
void TestbedApplication::switchScene(Scene* newScene) {

    if (newScene == mCurrentScene) return;

    mCurrentScene = newScene;

    // Get the engine settings of the scene
    float currentTimeStep = mEngineSettings.timeStep;
    mEngineSettings = mCurrentScene->getEngineSettings();
    mEngineSettings.timeStep = currentTimeStep;

    // Reset the scene
    mCurrentScene->reset();

    resizeEvent(Vector2i(0, 0));
}

// Check the OpenGL errors
void TestbedApplication::checkOpenGLErrorsInternal(const char* file, int line) {
    GLenum glError;

    // Get the OpenGL errors
    glError = glGetError();

    // While there are errors
    while (glError != GL_NO_ERROR) {

        std::string error;

        switch(glError) {
                case GL_INVALID_OPERATION:      error="INVALID_OPERATION";      break;
                case GL_INVALID_ENUM:           error="INVALID_ENUM";           break;
                case GL_INVALID_VALUE:          error="INVALID_VALUE";          break;
                case GL_OUT_OF_MEMORY:          error="OUT_OF_MEMORY";          break;
                case GL_INVALID_FRAMEBUFFER_OPERATION:  error="INVALID_FRAMEBUFFER_OPERATION";  break;
        }

        std::cerr << "OpenGL Error #" << error.c_str() << " - " << file << ": " << line << std::endl;

        // Get the next error
        glError = glGetError();
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

bool TestbedApplication::keyboardEvent(int key, int scancode, int action, int modifiers) {

    if (Screen::keyboardEvent(key, scancode, action, modifiers)) {
        return true;
    }

    // Close application on escape key
    if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS) {
        glfwSetWindowShouldClose(mGLFWWindow, GL_TRUE);
        return true;
    }

    return mCurrentScene->keyboardEvent(key, scancode, action, modifiers);
}

// Handle a mouse button event (default implementation: propagate to children)
bool TestbedApplication::mouseButtonEvent(const Vector2i &p, int button, bool down, int modifiers) {

    if (Screen::mouseButtonEvent(p, button, down, modifiers)) {
        return true;
    }

    // Get the mouse cursor position
    double x, y;
    glfwGetCursorPos(mGLFWWindow, &x, &y);

    return mCurrentScene->mouseButtonEvent(button, down, modifiers, x, y);
}

// Handle a mouse motion event (default implementation: propagate to children)
bool TestbedApplication::mouseMotionEvent(const Vector2i &p, const Vector2i &rel, int button, int modifiers) {

    if (Screen::mouseMotionEvent(p, rel, button, modifiers)) {
        return true;
    }

    int leftButtonState = glfwGetMouseButton(mGLFWWindow, GLFW_MOUSE_BUTTON_LEFT);
    int rightButtonState = glfwGetMouseButton(mGLFWWindow, GLFW_MOUSE_BUTTON_RIGHT);
    int middleButtonState = glfwGetMouseButton(mGLFWWindow, GLFW_MOUSE_BUTTON_MIDDLE);
    int altKeyState = glfwGetKey(mGLFWWindow, GLFW_KEY_LEFT_ALT);

    return mCurrentScene->mouseMotionEvent(p[0], p[1], leftButtonState, rightButtonState,
                                                  middleButtonState, altKeyState);
}

// Handle a mouse scroll event (default implementation: propagate to children)
bool TestbedApplication::scrollEvent(const Vector2i &p, const Vector2f &rel) {

    if (Screen::scrollEvent(p, rel)) {
        return true;
    }

    return mCurrentScene->scrollingEvent(rel[0], rel[1], SCROLL_SENSITIVITY);
}
