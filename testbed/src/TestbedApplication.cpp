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
const float TestbedApplication::SCROLL_SENSITIVITY = 0.02f;

// Constructor
TestbedApplication::TestbedApplication(bool isFullscreen)
                   : Screen(Vector2i(1280, 760), "Testbed ReactPhysics3D", true, isFullscreen),
                     mIsInitialized(false), mFPS(0), mNbFrames(0), mPreviousTime(0),
                     mUpdateTime(0), mPhysicsUpdateTime(0), mGui(this) {

    mCurrentScene = NULL;
    mIsMultisamplingActive = true;
    mWidth = 1280;
    mHeight = 720;
    mSinglePhysicsStepEnabled = false;
    mSinglePhysicsStepDone = false;
    mWindowToFramebufferRatio = Vector2(1, 1);
    mIsShadowMappingEnabled = true;
    mIsVSyncEnabled = true;
    mIsContactPointsDisplayed = false;

    init();
}

// Destructor
TestbedApplication::~TestbedApplication() {

    // Destroy all the scenes
    destroyScenes();

    // Destroy the window
    //glfwDestroyWindow(mWindow);

    // Terminate GLFW
    //glfwTerminate();
}

// Initialize the viewer
void TestbedApplication::init() {

    mGui.init();

    /*

    {

        // Set the GLFW error callback method
        glfwSetErrorCallback(error_callback);

        // Initialize the GLFW library
        if (!glfwInit()) {
             std::exit(EXIT_FAILURE);
        }

        // OpenGL version required
        glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
        glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
        glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
        glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);

        // Active the multi-sampling by default
        if (mIsMultisamplingActive) {
            glfwWindowHint(GLFW_SAMPLES, 4);
        }


        // Create the GLFW window
        mWindow = glfwCreateWindow(mWidth, mHeight,
                                   "ReactPhysics3D Testbed", NULL, NULL);
        if (!mWindow) {
            glfwTerminate();
            std::exit(EXIT_FAILURE);
        }
        glfwMakeContextCurrent(mWindow);

        // Vertical Synchronization
        enableVSync(mIsVSyncEnabled);

        // Initialize the GLEW library
        glewExperimental = GL_TRUE;
        GLenum errorGLEW = glewInit();
        if (errorGLEW != GLEW_OK) {

            // Problem: glewInit failed, something is wrong
            std::cerr << "GLEW Error : " << glewGetErrorString(errorGLEW) << std::endl;
            assert(false);
            std::exit(EXIT_FAILURE);
        }

        if (mIsMultisamplingActive) {
            glEnable(GL_MULTISAMPLE);
        }

        glfwSetKeyCallback(mWindow, keyboard);
        glfwSetMouseButtonCallback(mWindow, mouseButton);
        glfwSetCursorPosCallback(mWindow, mouseMotion);
        glfwSetScrollCallback(mWindow, scroll);

        // Define the background color (black)
        glClearColor(0, 0, 0, 1.0);

        // Create all the scenes
        createScenes();

        Gui::getInstance().setWindow(mScreen);

        // Init the GUI
        Gui::getInstance().init();

        mTimer.start();

    }
    */

    // Create all the scenes
    createScenes();

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

    for (int i=0; i<mScenes.size(); i++) {
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
    mPhysicsUpdateTime = glfwGetTime() - currentTime;

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

    resizeEvent(Vector2i(0, 0));

    update();

    int bufferWidth, bufferHeight;
    glfwMakeContextCurrent(mGLFWWindow);
    glfwGetFramebufferSize(mGLFWWindow, &bufferWidth, &bufferHeight);

    // Set the viewport of the scene
    mCurrentScene->setViewport(0, 0, bufferWidth, bufferHeight);

    // Render the scene
    mCurrentScene->render();

    // Display the GUI
    //Gui::getInstance().render();

    // Compute the current framerate
    computeFPS();

    // Check the OpenGL errors
    checkOpenGLErrors();
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

// Render
/*
void TestbedApplication::render() {

    int bufferWidth, bufferHeight;
    glfwGetFramebufferSize(mGLFWWindow, &bufferWidth, &bufferHeight);

    int windowWidth, windowHeight;
    glfwGetWindowSize(mGLFWWindow, &windowWidth, &windowHeight);

    // Compute the window to framebuffer ratio
    mWindowToFramebufferRatio.x = float(bufferWidth) / float(windowWidth);
    mWindowToFramebufferRatio.y = float(bufferHeight) / float(windowHeight);

    // Set the viewport of the scene
    mCurrentScene->setViewport(mWindowToFramebufferRatio.x * LEFT_PANE_WIDTH,
                               0,
                               bufferWidth - mWindowToFramebufferRatio.x * LEFT_PANE_WIDTH,
                               bufferHeight);

    // Render the scene
    mCurrentScene->render();

    // Display the GUI
    Gui::getInstance().render();

    // Check the OpenGL errors
    checkOpenGLErrors();
}
*/

// Set the dimension of the camera viewport
void TestbedApplication::reshape() {


}

/*
// Start the main loop where rendering occur
void TestbedApplication::startMainLoop() {

    // Loop until the user closes the window
    while (!glfwWindowShouldClose(mGLFWWindow)) {

        checkOpenGLErrors();

        // Call the update function
        update();

        // Render the application
        //render();

        // Swap front and back buffers
        glfwSwapBuffers(mGLFWWindow);

        // Process events
        glfwPollEvents();

        // Compute the current framerate
        computeFPS();

        checkOpenGLErrors();
    }
}
*/

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

    mNbFrames++;

    //  Get the number of seconds since start
    mCurrentTime = glfwGetTime();

    //  Calculate time passed
    mUpdateTime = mCurrentTime - mPreviousTime;
    double timeInterval = mUpdateTime * 1000.0;

    // Update the FPS counter each second
    if(timeInterval > 0.0001) {

        //  calculate the number of frames per second
        mFPS = static_cast<double>(mNbFrames) / timeInterval;
        mFPS *= 1000.0;

        //  Set time
        mPreviousTime = mCurrentTime;

        //  Reset frame count
        mNbFrames = 0;
    }
}

// GLFW error callback method
void TestbedApplication::error_callback(int error, const char* description) {
    fputs(description, stderr);
}

// Callback method to receive keyboard events
void TestbedApplication::keyboard(GLFWwindow* window, int key, int scancode,
                                  int action, int mods) {
    //getInstance().mCurrentScene->keyboardEvent(key, scancode, action, mods);
}

bool TestbedApplication::keyboardEvent(int key, int scancode, int action, int modifiers) {

    if (Screen::keyboardEvent(key, scancode, action, modifiers)) {
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

    // Update scroll on the GUI
    //Gui::getInstance().setScroll(xAxis, yAxis);

    return mCurrentScene->scrollingEvent(rel[0], rel[1], SCROLL_SENSITIVITY);
}

// Callback method to receive scrolling events
void TestbedApplication::scroll(GLFWwindow* window, double xAxis, double yAxis) {

    // Update scroll on the GUI
    //Gui::getInstance().setScroll(xAxis, yAxis);

    //getInstance().mCurrentScene->scrollingEvent(xAxis, yAxis, SCROLL_SENSITIVITY);
}

// Called when a mouse button event occurs
void TestbedApplication::mouseButton(GLFWwindow* window, int button, int action, int mods) {

    // Get the mouse cursor position
    double x, y;
    glfwGetCursorPos(window, &x, &y);

    //getInstance().mCurrentScene->mouseButtonEvent(button, action, mods, x, y);
}

// Called when a mouse motion event occurs
void TestbedApplication::mouseMotion(GLFWwindow* window, double x, double y) {

    int leftButtonState = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT);
    int rightButtonState = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT);
    int middleButtonState = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE);
    int altKeyState = glfwGetKey(window, GLFW_KEY_LEFT_ALT);

    /*getInstance().mCurrentScene->mouseMotionEvent(x, y, leftButtonState, rightButtonState,
                                                  middleButtonState, altKeyState);*/
}
