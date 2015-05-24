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
#include "raycast/RaycastScene.h"

using namespace openglframework;
using namespace jointsscene;
using namespace cubesscene;
using namespace raycastscene;
using namespace collisionshapesscene;

// Initialization of static variables
const float TestbedApplication::SCROLL_SENSITIVITY = 0.02f;

// Create and return the singleton instance of this class
TestbedApplication& TestbedApplication::getInstance() {
    static TestbedApplication instance;
    return instance;
}

// Constructor
TestbedApplication::TestbedApplication() : mFPS(0), mNbFrames(0), mPreviousTime(0){

    mCurrentScene = NULL;
    mEngineSettings.timeStep = DEFAULT_TIMESTEP;
    mIsMultisamplingActive = true;
    mWidth = 1280;
    mHeight = 720;
}

// Destructor
TestbedApplication::~TestbedApplication() {

    // TODO : Check that this method is called at the end

    // Destroy all the scenes
    destroyScenes();

    // Destroy the window
    glfwDestroyWindow(mWindow);

    // Terminate GLFW
    glfwTerminate();
}

// Initialize the viewer
void TestbedApplication::init() {

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

    // Disable Vertical Synchronization
    glfwSwapInterval(0);

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
    glClearColor(0.0, 0.0, 0.0, 1.0);

    // Create all the scenes
    createScenes();

    Gui::getInstance().setWindow(mWindow);

    // Init the GUI
    //Gui::getInstance().init();

    mTimer.start();
}

// Create all the scenes
void TestbedApplication::createScenes() {

    // Cubes scene
    CubesScene* cubeScene = new CubesScene("Cubes");
    mScenes.push_back(cubeScene);

    /*
    // Joints scene
    JointsScene* jointsScene = new JointsScene("Joints");
    mScenes.push_back(jointsScene);
    */

    // Collision shapes scene
    CollisionShapesScene* collisionShapesScene = new CollisionShapesScene("Collision Shapes");
    mScenes.push_back(collisionShapesScene);

    /*
    // Raycast scene
    RaycastScene* raycastScene = new RaycastScene("Raycast");
    mScenes.push_back(raycastScene);
    */

    assert(mScenes.size() > 0);
    mCurrentScene = mScenes[1];
}

// Remove all the scenes
void TestbedApplication::destroyScenes() {

    for (int i=0; i<mScenes.size(); i++) {
        delete mScenes[i];
    }

    mCurrentScene = NULL;
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

    // Update the physics
    updatePhysics();

    // Compute the interpolation factor
    float factor = mTimer.computeInterpolationFactor(mEngineSettings.timeStep);
    assert(factor >= 0.0f && factor <= 1.0f);

    // Notify the scene about the interpolation factor
    mCurrentScene->setInterpolationFactor(factor);

    // Update the scene
    mCurrentScene->update();

    // Compute the current framerate
    //application->computeFPS();
}

// Render
void TestbedApplication::render() {

    // Render the scene
    mCurrentScene->render();

    // Display the GUI
    //Gui::getInstance().render();

    // Check the OpenGL errors
    checkOpenGLErrors();
}

// Set the dimension of the camera viewport
void TestbedApplication::reshape() {

    // Get the framebuffer dimension
    int width, height;
    glfwGetFramebufferSize(mWindow, &width, &height);

    // Resize the camera viewport
    mCurrentScene->reshape(width, height);

    // Resize the OpenGL viewport
    glViewport(0, 0, width, height);

    // Update the window size of the scene
    int windowWidth, windowHeight;
    glfwGetWindowSize(mWindow, &windowWidth, &windowHeight);
    mCurrentScene->setWindowDimension(windowWidth, windowHeight);
}

// Start the main loop where rendering occur
void TestbedApplication::startMainLoop() {

    // Loop until the user closes the window
    while (!glfwWindowShouldClose(mWindow)) {

        checkOpenGLErrors();

        // Reshape the viewport
        reshape();

        // Call the update function
        update();

        // Render the application
        render();

        // Swap front and back buffers
        glfwSwapBuffers(mWindow);

        // Process events
        glfwPollEvents();

        checkOpenGLErrors();
    }
}

// Check the OpenGL errors
void TestbedApplication::checkOpenGLErrors() {
    GLenum glError;

    // Get the OpenGL errors
    glError = glGetError();

    // While there are errors
    while (glError != GL_NO_ERROR) {

        // Get the error string
        const GLubyte* stringError = gluErrorString(glError);

        // Display the error
        if (stringError)
            std::cerr << "OpenGL Error #" << glError << "(" << gluErrorString(glError) << ")" << std::endl;
        else
            std::cerr << "OpenGL Error #" << glError << " (no message available)" << std::endl;

        // Get the next error
        glError = glGetError();
    }
}


// Compute the FPS
void TestbedApplication::computeFPS() {

    mNbFrames++;

    //  Get the number of milliseconds since glutInit called
    mCurrentTime = glfwGetTime();

    //  Calculate time passed
    double timeInterval = mCurrentTime - mPreviousTime;

    // Update the FPS counter each second
    if(timeInterval > 1.0) {

        //  calculate the number of frames per second
        mFPS = static_cast<double>(mNbFrames) / timeInterval;

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

// Display the GUI
void TestbedApplication::displayGUI() {

    // Display the FPS
    displayFPS();
}

// Display the FPS
void TestbedApplication::displayFPS() {

    /*
    std::stringstream ss;
    ss << mFPS;
    std::string fpsString = ss.str();
    std::string windowTitle = mWindowTitle + " | FPS : " + fpsString;
    glfwSetWindowTitle(mWindow, windowTitle.c_str());
    */
}

// Callback method to receive keyboard events
void TestbedApplication::keyboard(GLFWwindow* window, int key, int scancode,
                                  int action, int mods) {
    getInstance().mCurrentScene->keyboardEvent(key, scancode, action, mods);
}

// Callback method to receive scrolling events
void TestbedApplication::scroll(GLFWwindow* window, double xAxis, double yAxis) {
    getInstance().mCurrentScene->scrollingEvent(xAxis, yAxis, SCROLL_SENSITIVITY);
}

// Called when a mouse button event occurs
void TestbedApplication::mouseButton(GLFWwindow* window, int button, int action, int mods) {

    // Get the mouse cursor position
    double x, y;
    glfwGetCursorPos(window, &x, &y);

    getInstance().mCurrentScene->mouseButtonEvent(button, action, mods, x, y);
}

// Called when a mouse motion event occurs
void TestbedApplication::mouseMotion(GLFWwindow* window, double x, double y) {

    int leftButtonState = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT);
    int rightButtonState = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT);
    int middleButtonState = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE);
    int altKeyState = glfwGetKey(window, GLFW_KEY_LEFT_ALT);

    getInstance().mCurrentScene->mouseMotionEvent(x, y, leftButtonState, rightButtonState,
                                                  middleButtonState, altKeyState);
}
