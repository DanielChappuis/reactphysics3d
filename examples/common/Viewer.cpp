/********************************************************************************
* ReactPhysics3D physics library, http://code.google.com/p/reactphysics3d/      *
* Copyright (c) 2010-2013 Daniel Chappuis                                       *
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
#include "Viewer.h"
#include "openglframework.h"
#include <iostream>

using namespace openglframework;

// Initialization of static variables
const float Viewer::SCROLL_SENSITIVITY = 0.02f;

// Constructor
Viewer::Viewer() : mFPS(0), mNbFrames(0), mPreviousTime(0) {

}

// Destructor
Viewer::~Viewer() {

    // Destroy the window
    glfwDestroyWindow(mWindow);

    // Terminate GLFW
    glfwTerminate();
}

// Initialize the viewer
void Viewer::init(int argc, char** argv, const std::string& windowsTitle,
                  const Vector2& windowsSize, const Vector2& windowsPosition,
                  bool isMultisamplingActive) {

    mWindowTitle = windowsTitle;

    // Set the GLFW error callback method
    glfwSetErrorCallback(error_callback);

    // Initialize the GLFW library
    if (!glfwInit()) {
         exit(EXIT_FAILURE);
    }

    // Active the multi-sampling by default
    if (isMultisamplingActive) {
        glfwWindowHint(GLFW_SAMPLES, 4);
    }

    // Create the GLFW window
    mWindow = glfwCreateWindow(static_cast<int>(windowsSize.x),
                               static_cast<int>(windowsSize.y), mWindowTitle.c_str(), NULL, NULL);
    if (!mWindow) {
        glfwTerminate();
        exit(EXIT_FAILURE);
    }
    glfwMakeContextCurrent(mWindow);

    // Disable Vertical Synchronization
    glfwSwapInterval(0);

    // Initialize the GLEW library
    GLenum errorGLEW = glewInit();
    if (errorGLEW != GLEW_OK) {

        // Problem: glewInit failed, something is wrong
        std::cerr << "GLEW Error : " << glewGetErrorString(errorGLEW) << std::endl;
        assert(false);
        exit(EXIT_FAILURE);
    }

    if (isMultisamplingActive) {
        glEnable(GL_MULTISAMPLE);
    }
}

// Set the dimension of the camera viewport
void Viewer::reshape() {

    // Get the framebuffer dimension
    int width, height;
    glfwGetFramebufferSize(mWindow, &width, &height);

    // Resize the viewport
    mCamera.setDimensions(width, height);
    glViewport(0, 0, width, height);
}

// Start the main loop where rendering occur
void Viewer::startMainLoop() {

    // Loop until the user closes the window
    while (!glfwWindowShouldClose(mWindow)) {

        // Reshape the viewport
        reshape();

        // Call the update function
        (*mUpdateFunctionPointer)();

        // Swap front and back buffers
        glfwSwapBuffers(mWindow);

        // Poll for and process events
        glfwPollEvents();
    }
}

// Set the camera so that we can view the whole scene
void Viewer::resetCameraToViewAll() {

    // Move the camera to the origin of the scene
    mCamera.translateWorld(-mCamera.getOrigin());

    // Move the camera to the center of the scene
    mCamera.translateWorld(mCenterScene);

    // Set the zoom of the camera so that the scene center is
    // in negative view direction of the camera
    mCamera.setZoom(1.0);
}

// Called when a mouse button event occurs
void Viewer::mouseButtonEvent(int button, int action) {

    // Get the mouse cursor position
    double x, y;
    glfwGetCursorPos(mWindow, &x, &y);

    // If the mouse button is pressed
    if (action == GLFW_PRESS) {
        mLastMouseX = x;
        mLastMouseY = y;
        mIsLastPointOnSphereValid = mapMouseCoordinatesToSphere(x, y, mLastPointOnSphere);        
    }
    else {  // If the mouse button is released
        mIsLastPointOnSphereValid = false;
    }
}

// Called when a mouse motion event occurs
void Viewer::mouseMotionEvent(double xMouse, double yMouse) {

    int leftButtonState = glfwGetMouseButton(mWindow, GLFW_MOUSE_BUTTON_LEFT);
    int rightButtonState = glfwGetMouseButton(mWindow, GLFW_MOUSE_BUTTON_RIGHT);
    int middleButtonState = glfwGetMouseButton(mWindow, GLFW_MOUSE_BUTTON_MIDDLE);
    int altKeyState = glfwGetKey(mWindow, GLFW_KEY_LEFT_ALT);

    // Zoom
    if (leftButtonState == GLFW_PRESS && altKeyState == GLFW_PRESS) {

        // Get the window dimension
        int width, height;
        glfwGetWindowSize(mWindow, &width, &height);

        float dy = static_cast<float>(yMouse - mLastMouseY);
        float h = static_cast<float>(height);

        // Zoom the camera
        zoom(-dy / h);
    }
    // Translation
    else if (middleButtonState == GLFW_PRESS || rightButtonState == GLFW_PRESS ||
             (leftButtonState == GLFW_PRESS && altKeyState == GLFW_PRESS)) {
        translate(xMouse, yMouse);
    }
    // Rotation
    else if (leftButtonState == GLFW_PRESS) {
        rotate(xMouse, yMouse);
    }

    // Remember the mouse position
    mLastMouseX = xMouse;
    mLastMouseY = yMouse;
    mIsLastPointOnSphereValid = mapMouseCoordinatesToSphere(xMouse, yMouse, mLastPointOnSphere);
}

// Called when a scrolling event occurs
void Viewer::scrollingEvent(float scrollAxis) {
    zoom(scrollAxis * SCROLL_SENSITIVITY);
}

// Map the mouse x,y coordinates to a point on a sphere
bool Viewer::mapMouseCoordinatesToSphere(double xMouse, double yMouse, Vector3& spherePoint) const {

    // Get the window dimension
    int width, height;
    glfwGetWindowSize(mWindow, &width, &height);

    if ((xMouse >= 0) && (xMouse <= width) && (yMouse >= 0) && (yMouse <= height)) {
        float x = float(xMouse - 0.5f * width) / float(width);
        float y = float(0.5f * height - yMouse) / float(height);
        float sinx = sin(PI * x * 0.5f);
        float siny = sin(PI * y * 0.5f);
        float sinx2siny2 = sinx * sinx + siny * siny;

        // Compute the point on the sphere
        spherePoint.x = sinx;
        spherePoint.y = siny;
        spherePoint.z = (sinx2siny2 < 1.0) ? sqrt(1.0f - sinx2siny2) : 0.0f;

        return true;
    }

    return false;
}

// Zoom the camera
void Viewer::zoom(float zoomDiff) {

    // Zoom the camera
    mCamera.setZoom(zoomDiff);
}

// Translate the camera
void Viewer::translate(int xMouse, int yMouse) {
   float dx = static_cast<float>(xMouse - mLastMouseX);
   float dy = static_cast<float>(yMouse - mLastMouseY);

   // Translate the camera
   mCamera.translateCamera(-dx / float(mCamera.getWidth()),
                           -dy / float(mCamera.getHeight()), mCenterScene);
}

// Rotate the camera
void Viewer::rotate(int xMouse, int yMouse) {

    if (mIsLastPointOnSphereValid) {

        Vector3 newPoint3D;
        bool isNewPointOK = mapMouseCoordinatesToSphere(xMouse, yMouse, newPoint3D);

        if (isNewPointOK) {
            Vector3 axis = mLastPointOnSphere.cross(newPoint3D);
            float cosAngle = mLastPointOnSphere.dot(newPoint3D);

            float epsilon = std::numeric_limits<float>::epsilon();
            if (fabs(cosAngle) < 1.0f && axis.length() > epsilon) {
                axis.normalize();
                float angle = 2.0f * acos(cosAngle);

                // Rotate the camera around the center of the scene
                mCamera.rotateAroundLocalPoint(axis, -angle, mCenterScene);
            }
        }        
    }
}

// Check the OpenGL errors
void Viewer::checkOpenGLErrors() {
    GLenum glError;

    // Get the OpenGL errors
    glError = glGetError();

    // While there are errors
    while (glError != GL_NO_ERROR) {

        // Get the error string
        const GLubyte* stringError = gluErrorString(glError);

        // Display the error
        if (stringError)
            std::cerr << "OpenGL Error #" << glError << "(" << gluErrorString(glError) << std::endl;
        else
            std::cerr << "OpenGL Error #" << glError << " (no message available)" << std::endl;

        // Get the next error
        glError = glGetError();
    }
}


// Compute the FPS
void Viewer::computeFPS() {

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
void Viewer::error_callback(int error, const char* description) {
    fputs(description, stderr);
}

// Display the GUI
void Viewer::displayGUI() {

    // Display the FPS
    displayFPS();
}

// Display the FPS
void Viewer::displayFPS() {

    std::string windowTitle = mWindowTitle + " | FPS : " + std::to_string(mFPS);
    glfwSetWindowTitle(mWindow, windowTitle.c_str());
}
