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

#ifndef VIEWER_H
#define VIEWER_H

// Libraries
#include "openglframework.h"
#include <GLFW/glfw3.h>

// Class Viewer
class Viewer {

    private :

        // -------------------- Constants -------------------- //

        static const float SCROLL_SENSITIVITY;

        // -------------------- Attributes -------------------- //

        /// GLFW window
        GLFWwindow* mWindow;

        /// Window title
        std::string mWindowTitle;

        /// Camera
        openglframework::Camera mCamera;

        /// Center of the scene
        openglframework::Vector3 mCenterScene;

        /// Last mouse coordinates on the windows
        double mLastMouseX, mLastMouseY;

        /// Last point computed on a sphere (for camera rotation)
        openglframework::Vector3 mLastPointOnSphere;

        /// True if the last point computed on a sphere (for camera rotation) is valid
        bool mIsLastPointOnSphereValid;

        /// Current number of frames per seconds
        double mFPS;

        /// Number of frames during the last second
        int mNbFrames;

        /// Current time for fps computation
        double mCurrentTime;

        /// Previous time for fps computation
        double mPreviousTime;

        /// Pointer to the update function
        void (*mUpdateFunctionPointer)();

        // -------------------- Methods -------------------- //

        bool mapMouseCoordinatesToSphere(double xMouse, double yMouse, openglframework::Vector3& spherePoint) const;

    public :

        // -------------------- Methods -------------------- //

        /// Constructor
        Viewer();

        /// Destructor
        ~Viewer();

        // -------------------- Methods -------------------- //

        /// Initialize the viewer
        void init(int argc, char** argv, const std::string& windowsTitle,
                  const openglframework::Vector2& windowsSize,
                  const openglframework::Vector2& windowsPosition,
                  bool isMultisamplingActive = false);

        /// Start the main loop where rendering occur
        void startMainLoop();

        /// Called when the windows is reshaped
        void reshape();

        /// Set the scene position (where the camera needs to look at)
        void setScenePosition(const openglframework::Vector3& position, float sceneRadius);

        /// Set the camera so that we can view the whole scene
        void resetCameraToViewAll();

        /// Zoom the camera
        void zoom(float zoomDiff);

        /// Translate the camera
        void translate(int xMouse, int yMouse);

        /// Rotate the camera
        void rotate(int xMouse, int yMouse);

        /// Get the camera
        openglframework::Camera& getCamera();

        /// Called when a GLUT mouse button event occurs
        void mouseButtonEvent(int button, int action);

        /// Called when a GLUT mouse motion event occurs
        void mouseMotionEvent(double xMouse, double yMouse);

        /// Called when a scrolling event occurs
        void scrollingEvent(float scrollAxis);

        /// Check the OpenGL errors
        static void checkOpenGLErrors();

        /// Display the FPS
        void displayFPS();

        /// Compute the FPS
        void computeFPS();

        /// Display the GUI
        void displayGUI();

        /// GLFW error callback method
        static void error_callback(int error, const char* description);

        /// Register the update function that has to be called each frame
        void registerUpdateFunction(void (*updateFunctionPointer)());

        /// Register a keyboard callback method
        void registerKeyboardCallback(GLFWkeyfun method);

        /// Register a mouse button callback method
        void registerMouseButtonCallback(GLFWmousebuttonfun method);

        /// Register a mouse cursor motion callback method
        void registerMouseCursorCallback(GLFWcursorposfun method);

        /// Register a scrolling cursor callback method
        void registerScrollingCallback(GLFWscrollfun method);

};

// Set the scene position (where the camera needs to look at)
inline void Viewer::setScenePosition(const openglframework::Vector3& position, float sceneRadius) {

    // Set the position and radius of the scene
    mCenterScene = position;
    mCamera.setSceneRadius(sceneRadius);

    // Reset the camera position and zoom in order to view all the scene
    resetCameraToViewAll();
}

// Get the camera
inline openglframework::Camera& Viewer::getCamera() {
   return mCamera;
}

// Register the update function that has to be called each frame
inline void Viewer::registerUpdateFunction(void (*updateFunctionPointer)()) {
    mUpdateFunctionPointer = updateFunctionPointer;
}

// Register a keyboard callback method
inline void Viewer::registerKeyboardCallback(GLFWkeyfun method) {
    glfwSetKeyCallback(mWindow, method);
}

// Register a mouse button callback method
inline void Viewer::registerMouseButtonCallback(GLFWmousebuttonfun method) {
    glfwSetMouseButtonCallback(mWindow, method);
}

// Register a mouse cursor motion callback method
inline void Viewer::registerMouseCursorCallback(GLFWcursorposfun method) {
    glfwSetCursorPosCallback(mWindow, method);
}

// Register a scrolling cursor callback method
inline void Viewer::registerScrollingCallback(GLFWscrollfun method) {
    glfwSetScrollCallback(mWindow, method);
}

#endif
