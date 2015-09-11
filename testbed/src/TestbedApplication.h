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

#ifndef TESTBED_APPLICATION_H
#define	TESTBED_APPLICATION_H

// Libraries
#include "openglframework.h"
#include "GUI.h"
#include "Scene.h"
#include "Timer.h"
#include <GLFW/glfw3.h>

// Constants
const float DEFAULT_TIMESTEP = 1.0f / 60.0f;

/// Class TestbedApplication
/// Singleton class representing the application.
class TestbedApplication {

    private :

        // -------------------- Constants -------------------- //

        static const float SCROLL_SENSITIVITY;

        // -------------------- Attributes -------------------- //

        /// GLFW window
        GLFWwindow* mWindow;

        /// Timer
        Timer mTimer;

        /// List of 3D scenes
        std::vector<Scene*> mScenes;

        /// Current 3D scene
        Scene* mCurrentScene;

        /// Physics engine settings
        EngineSettings mEngineSettings;

        /// Current number of frames per seconds
        double mFPS;

        /// Number of frames during the last second
        int mNbFrames;

        /// Current time for fps computation (in seconds)
        double mCurrentTime;

        /// Previous time for fps computation (in seconds)
        double mPreviousTime;

        /// Update time (in seconds)
        double mUpdateTime;

        /// Physics update time (in seconds)
        double mPhysicsUpdateTime;

        /// True if multisampling is active
        bool mIsMultisamplingActive;

        /// Width and height of the window
        int mWidth, mHeight;

        /// True if the next simulation update is a single physics step
        bool mSinglePhysicsStepEnabled;

        /// True if the single physics step has been taken already
        bool mSinglePhysicsStepDone;

        openglframework::Vector2 mWindowToFramebufferRatio;

        /// True if shadow mapping is enabled
        bool mIsShadowMappingEnabled;

        /// True if contact points are displayed
        bool mIsContactPointsDisplayed;

        /// True if vsync is enabled
        bool mIsVSyncEnabled;

        // -------------------- Methods -------------------- //

        /// Private constructor (for the singleton class)
        TestbedApplication();

        /// Private copy-constructor (for the singleton class)
        TestbedApplication(TestbedApplication const&);

        /// Private assignment operator (for the singleton class)
        void operator=(TestbedApplication const&);

        /// Update the physics of the current scene
        void updatePhysics();

        /// Update
        void update();

        /// Update the simulation by taking a single physics step
        void updateSinglePhysicsStep();

        /// Called when the windows is reshaped
        void reshape();

        /// Render
        void render();

        /// Check the OpenGL errors
        static void checkOpenGLErrors();

        /// Compute the FPS
        void computeFPS();

        /// GLFW error callback method
        static void error_callback(int error, const char* description);

        /// Callback method to receive keyboard events
        static void keyboard(GLFWwindow* window, int key, int scancode, int action, int mods);

        /// Callback method to receive scrolling events
        static void scroll(GLFWwindow* window, double xAxis, double yAxis);

        /// Called when a mouse button event occurs
        static void mouseButton(GLFWwindow* window, int button, int action, int mods);

        /// Called when a mouse motion event occurs
        static void mouseMotion(GLFWwindow* window, double x, double y);

        /// Initialize all the scenes
        void createScenes();

        /// Remove all the scenes
        void destroyScenes();

        /// Return the list of the scenes
        std::vector<Scene*> getScenes();

        /// Start/stop the simulation
        void togglePlayPauseSimulation();

        /// Restart the simulation
        void restartSimulation();

        /// Set the variable to know if we need to take a single physics step
        void toggleTakeSinglePhysicsStep();

        /// Enable/Disable shadow mapping
        void enableShadows(bool enable);

        /// Display/Hide contact points
        void displayContactPoints(bool display);

    public :

        // -------------------- Methods -------------------- //

        /// Create and return the singleton instance of this class
        static TestbedApplication& getInstance();

        /// Destructor
        ~TestbedApplication();

        /// Initialize the application
        void init();

        /// Start the main loop where rendering occur
        void startMainLoop();

        /// Change the current scene
        void switchScene(Scene* newScene);

        /// Enable/Disable Vertical synchronization
        void enableVSync(bool enable);

        // -------------------- Friendship -------------------- //

        friend class Gui;
};

// Return the list of the scenes
inline std::vector<Scene*> TestbedApplication::getScenes() {
    return mScenes;
}

// Start the simulation
inline void TestbedApplication::togglePlayPauseSimulation() {

    if (mTimer.isRunning()) {
        mTimer.stop();
    }
    else {
        mTimer.start();
    }
}

// Restart the simulation
inline void TestbedApplication::restartSimulation() {
    mCurrentScene->reset();
    mTimer.start();
}

// Take a single step of simulation
inline void TestbedApplication::toggleTakeSinglePhysicsStep() {
    mSinglePhysicsStepEnabled = true;
    mSinglePhysicsStepDone = false;

    if (mTimer.isRunning()) {
        mSinglePhysicsStepEnabled = false;
    }
}

// Enable/Disable shadow mapping
inline void TestbedApplication::enableShadows(bool enable) {
    mIsShadowMappingEnabled = enable;
}

/// Display/Hide contact points
inline void TestbedApplication::displayContactPoints(bool display) {
    mIsContactPointsDisplayed = display;
}

// Enable/Disable Vertical synchronization
inline void TestbedApplication::enableVSync(bool enable) {
    mIsVSyncEnabled = enable;
    if (mIsVSyncEnabled) {
        glfwSwapInterval(1);
    }
    else {
        glfwSwapInterval(0);
    }
}

#endif
