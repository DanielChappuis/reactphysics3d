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

#ifndef TESTBED_APPLICATION_H
#define	TESTBED_APPLICATION_H

// Libraries
#include "openglframework.h"
#include "Gui.h"
#include "Scene.h"
#include "Timer.h"
#include <GLFW/glfw3.h>
#include "TestbedLogger.h"

using namespace nanogui;


// Macro for OpenGL errors
#define checkOpenGLErrors() checkOpenGLErrorsInternal(__FILE__,__LINE__)

/// Class TestbedApplication
class TestbedApplication {

    private :

        // -------------------- Constants -------------------- //

        static const float SCROLL_SENSITIVITY;
        static constexpr bool IS_FULLSCREEN = false;

        // -------------------- Attributes -------------------- //

        bool mIsInitialized;

        GLFWwindow* mWindow;
        Screen* mScreen;

        Gui mGui;

        /// Timer
        Timer mTimer;

        /// Physics common object
        rp3d::PhysicsCommon mPhysicsCommon;

        /// List of 3D scenes
        std::vector<Scene*> mScenes;

        /// Current 3D scene
        Scene* mCurrentScene;

        /// Physics engine default settings
        EngineSettings mDefaultEngineSettings;

        /// Current number of frames per seconds
        double mFPS;

        /// Number of frames during the last second
        int mNbFrames;

        /// Current time for fps computation (in seconds)
        double mCurrentTime;

        /// Previous time for fps computation (in seconds)
        double mPreviousTime;

        /// Last time the FPS have been computed
        double mLastTimeComputedFPS;

        /// Update time (in seconds)
        double mFrameTime;

        /// Total physics update time (in seconds)
        double mTotalPhysicsTime;

        /// Time of a single physics step (in seconds)
        double mPhysicsStepTime;

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
        bool mAreContactPointsDisplayed;

        /// True if contact normals are displayed
        bool mAreContactNormalsDisplayed;

        /// True if the broad phase AABBs are displayed
        bool mAreBroadPhaseAABBsDisplayed;

        /// True if the AABBs of the colliders are displayed
        bool mAreCollidersAABBsDisplayed;

        /// True if the collision shapes are displayed
        bool mAreCollisionShapesDisplayed;

        /// True if the wireframe rendering is enabled
        bool mAreObjectsWireframeEnabled;

        /// True if vsync is enabled
        bool mIsVSyncEnabled;

        /// True if the debug renderer is enabled
        bool mIsDebugRendererEnabled;

        /// Logger
        TestbedLogger mLogger;

        // -------------------- Methods -------------------- //

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

        /// Compute the FPS
        void computeFPS();

        /// Initialize all the scenes
        void createScenes();

        /// Remove all the scenes
        void destroyScenes();

        /// Return the list of the scenes
        std::vector<Scene*> getScenes();

        /// Start/stop the simulation
        void togglePlayPauseSimulation();

        /// Play the simulation
        void playSimulation();

        /// Pause the simulation
        void pauseSimulation();

        /// Restart the simulation
        void restartSimulation();

        /// Set the variable to know if we need to take a single physics step
        void toggleTakeSinglePhysicsStep();

        /// Return the engine settings of the current scene
        EngineSettings& getCurrentSceneEngineSettings();

    public :

        // -------------------- Methods -------------------- //

        /// Private constructor (for the singleton class)
        TestbedApplication();

        /// Destructor
        ~TestbedApplication();

        /// Render the content of the application
        void render();

        /// Window resize event handler
        bool onWindowResized(int width, int height);

        /// Default keyboard event handler
        void keyboard_event(int key, int scancode, int action, int modifiers);

        /// Handle a mouse button event (default implementation: propagate to children)
        void mouse_button_event(int button, int action, int modifiers);

        /// Handle a mouse motion event (default implementation: propagate to children)
        void mouse_motion_event(double x, double y);

        /// Handle a mouse scroll event
        void scroll_event(double x, double y);

        /// Start the application
        void start();

        /// Change the current scene
        void switchScene(Scene* newScene);

        /// Enable/Disable Vertical synchronization
        void enableVSync(bool enable);

        /// Notify that the engine settings have changed
        void notifyEngineSetttingsChanged();

        /// Called when an OpenGL Error occurs
        static void GLAPIENTRY onOpenGLError(GLenum source, GLenum type, GLuint id, GLenum severity, GLsizei length,
                              const GLchar* message, const void* userParam );

        // -------------------- Friendship -------------------- //

        friend class Gui;
};

// Return the list of the scenes
inline std::vector<Scene*> TestbedApplication::getScenes() {
    return mScenes;
}

// Toggle play/pause for the simulation
inline void TestbedApplication::togglePlayPauseSimulation() {

    if (mTimer.isRunning()) {
        mTimer.stop();
    }
    else {
        mTimer.start();
    }
}

// Play the simulation
inline void TestbedApplication::playSimulation() {
    if (!mTimer.isRunning()) mTimer.start();
}

// Pause the simulation
inline void TestbedApplication::pauseSimulation() {
    if (mTimer.isRunning()) mTimer.stop();
}

// Restart the simulation
inline void TestbedApplication::restartSimulation() {
    mTimer.reset();
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

// Return the engine settings of the current scene
inline EngineSettings& TestbedApplication::getCurrentSceneEngineSettings() {
   return mCurrentScene->getEngineSettings();
}

#endif
