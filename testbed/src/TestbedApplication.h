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

using namespace nanogui;

// Macro for OpenGL errors
#define checkOpenGLErrors() checkOpenGLErrorsInternal(__FILE__,__LINE__)

/// Class TestbedApplication
class TestbedApplication : public Screen {

    private :

        // -------------------- Constants -------------------- //

        static const float SCROLL_SENSITIVITY;

        // -------------------- Attributes -------------------- //

        bool mIsInitialized;

        Gui mGui;

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

    public :

        // -------------------- Methods -------------------- //

        /// Private constructor (for the singleton class)
        TestbedApplication(bool isFullscreen, int windowWidth, int windowHeight);

        /// Destructor
        virtual ~TestbedApplication() override;

        /// Render the content of the application
        virtual void draw_contents() override;

        /// Window resize event handler
        virtual bool resize_event(const Vector2i& size) override;

        /// Default keyboard event handler
        virtual bool keyboard_event(int key, int scancode, int action, int modifiers) override;

        /// Handle a mouse button event (default implementation: propagate to children)
        virtual bool mouse_button_event(const Vector2i &p, int button, bool down, int modifiers) override;

        /// Handle a mouse motion event (default implementation: propagate to children)
        virtual bool mouse_motion_event(const Vector2i &p, const Vector2i &rel, int button, int modifiers) override;

        /// Handle a mouse scroll event (default implementation: propagate to children)
        virtual bool scroll_event(const Vector2i &p, const Vector2f &rel) override;

        /// Initialize the application
        void init();

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

#endif
