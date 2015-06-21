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
#include "Gui.h"
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

        /// Current time for fps computation
        double mCurrentTime;

        /// Previous time for fps computation
        double mPreviousTime;

        /// True if multisampling is active
        bool mIsMultisamplingActive;

        /// Width and height of the window
        int mWidth, mHeight;

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

        /// Called when the windows is reshaped
        void reshape();

        /// Render
        void render();

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

        // -------------------- Friendship -------------------- //

        friend class Gui;
};

// Return the list of the scenes
inline std::vector<Scene*> TestbedApplication::getScenes() {
    return mScenes;
}


#endif
