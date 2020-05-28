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

#ifndef GUI_H
#define	GUI_H

// Libraries
#include <nanogui/opengl.h>
#include <nanogui/nanogui.h>
#include "openglframework.h"
#include <sstream>
#include <iomanip>

using namespace openglframework;
using namespace nanogui;

const double TIME_INTERVAL_DISPLAY_PROFILING_INFO = 1;

// Declarations
class TestbedApplication;

// Class Gui
class Gui {

    protected :

        enum LeftPane {SCENES, PHYSICS, RENDERING, PROFILING};

        // -------------------- Constants -------------------- //


        // -------------------- Attributes -------------------- //

        // Pointer to the application
        TestbedApplication* mApp;

        static double mScrollX, mScrollY;

        // Simulation panel
        Widget* mSimulationPanel;

        // Settings Panel
        Widget* mSettingsPanel;
        Widget* mPhysicsPanel;
        Widget* mRenderingPanel;

        // Profiling panel
        Label* mFPSLabel;
        Label* mFrameTimeLabel;
        Label* mTotalPhysicsTimeLabel;
        Label* mPhysicsStepTimeLabel;

        std::vector<CheckBox*> mCheckboxesScenes;

        // -------------------- Methods -------------------- //

        static void resetScroll();

        /// Current time (in seconds) from last profiling time display
        static double mTimeSinceLastProfilingDisplay;

        /// Cached Framerate
        static double mCachedFPS;

        /// Cached update time
        static double mCachedUpdateTime;

        // Cached total update physics time
        static double mCachedTotalPhysicsUpdateTime;

        // Cached update single physics step time
        static double mCachedPhysicsStepTime;

        // -------------------- Methods -------------------- //

        void createSimulationPanel();

        void createSettingsPanel();

        void createProfilingPanel();

        // Convert float value to string
        std::string floatToString(float value, int precision);

    public :

        // -------------------- Methods -------------------- //

        /// Constructor
        Gui(TestbedApplication* app);

        /// Destructor
        ~Gui();

        /// Initialize the GUI
        void init();

        /// Update the GUI
        void update();

        /// Display the GUI
        void render();

        static void setScroll(double scrollX, double scrollY);
};

inline void Gui::resetScroll() {
    mScrollX = 0.0;
    mScrollY = 0.0;
}

inline void Gui::setScroll(double scrollX, double scrollY) {
    mScrollX = scrollX;
    mScrollY = scrollY;
}

inline std::string Gui::floatToString(float value, int precision) {
    std::stringstream ss;
    ss << std::fixed << std::setprecision(precision) << value;
    return ss.str();
}

#endif
