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

#ifndef GUI_H
#define	GUI_H

// Libraries
#include <nanogui/opengl.h>
#include <nanogui/nanogui.h>
#include "openglframework.h"


using namespace openglframework;
using namespace nanogui;

// Declarations
class TestbedApplication;

// Class Gui
class Gui {

    protected :

        enum LeftPane {SCENES, PHYSICS, RENDERING, PROFILING};

        // -------------------- Constants -------------------- //


        // -------------------- Attributes -------------------- //

        Screen* mScreen;

        static double mScrollX, mScrollY;

        // -------------------- Methods -------------------- //

        static void displayLeftPane();

        /// Display the list of scenes
        static void displayScenesPane();

        static void displayPhysicsPane();
        static void displayRenderingPane();
        static void displayProfilingPane();

        static void resetScroll();

        /// Current time (in seconds) from last profiling time display
        static double mTimeSinceLastProfilingDisplay;

        /// Cached Framerate
        static double mCachedFPS;

        /// Cached update time
        static double mCachedUpdateTime;

        // Cached update physics time
        static double mCachedPhysicsUpdateTime;


    public :

        // -------------------- Methods -------------------- //

        /// Constructor
        Gui(Screen* screen);

        /// Destructor
        ~Gui();

        /// Initialize the GUI
        void init();

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

#endif
