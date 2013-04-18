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

// Class Viewer
class Viewer : public openglframework::GlutViewer {

    private :

        // -------------------- Attributes -------------------- //

        /// Current number of frames per seconds
        int fps;

        /// Number of frames during the last second
        int nbFrames;

        /// Current time for fps computation
        int currentTime;

        /// Previous time for fps computation
        int previousTime;

        // -------------------- Methods -------------------- //

        /// Display the FPS
        void displayFPS();

    public :

        // -------------------- Methods -------------------- //

        /// Constructor
        Viewer();

        /// Compute the FPS
        void computeFPS();

        /// Display the GUI
        void displayGUI();

};

#endif
