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
#include <sstream>

// Constructor
Viewer::Viewer() : openglframework::GlutViewer(), fps(0), nbFrames(0) {

}

// Compute the FPS
void Viewer::computeFPS() {

    nbFrames++;

    //  Get the number of milliseconds since glutInit called
    currentTime = glutGet(GLUT_ELAPSED_TIME);

    //  Calculate time passed
    int timeInterval = currentTime - previousTime;

    // Update the FPS counter each second
    if(timeInterval > 1000){

        //  calculate the number of frames per second
        fps = static_cast<int>(nbFrames / (timeInterval / 1000.0f));

         //  Set time
         previousTime = currentTime;

         //  Reset frame count
         nbFrames = 0;
    }
}

// Display the GUI
void Viewer::displayGUI() {

    // Display the FPS
    displayFPS();
}

// Display the FPS
void Viewer::displayFPS() {

#ifdef USE_FREEGLUT
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(0, mCamera.getWidth(), mCamera.getHeight(), 0, -1, 1);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    glRasterPos2i(10, 20);
    glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
    std::stringstream ss;
    ss << "FPS : " << fps;
    glutBitmapString(GLUT_BITMAP_HELVETICA_12, (const unsigned char*)ss.str().c_str());
#endif
}
