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
#include "Scene.h"
#include <sstream>

// Declarations
void simulate();
void display();
void displayFPS();
void computeFPS();
void reshape(int width, int height);
void mouseButton(int button, int state, int x, int y);
void mouseMotion(int x, int y);
void keyboardSpecial(int key, int x, int y);
void init();

// Namespaces
using namespace openglframework;

// Global variables
GlutViewer* viewer;
Scene* scene;
int fps;
int nbFrames;
int currentTime;
int previousTime;
int width, height;

// Main function
int main(int argc, char** argv) {

    // Create and initialize the Viewer
    viewer = new GlutViewer();
    Vector2 windowsSize = Vector2(800, 600);
    Vector2 windowsPosition = Vector2(100, 100);
    width = windowsSize.x;
    height = windowsSize.y;
    bool initOK = viewer->init(argc, argv, "ReactPhysics3D Examples - Falling Cubes", windowsSize, windowsPosition);
    if (!initOK) return 1;

    // Create the scene
    scene = new Scene(viewer);

    init();

    nbFrames = 0;

    // Glut Idle function that is continuously called
    glutIdleFunc(simulate);
    glutDisplayFunc(display);
    glutReshapeFunc(reshape);
    glutMouseFunc(mouseButton);
    glutMotionFunc(mouseMotion);
    glutSpecialFunc(keyboardSpecial);

    // Glut main looop
    glutMainLoop();

    delete viewer;
    delete scene;

    return 0;
}

// Simulate function
void simulate() {

    // Physics simulation
    scene->simulate();

    computeFPS();

    // Ask GLUT to render the scene
    glutPostRedisplay ();
}

// Initialization
void init() {

    // Define the background color (black)
    glClearColor(0.0, 0.0, 0.0, 1.0);
}

// Reshape function
void reshape(int newWidth, int newHeight) {
    viewer->reshape(newWidth, newHeight);
    width = newWidth;
    height = newHeight;
}

// Called when a mouse button event occurs
void mouseButton(int button, int state, int x, int y) {
    viewer->mouseButtonEvent(button, state, x, y);
}

// Called when a mouse motion event occurs
void mouseMotion(int x, int y) {
    viewer->mouseMotionEvent(x, y);
}

// Called when the user hits a special key on the keyboard
void keyboardSpecial(int key, int x, int y) {
   /*
    if(key=='0')
            exit(0);
    if(key== GLUT_KEY_RIGHT) {
    */
}

// Display the scene
void display() {

    // Render the scene
    scene->render();

    // Display the FPS
    displayFPS();

    // Swap the buffers
    glutSwapBuffers();

    // Check the OpenGL errors
    GlutViewer::checkOpenGLErrors();
}

// Compute the FPS
void computeFPS() {
    nbFrames++;

    //  Get the number of milliseconds since glutInit called
    currentTime = glutGet(GLUT_ELAPSED_TIME);

    //  Calculate time passed
    int timeInterval = currentTime - previousTime;

    // Update the FPS counter each second
    if(timeInterval > 1000){

        //  calculate the number of frames per second
        fps = nbFrames / (timeInterval / 1000.0f);

         //  Set time
         previousTime = currentTime;

         //  Reset frame count
         nbFrames = 0;
    }
}

// Display the FPS
void displayFPS() {

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(0, width, height, 0, -1, 1);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    glRasterPos2i(10, 20);
    glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
    std::stringstream ss;
    ss << "FPS : " << fps;
    glutBitmapString(GLUT_BITMAP_HELVETICA_12, (const unsigned char*)ss.str().c_str());
}
