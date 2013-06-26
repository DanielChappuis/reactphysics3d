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
#include "Viewer.h"

// Declarations
void simulate();
void display();
void finish();
void reshape(int width, int height);
void mouseButton(int button, int state, int x, int y);
void mouseMotion(int x, int y);
void keyboard(unsigned char key, int x, int y);
void init();

// Namespaces
using namespace openglframework;

// Global variables
Viewer* viewer;
Scene* scene;

// Main function
int main(int argc, char** argv) {

    // Create and initialize the Viewer
    viewer = new Viewer();
    Vector2 windowsSize = Vector2(800, 600);
    Vector2 windowsPosition = Vector2(100, 100);
    bool initOK = viewer->init(argc, argv, "ReactPhysics3D Examples - Joints", windowsSize, windowsPosition);
    if (!initOK) return 1;

    // Create the scene
    scene = new Scene(viewer);

    init();

    // Glut Idle function that is continuously called
    glutIdleFunc(simulate);
    glutDisplayFunc(display);
    glutReshapeFunc(reshape);
    glutMouseFunc(mouseButton);
    glutMotionFunc(mouseMotion);
    glutKeyboardFunc(keyboard);
    glutCloseFunc(finish);

    // Glut main looop
    glutMainLoop();

    return 0;
}

// Simulate function
void simulate() {

    // Physics simulation
    scene->simulate();

    viewer->computeFPS();

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
void keyboard(unsigned char key, int x, int y) {
    switch(key) {

        // Escape key
        case 27:
            glutLeaveMainLoop();
            break;

        // Space bar
        case 32:
            scene->pauseContinueSimulation();
            break;
    }
}

// End of the application
void finish() {

    // Destroy the viewer and the scene
    delete viewer;
    delete scene;
}

// Display the scene
void display() {

    // Render the scene
    scene->render();

    // Display the FPS
    viewer->displayGUI();

    // Swap the buffers
    glutSwapBuffers();

    // Check the OpenGL errors
    GlutViewer::checkOpenGLErrors();
}


