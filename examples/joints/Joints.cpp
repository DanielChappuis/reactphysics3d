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
#include "../common/Viewer.h"

// Declarations
void simulate();
void update();
void render();
void mouseButton(GLFWwindow* window, int button, int action, int mods);
void mouseMotion(GLFWwindow* window, double x, double y);
void keyboard(GLFWwindow* window, int key, int scancode, int action, int mods);
void scroll(GLFWwindow* window, double xAxis, double yAxis);
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
    viewer->init(argc, argv, "ReactPhysics3D Examples - Joints", windowsSize, windowsPosition, true);

    // Register callback methods
    viewer->registerUpdateFunction(update);
    viewer->registerKeyboardCallback(keyboard);
    viewer->registerMouseButtonCallback(mouseButton);
    viewer->registerMouseCursorCallback(mouseMotion);
    viewer->registerScrollingCallback(scroll);

    // Create the scene
    scene = new Scene(viewer);

    init();

    viewer->startMainLoop();

    delete viewer;
    delete scene;

    return 0;
}

// Update function that is called each frame
void update() {

    // Take a simulation step
    simulate();

    // Render
    render();
}

// Simulate function
void simulate() {

    // Physics simulation
    scene->simulate();

    viewer->computeFPS();
}

// Initialization
void init() {

    // Define the background color (black)
    glClearColor(0.0, 0.0, 0.0, 1.0);
}

// Callback method to receive keyboard events
void keyboard(GLFWwindow* window, int key, int scancode, int action, int mods) {
    if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS) {
        glfwSetWindowShouldClose(window, GL_TRUE);
    }
    else if (key == GLFW_KEY_SPACE && action == GLFW_PRESS) {
        scene->pauseContinueSimulation();
    }
}

// Callback method to receive scrolling events
void scroll(GLFWwindow* window, double xAxis, double yAxis) {
    viewer->scrollingEvent(static_cast<float>(yAxis));
}

// Called when a mouse button event occurs
void mouseButton(GLFWwindow* window, int button, int action, int mods) {
    viewer->mouseButtonEvent(button, action);
}

// Called when a mouse motion event occurs
void mouseMotion(GLFWwindow* window, double x, double y) {
    viewer->mouseMotionEvent(x, y);
}

// Display the scene
void render() {

    // Render the scene
    scene->render();

    // Display the FPS
    viewer->displayGUI();

    // Check the OpenGL errors
    Viewer::checkOpenGLErrors();
}


