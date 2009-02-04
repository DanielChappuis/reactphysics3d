/****************************************************************************
 * Copyright (C) 2009      Daniel Chappuis                                  *
 ****************************************************************************
 * This file is part of ReactPhysics3D.                                     *
 *                                                                          *
 * ReactPhysics3D is free software: you can redistribute it and/or modify   *
 * it under the terms of the GNU General Public License as published by     *
 * the Free Software Foundation, either version 3 of the License, or        *
 * (at your option) any later version.                                      *
 *                                                                          *
 * ReactPhysics3D is distributed in the hope that it will be useful,        *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of           *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the             *
 * GNU General Public License for more details.                             *
 *                                                                          *
 * You should have received a copy of the GNU General Public License        *
 * along with ReactPhysics3D. If not, see <http://www.gnu.org/licenses/>.   *
 ***************************************************************************/

// Libraries
#include "Simulation.h"
#include "ReactDemo.h"
#include <iostream>

// Constructor of the class Simulation
Simulation::Simulation() {
    simRunning = false;
    mouseButtonPressed = false;
    fps = 0.0;
}

// Destructor of the class Simulation
Simulation::~Simulation() {

}

// Method to start the simulation
void Simulation::start() {
    // Initialisation of the OpenGL settings for the scene
    scene.init();

    // Reshape the windows for the first time
    scene.reshape(WINWIDTH, WINHEIGHT);

    // Activation of the simulation
    simRunning = true;

    // Get the current time
    currentFrameTime = SDL_GetTicks();

    // Main loop of the simulation
    while(simRunning) {
        // Check if an SDL event occured and make the apropriate actions
        checkEvents();

        // Display the actual scene
        scene.display(context);

        // Compute the fps (framerate)
        computeFps();

        //std::cout << fps << std::endl;
    }
}

// This method checks if an events occur and call the apropriate method
void Simulation::checkEvents() {
    SDL_Event event;                                // An SDL event

    // Zoom of the outside camera
    if (SDL_GetKeyState(NULL)[SDLK_UP]) {
        scene.getOutSideCamera().decreaseDistance(fps);
    }
    else if(SDL_GetKeyState(NULL)[SDLK_DOWN]) {
        scene.getOutSideCamera().increaseDistance(fps);
    }

    // Check in the stack of events
    while(SDL_PollEvent(&event)) {
        // Check an event
        switch(event.type) {
            // An QUIT event occur
            case SDL_QUIT:          simRunning = false;
                                    break;

            // A keyboard key has been pushed
            case SDL_KEYDOWN:       // The Esc key has been pushed then we end the simulation
                                    if (event.key.keysym.sym == SDLK_ESCAPE)
                                    simRunning = false;
                                    break;

            // The size of the windows changed then we reshape the windows
            case SDL_VIDEORESIZE:  scene.reshape(event.resize.w, event.resize.h);
				                    break;

			// If the mouse moved
			case SDL_MOUSEMOTION:    if (SDL_GetMouseState(NULL, NULL)&SDL_BUTTON(1)) {
                                        // Rotation of the outSideCamera
                                        // TODO : Problem here when we try to implement fps indepence (if we try to
                                        //        replace 60 by the variable fps
                                        scene.getOutSideCamera().modifyHorizontalAngleRotation(event.motion.xrel, 30);
                                        scene.getOutSideCamera().modifyVerticalAngleRotation(event.motion.yrel, 30);
                                    }
        }
    }
}

// Compute the framerate (fps) of the application
void Simulation::computeFps() {
    double lastFrameTime = currentFrameTime;

    // Get the current time
    currentFrameTime = SDL_GetTicks();

    // Compute the new framerate
    fps = 1000 / double(currentFrameTime - lastFrameTime);
}
