/****************************************************************************
 * Copyright (C) 2009      Daniel Chappuis                                  *
 ****************************************************************************
 * This file is part of ReactPhysics3D.                                     *
 *                                                                          *
 * ReactPhysics3D is free software: you can redistribute it and/or modify   *
 * it under the terms of the GNU Lesser General Public License as published *
 * by the Free Software Foundation, either version 3 of the License, or     *
 * (at your option) any later version.                                      *
 *                                                                          *
 * ReactPhysics3D is distributed in the hope that it will be useful,        *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of           *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the             *
 * GNU Lesser General Public License for more details.                      *
 *                                                                          *
 * You should have received a copy of the GNU Lesser General Public License *
 * along with ReactPhysics3D. If not, see <http://www.gnu.org/licenses/>.   *
 ***************************************************************************/

// Libraries
#include "Simulation.h"
#include "ReactDemo.h"
#include <iostream>

// We want to use the ReactPhysics3D namespace
using namespace reactphysics3d;

// Constructor of the class Simulation
Simulation::Simulation()
           :world(new PhysicsWorld(Vector3D(0.0, -9.8, 0.0))), engine(new PhysicsEngine(world, 0.005)), scene(this->world) {    // TODO : Change the timestep here after debugging
    simRunning = false;
    isStarted = false;
    mouseButtonPressed = false;
    nbFrame = 0;
    lastFrameTime = 0.0;
    fps = 0.0;
}

// Destructor of the class Simulation
Simulation::~Simulation() {
    // Delete the physics world object
    delete world;
    delete engine;
}

// Method to start the simulation
void Simulation::start() {
    // Initialisation of the OpenGL settings for the scene
    scene.init();

    // Reshape the windows for the first time
    scene.reshape(WINWIDTH, WINHEIGHT);

    // Add every rigid body to the dynamic world
    for (int i=0; i<context.getNbObjects(); ++i) {
        world->addBody(context.getObject(i).getRigidBody());
    }

    // Activation of the simulation
    simRunning = true;
    isStarted = true;

    // Get the current time
    //lastFrameTime = SDL_GetTicks();

    // Initialize the display time
    //engine->initializeDisplayTime(SDL_GetTicks()/1000.0);

    // Start the physics simulation
    engine->start();

    //double time = 1.0;

    // Main loop of the simulation
    while(simRunning) {
        double time = SDL_GetTicks()/1000.0;
        //time += 0.01;

        //std::cout << "************************************************* Time : " << time << std::endl;

        // Update the display time
        //engine->updateDisplayTime(time);

        // Update the physics
        if (isStarted)
            engine->update();

        // Check if an SDL event occured and make the apropriate actions
        checkEvents();

        // Display the actual scene
        scene.display(context);

        // Compute the fps (framerate)
        computeFps();
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
                                    //simRunning = false;
                                    if (isStarted) {
                                        engine->stop();
                                        isStarted = false;
                                    }
                                    else {
                                        engine->start();
                                        isStarted = true;
                                    }
                                    break;

            // The size of the windows changed then we reshape the windows
            case SDL_VIDEORESIZE:  scene.reshape(event.resize.w, event.resize.h);
				                    break;

			// If the mouse moved
			case SDL_MOUSEMOTION:    if (SDL_GetMouseState(NULL, NULL)&SDL_BUTTON(1)) {
                                        // Rotation of the outSideCamera
                                        scene.getOutSideCamera().modifyHorizontalAngleRotation(event.motion.xrel, fps);
                                        scene.getOutSideCamera().modifyVerticalAngleRotation(event.motion.yrel, fps);
                                    }
        }
    }
}

// Compute the framerate (fps) of the application
void Simulation::computeFps() {

    // Increment the number of frame in the last second
    nbFrame++;

    // Get the current  time
	double currentTime = SDL_GetTicks();

    // Compute the framerate
	if (currentTime - lastFrameTime > 1000.0) {
		fps = nbFrame * 1000.0/(currentTime-lastFrameTime);
	 	lastFrameTime = currentTime;
		nbFrame = 0;
	}
}
