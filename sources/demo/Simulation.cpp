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
           :world(new PhysicsWorld(Vector3D(0.0, -0.6, 0.0))), engine(world, Time(0.01)), scene(this->world) {
    simRunning = false;
    mouseButtonPressed = false;
    nbFrame = 0;
    lastFrameTime = 0.0;
    fps = 0.0;
}

// Destructor of the class Simulation
Simulation::~Simulation() {
    // Delete the physics world object
    delete world;
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

    // Get the current time
    lastFrameTime = SDL_GetTicks();

    PhysicsEngine* pEngine = &engine;

    // Initialize the display time
    pEngine->initializeDisplayTime(Time(SDL_GetTicks()/1000.0));

    // Start the physics simulation
    pEngine->start();

    //double time = 1.0;

    // Main loop of the simulation
    while(simRunning) {
        // Check if an SDL event occured and make the apropriate actions
        checkEvents();

        double time = SDL_GetTicks()/1000.0;
        //time += 0.01;

        //std::cout << "************************************************* Time : " << time << std::endl;

        // Update the display time
        pEngine->updateDisplayTime(Time(time));

        // Update the physics
        pEngine->updateCollision();

        // Display the actual scene
        scene.display(context);

        // Compute the fps (framerate)
        computeFps();
        //std::cout << "FPS : " << fps << std::endl;

        /*
        BodyState state = context.getObject(0).getRigidBody()->getInterpolatedState();
        Vector3D velocity = context.getObject(0).getRigidBody()->getInterpolatedState().getAngularVelocity();
        //std::cout << "Velocity 0 : " << velocity.getX() << ", " << velocity.getY() << ", " << velocity.getZ() << ")" << std::endl;
        double x = state.getPosition().getX();
        double y = state.getPosition().getY();
        double z = state.getPosition().getZ();
        std::cout << "Position Cube 0 : (" << x << ", " << y << ", " << z << ")" << std::endl;
        std::cout << "angular velocity 0 : " << velocity.length() << std::endl;;

        BodyState state1 = context.getObject(1).getRigidBody()->getInterpolatedState();
        Vector3D velocity1 = context.getObject(1).getRigidBody()->getInterpolatedState().getAngularVelocity();
        //std::cout << "Velocity 1 : " << velocity1.getX() << ", " << velocity1.getY() << ", " << velocity1.getZ() << ")" << std::endl;
        double x1 = state1.getPosition().getX();
        double y1 = state1.getPosition().getY();
        double z1 = state1.getPosition().getZ();
        std::cout << "Position Cube 1 : (" << x1 << ", " << y1 << ", " << z1 << ")" << std::endl;
        std::cout << "angular velocity 1 : " << velocity1.length() << std::endl;

        BodyState state2 = context.getObject(2).getRigidBody()->getInterpolatedState();
        Quaternion velocity2 = context.getObject(2).getRigidBody()->getInterpolatedState().getOrientation();
        //std::cout << "Velocity 2 : " << velocity2.getX() << ", " << velocity2.getY() << ", " << velocity2.getZ() << ")" << std::endl;
        double x2 = state2.getPosition().getX();
        double y2 = state2.getPosition().getY();
        double z2 = state2.getPosition().getZ();
        std::cout << "Position Cube 2: (" << x2 << ", " << y2 << ", " << z2 << ")" << std::endl;
        std::cout << "quaternion orientation 2 : " << velocity2.getX() << ", " << velocity2.getY() << ", " << velocity2.getZ() << ", " << velocity2.getW() << ")" << std::endl;;
        */

        /*
        double a;
        if (time > 5.0) {
            std::cin >> a;
        }
        */

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
