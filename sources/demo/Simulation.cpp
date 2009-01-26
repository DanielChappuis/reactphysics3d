
// Libraries
#include "Simulation.h"
#include "ReactDemo.h"


// Constructor of the class Simulation
Simulation::Simulation() {    
    simRunning = false;
    mouseButtonPressed = false;
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

    // Main loop of the simulation
    while(simRunning) {
        // Check if an SDL event occured and make the apropriate actions
        checkEvents();
        
        // Display the actual scene
        scene.display(context);
    }
}

// This method checks if an events occur and call the apropriate method
void Simulation::checkEvents() {
    SDL_Event event;                                // An SDL event

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
			case SDL_MOUSEMOTION:    if(SDL_GetMouseState(NULL, NULL)&SDL_BUTTON(1)) {
                                            // If the left mouse button is pressed then change the angle
                                            scene.setCameraAngle1(scene.getCameraAngle1() + event.motion.xrel % 360);
                                            scene.setCameraAngle2(scene.getCameraAngle2() + event.motion.yrel % 360);
                                     }
                                     break;
        }
    }
}
