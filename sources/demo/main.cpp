
// Libraries
#include "Simulation.h"
#include "ReactDemo.h"

#include <iostream>
#include <SDL/SDL.h>


// Prototypes
int initSDL();

// global variables
SDL_Surface * pScreen;                          // Pointer to the SDL windows

// Main function
int main(int argc, char** argv) {

    if(initSDL() > 0) {
        // If initSDL return an error then we exit the program
        return EXIT_FAILURE;
    }

    // Initialize the glut library. We will use glut only to display shapes like cube or sphere
    glutInit(&argc, argv);

    // Create a Simulation object used to simulate a physic world
    Simulation simulation;

    // Start the simulation
    simulation.start();

    std::cerr << "Fin normale du programme" << std::endl;

    // To avoid warnings notifying that argc et argv aren't used
    (argc);
    (argv);

    return (EXIT_SUCCESS);

    return 0;
}

// Methode to initialise the SDL and the windows
int initSDL() {
    // Initialisation of SDL
    if( SDL_Init(SDL_INIT_VIDEO) < 0) {
        std::cerr << "Echec SDL_Init : " << SDL_GetError() << std::endl;
        return 1;
    }

    // Select the method to exit
    atexit( SDL_Quit );

    // Active double buffer mode
    SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);

    // Select the Depth Buffer size
	SDL_GL_SetAttribute( SDL_GL_DEPTH_SIZE, 16);

    // Activation of displaying in windows mode
    SDL_Surface* pScreen = SDL_SetVideoMode(WINWIDTH, WINHEIGHT, 32, SDL_OPENGL | SDL_GL_DOUBLEBUFFER | SDL_RESIZABLE | SDL_SWSURFACE);

    // Check that displaying is activated
    if( !pScreen ) {
        std::cerr << "Echec de creation de la fenetre en 640x480 : " << SDL_GetError() << std::endl;
        return 1;
    }

    // Define the window title and the window icon
    SDL_WM_SetCaption("React Demo 0.0.1", NULL);

     // Get the state of the Double Buffer parameter
    int nValue;
    if( SDL_GL_GetAttribute(SDL_GL_DOUBLEBUFFER, &nValue) < 0) {
        std::cerr << "Echec de recuperation du parametre SDL_GL_DOUBLEBUFFER : " << SDL_GetError() << std::endl;
        return 1;
    }

    // Check that Double Buffer mode is activated
    if(nValue != 1) {
        std::cerr << "Erreur : SDL_GL_DOUBLEBUFFER inactif" << std::endl;
        return 1;
    }

    return 0;
}
