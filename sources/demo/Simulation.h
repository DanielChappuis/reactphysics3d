#ifndef SIMULATION_H
#define SIMULATION_H

// Librairies
#include "Context.h"
#include "Scene.h"

// Class Simulation
class Simulation {
    private :
        Scene scene;                    // Scene object for displaying the simulation
        Context context;                // Context of the simulation
        bool simRunning;                // True if the simulation is running and false otherwise
        bool mouseButtonPressed;        // True if the left mouse button is pressed
    
    public :
         Simulation();                  // Constructor of the class
         ~Simulation();                 // Destructor of the class                              
         void start();                  // Start the simulation   
         void checkEvents();            // Check if SDL events occured and make the apropriate actions     
};


#endif SIMULATION_H
