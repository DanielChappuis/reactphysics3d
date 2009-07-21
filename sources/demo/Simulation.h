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

#ifndef SIMULATION_H
#define SIMULATION_H

// Librairies
#include "Context.h"
#include "Scene.h"
#include "../reactphysics3d/reactphysics3d.h"

// Class Simulation
class Simulation {
    private :
        Scene scene;                    // Scene object for displaying the simulation
        Context context;                // Context of the simulation
        rp3d::CollisionWorld* world;    // Pointer to the collision world that contains bodies of the simulation
        rp3d::CollisionEngine engine;   // Collision engine for the physics of the simulation
        bool simRunning;                // True if the simulation is running and false otherwise
        bool mouseButtonPressed;        // True if the left mouse button is pressed
        double lastFrameTime;           // Last frame time
        int nbFrame;                    // Number of frame (used to compute the framerate)
        double fps;                     // Framerate of the application

        void computeFps();              // Compute the framerate of the application

    public :
         Simulation();                  // Constructor of the class
         ~Simulation();                 // Destructor of the class
         void start();                  // Start the simulation
         void checkEvents();            // Check if SDL events occured and make the apropriate actions
};

#endif
