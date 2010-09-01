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

#ifndef TIMER_H
#define TIMER_H

// Libraries
#include <stdexcept>
#include <iostream>
#include <ctime>
#include <cassert>

// Namespace ReactPhysics3D
namespace reactphysics3d {

/*  -------------------------------------------------------------------
    Class Timer :
        This class will take care of the time in the physics engine.
    -------------------------------------------------------------------
*/
class Timer {
    private :
        double timeStep;                  // Timestep dt of the physics engine (timestep > 0.0)
        long double time;                 // Current time of the physics engine
        long double lastUpdateTime;       // Last time the timer has been updated
        long double deltaTime;            // Time difference between the two last timer update() calls
        double accumulator;               // Used to fix the time step and avoid strange time effects
        bool isRunning;                   // True if the timer is running

    public :
        Timer(double timeStep);                                                 // Constructor
        virtual ~Timer();                                                       // Destructor

        double getTimeStep() const;                                             // Return the timestep of the physics engine
        void setTimeStep(double timeStep) throw(std::invalid_argument);         // Set the timestep of the physics engine
        long double getTime() const;                                            // Return the current time
        void start();                                                           // Start the timer
        void stop();                                                            // Stop the timer
        bool getIsRunning() const;                                              // Return true if the timer is running
        bool isPossibleToTakeStep() const;                                      // True if it's possible to take a new step
        void update();                                                          // Compute the time since the last update() call and add it to the accumulator
        void nextStep();                                                        // Take a new step => update the timer by adding the timeStep value to the current time
        double computeInterpolationFactor();                                    // Compute the interpolation factor
};

// --- Inline functions --- //

// Return the timestep of the physics engine
inline double Timer::getTimeStep() const {
    return timeStep;
}

// Set the timestep of the physics engine
inline void Timer::setTimeStep(double timeStep) throw(std::invalid_argument) {
    // Check if the timestep is different from zero
    if (timeStep != 0.0) {
        this->timeStep = timeStep;
    }
    else {
        // We throw an exception
        throw std::invalid_argument("Exception in Timer : the timestep has to be different from zero");
    }
}

// Return the current time
inline long double Timer::getTime() const {
    return time;
}

// Return if the timer is running
inline bool Timer::getIsRunning() const {
    return isRunning;
}

// Start the timer
inline void Timer::start() {
    if (!isRunning) {
        // Initialize the lastUpdateTime with the current time in seconds
        lastUpdateTime = std::clock() / double(CLOCKS_PER_SEC);
        accumulator = 0.0;
        isRunning = true;
    }
}

// Stop the timer
inline void Timer::stop() {
    if (isRunning) {
        isRunning = false;
    }
}

// True if it's possible to take a new step
inline bool Timer::isPossibleToTakeStep() const {
    return (accumulator >= timeStep);
}

// Take a new step => update the timer by adding the timeStep value to the current time
inline void Timer::nextStep() {
    assert(isRunning);

    // Update the current time of the physics engine
    time += timeStep;

    // Update the accumulator value
    accumulator -= timeStep;
}

// Compute the interpolation factor
inline double Timer::computeInterpolationFactor() {
    return (accumulator / timeStep);
}

// Compute the time since the last update() call and add it to the accumulator
inline void Timer::update() {

    // Compute the current time is seconds
    long double currentTime = std::clock() / double(CLOCKS_PER_SEC);
    
    // Compute the delta display time between two display frames
    deltaTime = currentTime - lastUpdateTime;

    // Update the current display time
    lastUpdateTime = currentTime;

    // Update the accumulator value
    accumulator += deltaTime;
}

} // End of the ReactPhysics3D namespace

 #endif
