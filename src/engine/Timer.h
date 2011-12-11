/********************************************************************************
* ReactPhysics3D physics library, http://code.google.com/p/reactphysics3d/      *
* Copyright (c) 2010-2012 Daniel Chappuis                                       *
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

#ifndef TIMER_H
#define TIMER_H

// Libraries
#include <stdexcept>
#include <iostream>
#include <ctime>
#include <cassert>
#include "../configuration.h"

#if defined(WINDOWS_OS)   // For Windows platform
   #include <windows.h>
#else                                   // For Mac OS or Linux platform
   #include <sys/time.h>
#endif


// Namespace ReactPhysics3D
namespace reactphysics3d {

/*  -------------------------------------------------------------------
    Class Timer :
        This class will take care of the time in the physics engine. It
        uses fuunctions that depend on the current platform to get the
        current time.
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
        
#if defined(WINDOWS_OS)
        LARGE_INTEGER ticksPerSecond;
        LARGE_INTEGER ticks;
        QueryPerformanceFrequency(&ticksPerSecond);
        QueryPerformanceCounter(&ticks);
        lastUpdateTime = double(ticks.QuadPart) / double(ticksPerSecond.QuadPart);
#else
        // Initialize the lastUpdateTime with the current time in seconds
        timeval timeValue;
        gettimeofday(&timeValue, NULL);
        lastUpdateTime = timeValue.tv_sec + (timeValue.tv_usec / 1000000.0);
#endif
        
        accumulator = 0.0;
        isRunning = true;
    }
}

// Stop the timer
inline void Timer::stop() {
    isRunning = false;
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
    long double currentTime;
    
#if defined(WINDOWS_OS)
   LARGE_INTEGER ticksPerSecond;
   LARGE_INTEGER ticks;
   QueryPerformanceFrequency(&ticksPerSecond);
   QueryPerformanceCounter(&ticks);
   currentTime = double(ticks.QuadPart) / double(ticksPerSecond.QuadPart);
#else
    // Compute the current time is seconds
    timeval timeValue;
    gettimeofday(&timeValue, NULL);
    currentTime = timeValue.tv_sec + (timeValue.tv_usec / 1000000.0);
#endif
    
    // Compute the delta display time between two display frames
    deltaTime = currentTime - lastUpdateTime;

    // Update the current display time
    lastUpdateTime = currentTime;

    // Update the accumulator value
    accumulator += deltaTime;
}

} // End of the ReactPhysics3D namespace

 #endif
