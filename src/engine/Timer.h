/********************************************************************************
* ReactPhysics3D physics library, http://code.google.com/p/reactphysics3d/      *
* Copyright (c) 2010 Daniel Chappuis                                            *
*********************************************************************************
*                                                                               *
* Permission is hereby granted, free of charge, to any person obtaining a copy  *
* of this software and associated documentation files (the "Software"), to deal *
* in the Software without restriction, including without limitation the rights  *
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell     *
* copies of the Software, and to permit persons to whom the Software is         *
* furnished to do so, subject to the following conditions:                      *
*                                                                               *
* The above copyright notice and this permission notice shall be included in    *
* all copies or substantial portions of the Software.                           *
*                                                                               *
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR    *
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,      *
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE   *
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER        *
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, *
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN     *
* THE SOFTWARE.                                                                 *
********************************************************************************/

#ifndef TIMER_H
#define TIMER_H

// TODO : Test this code on Windows platform
// TODO : Test this code on Linux platform

// Libraries
#include <stdexcept>
#include <iostream>
#include <ctime>
#include <cassert>

#if defined(WIN32) || defined(_WIN32)   // For Windows platform
   #define WINDOWS_TIME
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
        
#ifdef WINDOWS_TIME
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
    
#ifdef WINDOWS_TIME
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
