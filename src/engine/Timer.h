/********************************************************************************
* ReactPhysics3D physics library, http://code.google.com/p/reactphysics3d/      *
* Copyright (c) 2010-2013 Daniel Chappuis                                       *
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


/// Namespace ReactPhysics3D
namespace reactphysics3d {

// Class Timer
/**
 * This class will take care of the time in the physics engine. It
 * uses fuunctions that depend on the current platform to get the
 * current time.
 */
class Timer {

    private :

        // -------------------- Attributes -------------------- //

        /// Timestep dt of the physics engine (timestep > 0.0)
        double mTimeStep;

        /// Current time of the physics engine
        long double mTime;

        /// Last time the timer has been updated
        long double mLastUpdateTime;

        /// Time difference between the two last timer update() calls
        long double mDeltaTime;

        /// Used to fix the time step and avoid strange time effects
        double mAccumulator;

        /// True if the timer is running
        bool mIsRunning;

        // -------------------- Methods -------------------- //

        /// Private copy-constructor
        Timer(const Timer& timer);

        /// Private assignment operator
        Timer& operator=(const Timer& timer);

    public :

        // -------------------- Methods -------------------- //

        /// Constructor
        Timer(double timeStep);

        /// Destructor
        virtual ~Timer();

        /// Return the timestep of the physics engine
        double getTimeStep() const;

        /// Set the timestep of the physics engine
        void setTimeStep(double timeStep);

        /// Return the current time
        long double getTime() const;

        /// Start the timer
        void start();

        /// Stop the timer
        void stop();

        /// Return true if the timer is running
        bool getIsRunning() const;

        /// True if it's possible to take a new step
        bool isPossibleToTakeStep() const;

        /// Compute the time since the last update() call and add it to the accumulator
        void update();

        /// Take a new step => update the timer by adding the timeStep value to the current time
        void nextStep();

        /// Compute the interpolation factor
        double computeInterpolationFactor();
};

// Return the timestep of the physics engine
inline double Timer::getTimeStep() const {
    return mTimeStep;
}

// Set the timestep of the physics engine
inline void Timer::setTimeStep(double timeStep) {
    assert(timeStep > 0.0f);
    mTimeStep = timeStep;
}

// Return the current time
inline long double Timer::getTime() const {
    return mTime;
}

// Return if the timer is running
inline bool Timer::getIsRunning() const {
    return mIsRunning;
}

// Start the timer
inline void Timer::start() {
    if (!mIsRunning) {
        
#if defined(WINDOWS_OS)
        LARGE_INTEGER ticksPerSecond;
        LARGE_INTEGER ticks;
        QueryPerformanceFrequency(&ticksPerSecond);
        QueryPerformanceCounter(&ticks);
        mLastUpdateTime = double(ticks.QuadPart) / double(ticksPerSecond.QuadPart);
#else
        // Initialize the lastUpdateTime with the current time in seconds
        timeval timeValue;
        gettimeofday(&timeValue, NULL);
        mLastUpdateTime = timeValue.tv_sec + (timeValue.tv_usec / 1000000.0);
#endif
        
        mAccumulator = 0.0;
        mIsRunning = true;
    }
}

// Stop the timer
inline void Timer::stop() {
    std::cout << "Timer stop" << std::endl;
    mIsRunning = false;
}

// True if it's possible to take a new step
inline bool Timer::isPossibleToTakeStep() const {
    return (mAccumulator >= mTimeStep);
}

// Take a new step => update the timer by adding the timeStep value to the current time
inline void Timer::nextStep() {
    assert(mIsRunning);

    // Update the current time of the physics engine
    mTime += mTimeStep;

    // Update the accumulator value
    mAccumulator -= mTimeStep;
}

// Compute the interpolation factor
inline double Timer::computeInterpolationFactor() {
    return (mAccumulator / mTimeStep);
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
    mDeltaTime = currentTime - mLastUpdateTime;

    // Update the current display time
    mLastUpdateTime = currentTime;

    // Update the accumulator value
    mAccumulator += mDeltaTime;
}

}

 #endif
