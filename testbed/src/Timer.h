/********************************************************************************
* ReactPhysics3D physics library, http://www.reactphysics3d.com                 *
* Copyright (c) 2010-2016 Daniel Chappuis                                       *
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
#include <reactphysics3d/configuration.h>

#if defined(WINDOWS_OS)   // For Windows platform
   #define NOMINMAX       // This is used to avoid definition of max() and min() macros
   #include <windows.h>
#else                                   // For Mac OS or Linux platform
   #include <sys/time.h>
#endif

// Class Timer
/**
 * This class will take care of the time in the physics engine. It
 * uses functions that depend on the current platform to get the
 * current time.
 */
class Timer {

    private :

        // -------------------- Attributes -------------------- //

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
        Timer();

        /// Destructor
        ~Timer();

        /// Return the current time of the physics engine
        long double getPhysicsTime() const;

        /// Start the timer
        void start();

        /// Stop the timer
        void stop();

        /// Return true if the timer is running
        bool isRunning() const;

        /// True if it's possible to take a new step
        bool isPossibleToTakeStep(float timeStep) const;

        /// Compute the time since the last update() call and add it to the accumulator
        void update();

        /// Take a new step => update the timer by adding the timeStep value to the current time
        void nextStep(float timeStep);

        /// Compute the interpolation factor
        float computeInterpolationFactor(float timeStep);

        /// Return the current time of the system in seconds
        static long double getCurrentSystemTime();
};

// Return the current time
inline long double Timer::getPhysicsTime() const {
    return mLastUpdateTime;
}

// Return if the timer is running
inline bool Timer::isRunning() const {
    return mIsRunning;
}

// Start the timer
inline void Timer::start() {
    if (!mIsRunning) {

        // Get the current system time
        mLastUpdateTime = getCurrentSystemTime();
        
        mAccumulator = 0.0;
        mIsRunning = true;
    }
}

// Stop the timer
inline void Timer::stop() {
    mIsRunning = false;
}

// True if it's possible to take a new step
inline bool Timer::isPossibleToTakeStep(float timeStep) const {
    return (mAccumulator >= timeStep);
}

// Take a new step => update the timer by adding the timeStep value to the current time
inline void Timer::nextStep(float timeStep) {
    assert(mIsRunning);

    // Update the accumulator value
    mAccumulator -= timeStep;
}

// Compute the interpolation factor
inline float Timer::computeInterpolationFactor(float timeStep) {
    return (float(mAccumulator) / timeStep);
}

// Compute the time since the last update() call and add it to the accumulator
inline void Timer::update() {

    // Get the current system time
    long double currentTime = getCurrentSystemTime();
    
    // Compute the delta display time between two display frames
    mDeltaTime = currentTime - mLastUpdateTime;

    // Update the current display time
    mLastUpdateTime = currentTime;

    // Update the accumulator value
    mAccumulator += mDeltaTime;
}

 #endif
