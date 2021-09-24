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
#include <chrono>
#include <cassert>
#include <reactphysics3d/configuration.h>

// Class Timer
/**
 * This class will take care of the time in the physics engine. It
 * uses functions that depend on the current platform to get the
 * current time.
 */
class Timer {

    using clock = std::chrono::high_resolution_clock;

    private :

        // -------------------- Attributes -------------------- //

        /// Start physics time
        std::chrono::time_point<clock> mStartTime;

        /// Last time the timer has been updated
        std::chrono::time_point<clock> mLastUpdateTime;

        /// Used to fix the time step and avoid strange time effects
        std::chrono::duration<double> mAccumulator;

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
        Timer() : mAccumulator(0), mIsRunning(false) {

        }

        /// Return the elapsed physics time
        std::chrono::duration<double> getElapsedPhysicsTime() const;

        /// Start the timer
        void start();

        /// Stop the timer
        void stop();

        /// Reset the timer to zero
        void reset();

        /// Return true if the timer is running
        bool isRunning() const;

        /// True if it's possible to take a new step
        bool isPossibleToTakeStep(std::chrono::duration<double> timeStep) const;

        /// Compute the time since the last update() call and add it to the accumulator
        void update();

        /// Take a new step => update the timer by adding the timeStep value to the current time
        void nextStep(std::chrono::duration<double> timeStep);

        /// Compute the interpolation factor
        float computeInterpolationFactor(std::chrono::duration<double> timeStep);

        /// Return the current time of the system in seconds
        static std::chrono::time_point<clock> getCurrentSystemTime();
};

// Return the elapsed physics time
inline std::chrono::duration<double> Timer::getElapsedPhysicsTime() const {
    return mLastUpdateTime - mStartTime;
}

// Return if the timer is running
inline bool Timer::isRunning() const {
    return mIsRunning;
}

// Start the timer
inline void Timer::start() {

    if (!mIsRunning) {

        // Get the current system time
        mLastUpdateTime = clock::now();
        
        mAccumulator = std::chrono::duration<double>::zero();
        mIsRunning = true;
    }
}

// Stop the timer
inline void Timer::stop() {
    mIsRunning = false;
}

// Reset the timer to zero
inline void Timer::reset() {
    mAccumulator = std::chrono::milliseconds::zero();
    mStartTime = clock::now();
    mLastUpdateTime = mStartTime;
}

// True if it's possible to take a new step
inline bool Timer::isPossibleToTakeStep(std::chrono::duration<double> timeStep) const {
    return (mAccumulator >= timeStep);
}

// Take a new step => update the timer by adding the timeStep value to the current time
inline void Timer::nextStep(std::chrono::duration<double> timeStep) {
    assert(mIsRunning);

    // Update the accumulator value
    mAccumulator -= timeStep;
}

// Compute the interpolation factor
inline float Timer::computeInterpolationFactor(std::chrono::duration<double> timeStep) {
    return float(mAccumulator.count() / timeStep.count());
}

// Compute the time since the last update() call and add it to the accumulator
inline void Timer::update() {

    // Get the current system time
    std::chrono::time_point<clock> currentTime = clock::now();
    
    // Compute the delta display time between two display frames
    std::chrono::duration<double> deltaTime = currentTime - mLastUpdateTime;

    // Update the current display time
    mLastUpdateTime = currentTime;

    // Update the accumulator value
    mAccumulator += deltaTime;
}

 #endif
