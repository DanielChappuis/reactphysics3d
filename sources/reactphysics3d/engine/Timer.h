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
        long double currentDisplayTime;   // Current display time
        long double deltaDisplayTime;     // Current time difference between two display frames
        double accumulator;               // Used to fix the time step and avoid strange time effects
        bool isRunning;                   // True if the timer is running

    public :
        Timer(long double initialTime, double timeStep) throw(std::invalid_argument);  // Constructor
        Timer(const Timer& timer);                                                     // Copy-constructor
        virtual ~Timer();                                                              // Destructor

        double getTimeStep() const;                                               // Return the timestep of the physics engine
        void setTimeStep(double timeStep) throw(std::invalid_argument);    // Set the timestep of the physics engine
        long double getTime() const;                                                   // Return the current time
        void setTime(long double time);                                         // Set the current time
        bool getIsRunning() const;                                              // Return if the timer is running
        void setIsRunning(bool isRunning);                                      // Set if the timer is running
        double getAccumulator() const;                                          // Return the accumulator value
        void setCurrentDisplayTime(long double displayTime);                    // Set the current display time

        void update();                                                          // Update the timer by adding some time value (or timeStep by default) to the current time
        double getInterpolationFactor() const;                                  // Compute and return the interpolation factor between two body states
        void updateDisplayTime(long double newDisplayTime);                     // Set the new currentDisplayTime value
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

// Set the current time
inline void Timer::setTime(long double time) {
    this->time = time;
}

// Return if the timer is running
inline bool Timer::getIsRunning() const {
    return isRunning;
}

// Set if the timer is running
inline void Timer::setIsRunning(bool isRunning) {
    this->isRunning = isRunning;
}

// Return the accumulator value
inline double Timer::getAccumulator() const {
    return accumulator;
}

// Set the current display time
inline void Timer::setCurrentDisplayTime(long double currentDisplayTime) {
    this->currentDisplayTime = currentDisplayTime;
}

// Update the timer by adding the timeStep to the current time
inline void Timer::update() {
    // Check if the timer is running
    if (isRunning) {
        // Update the current time of the physics engine
        time += timeStep;

        // Update the accumulator value
        accumulator -= timeStep;
    }
}

// Compute and return the interpolation factor between two body states
inline double Timer::getInterpolationFactor() const {
    // Compute and return the interpolation factor
    return (accumulator / timeStep);
}

// Set the new currentDisplayTime value
inline void Timer::updateDisplayTime(long double newDisplayTime) {
    // Compute the delta display time between two display frames
    deltaDisplayTime = newDisplayTime - currentDisplayTime;

    // Update the current display time
    currentDisplayTime = newDisplayTime;

    // Update the accumulator value
    accumulator += deltaDisplayTime;
}

} // End of the ReactPhysics3D namespace

 #endif
