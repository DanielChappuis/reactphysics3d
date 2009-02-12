/****************************************************************************
 * Copyright (C) 2009      Daniel Chappuis                                  *
 ****************************************************************************
 * This file is part of ReactPhysics3D.                                     *
 *                                                                          *
 * ReactPhysics3D is free software: you can redistribute it and/or modify   *
 * it under the terms of the GNU General Public License as published by     *
 * the Free Software Foundation, either version 3 of the License, or        *
 * (at your option) any later version.                                      *
 *                                                                          *
 * ReactPhysics3D is distributed in the hope that it will be useful,        *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of           *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the             *
 * GNU General Public License for more details.                             *
 *                                                                          *
 * You should have received a copy of the GNU General Public License        *
 * along with ReactPhysics3D. If not, see <http://www.gnu.org/licenses/>.   *
 ***************************************************************************/

 #ifndef TIMER_H
 #define TIMER_H

 // Libraries
 #include "../physics/physics.h"
 #include <stdexcept>

 // Namespace ReactPhysics3D
 namespace reactphysics3d {

/*  -------------------------------------------------------------------
    Class Timer :
        This class will take care of the time in the physics engine.
    -------------------------------------------------------------------
*/
class Timer {
    private :
        Time timeStep;                  // Timestep dt of the physics engine (timestep > 0.0)
        Time time;                      // Current time of the physics engine
        Time currentDisplayTime;        // Current display time
        Time deltaDisplayTime;          // Current time difference between two display frames
        double accumulator;             // Used to fix the time step and avoid strange time effects
        bool isRunning;                 // True if the timer is running

    public :
        Timer(const Time& initialTime, const Time& timeStep) throw(std::invalid_argument);  // Constructor
        Timer(const Timer& timer);                                                          // Copy-constructor
        virtual ~Timer();                                                                   // Destructor

        Time getTimeStep() const;                                               // Return the timestep of the physics engine
        void setTimeStep(const Time& timeStep) throw(std::invalid_argument);    // Set the timestep of the physics engine
        Time getTime() const;                                                   // Return the current time
        void setTime(const Time& time);                                         // Set the current time
        bool getIsRunning() const;                                              // Return if the timer is running
        void setIsRunning(bool isRunning);                                      // Set if the timer is running
        double getAccumulator() const;                                           // Return the accumulator value

        void update();                                          // Update the timer
        double getInterpolationFactor() const;                  // Compute and return the interpolation factor between two body states
        void updateDisplayTime(const Time& newDisplayTime);     // Set the new currentDisplayTime value
};

// --- Inline functions --- //

// Return the timestep of the physics engine
inline Time Timer::getTimeStep() const {
    return timeStep;
}

// Set the timestep of the physics engine
inline void Timer::setTimeStep(const Time& timeStep) throw(std::invalid_argument) {
    // Check if the timestep is different from zero
    if (timeStep.getValue() != 0.0) {
        this->timeStep = timeStep;
    }
    else {
        // We throw an exception
        throw std::invalid_argument("Exception in Timer : the timestep has to be different from zero");
    }
}

// Return the current time
inline Time Timer::getTime() const {
    return time;
}

// Set the current time
inline void Timer::setTime(const Time& time) {
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

// Update the timer
inline void Timer::update() {
    // Update the current time of the physics engine
    time.setValue(time.getValue() + timeStep.getValue());

    // Update the accumulator value
    accumulator -= timeStep.getValue();
}

// Compute and return the interpolation factor between two body states
inline double Timer::getInterpolationFactor() const {
    // Compute and return the interpolation factor
    return (accumulator / timeStep.getValue());
}

// Set the new currentDisplayTime value
inline void Timer::updateDisplayTime(const Time& newDisplayTime) {
    // Compute the delta display time between two display frames
    deltaDisplayTime.setValue(newDisplayTime.getValue() - currentDisplayTime.getValue());

    // Update the current display time
    currentDisplayTime.setValue(newDisplayTime.getValue());

    // Update the accumulator value
    accumulator += deltaDisplayTime.getValue();
}

} // End of the ReactPhysics3D namespace

 #endif
