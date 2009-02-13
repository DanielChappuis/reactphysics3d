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

// Libraries
#include "Timer.h"

// We want to use the ReactPhysics3D namespace
using namespace reactphysics3d;

// Constructor
Timer::Timer(const Time& initialTime, const Time& timeStep) throw(std::invalid_argument)
      : timeStep(timeStep), time(initialTime), currentDisplayTime(Time(0.0)), deltaDisplayTime(Time(0.0)) {
    // Check if the timestep is different from zero
    if (timeStep.getValue() != 0.0) {
        accumulator = 0.0;
        isRunning = false;
    }
    else {
        // We throw an exception
        throw std::invalid_argument("Exception in Timer constructor : The timestep has to be different from zero");
    }
}


// Copy-constructor
Timer::Timer(const Timer& timer)
      : timeStep(timer.timeStep), time(timer.time), currentDisplayTime(timer.currentDisplayTime),
        deltaDisplayTime(timer.deltaDisplayTime), accumulator(timer.accumulator) {
    isRunning = timer.isRunning;
}

// Destructor
Timer::~Timer() {

}









