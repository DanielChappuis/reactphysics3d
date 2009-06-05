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

 // Libraries
 #include "Time.h"

// We want to use the ReactPhysics3D namespace
using namespace reactphysics3d;

// Constructor
Time::Time() {
    value = 0.0;
}

// Constructor with arguments
Time::Time(double value) throw(std::invalid_argument) {
    // Check if the value is positive
    if (value >= 0.0) {
        this->value = value;
    }
    else {
        // We throw an exception
        throw std::invalid_argument("Exception in Time : Wrong argument, a time value has to be positive");
    }
 }

 // Copy-constructor
 Time::Time(const Time& time) {
    this->value = time.value;
 }

 // Destructor
 Time::~Time() {

 }
