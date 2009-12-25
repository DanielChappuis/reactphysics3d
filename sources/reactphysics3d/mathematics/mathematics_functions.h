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

#ifndef MATHEMATICS_FUNCTIONS_H
#define MATHEMATICS_FUNCTIONS_H

// Libraries
#include "constants.h"

// ReactPhysics3D namespace
namespace reactphysics3d {

// ---------- Mathematics functions ---------- //

// function to test if two numbers are (almost) equal
// We test if two numbers a and b are such that (a-b) are in [-EPSILON; EPSILON]
inline bool approxEqual(double a, double b) {
    double difference = a - b;
    return (difference < EPSILON && difference > -EPSILON);
}

} // End of ReactPhysics3D namespace



#endif
