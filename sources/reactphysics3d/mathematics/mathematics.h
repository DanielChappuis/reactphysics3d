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

// TODO : Mathematics library : Check everywhere that in member methods we use attributes access instead of getter and setter.

#ifndef MATHEMATICS_H
#define MATHEMATICS_H

// Libraries
#include "Matrix.h"
#include "Matrix3x3.h"
#include "Polygon3D.h"
#include "Quaternion.h"
#include "Segment3D.h"
#include "Vector.h"
#include "Vector3D.h"
#include "constants.h"
#include "exceptions.h"
#include <cstdio>

// ---------- Mathematics functions ---------- //

// Method to test if two numbers are (almost) equal
// We test if two numbers a and b are such that (a-b) are in [-EPSILON; EPSILON]
inline bool equal(double a, double b) {
    double difference = a - b;
    return (difference < EPSILON && difference > -EPSILON);
}

#endif
