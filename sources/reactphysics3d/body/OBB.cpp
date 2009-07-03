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
#include "OBB.h"

// We want to use the ReactPhysics3D namespace
using namespace reactphysics3d;

// Constructor
OBB::OBB(const Vector3D& center, const Vector3D& axis1, const Vector3D& axis2,
            const Vector3D& axis3, double extent1, double extent2, double extent3) {
    this->center = center;

    this->axis[0] = axis1;
    this->axis[1] = axis2;
    this->axis[2] = axis3;

    this->extent[0] = extent1;
    this->extent[1] = extent2;
    this->extent[2] = extent3;
}

// Copy-Constructor
OBB::OBB(const OBB& obb) {
    this->center = obb.center;

    this->axis[0] = obb.axis[0];
    this->axis[1] = obb.axis[1];
    this->axis[2] = obb.axis[2];

    this->extent[0] = obb.extent[0];
    this->extent[1] = obb.extent[1];
    this->extent[2] = obb.extent[2];
}

// Destructor
OBB::~OBB() {

}
