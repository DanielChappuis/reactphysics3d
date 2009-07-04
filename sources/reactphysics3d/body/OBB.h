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

#ifndef OBB_H
#define OBB_H

// Libraries
#include "BoundingVolume.h"
#include "../mathematics/mathematics.h"

// ReactPhysics3D namespace
namespace reactphysics3d {

/*  -------------------------------------------------------------------
    Class OBB :
        This class represents a bounding volume of type "Oriented Bounding
        Box". It's a box that has a given orientation is space.
    -------------------------------------------------------------------
*/
class OBB : public BoundingVolume {
    private :
        Vector3D center;            // Center point of the OBB
        Vector3D axis[3];           // Array that contains the three axis of the OBB
        double extent[3];           // Array that contains the three extents size of the OBB

    public :
        OBB(const Vector3D& center, const Vector3D& axis1, const Vector3D& axis2,
            const Vector3D& axis3, double extent1, double extent2, double extent3);     // Constructor
        OBB(const OBB& obb);                                                            // Copy-Constructor
        virtual ~OBB();                                                                 // Destructor

        Vector3D getCenter() const;                                                             // Return the center point of the OBB
        void setCenter(const Vector3D& center);                                                 // Set the center point
        Vector3D getAxis(unsigned int index) const throw(std::invalid_argument);                // Return an axis of the OBB
        void setAxis(unsigned int index, const Vector3D& axis) throw(std::invalid_argument);    // Set an axis
        double getExtent(unsigned int index) const throw(std::invalid_argument);                // Return an extent value
        void setExtent(unsigned int index, double extent) throw(std::invalid_argument);         // Set an extent value
};

// TODO : Don't forget that we need to code a way that a change in the orientation of a rigid body imply
//        a change in the orientation (center and axis) of the corresponding OBB.

// Return the center point of the OBB
inline Vector3D OBB::getCenter() const {
    return center;
}

// Set the center point
inline void OBB::setCenter(const Vector3D& center) {
    this->center = center;
}

// Return an axis of the OBB
inline Vector3D OBB::getAxis(unsigned int index) const throw(std::invalid_argument) {
    // Check if the index value is valid
    if (index >= 0 && index <3) {
        return axis[index];
    }
    else {
        // The index value is not valid, we throw an exception
        throw std::invalid_argument("Exception : The index value has to be between 0 and 2");
    }
}

// Set an axis
inline void OBB::setAxis(unsigned int index, const Vector3D& axis) throw(std::invalid_argument) {
    // Check if the index value is valid
    if (index >= 0 && index <3) {
        this->axis[index] = axis;
    }
    else {
        // The index value is not valid, we throw an exception
        throw std::invalid_argument("Exception : The index value has to be between 0 and 2");
    }
}

// Return an extent value
inline double OBB::getExtent(unsigned int index) const throw(std::invalid_argument) {
    // Check if the index value is valid
    if (index >= 0 && index <3) {
        return extent[index];
    }
    else {
        // The index value is not valid, we throw an exception
        throw std::invalid_argument("Exception : The index value has to be between 0 and 2");
    }
}

// Set an extent value
inline void OBB::setExtent(unsigned int index, double extent) throw(std::invalid_argument) {
    // Check if the index value is valid
    if (index >= 0 && index <3) {
        this->extent[index] = extent;
    }
    else {
        // The index value is not valid, we throw an exception
        throw std::invalid_argument("Exception : The index value has to be between 0 and 2");
    }
}


}; // End of the ReactPhysics3D namespace

#endif
