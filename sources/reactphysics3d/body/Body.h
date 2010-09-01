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

#ifndef BODY_H
#define BODY_H

// Libraries
#include <stdexcept>
#include <cassert>


// Namespace reactphysics3d
namespace reactphysics3d {

class BroadBoundingVolume;
class NarrowBoundingVolume;

/*  -------------------------------------------------------------------
    Class Body :
        This class is an abstract class to represent body of the physics
        engine.
    -------------------------------------------------------------------
*/
class Body {
    protected :
        double mass;                                    // Mass of the body
        BroadBoundingVolume* broadBoundingVolume;       // Bounding volume used for the broad-phase collision detection
        NarrowBoundingVolume* narrowBoundingVolume;     // Bounding volume used for the narrow-phase collision detection
        bool isMotionEnabled;                           // True if the body is able to move
        bool isCollisionEnabled;                        // True if the body can collide with others bodies

        void setBroadBoundingVolume(BroadBoundingVolume* broadBoundingVolume);      // Set the broad-phase bounding volume
        void setNarrowBoundingVolume(NarrowBoundingVolume* narrowBoundingVolume);   // Set the narrow-phase bounding volume

    public :
        Body(double mass) throw(std::invalid_argument);    // Constructor
        virtual ~Body();                                   // Destructor

        double getMass() const;                                         // Return the mass of the body
        void setMass(double mass);                                      // Set the mass of the body
        bool getIsMotionEnabled() const;                                // Return if the rigid body can move
        void setIsMotionEnabled(bool isMotionEnabled);                  // Set the value to true if the body can move
        bool getIsCollisionEnabled() const;                             // Return true if the body can collide with others bodies
        void setIsCollisionEnabled(bool isCollisionEnabled);            // Set the isCollisionEnabled value
        const BroadBoundingVolume* getBroadBoundingVolume() const;      // Return the broad-phase bounding volume
        const NarrowBoundingVolume* getNarrowBoundingVolume() const;    // Return the narrow-phase bounding volume of the body
};

// Method that return the mass of the body
inline double Body::getMass() const {
    return mass;
};

// Return if the rigid body can move
inline bool Body::getIsMotionEnabled() const {
    return isMotionEnabled;
}

// Set the value to true if the body can move
inline void Body::setIsMotionEnabled(bool isMotionEnabled) {
    this->isMotionEnabled = isMotionEnabled;
}

// Method that set the mass of the body
inline void Body::setMass(double mass) {
    this->mass = mass;
}

 // Return true if the body can collide with others bodies
inline bool Body::getIsCollisionEnabled() const {
    return isCollisionEnabled;
}

// Set the isCollisionEnabled value
inline void Body::setIsCollisionEnabled(bool isCollisionEnabled) {
    this->isCollisionEnabled = isCollisionEnabled;
}

// Return the broad-phase bounding volume
inline const BroadBoundingVolume* Body::getBroadBoundingVolume() const {
    return broadBoundingVolume;
}

// Return the oriented bounding box of the rigid body
inline const NarrowBoundingVolume* Body::getNarrowBoundingVolume() const {
    return narrowBoundingVolume;
}

}

 #endif
