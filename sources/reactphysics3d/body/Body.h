/********************************************************************************
* ReactPhysics3D physics library, http://code.google.com/p/reactphysics3d/      *
* Copyright (c) 2010 Daniel Chappuis                                            *
*********************************************************************************
*                                                                               *
* Permission is hereby granted, free of charge, to any person obtaining a copy  *
* of this software and associated documentation files (the "Software"), to deal *
* in the Software without restriction, including without limitation the rights  *
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell     *
* copies of the Software, and to permit persons to whom the Software is         *
* furnished to do so, subject to the following conditions:                      *
*                                                                               *
* The above copyright notice and this permission notice shall be included in    *
* all copies or substantial portions of the Software.                           *
*                                                                               *
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR    *
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,      *
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE   *
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER        *
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, *
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN     *
* THE SOFTWARE.                                                                 *
********************************************************************************/

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
