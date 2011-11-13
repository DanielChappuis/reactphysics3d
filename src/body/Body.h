/********************************************************************************
* ReactPhysics3D physics library, http://code.google.com/p/reactphysics3d/      *
* Copyright (c) 2010-2012 Daniel Chappuis                                       *
*********************************************************************************
*                                                                               *
* This software is provided 'as-is', without any express or implied warranty.   *
* In no event will the authors be held liable for any damages arising from the  *
* use of this software.                                                         *
*                                                                               *
* Permission is granted to anyone to use this software for any purpose,         *
* including commercial applications, and to alter it and redistribute it        *
* freely, subject to the following restrictions:                                *
*                                                                               *
* 1. The origin of this software must not be misrepresented; you must not claim *
*    that you wrote the original software. If you use this software in a        *
*    product, an acknowledgment in the product documentation would be           *
*    appreciated but is not required.                                           *
*                                                                               *
* 2. Altered source versions must be plainly marked as such, and must not be    *
*    misrepresented as being the original software.                             *
*                                                                               *
* 3. This notice may not be removed or altered from any source distribution.    *
*                                                                               *
********************************************************************************/

#ifndef BODY_H
#define BODY_H

// Libraries
#include <stdexcept>
#include <cassert>
#include "../mathematics/Transform.h"
#include "../shapes/AABB.h"
#include "../shapes/Shape.h"
#include "../constants.h"

// Namespace reactphysics3d
namespace reactphysics3d {


/*  -------------------------------------------------------------------
    Class Body :
        This class is an abstract class to represent body of the physics
        engine.
    -------------------------------------------------------------------
*/
class Body {
    protected :
        Shape* shape;                   // Collision shape of the body
        double mass;                    // Mass of the body
        Transform transform;            // Position and orientation of the body
        Transform oldTransform;         // Last position and orientation of the body
        double interpolationFactor;     // Interpolation factor used for the state interpolation
        bool isMotionEnabled;           // True if the body is able to move
        bool isCollisionEnabled;        // True if the body can collide with others bodies
        AABB* aabb;                     // Axis-Aligned Bounding Box for Broad-Phase collision detection
        luint id;                       // ID of the body

    public :
        Body(const Transform& transform, Shape* shape, double mass, long unsigned int id);      // Constructor
        virtual ~Body();                                                                        // Destructor

        luint getID() const;                                            // Return the id of the body
        Shape* getShape() const;                                        // Return the collision shape
        void setShape(Shape* shape);                                    // Set the collision shape
        double getMass() const;                                         // Return the mass of the body
        void setMass(double mass);                                      // Set the mass of the body
        const Transform& getTransform() const;                          // Return the current position and orientation
        void setTransform(const Transform& transform);                  // Set the current position and orientation
        const AABB* getAABB() const;                                    // Return the AAABB of the body
        Transform getInterpolatedTransform() const;                     // Return the interpolated transform for rendering
        void setInterpolationFactor(double factor);                     // Set the interpolation factor of the body
        bool getIsMotionEnabled() const;                                // Return if the rigid body can move
        void setIsMotionEnabled(bool isMotionEnabled);                  // Set the value to true if the body can move
        bool getIsCollisionEnabled() const;                             // Return true if the body can collide with others bodies
        void setIsCollisionEnabled(bool isCollisionEnabled);            // Set the isCollisionEnabled value
        void updateOldTransform();                                      // Update the old transform with the current one
        void updateAABB();                                              // Update the Axis-Aligned Bounding Box coordinates
};

// Return the id of the body
inline luint Body::getID() const {
    return id;
}                               

// Return the collision shape
inline Shape* Body::getShape() const {
    assert(shape);
    return shape;
}

// Set the collision shape
inline void Body::setShape(Shape* shape) {
    assert(shape);
    this->shape = shape;
}

// Method that return the mass of the body
inline double Body::getMass() const {
    return mass;
};

// Return the interpolated transform for rendering
inline Transform Body::getInterpolatedTransform() const {
    return Transform::interpolateTransforms(oldTransform, transform, interpolationFactor);
}

// Set the interpolation factor of the body
inline void Body::setInterpolationFactor(double factor) {
    // Set the factor
    interpolationFactor = factor;
}

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

// Return the current position and orientation
inline const Transform& Body::getTransform() const {
    return transform;
}

// Set the current position and orientation
inline void Body::setTransform(const Transform& transform) {
    this->transform = transform;
}

// Return the AAABB of the body
inline const AABB* Body::getAABB() const {
    return aabb;
}

 // Return true if the body can collide with others bodies
inline bool Body::getIsCollisionEnabled() const {
    return isCollisionEnabled;
}

// Set the isCollisionEnabled value
inline void Body::setIsCollisionEnabled(bool isCollisionEnabled) {
    this->isCollisionEnabled = isCollisionEnabled;
}

// Update the old transform with the current one
// This is used to compute the interpolated position and orientation of the body
inline void Body::updateOldTransform() {
    oldTransform = transform;
}

// Update the rigid body in order to reflect a change in the body state
inline void Body::updateAABB() {
    // Update the AABB
    aabb->update(transform, shape->getLocalExtents(OBJECT_MARGIN));
}


}

 #endif
