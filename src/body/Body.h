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
#include "../collision/shapes/AABB.h"
#include "../collision/shapes/CollisionShape.h"
#include "../configuration.h"

// Namespace reactphysics3d
namespace reactphysics3d {


/*  -------------------------------------------------------------------
    Class Body :
        This class is an abstract class to represent a body of the physics
        engine.
    -------------------------------------------------------------------
*/
class Body {
    protected :
        CollisionShape* collisionShape; // Collision shape of the body
        decimal mass;                   // Mass of the body
        Transform transform;            // Position and orientation of the body
        Transform oldTransform;         // Last position and orientation of the body
        decimal interpolationFactor;    // Interpolation factor used for the state interpolation
        bool isActive;                  // True if the body is active (not sleeping because of deactivation)
        bool isMotionEnabled;           // True if the body is able to move
        bool isCollisionEnabled;        // True if the body can collide with others bodies
        AABB* aabb;                     // Axis-Aligned Bounding Box for Broad-Phase collision detection
        bodyindex id;                   // ID of the body
        bool hasMoved;                  // True if the body has moved during the last frame

    public :
        Body(const Transform& transform, CollisionShape* collisionShape, decimal mass, bodyindex id);     // Constructor
        virtual ~Body();                                                                            // Destructor

        bodyindex getID() const;                                // Return the id of the body
        bool getHasMoved() const;                               // Return true if the body has moved during the last frame
        void setHasMoved(bool hasMoved);                        // Set the hasMoved variable (true if the body has moved during the last frame)
        CollisionShape* getCollisionShape() const;              // Return the collision shape
        void setCollisionShape(CollisionShape* collisionShape); // Set the collision shape
        decimal getMass() const;                                // Return the mass of the body
        void setMass(decimal mass);                             // Set the mass of the body
        bool getIsActive() const;                               // Return true if the body is active
        void setIsActive(bool isActive);                        // Set the isActive variable
        const Transform& getTransform() const;                  // Return the current position and orientation
        void setTransform(const Transform& transform);          // Set the current position and orientation
        const AABB* getAABB() const;                            // Return the AAABB of the body
        Transform getInterpolatedTransform() const;             // Return the interpolated transform for rendering
        void setInterpolationFactor(decimal factor);            // Set the interpolation factor of the body
        bool getIsMotionEnabled() const;                        // Return if the rigid body can move
        void setIsMotionEnabled(bool isMotionEnabled);          // Set the value to true if the body can move
        bool getIsCollisionEnabled() const;                     // Return true if the body can collide with others bodies
        void setIsCollisionEnabled(bool isCollisionEnabled);    // Set the isCollisionEnabled value
        void updateOldTransform();                              // Update the old transform with the current one
        void updateAABB();                                      // Update the Axis-Aligned Bounding Box coordinates

        // Operators
        bool operator<(const Body& body2) const;                // Smaller than operator
        bool operator>(const Body& body2) const;                // Larger than operator
        bool operator==(const Body& body2) const;               // Equal operator
        bool operator!=(const Body& body2) const;               // Equal operator
};

// Return the id of the body
inline bodyindex Body::getID() const {
    return id;
}

// Return true if the body has moved during the last frame
inline bool Body::getHasMoved() const {
    return hasMoved;
}

// Set the hasMoved variable (true if the body has moved during the last frame)
inline void Body::setHasMoved(bool hasMoved) {
    this->hasMoved = hasMoved;
}

// Return the collision shape
inline CollisionShape *Body::getCollisionShape() const {
    assert(collisionShape);
    return collisionShape;
}

// Set the collision shape
inline void Body::setCollisionShape(CollisionShape *collisionShape) {
    assert(collisionShape);
    this->collisionShape = collisionShape;
}

// Method that return the mass of the body
inline decimal Body::getMass() const {
    return mass;
};

// Method that set the mass of the body
inline void Body::setMass(decimal mass) {
    this->mass = mass;
}

// Return true if the body is active
inline bool Body::getIsActive() const {
    return isActive;
}                  

// Set the isActive variable
inline void Body::setIsActive(bool isActive) {
    this->isActive = isActive;
}                        

// Return the interpolated transform for rendering
inline Transform Body::getInterpolatedTransform() const {
    return Transform::interpolateTransforms(oldTransform, transform, interpolationFactor);
}

// Set the interpolation factor of the body
inline void Body::setInterpolationFactor(decimal factor) {
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

// Return the current position and orientation
inline const Transform& Body::getTransform() const {
    return transform;
}

// Set the current position and orientation
inline void Body::setTransform(const Transform& transform) {

    // Check if the body has moved
    if (this->transform != transform) {
        hasMoved = true;
    }

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
    
    // TODO : An AABB should not be updated every frame but only if the body has moved
    
    // Update the AABB
    aabb->update(transform, collisionShape->getLocalExtents(OBJECT_MARGIN));
}

// Smaller than operator
inline bool Body::operator<(const Body& body2) const {
    return (id < body2.id);
} 

// Larger than operator
inline bool Body::operator>(const Body& body2) const {
    return (id > body2.id);
} 

// Equal operator
inline bool Body::operator==(const Body& body2) const {
    return (id == body2.id);
}
        
// Equal operator
inline bool Body::operator!=(const Body& body2) const {
    return (id != body2.id);
}               

}

 #endif
