/********************************************************************************
* ReactPhysics3D physics library, http://code.google.com/p/reactphysics3d/      *
* Copyright (c) 2011 Daniel Chappuis                                            *
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

#ifndef BOUNDING_SPHERE_H
#define BOUNDING_SPHERE_H

// Libraries
#include "Shape.h"
#include "../mathematics/mathematics.h"

// ReactPhysics3D namespace
namespace reactphysics3d {

/*  -------------------------------------------------------------------
    Class BoundingSphere :
        This class represents a sphere bounding volume.
    -------------------------------------------------------------------
*/
class BoundingSphere : public Shape {
    protected :
        Vector3D center;            // Center point of the sphere
        double radius;              // Radius of the sphere

    public :
        BoundingSphere(const Vector3D& center, double radius);      // Constructor
        virtual ~BoundingSphere();                                  // Destructor

        Vector3D getCenter() const;                                                             // Return the center point of the sphere
        void setCenter(const Vector3D& center);                                                 // Set the center point of the sphere
        double getRadius() const;                                                               // Return the radius of the sphere
        void setRadius(double radius);                                                          // Set the radius of the sphere
        virtual void update(const Vector3D& newCenter,
                            const Quaternion& rotationQuaternion);                              // Update the sphere orientation according to a new orientation of the rigid body
        // TODO : DELETE virtual AABB* computeAABB() const;                                                      // Return the corresponding AABB
        virtual Vector3D getSupportPoint(const Vector3D& direction, double margin=0.0) const;   // Return a support point in a given direction
        virtual Vector3D getLocalExtents() const;                                               // Return the local extents in x,y and z direction

#ifdef VISUAL_DEBUG
            virtual void draw() const;                              // Draw the sphere (only for testing purpose)
#endif
};

// Return the center point of the sphere
inline Vector3D BoundingSphere::getCenter() const {
    return center;
}

// Set the center point of the sphere
inline void BoundingSphere::setCenter(const Vector3D& center) {
    this->center = center;
}

// Get the radius of the sphere
inline double BoundingSphere::getRadius() const {
    return radius;
}

// Set the radius of the sphere
inline void BoundingSphere::setRadius(double radius) {
    this->radius = radius;
}

// Update the orientation of the shere according to the orientation of the rigid body
inline void BoundingSphere::update(const Vector3D& newCenter, const Quaternion& rotationQuaternion) {
    // Update the center of the sphere
    center = newCenter;
}

// Return a support point in a given direction
inline Vector3D BoundingSphere::getSupportPoint(const Vector3D& direction, double margin) const {
    assert(margin >= 0.0);
    double length = direction.length();

    // If the direction vector is not the zero vector
    if (length > 0.0) {
        // Return the support point of the sphere in the given direction
        return (radius + margin) * direction.getUnit();
    }

    // If the direction vector is the zero vector we return a point on the
    // boundary of the sphere
    return Vector3D(0, radius + margin, 0);
}

// Return the local extents of the shape (half-width) in x,y and z local direction
// This method is used to compute the AABB of the box
inline Vector3D BoundingSphere::getLocalExtents() const {
    return Vector3D(radius, radius, radius);
}

}; // End of the ReactPhysics3D namespace

#endif