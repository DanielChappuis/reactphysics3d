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

#ifndef CYLINDER_SHAPE_H
#define CYLINDER_SHAPE_H

// Libraries
#include "Shape.h"
#include "../mathematics/mathematics.h"

// TODO : CHECK THAT THE AABB IS CORRECT
// TODO : TEST THIS SHAPE WITH GJK AND EPA ALGORITHMS

// ReactPhysics3D namespace
namespace reactphysics3d {

/*  -------------------------------------------------------------------
    Class CylinderShape :
        This class represents a cylinder collision shape around the Y axis
        and centered at the origin. The cylinder is defined by its height
        and the radius of its base. The "transform" of the corresponding
        rigid body gives an orientation and a position to the cylinder.
    -------------------------------------------------------------------
*/
class CylinderShape : public Shape {
    private :
        double radius;              // Radius of the base
        double halfHeight;          // Half height of the cone

    public :
        CylinderShape(double radius, double height);        // Constructor
        virtual ~CylinderShape();                           // Destructor

        double getRadius() const;                                                                   // Return the radius
        void setRadius(double radius);                                                              // Set the radius
        double getHeight() const;                                                                   // Return the height
        void setHeight(double height);                                                              // Set the height
        virtual Vector3 getLocalSupportPoint(const Vector3& direction, double margin=0.0) const;  // Return a support point in a given direction
        virtual Vector3 getLocalExtents(double margin=0.0) const;                                  // Return the local extents in x,y and z direction


#ifdef VISUAL_DEBUG
        virtual void draw() const;                              // Draw the sphere (only for testing purpose)
#endif
};

// Return the radius
inline double CylinderShape::getRadius() const {
    return radius;
}

// Set the radius
inline void CylinderShape::setRadius(double radius) {
    this->radius = radius;
}

// Return the height
inline double CylinderShape::getHeight() const {
    return halfHeight * 2.0;
}

// Set the height
inline void CylinderShape::setHeight(double height) {
    this->halfHeight = height / 2.0;
}

// Return the local extents in x,y and z direction
inline Vector3 CylinderShape::getLocalExtents(double margin) const {
    return Vector3(radius + margin, halfHeight + margin, radius + margin);
}

}; // End of the ReactPhysics3D namespace

#endif

