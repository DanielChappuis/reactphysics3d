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

// Libraries
#include "CylinderShape.h"
#ifdef VISUAL_DEBUG
   #include <GLUT/glut.h>        // TODO : Remove this in the final version
   #include <OpenGL/gl.h>              // TODO : Remove this in the final version
#endif

using namespace reactphysics3d;

// Constructor
CylinderShape::CylinderShape(double radius, double height) : radius(radius), halfHeight(height/2.0) {

}

// Destructor
CylinderShape::~CylinderShape() {

}

// Return a local support point in a given direction
Vector3 CylinderShape::getLocalSupportPoint(const Vector3& direction, double margin) const {
    assert(margin >= 0.0);

    Vector3 supportPoint(0.0, 0.0, 0.0);
    double uDotv = direction.getY();
    Vector3 w(direction.getX(), 0.0, direction.getZ());
    double lengthW = sqrt(direction.getX() * direction.getX() + direction.getZ() * direction.getZ());

    if (lengthW != 0.0) {
        if (uDotv < 0.0) supportPoint.setY(-halfHeight);
        else supportPoint.setY(halfHeight);
        supportPoint += (radius / lengthW) * w;
    }
    else {
         if (uDotv < 0.0) supportPoint.setY(-halfHeight);
         else supportPoint.setY(halfHeight);
    }

    // Add the margin to the support point
    if (margin != 0.0) {
        Vector3 unitVec(0.0, 1.0, 0.0);
        if (direction.lengthSquare() > MACHINE_EPSILON * MACHINE_EPSILON) {
            unitVec = direction.getUnit();
        }
        supportPoint += unitVec * margin;
    }

    return supportPoint;
}

#ifdef VISUAL_DEBUG
// Draw the cone (only for debuging purpose)
void CylinderShape::draw() const {

    // Draw in red
    glColor3f(1.0, 0.0, 0.0);

    // Draw the sphere
    glutWireSphere(radius, 50, 50);
}
#endif
