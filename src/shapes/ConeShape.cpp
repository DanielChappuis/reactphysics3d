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
#include <complex>

#include "ConeShape.h"
#ifdef VISUAL_DEBUG
   #include <GL/freeglut.h>        // TODO : Remove this in the final version
   #include <GL/gl.h>              // TODO : Remove this in the final version
#endif

using namespace reactphysics3d;

// Constructor
ConeShape::ConeShape(double radius, double height) : radius(radius), halfHeight(height/2.0) {
    assert(radius > 0.0);
    assert(halfHeight > 0.0);
    
    // Compute the sine of the semi-angle at the apex point
    sinTheta = radius / (sqrt(radius * radius + height * height));
}

// Destructor
ConeShape::~ConeShape() {

}

// Return a local support point in a given direction
inline Vector3 ConeShape::getLocalSupportPoint(const Vector3& direction, double margin) const {
    assert(margin >= 0.0);

    const Vector3& v = direction;
    double sinThetaTimesLengthV = sinTheta * v.length();
    Vector3 supportPoint;

    if (v.getY() >= sinThetaTimesLengthV) {
        supportPoint = Vector3(0.0, halfHeight, 0.0);
    }
    else {
        double projectedLength = sqrt(v.getX() * v.getX() + v.getZ() * v.getZ());
        if (projectedLength > MACHINE_EPSILON) {
            double d = radius / projectedLength;
            supportPoint = Vector3(v.getX() * d, -halfHeight, v.getZ() * d);
        }
        else {
            supportPoint = Vector3(radius, -halfHeight, 0.0);
        }
    }

    // Add the margin to the support point
    if (margin != 0.0) {
        Vector3 unitVec(0.0, -1.0, 0.0);
        if (v.lengthSquare() > MACHINE_EPSILON * MACHINE_EPSILON) {
            unitVec = v.getUnit();
        }
        supportPoint += unitVec * margin;
    }

    return supportPoint;
}

#ifdef VISUAL_DEBUG
// Draw the cone (only for debuging purpose)
void ConeShape::draw() const {

    // Draw in red
    glColor3f(1.0, 0.0, 0.0);

    // Draw the sphere
    glutWireCone(radius, 2.0 * halfHeight, 50, 50);
}
#endif
