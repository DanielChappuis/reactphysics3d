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

// Libraries
#include <complex>
#include "../configuration.h"
#include "ConeCollider.h"

#if defined(VISUAL_DEBUG)
	#if defined(APPLE_OS)
		#include <GLUT/glut.h>
		#include <OpenGL/gl.h>
	#elif defined(WINDOWS_OS)
		#include <GL/glut.h>
		#include <GL/gl.h>
	#elif defined(LINUX_OS)
		#include <GL/freeglut.h>
		#include <GL/gl.h>
	#endif
#endif

using namespace reactphysics3d;

// Constructor
ConeCollider::ConeCollider(decimal radius, decimal height) : radius(radius), halfHeight(height/2.0) {
    assert(radius > 0.0);
    assert(halfHeight > 0.0);
    
    // Compute the sine of the semi-angle at the apex point
    sinTheta = radius / (sqrt(radius * radius + height * height));
}

// Destructor
ConeCollider::~ConeCollider() {

}

// Return a local support point in a given direction
inline Vector3 ConeCollider::getLocalSupportPoint(const Vector3& direction, decimal margin) const {
    assert(margin >= 0.0);

    const Vector3& v = direction;
    decimal sinThetaTimesLengthV = sinTheta * v.length();
    Vector3 supportPoint;

    if (v.getY() >= sinThetaTimesLengthV) {
        supportPoint = Vector3(0.0, halfHeight, 0.0);
    }
    else {
        decimal projectedLength = sqrt(v.getX() * v.getX() + v.getZ() * v.getZ());
        if (projectedLength > MACHINE_EPSILON) {
            decimal d = radius / projectedLength;
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
void ConeCollider::draw() const {

    // Draw in red
    glColor3f(1.0, 0.0, 0.0);

    // Draw the sphere
    glutWireCone(radius, 2.0 * halfHeight, 50, 50);
}
#endif
