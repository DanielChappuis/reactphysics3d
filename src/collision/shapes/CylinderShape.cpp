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
#include "CylinderShape.h"
#include "../../configuration.h"

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
CylinderShape::CylinderShape(decimal radius, decimal height)
                 : CollisionShape(CYLINDER), mRadius(radius), mHalfHeight(height/2.0) {

}

// Destructor
CylinderShape::~CylinderShape() {

}

// Return a local support point in a given direction
Vector3 CylinderShape::getLocalSupportPoint(const Vector3& direction, decimal margin) const {
    assert(margin >= 0.0);

    Vector3 supportPoint(0.0, 0.0, 0.0);
    decimal uDotv = direction.y;
    Vector3 w(direction.x, 0.0, direction.z);
    decimal lengthW = sqrt(direction.x * direction.x + direction.z * direction.z);

    if (lengthW != 0.0) {
        if (uDotv < 0.0) supportPoint.y = -mHalfHeight;
        else supportPoint.y = mHalfHeight;
        supportPoint += (mRadius / lengthW) * w;
    }
    else {
         if (uDotv < 0.0) supportPoint.y = -mHalfHeight;
         else supportPoint.y = mHalfHeight;
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
    glutWireSphere(mRadius, 50, 50);
}
#endif
