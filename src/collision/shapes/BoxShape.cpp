/********************************************************************************
* ReactPhysics3D physics library, http://code.google.com/p/reactphysics3d/      *
* Copyright (c) 2010-2013 Daniel Chappuis                                       *
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
#include "BoxShape.h"
#include "../../configuration.h"
#include <vector>
#include <cassert>

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
using namespace std;

// Constructor
BoxShape::BoxShape(const Vector3& extent) : CollisionShape(BOX), mExtent(extent) {

}

// Private copy-constructor
BoxShape::BoxShape(const BoxShape& shape) : CollisionShape(shape), mExtent(shape.mExtent) {

}

// Destructor
BoxShape::~BoxShape() {

}

// Return the local inertia tensor of the collision shape
void BoxShape::computeLocalInertiaTensor(Matrix3x3& tensor, decimal mass) const {
    decimal factor = (decimal(1.0) / decimal(3.0)) * mass;
    decimal xSquare = mExtent.x * mExtent.x;
    decimal ySquare = mExtent.y * mExtent.y;
    decimal zSquare = mExtent.z * mExtent.z;
    tensor.setAllValues(factor * (ySquare + zSquare), 0.0, 0.0,
                        0.0, factor * (xSquare + zSquare), 0.0,
                        0.0, 0.0, factor * (xSquare + ySquare));
}

#ifdef VISUAL_DEBUG
// Draw the Box (only for testing purpose)
void BoxShape::draw() const {
    decimal e1 = mExtent.x;
    decimal e2 = mExtent.y;
    decimal e3 = mExtent.z;

    // Draw in red
    glColor3f(1.0, 0.0, 0.0);

    // Draw the Box
    glBegin(GL_LINES);
        glVertex3f(e1, -e2, -e3);
        glVertex3f(e1, e2, -e3);

        glVertex3f(e1, -e2, -e3);
        glVertex3f(e1, -e2, e3);

        glVertex3f(e1, -e2, e3);
        glVertex3f(e1, e2, e3);

        glVertex3f(e1, e2, e3);
        glVertex3f(e1, e2, -e3);

        glVertex3f(-e1, -e2, -e3);
        glVertex3f(-e1, e2, -e3);

        glVertex3f(-e1, -e2, -e3);
        glVertex3f(-e1, -e2, e3);

        glVertex3f(-e1, -e2, e3);
        glVertex3f(-e1, e2, e3);

        glVertex3f(-e1, e2, e3);
        glVertex3f(-e1, e2, -e3);

        glVertex3f(e1, -e2, -e3);
        glVertex3f(-e1, -e2, -e3);

        glVertex3f(e1, -e2, -e3);
        glVertex3f(-e1, -e2, -e3);

        glVertex3f(e1, -e2, e3);
        glVertex3f(-e1, -e2, e3);

        glVertex3f(e1, e2, e3);
        glVertex3f(-e1, e2, e3);

    glEnd();
}
#endif
