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
#include "AABB.h"
#include "../../configuration.h"
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
AABB::AABB() : mBodyPointer(0) {
    
}

// Constructor
AABB::AABB(const Vector3& minCoordinates, const Vector3& maxCoordinates, Body* modyPointer)
     :mMinCoordinates(minCoordinates), mMaxCoordinates(maxCoordinates), mBodyPointer(modyPointer) {

}

// Constructor
AABB::AABB(const Transform& transform, const Vector3& extents) : mBodyPointer(0) {
    update(transform, extents);
}

// Destructor
AABB::~AABB() {

}

#ifdef VISUAL_DEBUG
// Draw the AABB (only for testing purpose)
void AABB::draw() const {

    // Draw in red
    glColor3f(1.0, 0.0, 0.0);

    // Draw the AABB
    glBegin(GL_LINES);
        glVertex3f(mMaxCoordinates.x, mMinCoordinates.y, mMinCoordinates.z);
        glVertex3f(mMaxCoordinates.x, mMaxCoordinates.y, mMinCoordinates.z);

        glVertex3f(mMaxCoordinates.x, mMinCoordinates.y, mMinCoordinates.z);
        glVertex3f(mMaxCoordinates.x, mMinCoordinates.y, mMaxCoordinates.z);

        glVertex3f(mMaxCoordinates.x, mMinCoordinates.y, mMaxCoordinates.z);
        glVertex3f(mMaxCoordinates.x, mMaxCoordinates.y, mMaxCoordinates.z);

        glVertex3f(mMaxCoordinates.x, mMaxCoordinates.y, mMinCoordinates.z);
        glVertex3f(mMaxCoordinates.x, mMaxCoordinates.y, mMaxCoordinates.z);

        glVertex3f(mMinCoordinates.x, mMinCoordinates.y, mMinCoordinates.z);
        glVertex3f(mMinCoordinates.x, mMaxCoordinates.y, mMinCoordinates.z);

        glVertex3f(mMinCoordinates.x, mMinCoordinates.y, mMinCoordinates.z);
        glVertex3f(mMinCoordinates.x, mMinCoordinates.y, mMaxCoordinates.z);

        glVertex3f(mMinCoordinates.x, mMinCoordinates.y, mMaxCoordinates.z);
        glVertex3f(mMinCoordinates.x, mMaxCoordinates.y, mMaxCoordinates.z);

        glVertex3f(mMinCoordinates.x, mMaxCoordinates.y, mMinCoordinates.z);
        glVertex3f(mMinCoordinates.x, mMaxCoordinates.y, mMaxCoordinates.z);

        glVertex3f(mMinCoordinates.x, mMinCoordinates.y, mMinCoordinates.z);
        glVertex3f(mMaxCoordinates.x, mMinCoordinates.y, mMinCoordinates.z);

        glVertex3f(mMinCoordinates.x, mMaxCoordinates.y, mMinCoordinates.z);
        glVertex3f(mMaxCoordinates.x, mMaxCoordinates.y, mMinCoordinates.z);

        glVertex3f(mMinCoordinates.x, mMaxCoordinates.y, mMaxCoordinates.z);
        glVertex3f(mMaxCoordinates.x, mMaxCoordinates.y, mMaxCoordinates.z);

        glVertex3f(mMinCoordinates.x, mMinCoordinates.y, mMaxCoordinates.z);
        glVertex3f(mMaxCoordinates.x, mMinCoordinates.y, mMaxCoordinates.z);

    glEnd();
}
#endif

