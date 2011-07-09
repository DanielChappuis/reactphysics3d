/********************************************************************************
* ReactPhysics3D physics library, http://code.google.com/p/reactphysics3d/      *
* Copyright (c) 2010 Daniel Chappuis                                            *
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
#include "AABB.h"
#include <cassert>
#ifdef VISUAL_DEBUG
   #include <GL/freeglut.h>
   #include <GL/gl.h>        
#endif

using namespace reactphysics3d;
using namespace std;

// Constructor
AABB::AABB() : bodyPointer(0) {
    
}

// Constructor
AABB::AABB(const Transform& transform, const Vector3D& extents) : bodyPointer(0) {
    update(transform, extents);
}

// Destructor
AABB::~AABB() {

}

#ifdef VISUAL_DEBUG
// Draw the OBB (only for testing purpose)
void AABB::draw() const {

    // Draw in red
    glColor3f(1.0, 0.0, 0.0);

    // Draw the OBB
    glBegin(GL_LINES);
        glVertex3f(maxCoordinates.getX(), minCoordinates.getY(), minCoordinates.getZ());
        glVertex3f(maxCoordinates.getX(), maxCoordinates.getY(), minCoordinates.getZ());

        glVertex3f(maxCoordinates.getX(), minCoordinates.getY(), minCoordinates.getZ());
        glVertex3f(maxCoordinates.getX(), minCoordinates.getY(), maxCoordinates.getZ());

        glVertex3f(maxCoordinates.getX(), minCoordinates.getY(), maxCoordinates.getZ());
        glVertex3f(maxCoordinates.getX(), maxCoordinates.getY(), maxCoordinates.getZ());

        glVertex3f(maxCoordinates.getX(), maxCoordinates.getY(), minCoordinates.getZ());
        glVertex3f(maxCoordinates.getX(), maxCoordinates.getY(), maxCoordinates.getZ());

        glVertex3f(minCoordinates.getX(), minCoordinates.getY(), minCoordinates.getZ());
        glVertex3f(minCoordinates.getX(), maxCoordinates.getY(), minCoordinates.getZ());

        glVertex3f(minCoordinates.getX(), minCoordinates.getY(), minCoordinates.getZ());
        glVertex3f(minCoordinates.getX(), minCoordinates.getY(), maxCoordinates.getZ());

        glVertex3f(minCoordinates.getX(), minCoordinates.getY(), maxCoordinates.getZ());
        glVertex3f(minCoordinates.getX(), maxCoordinates.getY(), maxCoordinates.getZ());

        glVertex3f(minCoordinates.getX(), maxCoordinates.getY(), minCoordinates.getZ());
        glVertex3f(minCoordinates.getX(), maxCoordinates.getY(), maxCoordinates.getZ());

        glVertex3f(minCoordinates.getX(), minCoordinates.getY(), minCoordinates.getZ());
        glVertex3f(maxCoordinates.getX(), minCoordinates.getY(), minCoordinates.getZ());

        glVertex3f(minCoordinates.getX(), maxCoordinates.getY(), minCoordinates.getZ());
        glVertex3f(maxCoordinates.getX(), maxCoordinates.getY(), minCoordinates.getZ());

        glVertex3f(minCoordinates.getX(), maxCoordinates.getY(), maxCoordinates.getZ());
        glVertex3f(maxCoordinates.getX(), maxCoordinates.getY(), maxCoordinates.getZ());

        glVertex3f(minCoordinates.getX(), minCoordinates.getY(), maxCoordinates.getZ());
        glVertex3f(maxCoordinates.getX(), minCoordinates.getY(), maxCoordinates.getZ());

    glEnd();
}
#endif

