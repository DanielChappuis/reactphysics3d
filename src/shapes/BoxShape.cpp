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
#include "BoxShape.h"
#include <vector>
#include <cassert>

#ifdef VISUAL_DEBUG
   #include <GL/freeglut.h>        // TODO : Remove this in the final version
   #include <GL/gl.h>              // TODO : Remove this in the final version
#endif

using namespace reactphysics3d;
using namespace std;

// Constructor
BoxShape::BoxShape(const Vector3& extent) : extent(extent) {

}

// Destructor
BoxShape::~BoxShape() {

}

#ifdef VISUAL_DEBUG
// Draw the Box (only for testing purpose)
void BoxShape::draw() const {
    double e1 = extent.getX();
    double e2 = extent.getY();
    double e3 = extent.getZ();

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
