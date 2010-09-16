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
   #include <GL/freeglut.h>        // TODO : Remove this in the final version
   #include <GL/gl.h>              // TODO : Remove this in the final version
#endif

using namespace reactphysics3d;
using namespace std;

// Constructor
AABB::AABB(const Vector3D& center,double extentX, double extentY, double extentZ) {
    this->center = center;

    this->extent[0] = extentX;
    this->extent[1] = extentY;
    this->extent[2] = extentZ;

    this->originalAABBExtent[0] = extentX;
    this->originalAABBExtent[1] = extentY;
    this->originalAABBExtent[2] = extentZ;
}

// Destructor
AABB::~AABB() {

}

#ifdef VISUAL_DEBUG
// Draw the OBB (only for testing purpose)
void AABB::draw() const {

    Vector3D s1 = center + Vector3D(extent[0], extent[1], -extent[2]);
    Vector3D s2 = center + Vector3D(extent[0], extent[1], extent[2]);
    Vector3D s3 = center + Vector3D(-extent[0], extent[1], extent[2]);
    Vector3D s4 = center + Vector3D(-extent[0], extent[1], -extent[2]);
    Vector3D s5 = center + Vector3D(extent[0], -extent[1], -extent[2]);
    Vector3D s6 = center + Vector3D(extent[0], -extent[1], extent[2]);
    Vector3D s7 = center + Vector3D(-extent[0], -extent[1], extent[2]);
    Vector3D s8 = center + Vector3D(-extent[0], -extent[1], -extent[2]);

    // Draw in red
    glColor3f(1.0, 0.0, 0.0);

    // Draw the OBB
    glBegin(GL_LINES);
        glVertex3f(s1.getX(), s1.getY(), s1.getZ());
        glVertex3f(s2.getX(), s2.getY(), s2.getZ());

        glVertex3f(s2.getX(), s2.getY(), s2.getZ());
        glVertex3f(s3.getX(), s3.getY(), s3.getZ());

        glVertex3f(s3.getX(), s3.getY(), s3.getZ());
        glVertex3f(s4.getX(), s4.getY(), s4.getZ());

        glVertex3f(s4.getX(), s4.getY(), s4.getZ());
        glVertex3f(s1.getX(), s1.getY(), s1.getZ());

        glVertex3f(s5.getX(), s5.getY(), s5.getZ());
        glVertex3f(s6.getX(), s6.getY(), s6.getZ());

        glVertex3f(s6.getX(), s6.getY(), s6.getZ());
        glVertex3f(s7.getX(), s7.getY(), s7.getZ());

        glVertex3f(s7.getX(), s7.getY(), s7.getZ());
        glVertex3f(s8.getX(), s8.getY(), s8.getZ());

        glVertex3f(s8.getX(), s8.getY(), s8.getZ());
        glVertex3f(s5.getX(), s5.getY(), s5.getZ());

        glVertex3f(s1.getX(), s1.getY(), s1.getZ());
        glVertex3f(s5.getX(), s5.getY(), s5.getZ());

        glVertex3f(s4.getX(), s4.getY(), s4.getZ());
        glVertex3f(s8.getX(), s8.getY(), s8.getZ());

        glVertex3f(s3.getX(), s3.getY(), s3.getZ());
        glVertex3f(s7.getX(), s7.getY(), s7.getZ());

        glVertex3f(s2.getX(), s2.getY(), s2.getZ());
        glVertex3f(s6.getX(), s6.getY(), s6.getZ());

    glEnd();
}
#endif

// Static method that computes an AABB from a set of vertices. The "center" argument corresponds to the center of the AABB
// This method allocates a new AABB object and return a pointer to the new allocated AABB object
AABB* AABB::computeFromVertices(const vector<Vector3D>& vertices, const Vector3D& center) {
    // TODO : Implement this method;
    return NULL;
}

