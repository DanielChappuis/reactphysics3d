/****************************************************************************
* Copyright (C) 2009      Daniel Chappuis                                  *
****************************************************************************
* This file is part of ReactPhysics3D.                                     *
*                                                                          *
* ReactPhysics3D is free software: you can redistribute it and/or modify   *
* it under the terms of the GNU Lesser General Public License as published *
* by the Free Software Foundation, either version 3 of the License, or     *
* (at your option) any later version.                                      *
*                                                                          *
* ReactPhysics3D is distributed in the hope that it will be useful,        *
* but WITHOUT ANY WARRANTY; without even the implied warranty of           *
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the             *
* GNU Lesser General Public License for more details.                      *
*                                                                          *
* You should have received a copy of the GNU Lesser General Public License *
* along with ReactPhysics3D. If not, see <http://www.gnu.org/licenses/>.   *
****************************************************************************/

// Libraries
#include "AABB.h"
#include <GL/freeglut.h>        // TODO : Remove this in the final version
#include <GL/gl.h>              // TODO : Remove this in the final version
#include <cassert>

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

// Static method that computes an AABB from a set of vertices. The "center" argument corresponds to the center of the AABB
// This method allocates a new AABB object and return a pointer to the new allocated AABB object
AABB* AABB::computeFromVertices(const vector<Vector3D>& vertices, const Vector3D& center) {
    // TODO : Implement this method;
    return 0;
}

