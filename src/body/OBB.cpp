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
#include "OBB.h"
#include <vector>
#include <cassert>

#ifdef VISUAL_DEBUG
   #include <GL/freeglut.h>        // TODO : Remove this in the final version
   #include <GL/gl.h>              // TODO : Remove this in the final version
#endif

using namespace reactphysics3d;
using namespace std;

// Constructor
OBB::OBB(const Vector3D& center, const Vector3D& axis1, const Vector3D& axis2,
         const Vector3D& axis3, double extent1, double extent2, double extent3) {
    this->center = center;

    oldAxis[0] = axis1.getUnit();
    oldAxis[1] = axis2.getUnit();
    oldAxis[2] = axis3.getUnit();

    this->axis[0] = oldAxis[0];
    this->axis[1] = oldAxis[1];
    this->axis[2] = oldAxis[2];

    this->extent[0] = extent1;
    this->extent[1] = extent2;
    this->extent[2] = extent3;
}

// Destructor
OBB::~OBB() {

}

#ifdef VISUAL_DEBUG
// Draw the OBB (only for testing purpose)
void OBB::draw() const {
    double e0 = extent[0];
    double e1 = extent[1];
    double e2 = extent[2];

    Vector3D s1 = center + (axis[0]*e0) + (axis[1]*e1) - (axis[2]*e2);
    Vector3D s2 = center + (axis[0]*e0) + (axis[1]*e1) + (axis[2]*e2);
    Vector3D s3 = center - (axis[0]*e0) + (axis[1]*e1) + (axis[2]*e2);
    Vector3D s4 = center - (axis[0]*e0) + (axis[1]*e1) - (axis[2]*e2);
    Vector3D s5 = center + (axis[0]*e0) - (axis[1]*e1) - (axis[2]*e2);
    Vector3D s6 = center + (axis[0]*e0) - (axis[1]*e1) + (axis[2]*e2);
    Vector3D s7 = center - (axis[0]*e0) - (axis[1]*e1) + (axis[2]*e2);
    Vector3D s8 = center - (axis[0]*e0) - (axis[1]*e1) - (axis[2]*e2);

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

        glVertex3f(center.getX(), center.getY(), center.getZ());
        glVertex3f(center.getX() + 8.0 * axis[1].getX(), center.getY() + 8.0 * axis[1].getY(), center.getZ() + 8.0 * axis[1].getZ());

    glEnd();
}
#endif

// Return all the vertices that are projected at the extreme of the projection of the bouding volume on the axis.
// If the extreme vertices are part of a face of the OBB, the returned vertices will be ordered according to the face.
vector<Vector3D> OBB::getExtremeVertices(const Vector3D& directionAxis) const {
    assert(directionAxis.length() != 0);

    vector<Vector3D> extremeVertices;

    // Check if the given axis is parallel to an axis on the OBB
    if (axis[0].isParallelWith(directionAxis)) {
        if (axis[0].dot(directionAxis) >= 0) {    // If both axis are in the same direction
            extremeVertices = getFace(0);           // The extreme is the face 0
        }
        else {
            extremeVertices = getFace(1);           // The extreme is the face 1
        }
    }
    else if(axis[1].isParallelWith(directionAxis)) {
        if (axis[1].dot(directionAxis) >= 0) {    // If both axis are in the same direction
           extremeVertices = getFace(2);            // The extreme is the face 2
        }
        else {
            extremeVertices = getFace(3);           // The extreme is the face 3
        }

    }
    else if(axis[2].isParallelWith(directionAxis)) {
        if (axis[2].dot(directionAxis) >= 0) {     // If both axis are in the same direction
          extremeVertices = getFace(4);             // The extreme is the face 4
        }
        else {
            extremeVertices = getFace(5);           // The extreme is the face 5
        }
    }
    else {  // The extreme is made of an unique vertex or an edge
        double maxProjectionLength = 0.0;           // Longest projection length of a vertex onto the projection axis

        // For each vertex of the OBB
        for (unsigned int i=0; i<8; ++i) {
            Vector3D vertex = getVertex(i);

            // Compute the projection length of the current vertex onto the projection axis
            double projectionLength = directionAxis.dot(vertex-center) / directionAxis.length();

            // If we found a bigger projection length
            if (projectionLength > maxProjectionLength + EPSILON) {
                maxProjectionLength = projectionLength;
                extremeVertices.clear();
                extremeVertices.push_back(vertex);
            }
            else if (approxEqual(projectionLength, maxProjectionLength)) {
                extremeVertices.push_back(vertex);
            }
        }
        
        assert(extremeVertices.size() == 1 || extremeVertices.size() == 2);
    }

    // An extreme should be a unique vertex, an edge or a face
    assert(extremeVertices.size() == 1 || extremeVertices.size() == 2 || extremeVertices.size() == 4);

    // Return the extreme vertices
    return extremeVertices;
}

// Static method that computes an OBB from a set of vertices. The "center" argument corresponds to the center of the OBB
// This method allocates a new OBB object and return a pointer to the new allocated OBB object
OBB* OBB::computeFromVertices(const vector<Vector3D>& vertices, const Vector3D& center) {
    // TODO : Implement this method;
    return NULL;
}

// Return the corresponding AABB
AABB* OBB::computeAABB() const {
    double maxLength[] = {0.0, 0.0, 0.0};   // Maximum length for each of the three x,y and z axis
    Vector3D vertex;
    double length;
    
    // For each vertex of the OBB
    for (int i = 0; i<8; i++) {
        vertex = getVertex(i) - center;
        for (int j=0; j<3; j++) {
            length = std::abs(vertex.getValue(j));
            if (length > maxLength[j]) {
                maxLength[j] = length;
            }
        }
    }

    // Create and return the AABB
    return new AABB(center, maxLength[0], maxLength[1], maxLength[2]);
}
