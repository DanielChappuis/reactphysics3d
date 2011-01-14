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
#include "Box.h"
#include <GL/freeglut.h>

// Use the ReactPhysics3D namespace
using namespace reactphysics3d;

// Constructor
Box::Box(double size, double mass, Vector3D& position, rp3d::Quaternion& orientation) {
    this->size = size;

    // Local inertia tensor of a cube
    rp3d::Matrix3x3 inertiaTensor(1.0/12.0*mass*2*size*size, 0.0, 0.0,
                                  0.0, 1.0/12.0*mass*2*size*size, 0.0,
                                  0.0, 0.0, 1.0/12.0*mass*2*size*size);

    // Creation of the bounding volume for the collision
    // The bounding volume is an Oriented Bounding Box (OBB)
    // The first three arguments are the three axis direction of the OBB (here the x,y and z axis)
    // and the last three arguments are the corresponding half extents of the OBB in those direction.
    // Here the rigid body is a cube and therefore the three half extents are the half of the size
    // of the cube in all OBB directions.
    rp3d::OBB* boundingVolume = new OBB(position, Vector3D(1.0, 0.0, 0.0), Vector3D(0.0, 1.0, 0.0), Vector3D(0.0, 0.0, 1.0), size/2.0, size/2.0, size/2);

    // Create the rigid body that will be used to simulate the physics of the box
    rigidBody = new RigidBody(position, orientation, mass, inertiaTensor, boundingVolume);
}

// Destructor
Box::~Box() {
    // Delete the physics body
    delete rigidBody;
}

// Draw the box
void Box::draw() const {

    // Get the current position of the rigid body (for animation you should use the
    // getInterpolatedPosition() function instead of getPosition()
    Vector3D position = rigidBody->getInterpolatedPosition();

    // Get the current orientation of the rigid body (represented by a quaternion)
    Quaternion orientation = rigidBody->getInterpolatedOrientation();

    // Use the returned quaternion to get the rotation axis and rotation angle
    Vector3D orientationAxis;
    double orientationAngle;
    orientation.getRotationAngleAxis(orientationAngle, orientationAxis);

    glPushMatrix();

    // Translation of the box to its position
    glTranslatef(position.getValue(0), position.getValue(1), position.getValue(2));

    // Rotation of the box according to its orientation
    glRotatef(orientationAngle/PI*180.0, orientationAxis.getX(), orientationAxis.getY(), orientationAxis.getZ());

    // Draw the cube
    glutSolidCube(size);
    
    glPopMatrix();
}

