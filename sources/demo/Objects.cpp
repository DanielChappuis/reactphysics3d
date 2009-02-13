/****************************************************************************
 * Copyright (C) 2009      Daniel Chappuis                                  *
 ****************************************************************************
 * This file is part of ReactPhysics3D.                                     *
 *                                                                          *
 * ReactPhysics3D is free software: you can redistribute it and/or modify   *
 * it under the terms of the GNU General Public License as published by     *
 * the Free Software Foundation, either version 3 of the License, or        *
 * (at your option) any later version.                                      *
 *                                                                          *
 * ReactPhysics3D is distributed in the hope that it will be useful,        *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of           *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the             *
 * GNU General Public License for more details.                             *
 *                                                                          *
 * You should have received a copy of the GNU General Public License        *
 * along with ReactPhysics3D. If not, see <http://www.gnu.org/licenses/>.   *
 ***************************************************************************/

// Libraries
#include "Objects.h"

//#include <windows.h>            // To avoid an error due to the #include <GL/glut.h>
#include <GL/freeglut.h>
#include <GL/gl.h>
#include <GL/glu.h>
#include <cmath>
#include <iostream>

// ----- Class Object ----- //

// Constructor of the class Object
Object::Object(const Vector3D& position, const Kilogram& mass, const Matrix3x3& inertiaTensor)
        :rigidBody(new RigidBody(position, mass, inertiaTensor)) {

}

// Destructor of the class Object
Object::~Object() {
    // Delete the rigid body object
    delete rigidBody;
}

// Return the pointer to the rigid body
RigidBody* Object::getRigidBody() {
    return rigidBody;
}

// ----- Class Cube ----- //

// Static attributes
const Matrix3x3 Cube::inertiaTensor;

// Constructor of the class Cube
Cube::Cube(const Vector3D& position, float size, const Kilogram& mass)
     :Object(position, mass, Matrix3x3(1.0/12.0*mass.getValue()*2*size*size, 0.0, 0.0,
                                        0.0, 1.0/12.0*mass.getValue()*2*size*size, 0.0,
                                        0.0, 0.0, 1.0/12.0*mass.getValue()*2*size*size)) {
    this->size = size;
}

// Destructor of the classe Cube
Cube::~Cube() {

}

// Method to draw the cube
void Cube::draw() const {

    // Get the interpolated state of the rigid body
    BodyState state = rigidBody->getInterpolatedState();

    // Position of the cube
    double x = state.getPosition().getX();
    double y = state.getPosition().getY();
    double z = state.getPosition().getZ();

    // Orientation of the cube
    Vector3D orientationAxis;
    double orientationAngle;
    state.getOrientation().getRotationAngleAxis(orientationAngle, orientationAxis);

    // Translation of the cube to its position
    glTranslatef(x, y, z);

    // Rotation of the cube according to its orientation
    glRotatef(orientationAngle/pi*180.0, orientationAxis.getX(), orientationAxis.getY(), orientationAxis.getZ());

    // Draw the cube
    glutSolidCube(size);
}


// ----- Class Plane ----- //

// Constructor of the class Plane
Plane::Plane(const Vector3D& position, float width, float height, const Vector3D& d1, const Vector3D& d2, const Kilogram& mass)
      :Object(position, mass, Matrix3x3(1.0/12.0*mass.getValue()*height*height, 0.0, 0.0,
                                        0.0, 1.0/12.0*mass.getValue()*(width*width+height*height), 0.0,
                                        0.0, 0.0, 1.0/12.0*mass.getValue()*width*width)) {
    this->width = width;
    this->height = height;
    this->d1 = d1;
    this->d2 = d2;

    // By default Planes in the demo cannot move
    rigidBody->setIsMotionEnabled(false);

    // Compute the unit normal vector of the plane by a cross product
    normalVector = d1.crossProduct(d2).getUnit();
}

// Destructor of the class Plane
Plane::~Plane() {

}

// Method used to draw the plane
void Plane::draw() const {

       // Get the interpolated state of the rigid body
       BodyState state = rigidBody->getInterpolatedState();

       // Get the position of the rigid body
       double x = state.getPosition().getX();
       double y = state.getPosition().getY();
       double z = state.getPosition().getZ();

       // Translation of the cube to its position
       glTranslatef(x, y, z);

       float halfWidth = width / 2.0;
       float halfHeight = height / 2.0;

       // Draw the plane
       glBegin(GL_POLYGON);
            glColor3f(1.0, 1.0, 1.0);
            glVertex3f(x + d1.getX() * halfWidth + d2.getX() * halfHeight , y + d1.getY() * halfWidth +  d2.getY() * halfHeight
                        , z + d1.getZ() * halfWidth + d2.getZ() * halfHeight);
            glNormal3f(normalVector.getX(), normalVector.getY(), normalVector.getZ());
            glVertex3f(x + d1.getX() * halfWidth - d2.getX() * halfHeight , y + d1.getY() * halfWidth -  d2.getY() * halfHeight
                        , z + d1.getZ() * halfWidth - d2.getZ() * halfHeight);
            glNormal3f(normalVector.getX(), normalVector.getY(), normalVector.getZ());
            glVertex3f(x - d1.getX() * halfWidth - d2.getX() * halfHeight , y - d1.getY() * halfWidth -  d2.getY() * halfHeight
                        , z - d1.getZ() * halfWidth - d2.getZ() * halfHeight);
            glNormal3f(normalVector.getX(), normalVector.getY(), normalVector.getZ());
            glVertex3f(x - d1.getX() * halfWidth + d2.getX() * halfHeight , y - d1.getY() * halfWidth +  d2.getY() * halfHeight
                        , z - d1.getZ() * halfWidth + d2.getZ() * halfHeight);
       glEnd();
}



