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
#include <math.h>

// ----- Class Object ----- //

// Constructor of the class Object
Object::Object(const Position& position) {
    this->position = position;
}

// Destructor of the class Object
Object::~Object() {

}

// ----- Structure Position ----- //

// Constructor without arguments of the structure Position
Object::Position::Position() {
    x = 0.0;
    y = 0.0;
    z = 0.0;
}

// Constructor of the structure Position
Object::Position::Position(double x, double y, double z) {
  this->x = x;
  this->y = y;
  this->z = z;
};

// ----- Class Cube ----- //

// Constructor of the class Cube
Cube::Cube(const Position& position, float size)
     :Object(position) {
    this->size = size;
}

// Destructor of the classe Cube
Cube::~Cube() {

}

// Method to draw the cube
void Cube::draw() const {
       // Translation of the cube to its position
       glTranslatef(position.x, position.y, position.z);

       // Draw the cube
       glutSolidCube(size);
}


// ----- Class Plane ----- //

// Constructor of the class Plane
Plane::Plane(const Position& position, float width, float height, const Vector3D& d1, const Vector3D& d2)
      :Object(position) {
    this->width = width;
    this->height = height;
    this->d1 = d1;
    this->d2 = d2;

    // Compute the unit normal vector of the plane by a cross product
    normalVector = d1.crossProduct(d2).getUnit();
}

// Destructor of the class Plane
Plane::~Plane() {

}

// Method used to draw the plane
void Plane::draw() const {
       // Translation of the cube to its position
       glTranslatef(position.x, position.y, position.z);

       float halfWidth = width / 2.0;
       float halfHeight = height / 2.0;

       // Draw the plane
       glBegin(GL_POLYGON);
            glColor3f(1.0, 1.0, 1.0);
            glVertex3f(position.x + d1.getX() * halfWidth + d2.getX() * halfHeight , position.y + d1.getY() * halfWidth +  d2.getY() * halfHeight
                        , position.z + d1.getZ() * halfWidth + d2.getZ() * halfHeight);
            glNormal3f(normalVector.getX(), normalVector.getY(), normalVector.getZ());
            glVertex3f(position.x + d1.getX() * halfWidth - d2.getX() * halfHeight , position.y + d1.getY() * halfWidth -  d2.getY() * halfHeight
                        , position.z + d1.getZ() * halfWidth - d2.getZ() * halfHeight);
            glNormal3f(normalVector.getX(), normalVector.getY(), normalVector.getZ());
            glVertex3f(position.x - d1.getX() * halfWidth - d2.getX() * halfHeight , position.y - d1.getY() * halfWidth -  d2.getY() * halfHeight
                        , position.z - d1.getZ() * halfWidth - d2.getZ() * halfHeight);
            glNormal3f(normalVector.getX(), normalVector.getY(), normalVector.getZ());
            glVertex3f(position.x - d1.getX() * halfWidth + d2.getX() * halfHeight , position.y - d1.getY() * halfWidth +  d2.getY() * halfHeight
                        , position.z - d1.getZ() * halfWidth + d2.getZ() * halfHeight);
       glEnd();
}



