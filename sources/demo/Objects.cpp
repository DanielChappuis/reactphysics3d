
// Libraries
#include "Objects.h"

//#include <windows.h>            // To avoid an error due to the #include <GL/glut.h>
#include <GL/freeglut.h>
#include <math.h>


// ----- Structure Vector ----- //

// Constructor without arguments of the structure Vector
Vector::Vector() {
    x = 0.0;
    y = 0.0;
    z = 0.0;
}

// Constructor of the structure Vector
Vector::Vector(double x, double y, double z) {
    this->x = x;
    this->y = y;
    this->z = z;
};

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
Plane::Plane(const Position& position, float width, float height, const Vector& d1, const Vector& d2)
      :Object(position) {
    this->width = width;
    this->height = height;
    this->d1 = d1;
    this->d2 = d2;

    // Compute the unit normal vector of the plane by a cross product
    normalVector.x = d1.y * d2.z - d1.z * d2.y;
    normalVector.y = d1.z * d2.x - d1.x * d2.z;
    normalVector.z = d1.x * d2.y - d1.y * d2.x;
    float length = sqrt(normalVector.x * normalVector.x + normalVector.y * normalVector.y + normalVector.z * normalVector.z);
    normalVector.x = normalVector.x / length;
    normalVector.y = normalVector.y / length;
    normalVector.z = normalVector.z / length;
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
            glVertex3f(position.x + d1.x * halfWidth + d2.x * halfHeight , position.y + d1.y * halfWidth +  d2.y * halfHeight
                        , position.z + d1.z * halfWidth + d2.z * halfHeight);
            glNormal3f(normalVector.x, normalVector.y, normalVector.z);
            glVertex3f(position.x + d1.x * halfWidth - d2.x * halfHeight , position.y + d1.y * halfWidth -  d2.y * halfHeight
                        , position.z + d1.z * halfWidth - d2.z * halfHeight);
            glNormal3f(normalVector.x, normalVector.y, normalVector.z);
            glVertex3f(position.x - d1.x * halfWidth - d2.x * halfHeight , position.y - d1.y * halfWidth -  d2.y * halfHeight
                        , position.z - d1.z * halfWidth - d2.z * halfHeight);
            glNormal3f(normalVector.x, normalVector.y, normalVector.z);
            glVertex3f(position.x - d1.x * halfWidth + d2.x * halfHeight , position.y - d1.y * halfWidth +  d2.y * halfHeight
                        , position.z - d1.z * halfWidth + d2.z * halfHeight);
       glEnd();
}



