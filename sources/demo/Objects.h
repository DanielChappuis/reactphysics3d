#ifndef OBJECTS_H
#define OBJECTS_H

/*
    Here we define all the objects that can appear in the simulation like cube, sphere, plane, ...
*/

struct Vector {
    double x;                                        // x component
    double y;                                        // y component
    double z;                                        // z component

    // Methods
    Vector();                                       // Constructor without arguments of the structure Vector
    Vector(double x, double y, double z);              // Constructor of the structure Vector
};

// ----- Class Object (abstract) ----- //
// Represent an object of the simulation
class Object {
    public :
        // Structure Object::Position
        struct Position {
            double x;                                        // x coordinate
            double y;                                        // y coordinate
            double z;                                        // z coordinate

            // Methods
            Position();                                     // Constructor without arguments of the structure Position
            Position(double x, double y, double z);         // Constructor of the structure Position
        } position;                                         // Position of the object
        Object(const Position& position);                   // Constructor of the class Object
        virtual ~Object();                                  // Destructor of the class Object
        virtual void draw() const =0;                       // pure virtual method to draw the object
};

// ----- Class Cube ----- //
// Represente a Cube in the simulation
class Cube : public Object {
    private :
        float size;                                 // Size of a side in the cube

    public :
        Cube(const Position& position, float size);      // Constructor of the class cube
        virtual ~Cube();                                 // Destructor of the class cube
        virtual void draw() const;                       // Method to draw the cube
};


// ----- Class Plane ---- //
// Represent a plane in the simulation
class Plane : public Object {
    public :
        float width;                                                                                        // Width of the plane
        float height;                                                                                       // Height of the plane
        Vector d1;                                                                                          // Unit vector in the plane
        Vector d2;                                                                                          // Unit vector in the plane
        Vector normalVector;                                                                                // Unit normal vector of the plane
        Plane(const Position& position, float width, float height, const Vector& d1, const Vector& d2);     // Constructor of the class Plane
        virtual ~Plane();                                                                                   // Destructor of the class Plane
        virtual void draw() const;                                                                          // Method to draw the plane
};

#endif
