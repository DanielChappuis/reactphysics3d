
// Libraries
#include "Context.h"

#include <iostream>
#include <vector>


// Constructor of the class Context
Context::Context() {
    
    // We add some objects in the context at the beginning ---> THESE THINGS WILL BE STORE IN A TEXT FILE
    Cube * cube1 = new Cube(Object::Position(-2.0, 1.0, -6.0), 2.0);
    Cube * cube2 = new Cube(Object::Position(0.0, 1.5, 6.0), 3.0);
    Cube * cube3 = new Cube(Object::Position(4.0, 4.0, -2.0), 2.0);
    Plane * plane1 = new Plane(Object::Position(0.0, 0.0, 0.0), 20.0, 30.0, Vector(-1.0, 0.0, 0.0), Vector(0.0, 0.0, 1.0));
    
    addObject(cube1);
    addObject(cube2);
    addObject(cube3);
    addObject(plane1);
}


// Destructor of the class Context
Context::~Context() {
    // Delete all the objects in vectObjects
    for(int i=0; i<vectObjects.size(); ++i) {
        delete vectObjects[0];    
    }
}

// Method to get an object from the context
Object& Context::getObject(int objectIndex) const {
    // WE HAVE TO ADD HERE AN EXCEPTION IMPLEMENTATION

    // Return the object from the context
    return (*vectObjects.at(objectIndex));     // THROWN AN EXCEPTION IF INDEX IS OUT OF THE BOUNDS
}

// Method for adding an object into the context
void Context::addObject(Object* object) {
       if (object != NULL) {    
            // Add the object into the context
            vectObjects.push_back(object);
       }
}
     
// Method to remove an object from the context
void Context::removeObject(int objectIndex) {
    // WE HAVE TO ADD HERE AN EXCEPTION IMPLEMENTATION
    
    // Restore the memory of the element
    delete vectObjects[objectIndex];
    
    // Erase the element in the vector
    vectObjects.erase(vectObjects.begin()+objectIndex);
}
