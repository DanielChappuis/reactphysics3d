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
***************************************************************************/

// Libraries
#include "Context.h"
#include "../reactphysics3d/reactphysics3d.h"
#include <iostream>
#include <vector>

// We want to use the ReactPhysics3D namespace
using namespace reactphysics3d;

// Constructor of the class Context
Context::Context() {

    Cube* cube1 = new Cube(Vector3D(5.0, 13.0, 1), Quaternion(1.0, 1.0, 0.0, 0.0), 4.0, Kilogram(3.0));
    Cube* cube2 = new Cube(Vector3D(5.0, 13.0, 9), Quaternion(0.5, 0.5, 0.5, 0.0), 4.0, Kilogram(3.0));
    cube1->getRigidBody()->setLinearVelocity(Vector3D(0.0, 0.0, 0.5));
    cube2->getRigidBody()->setLinearVelocity(Vector3D(0.0, 0.0, -0.5));

    //Cube* cube2 = new Cube(Vector3D(0.0, 17, 8.0), Quaternion(0.0, 1.0, 0.0, 0.0), 3.0, Kilogram(2.0));
    //Cube* cube3 = new Cube(Vector3D(4.0, 17, -2.0), Quaternion(0.0, 1.0, 0.0, 0.0), 2.0, Kilogram(11.0));
    //Plane* plane1 = new Plane(Vector3D(0.0, 0.0, 0.0), Quaternion(0.0, 1.0, 0.0, 0.0), 20.0, 30.0, Vector3D(-1.0, 0.0, 0.0), Vector3D(0.0, 0.0, 1.0), Kilogram(10.0));

    addObject(cube1);
    addObject(cube2);
    //addObject(cube3);
    //addObject(plane1);
}


// Destructor of the class Context
Context::~Context() {
    // Delete all the objects in vectObjects
    for(unsigned int i=0; i<vectObjects.size(); ++i) {
        delete vectObjects[i];
    }
}

// Method to get an object from the context
Object& Context::getObject(int objectIndex) const {
    // TODO : WE HAVE TO ADD HERE AN EXCEPTION IMPLEMENTATION

    // Return the object from the context
    return (*vectObjects.at(objectIndex));     // TODO : THROWN AN EXCEPTION IF INDEX IS OUT OF THE BOUNDS
}

// Method for adding an object into the context
void Context::addObject(Object* object) {
    if (object != 0) {
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
