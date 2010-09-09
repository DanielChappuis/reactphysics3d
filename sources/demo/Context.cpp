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
#include "../../libraries/boost_1_43_0/boost/numeric/ublas/matrix.hpp"
#include "../../libraries/boost_1_43_0/boost/numeric/ublas/io.hpp"

// We want to use the ReactPhysics3D namespace
using namespace reactphysics3d;

// Constructor of the class Context
Context::Context() {

    /*
    using namespace boost::numeric::ublas;
    matrix<double> m (3, 3);
    for (unsigned i = 0; i < m.size1 (); ++ i)
        for (unsigned j = 0; j < m.size2 (); ++ j)
            m (i, j) = 3 * i + j;
    std::cout << m << std::endl;
    */
    
    /*
    //Cube* cube1 = new Cube(Vector3D(0, 10.0, 0), Quaternion(1.0, 1.0, 1.0, 0.0), 2.0, 1.0);
    //Cube* cube2 = new Cube(Vector3D(0.0, 0.0, 0.0), Quaternion(1.0, 1.0, 0.0, 0.0), 4.0, 1.0);
    Cube* cube1 = new Cube(Vector3D(2.2, 17.0, 2.2), Quaternion(1.0, 1.0, 0.3, 0.0), 2.0, 1.0);
    //Cube* cube2 = new Cube(Vector3D(0.0, 0.0, 0.0), Quaternion(1.0, 1.0, 0.0, 0.0), 4.0, 1.0);
    //cube1->getRigidBody()->setLinearVelocity(Vector3D(0.9, -0.9, 0.9));
    //cube2->getRigidBody()->setLinearVelocity(Vector3D(0.0, 0.0, 0.0));
    //cube2->getRigidBody()->setIsMotionEnabled(false);
    cube1->getRigidBody()->setRestitution(0.6);
    //cube2->getRigidBody()->setRestitution(0.6);
    addObject(cube1);
    //addObject(cube2);
    */
  
    for (int i=3; i<30; i=i+3) {
       Cube* cube = new Cube(Vector3D(1, i, 1+i*0.002), Quaternion(1.0, 1.0, 0.0, 0.0), 2.0, 4.0);
       cube->getRigidBody()->setRestitution(0.5);
       addObject(cube);
    }

    Plane* plane1 = new Plane(Vector3D(0.0, 0.0, 0.0), Quaternion(0.0, 1.0, 0.1 , 0.0), 20.0, 30.0, Vector3D(-1.0, 0.0, 0.0), Vector3D(0.0, 0.0, 1.0), 1.0);
    plane1->getRigidBody()->setRestitution(0.5);
    plane1->getRigidBody()->setIsMotionEnabled(false);
    addObject(plane1);
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

    // Return the object from the context
    return (*vectObjects.at(objectIndex));
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
