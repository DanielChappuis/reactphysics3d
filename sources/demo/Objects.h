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

#ifndef OBJECTS_H
#define OBJECTS_H

// Libraries
#include "../reactphysics3d/reactphysics3d.h"

// Namespaces
using namespace reactphysics3d;

/*
    Here we define all the objects that can appear in the simulation like cube, sphere, plane, ...
*/

// ----- Class Object (abstract) ----- //
// Represent an object of the simulation
class Object {
    protected :
        RigidBody* rigidBody;                                 // Rigid Body that represents the object

    public :
        Object(const Vector3D& position, const Kilogram& mass, const Matrix3x3& inertiaTensor);     // Constructor of the class Object
        virtual ~Object();                                                                          // Destructor of the class Object

        virtual void draw() const =0;                                                               // pure virtual method to draw the object
        RigidBody* getRigidBody();                                                                  // Return the pointer to the rigid body
};

// ----- Class Cube ----- //
// Represente a Cube in the simulation
class Cube : public Object {
    private :
        float size;                                      // Size of a side in the cube
        static const Matrix3x3 inertiaTensor;            // Inertia tensor of a cube
    public :
        Cube(const Vector3D& position, float size, const Kilogram& mass);       // Constructor of the class cube
        virtual ~Cube();                                                        // Destructor of the class cube
        virtual void draw() const;                                              // Method to draw the cube
};


// ----- Class Plane ---- //
// Represent a plane in the simulation
class Plane : public Object {
    private :
        float width;                                                                                            // Width of the plane
        float height;                                                                                           // Height of the plane
        Vector3D d1;                                                                                            // Unit vector in the plane
        Vector3D d2;                                                                                            // Unit vector in the plane
        Vector3D normalVector;                                                                                  // Unit normal vector of the plane

    public :
        Plane(const Vector3D& position, float width, float height, const Vector3D& d1, const Vector3D& d2, const Kilogram& mass);   // Constructor of the class Plane
        virtual ~Plane();                                                                                                           // Destructor of the class Plane
        virtual void draw() const;                                                                                                  // Method to draw the plane
};

#endif
