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
#include "PhysicsWorld.h"
#include <algorithm>

// Namespaces
using namespace reactphysics3d;
using namespace std;

// Constructor
PhysicsWorld::PhysicsWorld(const Vector3& gravity)
             : gravity(gravity), isGravityOn(true) {

}

// Destructor
PhysicsWorld::~PhysicsWorld() {
    // Remove and free the memory of all constraints
    removeAllConstraints();
}

// Remove all collision contacts constraints
void PhysicsWorld::removeAllContactConstraints() {
    // For all constraints
    for (vector<Constraint*>::iterator it = constraints.begin(); it != constraints.end(); ) {

        // Try a downcasting
        Contact* contact = dynamic_cast<Contact*>(*it);

        // If the constraint is a contact
        if (contact) {
            // Delete the  contact
            delete (*it);
            it = constraints.erase(it);
        }
        else {
            ++it;
        }
    }
}

// Remove all constraints in the physics world and also delete them (free their memory)
void PhysicsWorld::removeAllConstraints() {
    for (vector<Constraint*>::iterator it = constraints.begin(); it != constraints.end(); ++it) {
        delete *it;
    }
    constraints.clear();
}

