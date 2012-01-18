/********************************************************************************
* ReactPhysics3D physics library, http://code.google.com/p/reactphysics3d/      *
* Copyright (c) 2010-2012 Daniel Chappuis                                       *
*********************************************************************************
*                                                                               *
* This software is provided 'as-is', without any express or implied warranty.   *
* In no event will the authors be held liable for any damages arising from the  *
* use of this software.                                                         *
*                                                                               *
* Permission is granted to anyone to use this software for any purpose,         *
* including commercial applications, and to alter it and redistribute it        *
* freely, subject to the following restrictions:                                *
*                                                                               *
* 1. The origin of this software must not be misrepresented; you must not claim *
*    that you wrote the original software. If you use this software in a        *
*    product, an acknowledgment in the product documentation would be           *
*    appreciated but is not required.                                           *
*                                                                               *
* 2. Altered source versions must be plainly marked as such, and must not be    *
*    misrepresented as being the original software.                             *
*                                                                               *
* 3. This notice may not be removed or altered from any source distribution.    *
*                                                                               *
********************************************************************************/

// Libraries
#include "RigidBody.h"
#include "../colliders/Collider.h"

// We want to use the ReactPhysics3D namespace
using namespace reactphysics3d;

 // Constructor
 RigidBody::RigidBody(const Transform& transform, decimal mass, const Matrix3x3& inertiaTensorLocal, Collider* collider, long unsigned id)
           : Body(transform, collider, mass, id), inertiaTensorLocal(inertiaTensorLocal),
             inertiaTensorLocalInverse(inertiaTensorLocal.getInverse()), massInverse(1.0/mass) {

    restitution = 1.0;

    // Set the body pointer of the AABB and the collider
    aabb->setBodyPointer(this);
    collider->setBodyPointer(this);

    assert(collider);
    assert(aabb);
}

// Destructor
RigidBody::~RigidBody() {

};

