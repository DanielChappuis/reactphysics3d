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
#include "Contact.h"

using namespace reactphysics3d;
using namespace std;

// Constructor
Contact::Contact(RigidBody* const body1, RigidBody* const body2, const ContactInfo* contactInfo)
        : Constraint(body1, body2, 3, true, CONTACT), mNormal(contactInfo->normal),
          mPenetrationDepth(contactInfo->penetrationDepth),
          mLocalPointOnBody1(contactInfo->localPoint1),
          mLocalPointOnBody2(contactInfo->localPoint2),
          mWorldPointOnBody1(body1->getTransform() * contactInfo->localPoint1),
          mWorldPointOnBody2(body2->getTransform() * contactInfo->localPoint2),
          mIsRestingContact(false), mFrictionVectors(2, Vector3(0, 0, 0)) {
    assert(mPenetrationDepth > 0.0);

}

// Destructor
Contact::~Contact() {

}
