/********************************************************************************
* ReactPhysics3D physics library, http://www.reactphysics3d.com                 *
* Copyright (c) 2010-2016 Daniel Chappuis                                       *
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
#include "ContactPoint.h"
#include "collision/ProxyShape.h"

using namespace reactphysics3d;
using namespace std;

// Constructor
ContactPoint::ContactPoint(const ContactPointInfo& contactInfo)
             : mBody1(contactInfo.shape1->getBody()), mBody2(contactInfo.shape2->getBody()),
               mNormal(contactInfo.normal),
               mPenetrationDepth(contactInfo.penetrationDepth),
               mLocalPointOnBody1(contactInfo.localPoint1),
               mLocalPointOnBody2(contactInfo.localPoint2),
               mWorldPointOnBody1(contactInfo.shape1->getBody()->getTransform() *
                                  contactInfo.shape1->getLocalToBodyTransform() *
                                  contactInfo.localPoint1),
               mWorldPointOnBody2(contactInfo.shape2->getBody()->getTransform() *
                                  contactInfo.shape2->getLocalToBodyTransform() *
                                  contactInfo.localPoint2),
               mIsRestingContact(false) {

    mFrictionVectors[0] = Vector3(0, 0, 0);
    mFrictionVectors[1] = Vector3(0, 0, 0);

    assert(mPenetrationDepth > 0.0);

}

// Destructor
ContactPoint::~ContactPoint() {

}
