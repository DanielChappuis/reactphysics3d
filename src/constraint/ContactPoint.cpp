/********************************************************************************
* ReactPhysics3D physics library, http://www.reactphysics3d.com                 *
* Copyright (c) 2010-2024 Daniel Chappuis                                       *
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
#include <reactphysics3d/constraint/ContactPoint.h>
#include <reactphysics3d/collision/Collider.h>

using namespace reactphysics3d;
using namespace std;

// Constructor
ContactPoint::ContactPoint(const ContactPointInfo* contactInfo, decimal persistentContactDistanceThreshold)
             : mNormal(contactInfo->normal),
               mPenetrationDepth(contactInfo->penetrationDepth),
               mLocalPointOnShape1(contactInfo->localPoint1),
               mLocalPointOnShape2(contactInfo->localPoint2),
               mIsRestingContact(false), mPenetrationImpulse(0), mIsObsolete(false),
               mNext(nullptr), mPrevious(nullptr),
               mPersistentContactDistanceThreshold(persistentContactDistanceThreshold) {

    assert(mPenetrationDepth > decimal(0.0));
    assert(mNormal.lengthSquare() > decimal(0.8));

    mIsObsolete = false;
}

// Constructor
ContactPoint::ContactPoint(const ContactPointInfo& contactInfo, decimal persistentContactDistanceThreshold)
             : mNormal(contactInfo.normal),
               mPenetrationDepth(contactInfo.penetrationDepth),
               mLocalPointOnShape1(contactInfo.localPoint1),
               mLocalPointOnShape2(contactInfo.localPoint2),
               mIsRestingContact(false), mPenetrationImpulse(0), mIsObsolete(false),
               mNext(nullptr), mPrevious(nullptr),
               mPersistentContactDistanceThreshold(persistentContactDistanceThreshold) {

    assert(mPenetrationDepth > decimal(0.0));
    assert(mNormal.lengthSquare() > decimal(0.8));

    mIsObsolete = false;
}

// Update the contact point with a new one that is similar (very close)
/// The idea is to keep the cache impulse (for warm starting the contact solver)
void ContactPoint::update(const ContactPointInfo* contactInfo) {

    assert(isSimilarWithContactPoint(contactInfo));
    assert(contactInfo->penetrationDepth > decimal(0.0));

    mNormal = contactInfo->normal;
    mPenetrationDepth = contactInfo->penetrationDepth;
    mLocalPointOnShape1 = contactInfo->localPoint1;
    mLocalPointOnShape2 = contactInfo->localPoint2;

    mIsObsolete = false;
}
