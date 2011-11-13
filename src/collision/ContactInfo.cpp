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
#include "ContactInfo.h"

using namespace reactphysics3d;


// Constructor for GJK
ContactInfo::ContactInfo(Body* body1, Body* body2, const Vector3& normal, double penetrationDepth,
                         const Vector3& localPoint1, const Vector3& localPoint2,
                         const Transform& transform1, const Transform& transform2)
            : body1(body1), body2(body2), normal(normal), penetrationDepth(penetrationDepth),
              localPoint1(localPoint1), localPoint2(localPoint2), worldPoint1(transform1 * localPoint1), worldPoint2(transform2 * localPoint2) {

}
