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
#include "SphereVsSphereAlgorithm.h"
#include "../../colliders/SphereCollider.h"

// We want to use the ReactPhysics3D namespace
using namespace reactphysics3d;

// Constructor
SphereVsSphereAlgorithm::SphereVsSphereAlgorithm(CollisionDetection& collisionDetection)
                        :NarrowPhaseAlgorithm(collisionDetection) {
    
}

// Destructor
SphereVsSphereAlgorithm::~SphereVsSphereAlgorithm() {
    
}   

bool SphereVsSphereAlgorithm::testCollision(const Collider* collider1, const Transform& transform1,
                                            const Collider* collider2, const Transform& transform2, ContactInfo*& contactInfo) {
    
    // Get the sphere colliders
    const SphereCollider* sphereCollider1 = dynamic_cast<const SphereCollider*>(collider1);
    const SphereCollider* sphereCollider2 = dynamic_cast<const SphereCollider*>(collider2);
    
    // Compute the distance between the centers
    Vector3 vectorBetweenCenters = transform2.getPosition() - transform1.getPosition();
    decimal squaredDistanceBetweenCenters = vectorBetweenCenters.lengthSquare();

    // Compute the sum of the radius
    decimal sumRadius = sphereCollider1->getRadius() + sphereCollider2->getRadius();
    
    // If the sphere colliders intersect
    if (squaredDistanceBetweenCenters <= sumRadius * sumRadius) {
        Vector3 centerSphere2InBody1LocalSpace = transform1.inverse() * transform2.getPosition();
        Vector3 centerSphere1InBody2LocalSpace = transform2.inverse() * transform1.getPosition();
        Vector3 intersectionOnBody1 = sphereCollider1->getRadius() * centerSphere2InBody1LocalSpace.getUnit();
        Vector3 intersectionOnBody2 = sphereCollider2->getRadius() * centerSphere1InBody2LocalSpace.getUnit();
        decimal penetrationDepth = sumRadius - std::sqrt(squaredDistanceBetweenCenters);
        contactInfo = new ContactInfo(vectorBetweenCenters.getUnit(), penetrationDepth, intersectionOnBody1,
                                      intersectionOnBody2, transform1, transform2);
    
        return true;
    }
    
    return false;
}