/********************************************************************************
* ReactPhysics3D physics library, http://www.reactphysics3d.com                 *
* Copyright (c) 2010-2019 Daniel Chappuis                                       *
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
#include "collision/CollisionCallback.h"
#include "engine/OverlappingPair.h"
#include "memory/MemoryAllocator.h"
#include "collision/ContactManifold.h"
#include "memory/MemoryManager.h"

// We want to use the ReactPhysics3D namespace
using namespace reactphysics3d;

// Constructor
CollisionCallback::CollisionCallbackInfo::CollisionCallbackInfo(OverlappingPair* pair, MemoryManager& memoryManager) :
    contactManifoldElements(nullptr), body1(pair->getShape1()->getBody()),
    body2(pair->getShape2()->getBody()),
    proxyShape1(pair->getShape1()), proxyShape2(pair->getShape2()),
    mMemoryManager(memoryManager) {

    assert(pair != nullptr);

    const ContactManifoldSet& manifoldSet = pair->getContactManifoldSet();

    // For each contact manifold in the set of manifolds in the pair
    ContactManifold* contactManifold = manifoldSet.getContactManifolds();
	assert(contactManifold != nullptr);
    while (contactManifold != nullptr) {

        assert(contactManifold->getNbContactPoints() > 0);

        // Add the contact manifold at the beginning of the linked
        // list of contact manifolds of the first body
        ContactManifoldListElement* element = new (mMemoryManager.allocate(MemoryManager::AllocationType::Pool,
                                                                           sizeof(ContactManifoldListElement)))
                                                      ContactManifoldListElement(contactManifold,
                                                                         contactManifoldElements);
        contactManifoldElements = element;

        contactManifold = contactManifold->getNext();
    }
}

// Destructor
CollisionCallback::CollisionCallbackInfo::~CollisionCallbackInfo() {

    // Release memory allocator for the contact manifold list elements
    ContactManifoldListElement* element = contactManifoldElements;
    while (element != nullptr) {

        ContactManifoldListElement* nextElement = element->getNext();

        // Delete and release memory
        element->~ContactManifoldListElement();
        mMemoryManager.release(MemoryManager::AllocationType::Pool, element,
                               sizeof(ContactManifoldListElement));

        element = nextElement;
    }
}

