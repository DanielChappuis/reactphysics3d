/********************************************************************************
* ReactPhysics3D physics library, http://www.reactphysics3d.com                 *
* Copyright (c) 2010-2015 Daniel Chappuis                                       *
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
#include "ContactManifoldSet.h"

using namespace reactphysics3d;

// Constructor
ContactManifoldSet::ContactManifoldSet(ProxyShape* shape1, ProxyShape* shape2,
                                       MemoryAllocator& memoryAllocator, int nbMaxManifolds)
                   : mNbMaxManifolds(nbMaxManifolds), mNbManifolds(0), mShape1(shape1),
                     mShape2(shape2), mMemoryAllocator(memoryAllocator) {

}

// Destructor
ContactManifoldSet::~ContactManifoldSet() {

}

// Add a contact point to the manifold set
void ContactManifoldSet::addContactPoint(ContactPoint* contact) {

    // If there is no contact manifold yet
    if (mNbManifolds == 0) {

        createManifold();
        mManifolds[0]->addContactPoint(contact);

        return;
    }


}

// Return the index of the contact manifold with a similar average normal.
// If no manifold has close enough average normal, it returns -1
int ContactManifoldSet::selectManifoldWithSimilarNormal(const Vector3& normal) {

    decimal maxDotProduct;
    int indexManifold = -1;
    for (int i=0; i<mNbManifolds; i++) {
        decimal dotProduct = normal.dot(mManifolds[i]->getAverageContactNormal());
        if (dotProduct > maxDotProduct) {
            maxDotProduct = dotProduct;
            indexManifold = i;
        }
    }

    return indexManifold;
}

// Update the contact manifolds
void ContactManifoldSet::update(const Transform& transform1, const Transform& transform2) {

    for (int i=0; i<mNbManifolds; i++) {
        mManifolds[i]->update(transform1, transform2);
    }
}

// Clear the contact manifold set
void ContactManifoldSet::clear() {

}

// Create a new contact manifold and add it to the set
void ContactManifoldSet::createManifold() {
    assert(mNbManifolds < MAX_MANIFOLDS_IN_CONTACT_MANIFOLD_SET);

    mManifolds[mNbManifolds] = new (mMemoryAllocator.allocate(sizeof(ContactManifold)))
                                    ContactManifold(mShape1, mShape2, mMemoryAllocator);
    mNbManifolds++;
}

// Remove a contact manifold from the set
void ContactManifoldSet::removeManifold(int index) {

    assert(index >= 0 && index < mNbManifolds);

    // Delete the new contact
    mManifolds[index]->~ContactManifold();
    mMemoryAllocator.release(mManifolds[index], sizeof(ContactManifold));

    mNbManifolds--;
}
