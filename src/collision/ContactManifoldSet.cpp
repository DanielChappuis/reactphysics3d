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
#include "ContactManifoldSet.h"

using namespace reactphysics3d;

// Constructor
ContactManifoldSet::ContactManifoldSet(ProxyShape* shape1, ProxyShape* shape2,
                                       MemoryAllocator& memoryAllocator, int nbMaxManifolds)
                   : mNbMaxManifolds(nbMaxManifolds), mNbManifolds(0), mShape1(shape1),
                     mShape2(shape2), mMemoryAllocator(memoryAllocator) {
    assert(nbMaxManifolds >= 1);
}

// Destructor
ContactManifoldSet::~ContactManifoldSet() {

    // Clear all the contact manifolds
    clear();
}

// Add a contact point to the manifold set
void ContactManifoldSet::addContactPoint(ContactPoint* contact) {

    // Compute an Id corresponding to the normal direction (using a cubemap)
    short int normalDirectionId = computeCubemapNormalId(contact->getNormal());

    // If there is no contact manifold yet
    if (mNbManifolds == 0) {

        createManifold(normalDirectionId);
        mManifolds[0]->addContactPoint(contact);
        assert(mManifolds[mNbManifolds-1]->getNbContactPoints() > 0);
        for (int i=0; i<mNbManifolds; i++) {
            assert(mManifolds[i]->getNbContactPoints() > 0);
        }

        return;
    }

    // Select the manifold with the most similar normal (if exists)
    int similarManifoldIndex = 0;
    if (mNbMaxManifolds > 1) {
        similarManifoldIndex = selectManifoldWithSimilarNormal(normalDirectionId);
    }

    // If a similar manifold has been found
    if (similarManifoldIndex != -1) {

        // Add the contact point to that similar manifold
        mManifolds[similarManifoldIndex]->addContactPoint(contact);
        assert(mManifolds[similarManifoldIndex]->getNbContactPoints() > 0);

        return;
    }

    // If the maximum number of manifold has not been reached yet
    if (mNbManifolds < mNbMaxManifolds) {

        // Create a new manifold for the contact point
        createManifold(normalDirectionId);
        mManifolds[mNbManifolds-1]->addContactPoint(contact);
        for (int i=0; i<mNbManifolds; i++) {
            assert(mManifolds[i]->getNbContactPoints() > 0);
        }

        return;
    }

    // The contact point will be in a new contact manifold, we now have too much
    // manifolds condidates. We need to remove one. We choose to keep the manifolds
    // with the largest contact depth among their points
    int smallestDepthIndex = -1;
    decimal minDepth = contact->getPenetrationDepth();
    assert(mNbManifolds == mNbMaxManifolds);
    for (int i=0; i<mNbManifolds; i++) {
        decimal depth = mManifolds[i]->getLargestContactDepth();
        if (depth < minDepth) {
            minDepth = depth;
            smallestDepthIndex = i;
        }
    }

    // If we do not want to keep to new manifold (not created yet) with the
    // new contact point
    if (smallestDepthIndex == -1) {

        // Delete the new contact
        contact->~ContactPoint();
        mMemoryAllocator.release(contact, sizeof(ContactPoint));

        return;
    }

    assert(smallestDepthIndex >= 0 && smallestDepthIndex < mNbManifolds);

    // Here we need to replace an existing manifold with a new one (that contains
    // the new contact point)
    removeManifold(smallestDepthIndex);
    createManifold(normalDirectionId);
    mManifolds[mNbManifolds-1]->addContactPoint(contact);
    assert(mManifolds[mNbManifolds-1]->getNbContactPoints() > 0);
    for (int i=0; i<mNbManifolds; i++) {
        assert(mManifolds[i]->getNbContactPoints() > 0);
    }

    return;
}

// Return the index of the contact manifold with a similar average normal.
// If no manifold has close enough average normal, it returns -1
int ContactManifoldSet::selectManifoldWithSimilarNormal(short int normalDirectionId) const {

    // Return the Id of the manifold with the same normal direction id (if exists)
    for (int i=0; i<mNbManifolds; i++) {
        if (normalDirectionId == mManifolds[i]->getNormalDirectionId()) {
            return i;
        }
    }

    return -1;
}

// Map the normal vector into a cubemap face bucket (a face contains 4x4 buckets)
// Each face of the cube is divided into 4x4 buckets. This method maps the
// normal vector into of the of the bucket and returns a unique Id for the bucket
short int ContactManifoldSet::computeCubemapNormalId(const Vector3& normal) const {

    assert(normal.lengthSquare() > MACHINE_EPSILON);

    int faceNo;
    decimal u, v;
    decimal max = max3(fabs(normal.x), fabs(normal.y), fabs(normal.z));
    Vector3 normalScaled = normal / max;

    if (normalScaled.x >= normalScaled.y && normalScaled.x >= normalScaled.z) {
        faceNo = normalScaled.x > 0 ? 0 : 1;
        u = normalScaled.y;
        v = normalScaled.z;
    }
    else if (normalScaled.y >= normalScaled.x && normalScaled.y >= normalScaled.z) {
        faceNo = normalScaled.y > 0 ? 2 : 3;
        u = normalScaled.x;
        v = normalScaled.z;
    }
    else {
        faceNo = normalScaled.z > 0 ? 4 : 5;
        u = normalScaled.x;
        v = normalScaled.y;
    }

    int indexU = floor(((u + 1)/2) * CONTACT_CUBEMAP_FACE_NB_SUBDIVISIONS);
    int indexV = floor(((v + 1)/2) * CONTACT_CUBEMAP_FACE_NB_SUBDIVISIONS);
    if (indexU == CONTACT_CUBEMAP_FACE_NB_SUBDIVISIONS) indexU--;
    if (indexV == CONTACT_CUBEMAP_FACE_NB_SUBDIVISIONS) indexV--;

    const int nbSubDivInFace = CONTACT_CUBEMAP_FACE_NB_SUBDIVISIONS * CONTACT_CUBEMAP_FACE_NB_SUBDIVISIONS;
    return faceNo * 200 + indexU * nbSubDivInFace + indexV;
}

// Update the contact manifolds
void ContactManifoldSet::update() {

    for (int i=mNbManifolds-1; i>=0; i--) {

        // Update the contact manifold
        mManifolds[i]->update(mShape1->getBody()->getTransform() * mShape1->getLocalToBodyTransform(),
                              mShape2->getBody()->getTransform() * mShape2->getLocalToBodyTransform());

        // Remove the contact manifold if has no contact points anymore
        if (mManifolds[i]->getNbContactPoints() == 0) {
            removeManifold(i);
        }
    }
}

// Clear the contact manifold set
void ContactManifoldSet::clear() {

    // Destroy all the contact manifolds
    for (int i=mNbManifolds-1; i>=0; i--) {
        removeManifold(i);
    }

    assert(mNbManifolds == 0);
}

// Create a new contact manifold and add it to the set
void ContactManifoldSet::createManifold(short int normalDirectionId) {
    assert(mNbManifolds < mNbMaxManifolds);

    mManifolds[mNbManifolds] = new (mMemoryAllocator.allocate(sizeof(ContactManifold)))
                                    ContactManifold(mShape1, mShape2, mMemoryAllocator, normalDirectionId);
    mNbManifolds++;
}

// Remove a contact manifold from the set
void ContactManifoldSet::removeManifold(int index) {

    assert(mNbManifolds > 0);
    assert(index >= 0 && index < mNbManifolds);

    // Delete the new contact
    mManifolds[index]->~ContactManifold();
    mMemoryAllocator.release(mManifolds[index], sizeof(ContactManifold));

    for (int i=index; (i+1) < mNbManifolds; i++) {
        mManifolds[i] = mManifolds[i+1];
    }

    mNbManifolds--;
}
