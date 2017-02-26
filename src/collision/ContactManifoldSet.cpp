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
                                       PoolAllocator& memoryAllocator, int nbMaxManifolds)
                   : mNbMaxManifolds(nbMaxManifolds), mNbManifolds(0), mShape1(shape1),
                     mShape2(shape2), mMemoryAllocator(memoryAllocator) {

}

// Destructor
ContactManifoldSet::~ContactManifoldSet() {

    // Clear all the contact manifolds
    clear();
}

void ContactManifoldSet::addContactManifold(const ContactManifoldInfo& contactManifoldInfo) {

    assert(contactManifoldInfo.getFirstContactPointInfo() != nullptr);

    // If there is no contact manifold yet
    if (mNbManifolds == 0) {

        // If the maximum number of manifold is 1
        if (mNbMaxManifolds == 1) {
            createManifold(contactManifoldInfo, 0);
        }
        else {

            // Compute an Id corresponding to the normal direction (using a cubemap)
            short int normalDirectionId = computeCubemapNormalId(contactManifoldInfo.getFirstContactPointInfo()->normal);

            createManifold(contactManifoldInfo, normalDirectionId);
        }
    }
    else {   // If there is already at least one contact manifold in the set

        // If the maximum number of manifold is 1
        if (mNbMaxManifolds == 1) {

            // Replace the old manifold with the new one
            updateManifoldWithNewOne(0, contactManifoldInfo);
        }
        else {

            // Compute an Id corresponding to the normal direction (using a cubemap)
            short int normalDirectionId = computeCubemapNormalId(contactManifoldInfo.getFirstContactPointInfo()->normal);

            // Select the manifold with the most similar normal (if exists)
            int similarManifoldIndex = 0;
            if (mNbMaxManifolds > 1) {
                similarManifoldIndex = selectManifoldWithSimilarNormal(normalDirectionId);
            }

            // If a similar manifold has been found
            if (similarManifoldIndex != -1) {

                // Replace the old manifold with the new one
                updateManifoldWithNewOne(similarManifoldIndex, contactManifoldInfo);
            }
            else {

                // If we have not reach the maximum number of manifolds
                if (mNbManifolds < mNbMaxManifolds) {

                    // Create the new contact manifold
                    createManifold(contactManifoldInfo, normalDirectionId);
                }
                else {

                    decimal newManifoldPenDepth = contactManifoldInfo.getLargestPenetrationDepth();

                    // We have reached the maximum number of manifold, we do not
                    // want to keep the manifold with the smallest penetration detph
                    int smallestPenDepthManifoldIndex = getManifoldWithSmallestContactPenetrationDepth(newManifoldPenDepth);

                    // If the manifold with the smallest penetration depth is not the new one,
                    // we have to keep the new manifold and remove the one with the smallest depth
                    if (smallestPenDepthManifoldIndex >= 0) {
                        removeManifold(smallestPenDepthManifoldIndex);
                        createManifold(contactManifoldInfo, normalDirectionId);
                    }
                }
            }
        }
    }
}

// Update a previous similar manifold with a new one
void ContactManifoldSet::updateManifoldWithNewOne(int oldManifoldIndex, const ContactManifoldInfo& newManifold) {

   // For each contact point of the previous manifold
   for (int i=0; i<mManifolds[oldManifoldIndex]->getNbContactPoints(); ) {

       ContactPoint* contactPoint = mManifolds[oldManifoldIndex]->getContactPoint(i);

       // For each contact point in the new manifold
       ContactPointInfo* newPoint = newManifold.getFirstContactPointInfo();
       bool needToRemovePoint = true;
       while (newPoint != nullptr) {

            // If the new contact point is similar (very close) to the old contact point
            if (!newPoint->isUsed && contactPoint->isSimilarWithContactPoint(newPoint)) {

                // Replace (update) the old contact point with the new one
                contactPoint->update(newPoint, mManifolds[oldManifoldIndex]->getShape1()->getLocalToWorldTransform(),
                                               mManifolds[oldManifoldIndex]->getShape2()->getLocalToWorldTransform());
                needToRemovePoint = false;
                newPoint->isUsed = true;
                break;
            }

            newPoint = newPoint->next;
       }

       // If no new contact point is similar to the old one
       if (needToRemovePoint) {

           // Remove the old contact point
           mManifolds[oldManifoldIndex]->removeContactPoint(i);
       }
       else {
           i++;
       }
   }

   // For each point of the new manifold that have not been used yet (to update
   // an existing point in the previous manifold), add it into the previous manifold
   const ContactPointInfo* newPointInfo = newManifold.getFirstContactPointInfo();
   while (newPointInfo != nullptr) {

        if (!newPointInfo->isUsed) {
            mManifolds[oldManifoldIndex]->addContactPoint(newPointInfo);
        }

        newPointInfo = newPointInfo->next;
   }
}

// Return the manifold with the smallest contact penetration depth
int ContactManifoldSet::getManifoldWithSmallestContactPenetrationDepth(decimal initDepth) const {

    // The contact point will be in a new contact manifold, we now have too much
    // manifolds condidates. We need to remove one. We choose to remove the manifold
    // with the smallest contact depth among their points
    int smallestDepthManifoldIndex = -1;
    decimal minDepth = initDepth;
    assert(mNbManifolds == mNbMaxManifolds);
    for (int i=0; i<mNbManifolds; i++) {
        decimal depth = mManifolds[i]->getLargestContactDepth();
        if (depth < minDepth) {
            minDepth = depth;
            smallestDepthManifoldIndex = i;
        }
    }

    return smallestDepthManifoldIndex;
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
    decimal max = max3(std::fabs(normal.x), std::fabs(normal.y), std::fabs(normal.z));
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

    int indexU = std::floor(((u + 1)/2) * CONTACT_CUBEMAP_FACE_NB_SUBDIVISIONS);
    int indexV = std::floor(((v + 1)/2) * CONTACT_CUBEMAP_FACE_NB_SUBDIVISIONS);
    if (indexU == CONTACT_CUBEMAP_FACE_NB_SUBDIVISIONS) indexU--;
    if (indexV == CONTACT_CUBEMAP_FACE_NB_SUBDIVISIONS) indexV--;

    const int nbSubDivInFace = CONTACT_CUBEMAP_FACE_NB_SUBDIVISIONS * CONTACT_CUBEMAP_FACE_NB_SUBDIVISIONS;
    return faceNo * 200 + indexU * nbSubDivInFace + indexV;
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
void ContactManifoldSet::createManifold(const ContactManifoldInfo& manifoldInfo, short int normalDirectionId) {
    assert(mNbManifolds < mNbMaxManifolds);

    mManifolds[mNbManifolds] = new (mMemoryAllocator.allocate(sizeof(ContactManifold)))
                                    ContactManifold(manifoldInfo, mShape1, mShape2, mMemoryAllocator, normalDirectionId);
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
