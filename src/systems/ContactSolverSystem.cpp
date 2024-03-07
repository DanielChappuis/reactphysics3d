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
#include <reactphysics3d/systems/ContactSolverSystem.h>
#include <reactphysics3d/engine/PhysicsWorld.h>
#include <reactphysics3d/body/RigidBody.h>
#include <reactphysics3d/constraint/ContactPoint.h>
#include <reactphysics3d/utils/Profiler.h>
#include <reactphysics3d/engine/Island.h>
#include <reactphysics3d/collision/Collider.h>
#include <reactphysics3d/components/BodyComponents.h>
#include <reactphysics3d/components/ColliderComponents.h>
#include <reactphysics3d/collision/ContactManifold.h>
#include <algorithm>

using namespace reactphysics3d;
using namespace std;

// Constants initialization
const decimal ContactSolverSystem::BETA = decimal(0.2);
const decimal ContactSolverSystem::BETA_SPLIT_IMPULSE = decimal(0.2);
const decimal ContactSolverSystem::SLOP = decimal(0.01);

// Constructor
ContactSolverSystem::ContactSolverSystem(MemoryManager& memoryManager, PhysicsWorld& world, Islands& islands,
                                         BodyComponents& bodyComponents, RigidBodyComponents& rigidBodyComponents,
                                         ColliderComponents& colliderComponents, decimal& restitutionVelocityThreshold)
              :mMemoryManager(memoryManager), mWorld(world), mTimeStep(-1), mRestitutionVelocityThreshold(restitutionVelocityThreshold),
               mContactConstraints(nullptr), mContactPoints(nullptr),
               mNbContactPoints(0), mNbContactManifolds(0),
               mIslands(islands), mAllContactManifolds(nullptr), mAllContactPoints(nullptr),
               mBodyComponents(bodyComponents), mRigidBodyComponents(rigidBodyComponents),
               mColliderComponents(colliderComponents), mIsSplitImpulseActive(true) {

#ifdef IS_RP3D_PROFILING_ENABLED

        mProfiler = nullptr;
#endif

}

// Initialize the contact constraints
void ContactSolverSystem::init(Array<ContactManifold>* contactManifolds, Array<ContactPoint>* contactPoints, decimal timeStep) {

    mAllContactManifolds = contactManifolds;
    mAllContactPoints = contactPoints;

    RP3D_PROFILE("ContactSolver::init()", mProfiler);

    mTimeStep = timeStep;

    const uint32 nbContactManifolds = static_cast<uint32>(mAllContactManifolds->size());
    const uint32 nbContactPoints = static_cast<uint32>(mAllContactPoints->size());

    mNbContactManifolds = 0;
    mNbContactPoints = 0;

    mContactConstraints = nullptr;
    mContactPoints = nullptr;

    if (nbContactManifolds == 0 || nbContactPoints == 0) return;

    mContactPoints = static_cast<ContactPointSolver*>(mMemoryManager.allocate(MemoryManager::AllocationType::Frame,
                                                                              sizeof(ContactPointSolver) * nbContactPoints));
    assert(mContactPoints != nullptr);

    mContactConstraints = static_cast<ContactManifoldSolver*>(mMemoryManager.allocate(MemoryManager::AllocationType::Frame,
                                                                                      sizeof(ContactManifoldSolver) * nbContactManifolds));
    assert(mContactConstraints != nullptr);

    // For each island of the world
    const uint32 nbIslands = mIslands.getNbIslands();
    for (uint32 i = 0; i < nbIslands; i++) {

        if (mIslands.nbContactManifolds[i] > 0) {
            initializeForIsland(i);
        }
    }

    // Warmstarting
    warmStart();
}

// Release allocated memory
void ContactSolverSystem::reset() {

    if (mAllContactPoints->size() > 0) mMemoryManager.release(MemoryManager::AllocationType::Frame, mContactPoints, sizeof(ContactPointSolver) * mAllContactPoints->size());
    if (mAllContactManifolds->size() > 0) mMemoryManager.release(MemoryManager::AllocationType::Frame, mContactConstraints, sizeof(ContactManifoldSolver) * mAllContactManifolds->size());
}

// Initialize the constraint solver for a given island
void ContactSolverSystem::initializeForIsland(uint32 islandIndex) {

    RP3D_PROFILE("ContactSolver::initializeForIsland()", mProfiler);

    assert(mIslands.nbBodiesInIsland[islandIndex] > 0);
    assert(mIslands.nbContactManifolds[islandIndex] > 0);

    // For each contact manifold of the island
    const uint32 contactManifoldsIndex = mIslands.contactManifoldsIndices[islandIndex];
    const uint32 nbContactManifolds = mIslands.nbContactManifolds[islandIndex];
    for (uint32 m=contactManifoldsIndex; m < contactManifoldsIndex + nbContactManifolds; m++) {

        ContactManifold& externalManifold = (*mAllContactManifolds)[m];

        assert(externalManifold.nbContactPoints > 0);

        const uint32 rigidBodyIndex1 = mRigidBodyComponents.getEntityIndex(externalManifold.bodyEntity1);
        const uint32 rigidBodyIndex2 = mRigidBodyComponents.getEntityIndex(externalManifold.bodyEntity2);

        const uint32 collider1Index = mColliderComponents.getEntityIndex(externalManifold.colliderEntity1);
        const uint32 collider2Index = mColliderComponents.getEntityIndex(externalManifold.colliderEntity2);

        // Get the position of the two bodies
        const Vector3& x1 = mRigidBodyComponents.mCentersOfMassWorld[rigidBodyIndex1];
        const Vector3& x2 = mRigidBodyComponents.mCentersOfMassWorld[rigidBodyIndex2];

        // Initialize the internal contact manifold structure using the external contact manifold
        new (mContactConstraints + mNbContactManifolds) ContactManifoldSolver();
        mContactConstraints[mNbContactManifolds].rigidBodyComponentIndexBody1 = rigidBodyIndex1;
        mContactConstraints[mNbContactManifolds].rigidBodyComponentIndexBody2 = rigidBodyIndex2;
        mContactConstraints[mNbContactManifolds].inverseInertiaTensorBody1 = mRigidBodyComponents.mInverseInertiaTensorsWorld[rigidBodyIndex1];
        mContactConstraints[mNbContactManifolds].inverseInertiaTensorBody2 = mRigidBodyComponents.mInverseInertiaTensorsWorld[rigidBodyIndex2];
        mContactConstraints[mNbContactManifolds].massInverseBody1 = mRigidBodyComponents.mInverseMasses[rigidBodyIndex1];
        mContactConstraints[mNbContactManifolds].massInverseBody2 = mRigidBodyComponents.mInverseMasses[rigidBodyIndex2];
        mContactConstraints[mNbContactManifolds].linearLockAxisFactorBody1 = mRigidBodyComponents.mLinearLockAxisFactors[rigidBodyIndex1];
        mContactConstraints[mNbContactManifolds].linearLockAxisFactorBody2 = mRigidBodyComponents.mLinearLockAxisFactors[rigidBodyIndex2];
        mContactConstraints[mNbContactManifolds].angularLockAxisFactorBody1 = mRigidBodyComponents.mAngularLockAxisFactors[rigidBodyIndex1];
        mContactConstraints[mNbContactManifolds].angularLockAxisFactorBody2 = mRigidBodyComponents.mAngularLockAxisFactors[rigidBodyIndex2];
        mContactConstraints[mNbContactManifolds].nbContacts = externalManifold.nbContactPoints;
        mContactConstraints[mNbContactManifolds].frictionCoefficient = computeMixedFrictionCoefficient(mColliderComponents.mMaterials[collider1Index], mColliderComponents.mMaterials[collider2Index]);
        mContactConstraints[mNbContactManifolds].externalContactManifold = &externalManifold;
        mContactConstraints[mNbContactManifolds].normal.setToZero();
        mContactConstraints[mNbContactManifolds].frictionPointBody1.setToZero();
        mContactConstraints[mNbContactManifolds].frictionPointBody2.setToZero();

        // Get the velocities of the bodies
        const Vector3& v1 = mRigidBodyComponents.mLinearVelocities[rigidBodyIndex1];
        const Vector3& w1 = mRigidBodyComponents.mAngularVelocities[rigidBodyIndex1];
        const Vector3& v2 = mRigidBodyComponents.mLinearVelocities[rigidBodyIndex2];
        const Vector3& w2 = mRigidBodyComponents.mAngularVelocities[rigidBodyIndex2];

        const Transform& collider1LocalToWorldTransform = mColliderComponents.mLocalToWorldTransforms[collider1Index];
        const Transform& collider2LocalToWorldTransform = mColliderComponents.mLocalToWorldTransforms[collider2Index];

        // For each  contact point of the contact manifold
        assert(externalManifold.nbContactPoints > 0);
        const uint32 contactPointsStartIndex = externalManifold.contactPointsIndex;
        const uint32 nbContactPoints = static_cast<uint>(externalManifold.nbContactPoints);
        for (uint32 c=contactPointsStartIndex; c < contactPointsStartIndex + nbContactPoints; c++) {

            ContactPoint& externalContact = (*mAllContactPoints)[c];

            new (mContactPoints + mNbContactPoints) ContactPointSolver();
            mContactPoints[mNbContactPoints].externalContact = &externalContact;
            mContactPoints[mNbContactPoints].normal = externalContact.getNormal();

            // Get the contact point on the two bodies
            const Vector3 p1 = collider1LocalToWorldTransform * externalContact.getLocalPointOnShape1();
            const Vector3 p2 = collider2LocalToWorldTransform * externalContact.getLocalPointOnShape2();

            mContactPoints[mNbContactPoints].r1.x = p1.x - x1.x;
            mContactPoints[mNbContactPoints].r1.y = p1.y - x1.y;
            mContactPoints[mNbContactPoints].r1.z = p1.z - x1.z;
            mContactPoints[mNbContactPoints].r2.x = p2.x - x2.x;
            mContactPoints[mNbContactPoints].r2.y = p2.y - x2.y;
            mContactPoints[mNbContactPoints].r2.z = p2.z - x2.z;
            mContactPoints[mNbContactPoints].penetrationDepth = externalContact.getPenetrationDepth();
            mContactPoints[mNbContactPoints].isRestingContact = externalContact.getIsRestingContact();
            externalContact.setIsRestingContact(true);
            mContactPoints[mNbContactPoints].penetrationImpulse = externalContact.getPenetrationImpulse();
            mContactPoints[mNbContactPoints].penetrationSplitImpulse = 0.0;

            mContactConstraints[mNbContactManifolds].frictionPointBody1.x += p1.x;
            mContactConstraints[mNbContactManifolds].frictionPointBody1.y += p1.y;
            mContactConstraints[mNbContactManifolds].frictionPointBody1.z += p1.z;
            mContactConstraints[mNbContactManifolds].frictionPointBody2.x += p2.x;
            mContactConstraints[mNbContactManifolds].frictionPointBody2.y += p2.y;
            mContactConstraints[mNbContactManifolds].frictionPointBody2.z += p2.z;

            // Compute the velocity difference
            // deltaV = v2 + w2.cross(mContactPoints[mNbContactPoints].r2) - v1 - w1.cross(mContactPoints[mNbContactPoints].r1);
            Vector3 deltaV(v2.x + w2.y * mContactPoints[mNbContactPoints].r2.z - w2.z * mContactPoints[mNbContactPoints].r2.y
                           - v1.x - w1.y * mContactPoints[mNbContactPoints].r1.z + w1.z * mContactPoints[mNbContactPoints].r1.y,
                           v2.y + w2.z * mContactPoints[mNbContactPoints].r2.x - w2.x * mContactPoints[mNbContactPoints].r2.z
                           - v1.y - w1.z * mContactPoints[mNbContactPoints].r1.x + w1.x * mContactPoints[mNbContactPoints].r1.z,
                           v2.z + w2.x * mContactPoints[mNbContactPoints].r2.y - w2.y * mContactPoints[mNbContactPoints].r2.x
                           - v1.z - w1.x * mContactPoints[mNbContactPoints].r1.y + w1.y * mContactPoints[mNbContactPoints].r1.x);

            // r1CrossN = mContactPoints[mNbContactPoints].r1.cross(mContactPoints[mNbContactPoints].normal);
            Vector3 r1CrossN(mContactPoints[mNbContactPoints].r1.y * mContactPoints[mNbContactPoints].normal.z -
                             mContactPoints[mNbContactPoints].r1.z * mContactPoints[mNbContactPoints].normal.y,
                             mContactPoints[mNbContactPoints].r1.z * mContactPoints[mNbContactPoints].normal.x -
                             mContactPoints[mNbContactPoints].r1.x * mContactPoints[mNbContactPoints].normal.z,
                             mContactPoints[mNbContactPoints].r1.x * mContactPoints[mNbContactPoints].normal.y -
                             mContactPoints[mNbContactPoints].r1.y * mContactPoints[mNbContactPoints].normal.x);
            // r2CrossN = mContactPoints[mNbContactPoints].r2.cross(mContactPoints[mNbContactPoints].normal);
            Vector3 r2CrossN(mContactPoints[mNbContactPoints].r2.y * mContactPoints[mNbContactPoints].normal.z -
                             mContactPoints[mNbContactPoints].r2.z * mContactPoints[mNbContactPoints].normal.y,
                             mContactPoints[mNbContactPoints].r2.z * mContactPoints[mNbContactPoints].normal.x -
                             mContactPoints[mNbContactPoints].r2.x * mContactPoints[mNbContactPoints].normal.z,
                             mContactPoints[mNbContactPoints].r2.x * mContactPoints[mNbContactPoints].normal.y -
                             mContactPoints[mNbContactPoints].r2.y * mContactPoints[mNbContactPoints].normal.x);

            mContactPoints[mNbContactPoints].i1TimesR1CrossN = mContactConstraints[mNbContactManifolds].inverseInertiaTensorBody1 * r1CrossN;
            mContactPoints[mNbContactPoints].i2TimesR2CrossN = mContactConstraints[mNbContactManifolds].inverseInertiaTensorBody2 * r2CrossN;

            // Compute the inverse mass matrix K for the penetration constraint
            decimal massPenetration = mContactConstraints[mNbContactManifolds].massInverseBody1 + mContactConstraints[mNbContactManifolds].massInverseBody2 +
                    ((mContactPoints[mNbContactPoints].i1TimesR1CrossN).cross(mContactPoints[mNbContactPoints].r1)).dot(mContactPoints[mNbContactPoints].normal) +
                    ((mContactPoints[mNbContactPoints].i2TimesR2CrossN).cross(mContactPoints[mNbContactPoints].r2)).dot(mContactPoints[mNbContactPoints].normal);
            mContactPoints[mNbContactPoints].inversePenetrationMass = massPenetration > decimal(0.0) ? decimal(1.0) / massPenetration : decimal(0.0);

            // Compute the restitution velocity bias "b". We compute this here instead
            // of inside the solve() method because we need to use the velocity difference
            // at the beginning of the contact. Note that if it is a resting contact (normal
            // velocity bellow a given threshold), we do not add a restitution velocity bias
            mContactPoints[mNbContactPoints].restitutionBias = 0.0;
            // deltaVDotN = deltaV.dot(mContactPoints[mNbContactPoints].normal);
            decimal deltaVDotN = deltaV.x * mContactPoints[mNbContactPoints].normal.x +
                                 deltaV.y * mContactPoints[mNbContactPoints].normal.y +
                                 deltaV.z * mContactPoints[mNbContactPoints].normal.z;
            const decimal restitutionFactor = computeMixedRestitutionFactor(mColliderComponents.mMaterials[collider1Index], mColliderComponents.mMaterials[collider2Index]);
            if (deltaVDotN < -mRestitutionVelocityThreshold) {
                mContactPoints[mNbContactPoints].restitutionBias = restitutionFactor * deltaVDotN;
            }

            mContactConstraints[mNbContactManifolds].normal.x += mContactPoints[mNbContactPoints].normal.x;
            mContactConstraints[mNbContactManifolds].normal.y += mContactPoints[mNbContactPoints].normal.y;
            mContactConstraints[mNbContactManifolds].normal.z += mContactPoints[mNbContactPoints].normal.z;

            mNbContactPoints++;
        }

        mContactConstraints[mNbContactManifolds].frictionPointBody1 /= static_cast<decimal>(mContactConstraints[mNbContactManifolds].nbContacts);
        mContactConstraints[mNbContactManifolds].frictionPointBody2 /= static_cast<decimal>(mContactConstraints[mNbContactManifolds].nbContacts);
        mContactConstraints[mNbContactManifolds].r1Friction.x = mContactConstraints[mNbContactManifolds].frictionPointBody1.x - x1.x;
        mContactConstraints[mNbContactManifolds].r1Friction.y = mContactConstraints[mNbContactManifolds].frictionPointBody1.y - x1.y;
        mContactConstraints[mNbContactManifolds].r1Friction.z = mContactConstraints[mNbContactManifolds].frictionPointBody1.z - x1.z;
        mContactConstraints[mNbContactManifolds].r2Friction.x = mContactConstraints[mNbContactManifolds].frictionPointBody2.x - x2.x;
        mContactConstraints[mNbContactManifolds].r2Friction.y = mContactConstraints[mNbContactManifolds].frictionPointBody2.y - x2.y;
        mContactConstraints[mNbContactManifolds].r2Friction.z = mContactConstraints[mNbContactManifolds].frictionPointBody2.z - x2.z;
        mContactConstraints[mNbContactManifolds].oldFrictionVector1 = externalManifold.frictionVector1;
        mContactConstraints[mNbContactManifolds].oldFrictionVector2 = externalManifold.frictionVector2;

        // Initialize the accumulated impulses with the previous step accumulated impulses
        mContactConstraints[mNbContactManifolds].friction1Impulse = externalManifold.frictionImpulse1;
        mContactConstraints[mNbContactManifolds].friction2Impulse = externalManifold.frictionImpulse2;
        mContactConstraints[mNbContactManifolds].frictionTwistImpulse = externalManifold.frictionTwistImpulse;

        mContactConstraints[mNbContactManifolds].normal.normalize();

        // deltaVFrictionPoint = v2 + w2.cross(mContactConstraints[mNbContactManifolds].r2Friction) -
        //                              v1 - w1.cross(mContactConstraints[mNbContactManifolds].r1Friction);
        Vector3 deltaVFrictionPoint(v2.x + w2.y * mContactConstraints[mNbContactManifolds].r2Friction.z -
                                    w2.z * mContactConstraints[mNbContactManifolds].r2Friction.y -
                                      v1.x - w1.y * mContactConstraints[mNbContactManifolds].r1Friction.z +
                                      w1.z * mContactConstraints[mNbContactManifolds].r1Friction.y,
                                   v2.y + w2.z * mContactConstraints[mNbContactManifolds].r2Friction.x -
                                    w2.x * mContactConstraints[mNbContactManifolds].r2Friction.z -
                                      v1.y - w1.z * mContactConstraints[mNbContactManifolds].r1Friction.x +
                                      w1.x * mContactConstraints[mNbContactManifolds].r1Friction.z,
                                   v2.z + w2.x * mContactConstraints[mNbContactManifolds].r2Friction.y -
                                    w2.y * mContactConstraints[mNbContactManifolds].r2Friction.x -
                                      v1.z - w1.x * mContactConstraints[mNbContactManifolds].r1Friction.y +
                                      w1.y * mContactConstraints[mNbContactManifolds].r1Friction.x);

        // Compute the friction vectors
        computeFrictionVectors(deltaVFrictionPoint, mContactConstraints[mNbContactManifolds]);

        // Compute the inverse mass matrix K for the friction constraints at the center of
        // the contact manifold
        mContactConstraints[mNbContactManifolds].r1CrossT1 = mContactConstraints[mNbContactManifolds].r1Friction.cross(mContactConstraints[mNbContactManifolds].frictionVector1);
        mContactConstraints[mNbContactManifolds].r1CrossT2 = mContactConstraints[mNbContactManifolds].r1Friction.cross(mContactConstraints[mNbContactManifolds].frictionVector2);
        mContactConstraints[mNbContactManifolds].r2CrossT1 = mContactConstraints[mNbContactManifolds].r2Friction.cross(mContactConstraints[mNbContactManifolds].frictionVector1);
        mContactConstraints[mNbContactManifolds].r2CrossT2 = mContactConstraints[mNbContactManifolds].r2Friction.cross(mContactConstraints[mNbContactManifolds].frictionVector2);
        decimal friction1Mass = mContactConstraints[mNbContactManifolds].massInverseBody1 + mContactConstraints[mNbContactManifolds].massInverseBody2 +
                                ((mContactConstraints[mNbContactManifolds].inverseInertiaTensorBody1 * mContactConstraints[mNbContactManifolds].r1CrossT1).cross(mContactConstraints[mNbContactManifolds].r1Friction)).dot(
                                mContactConstraints[mNbContactManifolds].frictionVector1) +
                                ((mContactConstraints[mNbContactManifolds].inverseInertiaTensorBody2 * mContactConstraints[mNbContactManifolds].r2CrossT1).cross(mContactConstraints[mNbContactManifolds].r2Friction)).dot(
                                mContactConstraints[mNbContactManifolds].frictionVector1);
        decimal friction2Mass = mContactConstraints[mNbContactManifolds].massInverseBody1 + mContactConstraints[mNbContactManifolds].massInverseBody2 +
                                ((mContactConstraints[mNbContactManifolds].inverseInertiaTensorBody1 * mContactConstraints[mNbContactManifolds].r1CrossT2).cross(mContactConstraints[mNbContactManifolds].r1Friction)).dot(
                                mContactConstraints[mNbContactManifolds].frictionVector2) +
                                ((mContactConstraints[mNbContactManifolds].inverseInertiaTensorBody2 * mContactConstraints[mNbContactManifolds].r2CrossT2).cross(mContactConstraints[mNbContactManifolds].r2Friction)).dot(
                                mContactConstraints[mNbContactManifolds].frictionVector2);
        decimal frictionTwistMass = mContactConstraints[mNbContactManifolds].normal.dot(mContactConstraints[mNbContactManifolds].inverseInertiaTensorBody1 *
                                       mContactConstraints[mNbContactManifolds].normal) +
                                    mContactConstraints[mNbContactManifolds].normal.dot(mContactConstraints[mNbContactManifolds].inverseInertiaTensorBody2 *
                                       mContactConstraints[mNbContactManifolds].normal);
        mContactConstraints[mNbContactManifolds].inverseFriction1Mass = friction1Mass > decimal(0.0) ? decimal(1.0) / friction1Mass : decimal(0.0);
        mContactConstraints[mNbContactManifolds].inverseFriction2Mass = friction2Mass > decimal(0.0) ? decimal(1.0) / friction2Mass : decimal(0.0);
        mContactConstraints[mNbContactManifolds].inverseTwistFrictionMass = frictionTwistMass > decimal(0.0) ? decimal(1.0) / frictionTwistMass : decimal(0.0);

        mNbContactManifolds++;
    }
}

// Warm start the solver.
/// For each constraint, we apply the previous impulse (from the previous step)
/// at the beginning. With this technique, we will converge faster towards
/// the solution of the linear system
void ContactSolverSystem::warmStart() {

    RP3D_PROFILE("ContactSolver::warmStart()", mProfiler);

    uint32 contactPointIndex = 0;

    // For each constraint
    for (uint32 c=0; c<mNbContactManifolds; c++) {

        bool atLeastOneRestingContactPoint = false;

        for (short int i=0; i<mContactConstraints[c].nbContacts; i++) {

            // If it is not a new contact (this contact was already existing at last time step)
            if (mContactPoints[contactPointIndex].isRestingContact) {

                const uint32 rigidBody1Index = mContactConstraints[c].rigidBodyComponentIndexBody1;
                const uint32 rigidBody2Index = mContactConstraints[c].rigidBodyComponentIndexBody2;

                atLeastOneRestingContactPoint = true;

                // --------- Penetration --------- //

                // Update the velocities of the body 1 by applying the impulse P
                Vector3 impulsePenetration(mContactPoints[contactPointIndex].normal.x * mContactPoints[contactPointIndex].penetrationImpulse,
                                           mContactPoints[contactPointIndex].normal.y * mContactPoints[contactPointIndex].penetrationImpulse,
                                           mContactPoints[contactPointIndex].normal.z * mContactPoints[contactPointIndex].penetrationImpulse);
                mRigidBodyComponents.mConstrainedLinearVelocities[rigidBody1Index].x -= mContactConstraints[c].massInverseBody1 * impulsePenetration.x * mContactConstraints[c].linearLockAxisFactorBody1.x;
                mRigidBodyComponents.mConstrainedLinearVelocities[rigidBody1Index].y -= mContactConstraints[c].massInverseBody1 * impulsePenetration.y * mContactConstraints[c].linearLockAxisFactorBody1.y;
                mRigidBodyComponents.mConstrainedLinearVelocities[rigidBody1Index].z -= mContactConstraints[c].massInverseBody1 * impulsePenetration.z * mContactConstraints[c].linearLockAxisFactorBody1.z;

                mRigidBodyComponents.mConstrainedAngularVelocities[rigidBody1Index].x -= mContactPoints[contactPointIndex].i1TimesR1CrossN.x * mContactConstraints[c].angularLockAxisFactorBody1.x * mContactPoints[contactPointIndex].penetrationImpulse;
                mRigidBodyComponents.mConstrainedAngularVelocities[rigidBody1Index].y -= mContactPoints[contactPointIndex].i1TimesR1CrossN.y * mContactConstraints[c].angularLockAxisFactorBody1.y * mContactPoints[contactPointIndex].penetrationImpulse;
                mRigidBodyComponents.mConstrainedAngularVelocities[rigidBody1Index].z -= mContactPoints[contactPointIndex].i1TimesR1CrossN.z * mContactConstraints[c].angularLockAxisFactorBody1.z * mContactPoints[contactPointIndex].penetrationImpulse;

                // Update the velocities of the body 2 by applying the impulse P
                mRigidBodyComponents.mConstrainedLinearVelocities[rigidBody2Index].x += mContactConstraints[c].massInverseBody2 * impulsePenetration.x * mContactConstraints[c].linearLockAxisFactorBody2.x;
                mRigidBodyComponents.mConstrainedLinearVelocities[rigidBody2Index].y += mContactConstraints[c].massInverseBody2 * impulsePenetration.y * mContactConstraints[c].linearLockAxisFactorBody2.y;
                mRigidBodyComponents.mConstrainedLinearVelocities[rigidBody2Index].z += mContactConstraints[c].massInverseBody2 * impulsePenetration.z * mContactConstraints[c].linearLockAxisFactorBody2.z;

                mRigidBodyComponents.mConstrainedAngularVelocities[rigidBody2Index].x += mContactPoints[contactPointIndex].i2TimesR2CrossN.x * mContactConstraints[c].angularLockAxisFactorBody2.x * mContactPoints[contactPointIndex].penetrationImpulse;
                mRigidBodyComponents.mConstrainedAngularVelocities[rigidBody2Index].y += mContactPoints[contactPointIndex].i2TimesR2CrossN.y * mContactConstraints[c].angularLockAxisFactorBody2.y * mContactPoints[contactPointIndex].penetrationImpulse;
                mRigidBodyComponents.mConstrainedAngularVelocities[rigidBody2Index].z += mContactPoints[contactPointIndex].i2TimesR2CrossN.z * mContactConstraints[c].angularLockAxisFactorBody2.z * mContactPoints[contactPointIndex].penetrationImpulse;
            }
            else {  // If it is a new contact point

                // Initialize the accumulated impulses to zero
                mContactPoints[contactPointIndex].penetrationImpulse = 0.0;
            }

            contactPointIndex++;
        }

        // If we solve the friction constraints at the center of the contact manifold and there is
        // at least one resting contact point in the contact manifold
        if (atLeastOneRestingContactPoint) {

            // Project the old friction impulses (with old friction vectors) into the new friction
            // vectors to get the new friction impulses
            Vector3 oldFrictionImpulse(mContactConstraints[c].friction1Impulse * mContactConstraints[c].oldFrictionVector1.x +
                                         mContactConstraints[c].friction2Impulse * mContactConstraints[c].oldFrictionVector2.x,
                                       mContactConstraints[c].friction1Impulse * mContactConstraints[c].oldFrictionVector1.y +
                                         mContactConstraints[c].friction2Impulse * mContactConstraints[c].oldFrictionVector2.y,
                                       mContactConstraints[c].friction1Impulse * mContactConstraints[c].oldFrictionVector1.z +
                                         mContactConstraints[c].friction2Impulse * mContactConstraints[c].oldFrictionVector2.z);
            mContactConstraints[c].friction1Impulse = oldFrictionImpulse.dot(mContactConstraints[c].frictionVector1);
            mContactConstraints[c].friction2Impulse = oldFrictionImpulse.dot(mContactConstraints[c].frictionVector2);

            // ------ First friction constraint at the center of the contact manifold ------ //

            // Compute the impulse P = J^T * lambda
            Vector3 angularImpulseBody1(-mContactConstraints[c].r1CrossT1.x * mContactConstraints[c].friction1Impulse,
                                        -mContactConstraints[c].r1CrossT1.y * mContactConstraints[c].friction1Impulse,
                                        -mContactConstraints[c].r1CrossT1.z * mContactConstraints[c].friction1Impulse);
            Vector3 linearImpulseBody2(mContactConstraints[c].frictionVector1.x * mContactConstraints[c].friction1Impulse,
                                       mContactConstraints[c].frictionVector1.y * mContactConstraints[c].friction1Impulse,
                                       mContactConstraints[c].frictionVector1.z * mContactConstraints[c].friction1Impulse);
            Vector3 angularImpulseBody2(mContactConstraints[c].r2CrossT1.x * mContactConstraints[c].friction1Impulse,
                                        mContactConstraints[c].r2CrossT1.y * mContactConstraints[c].friction1Impulse,
                                        mContactConstraints[c].r2CrossT1.z * mContactConstraints[c].friction1Impulse);

            const uint32 rigidBody1Index = mContactConstraints[c].rigidBodyComponentIndexBody1;
            const uint32 rigidBody2Index = mContactConstraints[c].rigidBodyComponentIndexBody2;

            // Update the velocities of the body 1 by applying the impulse P
            mRigidBodyComponents.mConstrainedLinearVelocities[rigidBody1Index] -= mContactConstraints[c].massInverseBody1 * linearImpulseBody2 * mContactConstraints[c].linearLockAxisFactorBody1;
            mRigidBodyComponents.mConstrainedAngularVelocities[rigidBody1Index] += mContactConstraints[c].angularLockAxisFactorBody1 * (mContactConstraints[c].inverseInertiaTensorBody1 * angularImpulseBody1);

            // Update the velocities of the body 1 by applying the impulse P
            mRigidBodyComponents.mConstrainedLinearVelocities[rigidBody2Index] += mContactConstraints[c].massInverseBody2 * linearImpulseBody2 * mContactConstraints[c].linearLockAxisFactorBody2;
            mRigidBodyComponents.mConstrainedAngularVelocities[rigidBody2Index] += mContactConstraints[c].angularLockAxisFactorBody2 * (mContactConstraints[c].inverseInertiaTensorBody2 * angularImpulseBody2);

            // ------ Second friction constraint at the center of the contact manifold ----- //

            // Compute the impulse P = J^T * lambda
            angularImpulseBody1.x = -mContactConstraints[c].r1CrossT2.x * mContactConstraints[c].friction2Impulse;
            angularImpulseBody1.y = -mContactConstraints[c].r1CrossT2.y * mContactConstraints[c].friction2Impulse;
            angularImpulseBody1.z = -mContactConstraints[c].r1CrossT2.z * mContactConstraints[c].friction2Impulse;
            linearImpulseBody2.x = mContactConstraints[c].frictionVector2.x * mContactConstraints[c].friction2Impulse;
            linearImpulseBody2.y = mContactConstraints[c].frictionVector2.y * mContactConstraints[c].friction2Impulse;
            linearImpulseBody2.z = mContactConstraints[c].frictionVector2.z * mContactConstraints[c].friction2Impulse;
            angularImpulseBody2.x = mContactConstraints[c].r2CrossT2.x * mContactConstraints[c].friction2Impulse;
            angularImpulseBody2.y = mContactConstraints[c].r2CrossT2.y * mContactConstraints[c].friction2Impulse;
            angularImpulseBody2.z = mContactConstraints[c].r2CrossT2.z * mContactConstraints[c].friction2Impulse;

            // Update the velocities of the body 1 by applying the impulse P
            mRigidBodyComponents.mConstrainedLinearVelocities[rigidBody1Index].x -= mContactConstraints[c].massInverseBody1 * linearImpulseBody2.x * mContactConstraints[c].linearLockAxisFactorBody1.x;
            mRigidBodyComponents.mConstrainedLinearVelocities[rigidBody1Index].y -= mContactConstraints[c].massInverseBody1 * linearImpulseBody2.y * mContactConstraints[c].linearLockAxisFactorBody1.y;
            mRigidBodyComponents.mConstrainedLinearVelocities[rigidBody1Index].z -= mContactConstraints[c].massInverseBody1 * linearImpulseBody2.z * mContactConstraints[c].linearLockAxisFactorBody1.z;

            mRigidBodyComponents.mConstrainedAngularVelocities[rigidBody1Index] += mContactConstraints[c].angularLockAxisFactorBody1 * (mContactConstraints[c].inverseInertiaTensorBody1 * angularImpulseBody1);

            // Update the velocities of the body 2 by applying the impulse P
            mRigidBodyComponents.mConstrainedLinearVelocities[rigidBody2Index].x += mContactConstraints[c].massInverseBody2 * linearImpulseBody2.x * mContactConstraints[c].linearLockAxisFactorBody2.x;
            mRigidBodyComponents.mConstrainedLinearVelocities[rigidBody2Index].y += mContactConstraints[c].massInverseBody2 * linearImpulseBody2.y * mContactConstraints[c].linearLockAxisFactorBody2.y;
            mRigidBodyComponents.mConstrainedLinearVelocities[rigidBody2Index].z += mContactConstraints[c].massInverseBody2 * linearImpulseBody2.z * mContactConstraints[c].linearLockAxisFactorBody2.z;

            mRigidBodyComponents.mConstrainedAngularVelocities[rigidBody2Index] += mContactConstraints[c].angularLockAxisFactorBody2 * (mContactConstraints[c].inverseInertiaTensorBody2 * angularImpulseBody2);

            // ------ Twist friction constraint at the center of the contact manifold ------ //

            // Compute the impulse P = J^T * lambda
            angularImpulseBody1.x = -mContactConstraints[c].normal.x * mContactConstraints[c].frictionTwistImpulse;
            angularImpulseBody1.y = -mContactConstraints[c].normal.y * mContactConstraints[c].frictionTwistImpulse;
            angularImpulseBody1.z = -mContactConstraints[c].normal.z * mContactConstraints[c].frictionTwistImpulse;

            angularImpulseBody2.x = mContactConstraints[c].normal.x * mContactConstraints[c].frictionTwistImpulse;
            angularImpulseBody2.y = mContactConstraints[c].normal.y * mContactConstraints[c].frictionTwistImpulse;
            angularImpulseBody2.z = mContactConstraints[c].normal.z * mContactConstraints[c].frictionTwistImpulse;

            // Update the velocities of the body 1 by applying the impulse P
            mRigidBodyComponents.mConstrainedAngularVelocities[rigidBody1Index] += mContactConstraints[c].angularLockAxisFactorBody1 * (mContactConstraints[c].inverseInertiaTensorBody1 *  angularImpulseBody1);

            // Update the velocities of the body 2 by applying the impulse P
            mRigidBodyComponents.mConstrainedAngularVelocities[rigidBody2Index] += mContactConstraints[c].angularLockAxisFactorBody2 * (mContactConstraints[c].inverseInertiaTensorBody2 * angularImpulseBody2);

            // Update the velocities of the body 1 by applying the impulse P
            mRigidBodyComponents.mConstrainedAngularVelocities[rigidBody1Index] -= mContactConstraints[c].angularLockAxisFactorBody1 * (mContactConstraints[c].inverseInertiaTensorBody1 * angularImpulseBody2);

            // Update the velocities of the body 1 by applying the impulse P
            mRigidBodyComponents.mConstrainedAngularVelocities[rigidBody2Index] += mContactConstraints[c].angularLockAxisFactorBody2 * (mContactConstraints[c].inverseInertiaTensorBody2 * angularImpulseBody2);
        }
        else {  // If it is a new contact manifold

            // Initialize the accumulated impulses to zero
            mContactConstraints[c].friction1Impulse = 0.0;
            mContactConstraints[c].friction2Impulse = 0.0;
            mContactConstraints[c].frictionTwistImpulse = 0.0;
        }
    }
}

// Solve the contacts
void ContactSolverSystem::solve() {

    RP3D_PROFILE("ContactSolverSystem::solve()", mProfiler);

    decimal deltaLambda;
    decimal lambdaTemp;
    uint32 contactPointIndex = 0;

    const decimal beta = mIsSplitImpulseActive ? BETA_SPLIT_IMPULSE : BETA;

    // For each contact manifold
    for (uint32 c=0; c<mNbContactManifolds; c++) {

        decimal sumPenetrationImpulse = 0.0;

        const uint32 rigidBody1Index = mContactConstraints[c].rigidBodyComponentIndexBody1;
        const uint32 rigidBody2Index = mContactConstraints[c].rigidBodyComponentIndexBody2;

        // Get the constrained velocities
        const Vector3& v1 = mRigidBodyComponents.mConstrainedLinearVelocities[rigidBody1Index];
        const Vector3& w1 = mRigidBodyComponents.mConstrainedAngularVelocities[rigidBody1Index];
        const Vector3& v2 = mRigidBodyComponents.mConstrainedLinearVelocities[rigidBody2Index];
        const Vector3& w2 = mRigidBodyComponents.mConstrainedAngularVelocities[rigidBody2Index];

        for (short int i=0; i<mContactConstraints[c].nbContacts; i++) {

            // --------- Penetration --------- //

            // Compute J*v
            //Vector3 deltaV = v2 + w2.cross(mContactPoints[contactPointIndex].r2) - v1 - w1.cross(mContactPoints[contactPointIndex].r1);
            Vector3 deltaV(v2.x + w2.y * mContactPoints[contactPointIndex].r2.z - w2.z * mContactPoints[contactPointIndex].r2.y - v1.x -
                           w1.y * mContactPoints[contactPointIndex].r1.z + w1.z * mContactPoints[contactPointIndex].r1.y,
                           v2.y + w2.z * mContactPoints[contactPointIndex].r2.x - w2.x * mContactPoints[contactPointIndex].r2.z - v1.y -
                           w1.z * mContactPoints[contactPointIndex].r1.x + w1.x * mContactPoints[contactPointIndex].r1.z,
                           v2.z + w2.x * mContactPoints[contactPointIndex].r2.y - w2.y * mContactPoints[contactPointIndex].r2.x - v1.z -
                           w1.x * mContactPoints[contactPointIndex].r1.y + w1.y * mContactPoints[contactPointIndex].r1.x);
            decimal deltaVDotN = deltaV.x * mContactPoints[contactPointIndex].normal.x + deltaV.y * mContactPoints[contactPointIndex].normal.y +
                                 deltaV.z * mContactPoints[contactPointIndex].normal.z;
            decimal Jv = deltaVDotN;

            // Compute the bias "b" of the constraint
            decimal biasPenetrationDepth = 0.0;
            if (mContactPoints[contactPointIndex].penetrationDepth > SLOP) {
                biasPenetrationDepth = -(beta/mTimeStep) * std::max(0.0f, float(mContactPoints[contactPointIndex].penetrationDepth - SLOP));
            }
            decimal b = biasPenetrationDepth + mContactPoints[contactPointIndex].restitutionBias;

            // Compute the Lagrange multiplier lambda
            if (mIsSplitImpulseActive) {
                deltaLambda = - (Jv + mContactPoints[contactPointIndex].restitutionBias) *
                        mContactPoints[contactPointIndex].inversePenetrationMass;
            }
            else {
                deltaLambda = - (Jv + b) * mContactPoints[contactPointIndex].inversePenetrationMass;
            }
            lambdaTemp = mContactPoints[contactPointIndex].penetrationImpulse;
            mContactPoints[contactPointIndex].penetrationImpulse = std::max(mContactPoints[contactPointIndex].penetrationImpulse +
                                                       deltaLambda, decimal(0.0));
            deltaLambda = mContactPoints[contactPointIndex].penetrationImpulse - lambdaTemp;

            Vector3 linearImpulse(mContactPoints[contactPointIndex].normal.x * deltaLambda,
                                  mContactPoints[contactPointIndex].normal.y * deltaLambda,
                                  mContactPoints[contactPointIndex].normal.z * deltaLambda);

            // Update the velocities of the body 1 by applying the impulse P
            mRigidBodyComponents.mConstrainedLinearVelocities[rigidBody1Index].x -= mContactConstraints[c].massInverseBody1 * linearImpulse.x * mContactConstraints[c].linearLockAxisFactorBody1.x;
            mRigidBodyComponents.mConstrainedLinearVelocities[rigidBody1Index].y -= mContactConstraints[c].massInverseBody1 * linearImpulse.y * mContactConstraints[c].linearLockAxisFactorBody1.y;
            mRigidBodyComponents.mConstrainedLinearVelocities[rigidBody1Index].z -= mContactConstraints[c].massInverseBody1 * linearImpulse.z * mContactConstraints[c].linearLockAxisFactorBody1.z;

            mRigidBodyComponents.mConstrainedAngularVelocities[rigidBody1Index].x -= mContactPoints[contactPointIndex].i1TimesR1CrossN.x * mContactConstraints[c].angularLockAxisFactorBody1.x * deltaLambda;
            mRigidBodyComponents.mConstrainedAngularVelocities[rigidBody1Index].y -= mContactPoints[contactPointIndex].i1TimesR1CrossN.y * mContactConstraints[c].angularLockAxisFactorBody1.y * deltaLambda;
            mRigidBodyComponents.mConstrainedAngularVelocities[rigidBody1Index].z -= mContactPoints[contactPointIndex].i1TimesR1CrossN.z * mContactConstraints[c].angularLockAxisFactorBody1.z * deltaLambda;

            // Update the velocities of the body 2 by applying the impulse P
            mRigidBodyComponents.mConstrainedLinearVelocities[rigidBody2Index].x += mContactConstraints[c].massInverseBody2 * linearImpulse.x * mContactConstraints[c].linearLockAxisFactorBody2.x;
            mRigidBodyComponents.mConstrainedLinearVelocities[rigidBody2Index].y += mContactConstraints[c].massInverseBody2 * linearImpulse.y * mContactConstraints[c].linearLockAxisFactorBody2.y;
            mRigidBodyComponents.mConstrainedLinearVelocities[rigidBody2Index].z += mContactConstraints[c].massInverseBody2 * linearImpulse.z * mContactConstraints[c].linearLockAxisFactorBody2.z;

            mRigidBodyComponents.mConstrainedAngularVelocities[rigidBody2Index].x += mContactPoints[contactPointIndex].i2TimesR2CrossN.x * mContactConstraints[c].angularLockAxisFactorBody2.x * deltaLambda;
            mRigidBodyComponents.mConstrainedAngularVelocities[rigidBody2Index].y += mContactPoints[contactPointIndex].i2TimesR2CrossN.y * mContactConstraints[c].angularLockAxisFactorBody2.y * deltaLambda;
            mRigidBodyComponents.mConstrainedAngularVelocities[rigidBody2Index].z += mContactPoints[contactPointIndex].i2TimesR2CrossN.z * mContactConstraints[c].angularLockAxisFactorBody2.z * deltaLambda;

            sumPenetrationImpulse += mContactPoints[contactPointIndex].penetrationImpulse;

            // If the split impulse position correction is active
            if (mIsSplitImpulseActive) {

                // Split impulse (position correction)
                const Vector3& v1Split = mRigidBodyComponents.mSplitLinearVelocities[rigidBody1Index];
                const Vector3& w1Split = mRigidBodyComponents.mSplitAngularVelocities[rigidBody1Index];
                const Vector3& v2Split = mRigidBodyComponents.mSplitLinearVelocities[rigidBody2Index];
                const Vector3& w2Split = mRigidBodyComponents.mSplitAngularVelocities[rigidBody2Index];

                //Vector3 deltaVSplit = v2Split + w2Split.cross(mContactPoints[contactPointIndex].r2) - v1Split - w1Split.cross(mContactPoints[contactPointIndex].r1);
                Vector3 deltaVSplit(v2Split.x + w2Split.y * mContactPoints[contactPointIndex].r2.z - w2Split.z * mContactPoints[contactPointIndex].r2.y - v1Split.x -
                                    w1Split.y * mContactPoints[contactPointIndex].r1.z + w1Split.z * mContactPoints[contactPointIndex].r1.y,
                                    v2Split.y + w2Split.z * mContactPoints[contactPointIndex].r2.x - w2Split.x * mContactPoints[contactPointIndex].r2.z - v1Split.y -
                                    w1Split.z * mContactPoints[contactPointIndex].r1.x + w1Split.x * mContactPoints[contactPointIndex].r1.z,
                                    v2Split.z + w2Split.x * mContactPoints[contactPointIndex].r2.y - w2Split.y * mContactPoints[contactPointIndex].r2.x - v1Split.z -
                                    w1Split.x * mContactPoints[contactPointIndex].r1.y + w1Split.y * mContactPoints[contactPointIndex].r1.x);
                decimal JvSplit = deltaVSplit.x * mContactPoints[contactPointIndex].normal.x +
                                  deltaVSplit.y * mContactPoints[contactPointIndex].normal.y +
                                  deltaVSplit.z * mContactPoints[contactPointIndex].normal.z;
                decimal deltaLambdaSplit = - (JvSplit + biasPenetrationDepth) *
                        mContactPoints[contactPointIndex].inversePenetrationMass;
                decimal lambdaTempSplit = mContactPoints[contactPointIndex].penetrationSplitImpulse;
                mContactPoints[contactPointIndex].penetrationSplitImpulse = std::max(
                            mContactPoints[contactPointIndex].penetrationSplitImpulse +
                            deltaLambdaSplit, decimal(0.0));
                deltaLambdaSplit = mContactPoints[contactPointIndex].penetrationSplitImpulse - lambdaTempSplit;

                Vector3 linearImpulse(mContactPoints[contactPointIndex].normal.x * deltaLambdaSplit,
                                      mContactPoints[contactPointIndex].normal.y * deltaLambdaSplit,
                                      mContactPoints[contactPointIndex].normal.z * deltaLambdaSplit);

                // Update the velocities of the body 1 by applying the impulse P
                mRigidBodyComponents.mSplitLinearVelocities[rigidBody1Index].x -= mContactConstraints[c].massInverseBody1 * linearImpulse.x * mContactConstraints[c].linearLockAxisFactorBody1.x;
                mRigidBodyComponents.mSplitLinearVelocities[rigidBody1Index].y -= mContactConstraints[c].massInverseBody1 * linearImpulse.y * mContactConstraints[c].linearLockAxisFactorBody1.y;
                mRigidBodyComponents.mSplitLinearVelocities[rigidBody1Index].z -= mContactConstraints[c].massInverseBody1 * linearImpulse.z * mContactConstraints[c].linearLockAxisFactorBody1.z;

                mRigidBodyComponents.mSplitAngularVelocities[rigidBody1Index].x -= mContactPoints[contactPointIndex].i1TimesR1CrossN.x * mContactConstraints[c].angularLockAxisFactorBody1.x * deltaLambdaSplit;
                mRigidBodyComponents.mSplitAngularVelocities[rigidBody1Index].y -= mContactPoints[contactPointIndex].i1TimesR1CrossN.y * mContactConstraints[c].angularLockAxisFactorBody1.y * deltaLambdaSplit;
                mRigidBodyComponents.mSplitAngularVelocities[rigidBody1Index].z -= mContactPoints[contactPointIndex].i1TimesR1CrossN.z * mContactConstraints[c].angularLockAxisFactorBody1.z * deltaLambdaSplit;

                // Update the velocities of the body 1 by applying the impulse P
                mRigidBodyComponents.mSplitLinearVelocities[rigidBody2Index].x += mContactConstraints[c].massInverseBody2 * linearImpulse.x * mContactConstraints[c].linearLockAxisFactorBody2.x;
                mRigidBodyComponents.mSplitLinearVelocities[rigidBody2Index].y += mContactConstraints[c].massInverseBody2 * linearImpulse.y * mContactConstraints[c].linearLockAxisFactorBody2.y;
                mRigidBodyComponents.mSplitLinearVelocities[rigidBody2Index].z += mContactConstraints[c].massInverseBody2 * linearImpulse.z * mContactConstraints[c].linearLockAxisFactorBody2.z;

                mRigidBodyComponents.mSplitAngularVelocities[rigidBody2Index].x += mContactPoints[contactPointIndex].i2TimesR2CrossN.x * mContactConstraints[c].angularLockAxisFactorBody2.x * deltaLambdaSplit;
                mRigidBodyComponents.mSplitAngularVelocities[rigidBody2Index].y += mContactPoints[contactPointIndex].i2TimesR2CrossN.y * mContactConstraints[c].angularLockAxisFactorBody2.y * deltaLambdaSplit;
                mRigidBodyComponents.mSplitAngularVelocities[rigidBody2Index].z += mContactPoints[contactPointIndex].i2TimesR2CrossN.z * mContactConstraints[c].angularLockAxisFactorBody2.z * deltaLambdaSplit;
            }

            contactPointIndex++;
        }

        // ------ First friction constraint at the center of the contact manifold ------ //

        // Compute J*v
        // deltaV = v2 + w2.cross(mContactConstraints[c].r2Friction) - v1 - w1.cross(mContactConstraints[c].r1Friction);
        Vector3 deltaV(v2.x + w2.y * mContactConstraints[c].r2Friction.z - w2.z * mContactConstraints[c].r2Friction.y - v1.x -
                       w1.y * mContactConstraints[c].r1Friction.z + w1.z * mContactConstraints[c].r1Friction.y,

                       v2.y + w2.z * mContactConstraints[c].r2Friction.x - w2.x * mContactConstraints[c].r2Friction.z - v1.y -
                       w1.z * mContactConstraints[c].r1Friction.x + w1.x * mContactConstraints[c].r1Friction.z,

                       v2.z + w2.x * mContactConstraints[c].r2Friction.y - w2.y * mContactConstraints[c].r2Friction.x - v1.z -
                       w1.x * mContactConstraints[c].r1Friction.y + w1.y * mContactConstraints[c].r1Friction.x);
        decimal Jv = deltaV.x * mContactConstraints[c].frictionVector1.x +
                     deltaV.y * mContactConstraints[c].frictionVector1.y +
                     deltaV.z * mContactConstraints[c].frictionVector1.z;

        // Compute the Lagrange multiplier lambda
        decimal deltaLambda = -Jv * mContactConstraints[c].inverseFriction1Mass;
        decimal frictionLimit = mContactConstraints[c].frictionCoefficient * sumPenetrationImpulse;
        lambdaTemp = mContactConstraints[c].friction1Impulse;
        mContactConstraints[c].friction1Impulse = std::max(-frictionLimit,
                                                    std::min(mContactConstraints[c].friction1Impulse +
                                                             deltaLambda, frictionLimit));
        deltaLambda = mContactConstraints[c].friction1Impulse - lambdaTemp;

        // Compute the impulse P=J^T * lambda
        Vector3 angularImpulseBody1(-mContactConstraints[c].r1CrossT1.x * deltaLambda,
                                    -mContactConstraints[c].r1CrossT1.y * deltaLambda,
                                    -mContactConstraints[c].r1CrossT1.z * deltaLambda);
        Vector3 linearImpulseBody2(mContactConstraints[c].frictionVector1.x * deltaLambda,
                                   mContactConstraints[c].frictionVector1.y * deltaLambda,
                                   mContactConstraints[c].frictionVector1.z * deltaLambda);
        Vector3 angularImpulseBody2(mContactConstraints[c].r2CrossT1.x * deltaLambda,
                                    mContactConstraints[c].r2CrossT1.y * deltaLambda,
                                    mContactConstraints[c].r2CrossT1.z * deltaLambda);

        // Update the velocities of the body 1 by applying the impulse P
        mRigidBodyComponents.mConstrainedLinearVelocities[rigidBody1Index].x -= mContactConstraints[c].massInverseBody1 * linearImpulseBody2.x * mContactConstraints[c].linearLockAxisFactorBody1.x;
        mRigidBodyComponents.mConstrainedLinearVelocities[rigidBody1Index].y -= mContactConstraints[c].massInverseBody1 * linearImpulseBody2.y * mContactConstraints[c].linearLockAxisFactorBody1.y;
        mRigidBodyComponents.mConstrainedLinearVelocities[rigidBody1Index].z -= mContactConstraints[c].massInverseBody1 * linearImpulseBody2.z * mContactConstraints[c].linearLockAxisFactorBody1.z;

        Vector3 angularVelocity1 = mContactConstraints[c].angularLockAxisFactorBody1 * (mContactConstraints[c].inverseInertiaTensorBody1 * angularImpulseBody1);
        mRigidBodyComponents.mConstrainedAngularVelocities[rigidBody1Index].x += angularVelocity1.x;
        mRigidBodyComponents.mConstrainedAngularVelocities[rigidBody1Index].y += angularVelocity1.y;
        mRigidBodyComponents.mConstrainedAngularVelocities[rigidBody1Index].z += angularVelocity1.z;

        // Update the velocities of the body 2 by applying the impulse P
        mRigidBodyComponents.mConstrainedLinearVelocities[rigidBody2Index].x += mContactConstraints[c].massInverseBody2 * linearImpulseBody2.x * mContactConstraints[c].linearLockAxisFactorBody2.x;
        mRigidBodyComponents.mConstrainedLinearVelocities[rigidBody2Index].y += mContactConstraints[c].massInverseBody2 * linearImpulseBody2.y * mContactConstraints[c].linearLockAxisFactorBody2.y;
        mRigidBodyComponents.mConstrainedLinearVelocities[rigidBody2Index].z += mContactConstraints[c].massInverseBody2 * linearImpulseBody2.z * mContactConstraints[c].linearLockAxisFactorBody2.z;

        Vector3 angularVelocity2 = mContactConstraints[c].angularLockAxisFactorBody2 * (mContactConstraints[c].inverseInertiaTensorBody2 * angularImpulseBody2);
        mRigidBodyComponents.mConstrainedAngularVelocities[rigidBody2Index].x += angularVelocity2.x;
        mRigidBodyComponents.mConstrainedAngularVelocities[rigidBody2Index].y += angularVelocity2.y;
        mRigidBodyComponents.mConstrainedAngularVelocities[rigidBody2Index].z += angularVelocity2.z;

        // ------ Second friction constraint at the center of the contact manifold ----- //

        // Compute J*v
        //deltaV = v2 + w2.cross(mContactConstraints[c].r2Friction) - v1 - w1.cross(mContactConstraints[c].r1Friction);
        deltaV.x = v2.x + w2.y * mContactConstraints[c].r2Friction.z - w2.z * mContactConstraints[c].r2Friction.y  - v1.x -
                   w1.y * mContactConstraints[c].r1Friction.z + w1.z * mContactConstraints[c].r1Friction.y;
        deltaV.y = v2.y + w2.z * mContactConstraints[c].r2Friction.x - w2.x * mContactConstraints[c].r2Friction.z  - v1.y -
                   w1.z * mContactConstraints[c].r1Friction.x + w1.x * mContactConstraints[c].r1Friction.z;
        deltaV.z = v2.z + w2.x * mContactConstraints[c].r2Friction.y - w2.y * mContactConstraints[c].r2Friction.x  - v1.z -
                   w1.x * mContactConstraints[c].r1Friction.y + w1.y * mContactConstraints[c].r1Friction.x;
        Jv = deltaV.x * mContactConstraints[c].frictionVector2.x + deltaV.y * mContactConstraints[c].frictionVector2.y +
             deltaV.z * mContactConstraints[c].frictionVector2.z;

        // Compute the Lagrange multiplier lambda
        deltaLambda = -Jv * mContactConstraints[c].inverseFriction2Mass;
        frictionLimit = mContactConstraints[c].frictionCoefficient * sumPenetrationImpulse;
        lambdaTemp = mContactConstraints[c].friction2Impulse;
        mContactConstraints[c].friction2Impulse = std::max(-frictionLimit,
                                                    std::min(mContactConstraints[c].friction2Impulse +
                                                             deltaLambda, frictionLimit));
        deltaLambda = mContactConstraints[c].friction2Impulse - lambdaTemp;

        // Compute the impulse P=J^T * lambda
        angularImpulseBody1.x = -mContactConstraints[c].r1CrossT2.x * deltaLambda;
        angularImpulseBody1.y = -mContactConstraints[c].r1CrossT2.y * deltaLambda;
        angularImpulseBody1.z = -mContactConstraints[c].r1CrossT2.z * deltaLambda;

        linearImpulseBody2.x = mContactConstraints[c].frictionVector2.x * deltaLambda;
        linearImpulseBody2.y = mContactConstraints[c].frictionVector2.y * deltaLambda;
        linearImpulseBody2.z = mContactConstraints[c].frictionVector2.z * deltaLambda;

        angularImpulseBody2.x = mContactConstraints[c].r2CrossT2.x * deltaLambda;
        angularImpulseBody2.y = mContactConstraints[c].r2CrossT2.y * deltaLambda;
        angularImpulseBody2.z = mContactConstraints[c].r2CrossT2.z * deltaLambda;

        // Update the velocities of the body 1 by applying the impulse P
        mRigidBodyComponents.mConstrainedLinearVelocities[rigidBody1Index].x -= mContactConstraints[c].massInverseBody1 * linearImpulseBody2.x * mContactConstraints[c].linearLockAxisFactorBody1.x;
        mRigidBodyComponents.mConstrainedLinearVelocities[rigidBody1Index].y -= mContactConstraints[c].massInverseBody1 * linearImpulseBody2.y * mContactConstraints[c].linearLockAxisFactorBody1.y;
        mRigidBodyComponents.mConstrainedLinearVelocities[rigidBody1Index].z -= mContactConstraints[c].massInverseBody1 * linearImpulseBody2.z * mContactConstraints[c].linearLockAxisFactorBody1.z;

        angularVelocity1 = mContactConstraints[c].angularLockAxisFactorBody1 * (mContactConstraints[c].inverseInertiaTensorBody1 * angularImpulseBody1);
        mRigidBodyComponents.mConstrainedAngularVelocities[rigidBody1Index].x += angularVelocity1.x;
        mRigidBodyComponents.mConstrainedAngularVelocities[rigidBody1Index].y += angularVelocity1.y;
        mRigidBodyComponents.mConstrainedAngularVelocities[rigidBody1Index].z += angularVelocity1.z;

        // Update the velocities of the body 2 by applying the impulse P
        mRigidBodyComponents.mConstrainedLinearVelocities[rigidBody2Index].x += mContactConstraints[c].massInverseBody2 * linearImpulseBody2.x * mContactConstraints[c].linearLockAxisFactorBody2.x;
        mRigidBodyComponents.mConstrainedLinearVelocities[rigidBody2Index].y += mContactConstraints[c].massInverseBody2 * linearImpulseBody2.y * mContactConstraints[c].linearLockAxisFactorBody2.y;
        mRigidBodyComponents.mConstrainedLinearVelocities[rigidBody2Index].z += mContactConstraints[c].massInverseBody2 * linearImpulseBody2.z * mContactConstraints[c].linearLockAxisFactorBody2.z;

        angularVelocity2 = mContactConstraints[c].angularLockAxisFactorBody2 * (mContactConstraints[c].inverseInertiaTensorBody2 * angularImpulseBody2);
        mRigidBodyComponents.mConstrainedAngularVelocities[rigidBody2Index].x += angularVelocity2.x;
        mRigidBodyComponents.mConstrainedAngularVelocities[rigidBody2Index].y += angularVelocity2.y;
        mRigidBodyComponents.mConstrainedAngularVelocities[rigidBody2Index].z += angularVelocity2.z;

        // ------ Twist friction constraint at the center of the contact manifol ------ //

        // Compute J*v
        deltaV = w2 - w1;
        Jv = deltaV.x * mContactConstraints[c].normal.x + deltaV.y * mContactConstraints[c].normal.y +
             deltaV.z * mContactConstraints[c].normal.z;

        deltaLambda = -Jv * (mContactConstraints[c].inverseTwistFrictionMass);
        frictionLimit = mContactConstraints[c].frictionCoefficient * sumPenetrationImpulse;
        lambdaTemp = mContactConstraints[c].frictionTwistImpulse;
        mContactConstraints[c].frictionTwistImpulse = std::max(-frictionLimit,
                                                        std::min(mContactConstraints[c].frictionTwistImpulse
                                                                 + deltaLambda, frictionLimit));
        deltaLambda = mContactConstraints[c].frictionTwistImpulse - lambdaTemp;

        // Compute the impulse P=J^T * lambda
        angularImpulseBody2.x = mContactConstraints[c].normal.x * deltaLambda;
        angularImpulseBody2.y = mContactConstraints[c].normal.y * deltaLambda;
        angularImpulseBody2.z = mContactConstraints[c].normal.z * deltaLambda;

        // Update the velocities of the body 1 by applying the impulse P
        angularVelocity1 = mContactConstraints[c].angularLockAxisFactorBody1 * (mContactConstraints[c].inverseInertiaTensorBody1 * angularImpulseBody2);
        mRigidBodyComponents.mConstrainedAngularVelocities[rigidBody1Index].x -= angularVelocity1.x;
        mRigidBodyComponents.mConstrainedAngularVelocities[rigidBody1Index].y -= angularVelocity1.y;
        mRigidBodyComponents.mConstrainedAngularVelocities[rigidBody1Index].z -= angularVelocity1.z;

        // Update the velocities of the body 1 by applying the impulse P
        angularVelocity2 = mContactConstraints[c].angularLockAxisFactorBody2 * (mContactConstraints[c].inverseInertiaTensorBody2 * angularImpulseBody2);
        mRigidBodyComponents.mConstrainedAngularVelocities[rigidBody2Index].x += angularVelocity2.x;
        mRigidBodyComponents.mConstrainedAngularVelocities[rigidBody2Index].y += angularVelocity2.y;
        mRigidBodyComponents.mConstrainedAngularVelocities[rigidBody2Index].z += angularVelocity2.z;
    }
}

// Store the computed impulses to use them to
// warm start the solver at the next iteration
void ContactSolverSystem::storeImpulses() {

    RP3D_PROFILE("ContactSolver::storeImpulses()", mProfiler);

    uint32 contactPointIndex = 0;

    // For each contact manifold
    for (uint32 c=0; c<mNbContactManifolds; c++) {

        for (short int i=0; i<mContactConstraints[c].nbContacts; i++) {

            mContactPoints[contactPointIndex].externalContact->setPenetrationImpulse(mContactPoints[contactPointIndex].penetrationImpulse);

            contactPointIndex++;
        }

        mContactConstraints[c].externalContactManifold->frictionImpulse1 = mContactConstraints[c].friction1Impulse;
        mContactConstraints[c].externalContactManifold->frictionImpulse2 = mContactConstraints[c].friction2Impulse;
        mContactConstraints[c].externalContactManifold->frictionTwistImpulse = mContactConstraints[c].frictionTwistImpulse;
        mContactConstraints[c].externalContactManifold->frictionVector1 = mContactConstraints[c].frictionVector1;
        mContactConstraints[c].externalContactManifold->frictionVector2 = mContactConstraints[c].frictionVector2;
    }
}

// Compute the two unit orthogonal vectors "t1" and "t2" that span the tangential friction plane
// for a contact manifold. The two vectors have to be such that : t1 x t2 = contactNormal.
void ContactSolverSystem::computeFrictionVectors(const Vector3& deltaVelocity, ContactManifoldSolver& contact) const {

    RP3D_PROFILE("ContactSolver::computeFrictionVectors()", mProfiler);

    assert(contact.normal.length() > decimal(0.0));

    // Compute the velocity difference vector in the tangential plane
    decimal deltaVDotNormal = deltaVelocity.dot(contact.normal);
    Vector3 normalVelocity = deltaVDotNormal * contact.normal;
    Vector3 tangentVelocity(deltaVelocity.x - normalVelocity.x, deltaVelocity.y - normalVelocity.y,
                            deltaVelocity.z - normalVelocity.z);

    // If the velocty difference in the tangential plane is not zero
    const decimal lengthTangentVelocity = tangentVelocity.length();
    if (lengthTangentVelocity > MACHINE_EPSILON) {

        // Compute the first friction vector in the direction of the tangent
        // velocity difference
        contact.frictionVector1 = tangentVelocity / lengthTangentVelocity;
    }
    else {

        // Get any orthogonal vector to the normal as the first friction vector
        contact.frictionVector1 = contact.normal.getOneUnitOrthogonalVector();
    }

    // The second friction vector is computed by the cross product of the first
    // friction vector and the contact normal
    contact.frictionVector2 = contact.normal.cross(contact.frictionVector1);
}
