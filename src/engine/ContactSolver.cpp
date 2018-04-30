/********************************************************************************
* ReactPhysics3D physics library, http://www.reactphysics3d.com                 *
* Copyright (c) 2010-2018 Daniel Chappuis                                       *
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
#include "ContactSolver.h"
#include "DynamicsWorld.h"
#include "body/RigidBody.h"
#include "constraint/ContactPoint.h"
#include "utils/Profiler.h"
#include "engine/Island.h"
#include "collision/ContactManifold.h"

using namespace reactphysics3d;
using namespace std;

// Constants initialization
const decimal ContactSolver::BETA = decimal(0.2);
const decimal ContactSolver::BETA_SPLIT_IMPULSE = decimal(0.2);
const decimal ContactSolver::SLOP = decimal(0.01);

// Constructor
ContactSolver::ContactSolver(MemoryManager& memoryManager, const WorldSettings& worldSettings)
              :mMemoryManager(memoryManager), mSplitLinearVelocities(nullptr),
               mSplitAngularVelocities(nullptr), mContactConstraints(nullptr),
               mContactPoints(nullptr), mLinearVelocities(nullptr), mAngularVelocities(nullptr),
               mIsSplitImpulseActive(true), mWorldSettings(worldSettings) {

#ifdef IS_PROFILING_ACTIVE
        mProfiler = nullptr;
#endif

}

// Initialize the contact constraints
void ContactSolver::init(Island** islands, uint nbIslands, decimal timeStep) {

    RP3D_PROFILE("ContactSolver::init()", mProfiler);

    mTimeStep = timeStep;

    // TODO : Try not to count manifolds and contact points here
    uint nbContactManifolds = 0;
    uint nbContactPoints = 0;
    for (uint i = 0; i < nbIslands; i++) {
        uint nbManifoldsInIsland = islands[i]->getNbContactManifolds();
        nbContactManifolds += nbManifoldsInIsland;

        for (uint j=0; j < nbManifoldsInIsland; j++) {
            nbContactPoints += islands[i]->getContactManifolds()[j]->getNbContactPoints();
        }
    }

    mNbContactManifolds = 0;
    mNbContactPoints = 0;

    mContactConstraints = nullptr;
    mContactPoints = nullptr;

    if (nbContactManifolds == 0 || nbContactPoints == 0) return;

    // TODO : Count exactly the number of constraints to allocate here
    mContactPoints = static_cast<ContactPointSolver*>(mMemoryManager.allocate(MemoryManager::AllocationType::Frame,
                                                                              sizeof(ContactPointSolver) * nbContactPoints));
    assert(mContactPoints != nullptr);

    mContactConstraints = static_cast<ContactManifoldSolver*>(mMemoryManager.allocate(MemoryManager::AllocationType::Frame,
                                                                                      sizeof(ContactManifoldSolver) * nbContactManifolds));
    assert(mContactConstraints != nullptr);

    // For each island of the world
    for (uint islandIndex = 0; islandIndex < nbIslands; islandIndex++) {

        if (islands[islandIndex]->getNbContactManifolds() > 0) {
            initializeForIsland(islands[islandIndex]);
        }
    }

    // Warmstarting
    warmStart();
}

// Initialize the constraint solver for a given island
void ContactSolver::initializeForIsland(Island* island) {

    RP3D_PROFILE("ContactSolver::initializeForIsland()", mProfiler);

    assert(island != nullptr);
    assert(island->getNbBodies() > 0);
    assert(island->getNbContactManifolds() > 0);
    assert(mSplitLinearVelocities != nullptr);
    assert(mSplitAngularVelocities != nullptr);

    // For each contact manifold of the island
    ContactManifold** contactManifolds = island->getContactManifolds();
    for (uint i=0; i<island->getNbContactManifolds(); i++) {

        ContactManifold* externalManifold = contactManifolds[i];

        assert(externalManifold->getNbContactPoints() > 0);

        // Get the two bodies of the contact
        RigidBody* body1 = static_cast<RigidBody*>(externalManifold->getBody1());
        RigidBody* body2 = static_cast<RigidBody*>(externalManifold->getBody2());
        assert(body1 != nullptr);
        assert(body2 != nullptr);

        // Get the two contact shapes
        const ProxyShape* shape1 = externalManifold->getShape1();
        const ProxyShape* shape2 = externalManifold->getShape2();

        // Get the position of the two bodies
        const Vector3& x1 = body1->mCenterOfMassWorld;
        const Vector3& x2 = body2->mCenterOfMassWorld;

        // Initialize the internal contact manifold structure using the external
        // contact manifold
        new (mContactConstraints + mNbContactManifolds) ContactManifoldSolver();
        mContactConstraints[mNbContactManifolds].indexBody1 = body1->mArrayIndex;
        mContactConstraints[mNbContactManifolds].indexBody2 = body2->mArrayIndex;
        mContactConstraints[mNbContactManifolds].inverseInertiaTensorBody1 = body1->getInertiaTensorInverseWorld();
        mContactConstraints[mNbContactManifolds].inverseInertiaTensorBody2 = body2->getInertiaTensorInverseWorld();
        mContactConstraints[mNbContactManifolds].massInverseBody1 = body1->mMassInverse;
        mContactConstraints[mNbContactManifolds].massInverseBody2 = body2->mMassInverse;
        mContactConstraints[mNbContactManifolds].nbContacts = externalManifold->getNbContactPoints();
        mContactConstraints[mNbContactManifolds].frictionCoefficient = computeMixedFrictionCoefficient(body1, body2);
        mContactConstraints[mNbContactManifolds].rollingResistanceFactor = computeMixedRollingResistance(body1, body2);
        mContactConstraints[mNbContactManifolds].externalContactManifold = externalManifold;
        mContactConstraints[mNbContactManifolds].normal.setToZero();
        mContactConstraints[mNbContactManifolds].frictionPointBody1.setToZero();
        mContactConstraints[mNbContactManifolds].frictionPointBody2.setToZero();

        // Get the velocities of the bodies
        const Vector3& v1 = mLinearVelocities[mContactConstraints[mNbContactManifolds].indexBody1];
        const Vector3& w1 = mAngularVelocities[mContactConstraints[mNbContactManifolds].indexBody1];
        const Vector3& v2 = mLinearVelocities[mContactConstraints[mNbContactManifolds].indexBody2];
        const Vector3& w2 = mAngularVelocities[mContactConstraints[mNbContactManifolds].indexBody2];

        // For each  contact point of the contact manifold
        ContactPoint* externalContact = externalManifold->getContactPoints();
        assert(externalContact != nullptr);
        while (externalContact != nullptr) {

            // Get the contact point on the two bodies
            Vector3 p1 = shape1->getLocalToWorldTransform() * externalContact->getLocalPointOnShape1();
            Vector3 p2 = shape2->getLocalToWorldTransform() * externalContact->getLocalPointOnShape2();

            new (mContactPoints + mNbContactPoints) ContactPointSolver();
            mContactPoints[mNbContactPoints].externalContact = externalContact;
            mContactPoints[mNbContactPoints].normal = externalContact->getNormal();
            mContactPoints[mNbContactPoints].r1.x = p1.x - x1.x;
            mContactPoints[mNbContactPoints].r1.y = p1.y - x1.y;
            mContactPoints[mNbContactPoints].r1.z = p1.z - x1.z;
            mContactPoints[mNbContactPoints].r2.x = p2.x - x2.x;
            mContactPoints[mNbContactPoints].r2.y = p2.y - x2.y;
            mContactPoints[mNbContactPoints].r2.z = p2.z - x2.z;
            mContactPoints[mNbContactPoints].penetrationDepth = externalContact->getPenetrationDepth();
            mContactPoints[mNbContactPoints].isRestingContact = externalContact->getIsRestingContact();
            externalContact->setIsRestingContact(true);
            mContactPoints[mNbContactPoints].penetrationImpulse = externalContact->getPenetrationImpulse();
            mContactPoints[mNbContactPoints].penetrationSplitImpulse = 0.0;

            mContactConstraints[mNbContactManifolds].frictionPointBody1.x += p1.x;
            mContactConstraints[mNbContactManifolds].frictionPointBody1.y += p1.y;
            mContactConstraints[mNbContactManifolds].frictionPointBody1.z += p1.z;
            mContactConstraints[mNbContactManifolds].frictionPointBody2.x += p2.x;
            mContactConstraints[mNbContactManifolds].frictionPointBody2.y += p2.y;
            mContactConstraints[mNbContactManifolds].frictionPointBody2.z += p2.z;

            // Compute the velocity difference
            //deltaV = v2 + w2.cross(mContactPoints[mNbContactPoints].r2) - v1 - w1.cross(mContactPoints[mNbContactPoints].r1);
            Vector3 deltaV(v2.x + w2.y * mContactPoints[mNbContactPoints].r2.z - w2.z * mContactPoints[mNbContactPoints].r2.y
                           - v1.x - w1.y * mContactPoints[mNbContactPoints].r1.z - w1.z * mContactPoints[mNbContactPoints].r1.y,
                           v2.y + w2.z * mContactPoints[mNbContactPoints].r2.x - w2.x * mContactPoints[mNbContactPoints].r2.z
                           - v1.y - w1.z * mContactPoints[mNbContactPoints].r1.x - w1.x * mContactPoints[mNbContactPoints].r1.z,
                           v2.z + w2.x * mContactPoints[mNbContactPoints].r2.y - w2.y * mContactPoints[mNbContactPoints].r2.x
                           - v1.z - w1.x * mContactPoints[mNbContactPoints].r1.y - w1.y * mContactPoints[mNbContactPoints].r1.x);

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
            const decimal restitutionFactor = computeMixedRestitutionFactor(body1, body2);
            if (deltaVDotN < -mWorldSettings.restitutionVelocityThreshold) {
                mContactPoints[mNbContactPoints].restitutionBias = restitutionFactor * deltaVDotN;
            }

            mContactConstraints[mNbContactManifolds].normal.x += mContactPoints[mNbContactPoints].normal.x;
            mContactConstraints[mNbContactManifolds].normal.y += mContactPoints[mNbContactPoints].normal.y;
            mContactConstraints[mNbContactManifolds].normal.z += mContactPoints[mNbContactPoints].normal.z;

            mNbContactPoints++;

            externalContact = externalContact->getNext();
        }

        mContactConstraints[mNbContactManifolds].frictionPointBody1 /=static_cast<decimal>(mContactConstraints[mNbContactManifolds].nbContacts);
        mContactConstraints[mNbContactManifolds].frictionPointBody2 /=static_cast<decimal>(mContactConstraints[mNbContactManifolds].nbContacts);
        mContactConstraints[mNbContactManifolds].r1Friction.x = mContactConstraints[mNbContactManifolds].frictionPointBody1.x - x1.x;
        mContactConstraints[mNbContactManifolds].r1Friction.y = mContactConstraints[mNbContactManifolds].frictionPointBody1.y - x1.y;
        mContactConstraints[mNbContactManifolds].r1Friction.z = mContactConstraints[mNbContactManifolds].frictionPointBody1.z - x1.z;
        mContactConstraints[mNbContactManifolds].r2Friction.x = mContactConstraints[mNbContactManifolds].frictionPointBody2.x - x2.x;
        mContactConstraints[mNbContactManifolds].r2Friction.y = mContactConstraints[mNbContactManifolds].frictionPointBody2.y - x2.y;
        mContactConstraints[mNbContactManifolds].r2Friction.z = mContactConstraints[mNbContactManifolds].frictionPointBody2.z - x2.z;
        mContactConstraints[mNbContactManifolds].oldFrictionVector1 = externalManifold->getFrictionVector1();
        mContactConstraints[mNbContactManifolds].oldFrictionVector2 = externalManifold->getFrictionVector2();

        // Initialize the accumulated impulses with the previous step accumulated impulses
        mContactConstraints[mNbContactManifolds].friction1Impulse = externalManifold->getFrictionImpulse1();
        mContactConstraints[mNbContactManifolds].friction2Impulse = externalManifold->getFrictionImpulse2();
        mContactConstraints[mNbContactManifolds].frictionTwistImpulse = externalManifold->getFrictionTwistImpulse();

        // Compute the inverse K matrix for the rolling resistance constraint
        bool isBody1DynamicType = body1->getType() == BodyType::DYNAMIC;
        bool isBody2DynamicType = body2->getType() == BodyType::DYNAMIC;
        mContactConstraints[mNbContactManifolds].inverseRollingResistance.setToZero();
        if (mContactConstraints[mNbContactManifolds].rollingResistanceFactor > 0 && (isBody1DynamicType || isBody2DynamicType)) {

            mContactConstraints[mNbContactManifolds].inverseRollingResistance = mContactConstraints[mNbContactManifolds].inverseInertiaTensorBody1 + mContactConstraints[mNbContactManifolds].inverseInertiaTensorBody2;
            decimal det = mContactConstraints[mNbContactManifolds].inverseRollingResistance.getDeterminant();

            // If the matrix is not inversible
            if (approxEqual(det, decimal(0.0))) {
               mContactConstraints[mNbContactManifolds].inverseRollingResistance.setToZero();
            }
            else {
               mContactConstraints[mNbContactManifolds].inverseRollingResistance = mContactConstraints[mNbContactManifolds].inverseRollingResistance.getInverse();
            }
        }

        mContactConstraints[mNbContactManifolds].normal.normalize();

        // deltaVFrictionPoint = v2 + w2.cross(mContactConstraints[mNbContactManifolds].r2Friction) -
        //                              v1 - w1.cross(mContactConstraints[mNbContactManifolds].r1Friction);
        Vector3 deltaVFrictionPoint(v2.x + w2.y * mContactConstraints[mNbContactManifolds].r2Friction.z -
                                    w2.z * mContactConstraints[mNbContactManifolds].r2Friction.y -
                                      v1.x - w1.y * mContactConstraints[mNbContactManifolds].r1Friction.z -
                                      w1.z * mContactConstraints[mNbContactManifolds].r1Friction.y,
                                   v2.y + w2.z * mContactConstraints[mNbContactManifolds].r2Friction.x -
                                    w2.x * mContactConstraints[mNbContactManifolds].r2Friction.z -
                                      v1.y - w1.z * mContactConstraints[mNbContactManifolds].r1Friction.x -
                                      w1.x * mContactConstraints[mNbContactManifolds].r1Friction.z,
                                   v2.z + w2.x * mContactConstraints[mNbContactManifolds].r2Friction.y -
                                    w2.y * mContactConstraints[mNbContactManifolds].r2Friction.x -
                                      v1.z - w1.x * mContactConstraints[mNbContactManifolds].r1Friction.y -
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
void ContactSolver::warmStart() {

    RP3D_PROFILE("ContactSolver::warmStart()", mProfiler);

    uint contactPointIndex = 0;

    // For each constraint
    for (uint c=0; c<mNbContactManifolds; c++) {

        bool atLeastOneRestingContactPoint = false;

        for (short int i=0; i<mContactConstraints[c].nbContacts; i++) {

            // If it is not a new contact (this contact was already existing at last time step)
            if (mContactPoints[contactPointIndex].isRestingContact) {

                atLeastOneRestingContactPoint = true;

                // --------- Penetration --------- //

                // Update the velocities of the body 1 by applying the impulse P
                Vector3 impulsePenetration(mContactPoints[contactPointIndex].normal.x * mContactPoints[contactPointIndex].penetrationImpulse,
                                           mContactPoints[contactPointIndex].normal.y * mContactPoints[contactPointIndex].penetrationImpulse,
                                           mContactPoints[contactPointIndex].normal.z * mContactPoints[contactPointIndex].penetrationImpulse);
                mLinearVelocities[mContactConstraints[c].indexBody1].x -= mContactConstraints[c].massInverseBody1 * impulsePenetration.x;
                mLinearVelocities[mContactConstraints[c].indexBody1].y -= mContactConstraints[c].massInverseBody1 * impulsePenetration.y;
                mLinearVelocities[mContactConstraints[c].indexBody1].z -= mContactConstraints[c].massInverseBody1 * impulsePenetration.z;

                mAngularVelocities[mContactConstraints[c].indexBody1].x -= mContactPoints[contactPointIndex].i1TimesR1CrossN.x * mContactPoints[contactPointIndex].penetrationImpulse;
                mAngularVelocities[mContactConstraints[c].indexBody1].y -= mContactPoints[contactPointIndex].i1TimesR1CrossN.y * mContactPoints[contactPointIndex].penetrationImpulse;
                mAngularVelocities[mContactConstraints[c].indexBody1].z -= mContactPoints[contactPointIndex].i1TimesR1CrossN.z * mContactPoints[contactPointIndex].penetrationImpulse;

                // Update the velocities of the body 2 by applying the impulse P
                mLinearVelocities[mContactConstraints[c].indexBody2].x += mContactConstraints[c].massInverseBody2 * impulsePenetration.x;
                mLinearVelocities[mContactConstraints[c].indexBody2].y += mContactConstraints[c].massInverseBody2 * impulsePenetration.y;
                mLinearVelocities[mContactConstraints[c].indexBody2].z += mContactConstraints[c].massInverseBody2 * impulsePenetration.z;

                mAngularVelocities[mContactConstraints[c].indexBody2].x += mContactPoints[contactPointIndex].i2TimesR2CrossN.x * mContactPoints[contactPointIndex].penetrationImpulse;
                mAngularVelocities[mContactConstraints[c].indexBody2].y += mContactPoints[contactPointIndex].i2TimesR2CrossN.y * mContactPoints[contactPointIndex].penetrationImpulse;
                mAngularVelocities[mContactConstraints[c].indexBody2].z += mContactPoints[contactPointIndex].i2TimesR2CrossN.z * mContactPoints[contactPointIndex].penetrationImpulse;
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

            // Update the velocities of the body 1 by applying the impulse P
            mLinearVelocities[mContactConstraints[c].indexBody1] -= mContactConstraints[c].massInverseBody1 * linearImpulseBody2;
            mAngularVelocities[mContactConstraints[c].indexBody1] += mContactConstraints[c].inverseInertiaTensorBody1 * angularImpulseBody1;

            // Update the velocities of the body 1 by applying the impulse P
            mLinearVelocities[mContactConstraints[c].indexBody2] += mContactConstraints[c].massInverseBody2 * linearImpulseBody2;
            mAngularVelocities[mContactConstraints[c].indexBody2] += mContactConstraints[c].inverseInertiaTensorBody2 * angularImpulseBody2;

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
            mLinearVelocities[mContactConstraints[c].indexBody1].x -= mContactConstraints[c].massInverseBody1 * linearImpulseBody2.x;
            mLinearVelocities[mContactConstraints[c].indexBody1].y -= mContactConstraints[c].massInverseBody1 * linearImpulseBody2.y;
            mLinearVelocities[mContactConstraints[c].indexBody1].z -= mContactConstraints[c].massInverseBody1 * linearImpulseBody2.z;

            mAngularVelocities[mContactConstraints[c].indexBody1] += mContactConstraints[c].inverseInertiaTensorBody1 * angularImpulseBody1;

            // Update the velocities of the body 2 by applying the impulse P
            mLinearVelocities[mContactConstraints[c].indexBody2].x += mContactConstraints[c].massInverseBody2 * linearImpulseBody2.x;
            mLinearVelocities[mContactConstraints[c].indexBody2].y += mContactConstraints[c].massInverseBody2 * linearImpulseBody2.y;
            mLinearVelocities[mContactConstraints[c].indexBody2].z += mContactConstraints[c].massInverseBody2 * linearImpulseBody2.z;

            mAngularVelocities[mContactConstraints[c].indexBody2] += mContactConstraints[c].inverseInertiaTensorBody2 * angularImpulseBody2;

            // ------ Twist friction constraint at the center of the contact manifold ------ //

            // Compute the impulse P = J^T * lambda
            angularImpulseBody1.x = -mContactConstraints[c].normal.x * mContactConstraints[c].frictionTwistImpulse;
            angularImpulseBody1.y = -mContactConstraints[c].normal.y * mContactConstraints[c].frictionTwistImpulse;
            angularImpulseBody1.z = -mContactConstraints[c].normal.z * mContactConstraints[c].frictionTwistImpulse;

            angularImpulseBody2.x = mContactConstraints[c].normal.x * mContactConstraints[c].frictionTwistImpulse;
            angularImpulseBody2.y = mContactConstraints[c].normal.y * mContactConstraints[c].frictionTwistImpulse;
            angularImpulseBody2.z = mContactConstraints[c].normal.z * mContactConstraints[c].frictionTwistImpulse;

            // Update the velocities of the body 1 by applying the impulse P
            mAngularVelocities[mContactConstraints[c].indexBody1] += mContactConstraints[c].inverseInertiaTensorBody1 * angularImpulseBody1;

            // Update the velocities of the body 2 by applying the impulse P
            mAngularVelocities[mContactConstraints[c].indexBody2] += mContactConstraints[c].inverseInertiaTensorBody2 * angularImpulseBody2;

            // ------ Rolling resistance at the center of the contact manifold ------ //

            // Compute the impulse P = J^T * lambda
            angularImpulseBody2 = mContactConstraints[c].rollingResistanceImpulse;

            // Update the velocities of the body 1 by applying the impulse P
            mAngularVelocities[mContactConstraints[c].indexBody1] -= mContactConstraints[c].inverseInertiaTensorBody1 * angularImpulseBody2;

            // Update the velocities of the body 1 by applying the impulse P
            mAngularVelocities[mContactConstraints[c].indexBody2] += mContactConstraints[c].inverseInertiaTensorBody2 * angularImpulseBody2;
        }
        else {  // If it is a new contact manifold

            // Initialize the accumulated impulses to zero
            mContactConstraints[c].friction1Impulse = 0.0;
            mContactConstraints[c].friction2Impulse = 0.0;
            mContactConstraints[c].frictionTwistImpulse = 0.0;
            mContactConstraints[c].rollingResistanceImpulse.setToZero();
        }
    }
}

// Solve the contacts
void ContactSolver::solve() {

    RP3D_PROFILE("ContactSolver::solve()", mProfiler);

    decimal deltaLambda;
    decimal lambdaTemp;
    uint contactPointIndex = 0;

    // For each contact manifold
    for (uint c=0; c<mNbContactManifolds; c++) {

        decimal sumPenetrationImpulse = 0.0;

        // Get the constrained velocities
        const Vector3& v1 = mLinearVelocities[mContactConstraints[c].indexBody1];
        const Vector3& w1 = mAngularVelocities[mContactConstraints[c].indexBody1];
        const Vector3& v2 = mLinearVelocities[mContactConstraints[c].indexBody2];
        const Vector3& w2 = mAngularVelocities[mContactConstraints[c].indexBody2];

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
            decimal beta = mIsSplitImpulseActive ? BETA_SPLIT_IMPULSE : BETA;
            decimal biasPenetrationDepth = 0.0;
            if (mContactPoints[contactPointIndex].penetrationDepth > SLOP) biasPenetrationDepth = -(beta/mTimeStep) *
                    max(0.0f, float(mContactPoints[contactPointIndex].penetrationDepth - SLOP));
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
            mLinearVelocities[mContactConstraints[c].indexBody1].x -= mContactConstraints[c].massInverseBody1 * linearImpulse.x;
            mLinearVelocities[mContactConstraints[c].indexBody1].y -= mContactConstraints[c].massInverseBody1 * linearImpulse.y;
            mLinearVelocities[mContactConstraints[c].indexBody1].z -= mContactConstraints[c].massInverseBody1 * linearImpulse.z;

            mAngularVelocities[mContactConstraints[c].indexBody1].x -= mContactPoints[contactPointIndex].i1TimesR1CrossN.x * deltaLambda;
            mAngularVelocities[mContactConstraints[c].indexBody1].y -= mContactPoints[contactPointIndex].i1TimesR1CrossN.y * deltaLambda;
            mAngularVelocities[mContactConstraints[c].indexBody1].z -= mContactPoints[contactPointIndex].i1TimesR1CrossN.z * deltaLambda;

            // Update the velocities of the body 2 by applying the impulse P
            mLinearVelocities[mContactConstraints[c].indexBody2].x += mContactConstraints[c].massInverseBody2 * linearImpulse.x;
            mLinearVelocities[mContactConstraints[c].indexBody2].y += mContactConstraints[c].massInverseBody2 * linearImpulse.y;
            mLinearVelocities[mContactConstraints[c].indexBody2].z += mContactConstraints[c].massInverseBody2 * linearImpulse.z;

            mAngularVelocities[mContactConstraints[c].indexBody2].x += mContactPoints[contactPointIndex].i2TimesR2CrossN.x * deltaLambda;
            mAngularVelocities[mContactConstraints[c].indexBody2].y += mContactPoints[contactPointIndex].i2TimesR2CrossN.y * deltaLambda;
            mAngularVelocities[mContactConstraints[c].indexBody2].z += mContactPoints[contactPointIndex].i2TimesR2CrossN.z * deltaLambda;

            sumPenetrationImpulse += mContactPoints[contactPointIndex].penetrationImpulse;

            // If the split impulse position correction is active
            if (mIsSplitImpulseActive) {

                // Split impulse (position correction)
                const Vector3& v1Split = mSplitLinearVelocities[mContactConstraints[c].indexBody1];
                const Vector3& w1Split = mSplitAngularVelocities[mContactConstraints[c].indexBody1];
                const Vector3& v2Split = mSplitLinearVelocities[mContactConstraints[c].indexBody2];
                const Vector3& w2Split = mSplitAngularVelocities[mContactConstraints[c].indexBody2];

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
                mSplitLinearVelocities[mContactConstraints[c].indexBody1].x -= mContactConstraints[c].massInverseBody1 * linearImpulse.x;
                mSplitLinearVelocities[mContactConstraints[c].indexBody1].y -= mContactConstraints[c].massInverseBody1 * linearImpulse.y;
                mSplitLinearVelocities[mContactConstraints[c].indexBody1].z -= mContactConstraints[c].massInverseBody1 * linearImpulse.z;

                mSplitAngularVelocities[mContactConstraints[c].indexBody1].x -= mContactPoints[contactPointIndex].i1TimesR1CrossN.x * deltaLambdaSplit;
                mSplitAngularVelocities[mContactConstraints[c].indexBody1].y -= mContactPoints[contactPointIndex].i1TimesR1CrossN.y * deltaLambdaSplit;
                mSplitAngularVelocities[mContactConstraints[c].indexBody1].z -= mContactPoints[contactPointIndex].i1TimesR1CrossN.z * deltaLambdaSplit;

                // Update the velocities of the body 1 by applying the impulse P
                mSplitLinearVelocities[mContactConstraints[c].indexBody2].x += mContactConstraints[c].massInverseBody2 * linearImpulse.x;
                mSplitLinearVelocities[mContactConstraints[c].indexBody2].y += mContactConstraints[c].massInverseBody2 * linearImpulse.y;
                mSplitLinearVelocities[mContactConstraints[c].indexBody2].z += mContactConstraints[c].massInverseBody2 * linearImpulse.z;

                mSplitAngularVelocities[mContactConstraints[c].indexBody2].x += mContactPoints[contactPointIndex].i2TimesR2CrossN.x * deltaLambdaSplit;
                mSplitAngularVelocities[mContactConstraints[c].indexBody2].y += mContactPoints[contactPointIndex].i2TimesR2CrossN.y * deltaLambdaSplit;
                mSplitAngularVelocities[mContactConstraints[c].indexBody2].z += mContactPoints[contactPointIndex].i2TimesR2CrossN.z * deltaLambdaSplit;
            }

            contactPointIndex++;
        }

        // ------ First friction constraint at the center of the contact manifol ------ //

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
        mLinearVelocities[mContactConstraints[c].indexBody1].x -= mContactConstraints[c].massInverseBody1 * linearImpulseBody2.x;
        mLinearVelocities[mContactConstraints[c].indexBody1].y -= mContactConstraints[c].massInverseBody1 * linearImpulseBody2.y;
        mLinearVelocities[mContactConstraints[c].indexBody1].z -= mContactConstraints[c].massInverseBody1 * linearImpulseBody2.z;

        mAngularVelocities[mContactConstraints[c].indexBody1] += mContactConstraints[c].inverseInertiaTensorBody1 * angularImpulseBody1;

        // Update the velocities of the body 2 by applying the impulse P
        mLinearVelocities[mContactConstraints[c].indexBody2].x += mContactConstraints[c].massInverseBody2 * linearImpulseBody2.x;
        mLinearVelocities[mContactConstraints[c].indexBody2].y += mContactConstraints[c].massInverseBody2 * linearImpulseBody2.y;
        mLinearVelocities[mContactConstraints[c].indexBody2].z += mContactConstraints[c].massInverseBody2 * linearImpulseBody2.z;

        mAngularVelocities[mContactConstraints[c].indexBody2] += mContactConstraints[c].inverseInertiaTensorBody2 * angularImpulseBody2;

        // ------ Second friction constraint at the center of the contact manifol ----- //

        // Compute J*v
        //deltaV = v2 + w2.cross(mContactConstraints[c].r2Friction) - v1 - w1.cross(mContactConstraints[c].r1Friction);
        deltaV.x = v2.x + w2.y * mContactConstraints[c].r2Friction.z - v2.z * mContactConstraints[c].r2Friction.y  - v1.x -
                   w1.y * mContactConstraints[c].r1Friction.z + w1.z * mContactConstraints[c].r1Friction.y;
        deltaV.y = v2.y + w2.z * mContactConstraints[c].r2Friction.x - v2.x * mContactConstraints[c].r2Friction.z  - v1.y -
                   w1.z * mContactConstraints[c].r1Friction.x + w1.x * mContactConstraints[c].r1Friction.z;
        deltaV.z = v2.z + w2.x * mContactConstraints[c].r2Friction.y - v2.y * mContactConstraints[c].r2Friction.x  - v1.z -
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
        mLinearVelocities[mContactConstraints[c].indexBody1].x -= mContactConstraints[c].massInverseBody1 * linearImpulseBody2.x;
        mLinearVelocities[mContactConstraints[c].indexBody1].y -= mContactConstraints[c].massInverseBody1 * linearImpulseBody2.y;
        mLinearVelocities[mContactConstraints[c].indexBody1].z -= mContactConstraints[c].massInverseBody1 * linearImpulseBody2.z;
        mAngularVelocities[mContactConstraints[c].indexBody1] += mContactConstraints[c].inverseInertiaTensorBody1 * angularImpulseBody1;

        // Update the velocities of the body 2 by applying the impulse P
        mLinearVelocities[mContactConstraints[c].indexBody2].x += mContactConstraints[c].massInverseBody2 * linearImpulseBody2.x;
        mLinearVelocities[mContactConstraints[c].indexBody2].y += mContactConstraints[c].massInverseBody2 * linearImpulseBody2.y;
        mLinearVelocities[mContactConstraints[c].indexBody2].z += mContactConstraints[c].massInverseBody2 * linearImpulseBody2.z;
        mAngularVelocities[mContactConstraints[c].indexBody2] += mContactConstraints[c].inverseInertiaTensorBody2 * angularImpulseBody2;

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
        mAngularVelocities[mContactConstraints[c].indexBody1] -= mContactConstraints[c].inverseInertiaTensorBody1 * angularImpulseBody2;

        // Update the velocities of the body 1 by applying the impulse P
        mAngularVelocities[mContactConstraints[c].indexBody2] += mContactConstraints[c].inverseInertiaTensorBody2 * angularImpulseBody2;

        // --------- Rolling resistance constraint at the center of the contact manifold --------- //

        if (mContactConstraints[c].rollingResistanceFactor > 0) {

            // Compute J*v
            const Vector3 JvRolling = w2 - w1;

            // Compute the Lagrange multiplier lambda
            Vector3 deltaLambdaRolling = mContactConstraints[c].inverseRollingResistance * (-JvRolling);
            decimal rollingLimit = mContactConstraints[c].rollingResistanceFactor * sumPenetrationImpulse;
            Vector3 lambdaTempRolling = mContactConstraints[c].rollingResistanceImpulse;
            mContactConstraints[c].rollingResistanceImpulse = clamp(mContactConstraints[c].rollingResistanceImpulse +
                                                                 deltaLambdaRolling, rollingLimit);
            deltaLambdaRolling = mContactConstraints[c].rollingResistanceImpulse - lambdaTempRolling;

            // Update the velocities of the body 1 by applying the impulse P
            mAngularVelocities[mContactConstraints[c].indexBody1] -= mContactConstraints[c].inverseInertiaTensorBody1 * deltaLambdaRolling;

            // Update the velocities of the body 2 by applying the impulse P
            mAngularVelocities[mContactConstraints[c].indexBody2] += mContactConstraints[c].inverseInertiaTensorBody2 * deltaLambdaRolling;
        }
    }
}

// Compute the collision restitution factor from the restitution factor of each body
decimal ContactSolver::computeMixedRestitutionFactor(RigidBody* body1,
                                                            RigidBody* body2) const {
    decimal restitution1 = body1->getMaterial().getBounciness();
    decimal restitution2 = body2->getMaterial().getBounciness();

    // Return the largest restitution factor
    return (restitution1 > restitution2) ? restitution1 : restitution2;
}

// Compute the mixed friction coefficient from the friction coefficient of each body
decimal ContactSolver::computeMixedFrictionCoefficient(RigidBody *body1,
                                                              RigidBody *body2) const {
    // Use the geometric mean to compute the mixed friction coefficient
    return std::sqrt(body1->getMaterial().getFrictionCoefficient() *
                body2->getMaterial().getFrictionCoefficient());
}

// Compute th mixed rolling resistance factor between two bodies
inline decimal ContactSolver::computeMixedRollingResistance(RigidBody* body1,
                                                            RigidBody* body2) const {
    return decimal(0.5f) * (body1->getMaterial().getRollingResistance() + body2->getMaterial().getRollingResistance());
}

// Store the computed impulses to use them to
// warm start the solver at the next iteration
void ContactSolver::storeImpulses() {

    RP3D_PROFILE("ContactSolver::storeImpulses()", mProfiler);

    uint contactPointIndex = 0;

    // For each contact manifold
    for (uint c=0; c<mNbContactManifolds; c++) {

        for (short int i=0; i<mContactConstraints[c].nbContacts; i++) {

            mContactPoints[contactPointIndex].externalContact->setPenetrationImpulse(mContactPoints[contactPointIndex].penetrationImpulse);

            contactPointIndex++;
        }

        mContactConstraints[c].externalContactManifold->setFrictionImpulse1(mContactConstraints[c].friction1Impulse);
        mContactConstraints[c].externalContactManifold->setFrictionImpulse2(mContactConstraints[c].friction2Impulse);
        mContactConstraints[c].externalContactManifold->setFrictionTwistImpulse(mContactConstraints[c].frictionTwistImpulse);
        mContactConstraints[c].externalContactManifold->setRollingResistanceImpulse(mContactConstraints[c].rollingResistanceImpulse);
        mContactConstraints[c].externalContactManifold->setFrictionVector1(mContactConstraints[c].frictionVector1);
        mContactConstraints[c].externalContactManifold->setFrictionVector2(mContactConstraints[c].frictionVector2);
    }
}

// Compute the two unit orthogonal vectors "t1" and "t2" that span the tangential friction plane
// for a contact manifold. The two vectors have to be such that : t1 x t2 = contactNormal.
void ContactSolver::computeFrictionVectors(const Vector3& deltaVelocity,
                                           ContactManifoldSolver& contact) const {

    RP3D_PROFILE("ContactSolver::computeFrictionVectors()", mProfiler);

    assert(contact.normal.length() > decimal(0.0));

    // Compute the velocity difference vector in the tangential plane
    Vector3 normalVelocity(deltaVelocity.x * contact.normal.x * contact.normal.x,
                           deltaVelocity.y * contact.normal.y * contact.normal.y,
                           deltaVelocity.z * contact.normal.z * contact.normal.z);
    Vector3 tangentVelocity(deltaVelocity.x - normalVelocity.x, deltaVelocity.y - normalVelocity.y,
                            deltaVelocity.z - normalVelocity.z);

    // If the velocty difference in the tangential plane is not zero
    decimal lengthTangenVelocity = tangentVelocity.length();
    if (lengthTangenVelocity > MACHINE_EPSILON) {

        // Compute the first friction vector in the direction of the tangent
        // velocity difference
        contact.frictionVector1 = tangentVelocity / lengthTangenVelocity;
    }
    else {

        // Get any orthogonal vector to the normal as the first friction vector
        contact.frictionVector1 = contact.normal.getOneUnitOrthogonalVector();
    }

    // The second friction vector is computed by the cross product of the firs
    // friction vector and the contact normal
    contact.frictionVector2 = contact.normal.cross(contact.frictionVector1).getUnit();
}
