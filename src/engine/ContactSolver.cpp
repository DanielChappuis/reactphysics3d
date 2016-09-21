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
#include "ContactSolver.h"
#include "DynamicsWorld.h"
#include "body/RigidBody.h"
#include "Profiler.h"
#include <limits>

using namespace reactphysics3d;
using namespace std;

// Constants initialization
const decimal ContactSolver::BETA = decimal(0.2);
const decimal ContactSolver::BETA_SPLIT_IMPULSE = decimal(0.2);
const decimal ContactSolver::SLOP= decimal(0.01);

// Constructor
ContactSolver::ContactSolver(const std::map<RigidBody*, uint>& mapBodyToVelocityIndex,
                             SingleFrameAllocator& singleFrameAllocator)
              :mSplitLinearVelocities(nullptr), mSplitAngularVelocities(nullptr),
               mSingleFrameAllocator(singleFrameAllocator),
               mPenetrationConstraints(nullptr), mFrictionConstraints(nullptr),
               mLinearVelocities(nullptr), mAngularVelocities(nullptr),
               mMapBodyToConstrainedVelocityIndex(mapBodyToVelocityIndex),
               mIsWarmStartingActive(true), mIsSplitImpulseActive(true),
               mIsSolveFrictionAtContactManifoldCenterActive(true) {

}

// Initialize the contact constraints
void ContactSolver::init(Island** islands, uint nbIslands, decimal timeStep) {

    mTimeStep = timeStep;

    // TODO : Try not to count manifolds here
    // Count the contact manifolds
    uint nbContactManifolds = 0;
    for (uint i = 0; i < nbIslands; i++) {
        nbContactManifolds += islands[i]->getNbContactManifolds();
    }

    mNbFrictionConstraints = 0;
    mNbPenetrationConstraints = 0;

    mPenetrationConstraints = nullptr;
    mFrictionConstraints = nullptr;

    if (nbContactManifolds == 0) return;

    // TODO : Count exactly the number of constraints to allocate here
    uint nbPenetrationConstraints = nbContactManifolds * MAX_CONTACT_POINTS_IN_MANIFOLD;
    mPenetrationConstraints = static_cast<PenetrationConstraint*>(mSingleFrameAllocator.allocate(sizeof(PenetrationConstraint) * nbPenetrationConstraints));
    //mPenetrationConstraints = new PenetrationConstraint[nbPenetrationConstraints];
    assert(mPenetrationConstraints != nullptr);
    //mPenetrationConstraints = new PenetrationConstraint[mNbContactManifolds * 4];

    mFrictionConstraints = static_cast<FrictionConstraint*>(mSingleFrameAllocator.allocate(sizeof(FrictionConstraint) * nbContactManifolds));
    //mFrictionConstraints = new FrictionConstraint[nbContactManifolds];
    assert(mFrictionConstraints != nullptr);
    //mFrictionConstraints = new FrictionConstraint[mNbContactManifolds];

    // For each island of the world
    for (uint islandIndex = 0; islandIndex < nbIslands; islandIndex++) {
        initializeForIsland(islands[islandIndex]);
    }

    // Warmstarting
    warmStart();
}

// Initialize the constraint solver for a given island
void ContactSolver::initializeForIsland(Island* island) {

    PROFILE("ContactSolver::initializeForIsland()");

    assert(island != nullptr);
    assert(island->getNbBodies() > 0);
    assert(mSplitLinearVelocities != nullptr);
    assert(mSplitAngularVelocities != nullptr);

    // For each contact manifold of the island
    ContactManifold** contactManifolds = island->getContactManifolds();
    for (uint i=0; i<island->getNbContactManifolds(); i++) {

        ContactManifold* externalManifold = contactManifolds[i];

        assert(externalManifold->getNbContactPoints() > 0);

        // Get the two bodies of the contact
        RigidBody* body1 = static_cast<RigidBody*>(externalManifold->getContactPoint(0)->getBody1());
        RigidBody* body2 = static_cast<RigidBody*>(externalManifold->getContactPoint(0)->getBody2());
        assert(body1 != nullptr);
        assert(body2 != nullptr);

        // TODO : Check if we have a better way to find the body index
        uint indexBody1 = mMapBodyToConstrainedVelocityIndex.find(body1)->second;
        uint indexBody2 = mMapBodyToConstrainedVelocityIndex.find(body2)->second;

        new (mFrictionConstraints + mNbFrictionConstraints) FrictionConstraint();
        mFrictionConstraints[mNbFrictionConstraints].indexBody1 = indexBody1;
        mFrictionConstraints[mNbFrictionConstraints].indexBody2 = indexBody2;
        mFrictionConstraints[mNbFrictionConstraints].contactManifold = externalManifold;

        // Get the position of the two bodies
        const Vector3& x1 = body1->mCenterOfMassWorld;
        const Vector3& x2 = body2->mCenterOfMassWorld;

        // Get the velocities of the bodies
        const Vector3& v1 = mLinearVelocities[indexBody1];
        const Vector3& w1 = mAngularVelocities[indexBody1];
        const Vector3& v2 = mLinearVelocities[indexBody2];
        const Vector3& w2 = mAngularVelocities[indexBody2];

        // Get the inertia tensors of both bodies
        Matrix3x3 I1 = body1->getInertiaTensorInverseWorld();
        Matrix3x3 I2 = body2->getInertiaTensorInverseWorld();

        mFrictionConstraints[mNbFrictionConstraints].inverseInertiaTensorBody1 = I1;
        mFrictionConstraints[mNbFrictionConstraints].inverseInertiaTensorBody2 = I2;

        // Initialize the internal contact manifold structure using the external
        // contact manifold
        mFrictionConstraints[mNbFrictionConstraints].massInverseBody1 = body1->mMassInverse;
        mFrictionConstraints[mNbFrictionConstraints].massInverseBody2 = body2->mMassInverse;
        //internalManifold.nbContacts = externalManifold->getNbContactPoints();
        decimal restitutionFactor = computeMixedRestitutionFactor(body1, body2);
        mFrictionConstraints[mNbFrictionConstraints].frictionCoefficient = computeMixedFrictionCoefficient(body1, body2);
        mFrictionConstraints[mNbFrictionConstraints].rollingResistanceFactor = computeMixedRollingResistance(body1, body2);
        mFrictionConstraints[mNbFrictionConstraints].hasAtLeastOneRestingContactPoint = false;
        //internalManifold.isBody1DynamicType = body1->getType() == BodyType::DYNAMIC;
        //internalManifold.isBody2DynamicType = body2->getType() == BodyType::DYNAMIC;

        bool isBody1DynamicType = body1->getType() == BodyType::DYNAMIC;
        bool isBody2DynamicType = body2->getType() == BodyType::DYNAMIC;

        // If we solve the friction constraints at the center of the contact manifold
        //if (mIsSolveFrictionAtContactManifoldCenterActive) {
            mFrictionConstraints[mNbFrictionConstraints].frictionPointBody1 = Vector3::zero();
            mFrictionConstraints[mNbFrictionConstraints].frictionPointBody2 = Vector3::zero();
            mFrictionConstraints[mNbFrictionConstraints].normal = Vector3::zero();
        //}

        // Compute the inverse K matrix for the rolling resistance constraint
        mFrictionConstraints[mNbFrictionConstraints].inverseRollingResistance.setToZero();
        if (mFrictionConstraints[mNbFrictionConstraints].rollingResistanceFactor > 0 && (isBody1DynamicType || isBody2DynamicType)) {
            mFrictionConstraints[mNbFrictionConstraints].inverseRollingResistance = I1 + I2;
            mFrictionConstraints[mNbFrictionConstraints].inverseRollingResistance = mFrictionConstraints[mNbFrictionConstraints].inverseRollingResistance.getInverse();
        }

        int nbContacts = 0;

        // For each  contact point of the contact manifold
        for (uint c=0; c<externalManifold->getNbContactPoints(); c++) {

            // Get a contact point
            ContactPoint* externalContact = externalManifold->getContactPoint(c);

            new (mPenetrationConstraints + mNbPenetrationConstraints) PenetrationConstraint();
            mPenetrationConstraints[mNbPenetrationConstraints].indexBody1 = indexBody1;
            mPenetrationConstraints[mNbPenetrationConstraints].indexBody2 = indexBody2;
            mPenetrationConstraints[mNbPenetrationConstraints].inverseInertiaTensorBody1 = I1;
            mPenetrationConstraints[mNbPenetrationConstraints].inverseInertiaTensorBody2 = I2;
            mPenetrationConstraints[mNbPenetrationConstraints].massInverseBody1 = body1->mMassInverse;
            mPenetrationConstraints[mNbPenetrationConstraints].massInverseBody2 = body2->mMassInverse;
            mPenetrationConstraints[mNbPenetrationConstraints].restitutionFactor = restitutionFactor;
            mPenetrationConstraints[mNbPenetrationConstraints].indexFrictionConstraint = mNbFrictionConstraints;
            mPenetrationConstraints[mNbPenetrationConstraints].contactPoint = externalContact;

            // Get the contact point on the two bodies
            Vector3 p1 = externalContact->getWorldPointOnBody1();
            Vector3 p2 = externalContact->getWorldPointOnBody2();

            mPenetrationConstraints[mNbPenetrationConstraints].r1 = p1 - x1;
            mPenetrationConstraints[mNbPenetrationConstraints].r2 = p2 - x2;

            //mPenetrationConstraints[penConstIndex].externalContact = externalContact;
            mPenetrationConstraints[mNbPenetrationConstraints].normal = externalContact->getNormal();
            mPenetrationConstraints[mNbPenetrationConstraints].penetrationDepth = externalContact->getPenetrationDepth();
            mPenetrationConstraints[mNbPenetrationConstraints].isRestingContact = externalContact->getIsRestingContact();

            mFrictionConstraints[mNbFrictionConstraints].hasAtLeastOneRestingContactPoint |= mPenetrationConstraints[mNbPenetrationConstraints].isRestingContact;

            externalContact->setIsRestingContact(true);
            //mPenetrationConstraints[penConstIndex].oldFrictionVector1 = externalContact->getFrictionVector1();
            //mPenetrationConstraints[penConstIndex].oldFrictionVector2 = externalContact->getFrictionVector2();
            mPenetrationConstraints[mNbPenetrationConstraints].penetrationImpulse = 0.0;
            //mPenetrationConstraints[penConstIndex].friction1Impulse = 0.0;
            //mPenetrationConstraints[penConstIndex].friction2Impulse = 0.0;
            //mPenetrationConstraints[penConstIndex].rollingResistanceImpulse = Vector3::zero();

            // If we solve the friction constraints at the center of the contact manifold
            //if (mIsSolveFrictionAtContactManifoldCenterActive) {
                mFrictionConstraints[mNbFrictionConstraints].frictionPointBody1 += p1;
                mFrictionConstraints[mNbFrictionConstraints].frictionPointBody2 += p2;
            //}

            // Compute the velocity difference
            Vector3 deltaV = v2 + w2.cross(mPenetrationConstraints[mNbPenetrationConstraints].r2) - v1 - w1.cross(mPenetrationConstraints[mNbPenetrationConstraints].r1);
            mPenetrationConstraints[mNbPenetrationConstraints].r1CrossN = mPenetrationConstraints[mNbPenetrationConstraints].r1.cross(mPenetrationConstraints[mNbPenetrationConstraints].normal);
            mPenetrationConstraints[mNbPenetrationConstraints].r2CrossN = mPenetrationConstraints[mNbPenetrationConstraints].r2.cross(mPenetrationConstraints[mNbPenetrationConstraints].normal);

            // Compute the inverse mass matrix K for the penetration constraint
            decimal massPenetration = mPenetrationConstraints[mNbPenetrationConstraints].massInverseBody1 + mPenetrationConstraints[mNbPenetrationConstraints].massInverseBody2 +
                    ((mPenetrationConstraints[mNbPenetrationConstraints].inverseInertiaTensorBody1 * mPenetrationConstraints[mNbPenetrationConstraints].r1CrossN ).cross(mPenetrationConstraints[mNbPenetrationConstraints].r1)).dot(mPenetrationConstraints[mNbPenetrationConstraints].normal) +
                    ((mPenetrationConstraints[mNbPenetrationConstraints].inverseInertiaTensorBody2 * mPenetrationConstraints[mNbPenetrationConstraints].r2CrossN ).cross(mPenetrationConstraints[mNbPenetrationConstraints].r2)).dot(mPenetrationConstraints[mNbPenetrationConstraints].normal);
            massPenetration > decimal(0.0) ? mPenetrationConstraints[mNbPenetrationConstraints].inversePenetrationMass = decimal(1.0) /
                                                                          massPenetration :
                                                                          decimal(0.0);

            // Compute the restitution velocity bias "b". We compute this here instead
            // of inside the solve() method because we need to use the velocity difference
            // at the beginning of the contact. Note that if it is a resting contact (normal
            // velocity bellow a given threshold), we do not add a restitution velocity bias
            mPenetrationConstraints[mNbPenetrationConstraints].restitutionBias = 0.0;
            decimal deltaVDotN = deltaV.dot(mPenetrationConstraints[mNbPenetrationConstraints].normal);
            if (deltaVDotN < -RESTITUTION_VELOCITY_THRESHOLD) {
                mPenetrationConstraints[mNbPenetrationConstraints].restitutionBias = mPenetrationConstraints[mNbPenetrationConstraints].restitutionFactor * deltaVDotN;
            }

            // If the warm starting of the contact solver is active
            if (mIsWarmStartingActive) {

                // Get the cached accumulated impulses from the previous step
                mPenetrationConstraints[mNbPenetrationConstraints].penetrationImpulse = externalContact->getPenetrationImpulse();
            }

            // Initialize the split impulses to zero
            mPenetrationConstraints[mNbPenetrationConstraints].penetrationSplitImpulse = 0.0;

            // If we solve the friction constraints at the center of the contact manifold
            //if (mIsSolveFrictionAtContactManifoldCenterActive) {
                mFrictionConstraints[mNbFrictionConstraints].normal += mPenetrationConstraints[mNbPenetrationConstraints].normal;
            //}

            mNbPenetrationConstraints++;
            nbContacts++;
        }

        // If we solve the friction constraints at the center of the contact manifold
        //if (mIsSolveFrictionAtContactManifoldCenterActive) {

            //mFrictionConstraints[mNbFrictionConstraints].normal = Vector3::zero();
            mFrictionConstraints[mNbFrictionConstraints].frictionPointBody1 /= nbContacts;
            mFrictionConstraints[mNbFrictionConstraints].frictionPointBody2 /= nbContacts;
            mFrictionConstraints[mNbFrictionConstraints].r1Friction = mFrictionConstraints[mNbFrictionConstraints].frictionPointBody1 - x1;
            mFrictionConstraints[mNbFrictionConstraints].r2Friction = mFrictionConstraints[mNbFrictionConstraints].frictionPointBody2 - x2;
            mFrictionConstraints[mNbFrictionConstraints].oldFrictionVector1 = externalManifold->getFrictionVector1();
            mFrictionConstraints[mNbFrictionConstraints].oldFrictionVector2 = externalManifold->getFrictionVector2();

            // If warm starting is active
            if (mIsWarmStartingActive) {

                // Initialize the accumulated impulses with the previous step accumulated impulses
                mFrictionConstraints[mNbFrictionConstraints].friction1Impulse = externalManifold->getFrictionImpulse1();
                mFrictionConstraints[mNbFrictionConstraints].friction2Impulse = externalManifold->getFrictionImpulse2();
                mFrictionConstraints[mNbFrictionConstraints].frictionTwistImpulse = externalManifold->getFrictionTwistImpulse();
            }
            else {

                // Initialize the accumulated impulses to zero
                mFrictionConstraints[mNbFrictionConstraints].friction1Impulse = 0.0;
                mFrictionConstraints[mNbFrictionConstraints].friction2Impulse = 0.0;
                mFrictionConstraints[mNbFrictionConstraints].frictionTwistImpulse = 0.0;
                mFrictionConstraints[mNbFrictionConstraints].rollingResistanceImpulse = Vector3(0, 0, 0);
            }

            mFrictionConstraints[mNbFrictionConstraints].normal.normalize();

            Vector3 deltaVFrictionPoint = v2 + w2.cross(mFrictionConstraints[mNbFrictionConstraints].r2Friction) -
                                          v1 - w1.cross(mFrictionConstraints[mNbFrictionConstraints].r1Friction);

            // Compute the friction vectors
            computeFrictionVectors(deltaVFrictionPoint, mFrictionConstraints[mNbFrictionConstraints]);

            // Compute the inverse mass matrix K for the friction constraints at the center of the contact manifold
            mFrictionConstraints[mNbFrictionConstraints].r1CrossT1 = mFrictionConstraints[mNbFrictionConstraints].r1Friction.cross(mFrictionConstraints[mNbFrictionConstraints].frictionVector1);
            mFrictionConstraints[mNbFrictionConstraints].r1CrossT2 = mFrictionConstraints[mNbFrictionConstraints].r1Friction.cross(mFrictionConstraints[mNbFrictionConstraints].frictionVector2);
            mFrictionConstraints[mNbFrictionConstraints].r2CrossT1 = mFrictionConstraints[mNbFrictionConstraints].r2Friction.cross(mFrictionConstraints[mNbFrictionConstraints].frictionVector1);
            mFrictionConstraints[mNbFrictionConstraints].r2CrossT2 = mFrictionConstraints[mNbFrictionConstraints].r2Friction.cross(mFrictionConstraints[mNbFrictionConstraints].frictionVector2);
            decimal friction1Mass = mFrictionConstraints[mNbFrictionConstraints].massInverseBody1 + mFrictionConstraints[mNbFrictionConstraints].massInverseBody2 +
                                    ((I1 * mFrictionConstraints[mNbFrictionConstraints].r1CrossT1).cross(mFrictionConstraints[mNbFrictionConstraints].r1Friction)).dot(
                                    mFrictionConstraints[mNbFrictionConstraints].frictionVector1) +
                                    ((I2 * mFrictionConstraints[mNbFrictionConstraints].r2CrossT1).cross(mFrictionConstraints[mNbFrictionConstraints].r2Friction)).dot(
                                    mFrictionConstraints[mNbFrictionConstraints].frictionVector1);
            decimal friction2Mass = mFrictionConstraints[mNbFrictionConstraints].massInverseBody1 + mFrictionConstraints[mNbFrictionConstraints].massInverseBody2 +
                                    ((I1 * mFrictionConstraints[mNbFrictionConstraints].r1CrossT2).cross(mFrictionConstraints[mNbFrictionConstraints].r1Friction)).dot(
                                    mFrictionConstraints[mNbFrictionConstraints].frictionVector2) +
                                    ((I2 * mFrictionConstraints[mNbFrictionConstraints].r2CrossT2).cross(mFrictionConstraints[mNbFrictionConstraints].r2Friction)).dot(
                                    mFrictionConstraints[mNbFrictionConstraints].frictionVector2);
            decimal frictionTwistMass = mFrictionConstraints[mNbFrictionConstraints].normal.dot(mFrictionConstraints[mNbFrictionConstraints].inverseInertiaTensorBody1 *
                                           mFrictionConstraints[mNbFrictionConstraints].normal) +
                                        mFrictionConstraints[mNbFrictionConstraints].normal.dot(mFrictionConstraints[mNbFrictionConstraints].inverseInertiaTensorBody2 *
                                           mFrictionConstraints[mNbFrictionConstraints].normal);
            friction1Mass > decimal(0.0) ? mFrictionConstraints[mNbFrictionConstraints].inverseFriction1Mass = decimal(1.0)/friction1Mass
                                                                         : decimal(0.0);
            friction2Mass > decimal(0.0) ? mFrictionConstraints[mNbFrictionConstraints].inverseFriction2Mass = decimal(1.0)/friction2Mass
                                                                         : decimal(0.0);
            frictionTwistMass > decimal(0.0) ? mFrictionConstraints[mNbFrictionConstraints].inverseTwistFrictionMass = decimal(1.0) /
                                                                                 frictionTwistMass :
                                                                                 decimal(0.0);

        mNbFrictionConstraints++;
    }
}

// Solve the contact constraints of one iteration of the solve
void ContactSolver::solve() {

    assert(mTimeStep > decimal(0.0));

    resetTotalPenetrationImpulse();
    solvePenetrationConstraints();
    solveFrictionConstraints();
}

// Warm start the solver.
/// For each constraint, we apply the previous impulse (from the previous step)
/// at the beginning. With this technique, we will converge faster towards
/// the solution of the linear system
void ContactSolver::warmStart() {

    PROFILE("ContactSolver::warmStart()");

    // Penetration constraints
    for (uint i=0; i<mNbPenetrationConstraints; i++) {

        // If it is not a new contact (this contact was already existing at last time step)
        if (mPenetrationConstraints[i].isRestingContact) {

            Vector3 linearImpulse = mPenetrationConstraints[i].normal * mPenetrationConstraints[i].penetrationImpulse;

            // Update the velocities of the body 1 by applying the impulse P
            mLinearVelocities[mPenetrationConstraints[i].indexBody1] += mPenetrationConstraints[i].massInverseBody1 *
                                                      (-linearImpulse);
            mAngularVelocities[mPenetrationConstraints[i].indexBody1] += mPenetrationConstraints[i].inverseInertiaTensorBody1 *
                                                       (-mPenetrationConstraints[i].r1CrossN * mPenetrationConstraints[i].penetrationImpulse);

            // Update the velocities of the body 1 by applying the impulse P
            mLinearVelocities[mPenetrationConstraints[i].indexBody2] += mPenetrationConstraints[i].massInverseBody2 *
                                                      linearImpulse;
            mAngularVelocities[mPenetrationConstraints[i].indexBody2] += mPenetrationConstraints[i].inverseInertiaTensorBody2 *
                                                       (-mPenetrationConstraints[i].r1CrossN * mPenetrationConstraints[i].penetrationImpulse);

        }
        else {  // If it is a new contact point

            // Initialize the accumulated impulses to zero
            mPenetrationConstraints[i].penetrationImpulse = 0.0;
        }
    }

    // Friction constraints
    for (uint i=0; i<mNbFrictionConstraints; i++) {

        // If we solve the friction constraints at the center of the contact manifold and there is
        // at least one resting contact point in the contact manifold
        if (mFrictionConstraints[i].hasAtLeastOneRestingContactPoint) {

            // Project the old friction impulses (with old friction vectors) into the new friction
            // vectors to get the new friction impulses
            Vector3 oldFrictionImpulse = mFrictionConstraints[i].friction1Impulse * mFrictionConstraints[i].oldFrictionVector1 +
                                         mFrictionConstraints[i].friction2Impulse * mFrictionConstraints[i].oldFrictionVector2;
            mFrictionConstraints[i].friction1Impulse = oldFrictionImpulse.dot(mFrictionConstraints[i].frictionVector1);
            mFrictionConstraints[i].friction2Impulse = oldFrictionImpulse.dot(mFrictionConstraints[i].frictionVector2);

            // ------ First friction constraint at the center of the contact manifold ------ //

            // Compute the impulse P = J^T * lambda
            Vector3 linearImpulseBody2 = mFrictionConstraints[i].frictionVector1 * mFrictionConstraints[i].friction1Impulse;
            Vector3 angularImpulseBody1 = -mFrictionConstraints[i].r1CrossT1 * mFrictionConstraints[i].friction1Impulse;
            Vector3 angularImpulseBody2 = mFrictionConstraints[i].r2CrossT1 * mFrictionConstraints[i].friction1Impulse;

            // Update the velocities of the body 1 by applying the impulse P
            mLinearVelocities[mFrictionConstraints[i].indexBody1] += mFrictionConstraints[i].massInverseBody1 * (-linearImpulseBody2);
            mAngularVelocities[mFrictionConstraints[i].indexBody1] += mFrictionConstraints[i].inverseInertiaTensorBody1 * angularImpulseBody1;

            // Update the velocities of the body 1 by applying the impulse P
            mLinearVelocities[mFrictionConstraints[i].indexBody2] += mFrictionConstraints[i].massInverseBody2 * linearImpulseBody2;
            mAngularVelocities[mFrictionConstraints[i].indexBody2] += mFrictionConstraints[i].inverseInertiaTensorBody2 * angularImpulseBody2;

            // ------ Second friction constraint at the center of the contact manifold ----- //

            // Compute the impulse P = J^T * lambda
            angularImpulseBody1 = -mFrictionConstraints[i].r1CrossT2 * mFrictionConstraints[i].friction2Impulse;
            linearImpulseBody2 = mFrictionConstraints[i].frictionVector2 * mFrictionConstraints[i].friction2Impulse;
            angularImpulseBody2 = mFrictionConstraints[i].r2CrossT2 * mFrictionConstraints[i].friction2Impulse;

            // Update the velocities of the body 1 by applying the impulse P
            mLinearVelocities[mFrictionConstraints[i].indexBody1] += mFrictionConstraints[i].massInverseBody1 * (-linearImpulseBody2);
            mAngularVelocities[mFrictionConstraints[i].indexBody1] += mFrictionConstraints[i].inverseInertiaTensorBody1 * angularImpulseBody1;

            // Update the velocities of the body 1 by applying the impulse P
            mLinearVelocities[mFrictionConstraints[i].indexBody2] += mFrictionConstraints[i].massInverseBody2 * linearImpulseBody2;
            mAngularVelocities[mFrictionConstraints[i].indexBody2] += mFrictionConstraints[i].inverseInertiaTensorBody2 * angularImpulseBody2;

            // ------ Twist friction constraint at the center of the contact manifold ------ //

            // Compute the impulse P = J^T * lambda
            angularImpulseBody2 = mFrictionConstraints[i].normal * mFrictionConstraints[i].frictionTwistImpulse;

            // Update the velocities of the body 1 by applying the impulse P
            mAngularVelocities[mFrictionConstraints[i].indexBody1] += mFrictionConstraints[i].inverseInertiaTensorBody1 * (-angularImpulseBody2);

            // Update the velocities of the body 1 by applying the impulse P
            mAngularVelocities[mFrictionConstraints[i].indexBody2] += mFrictionConstraints[i].inverseInertiaTensorBody2 * angularImpulseBody2;

            // ------ Rolling resistance at the center of the contact manifold ------ //

            // Compute the impulse P = J^T * lambda
            angularImpulseBody2 = mFrictionConstraints[i].rollingResistanceImpulse;

            // Update the velocities of the body 1 by applying the impulse P
            mAngularVelocities[mFrictionConstraints[i].indexBody1] += mFrictionConstraints[i].inverseInertiaTensorBody1 * (-angularImpulseBody2);

            // Update the velocities of the body 1 by applying the impulse P
            mAngularVelocities[mFrictionConstraints[i].indexBody2] += mFrictionConstraints[i].inverseInertiaTensorBody2 * angularImpulseBody2;
        }
        else {  // If it is a new contact manifold

            // Initialize the accumulated impulses to zero
            mFrictionConstraints[i].friction1Impulse = 0.0;
            mFrictionConstraints[i].friction2Impulse = 0.0;
            mFrictionConstraints[i].frictionTwistImpulse = 0.0;
            mFrictionConstraints[i].rollingResistanceImpulse.setToZero();
        }
    }
}

// Reset the total penetration impulse of friction constraints
void ContactSolver::resetTotalPenetrationImpulse() {

    for (uint i=0; i<mNbFrictionConstraints; i++) {
        mFrictionConstraints[i].totalPenetrationImpulse = decimal(0.0);
    }
}

// Solve the penetration constraints
void ContactSolver::solvePenetrationConstraints() {

    PROFILE("ContactSolver::solvePenetrationConstraints()");

    // TODO : Check that the PenetrationConstraint struct only contains variables that are
    //        used in this method, nothing more

    // TODO : Maybe solve split impulses and normal impulses separately

    decimal deltaLambda;
    decimal lambdaTemp;

    for (uint i=0; i<mNbPenetrationConstraints; i++) {

        // Get the constrained velocities
        Vector3& v1 = mLinearVelocities[mPenetrationConstraints[i].indexBody1];
        Vector3& w1 = mAngularVelocities[mPenetrationConstraints[i].indexBody1];
        Vector3& v2 = mLinearVelocities[mPenetrationConstraints[i].indexBody2];
        Vector3& w2 = mAngularVelocities[mPenetrationConstraints[i].indexBody2];

        // Compute J*v
        Vector3 deltaV = v2 + w2.cross(mPenetrationConstraints[i].r2) - v1 - w1.cross(mPenetrationConstraints[i].r1);
        decimal deltaVDotN = deltaV.dot(mPenetrationConstraints[i].normal);
        decimal Jv = deltaVDotN;

        // Compute the bias "b" of the constraint
        decimal beta = mIsSplitImpulseActive ? BETA_SPLIT_IMPULSE : BETA;
        decimal biasPenetrationDepth = 0.0;
        if (mPenetrationConstraints[i].penetrationDepth > SLOP) biasPenetrationDepth = -(beta/mTimeStep) *
                max(0.0f, float(mPenetrationConstraints[i].penetrationDepth - SLOP));
        decimal b = biasPenetrationDepth + mPenetrationConstraints[i].restitutionBias;

        // Compute the Lagrange multiplier lambda
        if (mIsSplitImpulseActive) {
            deltaLambda = - (Jv + mPenetrationConstraints[i].restitutionBias) *
                    mPenetrationConstraints[i].inversePenetrationMass;
        }
        else {
            deltaLambda = - (Jv + b) * mPenetrationConstraints[i].inversePenetrationMass;
        }
        lambdaTemp = mPenetrationConstraints[i].penetrationImpulse;
        mPenetrationConstraints[i].penetrationImpulse = std::max(mPenetrationConstraints[i].penetrationImpulse +
                                                   deltaLambda, decimal(0.0));
        deltaLambda = mPenetrationConstraints[i].penetrationImpulse - lambdaTemp;

        // Add the penetration impulse to the total impulse of the corresponding friction constraint
        mFrictionConstraints[mPenetrationConstraints[i].indexFrictionConstraint].totalPenetrationImpulse += mPenetrationConstraints[i].penetrationImpulse;

        // Update the velocities of the body 1 by applying the impulse P=J^T * lambda
        Vector3 linearImpulse = mPenetrationConstraints[i].normal * deltaLambda;
        v1 += mPenetrationConstraints[i].massInverseBody1 * (-linearImpulse);
        w1 += mPenetrationConstraints[i].inverseInertiaTensorBody1 * (-mPenetrationConstraints[i].r1CrossN * deltaLambda);

        // Update the velocities of the body 1 by applying the impulse P=J^T * lambda
        v2 += mPenetrationConstraints[i].massInverseBody2 * linearImpulse;
        w2 += mPenetrationConstraints[i].inverseInertiaTensorBody2 * (mPenetrationConstraints[i].r2CrossN * deltaLambda);

        // If the split impulse position correction is active
        if (mIsSplitImpulseActive) {

            // Split impulse (position correction)
            const Vector3& v1Split = mSplitLinearVelocities[mPenetrationConstraints[i].indexBody1];
            const Vector3& w1Split = mSplitAngularVelocities[mPenetrationConstraints[i].indexBody1];
            const Vector3& v2Split = mSplitLinearVelocities[mPenetrationConstraints[i].indexBody2];
            const Vector3& w2Split = mSplitAngularVelocities[mPenetrationConstraints[i].indexBody2];
            Vector3 deltaVSplit = v2Split + w2Split.cross(mPenetrationConstraints[i].r2) -
                    v1Split - w1Split.cross(mPenetrationConstraints[i].r1);
            decimal JvSplit = deltaVSplit.dot(mPenetrationConstraints[i].normal);
            decimal deltaLambdaSplit = - (JvSplit + biasPenetrationDepth) *
                    mPenetrationConstraints[i].inversePenetrationMass;
            decimal lambdaTempSplit = mPenetrationConstraints[i].penetrationSplitImpulse;
            mPenetrationConstraints[i].penetrationSplitImpulse = std::max(
                        mPenetrationConstraints[i].penetrationSplitImpulse +
                        deltaLambdaSplit, decimal(0.0));
            deltaLambdaSplit = mPenetrationConstraints[i].penetrationSplitImpulse - lambdaTempSplit;

            // Update the velocities of the body 1 by applying the impulse P=J^T * lambda
            Vector3 linearImpulse = mPenetrationConstraints[i].normal * deltaLambdaSplit;
            mSplitLinearVelocities[mPenetrationConstraints[i].indexBody1] += mPenetrationConstraints[i].massInverseBody1 * (-linearImpulse);
            mSplitAngularVelocities[mPenetrationConstraints[i].indexBody1] += mPenetrationConstraints[i].inverseInertiaTensorBody1 * (-mPenetrationConstraints[i].r1CrossN * deltaLambdaSplit);

            // Update the velocities of the body 1 by applying the impulse P=J^T * lambda
            mSplitLinearVelocities[mPenetrationConstraints[i].indexBody2] += mPenetrationConstraints[i].massInverseBody2 * linearImpulse;
            mSplitAngularVelocities[mPenetrationConstraints[i].indexBody2] += mPenetrationConstraints[i].inverseInertiaTensorBody2 * (mPenetrationConstraints[i].r2CrossN * deltaLambdaSplit);
        }
    }
}

// Solve the friction constraints
void ContactSolver::solveFrictionConstraints() {

    // TODO : Check that the FrictionConstraint struct only contains variables that are
    //        used in this method, nothing more

    PROFILE("ContactSolver::solveFrictionConstraints()");

    for (uint i=0; i<mNbFrictionConstraints; i++) {

        // Get the constrained velocities
        Vector3& v1 = mLinearVelocities[mFrictionConstraints[i].indexBody1];
        Vector3& w1 = mAngularVelocities[mFrictionConstraints[i].indexBody1];
        Vector3& v2 = mLinearVelocities[mFrictionConstraints[i].indexBody2];
        Vector3& w2 = mAngularVelocities[mFrictionConstraints[i].indexBody2];

        // ------ First friction constraint at the center of the contact manifol ------ //

        // Compute J*v
        Vector3 deltaV = v2 + w2.cross(mFrictionConstraints[i].r2Friction)
                - v1 - w1.cross(mFrictionConstraints[i].r1Friction);
        decimal Jv = deltaV.dot(mFrictionConstraints[i].frictionVector1);

        // Compute the Lagrange multiplier lambda
        decimal deltaLambda = -Jv * mFrictionConstraints[i].inverseFriction1Mass;
        decimal frictionLimit = mFrictionConstraints[i].frictionCoefficient * mFrictionConstraints[i].totalPenetrationImpulse;
        decimal lambdaTemp = mFrictionConstraints[i].friction1Impulse;
        mFrictionConstraints[i].friction1Impulse = std::max(-frictionLimit,
                                                    std::min(mFrictionConstraints[i].friction1Impulse +
                                                             deltaLambda, frictionLimit));
        deltaLambda = mFrictionConstraints[i].friction1Impulse - lambdaTemp;

        // Compute the impulse P=J^T * lambda
        Vector3 linearImpulseBody2 = mFrictionConstraints[i].frictionVector1 * deltaLambda;
        Vector3 linearImpulseBody1 = -linearImpulseBody2;
        Vector3 angularImpulseBody1 = -mFrictionConstraints[i].r1CrossT1 * deltaLambda;
        Vector3 angularImpulseBody2 = mFrictionConstraints[i].r2CrossT1 * deltaLambda;

        // Update the velocities of the body 1 by applying the impulse P
        v1 += mFrictionConstraints[i].massInverseBody1 * linearImpulseBody1;
        w1 += mFrictionConstraints[i].inverseInertiaTensorBody1 * angularImpulseBody1;

        // Update the velocities of the body 1 by applying the impulse P
        v2 += mFrictionConstraints[i].massInverseBody2 * linearImpulseBody2;
        w2 += mFrictionConstraints[i].inverseInertiaTensorBody2 * angularImpulseBody2;

        // ------ Second friction constraint at the center of the contact manifol ----- //

        // Compute J*v
        deltaV = v2 + w2.cross(mFrictionConstraints[i].r2Friction)
                - v1 - w1.cross(mFrictionConstraints[i].r1Friction);
        Jv = deltaV.dot(mFrictionConstraints[i].frictionVector2);

        // Compute the Lagrange multiplier lambda
        deltaLambda = -Jv * mFrictionConstraints[i].inverseFriction2Mass;
        frictionLimit = mFrictionConstraints[i].frictionCoefficient * mFrictionConstraints[i].totalPenetrationImpulse;
        lambdaTemp = mFrictionConstraints[i].friction2Impulse;
        mFrictionConstraints[i].friction2Impulse = std::max(-frictionLimit,
                                                   std::min(mFrictionConstraints[i].friction2Impulse +
                                                             deltaLambda, frictionLimit));
        deltaLambda = mFrictionConstraints[i].friction2Impulse - lambdaTemp;

        // Compute the impulse P=J^T * lambda
        linearImpulseBody2 = mFrictionConstraints[i].frictionVector2 * deltaLambda;
        linearImpulseBody1 = -linearImpulseBody2;
        angularImpulseBody1 = -mFrictionConstraints[i].r1CrossT2 * deltaLambda;
        angularImpulseBody2 = mFrictionConstraints[i].r2CrossT2 * deltaLambda;

        // Update the velocities of the body 1 by applying the impulse P
        v1 += mFrictionConstraints[i].massInverseBody1 * linearImpulseBody1;
        w1 += mFrictionConstraints[i].inverseInertiaTensorBody1 * angularImpulseBody1;

        // Update the velocities of the body 1 by applying the impulse P
        v2 += mFrictionConstraints[i].massInverseBody2 * linearImpulseBody2;
        w2 += mFrictionConstraints[i].inverseInertiaTensorBody2 * angularImpulseBody2;

        // ------ Twist friction constraint at the center of the contact manifol ------ //

        // Compute J*v
        deltaV = w2 - w1;
        Jv = deltaV.dot(mFrictionConstraints[i].normal);

        deltaLambda = -Jv * (mFrictionConstraints[i].inverseTwistFrictionMass);
        frictionLimit = mFrictionConstraints[i].frictionCoefficient * mFrictionConstraints[i].totalPenetrationImpulse;
        lambdaTemp = mFrictionConstraints[i].frictionTwistImpulse;
        mFrictionConstraints[i].frictionTwistImpulse = std::max(-frictionLimit,
                                                        std::min(mFrictionConstraints[i].frictionTwistImpulse
                                                                 + deltaLambda, frictionLimit));
        deltaLambda = mFrictionConstraints[i].frictionTwistImpulse - lambdaTemp;

        // Compute the impulse P=J^T * lambda
        linearImpulseBody1 = Vector3(0.0, 0.0, 0.0);
        linearImpulseBody2 = Vector3(0.0, 0.0, 0.0);
        angularImpulseBody2 = mFrictionConstraints[i].normal * deltaLambda;
        angularImpulseBody1 = -angularImpulseBody2;

        // Update the velocities of the body 1 by applying the impulse P
        v1 += mFrictionConstraints[i].massInverseBody1 * linearImpulseBody1;
        w1 += mFrictionConstraints[i].inverseInertiaTensorBody1 * angularImpulseBody1;

        // Update the velocities of the body 1 by applying the impulse P
        v2 += mFrictionConstraints[i].massInverseBody2 * linearImpulseBody2;
        w2 += mFrictionConstraints[i].inverseInertiaTensorBody2 * angularImpulseBody2;

        // --------- Rolling resistance constraint at the center of the contact manifold --------- //

        if (mFrictionConstraints[i].rollingResistanceFactor > 0) {

            // Compute J*v
            const Vector3 JvRolling = w2 - w1;

            // Compute the Lagrange multiplier lambda
            Vector3 deltaLambdaRolling = mFrictionConstraints[i].inverseRollingResistance * (-JvRolling);
            decimal rollingLimit = mFrictionConstraints[i].rollingResistanceFactor * mFrictionConstraints[i].totalPenetrationImpulse;
            Vector3 lambdaTempRolling = mFrictionConstraints[i].rollingResistanceImpulse;
            mFrictionConstraints[i].rollingResistanceImpulse = clamp(mFrictionConstraints[i].rollingResistanceImpulse +
                                                                 deltaLambdaRolling, rollingLimit);
            deltaLambdaRolling = mFrictionConstraints[i].rollingResistanceImpulse - lambdaTempRolling;

            // Compute the impulse P=J^T * lambda
            angularImpulseBody1 = -deltaLambdaRolling;
            angularImpulseBody2 = deltaLambdaRolling;

            // Update the velocities of the body 1 by applying the impulse P
            w1 += mFrictionConstraints[i].inverseInertiaTensorBody1 * angularImpulseBody1;

            // Update the velocities of the body 1 by applying the impulse P
            w2 += mFrictionConstraints[i].inverseInertiaTensorBody2 * angularImpulseBody2;
        }
    }
}

// Store the computed impulses to use them to
// warm start the solver at the next iteration
void ContactSolver::storeImpulses() {

    // Penetration constraints
    for (uint i=0; i<mNbPenetrationConstraints; i++) {
        mPenetrationConstraints[i].contactPoint->setPenetrationImpulse(mPenetrationConstraints[i].penetrationImpulse);
    }

    // Friction constraints
    for (uint i=0; i<mNbFrictionConstraints; i++) {

        mFrictionConstraints[i].contactManifold->setFrictionImpulse1(mFrictionConstraints[i].friction1Impulse);
        mFrictionConstraints[i].contactManifold->setFrictionImpulse2(mFrictionConstraints[i].friction2Impulse);
        mFrictionConstraints[i].contactManifold->setFrictionTwistImpulse(mFrictionConstraints[i].frictionTwistImpulse);
        mFrictionConstraints[i].contactManifold->setRollingResistanceImpulse(mFrictionConstraints[i].rollingResistanceImpulse);
        mFrictionConstraints[i].contactManifold->setFrictionVector1(mFrictionConstraints[i].frictionVector1);
        mFrictionConstraints[i].contactManifold->setFrictionVector2(mFrictionConstraints[i].frictionVector2);
    }

    /*
    if (mPenetrationConstraints != nullptr) {
    // TODO : Delete this
        delete[] mPenetrationConstraints;
    }

    if (mFrictionConstraints != nullptr) {
        delete[] mFrictionConstraints;
    }
    */
}

// Compute the two unit orthogonal vectors "t1" and "t2" that span the tangential friction plane
// for a contact manifold. The two vectors have to be such that : t1 x t2 = contactNormal.
void ContactSolver::computeFrictionVectors(const Vector3& deltaVelocity,
                                           FrictionConstraint& frictionConstraint) const {

    assert(frictionConstraint.normal.length() > MACHINE_EPSILON);

    // Compute the velocity difference vector in the tangential plane
    Vector3 normalVelocity = deltaVelocity.dot(frictionConstraint.normal) * frictionConstraint.normal;
    Vector3 tangentVelocity = deltaVelocity - normalVelocity;

    // If the velocty difference in the tangential plane is not zero
    decimal lengthTangenVelocity = tangentVelocity.length();
    if (lengthTangenVelocity > MACHINE_EPSILON) {

        // Compute the first friction vector in the direction of the tangent
        // velocity difference
        frictionConstraint.frictionVector1 = tangentVelocity / lengthTangenVelocity;
    }
    else {

        // Get any orthogonal vector to the normal as the first friction vector
        frictionConstraint.frictionVector1 = frictionConstraint.normal.getOneUnitOrthogonalVector();
    }

    // The second friction vector is computed by the cross product of the firs
    // friction vector and the contact normal
    frictionConstraint.frictionVector2 = frictionConstraint.normal.cross(frictionConstraint.frictionVector1).getUnit();
}
