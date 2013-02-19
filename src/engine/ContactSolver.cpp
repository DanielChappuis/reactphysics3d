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
#include "ContactSolver.h"
#include "DynamicsWorld.h"
#include "../body/RigidBody.h"

using namespace reactphysics3d;
using namespace std;

// Constants initialization
const decimal ContactSolver::BETA = decimal(0.2);
const decimal ContactSolver::BETA_SPLIT_IMPULSE = decimal(0.2);
const decimal ContactSolver::SLOP = decimal(0.01);

// Constructor
ContactSolver::ContactSolver(DynamicsWorld& world)
    :mWorld(world), mNbIterations(DEFAULT_CONSTRAINTS_SOLVER_NB_ITERATIONS), mContactConstraints(0),
      mLinearVelocities(0), mAngularVelocities(0), mIsWarmStartingActive(true),
      mIsSplitImpulseActive(true), mIsSolveFrictionAtContactManifoldCenterActive(true) {

}

// Destructor
ContactSolver::~ContactSolver() {

}

// Initialize the constraint solver
void ContactSolver::initialize() {

    // TODO : Use better memory allocation here
    mContactConstraints = new ContactManifoldSolver[mWorld.getNbContactManifolds()];

    mNbContactConstraints = 0;

    // For each contact manifold of the world
    vector<ContactManifold>::iterator it;
    for (it = mWorld.getContactManifoldsBeginIterator();
         it != mWorld.getContactManifoldsEndIterator(); ++it) {

        ContactManifold& externalContactManifold = *it;

        ContactManifoldSolver& internalContactManifold = mContactConstraints[mNbContactConstraints];

        assert(externalContactManifold.nbContacts > 0);

        // Get the two bodies of the contact
        RigidBody* body1 = externalContactManifold.contacts[0]->getBody1();
        RigidBody* body2 = externalContactManifold.contacts[0]->getBody2();

        // Fill in the body number maping
        mMapBodyToIndex.insert(make_pair(body1, mMapBodyToIndex.size()));
        mMapBodyToIndex.insert(make_pair(body2, mMapBodyToIndex.size()));

        // Add the two bodies of the constraint in the constraintBodies list
        mConstraintBodies.insert(body1);
        mConstraintBodies.insert(body2);

        // Get the position of the two bodies
        Vector3 x1 = body1->getTransform().getPosition();
        Vector3 x2 = body2->getTransform().getPosition();

        internalContactManifold.indexBody1 = mMapBodyToIndex[body1];
        internalContactManifold.indexBody2 = mMapBodyToIndex[body2];
        internalContactManifold.inverseInertiaTensorBody1 = body1->getInertiaTensorInverseWorld();
        internalContactManifold.inverseInertiaTensorBody2 = body2->getInertiaTensorInverseWorld();
        internalContactManifold.isBody1Moving = body1->getIsMotionEnabled();
        internalContactManifold.isBody2Moving = body2->getIsMotionEnabled();
        internalContactManifold.massInverseBody1 = body1->getMassInverse();
        internalContactManifold.massInverseBody2 = body2->getMassInverse();
        internalContactManifold.nbContacts = externalContactManifold.nbContacts;
        internalContactManifold.restitutionFactor = computeMixedRestitutionFactor(body1, body2);
        internalContactManifold.frictionCoefficient = computeMixedFrictionCoefficient(body1, body2);
        internalContactManifold.contactManifold = &(*it);

        // If we solve the friction constraints at the center of the contact manifold
        if (mIsSolveFrictionAtContactManifoldCenterActive) {
            internalContactManifold.frictionPointBody1 = Vector3(0.0, 0.0, 0.0);
            internalContactManifold.frictionPointBody2 = Vector3(0.0, 0.0, 0.0);
        }

        // For each contact point of the contact manifold
        for (uint c=0; c<externalContactManifold.nbContacts; c++) {

            ContactPointSolver& contactPoint = internalContactManifold.contacts[c];

            // Get a contact point
            Contact* externalContact = externalContactManifold.contacts[c];

            // Get the contact point on the two bodies
            Vector3 p1 = externalContact->getWorldPointOnBody1();
            Vector3 p2 = externalContact->getWorldPointOnBody2();

            contactPoint.contact = externalContact;
            contactPoint.normal = externalContact->getNormal();
            contactPoint.r1 = p1 - x1;
            contactPoint.r2 = p2 - x2;
            contactPoint.penetrationDepth = externalContact->getPenetrationDepth();
            contactPoint.isRestingContact = externalContact->getIsRestingContact();
            externalContact->setIsRestingContact(true);
            contactPoint.oldFrictionVector1 = externalContact->getFrictionVector1();
            contactPoint.oldFrictionVector2 = externalContact->getFrictionVector2();
            contactPoint.penetrationImpulse = 0.0;
            contactPoint.friction1Impulse = 0.0;
            contactPoint.friction2Impulse = 0.0;

            // If we solve the friction constraints at the center of the contact manifold
            if (mIsSolveFrictionAtContactManifoldCenterActive) {
                internalContactManifold.frictionPointBody1 += p1;
                internalContactManifold.frictionPointBody2 += p2;
            }
        }

        // If we solve the friction constraints at the center of the contact manifold
        if (mIsSolveFrictionAtContactManifoldCenterActive) {

            internalContactManifold.frictionPointBody1 /= static_cast<decimal>(internalContactManifold.nbContacts);
            internalContactManifold.frictionPointBody2 /= static_cast<decimal>(internalContactManifold.nbContacts);
            internalContactManifold.r1Friction = internalContactManifold.frictionPointBody1 - x1;
            internalContactManifold.r2Friction = internalContactManifold.frictionPointBody2 - x2;
            internalContactManifold.oldFrictionVector1 = externalContactManifold.frictionVector1;
            internalContactManifold.oldFrictionVector2 = externalContactManifold.frictionVector2;

            // If warm starting is active
            if (mIsWarmStartingActive) {

                // Initialize the accumulated impulses with the previous step accumulated impulses
                internalContactManifold.friction1Impulse = externalContactManifold.friction1Impulse;
                internalContactManifold.friction2Impulse = externalContactManifold.friction2Impulse;
                internalContactManifold.frictionTwistImpulse = externalContactManifold.frictionTwistImpulse;
            }
            else {

                // Initialize the accumulated impulses to zero
                internalContactManifold.friction1Impulse = 0.0;
                internalContactManifold.friction2Impulse = 0.0;
                internalContactManifold.frictionTwistImpulse = 0.0;
            }
        }

        mNbContactConstraints++;
    }

    // Compute the number of bodies that are part of some active constraint
    uint nbBodies = mConstraintBodies.size();

    // Allocated memory for velocities
    // TODO : Use better memory allocation here
    mLinearVelocities = new Vector3[nbBodies];
    mAngularVelocities = new Vector3[nbBodies];
    mSplitLinearVelocities = new Vector3[nbBodies];
    mSplitAngularVelocities = new Vector3[nbBodies];

    assert(mMapBodyToIndex.size() == nbBodies);
}

// Initialize the constrained bodies
void ContactSolver::initializeBodies() {

    // For each current body that is implied in some constraint
    set<RigidBody*>::iterator it;
    for (it = mConstraintBodies.begin(); it != mConstraintBodies.end(); ++it) {
        RigidBody* rigidBody = *it;
        assert(rigidBody);

        uint bodyNumber = mMapBodyToIndex[rigidBody];

        mLinearVelocities[bodyNumber] = rigidBody->getLinearVelocity() + mTimeStep * rigidBody->getMassInverse() * rigidBody->getExternalForce();
        mAngularVelocities[bodyNumber] = rigidBody->getAngularVelocity() + mTimeStep * rigidBody->getInertiaTensorInverseWorld() * rigidBody->getExternalTorque();
        mSplitLinearVelocities[bodyNumber] = Vector3(0, 0, 0);
        mSplitAngularVelocities[bodyNumber] = Vector3(0, 0, 0);
    }
}

// Initialize the contact constraints before solving the system
void ContactSolver::initializeContactConstraints() {
    
    // For each contact constraint
    for (uint c=0; c<mNbContactConstraints; c++) {

        ContactManifoldSolver& contactManifold = mContactConstraints[c];

        // Get the inertia tensors of both bodies
        Matrix3x3& I1 = contactManifold.inverseInertiaTensorBody1;
        Matrix3x3& I2 = contactManifold.inverseInertiaTensorBody2;

        // If we solve the friction constraints at the center of the contact manifold
        if (mIsSolveFrictionAtContactManifoldCenterActive) {
            contactManifold.normal = Vector3(0.0, 0.0, 0.0);
        }

        // Get the velocities of the bodies
        const Vector3& v1 = mLinearVelocities[contactManifold.indexBody1];
        const Vector3& w1 = mAngularVelocities[contactManifold.indexBody1];
        const Vector3& v2 = mLinearVelocities[contactManifold.indexBody2];
        const Vector3& w2 = mAngularVelocities[contactManifold.indexBody2];

        // For each contact point constraint
        for (uint i=0; i<contactManifold.nbContacts; i++) {

            ContactPointSolver& contactPoint = contactManifold.contacts[i];
            Contact* externalContact = contactPoint.contact;

            // Compute the velocity difference
            Vector3 deltaV = v2 + w2.cross(contactPoint.r2) - v1 - w1.cross(contactPoint.r1);

            contactPoint.r1CrossN = contactPoint.r1.cross(contactPoint.normal);
            contactPoint.r2CrossN = contactPoint.r2.cross(contactPoint.normal);

            decimal massPenetration = 0.0;
            if (contactManifold.isBody1Moving) {
                massPenetration += contactManifold.massInverseBody1 +
                        ((I1 * contactPoint.r1CrossN).cross(contactPoint.r1)).dot(contactPoint.normal);
            }
            if (contactManifold.isBody2Moving) {
                massPenetration += contactManifold.massInverseBody2 +
                        ((I2 * contactPoint.r2CrossN).cross(contactPoint.r2)).dot(contactPoint.normal);
            }
            massPenetration > 0.0 ? contactPoint.inversePenetrationMass = decimal(1.0) / massPenetration : decimal(0.0);

            if (!mIsSolveFrictionAtContactManifoldCenterActive) {

                // Compute the friction vectors
                computeFrictionVectors(deltaV, contactPoint);

                contactPoint.r1CrossT1 = contactPoint.r1.cross(contactPoint.frictionVector1);
                contactPoint.r1CrossT2 = contactPoint.r1.cross(contactPoint.frictionVector2);
                contactPoint.r2CrossT1 = contactPoint.r2.cross(contactPoint.frictionVector1);
                contactPoint.r2CrossT2 = contactPoint.r2.cross(contactPoint.frictionVector2);

                // Compute the inverse mass matrix K for the friction constraints at each contact point
                decimal friction1Mass = 0.0;
                decimal friction2Mass = 0.0;
                if (contactManifold.isBody1Moving) {
                    friction1Mass += contactManifold.massInverseBody1 + ((I1 * contactPoint.r1CrossT1).cross(contactPoint.r1)).dot(contactPoint.frictionVector1);
                    friction2Mass += contactManifold.massInverseBody1 + ((I1 * contactPoint.r1CrossT2).cross(contactPoint.r1)).dot(contactPoint.frictionVector2);
                }
                if (contactManifold.isBody2Moving) {
                    friction1Mass += contactManifold.massInverseBody2 + ((I2 * contactPoint.r2CrossT1).cross(contactPoint.r2)).dot(contactPoint.frictionVector1);
                    friction2Mass += contactManifold.massInverseBody2 + ((I2 * contactPoint.r2CrossT2).cross(contactPoint.r2)).dot(contactPoint.frictionVector2);
                }
                friction1Mass > 0.0 ? contactPoint.inverseFriction1Mass = decimal(1.0) / friction1Mass : decimal(0.0);
                friction2Mass > 0.0 ? contactPoint.inverseFriction2Mass = decimal(1.0) / friction2Mass : decimal(0.0);
            }

            // Compute the restitution velocity bias "b". We compute this here instead
            // of inside the solve() method because we need to use the velocity difference
            // at the beginning of the contact. Note that if it is a resting contact (normal velocity
            // under a given threshold), we don't add a restitution velocity bias
            contactPoint.restitutionBias = 0.0;
            decimal deltaVDotN = deltaV.dot(contactPoint.normal);
            // TODO : Use a constant here
            if (deltaVDotN < 1.0f) {
                contactPoint.restitutionBias = contactManifold.restitutionFactor * deltaVDotN;
            }

            // Get the cached lambda values of the constraint
            if (mIsWarmStartingActive) {
                contactPoint.penetrationImpulse = externalContact->getCachedLambda(0);
                contactPoint.friction1Impulse = externalContact->getCachedLambda(1);
                contactPoint.friction2Impulse = externalContact->getCachedLambda(2);
            }

            // Initialize the split impulses to zero
            contactPoint.penetrationSplitImpulse = 0.0;

            // If we solve the friction constraints at the center of the contact manifold
            if (mIsSolveFrictionAtContactManifoldCenterActive) {
                contactManifold.normal += contactPoint.normal;
            }
        }

        // If we solve the friction constraints at the center of the contact manifold
        if (mIsSolveFrictionAtContactManifoldCenterActive) {

            contactManifold.normal.normalize();

            Vector3 deltaVFrictionPoint = v2 + w2.cross(contactManifold.r2Friction) -
                                          v1 - w1.cross(contactManifold.r1Friction);

            // Compute the friction vectors
            computeFrictionVectors(deltaVFrictionPoint, contactManifold);

            // Compute the inverse mass matrix K for the friction constraints at the center of
            // the contact manifold
            contactManifold.r1CrossT1 = contactManifold.r1Friction.cross(contactManifold.frictionVector1);
            contactManifold.r1CrossT2 = contactManifold.r1Friction.cross(contactManifold.frictionVector2);
            contactManifold.r2CrossT1 = contactManifold.r2Friction.cross(contactManifold.frictionVector1);
            contactManifold.r2CrossT2 = contactManifold.r2Friction.cross(contactManifold.frictionVector2);
            decimal friction1Mass = 0.0;
            decimal friction2Mass = 0.0;
            if (contactManifold.isBody1Moving) {
                friction1Mass += contactManifold.massInverseBody1 + ((I1 * contactManifold.r1CrossT1).cross(contactManifold.r1Friction)).dot(contactManifold.frictionVector1);
                friction2Mass += contactManifold.massInverseBody1 + ((I1 * contactManifold.r1CrossT2).cross(contactManifold.r1Friction)).dot(contactManifold.frictionVector2);
            }
            if (contactManifold.isBody2Moving) {
                friction1Mass += contactManifold.massInverseBody2 + ((I2 * contactManifold.r2CrossT1).cross(contactManifold.r2Friction)).dot(contactManifold.frictionVector1);
                friction2Mass += contactManifold.massInverseBody2 + ((I2 * contactManifold.r2CrossT2).cross(contactManifold.r2Friction)).dot(contactManifold.frictionVector2);
            }
            friction1Mass > 0.0 ? contactManifold.inverseFriction1Mass = decimal(1.0) / friction1Mass : decimal(0.0);
            friction2Mass > 0.0 ? contactManifold.inverseFriction2Mass = decimal(1.0) / friction2Mass : decimal(0.0);
        }
    }
}

// Warm start the solver
// For each constraint, we apply the previous impulse (from the previous step)
// at the beginning. With this technique, we will converge faster towards
// the solution of the linear system
void ContactSolver::warmStart() {

    // For each constraint
    for (uint c=0; c<mNbContactConstraints; c++) {

        ContactManifoldSolver& contactManifold = mContactConstraints[c];

        bool atLeastOneRestingContactPoint = false;

        for (uint i=0; i<contactManifold.nbContacts; i++) {

            ContactPointSolver& contactPoint = contactManifold.contacts[i];

            // If it is not a new contact (this contact was already existing at last time step)
            if (contactPoint.isRestingContact) {

                atLeastOneRestingContactPoint = true;

                // --------- Penetration --------- //

                // Compute the impulse P=J^T * lambda
                const Impulse impulsePenetration = computePenetrationImpulse(contactPoint.penetrationImpulse,
                                                                             contactPoint);

                // Apply the impulse to the bodies of the constraint
                applyImpulse(impulsePenetration, contactManifold);

                // If we do not solve the friction constraints at the center of the contact manifold
                if (!mIsSolveFrictionAtContactManifoldCenterActive) {

                    // Project the old friction impulses (with old friction vectors) into the new friction
                    // vectors to get the new friction impulses
                    Vector3 oldFrictionImpulse = contactPoint.friction1Impulse * contactPoint.oldFrictionVector1 +
                            contactPoint.friction2Impulse * contactPoint.oldFrictionVector2;
                    contactPoint.friction1Impulse = oldFrictionImpulse.dot(contactPoint.frictionVector1);
                    contactPoint.friction2Impulse = oldFrictionImpulse.dot(contactPoint.frictionVector2);

                    // --------- Friction 1 --------- //

                    // Compute the impulse P=J^T * lambda
                    const Impulse impulseFriction1 = computeFriction1Impulse(contactPoint.friction1Impulse,
                                                                             contactPoint);

                    // Apply the impulses to the bodies of the constraint
                    applyImpulse(impulseFriction1, contactManifold);

                    // --------- Friction 2 --------- //

                    // Compute the impulse P=J^T * lambda
                   const Impulse impulseFriction2 = computeFriction2Impulse(contactPoint.friction2Impulse,
                                                                            contactPoint);

                    // Apply the impulses to the bodies of the constraint
                    applyImpulse(impulseFriction2, contactManifold);
                }
            }
            else {  // If it is a new contact point

                // Initialize the accumulated impulses to zero
                contactPoint.penetrationImpulse = 0.0;
                contactPoint.friction1Impulse = 0.0;
                contactPoint.friction2Impulse = 0.0;
            }
        }

        // If we solve the friction constraints at the center of the contact manifold and there is
        // at least one resting contact point in the contact manifold
        if (mIsSolveFrictionAtContactManifoldCenterActive && atLeastOneRestingContactPoint) {

            // Project the old friction impulses (with old friction vectors) into the new friction
            // vectors to get the new friction impulses
            Vector3 oldFrictionImpulse = contactManifold.friction1Impulse * contactManifold.oldFrictionVector1 +
                    contactManifold.friction2Impulse * contactManifold.oldFrictionVector2;
            contactManifold.friction1Impulse = oldFrictionImpulse.dot(contactManifold.frictionVector1);
            contactManifold.friction2Impulse = oldFrictionImpulse.dot(contactManifold.frictionVector2);

            // ------ First friction constraint at the center of the contact manifol ------ //

            // Compute the impulse P=J^T * lambda
            Vector3 linearImpulseBody1 = -contactManifold.frictionVector1 * contactManifold.friction1Impulse;
            Vector3 angularImpulseBody1 = -contactManifold.r1CrossT1 * contactManifold.friction1Impulse;
            Vector3 linearImpulseBody2 = contactManifold.frictionVector1 * contactManifold.friction1Impulse;
            Vector3 angularImpulseBody2 = contactManifold.r2CrossT1 * contactManifold.friction1Impulse;
            const Impulse impulseFriction1(linearImpulseBody1, angularImpulseBody1,
                                           linearImpulseBody2, angularImpulseBody2);

            // Apply the impulses to the bodies of the constraint
            applyImpulse(impulseFriction1, contactManifold);

            // ------ Second friction constraint at the center of the contact manifol ----- //

            // Compute the impulse P=J^T * lambda
            linearImpulseBody1 = -contactManifold.frictionVector2 * contactManifold.friction2Impulse;
            angularImpulseBody1 = -contactManifold.r1CrossT2 * contactManifold.friction2Impulse;
            linearImpulseBody2 = contactManifold.frictionVector2 * contactManifold.friction2Impulse;
            angularImpulseBody2 = contactManifold.r2CrossT2 * contactManifold.friction2Impulse;
            const Impulse impulseFriction2(linearImpulseBody1, angularImpulseBody1,
                                           linearImpulseBody2, angularImpulseBody2);

            // Apply the impulses to the bodies of the constraint
            applyImpulse(impulseFriction2, contactManifold);

            // ------ Twist friction constraint at the center of the contact manifol ------ //


            // Compute the impulse P=J^T * lambda
            linearImpulseBody1 = Vector3(0.0, 0.0, 0.0);
            angularImpulseBody1 = -contactManifold.normal * contactManifold.frictionTwistImpulse;
            linearImpulseBody2 = Vector3(0.0, 0.0, 0.0);
            angularImpulseBody2 = contactManifold.normal * contactManifold.frictionTwistImpulse;
            const Impulse impulseTwistFriction(linearImpulseBody1, angularImpulseBody1,
                                               linearImpulseBody2, angularImpulseBody2);

            // Apply the impulses to the bodies of the constraint
            applyImpulse(impulseTwistFriction, contactManifold);
        }
        else {  // If it is a new contact manifold

            // Initialize the accumulated impulses to zero
            contactManifold.friction1Impulse = 0.0;
            contactManifold.friction2Impulse = 0.0;
            contactManifold.frictionTwistImpulse = 0.0;
        }
    }
}

// Solve the contact constraints by applying sequential impulses
void ContactSolver::solveContactConstraints() {

    decimal deltaLambda;
    decimal lambdaTemp;
    uint iter;

    // For each iteration
    for(iter=0; iter<mNbIterations; iter++) {

        // For each constraint
        for (uint c=0; c<mNbContactConstraints; c++) {

            ContactManifoldSolver& contactManifold = mContactConstraints[c];

            decimal sumPenetrationImpulse = 0.0;

            const Vector3& v1 = mLinearVelocities[contactManifold.indexBody1];
            const Vector3& w1 = mAngularVelocities[contactManifold.indexBody1];
            const Vector3& v2 = mLinearVelocities[contactManifold.indexBody2];
            const Vector3& w2 = mAngularVelocities[contactManifold.indexBody2];

            for (uint i=0; i<contactManifold.nbContacts; i++) {

                ContactPointSolver& contactPoint = contactManifold.contacts[i];

                // --------- Penetration --------- //

                // Compute J*v
                Vector3 deltaV = v2 + w2.cross(contactPoint.r2) - v1 - w1.cross(contactPoint.r1);
                decimal deltaVDotN = deltaV.dot(contactPoint.normal);
                decimal Jv = deltaVDotN;

                // Compute the bias "b" of the constraint
                decimal beta = mIsSplitImpulseActive ? BETA_SPLIT_IMPULSE : BETA;
                decimal biasPenetrationDepth = 0.0;
                if (contactPoint.penetrationDepth > SLOP) biasPenetrationDepth = -(beta/mTimeStep) *
                        max(0.0f, float(contactPoint.penetrationDepth - SLOP));
                decimal b = biasPenetrationDepth + contactPoint.restitutionBias;

                // Compute the Lagrange multiplier
                if (mIsSplitImpulseActive) {
                    deltaLambda = - (Jv + contactPoint.restitutionBias) * contactPoint.inversePenetrationMass;
                }
                else {
                    deltaLambda = - (Jv + b) * contactPoint.inversePenetrationMass;
                }
                lambdaTemp = contactPoint.penetrationImpulse;
                contactPoint.penetrationImpulse = std::max(contactPoint.penetrationImpulse + deltaLambda, 0.0f);
                deltaLambda = contactPoint.penetrationImpulse - lambdaTemp;

                // Compute the impulse P=J^T * lambda
                const Impulse impulsePenetration = computePenetrationImpulse(deltaLambda, contactPoint);

                // Apply the impulse to the bodies of the constraint
                applyImpulse(impulsePenetration, contactManifold);

                sumPenetrationImpulse += contactPoint.penetrationImpulse;

                // If the split impulse position correction is active
                if (mIsSplitImpulseActive) {

                    // Split impulse (position correction)
                    const Vector3& v1Split = mSplitLinearVelocities[contactManifold.indexBody1];
                    const Vector3& w1Split = mSplitAngularVelocities[contactManifold.indexBody1];
                    const Vector3& v2Split = mSplitLinearVelocities[contactManifold.indexBody2];
                    const Vector3& w2Split = mSplitAngularVelocities[contactManifold.indexBody2];
                    Vector3 deltaVSplit = v2Split + w2Split.cross(contactPoint.r2) - v1Split - w1Split.cross(contactPoint.r1);
                    decimal JvSplit = deltaVSplit.dot(contactPoint.normal);
                    decimal deltaLambdaSplit = - (JvSplit + biasPenetrationDepth) * contactPoint.inversePenetrationMass;
                    decimal lambdaTempSplit = contactPoint.penetrationSplitImpulse;
                    contactPoint.penetrationSplitImpulse = std::max(contactPoint.penetrationSplitImpulse + deltaLambdaSplit, 0.0f);
                    deltaLambda = contactPoint.penetrationSplitImpulse - lambdaTempSplit;

                    // Compute the impulse P=J^T * lambda
                    const Impulse splitImpulsePenetration = computePenetrationImpulse(deltaLambdaSplit,
                                                                                      contactPoint);

                    applySplitImpulse(splitImpulsePenetration, contactManifold);
                }

                // If we do not solve the friction constraints at the center of the contact manifold
                if (!mIsSolveFrictionAtContactManifoldCenterActive) {

                    // --------- Friction 1 --------- //

                    // Compute J*v
                    deltaV = v2 + w2.cross(contactPoint.r2) - v1 - w1.cross(contactPoint.r1);
                    Jv = deltaV.dot(contactPoint.frictionVector1);

                    deltaLambda = -Jv;
                    deltaLambda *= contactPoint.inverseFriction1Mass;
                    decimal frictionLimit = contactManifold.frictionCoefficient * contactPoint.penetrationImpulse;
                    lambdaTemp = contactPoint.friction1Impulse;
                    contactPoint.friction1Impulse = std::max(-frictionLimit, std::min(contactPoint.friction1Impulse + deltaLambda, frictionLimit));
                    deltaLambda = contactPoint.friction1Impulse - lambdaTemp;

                    // Compute the impulse P=J^T * lambda
                    const Impulse impulseFriction1 = computeFriction1Impulse(deltaLambda, contactPoint);

                    // Apply the impulses to the bodies of the constraint
                    applyImpulse(impulseFriction1, contactManifold);

                    // --------- Friction 2 --------- //

                    // Compute J*v
                    deltaV = v2 + w2.cross(contactPoint.r2) - v1 - w1.cross(contactPoint.r1);
                    Jv = deltaV.dot(contactPoint.frictionVector2);

                    deltaLambda = -Jv;
                    deltaLambda *= contactPoint.inverseFriction2Mass;
                    frictionLimit = contactManifold.frictionCoefficient * contactPoint.penetrationImpulse;
                    lambdaTemp = contactPoint.friction2Impulse;
                    contactPoint.friction2Impulse = std::max(-frictionLimit, std::min(contactPoint.friction2Impulse + deltaLambda, frictionLimit));
                    deltaLambda = contactPoint.friction2Impulse - lambdaTemp;

                    // Compute the impulse P=J^T * lambda
                    const Impulse impulseFriction2 = computeFriction2Impulse(deltaLambda, contactPoint);

                    // Apply the impulses to the bodies of the constraint
                    applyImpulse(impulseFriction2, contactManifold);
                }
            }

            // If we solve the friction constraints at the center of the contact manifold
            if (mIsSolveFrictionAtContactManifoldCenterActive) {

                // ------ First friction constraint at the center of the contact manifol ------ //

                // Compute J*v
                Vector3 deltaV = v2 + w2.cross(contactManifold.r2Friction) - v1 - w1.cross(contactManifold.r1Friction);
                decimal Jv = deltaV.dot(contactManifold.frictionVector1);

                decimal deltaLambda = -Jv * contactManifold.inverseFriction1Mass;
                decimal frictionLimit = contactManifold.frictionCoefficient * sumPenetrationImpulse;
                lambdaTemp = contactManifold.friction1Impulse;
                contactManifold.friction1Impulse = std::max(-frictionLimit, std::min(contactManifold.friction1Impulse + deltaLambda, frictionLimit));
                deltaLambda = contactManifold.friction1Impulse - lambdaTemp;

                // Compute the impulse P=J^T * lambda
                Vector3 linearImpulseBody1 = -contactManifold.frictionVector1 * deltaLambda;
                Vector3 angularImpulseBody1 = -contactManifold.r1CrossT1 * deltaLambda;
                Vector3 linearImpulseBody2 = contactManifold.frictionVector1 * deltaLambda;
                Vector3 angularImpulseBody2 = contactManifold.r2CrossT1 * deltaLambda;
                const Impulse impulseFriction1(linearImpulseBody1, angularImpulseBody1,
                                               linearImpulseBody2, angularImpulseBody2);

                // Apply the impulses to the bodies of the constraint
                applyImpulse(impulseFriction1, contactManifold);

                // ------ Second friction constraint at the center of the contact manifol ----- //

                // Compute J*v
                deltaV = v2 + w2.cross(contactManifold.r2Friction) - v1 - w1.cross(contactManifold.r1Friction);
                Jv = deltaV.dot(contactManifold.frictionVector2);

                deltaLambda = -Jv * contactManifold.inverseFriction2Mass;
                frictionLimit = contactManifold.frictionCoefficient * sumPenetrationImpulse;
                lambdaTemp = contactManifold.friction2Impulse;
                contactManifold.friction2Impulse = std::max(-frictionLimit, std::min(contactManifold.friction2Impulse + deltaLambda, frictionLimit));
                deltaLambda = contactManifold.friction2Impulse - lambdaTemp;

                // Compute the impulse P=J^T * lambda
                linearImpulseBody1 = -contactManifold.frictionVector2 * deltaLambda;
                angularImpulseBody1 = -contactManifold.r1CrossT2 * deltaLambda;
                linearImpulseBody2 = contactManifold.frictionVector2 * deltaLambda;
                angularImpulseBody2 = contactManifold.r2CrossT2 * deltaLambda;
                const Impulse impulseFriction2(linearImpulseBody1, angularImpulseBody1,
                                               linearImpulseBody2, angularImpulseBody2);

                // Apply the impulses to the bodies of the constraint
                applyImpulse(impulseFriction2, contactManifold);

                // ------ Twist friction constraint at the center of the contact manifol ------ //

                // TODO : Put this in the initialization method
                decimal K = contactManifold.normal.dot(contactManifold.inverseInertiaTensorBody1 * contactManifold.normal) +
                           contactManifold.normal.dot(contactManifold.inverseInertiaTensorBody2 * contactManifold.normal);

                // Compute J*v
                deltaV = w2 - w1;
                Jv = deltaV.dot(contactManifold.normal);

                // TODO : Compute the inverse mass matrix here for twist friction
                deltaLambda = -Jv * (decimal(1.0) / K);
                frictionLimit = contactManifold.frictionCoefficient * sumPenetrationImpulse;
                lambdaTemp = contactManifold.frictionTwistImpulse;
                contactManifold.frictionTwistImpulse = std::max(-frictionLimit, std::min(contactManifold.frictionTwistImpulse + deltaLambda, frictionLimit));
                deltaLambda = contactManifold.frictionTwistImpulse - lambdaTemp;

                // Compute the impulse P=J^T * lambda
                linearImpulseBody1 = Vector3(0.0, 0.0, 0.0);
                angularImpulseBody1 = -contactManifold.normal * deltaLambda;
                linearImpulseBody2 = Vector3(0.0, 0.0, 0.0);;
                angularImpulseBody2 = contactManifold.normal * deltaLambda;
                const Impulse impulseTwistFriction(linearImpulseBody1, angularImpulseBody1,
                                                   linearImpulseBody2, angularImpulseBody2);

                // Apply the impulses to the bodies of the constraint
                applyImpulse(impulseTwistFriction, contactManifold);
            }
        }
    }
}

// Solve the constraints
void ContactSolver::solve(decimal timeStep) {

    mTimeStep = timeStep;

    // Initialize the solver
    initialize();

    initializeBodies();

    // Fill-in all the matrices needed to solve the LCP problem
    initializeContactConstraints();

    // Warm start the solver
    if (mIsWarmStartingActive) {
        warmStart();
    }

    // Solve the contact constraints
    solveContactConstraints();

    // Cache the lambda values in order to use them in the next step
    storeImpulses();
}

// Store the computed impulses to use them to
// warm start the solver at the next iteration
void ContactSolver::storeImpulses() {

    // For each constraint
    for (uint c=0; c<mNbContactConstraints; c++) {

        ContactManifoldSolver& contactManifold = mContactConstraints[c];

        for (uint i=0; i<contactManifold.nbContacts; i++) {

            ContactPointSolver& contactPoint = contactManifold.contacts[i];

            contactPoint.contact->setCachedLambda(0, contactPoint.penetrationImpulse);
            contactPoint.contact->setCachedLambda(1, contactPoint.friction1Impulse);
            contactPoint.contact->setCachedLambda(2, contactPoint.friction2Impulse);

            contactPoint.contact->setFrictionVector1(contactPoint.frictionVector1);
            contactPoint.contact->setFrictionVector2(contactPoint.frictionVector2);
        }

        contactManifold.contactManifold->friction1Impulse = contactManifold.friction1Impulse;
        contactManifold.contactManifold->friction2Impulse = contactManifold.friction2Impulse;
        contactManifold.contactManifold->frictionTwistImpulse = contactManifold.frictionTwistImpulse;
        contactManifold.contactManifold->frictionVector1 = contactManifold.frictionVector1;
        contactManifold.contactManifold->frictionVector2 = contactManifold.frictionVector2;
    }
}

// Apply an impulse to the two bodies of a constraint
void ContactSolver::applyImpulse(const Impulse& impulse, const ContactManifoldSolver& contactManifold) {

    // Update the velocities of the bodies by applying the impulse P
    if (contactManifold.isBody1Moving) {
        mLinearVelocities[contactManifold.indexBody1] += contactManifold.massInverseBody1 *
                                                    impulse.linearImpulseBody1;
        mAngularVelocities[contactManifold.indexBody1] += contactManifold.inverseInertiaTensorBody1 *
                                                     impulse.angularImpulseBody1;
    }
    if (contactManifold.isBody2Moving) {
        mLinearVelocities[contactManifold.indexBody2] += contactManifold.massInverseBody2 *
                                                    impulse.linearImpulseBody2;
        mAngularVelocities[contactManifold.indexBody2] += contactManifold.inverseInertiaTensorBody2 *
                                                     impulse.angularImpulseBody2;
    }
}

// Apply an impulse to the two bodies of a constraint
void ContactSolver::applySplitImpulse(const Impulse& impulse,
                                      const ContactManifoldSolver& contactManifold) {

    // Update the velocities of the bodies by applying the impulse P
    if (contactManifold.isBody1Moving) {
        mSplitLinearVelocities[contactManifold.indexBody1] += contactManifold.massInverseBody1 *
                                                    impulse.linearImpulseBody1;
        mSplitAngularVelocities[contactManifold.indexBody1] += contactManifold.inverseInertiaTensorBody1 *
                                                     impulse.angularImpulseBody1;
    }
    if (contactManifold.isBody2Moving) {
        mSplitLinearVelocities[contactManifold.indexBody2] += contactManifold.massInverseBody2 *
                                                    impulse.linearImpulseBody2;
        mSplitAngularVelocities[contactManifold.indexBody2] += contactManifold.inverseInertiaTensorBody2 *
                                                     impulse.angularImpulseBody2;
    }
}

// Compute the two unit orthogonal vectors "t1" and "t2" that span the tangential friction plane
// for a contact point constraint. The two vectors have to be such that : t1 x t2 = contactNormal.
void ContactSolver::computeFrictionVectors(const Vector3& deltaVelocity,
                                           ContactPointSolver& contactPoint) const {

    // Update the old friction vectors
    //contact.oldFrictionVector1 = contact.frictionVector1;
    //contact.oldFrictionVector2 = contact.frictionVector2;

    assert(contactPoint.normal.length() > 0.0);

    // Compute the velocity difference vector in the tangential plane
    Vector3 normalVelocity = deltaVelocity.dot(contactPoint.normal) * contactPoint.normal;
    Vector3 tangentVelocity = deltaVelocity - normalVelocity;

    // If the velocty difference in the tangential plane is not zero
    decimal lengthTangenVelocity = tangentVelocity.length();
    if (lengthTangenVelocity > 0.0) {

        // Compute the first friction vector in the direction of the tangent
        // velocity difference
        contactPoint.frictionVector1 = tangentVelocity / lengthTangenVelocity;
    }
    else {

        // Get any orthogonal vector to the normal as the first friction vector
        contactPoint.frictionVector1 = contactPoint.normal.getOneUnitOrthogonalVector();
    }

    // The second friction vector is computed by the cross product of the firs
    // friction vector and the contact normal
    contactPoint.frictionVector2 = contactPoint.normal.cross(contactPoint.frictionVector1).getUnit();
}

// Compute the two unit orthogonal vectors "t1" and "t2" that span the tangential friction plane
// for a contact constraint. The two vectors have to be such that : t1 x t2 = contactNormal.
void ContactSolver::computeFrictionVectors(const Vector3& deltaVelocity,
                                           ContactManifoldSolver& contact) const {

    assert(contact.normal.length() > 0.0);

    // Compute the velocity difference vector in the tangential plane
    Vector3 normalVelocity = deltaVelocity.dot(contact.normal) * contact.normal;
    Vector3 tangentVelocity = deltaVelocity - normalVelocity;

    // If the velocty difference in the tangential plane is not zero
    decimal lengthTangenVelocity = tangentVelocity.length();
    if (lengthTangenVelocity > 0.0) {

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

// Clean up the constraint solver
void ContactSolver::cleanup() {
    mMapBodyToIndex.clear();
    mConstraintBodies.clear();

    if (mContactConstraints != 0) {
        delete[] mContactConstraints;
        mContactConstraints = 0;
    }
    if (mLinearVelocities != 0) {
        delete[] mLinearVelocities;
        mLinearVelocities = 0;
    }
    if (mAngularVelocities != 0) {
        delete[] mAngularVelocities;
        mAngularVelocities = 0;
    }
}
