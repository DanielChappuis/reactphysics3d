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
#include "ConstraintSolver.h"
#include "DynamicsWorld.h"
#include "../body/RigidBody.h"

using namespace reactphysics3d;
using namespace std;


// Constructor
ConstraintSolver::ConstraintSolver(DynamicsWorld* world)
    :world(world), nbConstraints(0), mNbIterations(10), mContactConstraints(0), Vconstraint(0), Wconstraint(0), V1(0), W1(0) {

}

// Destructor
ConstraintSolver::~ConstraintSolver() {

}

 // Initialize the constraint solver before each solving
void ConstraintSolver::initialize() {
    
    nbConstraints = 0;

    // TOOD : Use better allocation here
    mContactConstraints = new ContactConstraint[world->getNbContactManifolds()];

    mNbContactConstraints = 0;

    // For each contact manifold of the world
    vector<ContactManifold>::iterator it;
    for (it = world->getContactManifoldsBeginIterator(); it != world->getContactManifoldsEndIterator(); ++it) {
        ContactManifold contactManifold = *it;

        ContactConstraint& constraint = mContactConstraints[mNbContactConstraints];

        assert(contactManifold.nbContacts > 0);

        RigidBody* body1 = contactManifold.contacts[0]->getBody1();
        RigidBody* body2 = contactManifold.contacts[0]->getBody2();

        // Fill in the body number maping
        mMapBodyToIndex.insert(make_pair(body1, mMapBodyToIndex.size()));
        mMapBodyToIndex.insert(make_pair(body2, mMapBodyToIndex.size()));

        // Add the two bodies of the constraint in the constraintBodies list
        mConstraintBodies.insert(body1);
        mConstraintBodies.insert(body2);

        Vector3 x1 = body1->getTransform().getPosition();
        Vector3 x2 = body2->getTransform().getPosition();

        constraint.indexBody1 = mMapBodyToIndex[body1];
        constraint.indexBody2 = mMapBodyToIndex[body2];
        constraint.inverseInertiaTensorBody1 = body1->getInertiaTensorInverseWorld();
        constraint.inverseInertiaTensorBody2 = body2->getInertiaTensorInverseWorld();
        constraint.isBody1Moving = body1->getIsMotionEnabled();
        constraint.isBody2Moving = body2->getIsMotionEnabled();
        constraint.massInverseBody1 = body1->getMassInverse();
        constraint.massInverseBody2 = body2->getMassInverse();
        constraint.nbContacts = contactManifold.nbContacts;

        // For each contact point of the contact manifold
        for (uint c=0; c<contactManifold.nbContacts; c++) {

            ContactPointConstraint& contactPointConstraint = constraint.contacts[c];

            // Get a contact point
            Contact* contact = contactManifold.contacts[c];

            Vector3 p1 = contact->getWorldPointOnBody1();
            Vector3 p2 = contact->getWorldPointOnBody2();

            contactPointConstraint.contact = contact;
            contactPointConstraint.normal = contact->getNormal();
            contactPointConstraint.r1 = p1 - x1;
            contactPointConstraint.r2 = p2 - x2;
            contactPointConstraint.frictionVector1 = contact->getFrictionVector1();
            contactPointConstraint.frictionVector2 = contact->getFrictionVector2();
        }

        mNbContactConstraints++;
    }

    // Compute the number of bodies that are part of some active constraint
    nbBodies = mConstraintBodies.size();

    Vconstraint = new Vector3[nbBodies];
    Wconstraint = new Vector3[nbBodies];
    V1 = new Vector3[nbBodies];
    W1 = new Vector3[nbBodies];

    assert(mMapBodyToIndex.size() == nbBodies);
}

// Initialize bodies velocities
void ConstraintSolver::initializeBodies() {

    // For each current body that is implied in some constraint
    RigidBody* rigidBody;
    RigidBody* body;
    uint b=0;
    for (set<RigidBody*>::iterator it = mConstraintBodies.begin(); it != mConstraintBodies.end(); ++it, b++) {
        rigidBody = *it;
        uint bodyNumber = mMapBodyToIndex[rigidBody];

        // TODO : Use polymorphism and remove this downcasting
        assert(rigidBody);

        // Compute the vector V1 with initial velocities values
        int bodyIndexArray = 6 * bodyNumber;
        V1[bodyNumber] = rigidBody->getLinearVelocity();
        W1[bodyNumber] = rigidBody->getAngularVelocity();

        // Compute the vector Vconstraint with final constraint velocities
        Vconstraint[bodyNumber] = Vector3(0, 0, 0);
        Wconstraint[bodyNumber] = Vector3(0, 0, 0);

        // Compute the vector with forces and torques values
        Vector3 externalForce = rigidBody->getExternalForce();
        Vector3 externalTorque = rigidBody->getExternalTorque();
        Fext[bodyIndexArray] = externalForce[0];
        Fext[bodyIndexArray + 1] = externalForce[1];
        Fext[bodyIndexArray + 2] = externalForce[2];
        Fext[bodyIndexArray + 3] = externalTorque[0];
        Fext[bodyIndexArray + 4] = externalTorque[1];
        Fext[bodyIndexArray + 5] = externalTorque[2];

        // Initialize the mass and inertia tensor matrices
        Minv_sp_inertia[bodyNumber].setAllValues(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        Minv_sp_mass_diag[bodyNumber] = 0.0;

        // If the motion of the rigid body is enabled
        if (rigidBody->getIsMotionEnabled()) {
            Minv_sp_inertia[bodyNumber] = rigidBody->getInertiaTensorInverseWorld();
            Minv_sp_mass_diag[bodyNumber] = rigidBody->getMassInverse();
        }
    }
}

// Fill in all the matrices needed to solve the LCP problem
// Notice that all the active constraints should have been evaluated first
void ConstraintSolver::initializeContactConstraints(decimal dt) {
    decimal oneOverDT = 1.0 / dt;
    
    // For each contact constraint
    for (uint c=0; c<mNbContactConstraints; c++) {

        ContactConstraint& constraint = mContactConstraints[c];

        uint indexBody1 = constraint.indexBody1;
        uint indexBody2 = constraint.indexBody2;

        Matrix3x3& I1 = constraint.inverseInertiaTensorBody1;
        Matrix3x3& I2 = constraint.inverseInertiaTensorBody2;

        // For each contact point constraint
        for (uint i=0; i<constraint.nbContacts; i++) {

            ContactPointConstraint& contact = constraint.contacts[i];
            Contact* realContact = contact.contact;

            contact.r1CrossN = contact.r1.cross(contact.normal);
            contact.r1CrossT1 = contact.r1.cross(contact.frictionVector1);
            contact.r1CrossT2 = contact.r1.cross(contact.frictionVector2);
            contact.r2CrossT1 = contact.r2.cross(contact.frictionVector1);
            contact.r2CrossT2 = contact.r2.cross(contact.frictionVector2);

            contact.inversePenetrationMass = 0.0;
            if (constraint.isBody1Moving) {
                contact.inversePenetrationMass += constraint.massInverseBody1 +
                        ((I1 * contact.r1CrossN).cross(contact.r1)).dot(contact.normal);
            }
            if (constraint.isBody2Moving) {
                contact.inversePenetrationMass += constraint.massInverseBody2 +
                        ((I2 * contact.r2CrossN).cross(contact.r2)).dot(contact.normal);
            }

            // Compute the inverse mass matrix K for the friction constraints
            contact.inverseFriction1Mass = 0.0;
            contact.inverseFriction2Mass = 0.0;
            if (constraint.isBody1Moving) {
                contact.inverseFriction1Mass += constraint.massInverseBody1 + ((I1 * contact.r1CrossT1).cross(contact.r1)).dot(contact.frictionVector1);
                contact.inverseFriction2Mass += constraint.massInverseBody1 + ((I1 * contact.r1CrossT2).cross(contact.r1)).dot(contact.frictionVector2);
            }
            if (constraint.isBody2Moving) {
                contact.inverseFriction1Mass += constraint.massInverseBody2 + ((I2 * contact.r2CrossT1).cross(contact.r2)).dot(contact.frictionVector1);
                contact.inverseFriction2Mass += constraint.massInverseBody2 + ((I2 * contact.r2CrossT2).cross(contact.r2)).dot(contact.frictionVector2);
            }

            // Fill in the J_sp matrix
            realContact->computeJacobianPenetration(contact.J_spBody1Penetration, contact.J_spBody2Penetration);
            realContact->computeJacobianFriction1(contact.J_spBody1Friction1, contact.J_spBody2Friction1);
            realContact->computeJacobianFriction2(contact.J_spBody1Friction2, contact.J_spBody2Friction2);

            // Fill in the body mapping matrix
            //for(int i=0; i<realContact->getNbConstraints(); i++) {
            //    bodyMapping[noConstraint+i][0] = constraint->getBody1();
            //    bodyMapping[noConstraint+i][1] = constraint->getBody2();
            //}

            // Fill in the limit vectors for the constraint
            realContact->computeLowerBoundPenetration(contact.lowerBoundPenetration);
            realContact->computeLowerBoundFriction1(contact.lowerBoundFriction1);
            realContact->computeLowerBoundFriction2(contact.lowerBoundFriction2);
            realContact->computeUpperBoundPenetration(contact.upperBoundPenetration);
            realContact->computeUpperBoundFriction1(contact.upperBoundFriction1);
            realContact->computeUpperBoundFriction2(contact.upperBoundFriction2);

            // Fill in the error vector
            realContact->computeErrorPenetration(contact.errorPenetration);
            contact.errorFriction1 = 0.0;
            contact.errorFriction2 = 0.0;

            // Get the cached lambda values of the constraint
            contact.penetrationImpulse = realContact->getCachedLambda(0);
            contact.friction1Impulse = realContact->getCachedLambda(1);
            contact.friction2Impulse = realContact->getCachedLambda(2);
            //for (int i=0; i<constraint->getNbConstraints(); i++) {
            //    lambdaInit[noConstraint + i] = constraint->getCachedLambda(i);
           // }

            contact.errorPenetration = 0.0;
            decimal slop = 0.005;
            if (realContact->getPenetrationDepth() > slop) {
                 contact.errorPenetration += 0.2 * oneOverDT * std::max(double(realContact->getPenetrationDepth() - slop), 0.0);
            }
            contact.errorFriction1 = 0.0;
            contact.errorFriction2 = 0.0;

            // ---------- Penetration ---------- //

            // b = errorValues * oneOverDT;
            contact.b_Penetration = contact.errorPenetration * oneOverDT;

            // Substract 1.0/dt*J*V to the vector b
            indexBody1 = constraint.indexBody1;
            indexBody2 = constraint.indexBody2;
            decimal multiplication = 0.0;
            int body1ArrayIndex = 6 * indexBody1;
            int body2ArrayIndex = 6 * indexBody2;
            for (uint i=0; i<3; i++) {
                multiplication += contact.J_spBody1Penetration[i] * V1[indexBody1][i];
                multiplication += contact.J_spBody1Penetration[i + 3] * W1[indexBody1][i];

                multiplication += contact.J_spBody2Penetration[i] * V1[indexBody2][i];
                multiplication += contact.J_spBody2Penetration[i + 3] * W1[indexBody2][i];
            }
            contact.b_Penetration -= multiplication * oneOverDT ;

            // Substract J*M^-1*F_ext to the vector b
            decimal value1 = 0.0;
            decimal value2 = 0.0;
            decimal sum1, sum2;
            value1 += contact.J_spBody1Penetration[0] * Minv_sp_mass_diag[indexBody1] * Fext[body1ArrayIndex] +
                    contact.J_spBody1Penetration[1] * Minv_sp_mass_diag[indexBody1] * Fext[body1ArrayIndex + 1] +
                    contact.J_spBody1Penetration[2] * Minv_sp_mass_diag[indexBody1] * Fext[body1ArrayIndex + 2];
            value2 += contact.J_spBody2Penetration[0] * Minv_sp_mass_diag[indexBody2] * Fext[body2ArrayIndex] +
                      contact.J_spBody2Penetration[1] * Minv_sp_mass_diag[indexBody2] * Fext[body2ArrayIndex + 1] +
                      contact.J_spBody2Penetration[2] * Minv_sp_mass_diag[indexBody2] * Fext[body2ArrayIndex + 2];
            for (uint i=0; i<3; i++) {
                sum1 = 0.0;
                sum2 = 0.0;
                for (uint j=0; j<3; j++) {
                    sum1 += contact.J_spBody1Penetration[3 + j] * Minv_sp_inertia[indexBody1].getValue(j, i);
                    sum2 += contact.J_spBody2Penetration[3 + j] * Minv_sp_inertia[indexBody2].getValue(j, i);
                }
                value1 += sum1 * Fext[body1ArrayIndex + 3 + i];
                value2 += sum2 * Fext[body2ArrayIndex + 3 + i];
            }

            contact.b_Penetration -= value1 + value2;

             // ---------- Friction 1 ---------- //

            // b = errorValues * oneOverDT;
            contact.b_Friction1 = contact.errorFriction1 * oneOverDT;

            // Substract 1.0/dt*J*V to the vector b
            multiplication = 0.0;
            for (uint i=0; i<3; i++) {
                multiplication += contact.J_spBody1Friction1[i] * V1[indexBody1][i];
                multiplication += contact.J_spBody1Friction1[i + 3] * W1[indexBody1][i];

                multiplication += contact.J_spBody2Friction1[i] * V1[indexBody2][i];
                multiplication += contact.J_spBody2Friction1[i + 3] * W1[indexBody2][i];
            }
            contact.b_Friction1 -= multiplication * oneOverDT ;

            // Substract J*M^-1*F_ext to the vector b
            value1 = 0.0;
            value2 = 0.0;
            value1 += contact.J_spBody1Friction1[0] * Minv_sp_mass_diag[indexBody1] * Fext[body1ArrayIndex] +
                    contact.J_spBody1Friction1[1] * Minv_sp_mass_diag[indexBody1] * Fext[body1ArrayIndex + 1] +
                    contact.J_spBody1Friction1[2] * Minv_sp_mass_diag[indexBody1] * Fext[body1ArrayIndex + 2];
            value2 += contact.J_spBody2Friction1[0] * Minv_sp_mass_diag[indexBody2] * Fext[body2ArrayIndex] +
                      contact.J_spBody2Friction1[1] * Minv_sp_mass_diag[indexBody2] * Fext[body2ArrayIndex + 1] +
                      contact.J_spBody2Friction1[2] * Minv_sp_mass_diag[indexBody2] * Fext[body2ArrayIndex + 2];
            for (uint i=0; i<3; i++) {
                sum1 = 0.0;
                sum2 = 0.0;
                for (uint j=0; j<3; j++) {
                    sum1 += contact.J_spBody1Friction1[3 + j] * Minv_sp_inertia[indexBody1].getValue(j, i);
                    sum2 += contact.J_spBody2Friction1[3 + j] * Minv_sp_inertia[indexBody2].getValue(j, i);
                }
                value1 += sum1 * Fext[body1ArrayIndex + 3 + i];
                value2 += sum2 * Fext[body2ArrayIndex + 3 + i];
            }

            contact.b_Friction1 -= value1 + value2;


             // ---------- Friction 2 ---------- //

            // b = errorValues * oneOverDT;
            contact.b_Friction2 = contact.errorFriction2 * oneOverDT;

            // Substract 1.0/dt*J*V to the vector b
            multiplication = 0.0;
            for (uint i=0; i<3; i++) {
                multiplication += contact.J_spBody1Friction2[i] * V1[indexBody1][i];
                multiplication += contact.J_spBody1Friction2[i + 3] * W1[indexBody1][i];

                multiplication += contact.J_spBody2Friction2[i] * V1[indexBody2][i];
                multiplication += contact.J_spBody2Friction2[i + 3] * W1[indexBody2][i];
            }
            contact.b_Friction2 -= multiplication * oneOverDT ;

            // Substract J*M^-1*F_ext to the vector b
            value1 = 0.0;
            value2 = 0.0;
            value1 += contact.J_spBody1Friction2[0] * Minv_sp_mass_diag[indexBody1] * Fext[body1ArrayIndex] +
                    contact.J_spBody1Friction2[1] * Minv_sp_mass_diag[indexBody1] * Fext[body1ArrayIndex + 1] +
                    contact.J_spBody1Friction2[2] * Minv_sp_mass_diag[indexBody1] * Fext[body1ArrayIndex + 2];
            value2 += contact.J_spBody2Friction2[0] * Minv_sp_mass_diag[indexBody2] * Fext[body2ArrayIndex] +
                      contact.J_spBody2Friction2[1] * Minv_sp_mass_diag[indexBody2] * Fext[body2ArrayIndex + 1] +
                      contact.J_spBody2Friction2[2] * Minv_sp_mass_diag[indexBody2] * Fext[body2ArrayIndex + 2];
            for (uint i=0; i<3; i++) {
                sum1 = 0.0;
                sum2 = 0.0;
                for (uint j=0; j<3; j++) {
                    sum1 += contact.J_spBody1Friction2[3 + j] * Minv_sp_inertia[indexBody1].getValue(j, i);
                    sum2 += contact.J_spBody2Friction2[3 + j] * Minv_sp_inertia[indexBody2].getValue(j, i);
                }
                value1 += sum1 * Fext[body1ArrayIndex + 3 + i];
                value2 += sum2 * Fext[body2ArrayIndex + 3 + i];
            }

            contact.b_Friction2 -= value1 + value2;
        }
    }


}

// Compute the matrix B_sp
void ConstraintSolver::computeMatrixB_sp() {
	uint indexConstraintArray, indexBody1, indexBody2;
	
    // For each constraint
    for (uint m = 0; m<mNbContactConstraints; m++) {

        ContactConstraint& constraint = mContactConstraints[m];

        for (uint c=0; c<constraint.nbContacts; c++) {

            ContactPointConstraint& contact = constraint.contacts[c];

            // ---------- Penetration ---------- //

            indexBody1 = constraint.indexBody1;
            indexBody2 = constraint.indexBody2;
            contact.B_spBody1Penetration[0] = Minv_sp_mass_diag[indexBody1] * contact.J_spBody1Penetration[0];
            contact.B_spBody1Penetration[1] = Minv_sp_mass_diag[indexBody1] * contact.J_spBody1Penetration[1];
            contact.B_spBody1Penetration[2] = Minv_sp_mass_diag[indexBody1] * contact.J_spBody1Penetration[2];
            contact.B_spBody2Penetration[0] = Minv_sp_mass_diag[indexBody2] * contact.J_spBody2Penetration[0];
            contact.B_spBody2Penetration[1] = Minv_sp_mass_diag[indexBody2] * contact.J_spBody2Penetration[1];
            contact.B_spBody2Penetration[2] = Minv_sp_mass_diag[indexBody2] * contact.J_spBody2Penetration[2];

            for (uint i=0; i<3; i++) {
                contact.B_spBody1Penetration[3 + i] = 0.0;
                contact.B_spBody2Penetration[3 + i] = 0.0;
                for (uint j=0; j<3; j++) {
                    contact.B_spBody1Penetration[3 + i] += Minv_sp_inertia[indexBody1].getValue(i, j) * contact.J_spBody1Penetration[3 + j];
                    contact.B_spBody2Penetration[3 + i] += Minv_sp_inertia[indexBody2].getValue(i, j) * contact.J_spBody2Penetration[3 + j];
                }
            }

            // ---------- Friction 1 ---------- //

            Matrix3x3& I1 = constraint.inverseInertiaTensorBody1;
            Matrix3x3& I2 = constraint.inverseInertiaTensorBody2;

            Vector3 JspLinBody1 = Vector3(0, 0, 0);
            Vector3 JspAngBody1 = Vector3(0, 0, 0);
            Vector3 JspLinBody2 = Vector3(0, 0, 0);
            Vector3 JspAngBody2 = Vector3(0, 0, 0);

            if (constraint.isBody1Moving) {
                JspLinBody1 += -constraint.massInverseBody1 * contact.frictionVector1;
                JspAngBody1 += I1 * (-contact.r1CrossT1);
            }
            if (constraint.isBody2Moving) {
                JspLinBody2 += constraint.massInverseBody2 * contact.frictionVector1;
                JspAngBody2 += I1 * (contact.r2CrossT1);
            }

            /*
            std::cout << "------ New Version ----" << std::endl;
            std::cout << "JspLinBody1 : " << JspLinBody1.getX() << ", " << JspLinBody1.getY() << ", " << JspLinBody1.getZ() << std::endl;
            std::cout << "JspAngBody1 : " << JspAngBody1.getX() << ", " << JspAngBody1.getY() << ", " << JspAngBody1.getZ() << std::endl;
            std::cout << "JspLinBody2 : " << JspLinBody2.getX() << ", " << JspLinBody2.getY() << ", " << JspLinBody2.getZ() << std::endl;
            std::cout << "JspAngBody2 : " << JspAngBody2.getX() << ", " << JspAngBody2.getY() << ", " << JspAngBody2.getZ() << std::endl;

            std::cout << ": friction vector 1 : " << contact.frictionVector1.getX() << ", " << contact.frictionVector1.getY() << ", " << contact.frictionVector1.getZ() << std::endl;
            std::cout << ": r1 x t1 : " << contact.r1CrossT1.getX() << ", " << contact.r1CrossT1.getY() << ", " << contact.r1CrossT1.getZ() << std::endl;
            std::cout << ": r2 x t1 : " << contact.r2CrossT1.getX() << ", " << contact.r2CrossT1.getY() << ", " << contact.r2CrossT1.getZ() << std::endl;
            std::cout << ": inverse mass body 1 = " << constraint.massInverseBody1 << std::endl;
            std::cout << ": inverse mass body 2 = " << constraint.massInverseBody2 << std::endl;
            */

            contact.B_spBody1Friction1[0] = Minv_sp_mass_diag[indexBody1] * contact.J_spBody1Friction1[0];
            contact.B_spBody1Friction1[1] = Minv_sp_mass_diag[indexBody1] * contact.J_spBody1Friction1[1];
            contact.B_spBody1Friction1[2] = Minv_sp_mass_diag[indexBody1] * contact.J_spBody1Friction1[2];
            contact.B_spBody2Friction1[0] = Minv_sp_mass_diag[indexBody2] * contact.J_spBody2Friction1[0];
            contact.B_spBody2Friction1[1] = Minv_sp_mass_diag[indexBody2] * contact.J_spBody2Friction1[1];
            contact.B_spBody2Friction1[2] = Minv_sp_mass_diag[indexBody2] * contact.J_spBody2Friction1[2];

            for (uint i=0; i<3; i++) {
                contact.B_spBody1Friction1[3 + i] = 0.0;
                contact.B_spBody2Friction1[3 + i] = 0.0;
                for (uint j=0; j<3; j++) {
                    contact.B_spBody1Friction1[3 + i] += Minv_sp_inertia[indexBody1].getValue(i, j) * contact.J_spBody1Friction1[3 + j];
                    contact.B_spBody2Friction1[3 + i] += Minv_sp_inertia[indexBody2].getValue(i, j) * contact.J_spBody2Friction1[3 + j];
                }
            }

            /*
            std::cout << "------ Old Version ----" << std::endl;
            std::cout << "JspLinBody1 : " << contact.B_spBody1Friction1[0] << ", " << contact.B_spBody1Friction1[1] << ", " << contact.B_spBody1Friction1[2] << std::endl;
            std::cout << "JspAngBody1 : " << contact.B_spBody1Friction1[3] << ", " << contact.B_spBody1Friction1[4] << ", " << contact.B_spBody1Friction1[5] << std::endl;
            std::cout << "JspLinBody2 : " << contact.B_spBody2Friction1[0] << ", " << contact.B_spBody2Friction1[1] << ", " << contact.B_spBody2Friction1[2] << std::endl;
            std::cout << "JspAngBody2 : " << contact.B_spBody2Friction1[3] << ", " << contact.B_spBody2Friction1[4] << ", " << contact.B_spBody2Friction1[5] << std::endl;
            */

            // ---------- Friction 2 ---------- //

            contact.B_spBody1Friction2[0] = Minv_sp_mass_diag[indexBody1] * contact.J_spBody1Friction2[0];
            contact.B_spBody1Friction2[1] = Minv_sp_mass_diag[indexBody1] * contact.J_spBody1Friction2[1];
            contact.B_spBody1Friction2[2] = Minv_sp_mass_diag[indexBody1] * contact.J_spBody1Friction2[2];
            contact.B_spBody2Friction2[0] = Minv_sp_mass_diag[indexBody2] * contact.J_spBody2Friction2[0];
            contact.B_spBody2Friction2[1] = Minv_sp_mass_diag[indexBody2] * contact.J_spBody2Friction2[1];
            contact.B_spBody2Friction2[2] = Minv_sp_mass_diag[indexBody2] * contact.J_spBody2Friction2[2];

            for (uint i=0; i<3; i++) {
                contact.B_spBody1Friction2[3 + i] = 0.0;
                contact.B_spBody2Friction2[3 + i] = 0.0;
                for (uint j=0; j<3; j++) {
                    contact.B_spBody1Friction2[3 + i] += Minv_sp_inertia[indexBody1].getValue(i, j) * contact.J_spBody1Friction2[3 + j];
                    contact.B_spBody2Friction2[3 + i] += Minv_sp_inertia[indexBody2].getValue(i, j) * contact.J_spBody2Friction2[3 + j];
                }
            }
        }
    }
}

// Compute the vector V_constraint (which corresponds to the constraint part of
// the final V2 vector) according to the formula
// V_constraint = dt * (M^-1 * J^T * lambda)
// Note that we use the vector V to store both V1 and V_constraint.
// Note that M^-1 * J^T = B.
// This method is called after that the LCP solver has computed lambda
void ConstraintSolver::computeVectorVconstraint(decimal dt) {
    uint indexBody1Array, indexBody2Array;
	uint j;
	
    // Compute dt * (M^-1 * J^T * lambda
    for (uint c=0; c<mNbContactConstraints; c++) {

        ContactConstraint& constraint = mContactConstraints[c];

        for (uint i=0; i<constraint.nbContacts; i++) {

            ContactPointConstraint& contact = constraint.contacts[i];

            // ---------- Penetration ---------- //

            indexBody1Array = constraint.indexBody1;
            indexBody2Array = constraint.indexBody2;
            for (j=0; j<3; j++) {
                Vconstraint[indexBody1Array][j] += contact.B_spBody1Penetration[j] * contact.penetrationImpulse * dt;
                Wconstraint[indexBody1Array][j] += contact.B_spBody1Penetration[j + 3] * contact.penetrationImpulse * dt;

                Vconstraint[indexBody2Array][j] += contact.B_spBody2Penetration[j] * contact.penetrationImpulse * dt;
                Wconstraint[indexBody2Array][j] += contact.B_spBody2Penetration[j + 3] * contact.penetrationImpulse * dt;
            }

            // ---------- Friction 1 ---------- //

            for (j=0; j<3; j++) {
                Vconstraint[indexBody1Array][j] += contact.B_spBody1Friction1[j] * contact.friction1Impulse * dt;
                Wconstraint[indexBody1Array][j] += contact.B_spBody1Friction1[j + 3] * contact.friction1Impulse * dt;

                Vconstraint[indexBody2Array][j] += contact.B_spBody2Friction1[j] * contact.friction1Impulse * dt;
                Wconstraint[indexBody2Array][j] += contact.B_spBody2Friction1[j + 3] * contact.friction1Impulse * dt;
            }

            // ---------- Friction 2 ---------- //

            for (j=0; j<3; j++) {
                Vconstraint[indexBody1Array][j] += contact.B_spBody1Friction2[j] * contact.friction2Impulse * dt;
                Wconstraint[indexBody1Array][j] += contact.B_spBody1Friction2[j + 3] * contact.friction2Impulse * dt;

                Vconstraint[indexBody2Array][j] += contact.B_spBody2Friction2[j] * contact.friction2Impulse * dt;
                Wconstraint[indexBody2Array][j] += contact.B_spBody2Friction2[j + 3] * contact.friction2Impulse * dt;
            }
        }
    }
}

// Solve a LCP problem using the Projected-Gauss-Seidel algorithm
// This method outputs the result in the lambda vector
void ConstraintSolver::solveLCP() {

   // for (uint i=0; i<nbConstraints; i++) {
   //         lambda[i] = lambdaInit[i];
   // }

    uint indexBody1Array, indexBody2Array;
    decimal deltaLambda;
    decimal lambdaTemp;
    uint iter;

    // Compute the vector a
    computeVectorA();

    // For each iteration
    for(iter=0; iter<mNbIterations; iter++) {

        // For each constraint
        for (uint c=0; c<mNbContactConstraints; c++) {

            ContactConstraint& constraint = mContactConstraints[c];

            for (uint i=0; i<constraint.nbContacts; i++) {

                ContactPointConstraint& contact = constraint.contacts[i];

                indexBody1Array = constraint.indexBody1;
                indexBody2Array = constraint.indexBody2;

                // --------- Penetration --------- //

                deltaLambda = contact.b_Penetration;
                for (uint j=0; j<3; j++) {
                    deltaLambda -= (contact.J_spBody1Penetration[j] * aLinear[indexBody1Array][j] + contact.J_spBody2Penetration[j] * aLinear[indexBody2Array][j]);
                    deltaLambda -= (contact.J_spBody1Penetration[j + 3] * aAngular[indexBody1Array][j] + contact.J_spBody2Penetration[j + 3] * aAngular[indexBody2Array][j]);
                }
                deltaLambda /= contact.inversePenetrationMass;
                lambdaTemp = contact.penetrationImpulse;
                contact.penetrationImpulse = std::max(contact.lowerBoundPenetration, std::min(contact.penetrationImpulse + deltaLambda, contact.upperBoundPenetration));
                deltaLambda = contact.penetrationImpulse - lambdaTemp;
                for (uint j=0; j<3; j++) {
                    aLinear[indexBody1Array][j] += contact.B_spBody1Penetration[j] * deltaLambda;
                    aAngular[indexBody1Array][j] += contact.B_spBody1Penetration[j + 3] * deltaLambda;

                    aLinear[indexBody2Array][j] += contact.B_spBody2Penetration[j] * deltaLambda;
                    aAngular[indexBody2Array][j] += contact.B_spBody2Penetration[j + 3] * deltaLambda;
                }

                // --------- Friction 1 --------- //

                deltaLambda = contact.b_Friction1;
                for (uint j=0; j<3; j++) {
                    deltaLambda -= (contact.J_spBody1Friction1[j] * aLinear[indexBody1Array][j] + contact.J_spBody2Friction1[j] * aLinear[indexBody2Array][j]);
                    deltaLambda -= (contact.J_spBody1Friction1[j + 3] * aAngular[indexBody1Array][j] + contact.J_spBody2Friction1[j + 3] * aAngular[indexBody2Array][j]);
                }
                deltaLambda /= contact.inverseFriction1Mass;
                lambdaTemp = contact.friction1Impulse;
                contact.friction1Impulse = std::max(contact.lowerBoundFriction1, std::min(contact.friction1Impulse + deltaLambda, contact.upperBoundFriction1));
                deltaLambda = contact.friction1Impulse - lambdaTemp;
                for (uint j=0; j<3; j++) {
                    aLinear[indexBody1Array][j] += contact.B_spBody1Friction1[j] * deltaLambda;
                    aAngular[indexBody1Array][j] += contact.B_spBody1Friction1[j + 3] * deltaLambda;

                    aLinear[indexBody2Array][j] += contact.B_spBody2Friction1[j] * deltaLambda;
                    aAngular[indexBody2Array][j] += contact.B_spBody2Friction1[j + 3] * deltaLambda;
                }

                // --------- Friction 2 --------- //

                deltaLambda = contact.b_Friction2;
                for (uint j=0; j<3; j++) {
                    deltaLambda -= (contact.J_spBody1Friction2[j] * aLinear[indexBody1Array][j] + contact.J_spBody2Friction2[j] * aLinear[indexBody2Array][j]);
                    deltaLambda -= (contact.J_spBody1Friction2[j + 3] * aAngular[indexBody1Array][j] + contact.J_spBody2Friction2[j + 3] * aAngular[indexBody2Array][j]);
                }
                deltaLambda /= contact.inverseFriction2Mass;
                lambdaTemp = contact.friction2Impulse;
                contact.friction2Impulse = std::max(contact.lowerBoundFriction2, std::min(contact.friction2Impulse + deltaLambda, contact.upperBoundFriction2));
                deltaLambda = contact.friction2Impulse - lambdaTemp;
                for (uint j=0; j<3; j++) {
                    aLinear[indexBody1Array][j] += contact.B_spBody1Friction2[j] * deltaLambda;
                    aAngular[indexBody1Array][j] += contact.B_spBody1Friction2[j + 3] * deltaLambda;

                    aLinear[indexBody2Array][j] += contact.B_spBody2Friction2[j] * deltaLambda;
                    aAngular[indexBody2Array][j] += contact.B_spBody2Friction2[j + 3] * deltaLambda;
                }
            }
        }
    }
}

// Compute the vector a used in the solve() method
// Note that a = B * lambda
void ConstraintSolver::computeVectorA() {
    uint i;
    uint indexBody1Array, indexBody2Array;
    
    // Init the vector a with zero values
    for (i=0; i<nbBodies; i++) {
        aLinear[i] = Vector3(0, 0, 0);
        aAngular[i] = Vector3(0, 0, 0);
    }

    // For each constraint
    for (uint c=0; c<mNbContactConstraints; c++) {

        ContactConstraint& constraint = mContactConstraints[c];

        for (uint i=0; i<constraint.nbContacts; i++) {

            ContactPointConstraint& contact = constraint.contacts[i];

            indexBody1Array = constraint.indexBody1;
            indexBody2Array = constraint.indexBody2;

            // --------- Penetration --------- //

            for (uint j=0; j<3; j++) {
                aLinear[indexBody1Array][j] += contact.B_spBody1Penetration[j] * contact.penetrationImpulse;
                aAngular[indexBody1Array][j] += contact.B_spBody1Penetration[j + 3] * contact.penetrationImpulse;

                aLinear[indexBody2Array][j] += contact.B_spBody2Penetration[j] * contact.penetrationImpulse;
                aAngular[indexBody2Array][j] += contact.B_spBody2Penetration[j + 3] * contact.penetrationImpulse;
            }

            // --------- Friction 1 --------- //

            for (uint j=0; j<3; j++) {
                aLinear[indexBody1Array][j] += contact.B_spBody1Friction1[j] * contact.friction1Impulse;
                aAngular[indexBody1Array][j] += contact.B_spBody1Friction1[j + 3] * contact.friction1Impulse;

                aLinear[indexBody2Array][j] += contact.B_spBody2Friction1[j] * contact.friction1Impulse;
                aAngular[indexBody2Array][j] += contact.B_spBody2Friction1[j + 3] * contact.friction1Impulse;
            }

            // --------- Friction 2 --------- //

            for (uint j=0; j<3; j++) {
                aLinear[indexBody1Array][j] += contact.B_spBody1Friction2[j] * contact.friction2Impulse;
                aAngular[indexBody1Array][j] += contact.B_spBody1Friction2[j + 3] * contact.friction2Impulse;

                aLinear[indexBody2Array][j] += contact.B_spBody2Friction2[j] * contact.friction2Impulse;
                aAngular[indexBody2Array][j] += contact.B_spBody2Friction2[j + 3] * contact.friction2Impulse;
            }
        }
    }
}

// Cache the lambda values in order to reuse them in the next step
// to initialize the lambda vector
void ConstraintSolver::cacheLambda() {
    
    // For each constraint
    for (uint c=0; c<mNbContactConstraints; c++) {

        ContactConstraint& constraint = mContactConstraints[c];

        for (uint i=0; i<constraint.nbContacts; i++) {

            ContactPointConstraint& contact = constraint.contacts[i];

            contact.contact->setCachedLambda(0, contact.penetrationImpulse);
            contact.contact->setCachedLambda(1, contact.friction1Impulse);
            contact.contact->setCachedLambda(2, contact.friction2Impulse);
        }
    }
}
