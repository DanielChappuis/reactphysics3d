/***************************************************************************
* Copyright (C) 2009      Daniel Chappuis                                  *
****************************************************************************
* This file is part of ReactPhysics3D.                                     *
*                                                                          *
* ReactPhysics3D is free software: you can redistribute it and/or modify   *
* it under the terms of the GNU Lesser General Public License as published *
* by the Free Software Foundation, either version 3 of the License, or     *
* (at your option) any later version.                                      *
*                                                                          *
* ReactPhysics3D is distributed in the hope that it will be useful,        *
* but WITHOUT ANY WARRANTY; without even the implied warranty of           *
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the             *
* GNU Lesser General Public License for more details.                      *
*                                                                          *
* You should have received a copy of the GNU Lesser General Public License *
* along with ReactPhysics3D. If not, see <http://www.gnu.org/licenses/>.   *
***************************************************************************/

// Libraries
#include "ConstraintSolver.h"
#include "../mathematics/lcp/LCPProjectedGaussSeidel.h"
#include "../body/RigidBody.h"

using namespace reactphysics3d;

// Constructor
ConstraintSolver::ConstraintSolver(PhysicsWorld* world)
                 :physicsWorld(world), bodyMapping(0), nbConstraints(0), lcpSolver(new LCPProjectedGaussSeidel(MAX_LCP_ITERATIONS)) {

}

// Destructor
ConstraintSolver::~ConstraintSolver() {

}

// Allocate all the matrices needed to solve the LCP problem
void ConstraintSolver::allocate() {
    nbConstraints = 0;
    Constraint* constraint;

    // For each constraint
    std::vector<Constraint*>::iterator it;
    for (it = physicsWorld->getConstraintsBeginIterator(); it != physicsWorld->getConstraintsEndIterator(); it++) {
        constraint = *it;

        // Evaluate the constraint
        constraint->evaluate();

        // If the constraint is active
        if (constraint->isActive()) {
            activeConstraints.push_back(constraint);

            // Add the two bodies of the constraint in the constraintBodies list
            constraintBodies.insert(constraint->getBody1());
            constraintBodies.insert(constraint->getBody2());

            // Fill in the body number maping
            bodyNumberMapping.insert(std::pair<Body*, unsigned int>(constraint->getBody1(), bodyNumberMapping.size()));
            bodyNumberMapping.insert(std::pair<Body*, unsigned int>(constraint->getBody2(), bodyNumberMapping.size()));

            // Update the size of the jacobian matrix
            nbConstraints += (1 + constraint->getNbAuxConstraints());
        }
    }

    assert(nbConstraints > 0);

    // Compute the number of bodies that are part of some active constraint
    nbBodies = bodyNumberMapping.size();

    bodyMapping = new Body**[nbConstraints];
    J_sp = new Matrix*[nbConstraints];
    B_sp = new Matrix*[2];
    B_sp[0] = new Matrix[nbConstraints];
    B_sp[1] = new Matrix[nbConstraints];
    for (uint i=0; i<nbConstraints; i++) {
        bodyMapping[i] = new Body*[2];
        J_sp[i] = new Matrix[2];
    }

    errorValues.changeSize(nbConstraints);
    b.changeSize(nbConstraints);
    lambda.changeSize(nbConstraints);
    lambdaInit.changeSize(nbConstraints);
    lowerBounds.changeSize(nbConstraints);
    upperBounds.changeSize(nbConstraints);
    Minv_sp = new Matrix[nbBodies];
    V1 = new Vector[nbBodies];
    Vconstraint = new Vector[nbBodies];
    Fext = new Vector[nbBodies];
}

// Fill in all the matrices needed to solve the LCP problem
// Notice that all the active constraints should have been evaluated first
void ConstraintSolver::fillInMatrices() {

    // For each active constraint
    uint noConstraint = 0;
    uint nbAuxConstraints = 0;
    for (uint c=0; c<activeConstraints.size(); c++) {
        
        Constraint* constraint = activeConstraints.at(c);

        // Fill in the J_sp matrix
        J_sp[noConstraint][0].changeSize(1, 6);
        J_sp[noConstraint][1].changeSize(1, 6);
        J_sp[noConstraint][0] = constraint->getBody1Jacobian();
        J_sp[noConstraint][1] = constraint->getBody2Jacobian();

        // Fill in the body mapping matrix
        bodyMapping[noConstraint][0] = constraint->getBody1();
        bodyMapping[noConstraint][1] = constraint->getBody2();

        // Fill in the limit vectors for the constraint
        lowerBounds.setValue(noConstraint, constraint->getLowerBound());
        upperBounds.setValue(noConstraint, constraint->getUpperBound());

        // Fill in the error vector
        errorValues.setValue(noConstraint, constraint->getErrorValue());

        // If it's a contact constraint
        Contact* contact = dynamic_cast<Contact*>(constraint);
        if (contact) {
            // Get the lambda init value from the cache if exists
            ContactCachingInfo* contactInfo = contactCache.getContactCachingInfo(contact->getBody1(), contact->getBody2(), contact->getPoint());
            if (contactInfo) {
                // The last lambda init value was in the cache
                lambdaInit.setValue(noConstraint, contactInfo->lambda);
            }
            else {
                // The las lambda init value was not in the cache
                lambdaInit.setValue(noConstraint, 0.0);
            }
        }
        else {
            // Set the lambda init value
            lambdaInit.setValue(noConstraint, 0.0);
        }

        nbAuxConstraints = constraint->getNbAuxConstraints();

        // If the current constraint has auxiliary constraints
        if (nbAuxConstraints > 0) {
            
            // For each auxiliary constraints
            for (uint i=1; i<=nbAuxConstraints; i++) {
                // Fill in the J_sp matrix
                J_sp[noConstraint+i][0].changeSize(1, 6);
                J_sp[noConstraint+i][1].changeSize(1, 6);
                J_sp[noConstraint+i][0] = constraint->getAuxJacobian().getSubMatrix(i-1, 0, 1, 6);
                J_sp[noConstraint+i][1] = constraint->getAuxJacobian().getSubMatrix(i-1, 6, 1, 6);

                // Fill in the body mapping matrix
                bodyMapping[noConstraint+i][0] = constraint->getBody1();
                bodyMapping[noConstraint+i][1] = constraint->getBody2();

                // Fill in the init lambda value for the constraint
                lambdaInit.setValue(noConstraint+i, 0.0);
            }

            // Fill in the limit vectors for the auxilirary constraints
            lowerBounds.fillInSubVector(noConstraint+1, constraint->getAuxLowerBounds());
            upperBounds.fillInSubVector(noConstraint+1, constraint->getAuxUpperBounds());
        }

        noConstraint += 1 + nbAuxConstraints;
    }

    // For each current body that is implied in some constraint
    RigidBody* rigidBody;
    Body* body;
    Vector v(6);
    Vector f(6);
    uint b=0;
    for (std::set<Body*>::iterator it = constraintBodies.begin(); it != constraintBodies.end(); it++, b++) {
        body = *it;
        uint bodyNumber = bodyNumberMapping.at(body);
        
        // TODO : Use polymorphism and remove this downcasting
        rigidBody = dynamic_cast<RigidBody*>(body);
        assert(rigidBody != 0);
        
        // Compute the vector V1 with initial velocities values
        v.fillInSubVector(0, rigidBody->getLinearVelocity());
        v.fillInSubVector(3, rigidBody->getAngularVelocity());
        V1[bodyNumber].changeSize(6);
        V1[bodyNumber] = v;

        // Compute the vector Vconstraint with final constraint velocities
        Vconstraint[bodyNumber].changeSize(6);
        Vconstraint[bodyNumber].initWithValue(0.0);
        
        // Compute the vector with forces and torques values
        f.fillInSubVector(0, rigidBody->getExternalForce());
        f.fillInSubVector(3, rigidBody->getExternalTorque());
        Fext[bodyNumber].changeSize(6);
        Fext[bodyNumber] = f;

        // Compute the inverse sparse mass matrix
        Matrix mInv(6,6);
        mInv.initWithValue(0.0);
        if (rigidBody->getIsMotionEnabled()) {
            mInv.fillInSubMatrix(0, 0, rigidBody->getMassInverse() * Matrix::identity(3));
            mInv.fillInSubMatrix(3, 3, rigidBody->getInertiaTensorInverseWorld());
        }
        Minv_sp[bodyNumber].changeSize(6, 6);
        Minv_sp[bodyNumber] = mInv;
    }
}

// Free the memory that was allocated in the allocate() method
void ConstraintSolver::freeMemory() {

    activeConstraints.clear();
    bodyNumberMapping.clear();
    constraintBodies.clear();

    // Free the bodyMaping array
    for (uint i=0; i<nbConstraints; i++) {
        delete[] bodyMapping[i];
        delete[] J_sp[i];
    }
    delete[] bodyMapping;
    delete[] J_sp;
    delete[] B_sp[0];
    delete[] B_sp[1];
    delete[] B_sp;
    delete[] Minv_sp;
    delete[] V1;
    delete[] Vconstraint;
    delete[] Fext;
}

// Compute the vector b
void ConstraintSolver::computeVectorB(double dt) {
    uint indexBody1, indexBody2;
    double oneOverDT = 1.0/dt;
    
    b = errorValues * oneOverDT;

    for (uint c = 0; c<nbConstraints; c++) {
        // Substract 1.0/dt*J*V to the vector b
        indexBody1 = bodyNumberMapping[bodyMapping[c][0]];
        indexBody2 = bodyNumberMapping[bodyMapping[c][1]];
        b.setValue(c, b.getValue(c) - (J_sp[c][0] * V1[indexBody1]).getValue(0,0) * oneOverDT);
        b.setValue(c, b.getValue(c) - (J_sp[c][1] * V1[indexBody2]).getValue(0,0) * oneOverDT);

        // Substract J*M^-1*F_ext to the vector b
        b.setValue(c, b.getValue(c) - ((J_sp[c][0] * Minv_sp[indexBody1]) * Fext[indexBody1]
                 + (J_sp[c][1] * Minv_sp[indexBody2])*Fext[indexBody2]).getValue(0,0));
    }
}

// Compute the matrix B_sp
void ConstraintSolver::computeMatrixB_sp() {
    uint indexBody1, indexBody2;

    // For each constraint
    for (uint c = 0; c<nbConstraints; c++) {
        indexBody1 = bodyNumberMapping[bodyMapping[c][0]];
        indexBody2 = bodyNumberMapping[bodyMapping[c][1]];
        B_sp[0][c].changeSize(6,1);
        B_sp[1][c].changeSize(6,1);
        B_sp[0][c] = Minv_sp[indexBody1] * J_sp[c][0].getTranspose();
        B_sp[1][c] = Minv_sp[indexBody2] * J_sp[c][1].getTranspose();
    }
}

// Compute the vector V_constraint (which corresponds to the constraint part of
// the final V2 vector) according to the formula
// V_constraint = dt * (M^-1 * J^T * lambda)
// Note that we use the vector V to store both V1 and V_constraint.
// Note that M^-1 * J^T = B.
// This method is called after that the LCP solver have computed lambda
void ConstraintSolver::computeVectorVconstraint(double dt) {
    uint indexBody1, indexBody2;

    // Compute dt * (M^-1 * J^T * lambda
    for (uint i=0; i<nbConstraints; i++) {
        indexBody1 = bodyNumberMapping[bodyMapping[i][0]];
        indexBody2 = bodyNumberMapping[bodyMapping[i][1]];
        Vconstraint[indexBody1] = Vconstraint[indexBody1] + (B_sp[0][i] * lambda.getValue(i)).getVector() * dt;
        Vconstraint[indexBody2] = Vconstraint[indexBody2] + (B_sp[1][i] * lambda.getValue(i)).getVector() * dt;
    }
}

// Clear and Fill in the contact cache with the new lambda values
void ConstraintSolver::updateContactCache() {
    // Clear the contact cache
    contactCache.clear();
    
    // For each active constraint
    uint noConstraint = 0;
    for (uint c=0; c<activeConstraints.size(); c++) {

        // If it's a contact constraint
        Contact* contact = dynamic_cast<Contact*>(activeConstraints.at(c));
        if (contact) {
            // Create a new ContactCachingInfo
            ContactCachingInfo contactInfo(contact->getBody1(), contact->getBody2(), contact->getPoint(), lambda.getValue(noConstraint));

            // Add it to the contact cache
            contactCache.addContactCachingInfo(contactInfo);
        }

        noConstraint += 1 + activeConstraints.at(c)->getNbAuxConstraints();
    }
}

// Solve the current LCP problem
void ConstraintSolver::solve(double dt) {
    // Allocate memory for the matrices
    allocate();

    // Fill-in all the matrices needed to solve the LCP problem
    fillInMatrices();

    // Compute the vector b
    computeVectorB(dt);

    // Compute the matrix B
    computeMatrixB_sp();

    // Solve the LCP problem (computation of lambda)
    lcpSolver->setLambdaInit(lambdaInit);
    lcpSolver->solve(J_sp, B_sp, nbConstraints, nbBodies, bodyMapping, bodyNumberMapping, b, lowerBounds, upperBounds, lambda);

    // Update the contact chaching informations
    updateContactCache();

    // Compute the vector Vconstraint
    computeVectorVconstraint(dt);
}
