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
ConstraintSolver::ConstraintSolver(PhysicsWorld& world)
                 :physicsWorld(world), bodyMapping(0) , lcpSolver(LCPProjectedGaussSeidel(MAX_LCP_ITERATIONS)) {

}

// Destructor
ConstraintSolver::~ConstraintSolver() {

}

// Allocate all the matrices needed to solve the LCP problem
void ConstraintSolver::allocate() {
    uint nbConstraints = 0;

    // For each constraint
    for (uint c=0; c<physicsWorld.getConstraints().size(); c++) {
        Constraint* constraint = physicsWorld.getConstraints().at(c);

        // Evaluate the constraint
        constraint->evaluate();

        // If the constraint is active
        if (constraint->isActive()) {
            activeConstraints.push_back(constraint);

            // Add the two bodies of the constraint in the constraintBodies list
            constraintBodies.push_back(constraint->getBody1());
            constraintBodies.push_back(constraint->getBody2());

            // Fill in the body number maping
            bodyNumberMapping.insert(std::pair<Body*, unsigned int>(constraint->getBody1(), bodyNumberMapping.size()));
            bodyNumberMapping.insert(std::pair<Body*, unsigned int>(constraint->getBody1(), bodyNumberMapping.size()));

            // Update the size of the jacobian matrix
            nbConstraints += (1 + constraint->getNbAuxConstraints());
        }
    }

    // Compute the number of bodies that are part of some active constraint
    nbBodies = bodyNumberMapping.size();

    bodyMapping = new Body**[nbConstraints];
    J_sp = new Matrix*[nbConstraints];
    B_sp = new Matrix*[2];
    for (uint i=0; i<nbConstraints; i++) {
        bodyMapping[i] = new Body*[2];
        J_sp[i] = new Matrix[2];
        B_sp[i] = new Matrix[nbConstraints];
    }

    errorValues = Vector(nbConstraints);
    b = Vector(nbConstraints);
    lambda = Vector(nbConstraints);
    lowerBounds = Vector(nbConstraints);
    upperBounds = Vector(nbConstraints);
    Minv_sp = new Matrix[nbBodies];
    V = new Vector[nbBodies];
    Fext = new Vector[nbBodies];
}

// Fill in all the matrices needed to solve the LCP problem
// Notice that all the active constraints should have been evaluated first
void ConstraintSolver::fillInMatrices() {

    // For each active constraint
    for (uint c=0; c<activeConstraints.size(); c++) {
        Constraint* constraint = activeConstraints.at(c);

        // Fill in the J_sp matrix
        J_sp[c][0] = constraint->getBody1Jacobian();
        J_sp[c][1] = constraint->getBody2Jacobian();


        // Fill in the body mapping matrix
        bodyMapping[c][0] = constraint->getBody1();
        bodyMapping[c][1] = constraint->getBody2();

        // Fill in the limit vectors for the constraint
        lowerBounds.fillInSubVector(c, constraint->getLowerBound());
        upperBounds.fillInSubVector(c, constraint->getUpperBound());

        // Fill in the error vector
        errorValues.fillInSubVector(c, constraint->getErrorValue());

        uint nbAuxConstraints = constraint->getNbAuxConstraints();

        // If the current constraint has auxiliary constraints
        if (nbAuxConstraints > 0) {
            
            // For each auxiliary constraints
            for (uint i=1; i<=nbAuxConstraints; i++) {
                // Fill in the J_sp matrix
                J_sp[c+i][0] = constraint->getAuxJacobian().getSubMatrix(i-1, 0, 1, 6);
                J_sp[c+i][1] = constraint->getAuxJacobian().getSubMatrix(i-1, 6, 1, 6);

                // Fill in the body mapping matrix
                bodyMapping[c+i][0] = constraint->getBody1();
                bodyMapping[c+i][1] = constraint->getBody2();
            }

            // Fill in the limit vectors for the auxilirary constraints
            lowerBounds.fillInSubVector(c+1, constraint->getAuxLowerBounds());
            upperBounds.fillInSubVector(c+1, constraint->getAuxUpperBounds());
        }
    }

    // For each current body that is implied in some constraint
    RigidBody* rigidBody;
    Body* body;
    Vector v(6);
    Vector f(6);
    for (uint b=0; b<nbBodies; b++) {
        body = constraintBodies.at(b);
        uint bodyNumber = bodyNumberMapping.at(body);
        
        // TODO : Use polymorphism and remove this downcasting
        rigidBody = dynamic_cast<RigidBody*>(body);
        assert(rigidBody != 0);
        
        // Compute the vector with velocities values
        v.fillInSubVector(0, rigidBody->getCurrentBodyState().getLinearVelocity());
        v.fillInSubVector(3, rigidBody->getCurrentBodyState().getAngularVelocity());
        V[bodyNumber] = v;
        
        // Compute the vector with forces and torques values
        f.fillInSubVector(0, rigidBody->getCurrentBodyState().getExternalForce());
        f.fillInSubVector(3, rigidBody->getCurrentBodyState().getExternalTorque());
        Fext[bodyNumber] = f;

        // Compute the inverse sparse mass matrix
        Matrix mInv(6,6);
        mInv.initWithValue(0.0);
        mInv.fillInSubMatrix(0, 0, rigidBody->getCurrentBodyState().getMassInverse().getValue() * Matrix::identity(3));
        mInv.fillInSubMatrix(3, 3, rigidBody->getCurrentBodyState().getInertiaTensorInverse());
        Minv_sp[bodyNumber] = mInv;
    }
}

// Free the memory that was allocated in the allocate() method
void ConstraintSolver::freeMemory() {

    activeConstraints.clear();
    bodyNumberMapping.clear();

    // Free the bodyMaping array
    for (uint i=0; i<nbBodies; i++) {
        delete[] bodyMapping[i];
    }
    delete[] bodyMapping;
    delete[] J_sp;
    delete[] B_sp;
    delete[] Minv_sp;
    delete[] V;
    delete[] Fext;
}

// Compute the vector b
void ConstraintSolver::computeVectorB(double dt) {
    uint indexBody1, indexBody2;
    double oneOverDT = 1.0/dt;
    
    b = errorValues * oneOverDT;

    for (uint c = 0; c<activeConstraints.size(); c++) {
        // Substract 1.0/dt*J*V to the vector b
        indexBody1 = bodyNumberMapping[bodyMapping[c][0]];
        indexBody2 = bodyNumberMapping[bodyMapping[c][1]];
        b.setValue(c, b.getValue(c) - oneOverDT * (J_sp[c][0] * V[indexBody1]).getValue(0,0));
        b.setValue(c, b.getValue(c) - oneOverDT * (J_sp[c][1] * V[indexBody2]).getValue(0,0));

        // Substract J*M^-1*F_ext to the vector b
        b.setValue(c, b.getValue(c) - ((J_sp[c][0] * Minv_sp[indexBody1]) * Fext[indexBody1]
                 + (J_sp[c][1] * Minv_sp[indexBody2])*Fext[indexBody2]).getValue(0,0));
    }
}

// Compute the matrix B_sp
void ConstraintSolver::computeMatrixB_sp() {
    uint indexBody1;
    uint indexBody2;

    // For each constraint
    for (uint c = 0; c<activeConstraints.size(); c++) {
        indexBody1 = bodyNumberMapping[bodyMapping[c][0]];
        indexBody2 = bodyNumberMapping[bodyMapping[c][1]];
        B_sp[0][c] = Minv_sp[indexBody1] * J_sp[c][0].getTranspose();
        B_sp[1][c] = Minv_sp[indexBody2] * J_sp[c][1].getTranspose();
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
    //lcpSolver.solve(A, b, lowLimits, highLimits, lambda);

    // TODO : Implement this method ...

    freeMemory();
}
