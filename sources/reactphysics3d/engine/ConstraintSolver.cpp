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
    for (it = physicsWorld->getConstraintsBeginIterator(); it <physicsWorld->getConstraintsEndIterator(); it++) {
        constraint = *it;

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
            bodyNumberMapping.insert(std::pair<Body*, unsigned int>(constraint->getBody2(), bodyNumberMapping.size()));

            // Update the size of the jacobian matrix
            nbConstraints += (1 + constraint->getNbAuxConstraints());
        }
    }

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
    lowerBounds.changeSize(nbConstraints);
    upperBounds.changeSize(nbConstraints);
    Minv_sp = new Matrix[nbBodies];
    V = new Vector[nbBodies];
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
    for (uint b=0; b<nbBodies; b++) {
        body = constraintBodies.at(b);
        uint bodyNumber = bodyNumberMapping.at(body);
        
        // TODO : Use polymorphism and remove this downcasting
        rigidBody = dynamic_cast<RigidBody*>(body);
        assert(rigidBody != 0);
        
        // Compute the vector with velocities values
        v.fillInSubVector(0, rigidBody->getCurrentBodyState().getLinearVelocity());
        v.fillInSubVector(3, rigidBody->getCurrentBodyState().getAngularVelocity());
        V[bodyNumber].changeSize(6);
        V[bodyNumber] = v;
        
        // Compute the vector with forces and torques values
        f.fillInSubVector(0, rigidBody->getCurrentBodyState().getExternalForce());
        f.fillInSubVector(3, rigidBody->getCurrentBodyState().getExternalTorque());
        Fext[bodyNumber].changeSize(6);
        Fext[bodyNumber] = f;

        // Compute the inverse sparse mass matrix
        Matrix mInv(6,6);
        mInv.initWithValue(0.0);
        mInv.fillInSubMatrix(0, 0, rigidBody->getCurrentBodyState().getMassInverse().getValue() * Matrix::identity(3));
        mInv.fillInSubMatrix(3, 3, rigidBody->getCurrentBodyState().getInertiaTensorInverse());
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
    delete[] V;
    delete[] Fext;
}

// Compute the vector b
void ConstraintSolver::computeVectorB(double dt) {
    uint indexBody1, indexBody2;
    double oneOverDT = 1.0/dt;
    
    b = errorValues * oneOverDT;

    for (uint c = 0; c<nbConstraints; c++) {
        int size1 = J_sp[c][0].getNbColumn(); // TODO : Delete this
        // Substract 1.0/dt*J*V to the vector b
        indexBody1 = bodyNumberMapping[bodyMapping[c][0]];
        indexBody2 = bodyNumberMapping[bodyMapping[c][1]];
        b.setValue(c, b.getValue(c) - (J_sp[c][0] * V[indexBody1]).getValue(0,0) * oneOverDT);
        b.setValue(c, b.getValue(c) - (J_sp[c][1] * V[indexBody2]).getValue(0,0) * oneOverDT);

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

// Compute the vector V2 according to the formula
// V2 = dt * (M^-1 * J^T * lambda + M^-1 * F_ext) + V1
// Note that we use the vector V to store both V1 and V2 and that at the beginning
// of this method, the vector V already contains the vector V1.
// Note that M^-1 * J^T = B.
// This method is called after that the LCP solver have computed lambda
void ConstraintSolver::computeVectorV(double dt) {
    uint indexBody1, indexBody2;

    // Compute dt * (M^-1 * J^T * lambda
    for (uint i=0; i<nbConstraints; i++) {
        indexBody1 = bodyNumberMapping[bodyMapping[i][0]];
        indexBody2 = bodyNumberMapping[bodyMapping[i][1]];
        V[indexBody1] = V[indexBody1] + (B_sp[indexBody1][i] * lambda.getValue(i)).getVector() * dt;
        V[indexBody2] = V[indexBody2] + (B_sp[indexBody2][i] * lambda.getValue(i)).getVector() * dt;
    }

    // Compute dt * (M^-1 * F_ext)
    for (uint i=0; i<nbBodies; i++) {
        V[i] = V[i] + (Minv_sp[i] * Fext[i]).getVector() * dt;
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
    Vector lambdaInit(nbConstraints);
    lcpSolver->setLambdaInit(lambdaInit);
    lcpSolver->solve(J_sp, B_sp, nbConstraints, nbBodies, bodyMapping, bodyNumberMapping, b, lowerBounds, upperBounds, lambda);

    // Compute the vector V2
    computeVectorV(dt);

    // Update the velocity of each body
    // TODO : Put this code somewhere else
    for (int i=0; i<nbBodies; i++) {
        RigidBody* body = dynamic_cast<RigidBody*>(constraintBodies.at(i));
        //std::cout << "Velocity Y before : " << body->getCurrentBodyState().getLinearVelocity().getY() << std::endl;
        //std::cout << "Velocity Y after  : " << V[bodyNumberMapping[constraintBodies.at(i)]].getValue(1) << std::endl;
    }

    freeMemory();
}
