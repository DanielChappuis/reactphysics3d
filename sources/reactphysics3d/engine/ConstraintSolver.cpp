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
#include "LCPProjectedGaussSeidel.h"

using namespace reactphysics3d;

// Constructor
ConstraintSolver::ConstraintSolver()
                 :bodies(0), nbBodies(0), bodyMapping(0) {
    // Creation of the LCP Solver
    lcpSolver = new LCPProjectedGaussSeidel(MAX_LCP_ITERATIONS);
}

// Destructor
ConstraintSolver::~ConstraintSolver() {

}

// Allocate all the matrices needed to solve the LCP problem
void ConstraintSolver::allocate(std::vector<Constraint*>& constraints, std::vector<Body*>& bodies) {
    unsigned int sizeJacobian = 0;
    this->bodies = bodies;
    this->nbBodies = bodies.size();

    // For each constraint
    for (unsigned int i=0; i<constraints.size(); ++i) {
        // Evaluate the constraint
        constraints.at(i)->evaluate(dt);

        // If the constraint is active
        if (constraints.at(i)->isActive()) {
            activeConstraints.push_back(constraints.at(i));

            /Description/ Update the size of the jacobian matrix
            sizeJacobian += (1 + constraints.at(i)->getNbAuxConstraints());
        }
    }

    // Allocate all the vectors and matrices
    J_sp = Matrix(sizeJacobian, 12);

    bodyMapping = new unsigned int[nbBodies];
    for (unsigned int i=0; i<nbBodies; i++) {
        bodyMapping[i] = new unsigned int[2];
    }

    errorVector = Vector(sizeJacobian);
    B_sp = Matrix(6*nbBodies, 2;
    b = Vector(totalSize);
    lambda = Vector(totalSize);
    lowLimits = Vector(totalSize);
    highLimits = Vector(totalSize);
    Minv_sp = Matrix(3*nbBodies, 6*nbBodies);
    V = Vector(6*nbBodies);
    Fc = Vector(6*nbBodies);
}

// Fill in all the matrices needed to solve the LCP problem
// Notice that all the active constraints should have been evaluated first
void ConstraintSolver::fillInMatrices() {
    int i,j;

    // For each active constraint
    for (unsigned int c=0; c<activeConstraints.size(); ++c) {
        i = activeConstraints.at(c)->getJacobianIndex();
        //j = i + activeConstraint.at(c)->getNbJacobianRows();
        J.fillInSubMatrix(i, 0, activeConstraints.at(c)->getBody1LinearJacobian());
        J.fillInSubMatrix(i, 3, activeConstraints.at(c)->getBody2LinearJacobian());
        J.fillInSubMatrix(i, 6, activeConstraints.at(c)->getBody1AngularJacobian());
        J.fillInSubMatrix(i, 9, activeConstraints.at(c)->getBody2AngularJacobian());
        errorVector.fillInSubVector(i, activeConstraints.at(c)->getErrorVector());
    }

    // For each active constraint
    for (unsigned int c=0; c<activeConstraints.size(); ++c) {
        i = activeConstraints.at(c)->getNbAuxiliaryVars();
        // TODO : add activeConstraints.at(c)->getAuxiliaryRowsAndCols(..., ...)
        b.fillInSubVector(i, activeConstraints.at(c)->getRightHandSideVector());
    }

    // For each current body of the simulation
    for (unsigned int b=0; b<nbBodies; ++b) {
        i = 6*b;
        Minv.fillInSubMatrix(i, 0, bodies.at(b)->getCurrentBodyState().getMassInverse().getValue() * Matrix::identity(3));
        Minv.fillInSubMatrix(i+3, 3, bodies.at(b)->getCurrentBodyState().getInertiaTensorInverse());
        u.fillInSubVector(i, bodies.at(b)->getCurrentBodyState().getLinearVelocity());
        u.fillInSubVector(i+3, bodies.at(b)->getCurrentBodyState().getAngularVelocity());
        fExt.fillInSubVector(i, bodies.at(b)->getCurrentBodyState().getExternalForce());
        Vector3D externalTorque = bodies.at(b)->getCurrentBodyState().getExternalTorque();
        Matrix3x3 inertia = bodies.at(b)->getInertiaTensor();
        Vector3D angularVelocity = bodies.at(b)->getCurrentBodyState().getAngularVelocity();
        fExt.fillInSubVector(i+3, externalTorque - (angularVelocity.crossProduct(inertia*angularVelocity)));
    }
}

// Free the memory that was allocated in the allocate() method
void ConstraintSolver::freeMemory() {
    // Free the bodyMaping array
    for (unsigned int i=0; i<nbBodies; i++) {
        delete[] bodyMapping[i];
    }
    delete[] bodyMapping;
}

// Solve the current LCP problem
void ConstraintSolver::solve(std::vector<Constraint*>& constraints, std::vector<Body*>* bodies, double dt) {
    // Update the current set of bodies in the physics world
    this->bodies = bodies;

    // Delete all the current actice constraints
    activeConstraints.clear();

    // Allocate memory for the matrices
    allocate(constraints, dt);

    // Fill-in all the matrices needed to solve the LCP problem
    fillInMatrices();

    // Solve the LCP problem (computation of lambda)
    lcpSolver.solve(A, b, lowLimits, highLimits, lambda);

    // TODO : Implement this method ...
}
