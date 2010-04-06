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
ConstraintSolver::ConstraintSolver(PhysicsWorld& physicsWorld)
                 :physicsWorld(physicsWorld), bodyMapping(0) , lcpSolver(LCPProjectedGaussSeidel(MAX_LCP_ITERATIONS)) {

}

// Destructor
ConstraintSolver::~ConstraintSolver() {

}

// Allocate all the matrices needed to solve the LCP problem
void ConstraintSolver::allocate() {
    unsigned int nbConstraints = 0;
    nbBodies = physicsWorld.getBodies().size();

    // TODO : Now we keep every bodies of the physics world in the "bodies" std:vector of the constraint solver.
    //        but maybe we could only keep track of the body that are part of some constraints.

    // For each constraint
    for (unsigned int i=0; i<physicsWorld.getConstraints().size(); ++i) {
        // Evaluate the constraint
        physicsWorld.getConstraints().at(i)->evaluate();

        // If the constraint is active
        if (physicsWorld.getConstraints().at(i)->isActive()) {
            activeConstraints.push_back(physicsWorld.getConstraints().at(i));

            // Update the size of the jacobian matrix
            nbConstraints += (1 + physicsWorld.getConstraints().at(i)->getNbAuxConstraints());
        }
    }

    bodyMapping = new Body**[nbConstraints];
    for (unsigned int i=0; i<nbConstraints; i++) {
        bodyMapping[i] = new Body*[2];
    }

    J_sp = Matrix(nbConstraints, 12);
    errorValues = Vector(nbConstraints);
    B_sp = Matrix(12, nbConstraints);
    b = Vector(nbConstraints);
    lambda = Vector(nbConstraints);
    lowerBounds = Vector(nbConstraints);
    upperBounds = Vector(nbConstraints);
    Minv_sp = Matrix(6*nbBodies, 6);
    V = Vector(6*nbBodies);
    Fext = Vector(6*nbBodies);
}

// Fill in all the matrices needed to solve the LCP problem
// Notice that all the active constraints should have been evaluated first
void ConstraintSolver::fillInMatrices() {

    // For each active constraint
    for (unsigned int c=0; c<activeConstraints.size(); c++) {
        Constraint* constraint = activeConstraints.at(c);

        // Fill in the J_sp matrix
        J_sp.fillInSubMatrix(c, 0, constraint->getBody1Jacobian());
        J_sp.fillInSubMatrix(c, 6, constraint->getBody2Jacobian());

        // Fill in the body mapping matrix
        bodyMapping[c][0] = constraint->getBody1();
        bodyMapping[c][1] = constraint->getBody2();

        // Fill in the limit vectors for the constraint
        lowerBounds.fillInSubVector(c, constraint->getLowerBound());
        upperBounds.fillInSubVector(c, constraint->getUpperBound());

        // Fill in the error vector
        errorValues.fillInSubVector(c, constraint->getErrorValue());

        unsigned int nbAuxConstraints = constraint->getNbAuxConstraints();

        // If the current constraint has auxiliary constraints
        if (nbAuxConstraints > 0) {
            // Fill in the J_sp matrix
            J_sp.fillInSubMatrix(c+1, 0, constraint->getAuxJacobian());

            // For each auxiliary constraints
            for (unsigned int i=1; i<nbAuxConstraints; i++) {
                // Fill in the body mapping matrix
                bodyMapping[c+i][0] = constraint->getBody1();
                bodyMapping[c+i][1] = constraint->getBody2();
            }

            // Fill in the limit vectors for the auxilirary constraints
            lowerBounds.fillInSubVector(c+1, constraint->getAuxLowerBounds());
            upperBounds.fillInSubVector(c+1, constraint->getAuxUpperBounds());
        }
    }

    // For each current body of the physics world
    for (unsigned int b=0; b<physicsWorld->getBodies().size(); b++) {
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
