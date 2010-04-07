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

    // For each constraint
    for (unsigned int c=0; c<physicsWorld.getConstraints().size(); c++) {
        Constraint* constraint = physicsWorld.getConstraints().at(c);

        // Evaluate the constraint
        constraint->evaluate();

        // If the constraint is active
        if (constraint->isActive()) {
            activeConstraints.push_back(constraint);

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
    for (unsigned int b=0; b<nbBodies; b++) {
        // Compute the vector with velocities values
        V.fillInSubVector(b*3, )
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

    activeConstraints.clear();
    bodyNumberMapping.clear();

    // Free the bodyMaping array
    for (unsigned int i=0; i<nbBodies; i++) {
        delete[] bodyMapping[i];
    }
    delete[] bodyMapping;
}

// Solve the current LCP problem
void ConstraintSolver::solve(double dt) {
    // Allocate memory for the matrices
    allocate();

    // Fill-in all the matrices needed to solve the LCP problem
    fillInMatrices();

    // Solve the LCP problem (computation of lambda)
    lcpSolver.solve(A, b, lowLimits, highLimits, lambda);

    // TODO : Implement this method ...

    freeMemory();
}
