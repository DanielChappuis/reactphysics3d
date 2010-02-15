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

// Constructor
ConstraintSolver::ConstraintSolver() {

}

// Destructor
ConstraintSolver~ConstraintSolver() {

}

// Allocate all the matrices needed to solve the LCP problem
void ConstraintSolver::allocate(std::vector<Constraint*>& constraints, double dt) {
    std::vector<Constraint*> activeConstraints;
    unsigned int sizeJacobian = 0;
    unsigned int nbAuxiliaryVars = 0;

    // For each constraint
    for (unsigned int i=0; i<constraints.size(); ++i) {
        // If the constraint is active
        if (constraints.at(i)->isActive()) {
            activeConstraints.push_back(constraints.at(i));

            // Evaluate the constraint
            constraints.at(i)->evaluate(dt);

            // Set the current jacobian index to the constraint
            constraints.at(i)->setJacobianIndex(sizeJacobian);

            // Update the size of the jacobian matrix
            sizeJacobian += constraints.at(i)->getNbJacobianRows();
        }
    }

    // For each active constraint
    for (unsigned int i=0; i<activeConstraints.size(); ++i) {
        // Set the auxiliary index
        activeConstraints.at(i)->setAuxiliaryIndex(sizeJacobian + nbAuxiliaryVars);

        // Update the number of auxiliary variables
        nbAuxiliaryVars += activeConstraints.at(i)->getNbAuxiliaryVars();
    }

    // TODO : Continue to implement this ...
}

// Fill in all the matrices needed to solve the LCP problem
void ConstraintSolver::fillInMatrices(std::vector<Constraint*> constraints) {

}

// Solve the current LCP problem
void ConstraintSolver::solve(std::vector<Constraint*> constraints, double dt) {

}
