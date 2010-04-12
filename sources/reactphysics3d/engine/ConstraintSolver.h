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

#ifndef CONSTRAINTSOLVER_H
#define CONSTRAINTSOLVER_H

// Libraries
#include "../constraint/Constraint.h"
#include "PhysicsWorld.h"
#include <map>

// ReactPhysics3D namespace
namespace reactphysics3d {

// Constants
const unsigned int MAX_LCP_ITERATIONS = 10;     // Maximum number of iterations when solving a LCP problem

 /*  -------------------------------------------------------------------
    Class ConstrainSolver :
        This class represents the constraint solver. The goal is to
        solve A constraint-base LCP problem.
    -------------------------------------------------------------------
*/
class ConstraintSolver {
    protected:
        LCPSolver& lcpSolver;                                               // LCP Solver
        PhysicsWorld& physicsWorld;                                         // Reference to the physics world
        std::vector<Constraint*> activeConstraints;                         // Current active constraints in the physics world
        std::vector<Body*> constraintBodies;                                // Bodies that are implied in some constraint
        unsigned int nbBodies;                                              // Current number of bodies in the physics world
        std::map<Body*, unsigned int> bodyNumberMapping;                    // Map a body pointer with its index number
        Body** bodyMapping;                                                 // 2-dimensional array that contains the mapping of body reference
                                                                            // in the J_sp and B_sp matrices. For instance the cell bodyMapping[i][j] contains
                                                                            // the integer index of the body that correspond to the 1x6 J_ij matrix in the
                                                                            // J_sp matrix. A integer body index refers to its index in the "bodies" std::vector
        Matrix J_sp;                                                        // Sparse representation of the jacobian matrix of all constraints
        Matrix B_sp;                                                        // Useful matrix in sparse representation
        Vector b;                                                           // Vector "b" of the LCP problem
        Vector lambda;                                                      // Lambda vector of  the LCP problem
        Vector errorValues;                                                 // Error vector of all constraints
        Vector lowerBounds;                                                 // Vector that contains the low limits for the variables of the LCP problem
        Vector upperBounds;                                                 // Vector that contains the high limits for the variables of the LCP problem
        Matrix Minv_sp;                                                     // Sparse representation of the Matrix that contains information about mass and inertia of each body
        Vector V;                                                           // Vector that contains linear and angular velocities of each body
        Vector Fext;                                                        // Vector that contains external forces and torques of each body
        void allocate();                                                    // Allocate all the matrices needed to solve the LCP problem
        void fillInMatrices();                                              // Fill in all the matrices needed to solve the LCP problem
        void freeMemory();                                                  // Free the memory that was allocated in the allocate() method

    public:
        ConstraintSolver();                                                 // Constructor
        virtual ~ConstraintSolver();                                        // Destructor
        void solve(double dt);                                              // Solve the current LCP problem
};

} // End of ReactPhysics3D namespace

#endif
