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
#include "../typeDefinitions.h"
#include "../constraint/Constraint.h"
#include "../mathematics/lcp/LCPSolver.h"
#include "ContactCache.h"
#include "PhysicsWorld.h"
#include <map>
#include <set>

// ReactPhysics3D namespace
namespace reactphysics3d {

// Constants
const uint MAX_LCP_ITERATIONS = 10;     // Maximum number of iterations when solving a LCP problem
const double AV_COUNTER_LIMIT = 500;    // Maximum number value of the avBodiesCounter or avConstraintsCounter
const double AV_PERCENT_TO_FREE = 0.5;  // We will free the memory if the current nb of bodies (or constraints) is
                                        // less than AV_PERCENT_TO_FREE * bodiesCapacity (or constraintsCapacity). This
                                        // is used to avoid to keep to much memory for a long time if the system doesn't
                                        // need that memory. This value is between 0.0 and 1.0

 /*  -------------------------------------------------------------------
    Class ConstrainSolver :
        This class represents the constraint solver. The goal is to
        solve A constraint-base LCP problem.
    -------------------------------------------------------------------
*/
class ConstraintSolver {
    protected:
        PhysicsWorld* physicsWorld;                             // Reference to the physics world
        LCPSolver* lcpSolver;                                   // LCP Solver
        ContactCache contactCache;                              // Contact cache
        std::vector<Constraint*> activeConstraints;             // Current active constraints in the physics world
        uint nbConstraints;                                     // Total number of constraints (with the auxiliary constraints)
        uint nbBodies;                                          // Current number of bodies in the physics world
        uint constraintsCapacity;                               // Number of constraints that are currently allocated in memory in the solver
        uint bodiesCapacity;                                    // Number of bodies that are currently allocated in memory in the solver
        uint avConstraintsCapacity;                             // Average constraint capacity
        uint avBodiesCapacity;                                  // Average bodies capacity
        uint avBodiesNumber;                                    // Total bodies number for average computation
        uint avConstraintsNumber;                               // Total constraints number for average computation
        uint avBodiesCounter;                                   // Counter used to compute the average
        uint avConstraintsCounter;
        std::set<Body*> constraintBodies;                       // Bodies that are implied in some constraint
        std::map<Body*, uint> bodyNumberMapping;                // Map a body pointer with its index number
        Body*** bodyMapping;                                    // 2-dimensional array that contains the mapping of body reference
                                                                // in the J_sp and B_sp matrices. For instance the cell bodyMapping[i][j] contains
                                                                // the pointer to the body that correspond to the 1x6 J_ij matrix in the
                                                                // J_sp matrix. A integer body index refers to its index in the "bodies" std::vector
        Matrix** J_sp;                                          // 2-dimensional array thar correspond to the sparse representation of the jacobian matrix of all constraints
                                                                // The dimension of this array is nbConstraints times 2. Each cell will contain
                                                                // a 1x6 matrix
        Matrix** B_sp;                                          // 2-dimensional array that correspond to a useful matrix in sparse representation
                                                                // The dimension of this array is 2 times nbConstraints. Each cell will contain
                                                                // a 6x1 matrix
        Vector b;                                               // Vector "b" of the LCP problem
        Vector lambda;                                          // Lambda vector of the LCP problem
        Vector lambdaInit;                                      // Lambda init vector for the LCP solver
        Vector errorValues;                                     // Error vector of all constraints
        Vector lowerBounds;                                     // Vector that contains the low limits for the variables of the LCP problem
        Vector upperBounds;                                     // Vector that contains the high limits for the variables of the LCP problem
        Matrix* Minv_sp;                                        // Sparse representation of the Matrix that contains information about mass and inertia of each body
                                                                // This is an array of size nbBodies that contains in each cell a 6x6 matrix
        Vector* V1;                                             // Array that contains for each body the Vector that contains linear and angular velocities
                                                                // Each cell contains a 6x1 vector with linear and angular velocities
        Vector* Vconstraint;                                    // Same kind of vector as V1 but contains the final constraint velocities
        Vector* Fext;                                           // Array that contains for each body the vector that contains external forces and torques
                                                                // Each cell contains a 6x1 vector with external force and torque.
        void initialize();                                      // Initialize the constraint solver before each solving
        void allocate();                                        // Allocate all the memory needed to solve the LCP problem
        void fillInMatrices();                                  // Fill in all the matrices needed to solve the LCP problem
        void computeVectorB(double dt);                         // Compute the vector b
        void computeMatrixB_sp();                               // Compute the matrix B_sp
        void computeVectorVconstraint(double dt);               // Compute the vector V2
        void updateContactCache();                              // Clear and Fill in the contact cache with the new lambda values
        void freeMemory(bool freeBodiesMemory);                 // Free some memory previously allocated for the constraint solver
        
    public:
        ConstraintSolver(PhysicsWorld* world);                                  // Constructor
        virtual ~ConstraintSolver();                                            // Destructor
        void solve(double dt);                                                  // Solve the current LCP problem
        bool isConstrainedBody(Body* body) const;                               // Return true if the body is in at least one constraint
        Vector3D getConstrainedLinearVelocityOfBody(Body* body);                // Return the constrained linear velocity of a body after solving the LCP problem
        Vector3D getConstrainedAngularVelocityOfBody(Body* body);               // Return the constrained angular velocity of a body after solving the LCP problem
        void cleanup();
};

// Return true if the body is in at least one constraint
inline bool ConstraintSolver::isConstrainedBody(Body* body) const {
    if(constraintBodies.find(body) != constraintBodies.end()) {
        return true;
    }
    return false;
}

// Return the constrained linear velocity of a body after solving the LCP problem
inline Vector3D ConstraintSolver::getConstrainedLinearVelocityOfBody(Body* body) {
    assert(isConstrainedBody(body));
    Vector vec = Vconstraint[bodyNumberMapping[body]].getSubVector(0, 3);
    return Vector3D(vec.getValue(0), vec.getValue(1), vec.getValue(2));

}

// Return the constrained angular velocity of a body after solving the LCP problem
inline Vector3D ConstraintSolver::getConstrainedAngularVelocityOfBody(Body* body) {
    assert(isConstrainedBody(body));
    Vector vec = Vconstraint[bodyNumberMapping[body]].getSubVector(3, 3);
    return Vector3D(vec.getValue(0), vec.getValue(1), vec.getValue(2));
}

// Solve the current LCP problem
inline void ConstraintSolver::solve(double dt) {
    // Allocate memory for the matrices
    initialize();

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

// Cleanup of the constraint solver
inline void ConstraintSolver::cleanup() {
    bodyNumberMapping.clear();
    constraintBodies.clear();
    activeConstraints.clear();
}

} // End of ReactPhysics3D namespace

#endif