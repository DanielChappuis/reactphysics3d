/********************************************************************************
* ReactPhysics3D physics library, http://code.google.com/p/reactphysics3d/      *
* Copyright (c) 2010 Daniel Chappuis                                            *
*********************************************************************************
*                                                                               *
* Permission is hereby granted, free of charge, to any person obtaining a copy  *
* of this software and associated documentation files (the "Software"), to deal *
* in the Software without restriction, including without limitation the rights  *
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell     *
* copies of the Software, and to permit persons to whom the Software is         *
* furnished to do so, subject to the following conditions:                      *
*                                                                               *
* The above copyright notice and this permission notice shall be included in    *
* all copies or substantial portions of the Software.                           *
*                                                                               *
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR    *
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,      *
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE   *
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER        *
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, *
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN     *
* THE SOFTWARE.                                                                 *
********************************************************************************/

#ifndef CONSTRAINTSOLVER_H
#define CONSTRAINTSOLVER_H

// Libraries
#include "../constants.h"
#include "../constraint/Constraint.h"
#include "../mathematics/lcp/LCPSolver.h"
#include "ContactCache.h"
#include "PhysicsWorld.h"
#include <map>
#include <set>

// ReactPhysics3D namespace
namespace reactphysics3d {

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
        Matrix1x6** J_sp;                                          // 2-dimensional array thar correspond to the sparse representation of the jacobian matrix of all constraints
                                                                // The dimension of this array is nbConstraints times 2. Each cell will contain
                                                                // a 1x6 matrix
        Vector6D** B_sp;                                          // 2-dimensional array that correspond to a useful matrix in sparse representation
                                                                // The dimension of this array is 2 times nbConstraints. Each cell will contain
                                                                // a 6x1 matrix
        Vector b;                                               // Vector "b" of the LCP problem
        Vector lambda;                                          // Lambda vector of the LCP problem
        Vector lambdaInit;                                      // Lambda init vector for the LCP solver
        Vector errorValues;                                     // Error vector of all constraints
        Vector lowerBounds;                                     // Vector that contains the low limits for the variables of the LCP problem
        Vector upperBounds;                                     // Vector that contains the high limits for the variables of the LCP problem
        Matrix6x6* Minv_sp;                                        // Sparse representation of the Matrix that contains information about mass and inertia of each body
                                                                // This is an array of size nbBodies that contains in each cell a 6x6 matrix
        Vector6D* V1;                                             // Array that contains for each body the Vector that contains linear and angular velocities
                                                                // Each cell contains a 6x1 vector with linear and angular velocities
        Vector6D* Vconstraint;                                    // Same kind of vector as V1 but contains the final constraint velocities
        Vector6D* Fext;                                           // Array that contains for each body the vector that contains external forces and torques
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
    const Vector6D& vec = Vconstraint[bodyNumberMapping[body]];
    return Vector3D(vec.getValue(0), vec.getValue(1), vec.getValue(2));

}

// Return the constrained angular velocity of a body after solving the LCP problem
inline Vector3D ConstraintSolver::getConstrainedAngularVelocityOfBody(Body* body) {
    assert(isConstrainedBody(body));
    const Vector6D& vec = Vconstraint[bodyNumberMapping[body]];
    return Vector3D(vec.getValue(3), vec.getValue(4), vec.getValue(5));
}

// Cleanup of the constraint solver
inline void ConstraintSolver::cleanup() {
    bodyNumberMapping.clear();
    constraintBodies.clear();
    activeConstraints.clear();
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

} // End of ReactPhysics3D namespace

#endif