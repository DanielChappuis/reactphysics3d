/********************************************************************************
* ReactPhysics3D physics library, http://code.google.com/p/reactphysics3d/      *
* Copyright (c) 2010-2012 Daniel Chappuis                                       *
*********************************************************************************
*                                                                               *
* This software is provided 'as-is', without any express or implied warranty.   *
* In no event will the authors be held liable for any damages arising from the  *
* use of this software.                                                         *
*                                                                               *
* Permission is granted to anyone to use this software for any purpose,         *
* including commercial applications, and to alter it and redistribute it        *
* freely, subject to the following restrictions:                                *
*                                                                               *
* 1. The origin of this software must not be misrepresented; you must not claim *
*    that you wrote the original software. If you use this software in a        *
*    product, an acknowledgment in the product documentation would be           *
*    appreciated but is not required.                                           *
*                                                                               *
* 2. Altered source versions must be plainly marked as such, and must not be    *
*    misrepresented as being the original software.                             *
*                                                                               *
* 3. This notice may not be removed or altered from any source distribution.    *
*                                                                               *
********************************************************************************/

#ifndef CONSTRAINT_SOLVER_H
#define CONSTRAINT_SOLVER_H

// Libraries
#include "../configuration.h"
#include "../constraint/Constraint.h"
#include "PhysicsWorld.h"
#include <map>
#include <set>

// ReactPhysics3D namespace
namespace reactphysics3d {
    
    
/*  -------------------------------------------------------------------
    Class ConstrainSolver :
        This class represents the constraint solver. The constraint solver
        is based on the theory from the paper "Iterative Dynamics with
        Temporal Coherence" from Erin Catto. We keep the same notations as
        in the paper. The idea is to construct a LCP problem and then solve
        it using a Projected Gauss Seidel (PGS) solver.
 
        The idea is to solve the following system for lambda :
                JM^-1J^T * lamdba - 1/dt * b_error + 1/dt * JV^1 + JM^-1F_ext >= 0
        
        By default, error correction using first world order projections (described
        by Erleben in section 4.16 in his PhD thesis "Stable, Robust, and
        Versatile Multibody Dynamics Animation") is used for very large penetration
        depths. Error correction with projection requires to solve another LCP problem
        that is simpler than the above one and by considering only the contact
        constraints. The LCP problem for error correction is the following one :
                J_contact M^-1 J_contact^T * lambda + 1/dt * -d >= 0
 
        where "d" is a vector with penetration depths. If the penetration depth of
       a given contact is not very large, we use the Baumgarte error correction (see
       the paper from Erin Catto).
        
    -------------------------------------------------------------------
*/
class ConstraintSolver {
    private:
        PhysicsWorld* physicsWorld;                     // Reference to the physics world
        std::vector<Constraint*> activeConstraints;     // Current active constraints in the physics world
        bool isErrorCorrectionActive;                   // True if error correction (with world order) is active
        uint nbIterationsLCP;                           // Number of iterations of the LCP solver
        uint nbIterationsLCPErrorCorrection;            // Number of iterations of the LCP solver for error correction
        uint nbConstraints;                             // Total number of constraints (with the auxiliary constraints)
        uint nbConstraintsError;                        // Number of constraints for error correction projection (only contact constraints)
        uint nbBodies;                                  // Current number of bodies in the physics world
        std::set<Body*> constraintBodies;               // Bodies that are implied in some constraint
        std::map<Body*, uint> bodyNumberMapping;        // Map a body pointer with its index number
        Body* bodyMapping[NB_MAX_CONSTRAINTS][2];       // 2-dimensional array that contains the mapping of body reference
                                                        // in the J_sp and B_sp matrices. For instance the cell bodyMapping[i][j] contains
                                                        // the pointer to the body that correspond to the 1x6 J_ij matrix in the
                                                        // J_sp matrix. An integer body index refers to its index in the "bodies" std::vector
        Body* bodyMappingError[NB_MAX_CONSTRAINTS][2];  // Same as bodyMapping but for error correction projection
        decimal J_sp[NB_MAX_CONSTRAINTS][2*6];          // 2-dimensional array that correspond to the sparse representation of the jacobian matrix of all constraints
                                                        // This array contains for each constraint two 1x6 Jacobian matrices (one for each body of the constraint)
                                                        // a 1x6 matrix
        decimal B_sp[2][6*NB_MAX_CONSTRAINTS];          // 2-dimensional array that correspond to a useful matrix in sparse representation
                                                        // This array contains for each constraint two 6x1 matrices (one for each body of the constraint)
                                                        // a 6x1 matrix
        decimal J_spError[NB_MAX_CONSTRAINTS][2*6];     // Same as J_sp for error correction projection (only contact constraints)
        decimal B_spError[2][6*NB_MAX_CONSTRAINTS];     // Same as B_sp for error correction projection (only contact constraints)
        decimal b[NB_MAX_CONSTRAINTS];                  // Vector "b" of the LCP problem
        decimal bError[NB_MAX_CONSTRAINTS];             // Vector "b" of the LCP problem for error correction projection
        decimal d[NB_MAX_CONSTRAINTS];                  // Vector "d"
        decimal a[6*NB_MAX_BODIES];                     // Vector "a"
        decimal aError[6*NB_MAX_BODIES];                // Vector "a" for error correction projection
        decimal penetrationDepths[NB_MAX_CONSTRAINTS];  // Array of penetration depths for error correction projection
        decimal lambda[NB_MAX_CONSTRAINTS];             // Lambda vector of the LCP problem
        decimal lambdaError[NB_MAX_CONSTRAINTS];        // Lambda vector of the LCP problem for error correction projections
        decimal lambdaInit[NB_MAX_CONSTRAINTS];         // Lambda init vector for the LCP solver
        decimal errorValues[NB_MAX_CONSTRAINTS];        // Error vector of all constraints
        decimal lowerBounds[NB_MAX_CONSTRAINTS];        // Vector that contains the low limits for the variables of the LCP problem
        decimal upperBounds[NB_MAX_CONSTRAINTS];        // Vector that contains the high limits for the variables of the LCP problem
        decimal lowerBoundsError[NB_MAX_CONSTRAINTS];   // Same as "lowerBounds" but for error correction projections
        decimal upperBoundsError[NB_MAX_CONSTRAINTS];   // Same as "upperBounds" but for error correction projections
        Matrix3x3 Minv_sp_inertia[NB_MAX_BODIES];       // 3x3 world inertia tensor matrix I for each body (from the Minv_sp matrix)
        decimal Minv_sp_mass_diag[NB_MAX_BODIES];       // Array that contains for each body the inverse of its mass
                                                        // This is an array of size nbBodies that contains in each cell a 6x6 matrix
        decimal V1[6*NB_MAX_BODIES];                    // Array that contains for each body the 6x1 vector that contains linear and angular velocities
                                                        // Each cell contains a 6x1 vector with linear and angular velocities
        decimal Vconstraint[6*NB_MAX_BODIES];           // Same kind of vector as V1 but contains the final constraint velocities
        decimal VconstraintError[6*NB_MAX_BODIES];      // Same kind of vector as V1 but contains the final constraint velocities
        decimal Fext[6*NB_MAX_BODIES];                  // Array that contains for each body the 6x1 vector that contains external forces and torques
                                                        // Each cell contains a 6x1 vector with external force and torque.
        void initialize();                              // Initialize the constraint solver before each solving
        void fillInMatrices(decimal dt);                // Fill in all the matrices needed to solve the LCP problem
        void computeVectorB(decimal dt);                // Compute the vector b
        void computeVectorBError(decimal dt);           // Compute the vector b for error correction projection
        void computeMatrixB_sp();                       // Compute the matrix B_sp
        void computeMatrixB_spErrorCorrection();        // Compute the matrix B_spError for error correction projection
        void computeVectorVconstraint(decimal dt);      // Compute the vector V2
        void computeVectorVconstraintError(decimal dt); // Same as computeVectorVconstraint() but for error correction projection
        void cacheLambda();                             // Cache the lambda values in order to reuse them in the next step to initialize the lambda vector
        void computeVectorA();                          // Compute the vector a used in the solve() method
        void computeVectorAError();                     // Same as computeVectorA() but for error correction projection
        void solveLCP();                                // Solve a LCP problem using Projected-Gauss-Seidel algorithm
        void solveLCPErrorCorrection();                 // Solve the LCP problem for error correction projection
    
   public:
        ConstraintSolver(PhysicsWorld* world);                         // Constructor
        virtual ~ConstraintSolver();                                   // Destructor
        void solve(decimal dt);                                         // Solve the current LCP problem
        bool isConstrainedBody(Body* body) const;                      // Return true if the body is in at least one constraint
        Vector3 getConstrainedLinearVelocityOfBody(Body* body);        // Return the constrained linear velocity of a body after solving the LCP problem
        Vector3 getConstrainedAngularVelocityOfBody(Body* body);       // Return the constrained angular velocity of a body after solving the LCP problem
        Vector3 getErrorConstrainedLinearVelocityOfBody(Body* body);   // Return the constrained linear velocity of a body after solving the LCP problem for error correction
        Vector3 getErrorConstrainedAngularVelocityOfBody(Body* body);  // Return the constrained angular velocity of a body after solving the LCP problem for error correction
        void cleanup();                                                 // Cleanup of the constraint solver
        void setPenetrationFactor(decimal penetrationFactor);           // Set the penetration factor 
        void setNbLCPIterations(uint nbIterations);                     // Set the number of iterations of the LCP solver
        void setIsErrorCorrectionActive(bool isErrorCorrectionActive);  // Set the isErrorCorrectionActive value
};

// Return true if the body is in at least one constraint
inline bool ConstraintSolver::isConstrainedBody(Body* body) const {
    if(constraintBodies.find(body) != constraintBodies.end()) {
        return true;
    }
    return false;
}

// Return the constrained linear velocity of a body after solving the LCP problem
inline Vector3 ConstraintSolver::getConstrainedLinearVelocityOfBody(Body* body) {
    assert(isConstrainedBody(body));
    uint indexBodyArray = 6 * bodyNumberMapping[body];
    return Vector3(Vconstraint[indexBodyArray], Vconstraint[indexBodyArray + 1], Vconstraint[indexBodyArray + 2]);
}

// Return the constrained angular velocity of a body after solving the LCP problem
inline Vector3 ConstraintSolver::getConstrainedAngularVelocityOfBody(Body* body) {
    assert(isConstrainedBody(body));
    uint indexBodyArray = 6 * bodyNumberMapping[body];
    return Vector3(Vconstraint[indexBodyArray + 3], Vconstraint[indexBodyArray + 4], Vconstraint[indexBodyArray + 5]);
}

// Return the constrained linear velocity of a body after solving the LCP problem for error correction
inline Vector3 ConstraintSolver::getErrorConstrainedLinearVelocityOfBody(Body* body) {
    //assert(isConstrainedBody(body));
    uint indexBodyArray = 6 * bodyNumberMapping[body];
    return Vector3(VconstraintError[indexBodyArray], VconstraintError[indexBodyArray + 1], VconstraintError[indexBodyArray + 2]);
}  

// Return the constrained angular velocity of a body after solving the LCP problem for error correction
inline Vector3 ConstraintSolver::getErrorConstrainedAngularVelocityOfBody(Body* body) {
    //assert(isConstrainedBody(body));
    uint indexBodyArray = 6 * bodyNumberMapping[body];
    return Vector3(VconstraintError[indexBodyArray + 3], VconstraintError[indexBodyArray + 4], VconstraintError[indexBodyArray + 5]);
} 

// Cleanup of the constraint solver
inline void ConstraintSolver::cleanup() {
    bodyNumberMapping.clear();
    constraintBodies.clear();
    activeConstraints.clear();
}

// Set the number of iterations of the LCP solver
inline void ConstraintSolver::setNbLCPIterations(uint nbIterations) {
    nbIterationsLCP = nbIterations;
}         


// Set the isErrorCorrectionActive value
inline void ConstraintSolver::setIsErrorCorrectionActive(bool isErrorCorrectionActive) {
    this->isErrorCorrectionActive = isErrorCorrectionActive;
}


// Solve the current LCP problem
inline void ConstraintSolver::solve(decimal dt) {
    
    // Initialize the solver
    initialize();

    // Fill-in all the matrices needed to solve the LCP problem
    fillInMatrices(dt);

    // Compute the vector b
    computeVectorB(dt);
    if (isErrorCorrectionActive) {
        computeVectorBError(dt);
    }

    // Compute the matrix B
    computeMatrixB_sp();
    if (isErrorCorrectionActive) {
        computeMatrixB_spErrorCorrection();
    }

    // Solve the LCP problem (computation of lambda)
    solveLCP();
    if (isErrorCorrectionActive) {
        solveLCPErrorCorrection();
    }

    // Cache the lambda values in order to use them in the next step
    cacheLambda();
    
    // Compute the vector Vconstraint
    computeVectorVconstraint(dt);
    if (isErrorCorrectionActive) {
        computeVectorVconstraintError(dt);
    }
}

} // End of ReactPhysics3D namespace

#endif