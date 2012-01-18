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

// Libraries
#include "ConstraintSolver.h"
#include "../body/RigidBody.h"

using namespace reactphysics3d;
using namespace std;


// Constructor
ConstraintSolver::ConstraintSolver(PhysicsWorld* world)
                 :physicsWorld(world), nbConstraints(0), nbIterationsLCP(DEFAULT_LCP_ITERATIONS),
                  nbIterationsLCPErrorCorrection(DEFAULT_LCP_ITERATIONS_ERROR_CORRECTION),
                  isErrorCorrectionActive(false) {

}

// Destructor
ConstraintSolver::~ConstraintSolver() {

}

 // Initialize the constraint solver before each solving
void ConstraintSolver::initialize() {
    Constraint* constraint;
    
    nbConstraints = 0;
    nbConstraintsError = 0;

    // For each constraint
    vector<Constraint*>::iterator it;
    for (it = physicsWorld->getConstraintsBeginIterator(); it != physicsWorld->getConstraintsEndIterator(); ++it) {
        constraint = *it;

        // If the constraint is active
        if (constraint->isActive()) {
            activeConstraints.push_back(constraint);

            // Add the two bodies of the constraint in the constraintBodies list
            constraintBodies.insert(constraint->getBody1());
            constraintBodies.insert(constraint->getBody2());

            // Fill in the body number maping
            bodyNumberMapping.insert(pair<Body*, unsigned int>(constraint->getBody1(), bodyNumberMapping.size()));
            bodyNumberMapping.insert(pair<Body*, unsigned int>(constraint->getBody2(), bodyNumberMapping.size()));

            // Update the size of the jacobian matrix
            nbConstraints += constraint->getNbConstraints();
            
            // Update the size of the jacobian matrix for error correction projection
            if (constraint->getType() == CONTACT) {
                nbConstraintsError++;
            }
        }
    }

    // Compute the number of bodies that are part of some active constraint
    nbBodies = bodyNumberMapping.size();

    assert(nbConstraints > 0 && nbConstraints <= NB_MAX_CONSTRAINTS);
    assert(nbConstraintsError > 0 && nbConstraintsError <= NB_MAX_CONSTRAINTS);
    assert(nbBodies > 0 && nbBodies <= NB_MAX_BODIES);
}

// Fill in all the matrices needed to solve the LCP problem
// Notice that all the active constraints should have been evaluated first
void ConstraintSolver::fillInMatrices(decimal dt) {
    Constraint* constraint;
    decimal oneOverDt = 1.0 / dt;

    // For each active constraint
    int noConstraint = 0;
    int noConstraintError = 0;
    
    for (int c=0; c<activeConstraints.size(); c++) {
        
        constraint = activeConstraints.at(c);

        // Fill in the J_sp matrix
        constraint->computeJacobian(noConstraint, J_sp);

        // Fill in the body mapping matrix
        for(int i=0; i<constraint->getNbConstraints(); i++) {
            bodyMapping[noConstraint+i][0] = constraint->getBody1();
            bodyMapping[noConstraint+i][1] = constraint->getBody2();
        }

        // Fill in the limit vectors for the constraint
        constraint->computeLowerBound(noConstraint, lowerBounds);
        constraint->computeUpperBound(noConstraint, upperBounds);

        // Fill in the error vector
        constraint->computeErrorValue(noConstraint, errorValues);

        // Get the cached lambda values of the constraint
        for (int i=0; i<constraint->getNbConstraints(); i++) {
            lambdaInit[noConstraint + i] = constraint->getCachedLambda(i);
        }

        // If the constraint is a contact
        if (constraint->getType() == CONTACT) {
            Contact* contact = dynamic_cast<Contact*>(constraint);
            decimal penetrationDepth = contact->getPenetrationDepth();

            // If error correction with projection is active
            if (isErrorCorrectionActive) {
                
                // Fill in the error correction projection parameters
                lowerBoundsError[noConstraintError] = lowerBounds[noConstraint];
                upperBoundsError[noConstraintError] = upperBounds[noConstraint];
                for (int i=0; i<12; i++) {
                  J_spError[noConstraintError][i] = J_sp[noConstraint][i];
                }
                bodyMappingError[noConstraintError][0] = constraint->getBody1();
                bodyMappingError[noConstraintError][1] = constraint->getBody2();
                penetrationDepths[noConstraintError] = contact->getPenetrationDepth();
                
                // If the penetration depth is small
                if (penetrationDepth < PENETRATION_DEPTH_THRESHOLD_ERROR_CORRECTION) {
                    // Use the Baumgarte error correction term for contacts instead of
                    // first order world projection
                    errorValues[noConstraint] += 0.1 * oneOverDt * contact->getPenetrationDepth();
                }

                noConstraintError++;
            }
            else {  // If error correction with projection is not active
                // Add the Baumgarte error correction term for contacts
                errorValues[noConstraint] += 0.1 * oneOverDt * contact->getPenetrationDepth();
            }
        }
        
        noConstraint += constraint->getNbConstraints();
    }

    // For each current body that is implied in some constraint
    RigidBody* rigidBody;
    Body* body;
    uint b=0;
    for (set<Body*>::iterator it = constraintBodies.begin(); it != constraintBodies.end(); ++it, b++) {
        body = *it;
        uint bodyNumber = bodyNumberMapping[body];
        
        // TODO : Use polymorphism and remove this downcasting
        rigidBody = dynamic_cast<RigidBody*>(body);
        assert(rigidBody);

        // Compute the vector V1 with initial velocities values
        Vector3 linearVelocity = rigidBody->getLinearVelocity();
        Vector3 angularVelocity = rigidBody->getAngularVelocity();
        int bodyIndexArray = 6 * bodyNumber;
        V1[bodyIndexArray] = linearVelocity[0];
        V1[bodyIndexArray + 1] = linearVelocity[1];
        V1[bodyIndexArray + 2] = linearVelocity[2];
        V1[bodyIndexArray + 3] = angularVelocity[0];
        V1[bodyIndexArray + 4] = angularVelocity[1];
        V1[bodyIndexArray + 5] = angularVelocity[2];

        // Compute the vector Vconstraint with final constraint velocities
        Vconstraint[bodyIndexArray] = 0.0;
        Vconstraint[bodyIndexArray + 1] = 0.0;
        Vconstraint[bodyIndexArray + 2] = 0.0;
        Vconstraint[bodyIndexArray + 3] = 0.0;
        Vconstraint[bodyIndexArray + 4] = 0.0;
        Vconstraint[bodyIndexArray + 5] = 0.0;
        
        // Compute the vector Vconstraint with final constraint velocities
        if (isErrorCorrectionActive) {
            VconstraintError[bodyIndexArray] = 0.0;
            VconstraintError[bodyIndexArray + 1] = 0.0;
            VconstraintError[bodyIndexArray + 2] = 0.0;
            VconstraintError[bodyIndexArray + 3] = 0.0;
            VconstraintError[bodyIndexArray + 4] = 0.0;
            VconstraintError[bodyIndexArray + 5] = 0.0;
        }
        
        // Compute the vector with forces and torques values
        Vector3 externalForce = rigidBody->getExternalForce();
        Vector3 externalTorque = rigidBody->getExternalTorque();
        Fext[bodyIndexArray] = externalForce[0];
        Fext[bodyIndexArray + 1] = externalForce[1];
        Fext[bodyIndexArray + 2] = externalForce[2];
        Fext[bodyIndexArray + 3] = externalTorque[0];
        Fext[bodyIndexArray + 4] = externalTorque[1];
        Fext[bodyIndexArray + 5] = externalTorque[2];
		
        // Initialize the mass and inertia tensor matrices
        Minv_sp_inertia[bodyNumber].setAllValues(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        Minv_sp_mass_diag[bodyNumber] = 0.0;

        // If the motion of the rigid body is enabled
        if (rigidBody->getIsMotionEnabled()) {
            Minv_sp_inertia[bodyNumber] = rigidBody->getInertiaTensorInverseWorld();
            Minv_sp_mass_diag[bodyNumber] = rigidBody->getMassInverse();
        }
    }
}

// Compute the vector b
void ConstraintSolver::computeVectorB(decimal dt) {
    uint indexBody1, indexBody2;
    decimal oneOverDT = 1.0 / dt;

    for (uint c = 0; c<nbConstraints; c++) {
        
        // b = errorValues * oneOverDT;
        b[c] = errorValues[c] * oneOverDT;
		
        // Substract 1.0/dt*J*V to the vector b
        indexBody1 = bodyNumberMapping[bodyMapping[c][0]];
        indexBody2 = bodyNumberMapping[bodyMapping[c][1]];
        decimal multiplication = 0.0;
        int body1ArrayIndex = 6 * indexBody1;
        int body2ArrayIndex = 6 * indexBody2;
        for (uint i=0; i<6; i++) {
                multiplication += J_sp[c][i] * V1[body1ArrayIndex + i];
                multiplication += J_sp[c][6 + i] * V1[body2ArrayIndex + i];
        }
        b[c] -= multiplication * oneOverDT ;

        // Substract J*M^-1*F_ext to the vector b
        decimal value1 = 0.0;
        decimal value2 = 0.0;
        decimal sum1, sum2;
        value1 += J_sp[c][0] * Minv_sp_mass_diag[indexBody1] * Fext[body1ArrayIndex] +
                          J_sp[c][1] * Minv_sp_mass_diag[indexBody1] * Fext[body1ArrayIndex + 1] +
                          J_sp[c][2] * Minv_sp_mass_diag[indexBody1] * Fext[body1ArrayIndex + 2];
        value2 += J_sp[c][6] * Minv_sp_mass_diag[indexBody2] * Fext[body2ArrayIndex] +
                           J_sp[c][7] * Minv_sp_mass_diag[indexBody2] * Fext[body2ArrayIndex + 1] +
                           J_sp[c][8] * Minv_sp_mass_diag[indexBody2] * Fext[body2ArrayIndex + 2];
        for (uint i=0; i<3; i++) {
                sum1 = 0.0;
                sum2 = 0.0;
                for (uint j=0; j<3; j++) {
                        sum1 += J_sp[c][3 + j] * Minv_sp_inertia[indexBody1].getValue(j, i);
                        sum2 += J_sp[c][9 + j] * Minv_sp_inertia[indexBody2].getValue(j, i);
                }
                value1 += sum1 * Fext[body1ArrayIndex + 3 + i];
                value2 += sum2 * Fext[body2ArrayIndex + 3 + i];
        }

        b[c] -= value1 + value2;	
    }
}

// Compute the vector b for error correction projection
void ConstraintSolver::computeVectorBError(decimal dt) {
    decimal oneOverDT = 1.0 / dt;

    for (uint c = 0; c<nbConstraintsError; c++) {
        
        // b = errorValues * oneOverDT;
        if (penetrationDepths[c] >= PENETRATION_DEPTH_THRESHOLD_ERROR_CORRECTION) {
            bError[c] = penetrationDepths[c] * oneOverDT;
        }
        else {
            bError[c] = 0.0;
        }
    }
}

// Compute the matrix B_sp
void ConstraintSolver::computeMatrixB_sp() {
	uint indexConstraintArray, indexBody1, indexBody2;
	
    // For each constraint
    for (uint c = 0; c<nbConstraints; c++) {

        indexConstraintArray = 6 * c;
        indexBody1 = bodyNumberMapping[bodyMapping[c][0]];
        indexBody2 = bodyNumberMapping[bodyMapping[c][1]];
        B_sp[0][indexConstraintArray] = Minv_sp_mass_diag[indexBody1] * J_sp[c][0];
        B_sp[0][indexConstraintArray + 1] = Minv_sp_mass_diag[indexBody1] * J_sp[c][1];
        B_sp[0][indexConstraintArray + 2] = Minv_sp_mass_diag[indexBody1] * J_sp[c][2];
        B_sp[1][indexConstraintArray] = Minv_sp_mass_diag[indexBody2] * J_sp[c][6];
        B_sp[1][indexConstraintArray + 1] = Minv_sp_mass_diag[indexBody2] * J_sp[c][7];
        B_sp[1][indexConstraintArray + 2] = Minv_sp_mass_diag[indexBody2] * J_sp[c][8];

        for (uint i=0; i<3; i++) {
            B_sp[0][indexConstraintArray + 3 + i] = 0.0;
            B_sp[1][indexConstraintArray + 3 + i] = 0.0;
            for (uint j=0; j<3; j++) {
                    B_sp[0][indexConstraintArray + 3 + i] += Minv_sp_inertia[indexBody1].getValue(i, j) * J_sp[c][3 + j];
                    B_sp[1][indexConstraintArray + 3 + i] += Minv_sp_inertia[indexBody2].getValue(i, j) * J_sp[c][9 + j];
            }
        }
    }
}

// Same as B_sp matrix but for error correction projection and
// therefore, only for contact constraints
void ConstraintSolver::computeMatrixB_spErrorCorrection() {
    uint indexConstraintArray, indexBody1, indexBody2;
	
    // For each contact constraint
    for (uint c = 0; c<nbConstraintsError; c++) {

        indexConstraintArray = 6 * c;
        indexBody1 = bodyNumberMapping[bodyMappingError[c][0]];
        indexBody2 = bodyNumberMapping[bodyMappingError[c][1]];
        B_spError[0][indexConstraintArray] = Minv_sp_mass_diag[indexBody1] * J_spError[c][0];
        B_spError[0][indexConstraintArray + 1] = Minv_sp_mass_diag[indexBody1] * J_spError[c][1];
        B_spError[0][indexConstraintArray + 2] = Minv_sp_mass_diag[indexBody1] * J_spError[c][2];
        B_spError[1][indexConstraintArray] = Minv_sp_mass_diag[indexBody2] * J_spError[c][6];
        B_spError[1][indexConstraintArray + 1] = Minv_sp_mass_diag[indexBody2] * J_spError[c][7];
        B_spError[1][indexConstraintArray + 2] = Minv_sp_mass_diag[indexBody2] * J_spError[c][8];

        for (uint i=0; i<3; i++) {
            B_spError[0][indexConstraintArray + 3 + i] = 0.0;
            B_spError[1][indexConstraintArray + 3 + i] = 0.0;

            for (uint j=0; j<3; j++) {
                B_spError[0][indexConstraintArray + 3 + i] += Minv_sp_inertia[indexBody1].getValue(i, j) * J_spError[c][3 + j];
                B_spError[1][indexConstraintArray + 3 + i] += Minv_sp_inertia[indexBody2].getValue(i, j) * J_spError[c][9 + j];
            }            
        }
    }
}

// Compute the vector V_constraint (which corresponds to the constraint part of
// the final V2 vector) according to the formula
// V_constraint = dt * (M^-1 * J^T * lambda)
// Note that we use the vector V to store both V1 and V_constraint.
// Note that M^-1 * J^T = B.
// This method is called after that the LCP solver has computed lambda
void ConstraintSolver::computeVectorVconstraint(decimal dt) {
    uint indexBody1Array, indexBody2Array, indexConstraintArray;
	uint j;
	
    // Compute dt * (M^-1 * J^T * lambda
    for (uint i=0; i<nbConstraints; i++) {
        indexBody1Array = 6 * bodyNumberMapping[bodyMapping[i][0]];
        indexBody2Array = 6 * bodyNumberMapping[bodyMapping[i][1]];
		indexConstraintArray = 6 * i;
		for (j=0; j<6; j++) {
			Vconstraint[indexBody1Array + j] += B_sp[0][indexConstraintArray + j] * lambda[i] * dt;
			Vconstraint[indexBody2Array + j] += B_sp[1][indexConstraintArray + j] * lambda[i] * dt;
		}
    }
}

// Same as computeVectorVconstraint() but for error correction projection
void ConstraintSolver::computeVectorVconstraintError(decimal dt) {
    uint indexBody1Array, indexBody2Array, indexConstraintArray;
	uint j;
	
    // Compute M^-1 * J^T * lambda
    for (uint i=0; i<nbConstraintsError; i++) {
        indexBody1Array = 6 * bodyNumberMapping[bodyMappingError[i][0]];
        indexBody2Array = 6 * bodyNumberMapping[bodyMappingError[i][1]];
		indexConstraintArray = 6 * i;
		for (j=0; j<6; j++) {
			VconstraintError[indexBody1Array + j] += B_spError[0][indexConstraintArray + j] * lambdaError[i];
			VconstraintError[indexBody2Array + j] += B_spError[1][indexConstraintArray + j] * lambdaError[i];
		}
    }
}


// Solve a LCP problem using the Projected-Gauss-Seidel algorithm
// This method outputs the result in the lambda vector
void ConstraintSolver::solveLCP() {

    for (uint i=0; i<nbConstraints; i++) {
            lambda[i] = lambdaInit[i];
    }

    uint indexBody1Array, indexBody2Array;
    decimal deltaLambda;
    decimal lambdaTemp;
    uint i, iter;

    // Compute the vector a
    computeVectorA();


    // For each constraint
    for (i=0; i<nbConstraints; i++) {
        uint indexConstraintArray = 6 * i;
        d[i] = 0.0;
        for (uint j=0; j<6; j++) {
                d[i] += J_sp[i][j] * B_sp[0][indexConstraintArray + j] + J_sp[i][6 + j] * B_sp[1][indexConstraintArray + j];
        }
    }

    for(iter=0; iter<nbIterationsLCP; iter++) {
        for (i=0; i<nbConstraints; i++) {
            indexBody1Array = 6 * bodyNumberMapping[bodyMapping[i][0]];
            indexBody2Array = 6 * bodyNumberMapping[bodyMapping[i][1]];
            uint indexConstraintArray = 6 * i;
            deltaLambda = b[i];
            for (uint j=0; j<6; j++) {
                    deltaLambda -= (J_sp[i][j] * a[indexBody1Array + j] + J_sp[i][6 + j] * a[indexBody2Array + j]);
            }
            deltaLambda /= d[i];
            lambdaTemp = lambda[i];
            lambda[i] = std::max(lowerBounds[i], std::min(lambda[i] + deltaLambda, upperBounds[i]));
            deltaLambda = lambda[i] - lambdaTemp;
            for (uint j=0; j<6; j++) {
                    a[indexBody1Array + j] += B_sp[0][indexConstraintArray + j] * deltaLambda;
                    a[indexBody2Array + j] += B_sp[1][indexConstraintArray + j] * deltaLambda;
            }
        }
    }
}

// Solve a LCP problem for error correction projection
// using the Projected-Gauss-Seidel algorithm
// This method outputs the result in the lambda vector
void ConstraintSolver::solveLCPErrorCorrection() {

    for (uint i=0; i<nbConstraintsError; i++) {
        lambdaError[i] = 0;
    }

    uint indexBody1Array, indexBody2Array;
    decimal deltaLambda;
    decimal lambdaTemp;
    uint i, iter;

    // Compute the vector a
    computeVectorAError();


    // For each constraint
    for (i=0; i<nbConstraintsError; i++) {
        uint indexConstraintArray = 6 * i;
        d[i] = 0.0;
        for (uint j=0; j<6; j++) {
           d[i] += J_spError[i][j] * B_spError[0][indexConstraintArray + j] + J_spError[i][6 + j] * B_spError[1][indexConstraintArray + j];
        }
    }

    for(iter=0; iter<nbIterationsLCPErrorCorrection; iter++) {
        for (i=0; i<nbConstraintsError; i++) {
            indexBody1Array = 6 * bodyNumberMapping[bodyMappingError[i][0]];
            indexBody2Array = 6 * bodyNumberMapping[bodyMappingError[i][1]];
            uint indexConstraintArray = 6 * i;
            deltaLambda = bError[i];
            for (uint j=0; j<6; j++) {
                deltaLambda -= (J_spError[i][j] * aError[indexBody1Array + j] + J_spError[i][6 + j] * aError[indexBody2Array + j]);
            }
            deltaLambda /= d[i];
            lambdaTemp = lambdaError[i];
            lambdaError[i] = std::max(lowerBoundsError[i], std::min(lambdaError[i] + deltaLambda, upperBoundsError[i]));
            deltaLambda = lambdaError[i] - lambdaTemp;
            for (uint j=0; j<6; j++) {
                    aError[indexBody1Array + j] += B_spError[0][indexConstraintArray + j] * deltaLambda;
                    aError[indexBody2Array + j] += B_spError[1][indexConstraintArray + j] * deltaLambda;
            }
        }
    }
}

// Compute the vector a used in the solve() method
// Note that a = B * lambda
void ConstraintSolver::computeVectorA() {
    uint i;
    uint indexBody1Array, indexBody2Array;
    
    // Init the vector a with zero values
    for (i=0; i<6*nbBodies; i++) {
       a[i] = 0.0;
    }

    for(i=0; i<nbConstraints; i++) {
        indexBody1Array = 6 * bodyNumberMapping[bodyMapping[i][0]];
        indexBody2Array = 6 * bodyNumberMapping[bodyMapping[i][1]];
		uint indexConstraintArray = 6 * i;
		for (uint j=0; j<6; j++) {
			a[indexBody1Array + j] += B_sp[0][indexConstraintArray + j] * lambda[i];
			a[indexBody2Array + j] += B_sp[1][indexConstraintArray + j] * lambda[i];
		}
    }
}

// Same as computeVectorA() but for error correction projection
void ConstraintSolver::computeVectorAError() {
    uint i;
    uint indexBody1Array, indexBody2Array;
    
    // Init the vector a with zero values
    for (i=0; i<6*nbBodies; i++) {
       aError[i] = 0.0;
    }

    for(i=0; i<nbConstraintsError; i++) {
        indexBody1Array = 6 * bodyNumberMapping[bodyMappingError[i][0]];
        indexBody2Array = 6 * bodyNumberMapping[bodyMappingError[i][1]];
		uint indexConstraintArray = 6 * i;
		for (uint j=0; j<6; j++) {
			aError[indexBody1Array + j] += B_spError[0][indexConstraintArray + j] * lambdaError[i];
			aError[indexBody2Array + j] += B_spError[1][indexConstraintArray + j] * lambdaError[i];
		}
    }
}


// Cache the lambda values in order to reuse them in the next step
// to initialize the lambda vector
void ConstraintSolver::cacheLambda() {
    
    // For each active constraint
    int noConstraint = 0;
    for (int c=0; c<activeConstraints.size(); c++) {

        // For each constraint of the contact
        for (int i=0; i<activeConstraints[c]->getNbConstraints(); i++) {
            // Get the lambda value that have just been computed
            activeConstraints[c]->setCachedLambda(i, lambda[noConstraint + i]);
        }

        noConstraint += activeConstraints[c]->getNbConstraints();
    }
}
