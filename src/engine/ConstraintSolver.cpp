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

// Libraries
#include "ConstraintSolver.h"
#include "../mathematics/lcp/LCPProjectedGaussSeidel.h"
#include "../body/RigidBody.h"

using namespace reactphysics3d;
using namespace std;

// Constructor
ConstraintSolver::ConstraintSolver(PhysicsWorld* world)
                 :physicsWorld(world), bodyMapping(0), nbConstraints(0), constraintsCapacity(0),
                  bodiesCapacity(0), avConstraintsCapacity(0), avBodiesCapacity(0), avBodiesNumber(0),
                  avConstraintsNumber(0), avBodiesCounter(0), avConstraintsCounter(0),
                  lcpSolver(new LCPProjectedGaussSeidel(MAX_LCP_ITERATIONS)) {

}

// Destructor
ConstraintSolver::~ConstraintSolver() {

}

 // Initialize the constraint solver before each solving
void ConstraintSolver::initialize() {
    Constraint* constraint;

    nbConstraints = 0;

    // For each constraint
    vector<Constraint*>::iterator it;
    for (it = physicsWorld->getConstraintsBeginIterator(); it != physicsWorld->getConstraintsEndIterator(); it++) {
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
        }
    }

    // Compute the number of bodies that are part of some active constraint
    nbBodies = bodyNumberMapping.size();

    assert(nbConstraints > 0);
    assert(nbBodies > 0);
    
    // Update the average bodies and constraints capacities
    if (avBodiesCounter > AV_COUNTER_LIMIT) {
        avBodiesCounter = 0;
        avBodiesNumber = 0;
    }
    if (avConstraintsCounter > AV_COUNTER_LIMIT) {
        avConstraintsCounter = 0;
        avConstraintsNumber = 0;
    }
    avBodiesCounter++;
    avConstraintsCounter++;
    avBodiesNumber += nbBodies;
    avConstraintsNumber += nbConstraints;
    avBodiesCapacity += (avBodiesNumber / avBodiesCounter);
    avConstraintsCapacity += (avConstraintsNumber / avConstraintsCounter);

    // Allocate the memory needed for the constraint solver
    allocate();
}

// Allocate all the memory needed to solve the LCP problem
// The goal of this method is to avoid to free and allocate the memory
// each time the constraint solver is called but only if the we effectively
// need more memory. Therefore if for instance the number of constraints to
// be solved is smaller than the constraints capacity, we don't free and reallocate
// memory because we don't need to. The problem now is that the constraints capacity
// can grow indefinitely. Therefore we use a way to free and reallocate the memory
// if the average number of constraints currently solved is far less than the current
// constraints capacity
void ConstraintSolver::allocate() {
    // If we need to allocate more memory for the bodies
    if (nbBodies > bodiesCapacity || avBodiesCapacity < AV_PERCENT_TO_FREE * bodiesCapacity) {
        freeMemory(true);
        bodiesCapacity = nbBodies;
        
        Minv_sp = new Matrix6x6[nbBodies];
        V1 = new Vector[nbBodies];
        Vconstraint = new Vector[nbBodies];
        Fext = new Vector[nbBodies];

        avBodiesNumber = 0;
        avBodiesCounter = 0;
    }

    // If we need to allocate more memory for the constraints
    if (nbConstraints > constraintsCapacity || constraintsCapacity < AV_PERCENT_TO_FREE * constraintsCapacity) {
        freeMemory(false);
        constraintsCapacity = nbConstraints;

        bodyMapping = new Body**[nbConstraints];
        J_sp = new Matrix1x6*[nbConstraints];
        B_sp = new Vector6D*[2];
        B_sp[0] = new Vector6D[nbConstraints];
        B_sp[1] = new Vector6D[nbConstraints];
        for (int i=0; i<nbConstraints; i++) {
            bodyMapping[i] = new Body*[2];
            J_sp[i] = new Matrix1x6[2];
        }

        errorValues.changeSize(nbConstraints);
        b.changeSize(nbConstraints);
        lambda.changeSize(nbConstraints);
        lambdaInit.changeSize(nbConstraints);
        lowerBounds.changeSize(nbConstraints);
        upperBounds.changeSize(nbConstraints);

        avConstraintsNumber = 0;
        avConstraintsCounter = 0;
    }
}

// Free the memory that was allocated in the allocate() method
// If the argument is true the method will free the memory
// associated to the bodies. In the other case, it will free
// the memory associated with the constraints
void ConstraintSolver::freeMemory(bool freeBodiesMemory) {

    // If we need to free the bodies memory
    if (freeBodiesMemory && bodiesCapacity > 0) {
        delete[] Minv_sp;
        delete[] V1;
        delete[] Vconstraint;
        delete[] Fext;
    }
    else if (constraintsCapacity > 0) { // If we need to free the constraints memory
        // Free the bodyMaping array
        for (uint i=0; i<constraintsCapacity; i++) {
            delete[] bodyMapping[i];
            delete[] J_sp[i];
        }

        delete[] bodyMapping;
        delete[] J_sp;
        delete[] B_sp[0];
        delete[] B_sp[1];
        delete[] B_sp;
    }
}

// Fill in all the matrices needed to solve the LCP problem
// Notice that all the active constraints should have been evaluated first
void ConstraintSolver::fillInMatrices() {
    Constraint* constraint;
    Contact* contact;
    ContactCachingInfo* contactInfo;

    // For each active constraint
    int noConstraint = 0;
    //uint nbAuxConstraints = 0;
    
    for (int c=0; c<activeConstraints.size(); c++) {
        
        constraint = activeConstraints.at(c);

        // Fill in the J_sp matrix
        constraint->computeJacobian(noConstraint, J_sp);
        //constraint->computeJacobian(noConstraint, J_sp);

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

        // Set the init lambda values
        contact = dynamic_cast<Contact*>(constraint);
        contactInfo = 0;
        if (contact) {
            // Get the lambda init value from the cache if exists
            contactInfo = contactCache.getContactCachingInfo(contact);
        }
        for (int i=0; i<constraint->getNbConstraints(); i++) {
            if (contactInfo) { // If the last lambda init value is in the cache
                lambdaInit.setValue(noConstraint + i, contactInfo->lambdas[i]);
            }
            else {  // The las lambda init value was not in the cache
                lambdaInit.setValue(noConstraint + i, 0.0);
            }
        }

        noConstraint += constraint->getNbConstraints();
    }

    // For each current body that is implied in some constraint
    RigidBody* rigidBody;
    Body* body;
    Vector v(6);
    Vector f(6);
    //Matrix identity = Matrix::identity(3);
    //Matrix mInv(6,6);
    uint b=0;
    for (set<Body*>::iterator it = constraintBodies.begin(); it != constraintBodies.end(); it++, b++) {
        body = *it;
        uint bodyNumber = bodyNumberMapping.at(body);
        
        // TODO : Use polymorphism and remove this downcasting
        rigidBody = dynamic_cast<RigidBody*>(body);
        assert(rigidBody != 0);
        
        // Compute the vector V1 with initial velocities values
        v.fillInSubVector(0, rigidBody->getLinearVelocity());
        v.fillInSubVector(3, rigidBody->getAngularVelocity());
        V1[bodyNumber].changeSize(6);
        V1[bodyNumber] = v;

        // Compute the vector Vconstraint with final constraint velocities
        Vconstraint[bodyNumber].changeSize(6);
        Vconstraint[bodyNumber].initWithValue(0.0);
        
        // Compute the vector with forces and torques values
        f.fillInSubVector(0, rigidBody->getExternalForce());
        f.fillInSubVector(3, rigidBody->getExternalTorque());
        Fext[bodyNumber].changeSize(6);
        Fext[bodyNumber] = f;

        // Compute the inverse sparse mass matrix
        Minv_sp[bodyNumber].initWithValue(0.0);
        const Matrix3x3& tensorInv = rigidBody->getInertiaTensorInverseWorld();
        if (rigidBody->getIsMotionEnabled()) {
            Minv_sp[bodyNumber].setValue(0, 0, rigidBody->getMassInverse());
            Minv_sp[bodyNumber].setValue(1, 1, rigidBody->getMassInverse());
            Minv_sp[bodyNumber].setValue(2, 2, rigidBody->getMassInverse());
            Minv_sp[bodyNumber].setValue(3, 3, tensorInv.getValue(0, 0));
            Minv_sp[bodyNumber].setValue(3, 4, tensorInv.getValue(0, 1));
            Minv_sp[bodyNumber].setValue(3, 5, tensorInv.getValue(0, 2));
            Minv_sp[bodyNumber].setValue(4, 3, tensorInv.getValue(1, 0));
            Minv_sp[bodyNumber].setValue(4, 4, tensorInv.getValue(1, 1));
            Minv_sp[bodyNumber].setValue(4, 5, tensorInv.getValue(1, 2));
            Minv_sp[bodyNumber].setValue(5, 3, tensorInv.getValue(2, 0));
            Minv_sp[bodyNumber].setValue(5, 4, tensorInv.getValue(2, 1));
            Minv_sp[bodyNumber].setValue(5, 5, tensorInv.getValue(2, 2));
            //mInv.fillInSubMatrix(0, 0, rigidBody->getMassInverse() * identity);
            //mInv.fillInSubMatrix(3, 3, rigidBody->getInertiaTensorInverseWorld());
        }
        //Minv_sp[bodyNumber].changeSize(6, 6);
        //Minv_sp[bodyNumber] = mInv;
    }
}

// Compute the vector b
void ConstraintSolver::computeVectorB(double dt) {
    uint indexBody1, indexBody2;
    double oneOverDT = 1.0/dt;
    
    b = errorValues * oneOverDT;

    for (uint c = 0; c<nbConstraints; c++) {
        // Substract 1.0/dt*J*V to the vector b
        indexBody1 = bodyNumberMapping[bodyMapping[c][0]];
        indexBody2 = bodyNumberMapping[bodyMapping[c][1]];
        b.setValue(c, b.getValue(c) - (Matrix(J_sp[c][0]) * V1[indexBody1]).getValue(0,0) * oneOverDT); // TODO : Remove conversion here
        b.setValue(c, b.getValue(c) - (Matrix(J_sp[c][1]) * V1[indexBody2]).getValue(0,0) * oneOverDT);

        // Substract J*M^-1*F_ext to the vector b
        b.setValue(c, b.getValue(c) - ((Matrix(J_sp[c][0]) * Matrix(Minv_sp[indexBody1])) * Fext[indexBody1]
                 + (Matrix(J_sp[c][1]) * Matrix(Minv_sp[indexBody2]))*Fext[indexBody2]).getValue(0,0));         // TODO : Delete conversion here
    }
}

// Compute the matrix B_sp
void ConstraintSolver::computeMatrixB_sp() {
    uint indexBody1, indexBody2;

    // For each constraint
    for (uint c = 0; c<nbConstraints; c++) {
        indexBody1 = bodyNumberMapping[bodyMapping[c][0]];
        indexBody2 = bodyNumberMapping[bodyMapping[c][1]];
        //B_sp[0][c].changeSize(6,1);
        //B_sp[1][c].changeSize(6,1);
        B_sp[0][c] = Minv_sp[indexBody1] * J_sp[c][0].getTranspose();
        B_sp[1][c] = Minv_sp[indexBody2] * J_sp[c][1].getTranspose();
    }
}

// Compute the vector V_constraint (which corresponds to the constraint part of
// the final V2 vector) according to the formula
// V_constraint = dt * (M^-1 * J^T * lambda)
// Note that we use the vector V to store both V1 and V_constraint.
// Note that M^-1 * J^T = B.
// This method is called after that the LCP solver have computed lambda
void ConstraintSolver::computeVectorVconstraint(double dt) {
    uint indexBody1, indexBody2;

    // Compute dt * (M^-1 * J^T * lambda
    for (uint i=0; i<nbConstraints; i++) {
        indexBody1 = bodyNumberMapping[bodyMapping[i][0]];
        indexBody2 = bodyNumberMapping[bodyMapping[i][1]];
        Vconstraint[indexBody1] = Vconstraint[indexBody1] + (Matrix(B_sp[0][i]) * lambda.getValue(i)).getVector() * dt;
        Vconstraint[indexBody2] = Vconstraint[indexBody2] + (Matrix(B_sp[1][i]) * lambda.getValue(i)).getVector() * dt; // TODO : Remove conversion here
    }
}

// Clear and Fill in the contact cache with the new lambda values
void ConstraintSolver::updateContactCache() {
    Contact* contact;
    ContactCachingInfo* contactInfo;
    int index;

    // Clear the contact cache
    contactCache.clear();
    
    // For each active constraint
    int noConstraint = 0;
    for (int c=0; c<activeConstraints.size(); c++) {
        index = noConstraint;

        // If it's a contact constraint
        contact = dynamic_cast<Contact*>(activeConstraints.at(c));
        if (contact) {
            
            // Get all the contact points of the contact
            vector<Vector3D> points;
            vector<double> lambdas;
            for (int i=0; i<contact->getNbPoints(); i++) {
                points.push_back(contact->getPoint(i));
            }

            // For each constraint of the contact
            for (int i=0; i<contact->getNbConstraints(); i++) {
                // Get the lambda value that have just been computed
                lambdas.push_back(lambda.getValue(noConstraint + i));
            }
            
            // Create a new ContactCachingInfo
            contactInfo = new ContactCachingInfo(contact->getBody1(), contact->getBody2(), points, lambdas);

            // Add it to the contact cache
            contactCache.addContactCachingInfo(contactInfo);
        }

        noConstraint += activeConstraints.at(c)->getNbConstraints();
    }
}
