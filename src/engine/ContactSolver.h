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
#include "../constraint/Contact.h"
#include "ContactManifold.h"
#include "../configuration.h"
#include "../constraint/Constraint.h"
#include <map>
#include <set>

// ReactPhysics3D namespace
namespace reactphysics3d {

// Declarations
class DynamicsWorld;

// Structure Impulse
struct Impulse {

    public:
        const Vector3 linearImpulseBody1;
        const Vector3 linearImpulseBody2;
        const Vector3 angularImpulseBody1;
        const Vector3 angularImpulseBody2;

        // Constructor
        Impulse(const Vector3& linearImpulseBody1, const Vector3& angularImpulseBody1,
                const Vector3& linearImpulseBody2, const Vector3& angularImpulseBody2)
            : linearImpulseBody1(linearImpulseBody1), angularImpulseBody1(angularImpulseBody1),
              linearImpulseBody2(linearImpulseBody2), angularImpulseBody2(angularImpulseBody2) {

        }
};


    
/*  -------------------------------------------------------------------
    Class ContactSolver :
        This class represents the contact solver that is used is solve rigid bodies contacts.
        The constraint solver is based on the "Sequential Impulse" technique described by
        Erin Catto in his GDC slides (http://code.google.com/p/box2d/downloads/list).

        A constraint between two bodies is represented by a function C(x) which is equal to zero
        when the constraint is satisfied. The condition C(x)=0 describes a valid position and the
        condition dC(x)/dt=0 describes a valid velocity. We have dC(x)/dt = Jv + b = 0 where J is
        the Jacobian matrix of the constraint, v is a vector that contains the velocity of both
        bodies and b is the constraint bias. We are looking for a force F_c that will act on the
        bodies to keep the constraint satisfied. Note that from the virtual work principle, we have
        F_c = J^t * lambda where J^t is the transpose of the Jacobian matrix and lambda is a
        Lagrange multiplier. Therefore, finding the force F_c is equivalent to finding the Lagrange
        multiplier lambda.

        An impulse P = F * dt where F is a force and dt is the timestep. We can apply impulses a
        body to change its velocity. The idea of the Sequential Impulse technique is to apply
        impulses to bodies of each constraints in order to keep the constraint satisfied.

        --- Step 1 ---

        First, we integrate the applied force F_a acting of each rigid body (like gravity, ...) and
        we obtain some new velocities v2' that tends to violate the constraints.

        v2' = v1 + dt * M^-1 * F_a

        where M is a matrix that contains mass and inertia tensor information.

        --- Step 2 ---

        During the second step, we iterate over all the constraints for a certain number of
        iterations and for each constraint we compute the impulse to apply to the bodies needed
        so that the new velocity of the bodies satisfy Jv + b = 0. From the Newton law, we know that
        M * deltaV = P_c where M is the mass of the body, deltaV is the difference of velocity and
        P_c is the constraint impulse to apply to the body. Therefore, we have
        v2 = v2' + M^-1 * P_c. For each constraint, we can compute the Lagrange multiplier lambda
        using : lambda = -m_c (Jv2' + b) where m_c = 1 / (J * M^-1 * J^t). Now that we have the
        Lagrange multiplier lambda, we can compute the impulse P_c = J^t * lambda * dt to apply to
        the bodies to satisfy the constraint.

        --- Step 3 ---

        In the third step, we integrate the new position x2 of the bodies using the new velocities
        v2 computed in the second step with : x2 = x1 + dt * v2.

        Note that in the following code (as it is also explained in the slides from Erin Catto),
        the value lambda is not only the lagrange multiplier but is the multiplication of the
        Lagrange multiplier with the timestep dt. Therefore, in the following code, when we use
        lambda, we mean (lambda * dt).

        We are using the accumulated impulse technique that is also described in the slides from
        Erin Catto.

        We are also using warm starting. The idea is to warm start the solver at the beginning of
        each step by applying the last impulstes for the constraints that we already existing at the
        previous step. This allows the iterative solver to converge faster towards the solution.

        For contact constraints, we are also using split impulses so that the position correction
        that uses Baumgarte stabilization does not change the momentum of the bodies.

        There are two ways to apply the friction constraints. Either the friction constraints are
        applied at each contact point or they are applied only at the center of the contact manifold
        between two bodies. If we solve the friction constraints at each contact point, we need
        two constraints (two tangential friction directions) and if we solve the friction constraints
        at the center of the contact manifold, we need two constraints for tangential friction but
        also another twist friction constraint to prevent spin of the body around the contact
        manifold center.
        
    -------------------------------------------------------------------
*/
class ContactSolver {

    private:

        // Structure ContactPoint
        // Contact solver internal data structure that to store all the
        // information relative to a contact point
        struct ContactPointSolver {

            decimal penetrationImpulse;      // Accumulated normal impulse
            decimal friction1Impulse;        // Accumulated impulse in the 1st friction direction
            decimal friction2Impulse;        // Accumulated impulse in the 2nd friction direction
            decimal penetrationSplitImpulse; // Accumulated split impulse for penetration correction
            Vector3 normal;                  // Normal vector of the contact
            Vector3 frictionVector1;         // First friction vector in the tangent plane
            Vector3 frictionVector2;         // Second friction vector in the tangent plane
            Vector3 oldFrictionVector1;      // Old first friction vector in the tangent plane
            Vector3 oldFrictionVector2;      // Old second friction vector in the tangent plane
            Vector3 r1;                      // Vector from the body 1 center to the contact point
            Vector3 r2;                      // Vector from the body 2 center to the contact point
            Vector3 r1CrossT1;               // Cross product of r1 with 1st friction vector
            Vector3 r1CrossT2;               // Cross product of r1 with 2nd friction vector
            Vector3 r2CrossT1;               // Cross product of r2 with 1st friction vector
            Vector3 r2CrossT2;               // Cross product of r2 with 2nd friction vector
            Vector3 r1CrossN;                // Cross product of r1 with the contact normal
            Vector3 r2CrossN;                // Cross product of r2 with the contact normal
            decimal penetrationDepth;        // Penetration depth
            decimal restitutionBias;         // Velocity restitution bias
            decimal inversePenetrationMass;  // Inverse of the matrix K for the penenetration
            decimal inverseFriction1Mass;    // Inverse of the matrix K for the 1st friction
            decimal inverseFriction2Mass;    // Inverse of the matrix K for the 2nd friction
            bool isRestingContact;           // True if the contact was existing last time step
            Contact* contact;                // Pointer to the external contact
        };

        // Structure ContactManifold
        // Contact solver internal data structure to store all the
        // information relative to a contact manifold
        struct ContactManifoldSolver {

            // TODO : Use a constant for the number of contact points

            uint indexBody1;                         // Index of body 1 in the constraint solver
            uint indexBody2;                         // Index of body 2 in the constraint solver
            decimal massInverseBody1;                // Inverse of the mass of body 1
            decimal massInverseBody2;                // Inverse of the mass of body 2
            Matrix3x3 inverseInertiaTensorBody1;     // Inverse inertia tensor of body 1
            Matrix3x3 inverseInertiaTensorBody2;     // Inverse inertia tensor of body 2
            bool isBody1Moving;                      // True if the body 1 is allowed to move
            bool isBody2Moving;                      // True if the body 2 is allowed to move
            ContactPointSolver contacts[4];          // Contact point constraints
            uint nbContacts;                         // Number of contact points
            decimal restitutionFactor;               // Mix of the restitution factor for two bodies
            decimal frictionCoefficient;             // Mix friction coefficient for the two bodies
            ContactManifold* contactManifold;        // Pointer to the external contact manifold

            // Variables used when friction constraints are apply at the center of the manifold //

            Vector3 normal;               // Average normal vector of the contact manifold
            Vector3 frictionPointBody1;   // Point on body 1 where to apply the friction constraints
            Vector3 frictionPointBody2;   // Point on body 2 where to apply the friction constraints
            Vector3 r1Friction;           // R1 vector for the friction constraints
            Vector3 r2Friction;           // R2 vector for the friction constraints
            Vector3 r1CrossT1;            // Cross product of r1 with 1st friction vector
            Vector3 r1CrossT2;            // Cross product of r1 with 2nd friction vector
            Vector3 r2CrossT1;            // Cross product of r2 with 1st friction vector
            Vector3 r2CrossT2;            // Cross product of r2 with 2nd friction vector
            decimal inverseFriction1Mass; // Matrix K for the first friction constraint
            decimal inverseFriction2Mass; // Matrix K for the second friction constraint
            Vector3 frictionVector1;      // First friction direction at contact manifold center
            Vector3 frictionVector2;      // Second friction direction at contact manifold center
            Vector3 oldFrictionVector1;   // Old 1st friction direction at contact manifold center
            Vector3 oldFrictionVector2;   // Old 2nd friction direction at contact manifold center
            decimal friction1Impulse;     // First friction direction impulse at manifold center
            decimal friction2Impulse;     // Second friction direction impulse at manifold center
            decimal frictionTwistImpulse; // Twist friction impulse at contact manifold center
        };

        // -------------------- Constants --------------------- //

        // Beta value for the penetration depth position correction without split impulses
        static const decimal BETA;

        // Beta value for the penetration depth position correction with split impulses
        static const decimal BETA_SPLIT_IMPULSE;

        // Slop distance (allowed penetration distance between bodies)
        static const decimal SLOP;

        // -------------------- Attributes -------------------- //

        // Reference to the world
        DynamicsWorld& mWorld;

        // Number of iterations of the constraints solver
        uint mNbIterations;

        // Split linear velocities for the position contact solver (split impulse)
        Vector3* mSplitLinearVelocities;

        // Split angular velocities for the position contact solver (split impulse)
        Vector3* mSplitAngularVelocities;

        // Current time step
        decimal mTimeStep;

        // Contact constraints
        ContactManifoldSolver* mContactConstraints;

        // Number of contact constraints
        uint mNbContactConstraints;

        // Constrained bodies
        std::set<RigidBody*> mConstraintBodies;

        // Pointer to the array of constrained linear velocities (state of the linear velocities
        // after solving the constraints)
        std::vector<Vector3>& mConstrainedLinearVelocities;

        // Pointer to the array of constrained angular velocities (state of the angular velocities
        // after solving the constraints)
        std::vector<Vector3>& mConstrainedAngularVelocities;

        // Reference to the map of rigid body to their index in the constrained velocities array
        const std::map<RigidBody*, uint>& mMapBodyToConstrainedVelocityIndex;

        // True if the warm starting of the solver is active
        bool mIsWarmStartingActive;

        // True if the split impulse position correction is active
        bool mIsSplitImpulseActive;

        // True if we solve 3 friction constraints at the contact manifold center only
        // instead of 2 friction constraints at each contact point
        bool mIsSolveFrictionAtContactManifoldCenterActive;

        // -------------------- Methods -------------------- //

        // Initialize the constraint solver
        void initialize();

        // Initialize the constrained bodies
        void initializeBodies();

        // Initialize the contact constraints before solving the system
        void initializeContactConstraints();

        // Store the computed impulses to use them to
        // warm start the solver at the next iteration
        void storeImpulses();

        // Warm start the solver
        void warmStart();

        // Solve the contact constraints by applying sequential impulses
        void solveContactConstraints();

        // Apply an impulse to the two bodies of a constraint
        void applyImpulse(const Impulse& impulse, const ContactManifoldSolver& contactManifold);

        // Apply an impulse to the two bodies of a constraint
        void applySplitImpulse(const Impulse& impulse, const ContactManifoldSolver& contactManifold);

        // Compute the collision restitution factor from the restitution factor of each body
        decimal computeMixedRestitutionFactor(const RigidBody* body1, const RigidBody* body2) const;

        // Compute the mixed friction coefficient from the friction coefficient of each body
        decimal computeMixedFrictionCoefficient(const RigidBody* body1, const RigidBody* body2)const;

        // Compute the two unit orthogonal vectors "t1" and "t2" that span the tangential friction
        // plane for a contact point constraint. The two vectors have to be
        // such that : t1 x t2 = contactNormal.
        void computeFrictionVectors(const Vector3& deltaVelocity,
                                    ContactPointSolver &contactPoint) const;

        // Compute the two unit orthogonal vectors "t1" and "t2" that span the tangential friction
        // plane for a contact constraint. The two vectors have to be
        // such that : t1 x t2 = contactNormal.
        void computeFrictionVectors(const Vector3& deltaVelocity,
                                    ContactManifoldSolver& contactPoint) const;

        // Compute a penetration constraint impulse
        const Impulse computePenetrationImpulse(decimal deltaLambda,
                                                const ContactPointSolver& contactPoint) const;

        // Compute the first friction constraint impulse
        const Impulse computeFriction1Impulse(decimal deltaLambda,
                                              const ContactPointSolver& contactPoint) const;

        // Compute the second friction constraint impulse
        const Impulse computeFriction2Impulse(decimal deltaLambda,
                                              const ContactPointSolver& contactPoint) const;

   public:

        // -------------------- Methods -------------------- //

        // Constructor
        ContactSolver(DynamicsWorld& mWorld, std::vector<Vector3>& constrainedLinearVelocities,
                      std::vector<Vector3>& constrainedAngularVelocities, const std::map<RigidBody*, uint>&
                      mapBodyToVelocityIndex);

        // Destructor
        virtual ~ContactSolver();

        // Solve the constraints
        void solve(decimal timeStep);

        // Return true if the body is in at least one constraint
        bool isConstrainedBody(RigidBody* body) const;

        // Return the constrained linear velocity of a body after solving the constraints
        Vector3 getConstrainedLinearVelocityOfBody(RigidBody *body);

        // Return the split linear velocity
        Vector3 getSplitLinearVelocityOfBody(RigidBody* body);

        // Return the constrained angular velocity of a body after solving the constraints
        Vector3 getConstrainedAngularVelocityOfBody(RigidBody* body);

        // Return the split angular velocity
        Vector3 getSplitAngularVelocityOfBody(RigidBody* body);

        // Clean up the constraint solver
        void cleanup();

        // Set the number of iterations of the constraint solver
        void setNbIterationsSolver(uint nbIterations);

        // Activate or Deactivate the split impulses for contacts
        void setIsSplitImpulseActive(bool isActive);

        // Activate or deactivate the solving of friction constraints at the center of
        // the contact manifold instead of solving them at each contact point
        void setIsSolveFrictionAtContactManifoldCenterActive(bool isActive);
};

// Return true if the body is in at least one constraint
inline bool ContactSolver::isConstrainedBody(RigidBody* body) const {
    return mConstraintBodies.count(body) == 1;
}

// Return the split linear velocity
inline Vector3 ContactSolver::getSplitLinearVelocityOfBody(RigidBody* body) {
    assert(isConstrainedBody(body));
    const uint indexBody = mMapBodyToConstrainedVelocityIndex.find(body)->second;
    return mSplitLinearVelocities[indexBody];
}

// Return the split angular velocity
inline Vector3 ContactSolver::getSplitAngularVelocityOfBody(RigidBody* body) {
    assert(isConstrainedBody(body));
    const uint indexBody = mMapBodyToConstrainedVelocityIndex.find(body)->second;
    return mSplitAngularVelocities[indexBody];
}

// Set the number of iterations of the constraint solver
inline void ContactSolver::setNbIterationsSolver(uint nbIterations) {
    mNbIterations = nbIterations;
}

// Activate or Deactivate the split impulses for contacts
inline void ContactSolver::setIsSplitImpulseActive(bool isActive) {
    mIsSplitImpulseActive = isActive;
}

// Activate or deactivate the solving of friction constraints at the center of
// the contact manifold instead of solving them at each contact point
inline void ContactSolver::setIsSolveFrictionAtContactManifoldCenterActive(bool isActive) {
    mIsSolveFrictionAtContactManifoldCenterActive = isActive;
}

// Compute the collision restitution factor from the restitution factor of each body
inline decimal ContactSolver::computeMixedRestitutionFactor(const RigidBody* body1,
                                                               const RigidBody* body2) const {
    decimal restitution1 = body1->getRestitution();
    decimal restitution2 = body2->getRestitution();

    // Return the largest restitution factor
    return (restitution1 > restitution2) ? restitution1 : restitution2;
}

// Compute the mixed friction coefficient from the friction coefficient of each body
inline decimal ContactSolver::computeMixedFrictionCoefficient(const RigidBody* body1,
                                                              const RigidBody* body2) const {
    // Use the geometric mean to compute the mixed friction coefficient
    return sqrt(body1->getFrictionCoefficient() * body2->getFrictionCoefficient());
}

// Compute a penetration constraint impulse
inline const Impulse ContactSolver::computePenetrationImpulse(decimal deltaLambda,
                                                          const ContactPointSolver& contactPoint)
                                                          const {
    return Impulse(-contactPoint.normal * deltaLambda, -contactPoint.r1CrossN * deltaLambda,
                   contactPoint.normal * deltaLambda, contactPoint.r2CrossN * deltaLambda);
}

// Compute the first friction constraint impulse
inline const Impulse ContactSolver::computeFriction1Impulse(decimal deltaLambda,
                                                        const ContactPointSolver& contactPoint)
                                                        const {
    return Impulse(-contactPoint.frictionVector1 * deltaLambda, -contactPoint.r1CrossT1 * deltaLambda,
                   contactPoint.frictionVector1 * deltaLambda, contactPoint.r2CrossT1 * deltaLambda);
}

// Compute the second friction constraint impulse
inline const Impulse ContactSolver::computeFriction2Impulse(decimal deltaLambda,
                                                        const ContactPointSolver& contactPoint)
                                                        const {
    return Impulse(-contactPoint.frictionVector2 * deltaLambda, -contactPoint.r1CrossT2 * deltaLambda,
                   contactPoint.frictionVector2 * deltaLambda, contactPoint.r2CrossT2 * deltaLambda);
}

}

#endif
