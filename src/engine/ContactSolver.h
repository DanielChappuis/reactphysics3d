/********************************************************************************
* ReactPhysics3D physics library, http://www.reactphysics3d.com                 *
* Copyright (c) 2010-2016 Daniel Chappuis                                       *
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

#ifndef REACTPHYSICS3D_CONTACT_SOLVER_H
#define REACTPHYSICS3D_CONTACT_SOLVER_H

// Libraries
#include "constraint/ContactPoint.h"
#include "configuration.h"
#include "constraint/Joint.h"
#include "collision/ContactManifold.h"
#include "Island.h"
#include "Impulse.h"
#include <map>
#include <set>

/// ReactPhysics3D namespace
namespace reactphysics3d {


// Class Contact Solver
/**
 * This class represents the contact solver that is used to solve rigid bodies contacts.
 * The constraint solver is based on the "Sequential Impulse" technique described by
 * Erin Catto in his GDC slides (http://code.google.com/p/box2d/downloads/list).
 *
 * A constraint between two bodies is represented by a function C(x) which is equal to zero
 * when the constraint is satisfied. The condition C(x)=0 describes a valid position and the
 * condition dC(x)/dt=0 describes a valid velocity. We have dC(x)/dt = Jv + b = 0 where J is
 * the Jacobian matrix of the constraint, v is a vector that contains the velocity of both
 * bodies and b is the constraint bias. We are looking for a force F_c that will act on the
 * bodies to keep the constraint satisfied. Note that from the virtual work principle, we have
 * F_c = J^t * lambda where J^t is the transpose of the Jacobian matrix and lambda is a
 * Lagrange multiplier. Therefore, finding the force F_c is equivalent to finding the Lagrange
 * multiplier lambda.
 *
 * An impulse P = F * dt where F is a force and dt is the timestep. We can apply impulses a
 * body to change its velocity. The idea of the Sequential Impulse technique is to apply
 * impulses to bodies of each constraints in order to keep the constraint satisfied.
 *
 * --- Step 1 ---
 *
 * First, we integrate the applied force F_a acting of each rigid body (like gravity, ...) and
 * we obtain some new velocities v2' that tends to violate the constraints.
 *
 * v2' = v1 + dt * M^-1 * F_a
 *
 * where M is a matrix that contains mass and inertia tensor information.
 *
 * --- Step 2 ---
 *
 * During the second step, we iterate over all the constraints for a certain number of
 * iterations and for each constraint we compute the impulse to apply to the bodies needed
 * so that the new velocity of the bodies satisfy Jv + b = 0. From the Newton law, we know that
 * M * deltaV = P_c where M is the mass of the body, deltaV is the difference of velocity and
 * P_c is the constraint impulse to apply to the body. Therefore, we have
 * v2 = v2' + M^-1 * P_c. For each constraint, we can compute the Lagrange multiplier lambda
 * using : lambda = -m_c (Jv2' + b) where m_c = 1 / (J * M^-1 * J^t). Now that we have the
 * Lagrange multiplier lambda, we can compute the impulse P_c = J^t * lambda * dt to apply to
 * the bodies to satisfy the constraint.
 *
 * --- Step 3 ---
 *
 * In the third step, we integrate the new position x2 of the bodies using the new velocities
 * v2 computed in the second step with : x2 = x1 + dt * v2.
 *
 * Note that in the following code (as it is also explained in the slides from Erin Catto),
 * the value lambda is not only the lagrange multiplier but is the multiplication of the
 * Lagrange multiplier with the timestep dt. Therefore, in the following code, when we use
 * lambda, we mean (lambda * dt).
 *
 * We are using the accumulated impulse technique that is also described in the slides from
 * Erin Catto.
 *
 * We are also using warm starting. The idea is to warm start the solver at the beginning of
 * each step by applying the last impulstes for the constraints that we already existing at the
 * previous step. This allows the iterative solver to converge faster towards the solution.
 *
 * For contact constraints, we are also using split impulses so that the position correction
 * that uses Baumgarte stabilization does not change the momentum of the bodies.
 *
 * There are two ways to apply the friction constraints. Either the friction constraints are
 * applied at each contact point or they are applied only at the center of the contact manifold
 * between two bodies. If we solve the friction constraints at each contact point, we need
 * two constraints (two tangential friction directions) and if we solve the friction
 * constraints at the center of the contact manifold, we need two constraints for tangential
 * friction but also another twist friction constraint to prevent spin of the body around the
 * contact manifold center.
 */
class ContactSolver {

    private:

        // Structure ContactPointSolver
        /**
         * Contact solver internal data structure that to store all the
         * information relative to a contact point
         */
        struct ContactPointSolver {

            /// Accumulated normal impulse
            decimal penetrationImpulse;

            /// Accumulated impulse in the 1st friction direction
            decimal friction1Impulse;

            /// Accumulated impulse in the 2nd friction direction
            decimal friction2Impulse;

            /// Accumulated split impulse for penetration correction
            decimal penetrationSplitImpulse;

            /// Accumulated rolling resistance impulse
            Vector3 rollingResistanceImpulse;

            /// Normal vector of the contact
            Vector3 normal;

            /// First friction vector in the tangent plane
            Vector3 frictionVector1;

            /// Second friction vector in the tangent plane
            Vector3 frictionVector2;

            /// Old first friction vector in the tangent plane
            Vector3 oldFrictionVector1;

            /// Old second friction vector in the tangent plane
            Vector3 oldFrictionVector2;

            /// Vector from the body 1 center to the contact point
            Vector3 r1;

            /// Vector from the body 2 center to the contact point
            Vector3 r2;

            /// Cross product of r1 with 1st friction vector
            Vector3 r1CrossT1;

            /// Cross product of r1 with 2nd friction vector
            Vector3 r1CrossT2;

            /// Cross product of r2 with 1st friction vector
            Vector3 r2CrossT1;

            /// Cross product of r2 with 2nd friction vector
            Vector3 r2CrossT2;

            /// Cross product of r1 with the contact normal
            Vector3 r1CrossN;

            /// Cross product of r2 with the contact normal
            Vector3 r2CrossN;

            /// Penetration depth
            decimal penetrationDepth;

            /// Velocity restitution bias
            decimal restitutionBias;

            /// Inverse of the matrix K for the penenetration
            decimal inversePenetrationMass;

            /// Inverse of the matrix K for the 1st friction
            decimal inverseFriction1Mass;

            /// Inverse of the matrix K for the 2nd friction
            decimal inverseFriction2Mass;

            /// True if the contact was existing last time step
            bool isRestingContact;

            /// Pointer to the external contact
            ContactPoint* externalContact;
        };

        // Structure ContactManifoldSolver
        /**
         * Contact solver internal data structure to store all the
         * information relative to a contact manifold.
         */
        struct ContactManifoldSolver {

            /// Index of body 1 in the constraint solver
            uint indexBody1;

            /// Index of body 2 in the constraint solver
            uint indexBody2;

            /// Inverse of the mass of body 1
            decimal massInverseBody1;

            // Inverse of the mass of body 2
            decimal massInverseBody2;

            /// Inverse inertia tensor of body 1
            Matrix3x3 inverseInertiaTensorBody1;

            /// Inverse inertia tensor of body 2
            Matrix3x3 inverseInertiaTensorBody2;

            /// Contact point constraints
            ContactPointSolver contacts[MAX_CONTACT_POINTS_IN_MANIFOLD];

            /// Number of contact points
            uint nbContacts;

            /// True if the body 1 is of type dynamic
            bool isBody1DynamicType;

            /// True if the body 2 is of type dynamic
            bool isBody2DynamicType;

            /// Mix of the restitution factor for two bodies
            decimal restitutionFactor;

            /// Mix friction coefficient for the two bodies
            decimal frictionCoefficient;

            /// Rolling resistance factor between the two bodies
            decimal rollingResistanceFactor;

            /// Pointer to the external contact manifold
            ContactManifold* externalContactManifold;

            // - Variables used when friction constraints are apply at the center of the manifold-//

            /// Average normal vector of the contact manifold
            Vector3 normal;

            /// Point on body 1 where to apply the friction constraints
            Vector3 frictionPointBody1;

            /// Point on body 2 where to apply the friction constraints
            Vector3 frictionPointBody2;

            /// R1 vector for the friction constraints
            Vector3 r1Friction;

            /// R2 vector for the friction constraints
            Vector3 r2Friction;

            /// Cross product of r1 with 1st friction vector
            Vector3 r1CrossT1;

            /// Cross product of r1 with 2nd friction vector
            Vector3 r1CrossT2;

            /// Cross product of r2 with 1st friction vector
            Vector3 r2CrossT1;

            /// Cross product of r2 with 2nd friction vector
            Vector3 r2CrossT2;

            /// Matrix K for the first friction constraint
            decimal inverseFriction1Mass;

            /// Matrix K for the second friction constraint
            decimal inverseFriction2Mass;

            /// Matrix K for the twist friction constraint
            decimal inverseTwistFrictionMass;

            /// Matrix K for the rolling resistance constraint
            Matrix3x3 inverseRollingResistance;

            /// First friction direction at contact manifold center
            Vector3 frictionVector1;

            /// Second friction direction at contact manifold center
            Vector3 frictionVector2;

            /// Old 1st friction direction at contact manifold center
            Vector3 oldFrictionVector1;

            /// Old 2nd friction direction at contact manifold center
            Vector3 oldFrictionVector2;

            /// First friction direction impulse at manifold center
            decimal friction1Impulse;

            /// Second friction direction impulse at manifold center
            decimal friction2Impulse;

            /// Twist friction impulse at contact manifold center
            decimal frictionTwistImpulse;

            /// Rolling resistance impulse
            Vector3 rollingResistanceImpulse;
        };

        // -------------------- Constants --------------------- //

        /// Beta value for the penetration depth position correction without split impulses
        static const decimal BETA;

        /// Beta value for the penetration depth position correction with split impulses
        static const decimal BETA_SPLIT_IMPULSE;

        /// Slop distance (allowed penetration distance between bodies)
        static const decimal SLOP;

        // -------------------- Attributes -------------------- //

        /// Split linear velocities for the position contact solver (split impulse)
        Vector3* mSplitLinearVelocities;

        /// Split angular velocities for the position contact solver (split impulse)
        Vector3* mSplitAngularVelocities;

        /// Current time step
        decimal mTimeStep;

        /// Contact constraints
        ContactManifoldSolver* mContactConstraints;

        /// Number of contact constraints
        uint mNbContactManifolds;

        /// Array of linear velocities
        Vector3* mLinearVelocities;

        /// Array of angular velocities
        Vector3* mAngularVelocities;

        /// Reference to the map of rigid body to their index in the constrained velocities array
        const std::map<RigidBody*, uint>& mMapBodyToConstrainedVelocityIndex;

        /// True if the warm starting of the solver is active
        bool mIsWarmStartingActive;

        /// True if the split impulse position correction is active
        bool mIsSplitImpulseActive;

        /// True if we solve 3 friction constraints at the contact manifold center only
        /// instead of 2 friction constraints at each contact point
        bool mIsSolveFrictionAtContactManifoldCenterActive;

        // -------------------- Methods -------------------- //

        /// Initialize the contact constraints before solving the system
        void initializeContactConstraints();

        /// Apply an impulse to the two bodies of a constraint
        void applyImpulse(const Impulse& impulse, const ContactManifoldSolver& manifold);

        /// Apply an impulse to the two bodies of a constraint
        void applySplitImpulse(const Impulse& impulse,
                               const ContactManifoldSolver& manifold);

        /// Compute the collision restitution factor from the restitution factor of each body
        decimal computeMixedRestitutionFactor(RigidBody *body1,
                                              RigidBody *body2) const;

        /// Compute the mixed friction coefficient from the friction coefficient of each body
        decimal computeMixedFrictionCoefficient(RigidBody* body1,
                                                RigidBody* body2) const;

        /// Compute th mixed rolling resistance factor between two bodies
        decimal computeMixedRollingResistance(RigidBody* body1, RigidBody* body2) const;

        /// Compute the two unit orthogonal vectors "t1" and "t2" that span the tangential friction
        /// plane for a contact point. The two vectors have to be
        /// such that : t1 x t2 = contactNormal.
        void computeFrictionVectors(const Vector3& deltaVelocity,
                                    ContactPointSolver &contactPoint) const;

        /// Compute the two unit orthogonal vectors "t1" and "t2" that span the tangential friction
        /// plane for a contact manifold. The two vectors have to be
        /// such that : t1 x t2 = contactNormal.
        void computeFrictionVectors(const Vector3& deltaVelocity,
                                    ContactManifoldSolver& contactPoint) const;

        /// Compute a penetration constraint impulse
        const Impulse computePenetrationImpulse(decimal deltaLambda,
                                                const ContactPointSolver& contactPoint) const;

        /// Compute the first friction constraint impulse
        const Impulse computeFriction1Impulse(decimal deltaLambda,
                                              const ContactPointSolver& contactPoint) const;

        /// Compute the second friction constraint impulse
        const Impulse computeFriction2Impulse(decimal deltaLambda,
                                              const ContactPointSolver& contactPoint) const;

   public:

        // -------------------- Methods -------------------- //

        /// Constructor
        ContactSolver(const std::map<RigidBody*, uint>& mapBodyToVelocityIndex);

        /// Destructor
        virtual ~ContactSolver();

        /// Initialize the constraint solver for a given island
        void initializeForIsland(decimal dt, Island* island);

        /// Set the split velocities arrays
        void setSplitVelocitiesArrays(Vector3* splitLinearVelocities,
                                      Vector3* splitAngularVelocities);

        /// Set the constrained velocities arrays
        void setConstrainedVelocitiesArrays(Vector3* constrainedLinearVelocities,
                                            Vector3* constrainedAngularVelocities);

        /// Warm start the solver.
        void warmStart();

        /// Store the computed impulses to use them to
        /// warm start the solver at the next iteration
        void storeImpulses();

        /// Solve the contacts
        void solve();

        /// Return true if the split impulses position correction technique is used for contacts
        bool isSplitImpulseActive() const;

        /// Activate or Deactivate the split impulses for contacts
        void setIsSplitImpulseActive(bool isActive);

        /// Activate or deactivate the solving of friction constraints at the center of
        /// the contact manifold instead of solving them at each contact point
        void setIsSolveFrictionAtContactManifoldCenterActive(bool isActive);

        /// Clean up the constraint solver
        void cleanup();
};

// Set the split velocities arrays
inline void ContactSolver::setSplitVelocitiesArrays(Vector3* splitLinearVelocities,
                                                    Vector3* splitAngularVelocities) {
    assert(splitLinearVelocities != NULL);
    assert(splitAngularVelocities != NULL);
    mSplitLinearVelocities = splitLinearVelocities;
    mSplitAngularVelocities = splitAngularVelocities;
}

// Set the constrained velocities arrays
inline void ContactSolver::setConstrainedVelocitiesArrays(Vector3* constrainedLinearVelocities,
                                                          Vector3* constrainedAngularVelocities) {
    assert(constrainedLinearVelocities != NULL);
    assert(constrainedAngularVelocities != NULL);
    mLinearVelocities = constrainedLinearVelocities;
    mAngularVelocities = constrainedAngularVelocities;
}

// Return true if the split impulses position correction technique is used for contacts
inline bool ContactSolver::isSplitImpulseActive() const {
    return mIsSplitImpulseActive;
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
inline decimal ContactSolver::computeMixedRestitutionFactor(RigidBody* body1,
                                                            RigidBody* body2) const {
    decimal restitution1 = body1->getMaterial().getBounciness();
    decimal restitution2 = body2->getMaterial().getBounciness();

    // Return the largest restitution factor
    return (restitution1 > restitution2) ? restitution1 : restitution2;
}

// Compute the mixed friction coefficient from the friction coefficient of each body
inline decimal ContactSolver::computeMixedFrictionCoefficient(RigidBody *body1,
                                                              RigidBody *body2) const {
    // Use the geometric mean to compute the mixed friction coefficient
    return sqrt(body1->getMaterial().getFrictionCoefficient() *
                body2->getMaterial().getFrictionCoefficient());
}

// Compute th mixed rolling resistance factor between two bodies
inline decimal ContactSolver::computeMixedRollingResistance(RigidBody* body1,
                                                            RigidBody* body2) const {
    return decimal(0.5f) * (body1->getMaterial().getRollingResistance() + body2->getMaterial().getRollingResistance());
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
    return Impulse(-contactPoint.frictionVector1 * deltaLambda,
                   -contactPoint.r1CrossT1 * deltaLambda,
                   contactPoint.frictionVector1 * deltaLambda,
                   contactPoint.r2CrossT1 * deltaLambda);
}

// Compute the second friction constraint impulse
inline const Impulse ContactSolver::computeFriction2Impulse(decimal deltaLambda,
                                                        const ContactPointSolver& contactPoint)
                                                        const {
    return Impulse(-contactPoint.frictionVector2 * deltaLambda,
                   -contactPoint.r1CrossT2 * deltaLambda,
                   contactPoint.frictionVector2 * deltaLambda,
                   contactPoint.r2CrossT2 * deltaLambda);
}

}

#endif
