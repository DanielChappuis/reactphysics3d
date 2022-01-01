/********************************************************************************
* ReactPhysics3D physics library, http://www.reactphysics3d.com                 *
* Copyright (c) 2010-2022 Daniel Chappuis                                       *
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

#ifndef REACTPHYSICS3D_CONTACT_SOLVER_SYSTEM_H
#define REACTPHYSICS3D_CONTACT_SOLVER_SYSTEM_H

// Libraries
#include <reactphysics3d/configuration.h>
#include <reactphysics3d/mathematics/Vector3.h>
#include <reactphysics3d/mathematics/Matrix3x3.h>
#include <reactphysics3d/containers/Array.h>
#include <reactphysics3d/engine/Material.h>

/// ReactPhysics3D namespace
namespace reactphysics3d {

// Declarations
class ContactPoint;
class Joint;
class ContactManifold;
class MemoryManager;
class Profiler;
class Island;
struct Islands;
class RigidBody;
class Collider;
class PhysicsWorld;
class CollisionBodyComponents;
class DynamicsComponents;
class RigidBodyComponents;
class ColliderComponents;

// Class ContactSolverSystem
/**
 * This class represents the contact solver system that is used to solve rigid bodies contacts.
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
class ContactSolverSystem {

    private:

        // Structure ContactPointSolver
        /**
         * Contact solver internal data structure that to store all the
         * information relative to a contact point
         */
        struct ContactPointSolver {

            /// Pointer to the external contact
            ContactPoint* externalContact;

            /// Normal vector of the contact
            Vector3 normal;

            /// Vector from the body 1 center to the contact point
            Vector3 r1;

            /// Vector from the body 2 center to the contact point
            Vector3 r2;

            /// Penetration depth
            decimal penetrationDepth;

            /// Velocity restitution bias
            decimal restitutionBias;

            /// Accumulated normal impulse
            decimal penetrationImpulse;

            /// Accumulated split impulse for penetration correction
            decimal penetrationSplitImpulse;

            /// Inverse of the matrix K for the penenetration
            decimal inversePenetrationMass;

            /// Cross product of r1 with the contact normal
            Vector3 i1TimesR1CrossN;

            /// Cross product of r2 with the contact normal
            Vector3 i2TimesR2CrossN;

            /// True if the contact was existing last time step
            bool isRestingContact;
        };

        // Structure ContactManifoldSolver
        /**
         * Contact solver internal data structure to store all the
         * information relative to a contact manifold.
         */
        struct ContactManifoldSolver {

            /// Pointer to the external contact manifold
            ContactManifold* externalContactManifold;

            /// Index of body 1 in the dynamics components arrays
            uint32 rigidBodyComponentIndexBody1;

            /// Index of body 2 in the dynamics components arrays
            uint32 rigidBodyComponentIndexBody2;

            /// Inverse of the mass of body 1
            decimal massInverseBody1;

            /// Inverse of the mass of body 2
            decimal massInverseBody2;

            /// Linear lock axis factor of body 1
            Vector3 linearLockAxisFactorBody1;

            /// Linear lock axis factor of body 2
            Vector3 linearLockAxisFactorBody2;

            /// Angular lock axis factor of body 1
            Vector3 angularLockAxisFactorBody1;

            /// Angular lock axis factor of body 2
            Vector3 angularLockAxisFactorBody2;

            /// Inverse inertia tensor of body 1
            Matrix3x3 inverseInertiaTensorBody1;

            /// Inverse inertia tensor of body 2
            Matrix3x3 inverseInertiaTensorBody2;

            /// Mix friction coefficient for the two bodies
            decimal frictionCoefficient;

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

            /// Number of contact points
            int8 nbContacts;
        };

        // -------------------- Constants --------------------- //

        /// Beta value for the penetration depth position correction without split impulses
        static const decimal BETA;

        /// Beta value for the penetration depth position correction with split impulses
        static const decimal BETA_SPLIT_IMPULSE;

        /// Slop distance (allowed penetration distance between bodies)
        static const decimal SLOP;

        // -------------------- Attributes -------------------- //

        /// Memory manager
        MemoryManager& mMemoryManager;

        /// Physics world
        PhysicsWorld& mWorld;

        /// Current time step
        decimal mTimeStep;

        /// Reference to the velocity threshold for contact velocity restitution
        decimal& mRestitutionVelocityThreshold;

        /// Contact constraints
        ContactManifoldSolver* mContactConstraints;

        /// Contact points
        ContactPointSolver* mContactPoints;

        /// Number of contact point constraints
        uint32 mNbContactPoints;

        /// Number of contact constraints
        uint32 mNbContactManifolds;

        /// Reference to the islands
        Islands& mIslands;

        /// Pointer to the array of contact manifolds from narrow-phase
        Array<ContactManifold>* mAllContactManifolds;

        /// Pointer to the array of contact points from narrow-phase
        Array<ContactPoint>* mAllContactPoints;

        /// Reference to the body components
        CollisionBodyComponents& mBodyComponents;

        /// Reference to the dynamics components
        RigidBodyComponents& mRigidBodyComponents;

        /// Reference to the colliders components
        ColliderComponents& mColliderComponents;

        /// True if the split impulse position correction is active
        bool mIsSplitImpulseActive;

#ifdef IS_RP3D_PROFILING_ENABLED

		/// Pointer to the profiler
		Profiler* mProfiler;

#endif

        // -------------------- Methods -------------------- //

        /// Compute the collision restitution factor from the restitution factor of each collider
        decimal computeMixedRestitutionFactor(const Material& material1, const Material& material2) const;

        /// Compute the mixed friction coefficient from the friction coefficient of each collider
        decimal computeMixedFrictionCoefficient(const Material &material1, const Material &material2) const;

        /// Compute the two unit orthogonal vectors "t1" and "t2" that span the tangential friction
        /// plane for a contact manifold. The two vectors have to be
        /// such that : t1 x t2 = contactNormal.
        void computeFrictionVectors(const Vector3& deltaVelocity,
                                    ContactManifoldSolver& contactPoint) const;

        /// Warm start the solver.
        void warmStart();

   public:

        // -------------------- Methods -------------------- //

        /// Constructor
        ContactSolverSystem(MemoryManager& memoryManager, PhysicsWorld& world, Islands& islands, CollisionBodyComponents& bodyComponents,
                      RigidBodyComponents& rigidBodyComponents, ColliderComponents& colliderComponents, decimal& restitutionVelocityThreshold);

        /// Destructor
        ~ContactSolverSystem() = default;

        /// Initialize the contact constraints
        void init(Array<ContactManifold>* contactManifolds, Array<ContactPoint>* contactPoints, decimal timeStep);

        /// Initialize the constraint solver for a given island
        void initializeForIsland(uint32 islandIndex);

        /// Store the computed impulses to use them to
        /// warm start the solver at the next iteration
        void storeImpulses();

        /// Solve the contacts
        void solve();

        /// Release allocated memory
        void reset();

        /// Return true if the split impulses position correction technique is used for contacts
        bool isSplitImpulseActive() const;

        /// Activate or Deactivate the split impulses for contacts
        void setIsSplitImpulseActive(bool isActive);

#ifdef IS_RP3D_PROFILING_ENABLED

		/// Set the profiler
		void setProfiler(Profiler* profiler);

#endif
};

// Return true if the split impulses position correction technique is used for contacts
RP3D_FORCE_INLINE bool ContactSolverSystem::isSplitImpulseActive() const {
    return mIsSplitImpulseActive;
}

// Activate or Deactivate the split impulses for contacts
RP3D_FORCE_INLINE void ContactSolverSystem::setIsSplitImpulseActive(bool isActive) {
    mIsSplitImpulseActive = isActive;
}

// Compute the collision restitution factor from the restitution factor of each collider
RP3D_FORCE_INLINE decimal ContactSolverSystem::computeMixedRestitutionFactor(const Material& material1, const Material& material2) const {

    const decimal restitution1 = material1.getBounciness();
    const decimal restitution2 = material2.getBounciness();

    // Return the largest restitution factor
    return (restitution1 > restitution2) ? restitution1 : restitution2;
}

// Compute the mixed friction coefficient from the friction coefficient of each collider
RP3D_FORCE_INLINE decimal ContactSolverSystem::computeMixedFrictionCoefficient(const Material& material1, const Material& material2) const {

    // Use the geometric mean to compute the mixed friction coefficient
    return material1.getFrictionCoefficientSqrt() * material2.getFrictionCoefficientSqrt();
}

#ifdef IS_RP3D_PROFILING_ENABLED

// Set the profiler
RP3D_FORCE_INLINE void ContactSolverSystem::setProfiler(Profiler* profiler) {

	mProfiler = profiler;
}

#endif

}

#endif
