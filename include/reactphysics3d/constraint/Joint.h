/********************************************************************************
* ReactPhysics3D physics library, http://www.reactphysics3d.com                 *
* Copyright (c) 2010-2024 Daniel Chappuis                                       *
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

#ifndef REACTPHYSICS3D_CONSTRAINT_H
#define REACTPHYSICS3D_CONSTRAINT_H

// Libraries
#include <reactphysics3d/configuration.h>
#include <reactphysics3d/body/RigidBody.h>
#include <reactphysics3d/mathematics/mathematics.h>

// ReactPhysics3D namespace
namespace reactphysics3d {

/// Enumeration for the type of a constraint
enum class JointType {BALLSOCKETJOINT, SLIDERJOINT, HINGEJOINT, FIXEDJOINT};

// Class declarations
struct ConstraintSolverData;
class Joint;
class RigidBody;

// Structure JointInfo
/**
 * This structure is used to gather the information needed to create a joint.
 */
struct JointInfo {

    public :

        // -------------------- Attributes -------------------- //

        /// First rigid body of the joint
        RigidBody* body1;

        /// Second rigid body of the joint
        RigidBody* body2;

        /// Type of the joint
        JointType type;

        /// Position correction technique used for the constraint (used for joints).
        /// By default, the NON_LINEAR_GAUSS_SEIDEL technique is used
        JointsPositionCorrectionTechnique positionCorrectionTechnique;

        /// True if the two bodies of the joint are allowed to collide with each other
        bool isCollisionEnabled;

        /// Constructor
        JointInfo(JointType constraintType)
                      : body1(nullptr), body2(nullptr), type(constraintType),
                        positionCorrectionTechnique(JointsPositionCorrectionTechnique::NON_LINEAR_GAUSS_SEIDEL),
                        isCollisionEnabled(true) {

        }

        /// Constructor
        JointInfo(RigidBody* rigidBody1, RigidBody* rigidBody2, JointType constraintType)
                      : body1(rigidBody1), body2(rigidBody2), type(constraintType),
                        positionCorrectionTechnique(JointsPositionCorrectionTechnique::NON_LINEAR_GAUSS_SEIDEL),
                        isCollisionEnabled(true) {
        }

        /// Destructor
        virtual ~JointInfo() = default;

};

// Class Joint
/**
 * This abstract class represents a joint between two bodies.
 */
class Joint {

    protected :

        // -------------------- Attributes -------------------- //

        /// Entity ID of the joint
        Entity mEntity;

        /// Reference to the physics world
        PhysicsWorld& mWorld;

        // -------------------- Methods -------------------- //

        /// Return the number of bytes used by the joint
        virtual size_t getSizeInBytes() const = 0;

        /// Awake the two bodies of the joint
        void awakeBodies() const;

    public :

        // -------------------- Methods -------------------- //

        /// Constructor
        Joint(Entity entity, PhysicsWorld& world);

        /// Destructor
        virtual ~Joint() = default;

        /// Deleted copy-constructor
        Joint(const Joint& constraint) = delete;

        /// Deleted assignment operator
        Joint& operator=(const Joint& constraint) = delete;

        /// Return the reference to the body 1
        RigidBody* getBody1() const;

        /// Return the reference to the body 2
        RigidBody* getBody2() const;

        /// Return the type of the constraint
        JointType getType() const;

        /// Return the force (in Newtons) on body 2 required to satisfy the joint constraint
        virtual Vector3 getReactionForce(decimal timeStep) const=0;

        /// Return the torque (in Newtons * meters) on body 2 required to satisfy the joint constraint
        virtual Vector3 getReactionTorque(decimal timeStep) const=0;

        /// Return true if the collision between the two bodies of the joint is enabled
        bool isCollisionEnabled() const;

        /// Return the entity id of the joint
        Entity getEntity() const;

        /// Return a string representation
        virtual std::string to_string() const=0;

        // -------------------- Friendship -------------------- //

        friend class PhysicsWorld;
        friend class Island;
        friend class ConstraintSolverSystem;
};

// Return the entity id of the joint
/**
 * @return The entity of the joint
 */
RP3D_FORCE_INLINE Entity Joint::getEntity() const {
    return mEntity;
}

}

#endif
