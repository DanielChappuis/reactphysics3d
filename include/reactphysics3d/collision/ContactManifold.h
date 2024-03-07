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

#ifndef REACTPHYSICS3D_CONTACT_MANIFOLD_H
#define	REACTPHYSICS3D_CONTACT_MANIFOLD_H

// Libraries
#include <reactphysics3d/collision/Collider.h>

/// ReactPhysics3D namespace
namespace reactphysics3d {

// Class declarations
class ContactManifold;
struct ContactManifoldInfo;
struct ContactPointInfo;
class Body;
class ContactPoint;
class DefaultPoolAllocator;

// Class ContactManifold
/**
 * This class represents a set of contact points between two bodies that
 * all have a similar contact normal direction. Usually, there is a single
 * contact manifold when two convex shapes are in contact. However, when
 * a convex shape collides with a concave shape, there might be several
 * contact manifolds with different normal directions.
 * The contact manifold is implemented in a way to cache the contact
 * points among the frames for better stability (warm starting of the
 * contact solver)
 */
class ContactManifold {

    public:

        // -------------------- Constants -------------------- //

        /// Maximum number of contact points in a reduced contact manifold
        static constexpr int MAX_CONTACT_POINTS_IN_MANIFOLD = 4;

        // -------------------- Attributes -------------------- //

        /// Index of the first contact point of the manifold in the array of contact points
        uint32 contactPointsIndex;

        /// Entity of the first body in contact
        Entity bodyEntity1;

        /// Entity of the second body in contact
        Entity bodyEntity2;

        /// Entity of the first collider in contact
        Entity colliderEntity1;

        /// Entity of the second collider in contact
        Entity colliderEntity2;

        /// Number of contacts in the cache
        uint8 nbContactPoints;

        /// First friction vector of the contact manifold
        Vector3 frictionVector1;

        /// Second friction vector of the contact manifold
        Vector3 frictionVector2;

        /// First friction constraint accumulated impulse
        decimal frictionImpulse1;

        /// Second friction constraint accumulated impulse
        decimal frictionImpulse2;

        /// Twist friction constraint accumulated impulse
        decimal frictionTwistImpulse;

        /// True if the contact manifold has already been added into an island
        bool isAlreadyInIsland;

    public:

        // -------------------- Methods -------------------- //

        /// Constructor
        ContactManifold(Entity bodyEntity1, Entity bodyEntity2, Entity colliderEntity1, Entity colliderEntity2,
                        uint32 contactPointsIndex, uint8 nbContactPoints);

        // -------------------- Friendship -------------------- //

        friend class PhysicsWorld;
        friend class Island;
        friend class Body;
        friend class ContactManifoldSet;
        friend class ContactSolverSystem;
        friend class CollisionDetectionSystem;
};

}

#endif

