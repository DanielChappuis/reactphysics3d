/********************************************************************************
* ReactPhysics3D physics library, http://www.reactphysics3d.com                 *
* Copyright (c) 2010-2018 Daniel Chappuis                                       *
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
#include "collision/ProxyShape.h"

/// ReactPhysics3D namespace
namespace reactphysics3d {

// Class declarations
class ContactManifold;
class ContactManifoldInfo;
struct ContactPointInfo;
class CollisionBody;
class ContactPoint;
class PoolAllocator;

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

    // TODO : Check if we can use public fields in this class (maybe this class is used by users directly)
    private:

        // -------------------- Constants -------------------- //

        /// Maximum number of contact points in a reduced contact manifold
        const int MAX_CONTACT_POINTS_IN_MANIFOLD = 4;

        // -------------------- Attributes -------------------- //

        // TODO : For each of the attributes, check if we need to keep it or not

        /// Index of the first contact point of the manifold in the list of contact points
        uint mContactPointsIndex;

        /// Entity of the first body in contact
        Entity bodyEntity1;

        /// Entity of the second body in contact
        Entity bodyEntity2;

        /// Entity of the first proxy-shape in contact
        Entity proxyShapeEntity1;

        /// Entity of the second proxy-shape in contact
        Entity proxyShapeEntity2;

        /// Number of contacts in the cache
        int8 mNbContactPoints;

        /// First friction vector of the contact manifold
        Vector3 mFrictionVector1;

        /// Second friction vector of the contact manifold
        Vector3 mFrictionVector2;

        /// First friction constraint accumulated impulse
        decimal mFrictionImpulse1;

        /// Second friction constraint accumulated impulse
        decimal mFrictionImpulse2;

        /// Twist friction constraint accumulated impulse
        decimal mFrictionTwistImpulse;

        /// Accumulated rolling resistance impulse
        Vector3 mRollingResistanceImpulse;

        /// True if the contact manifold has already been added into an island
        bool mIsAlreadyInIsland;

        // -------------------- Methods -------------------- //

        /// Return true if the contact manifold has already been added into an island
        bool isAlreadyInIsland() const;

        /// set the first friction vector at the center of the contact manifold
        void setFrictionVector1(const Vector3& mFrictionVector1);

        /// set the second friction vector at the center of the contact manifold
        void setFrictionVector2(const Vector3& mFrictionVector2);

        /// Set the first friction accumulated impulse
        void setFrictionImpulse1(decimal frictionImpulse1);

        /// Set the second friction accumulated impulse
        void setFrictionImpulse2(decimal frictionImpulse2);

        /// Set the friction twist accumulated impulse
        void setFrictionTwistImpulse(decimal frictionTwistImpulse);

        /// Set the accumulated rolling resistance impulse
        void setRollingResistanceImpulse(const Vector3& rollingResistanceImpulse);

        /// Return the first friction vector at the center of the contact manifold
        const Vector3& getFrictionVector1() const;

        /// Return the second friction vector at the center of the contact manifold
        const Vector3& getFrictionVector2() const;

        /// Return the first friction accumulated impulse
        decimal getFrictionImpulse1() const;

        /// Return the second friction accumulated impulse
        decimal getFrictionImpulse2() const;

        /// Return the friction twist accumulated impulse
        decimal getFrictionTwistImpulse() const;

    public:

        // -------------------- Methods -------------------- //

        /// Constructor
        ContactManifold(Entity bodyEntity1, Entity bodyEntity2, Entity proxyShapeEntity1, Entity proxyShapeEntity2,
                        uint contactPointsIndex, int8 nbContactPoints);

        /// Destructor
        ~ContactManifold();

        /// Copy-constructor
        ContactManifold(const ContactManifold& contactManifold) = default;

        /// Assignment operator
        ContactManifold& operator=(const ContactManifold& contactManifold) = default;

        /// Return the number of contact points in the manifold
        int8 getNbContactPoints() const;

        /// Return a pointer to the first contact point of the manifold
        ContactPoint* getContactPoints() const;

        // -------------------- Friendship -------------------- //

        friend class DynamicsWorld;
        friend class Island;
        friend class CollisionBody;
        friend class ContactManifoldSet;
        friend class ContactSolver;
        friend class CollisionDetection;
};

// Return the number of contact points in the manifold
inline int8 ContactManifold::getNbContactPoints() const {
    return mNbContactPoints;
}

// Return the first friction vector at the center of the contact manifold
inline const Vector3& ContactManifold::getFrictionVector1() const {
    return mFrictionVector1;
}

// set the first friction vector at the center of the contact manifold
inline void ContactManifold::setFrictionVector1(const Vector3& frictionVector1) {
    mFrictionVector1 = frictionVector1;
}

// Return the second friction vector at the center of the contact manifold
inline const Vector3& ContactManifold::getFrictionVector2() const {
    return mFrictionVector2;
}

// set the second friction vector at the center of the contact manifold
inline void ContactManifold::setFrictionVector2(const Vector3& frictionVector2) {
    mFrictionVector2 = frictionVector2;
}

// Return the first friction accumulated impulse
inline decimal ContactManifold::getFrictionImpulse1() const {
    return mFrictionImpulse1;
}

// Set the first friction accumulated impulse
inline void ContactManifold::setFrictionImpulse1(decimal frictionImpulse1) {
    mFrictionImpulse1 = frictionImpulse1;
}

// Return the second friction accumulated impulse
inline decimal ContactManifold::getFrictionImpulse2() const {
    return mFrictionImpulse2;
}

// Set the second friction accumulated impulse
inline void ContactManifold::setFrictionImpulse2(decimal frictionImpulse2) {
    mFrictionImpulse2 = frictionImpulse2;
}

// Return the friction twist accumulated impulse
inline decimal ContactManifold::getFrictionTwistImpulse() const {
    return mFrictionTwistImpulse;
}

// Set the friction twist accumulated impulse
inline void ContactManifold::setFrictionTwistImpulse(decimal frictionTwistImpulse) {
    mFrictionTwistImpulse = frictionTwistImpulse;
}

// Set the accumulated rolling resistance impulse
inline void ContactManifold::setRollingResistanceImpulse(const Vector3& rollingResistanceImpulse) {
    mRollingResistanceImpulse = rollingResistanceImpulse;
}

// Return true if the contact manifold has already been added into an island
inline bool ContactManifold::isAlreadyInIsland() const {
    return mIsAlreadyInIsland;
}

}

#endif

