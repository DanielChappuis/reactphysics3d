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

#ifndef TEST_COLLISION_WORLD_H
#define TEST_COLLISION_WORLD_H

// Libraries
#include "reactphysics3d.h"
#include "Test.h"

/// Reactphysics3D namespace
namespace reactphysics3d {

// Enumeration for categories
enum CollisionCategory {
    CATEGORY_1 = 0x0001,
    CATEGORY_2 = 0x0002,
    CATEGORY_3 = 0x0004
};

// Contact point collision data
struct CollisionPointData {

	Vector3 localPointBody1;
	Vector3 localPointBody2;
	decimal penetrationDepth;

	CollisionPointData(const Vector3& point1, const Vector3& point2, decimal penDepth) {
		localPointBody1 = point1;
		localPointBody2 = point2;
		penetrationDepth = penDepth;
	}

	bool isContactPointSimilarTo(const Vector3& pointBody1, const Vector3& pointBody2, decimal penDepth, decimal epsilon = 0.001) {

		return approxEqual(pointBody1, localPointBody1, epsilon) &&
			   approxEqual(pointBody2, localPointBody2, epsilon) &&
			   approxEqual(penetrationDepth, penDepth, epsilon);
	}
};

// Contact manifold collision data
struct CollisionManifoldData {

	std::vector<CollisionPointData> contactPoints;

	int getNbContactPoints() const {
		return contactPoints.size();
	}

	bool hasContactPointSimilarTo(const Vector3& localPointBody1, const Vector3& localPointBody2, decimal penetrationDepth, decimal epsilon = 0.001) {

		std::vector<CollisionPointData>::iterator it;
		for (it = contactPoints.begin(); it != contactPoints.end(); ++it) {

			if (it->isContactPointSimilarTo(localPointBody1, localPointBody2, penetrationDepth)) {
				return true;
			}
		}

		return false;
	}

};

// Collision data between two proxy shapes
struct CollisionData {

	std::pair<const ProxyShape*, const ProxyShape*> proxyShapes;
	std::pair<CollisionBody*, CollisionBody*> bodies;
	std::vector<CollisionManifoldData> contactManifolds;

	int getNbContactManifolds() const {
		return contactManifolds.size();
	}

	int getTotalNbContactPoints() const {
		
		int nbPoints = 0;

		std::vector<CollisionManifoldData>::const_iterator it;
		for (it = contactManifolds.begin(); it != contactManifolds.end(); ++it) {

			nbPoints += it->getNbContactPoints();
		}

		return nbPoints;
	}

	bool hasContactPointSimilarTo(const Vector3& localPointBody1, const Vector3& localPointBody2, decimal penetrationDepth, decimal epsilon = 0.001) {

		std::vector<CollisionManifoldData>::iterator it;
		for (it = contactManifolds.begin(); it != contactManifolds.end(); ++it) {

			if (it->hasContactPointSimilarTo(localPointBody1, localPointBody2, penetrationDepth)) {
				return true;
			}
		}

		return false;
	}

};

// Class
class WorldCollisionCallback : public CollisionCallback
{
	private:

		std::vector<std::pair<const ProxyShape*, const ProxyShape*>> mCollisionData;

		std::pair<const ProxyShape*, const ProxyShape*> getCollisionKeyPair(std::pair<const ProxyShape*, const ProxyShape*> pair) const {
			
			if (pair.first > pair.second) {
				return std::make_pair(pair.second, pair.first);
			}

			return pair;
		}

    public:

        WorldCollisionCallback()
        {
            reset();
        }

        void reset()
        {
			mCollisionData.clear();
        }

		bool hasContacts() const {
			return mCollisionData.size() > 0;
		}

		bool areProxyShapesColliding(const ProxyShape* proxyShape1, const ProxyShape* proxyShape2) {
			return std::find(mCollisionData.begin(), mCollisionData.end(), getCollisionKeyPair(std::make_pair(proxyShape1, proxyShape2))) != mCollisionData.end();
		}

        // This method will be called for each contact
        virtual void notifyContact(const CollisionCallbackInfo& collisionCallbackInfo) override {

			CollisionData collisionData;
			collisionData.bodies = std::make_pair(collisionCallbackInfo.body1, collisionCallbackInfo.body2);
			collisionData.proxyShapes = std::make_pair(collisionCallbackInfo.proxyShape1, collisionCallbackInfo.proxyShape2);

			ContactManifoldListElement* element = collisionCallbackInfo.contactManifoldElements;
			while (element != nullptr) {

				ContactManifold* contactManifold = element->getContactManifold();

				CollisionManifoldData collisionManifold;

				ContactPoint* contactPoint = contactManifold->getContactPoints();
				while (contactPoint != nullptr) {

                    CollisionPointData collisionPoint(contactPoint->getLocalPointOnShape1(), contactPoint->getLocalPointOnShape2(), contactPoint->getPenetrationDepth());
					collisionManifold.contactPoints.push_back(collisionPoint);

					contactPoint = contactPoint->getNext();
				}

				collisionData.contactManifolds.push_back(collisionManifold);

				element = element->getNext();
			}
        }
};

/// Overlap callback
class WorldOverlapCallback : public OverlapCallback {

	private:

		CollisionBody* mOverlapBody;

	public:

		/// Destructor
		virtual ~WorldOverlapCallback() {
			reset();
		}

		/// This method will be called for each reported overlapping bodies
		virtual void notifyOverlap(CollisionBody* collisionBody) override {

		}

		void reset() {
			mOverlapBody = nullptr;
		}

		bool hasOverlap() const {
			return mOverlapBody != nullptr;
		}

		CollisionBody* getOverlapBody() {
			return mOverlapBody;
		}
};

// Class TestCollisionWorld
/**
 * Unit test for the CollisionWorld class.
 */
class TestCollisionWorld : public Test {

    private :

        // ---------- Atributes ---------- //

        // Physics world
        CollisionWorld* mWorld;

        // Bodies
        CollisionBody* mBoxBody1;
		CollisionBody* mBoxBody2;
        CollisionBody* mSphereBody1;
        CollisionBody* mSphereBody2;

        // Collision shapes
        BoxShape* mBoxShape1;
		BoxShape* mBoxShape2;
        SphereShape* mSphereShape1;
		SphereShape* mSphereShape2;

        // Proxy shapes
        ProxyShape* mBoxProxyShape1;
		ProxyShape* mBoxProxyShape2;
        ProxyShape* mSphereProxyShape1;
        ProxyShape* mSphereProxyShape2;

        // Collision callback
        WorldCollisionCallback mCollisionCallback;

		// Overlap callback
		WorldOverlapCallback mOverlapCallback;

    public :

        // ---------- Methods ---------- //

        /// Constructor
        TestCollisionWorld(const std::string& name) : Test(name) {

            // Create the collision world
            mWorld = new CollisionWorld();

            // ---------- Boxes ---------- //
            Transform boxTransform1(Vector3(-20, 20, 0), Quaternion::identity());
            mBoxBody1 = mWorld->createCollisionBody(boxTransform1);
            mBoxShape1 = new BoxShape(Vector3(3, 3, 3));
            mBoxProxyShape1 = mBoxBody1->addCollisionShape(mBoxShape1, Transform::identity());

			Transform boxTransform2(Vector3(-10, 20, 0), Quaternion::identity());
			mBoxBody2 = mWorld->createCollisionBody(boxTransform2);
			mBoxShape2 = new BoxShape(Vector3(2, 2, 2));
			mBoxProxyShape2 = mBoxBody2->addCollisionShape(mBoxShape1, Transform::identity());

			// ---------- Spheres ---------- //
            mSphereShape1 = new SphereShape(3.0);
            Transform sphereTransform1(Vector3(10, 20, 0), Quaternion::identity());
            mSphereBody1 = mWorld->createCollisionBody(sphereTransform1);
            mSphereProxyShape1 = mSphereBody1->addCollisionShape(mSphereShape1, Transform::identity());

			mSphereShape2 = new SphereShape(5.0);
			Transform sphereTransform2(Vector3(20, 20, 0), Quaternion::identity());
			mSphereBody2 = mWorld->createCollisionBody(sphereTransform2);
			mSphereProxyShape2 = mSphereBody2->addCollisionShape(mSphereShape2, Transform::identity());
        }

        /// Destructor
        virtual ~TestCollisionWorld() {

            delete mBoxShape1;
            delete mBoxShape2;

			delete mSphereShape1;
			delete mSphereShape2;

			delete mWorld;
        }

        /// Run the tests
        void run() {

			testNoCollisions();
			testNoOverlap();
			testNoAABBOverlap();

			testAABBOverlap();

			testSphereVsSphereCollision();
			testSphereVsBoxCollision();

			testMultipleCollisions();
        }

		void testNoCollisions() {

			// All the shapes of the world are not touching when they are created.
			// Here we test that at the beginning, there is no collision at all.

			// ---------- Global test ---------- //

			mCollisionCallback.reset();
			mWorld->testCollision(&mCollisionCallback);
			test(!mCollisionCallback.hasContacts());

			// ---------- Single body test ---------- //

			mCollisionCallback.reset();
			mWorld->testCollision(mBoxBody1, &mCollisionCallback);
			test(!mCollisionCallback.hasContacts());

			mCollisionCallback.reset();
			mWorld->testCollision(mBoxBody2, &mCollisionCallback);
			test(!mCollisionCallback.hasContacts());

			mCollisionCallback.reset();
			mWorld->testCollision(mSphereBody1, &mCollisionCallback);
			test(!mCollisionCallback.hasContacts());

			mCollisionCallback.reset();
			mWorld->testCollision(mSphereBody2, &mCollisionCallback);
			test(!mCollisionCallback.hasContacts());

			// Two bodies test

			mCollisionCallback.reset();
			mWorld->testCollision(mBoxBody1, mBoxBody2, &mCollisionCallback);
			test(!mCollisionCallback.hasContacts());

			mCollisionCallback.reset();
			mWorld->testCollision(mSphereBody1, mSphereBody2, &mCollisionCallback);
			test(!mCollisionCallback.hasContacts());

			mCollisionCallback.reset();
			mWorld->testCollision(mBoxBody1, mSphereBody1, &mCollisionCallback);
			test(!mCollisionCallback.hasContacts());

			mCollisionCallback.reset();
			mWorld->testCollision(mBoxBody1, mSphereBody2, &mCollisionCallback);
			test(!mCollisionCallback.hasContacts());

			mCollisionCallback.reset();
			mWorld->testCollision(mBoxBody2, mSphereBody1, &mCollisionCallback);
			test(!mCollisionCallback.hasContacts());

			mCollisionCallback.reset();
			mWorld->testCollision(mBoxBody2, mSphereBody2, &mCollisionCallback);
			test(!mCollisionCallback.hasContacts());

		}

		void testNoOverlap() {

			// All the shapes of the world are not touching when they are created.
			// Here we test that at the beginning, there is no overlap at all.

			// ---------- Single body test ---------- //

			mOverlapCallback.reset();
			mWorld->testOverlap(mBoxBody1, &mOverlapCallback);
			test(!mOverlapCallback.hasOverlap());

			mOverlapCallback.reset();
			mWorld->testOverlap(mBoxBody2, &mOverlapCallback);
			test(!mOverlapCallback.hasOverlap());

			mOverlapCallback.reset();
			mWorld->testOverlap(mSphereBody1, &mOverlapCallback);
			test(!mOverlapCallback.hasOverlap());

			mOverlapCallback.reset();
			mWorld->testOverlap(mSphereBody2, &mOverlapCallback);
			test(!mOverlapCallback.hasOverlap());

			// Two bodies test

			test(!mWorld->testOverlap(mBoxBody1, mBoxBody2));
			test(!mWorld->testOverlap(mSphereBody1, mSphereBody2));
			test(!mWorld->testOverlap(mBoxBody1, mSphereBody1));
			test(!mWorld->testOverlap(mBoxBody1, mSphereBody2));
			test(!mWorld->testOverlap(mBoxBody2, mSphereBody1));
			test(!mWorld->testOverlap(mBoxBody2, mSphereBody2));
		}

		void testNoAABBOverlap() {

			// All the shapes of the world are not touching when they are created.
			// Here we test that at the beginning, there is no AABB overlap at all.

			// Two bodies test

			test(!mWorld->testAABBOverlap(mBoxBody1, mBoxBody2));
			test(!mWorld->testAABBOverlap(mSphereBody1, mSphereBody2));
			test(!mWorld->testAABBOverlap(mBoxBody1, mSphereBody1));
			test(!mWorld->testAABBOverlap(mBoxBody1, mSphereBody2));
			test(!mWorld->testAABBOverlap(mBoxBody2, mSphereBody1));
			test(!mWorld->testAABBOverlap(mBoxBody2, mSphereBody2));
		}

		void testAABBOverlap() {

			// TODO : Test the CollisionWorld::testAABBOverlap() method here
		}

		void testSphereVsSphereCollision() {



			// Move sphere 1 to collide with sphere 2
			mSphereBody1->setTransform(Transform(Vector3(30, 15, 10), Quaternion::identity()));

		}

        void testSphereVsBoxCollision() {

            // Move sphere 1 to collide with box
            mSphereBody1->setTransform(Transform(Vector3(10, 5, 0), Quaternion::identity()));

            // --------- Test collision with inactive bodies --------- //

            mCollisionCallback.reset();
            mBoxBody1->setIsActive(false);
            mSphereBody1->setIsActive(false);
            mSphereBody2->setIsActive(false);
            mWorld->testCollision(&mCollisionCallback);
          

            mBoxBody1->setIsActive(true);
            mSphereBody1->setIsActive(true);
            mSphereBody2->setIsActive(true);

            // --------- Test collision with collision filtering -------- //

            //mBoxProxyShape->setCollideWithMaskBits(CATEGORY_1 | CATEGORY_3);
            //mSphere1ProxyShape->setCollideWithMaskBits(CATEGORY_1 | CATEGORY_2);
            //mSphere2ProxyShape->setCollideWithMaskBits(CATEGORY_1);

            //mCollisionCallback.reset();
            //mWorld->testCollision(&mCollisionCallback);
            //test(mCollisionCallback.boxCollideWithSphere1);
            //test(!mCollisionCallback.sphere1CollideWithSphere2);

            //// Move sphere 1 to collide with sphere 2
            //mSphere1Body->setTransform(Transform(Vector3(30, 15, 10), Quaternion::identity()));

            //mCollisionCallback.reset();
            //mWorld->testCollision(&mCollisionCallback);
            //test(!mCollisionCallback.boxCollideWithSphere1);
            //test(mCollisionCallback.sphere1CollideWithSphere2);

            //mBoxProxyShape->setCollideWithMaskBits(CATEGORY_2);
            //mSphere1ProxyShape->setCollideWithMaskBits(CATEGORY_2);
            //mSphere2ProxyShape->setCollideWithMaskBits(CATEGORY_3);

            //mCollisionCallback.reset();
            //mWorld->testCollision(&mCollisionCallback);
            //test(!mCollisionCallback.boxCollideWithSphere1);
            //test(!mCollisionCallback.sphere1CollideWithSphere2);

            //// Move sphere 1 to collide with box
            //mSphere1Body->setTransform(Transform(Vector3(10, 5, 0), Quaternion::identity()));

            //mBoxProxyShape->setCollideWithMaskBits(0xFFFF);
            //mSphere1ProxyShape->setCollideWithMaskBits(0xFFFF);
            //mSphere2ProxyShape->setCollideWithMaskBits(0xFFFF);
        }

		void testMultipleCollisions() {

			// TODO : Test collisions without categories set

			// TODO : Test colliisons with categories set

			// Assign collision categories to proxy shapes
			//mBoxProxyShape->setCollisionCategoryBits(CATEGORY_1);
			//mSphere1ProxyShape->setCollisionCategoryBits(CATEGORY_1);
			//mSphere2ProxyShape->setCollisionCategoryBits(CATEGORY_2);
		}
 };

}

#endif
