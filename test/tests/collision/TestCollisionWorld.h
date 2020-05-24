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
#include <reactphysics3d/reactphysics3d.h>
#include "Test.h"
#include <reactphysics3d/constraint/ContactPoint.h>
#include <reactphysics3d/collision/ContactManifold.h>
#include <map>
#include <vector>

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

	bool isContactPointSimilarTo(const Vector3& pointBody1, const Vector3& pointBody2, decimal penDepth, decimal epsilon = 0.001) const {

		return approxEqual(pointBody1, localPointBody1, epsilon) &&
			   approxEqual(pointBody2, localPointBody2, epsilon) &&
			   approxEqual(penetrationDepth, penDepth, epsilon);
	}
};

// Contact pair collision data
struct ContactPairData {

	std::vector<CollisionPointData> contactPoints;

    uint getNbContactPoints() const {
		return contactPoints.size();
	}

	bool hasContactPointSimilarTo(const Vector3& localPointBody1, const Vector3& localPointBody2, decimal penetrationDepth, decimal epsilon = 0.001) const {

		std::vector<CollisionPointData>::const_iterator it;
		for (it = contactPoints.cbegin(); it != contactPoints.cend(); ++it) {

			Vector3 vec = it->localPointBody1;
			if (it->isContactPointSimilarTo(localPointBody1, localPointBody2, penetrationDepth)) {
				return true;
			}
		}

		return false;
	}

};

// Collision data between two colliders
struct CollisionData {

    std::pair<const Collider*, const Collider*> colliders;
	std::pair<CollisionBody*, CollisionBody*> bodies;
    std::vector<ContactPairData> contactPairs;

    int getNbContactPairs() const {
        return contactPairs.size();
	}

	int getTotalNbContactPoints() const {
		
		int nbPoints = 0;

        std::vector<ContactPairData>::const_iterator it;
        for (it = contactPairs.begin(); it != contactPairs.end(); ++it) {

			nbPoints += it->getNbContactPoints();
		}

		return nbPoints;
	}

	const CollisionBody* getBody1() const {
		return bodies.first;
	}

	const CollisionBody* getBody2() const {
		return bodies.second;
	}

	bool hasContactPointSimilarTo(const Vector3& localPointBody1, const Vector3& localPointBody2, decimal penetrationDepth, decimal epsilon = 0.001) const {

        std::vector<ContactPairData>::const_iterator it;
        for (it = contactPairs.cbegin(); it != contactPairs.cend(); ++it) {

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

		std::map<std::pair<const Collider*, const Collider*>, CollisionData> mCollisionDatas;

		std::pair<const Collider*, const Collider*> getCollisionKeyPair(std::pair<const Collider*, const Collider*> pair) const {
			
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
			mCollisionDatas.clear();
        }

		bool hasContacts() const {
			return mCollisionDatas.size() > 0;
		}

        bool areCollidersColliding(const Collider* collider1, const Collider* collider2) {
            return mCollisionDatas.find(getCollisionKeyPair(std::make_pair(collider1, collider2))) != mCollisionDatas.end();
		}

        const CollisionData* getCollisionData(const Collider* collider1, const Collider* collider2) const {
            std::map<std::pair<const Collider*, const Collider*>, CollisionData>::const_iterator it = mCollisionDatas.find(getCollisionKeyPair(std::make_pair(collider1, collider2)));
			if (it != mCollisionDatas.end()) {
					return &(it->second);
			}
			else {
				return nullptr;
			}
		}

        // This method is called when some contacts occur
        virtual void onContact(const CallbackData& callbackData) override {

            CollisionData collisionData;

            // For each contact pair
            for (uint p=0; p < callbackData.getNbContactPairs(); p++) {

                ContactPairData contactPairData;
                ContactPair contactPair = callbackData.getContactPair(p);

                collisionData.bodies = std::make_pair(contactPair.getBody1(), contactPair.getBody2());
                collisionData.colliders = std::make_pair(contactPair.getCollider1(), contactPair.getCollider2());

                // For each contact point
                for (uint c=0; c < contactPair.getNbContactPoints(); c++) {

                    ContactPoint contactPoint = contactPair.getContactPoint(c);

                    CollisionPointData collisionPoint(contactPoint.getLocalPointOnCollider1(), contactPoint.getLocalPointOnCollider2(), contactPoint.getPenetrationDepth());
                    contactPairData.contactPoints.push_back(collisionPoint);
                }

                collisionData.contactPairs.push_back(contactPairData);
            }

            mCollisionDatas.insert(std::make_pair(getCollisionKeyPair(collisionData.colliders), collisionData));
        }
};

/// Overlap callback
class WorldOverlapCallback : public OverlapCallback {

	private:

        std::vector<std::pair<CollisionBody*, CollisionBody*>> mOverlapBodies;

	public:

		/// Destructor
        virtual ~WorldOverlapCallback() override {
			reset();
		}

		/// This method will be called for each reported overlapping bodies
        virtual void onOverlap(CallbackData& callbackData) override {

            // For each overlapping pair
            for (uint i=0; i < callbackData.getNbOverlappingPairs(); i++) {

                OverlapPair overlapPair = callbackData.getOverlappingPair(i);
                mOverlapBodies.push_back(std::make_pair(overlapPair.getBody1(), overlapPair.getBody2()));
            }
		}

		void reset() {
			mOverlapBodies.clear();
		}

        bool hasOverlapWithBody(CollisionBody* collisionBody) const {

            for (uint i=0; i < mOverlapBodies.size(); i++) {

                if (mOverlapBodies[i].first == collisionBody || mOverlapBodies[i].second == collisionBody) {
                    return true;
                }
            }

            return false;
		}
};

// Class TestCollisionWorld
/**
 * Unit test for the CollisionWorld class.
 */
class TestCollisionWorld : public Test {

    private :

        // ---------- Atributes ---------- //

        PhysicsCommon mPhysicsCommon;

        // Physics world
        PhysicsWorld* mWorld;

        // Bodies
        CollisionBody* mBoxBody1;
		CollisionBody* mBoxBody2;
        CollisionBody* mSphereBody1;
        CollisionBody* mSphereBody2;
		CollisionBody* mCapsuleBody1;
		CollisionBody* mCapsuleBody2;
		CollisionBody* mConvexMeshBody1;
		CollisionBody* mConvexMeshBody2;
        CollisionBody* mConcaveMeshBody;

        // Collision shapes
        BoxShape* mBoxShape1;
		BoxShape* mBoxShape2;
        SphereShape* mSphereShape1;
		SphereShape* mSphereShape2;
		CapsuleShape* mCapsuleShape1;
		CapsuleShape* mCapsuleShape2;
		ConvexMeshShape* mConvexMeshShape1;
		ConvexMeshShape* mConvexMeshShape2;
        ConcaveMeshShape* mConcaveMeshShape;

        // Colliders
        Collider* mBoxCollider1;
        Collider* mBoxCollider2;
        Collider* mSphereCollider1;
        Collider* mSphereCollider2;
        Collider* mCapsuleCollider1;
        Collider* mCapsuleCollider2;
        Collider* mConvexMeshCollider1;
        Collider* mConvexMeshCollider2;
        Collider* mConcaveMeshCollider;

		PolygonVertexArray* mConvexMesh1PolygonVertexArray;
		PolygonVertexArray* mConvexMesh2PolygonVertexArray;
		PolyhedronMesh* mConvexMesh1PolyhedronMesh;
		PolyhedronMesh* mConvexMesh2PolyhedronMesh;
		PolygonVertexArray::PolygonFace* mConvexMeshPolygonFaces;

        TriangleVertexArray* mConcaveMeshTriangleVertexArray;
		Vector3 mConvexMesh1CubeVertices[8];
		Vector3 mConvexMesh2CubeVertices[8];
		int mConvexMeshCubeIndices[24];

        Vector3 mConcaveMeshPlaneVertices[36];
		int mConcaveMeshPlaneIndices[25 * 2 * 3];
        TriangleMesh* mConcaveTriangleMesh;

        // Collision callback
        WorldCollisionCallback mCollisionCallback;

		// Overlap callback
		WorldOverlapCallback mOverlapCallback;

    public :

        // ---------- Methods ---------- //

        /// Constructor
        TestCollisionWorld(const std::string& name) : Test(name) {

            // Create the collision world
            mWorld = mPhysicsCommon.createPhysicsWorld();

            // ---------- Boxes ---------- //
            Transform boxTransform1(Vector3(-20, 20, 0), Quaternion::identity());
            mBoxBody1 = mWorld->createCollisionBody(boxTransform1);
            mBoxShape1 = mPhysicsCommon.createBoxShape(Vector3(3, 3, 3));
            mBoxCollider1 = mBoxBody1->addCollider(mBoxShape1, Transform::identity());

			Transform boxTransform2(Vector3(-10, 20, 0), Quaternion::identity());
			mBoxBody2 = mWorld->createCollisionBody(boxTransform2);
            mBoxShape2 = mPhysicsCommon.createBoxShape(Vector3(4, 2, 8));
            mBoxCollider2 = mBoxBody2->addCollider(mBoxShape2, Transform::identity());

			// ---------- Spheres ---------- //
            mSphereShape1 = mPhysicsCommon.createSphereShape(3.0);
            Transform sphereTransform1(Vector3(10, 20, 0), Quaternion::identity());
            mSphereBody1 = mWorld->createCollisionBody(sphereTransform1);
            mSphereCollider1 = mSphereBody1->addCollider(mSphereShape1, Transform::identity());

            mSphereShape2 = mPhysicsCommon.createSphereShape(5.0);
			Transform sphereTransform2(Vector3(20, 20, 0), Quaternion::identity());
			mSphereBody2 = mWorld->createCollisionBody(sphereTransform2);
            mSphereCollider2 = mSphereBody2->addCollider(mSphereShape2, Transform::identity());

			// ---------- Capsules ---------- //
            mCapsuleShape1 = mPhysicsCommon.createCapsuleShape(2, 6);
            Transform capsuleTransform1(Vector3(-10, 0, 0), Quaternion::identity());
            mCapsuleBody1 = mWorld->createCollisionBody(capsuleTransform1);
            mCapsuleCollider1 = mCapsuleBody1->addCollider(mCapsuleShape1, Transform::identity());

            mCapsuleShape2 = mPhysicsCommon.createCapsuleShape(3, 4);
			Transform capsuleTransform2(Vector3(-20, 0, 0), Quaternion::identity());
			mCapsuleBody2 = mWorld->createCollisionBody(capsuleTransform2);
            mCapsuleCollider2 = mCapsuleBody2->addCollider(mCapsuleShape2, Transform::identity());

			// ---------- Convex Meshes ---------- //
			mConvexMesh1CubeVertices[0] = Vector3(-3, -3, 3);
			mConvexMesh1CubeVertices[1] = Vector3(3, -3, 3);
			mConvexMesh1CubeVertices[2] = Vector3(3, -3, -3);
			mConvexMesh1CubeVertices[3] = Vector3(-3, -3, -3);
			mConvexMesh1CubeVertices[4] = Vector3(-3, 3, 3);
			mConvexMesh1CubeVertices[5] = Vector3(3, 3, 3);
			mConvexMesh1CubeVertices[6] = Vector3(3, 3, -3);
			mConvexMesh1CubeVertices[7] = Vector3(-3, 3, -3);

			mConvexMeshCubeIndices[0] = 0; mConvexMeshCubeIndices[1] = 3; mConvexMeshCubeIndices[2] = 2; mConvexMeshCubeIndices[3] = 1;
			mConvexMeshCubeIndices[4] = 4; mConvexMeshCubeIndices[5] = 5; mConvexMeshCubeIndices[6] = 6; mConvexMeshCubeIndices[7] = 7;
			mConvexMeshCubeIndices[8] = 0; mConvexMeshCubeIndices[9] = 1; mConvexMeshCubeIndices[10] = 5; mConvexMeshCubeIndices[11] = 4;
			mConvexMeshCubeIndices[12] = 1; mConvexMeshCubeIndices[13] = 2; mConvexMeshCubeIndices[14] = 6; mConvexMeshCubeIndices[15] = 5;
			mConvexMeshCubeIndices[16] = 2; mConvexMeshCubeIndices[17] = 3; mConvexMeshCubeIndices[18] = 7; mConvexMeshCubeIndices[19] = 6;
			mConvexMeshCubeIndices[20] = 0; mConvexMeshCubeIndices[21] = 4; mConvexMeshCubeIndices[22] = 7; mConvexMeshCubeIndices[23] = 3;

			mConvexMeshPolygonFaces = new rp3d::PolygonVertexArray::PolygonFace[6];
			rp3d::PolygonVertexArray::PolygonFace* face = mConvexMeshPolygonFaces;
			for (int f = 0; f < 6; f++) {
				face->indexBase = f * 4;
				face->nbVertices = 4;
				face++;
			}
			mConvexMesh1PolygonVertexArray = new rp3d::PolygonVertexArray(8, &(mConvexMesh1CubeVertices[0]), sizeof(Vector3),
					&(mConvexMeshCubeIndices[0]), sizeof(int), 6, mConvexMeshPolygonFaces,
					rp3d::PolygonVertexArray::VertexDataType::VERTEX_FLOAT_TYPE,
					rp3d::PolygonVertexArray::IndexDataType::INDEX_INTEGER_TYPE);
            mConvexMesh1PolyhedronMesh = mPhysicsCommon.createPolyhedronMesh(mConvexMesh1PolygonVertexArray);
            mConvexMeshShape1 = mPhysicsCommon.createConvexMeshShape(mConvexMesh1PolyhedronMesh);
            Transform convexMeshTransform1(Vector3(10, 0, 0), Quaternion::identity());
            mConvexMeshBody1 = mWorld->createCollisionBody(convexMeshTransform1);
            mConvexMeshCollider1 = mConvexMeshBody1->addCollider(mConvexMeshShape1, Transform::identity());

			mConvexMesh2CubeVertices[0] = Vector3(-4, -2, 8);
			mConvexMesh2CubeVertices[1] = Vector3(4, -2, 8);
			mConvexMesh2CubeVertices[2] = Vector3(4, -2, -8);
			mConvexMesh2CubeVertices[3] = Vector3(-4, -2, -8);
			mConvexMesh2CubeVertices[4] = Vector3(-4, 2, 8);
			mConvexMesh2CubeVertices[5] = Vector3(4, 2, 8);
			mConvexMesh2CubeVertices[6] = Vector3(4, 2, -8);
			mConvexMesh2CubeVertices[7] = Vector3(-4, 2, -8);

			mConvexMesh2PolygonVertexArray = new rp3d::PolygonVertexArray(8, &(mConvexMesh2CubeVertices[0]), sizeof(Vector3),
					&(mConvexMeshCubeIndices[0]), sizeof(int), 6, mConvexMeshPolygonFaces,
					rp3d::PolygonVertexArray::VertexDataType::VERTEX_FLOAT_TYPE,
					rp3d::PolygonVertexArray::IndexDataType::INDEX_INTEGER_TYPE);
            mConvexMesh2PolyhedronMesh = mPhysicsCommon.createPolyhedronMesh(mConvexMesh2PolygonVertexArray);
            mConvexMeshShape2 = mPhysicsCommon.createConvexMeshShape(mConvexMesh2PolyhedronMesh);
            Transform convexMeshTransform2(Vector3(20, 0, 0), Quaternion::identity());
            mConvexMeshBody2 = mWorld->createCollisionBody(convexMeshTransform2);
            mConvexMeshCollider2 = mConvexMeshBody2->addCollider(mConvexMeshShape2, Transform::identity());

			// ---------- Concave Meshes ---------- //
			for (int i = 0; i < 6; i++) {
				for (int j = 0; j < 6; j++) {
                    mConcaveMeshPlaneVertices[i * 6 + j] = Vector3(-2.5f + i, 0, -2.5f + j);
				}
			}
            int triangleIndex = 0;
			for (int i = 0; i < 5; i++) {
				for (int j = 0; j < 5; j++) {

					// Triangle 1
                    mConcaveMeshPlaneIndices[triangleIndex * 3] = i * 6 + j;
                    mConcaveMeshPlaneIndices[triangleIndex * 3 + 1] =  (i+1) * 6 + (j+1);
                    mConcaveMeshPlaneIndices[triangleIndex * 3 + 2] = i * 6 + (j+1);
                    triangleIndex++;

                    // Triangle 2
                    mConcaveMeshPlaneIndices[triangleIndex * 3] = i * 6 + j;
                    mConcaveMeshPlaneIndices[triangleIndex * 3 + 1] =  (i+1) * 6 + j;
                    mConcaveMeshPlaneIndices[triangleIndex * 3 + 2] = (i+1) * 6 + (j+1);
                    triangleIndex++;
                }
			}

            mConcaveMeshTriangleVertexArray = new rp3d::TriangleVertexArray(36, &(mConcaveMeshPlaneVertices[0]), sizeof(Vector3),
					25, &(mConcaveMeshPlaneIndices[0]), 3 * sizeof(int),
					rp3d::TriangleVertexArray::VertexDataType::VERTEX_FLOAT_TYPE,
					rp3d::TriangleVertexArray::IndexDataType::INDEX_INTEGER_TYPE);

			// Add the triangle vertex array of the subpart to the triangle mesh
            Transform concaveMeshTransform(Vector3(0, -20, 0), Quaternion::identity());
            mConcaveTriangleMesh = mPhysicsCommon.createTriangleMesh();
            mConcaveTriangleMesh->addSubpart(mConcaveMeshTriangleVertexArray);
            mConcaveMeshShape = mPhysicsCommon.createConcaveMeshShape(mConcaveTriangleMesh);
            mConcaveMeshBody = mWorld->createCollisionBody(concaveMeshTransform);
            mConcaveMeshCollider = mConcaveMeshBody->addCollider(mConcaveMeshShape, rp3d::Transform::identity());
        }

        /// Destructor
        virtual ~TestCollisionWorld() {

            mPhysicsCommon.destroyPhysicsWorld(mWorld);

            mPhysicsCommon.destroyBoxShape(mBoxShape1);
            mPhysicsCommon.destroyBoxShape(mBoxShape2);

            mPhysicsCommon.destroySphereShape(mSphereShape1);
            mPhysicsCommon.destroySphereShape(mSphereShape2);

            mPhysicsCommon.destroyCapsuleShape(mCapsuleShape1);
            mPhysicsCommon.destroyCapsuleShape(mCapsuleShape2);

            mPhysicsCommon.destroyConvexMeshShape(mConvexMeshShape1);
            mPhysicsCommon.destroyConvexMeshShape(mConvexMeshShape2);

            mPhysicsCommon.destroyPolyhedronMesh(mConvexMesh1PolyhedronMesh);
            mPhysicsCommon.destroyPolyhedronMesh(mConvexMesh2PolyhedronMesh);

			delete mConvexMesh1PolygonVertexArray;
			delete mConvexMesh2PolygonVertexArray;
            delete[] mConvexMeshPolygonFaces;

            mPhysicsCommon.destroyConcaveMeshShape(mConcaveMeshShape);

            mPhysicsCommon.destroyTriangleMesh(mConcaveTriangleMesh);

            delete mConcaveMeshTriangleVertexArray;

        }

        /// Run the tests
        void run() {

			testNoCollisions();
			testNoOverlap();

			testSphereVsSphereCollision();
			testSphereVsBoxCollision();
			testSphereVsCapsuleCollision();
			testSphereVsConvexMeshCollision();
            testSphereVsConcaveMeshCollision();

            testBoxVsBoxCollision();
            testBoxVsConvexMeshCollision();
            testBoxVsCapsuleCollision();
            testBoxVsConcaveMeshCollision();

            testCapsuleVsCapsuleCollision();
            testCapsuleVsConcaveMeshCollision();

            testConvexMeshVsConvexMeshCollision();
            testConvexMeshVsCapsuleCollision();
            testConvexMeshVsConcaveMeshCollision();
        }

		void testNoCollisions() {

			// All the shapes of the world are not touching when they are created.
			// Here we test that at the beginning, there is no collision at all.

			// ---------- Global test ---------- //

			mCollisionCallback.reset();
            mWorld->testCollision(mCollisionCallback);
            rp3d_test(!mCollisionCallback.hasContacts());

			// ---------- Single body test ---------- //

			mCollisionCallback.reset();
            mWorld->testCollision(mBoxBody1, mCollisionCallback);
            rp3d_test(!mCollisionCallback.hasContacts());

			mCollisionCallback.reset();
            mWorld->testCollision(mBoxBody2, mCollisionCallback);
            rp3d_test(!mCollisionCallback.hasContacts());

			mCollisionCallback.reset();
            mWorld->testCollision(mSphereBody1, mCollisionCallback);
            rp3d_test(!mCollisionCallback.hasContacts());

			mCollisionCallback.reset();
            mWorld->testCollision(mSphereBody2, mCollisionCallback);
            rp3d_test(!mCollisionCallback.hasContacts());

			// Two bodies test

			mCollisionCallback.reset();
            mWorld->testCollision(mBoxBody1, mBoxBody2, mCollisionCallback);
            rp3d_test(!mCollisionCallback.hasContacts());

			mCollisionCallback.reset();
            mWorld->testCollision(mSphereBody1, mSphereBody2, mCollisionCallback);
            rp3d_test(!mCollisionCallback.hasContacts());

			mCollisionCallback.reset();
            mWorld->testCollision(mBoxBody1, mSphereBody1, mCollisionCallback);
            rp3d_test(!mCollisionCallback.hasContacts());

			mCollisionCallback.reset();
            mWorld->testCollision(mBoxBody1, mSphereBody2, mCollisionCallback);
            rp3d_test(!mCollisionCallback.hasContacts());

			mCollisionCallback.reset();
            mWorld->testCollision(mBoxBody2, mSphereBody1, mCollisionCallback);
            rp3d_test(!mCollisionCallback.hasContacts());

			mCollisionCallback.reset();
            mWorld->testCollision(mBoxBody2, mSphereBody2, mCollisionCallback);
            rp3d_test(!mCollisionCallback.hasContacts());
		}

		void testNoOverlap() {

			// All the shapes of the world are not touching when they are created.
			// Here we test that at the beginning, there is no overlap at all.

			// ---------- Single body test ---------- //

			mOverlapCallback.reset();
            mWorld->testOverlap(mBoxBody1, mOverlapCallback);
            rp3d_test(!mOverlapCallback.hasOverlapWithBody(mBoxBody1));

			mOverlapCallback.reset();
            mWorld->testOverlap(mBoxBody2, mOverlapCallback);
            rp3d_test(!mOverlapCallback.hasOverlapWithBody(mBoxBody2));

			mOverlapCallback.reset();
            mWorld->testOverlap(mSphereBody1, mOverlapCallback);
            rp3d_test(!mOverlapCallback.hasOverlapWithBody(mSphereBody1));

			mOverlapCallback.reset();
            mWorld->testOverlap(mSphereBody2, mOverlapCallback);
            rp3d_test(!mOverlapCallback.hasOverlapWithBody(mSphereBody2));

			// Two bodies test

            rp3d_test(!mWorld->testOverlap(mBoxBody1, mBoxBody2));
            rp3d_test(!mWorld->testOverlap(mSphereBody1, mSphereBody2));
            rp3d_test(!mWorld->testOverlap(mBoxBody1, mSphereBody1));
            rp3d_test(!mWorld->testOverlap(mBoxBody1, mSphereBody2));
            rp3d_test(!mWorld->testOverlap(mBoxBody2, mSphereBody1));
            rp3d_test(!mWorld->testOverlap(mBoxBody2, mSphereBody2));
		}

		void testSphereVsSphereCollision() {

			Transform initTransform1 = mSphereBody1->getTransform();
			Transform initTransform2 = mSphereBody2->getTransform();

			Transform transform1(Vector3(10, 20, 50), Quaternion::identity());
			Transform transform2(Vector3(17, 20, 50), Quaternion::fromEulerAngles(rp3d::PI / 8.0f, rp3d::PI / 4.0f, rp3d::PI / 16.0f));

			// Move spheres to collide with each other
			mSphereBody1->setTransform(transform1);
			mSphereBody2->setTransform(transform2);

			mOverlapCallback.reset();
            mWorld->testOverlap(mSphereBody1, mOverlapCallback);
            rp3d_test(mOverlapCallback.hasOverlapWithBody(mSphereBody1));

			mOverlapCallback.reset();
            mWorld->testOverlap(mSphereBody2, mOverlapCallback);
            rp3d_test(mOverlapCallback.hasOverlapWithBody(mSphereBody2));

			// ----- Test global collision test ----- // 

			mCollisionCallback.reset();
            mWorld->testCollision(mCollisionCallback);

            rp3d_test(mCollisionCallback.areCollidersColliding(mSphereCollider1, mSphereCollider2));

			// Get collision data
            const CollisionData* collisionData = mCollisionCallback.getCollisionData(mSphereCollider1, mSphereCollider2);
            rp3d_test(collisionData != nullptr);
            rp3d_test(collisionData->getNbContactPairs() == 1);
            rp3d_test(collisionData->getTotalNbContactPoints() == 1);

			// True if the bodies are swapped in the collision callback response
            bool swappedBodiesCollisionData = collisionData->getBody1()->getEntity() != mSphereBody1->getEntity();

			// Test contact points
			Vector3 localBody1Point(3, 0, 0);
			Vector3 localBody2Point = transform2.getInverse() * Vector3(12, 20, 50);
			decimal penetrationDepth = 1.0f;
            rp3d_test(collisionData->hasContactPointSimilarTo(swappedBodiesCollisionData ? localBody2Point : localBody1Point,
												         swappedBodiesCollisionData ? localBody1Point : localBody2Point,
													     penetrationDepth));

			// ----- Test collision against body 1 only ----- //

			mCollisionCallback.reset();
            mWorld->testCollision(mSphereBody1, mCollisionCallback);

            rp3d_test(mCollisionCallback.areCollidersColliding(mSphereCollider1, mSphereCollider2));

			// Get collision data
            collisionData = mCollisionCallback.getCollisionData(mSphereCollider1, mSphereCollider2);
            rp3d_test(collisionData != nullptr);
            rp3d_test(collisionData->getNbContactPairs() == 1);
            rp3d_test(collisionData->getTotalNbContactPoints() == 1);

			// True if the bodies are swapped in the collision callback response
            swappedBodiesCollisionData = collisionData->getBody1()->getEntity() != mSphereBody1->getEntity();

			// Test contact points
            rp3d_test(collisionData->hasContactPointSimilarTo(swappedBodiesCollisionData ? localBody2Point : localBody1Point,
														 swappedBodiesCollisionData ? localBody1Point : localBody2Point,
														 penetrationDepth));

			// ----- Test collision against body 2 only ----- //

			mCollisionCallback.reset();
            mWorld->testCollision(mSphereBody2, mCollisionCallback);

            rp3d_test(mCollisionCallback.areCollidersColliding(mSphereCollider1, mSphereCollider2));

			// Get collision data
            collisionData = mCollisionCallback.getCollisionData(mSphereCollider1, mSphereCollider2);
            rp3d_test(collisionData != nullptr);
            rp3d_test(collisionData->getNbContactPairs() == 1);
            rp3d_test(collisionData->getTotalNbContactPoints() == 1);

			// True if the bodies are swapped in the collision callback response
            swappedBodiesCollisionData = collisionData->getBody1()->getEntity() != mSphereBody1->getEntity();

			// Test contact points
            rp3d_test(collisionData->hasContactPointSimilarTo(swappedBodiesCollisionData ? localBody2Point : localBody1Point,
														 swappedBodiesCollisionData ? localBody1Point : localBody2Point,
														 penetrationDepth));

			// ----- Test collision against selected body 1 and 2 ----- //

			mCollisionCallback.reset();
            mWorld->testCollision(mSphereBody1, mSphereBody2, mCollisionCallback);

            rp3d_test(mCollisionCallback.areCollidersColliding(mSphereCollider1, mSphereCollider2));

			// Get collision data
            collisionData = mCollisionCallback.getCollisionData(mSphereCollider1, mSphereCollider2);
            rp3d_test(collisionData != nullptr);
            rp3d_test(collisionData->getNbContactPairs() == 1);
            rp3d_test(collisionData->getTotalNbContactPoints() == 1);

			// True if the bodies are swapped in the collision callback response
            swappedBodiesCollisionData = collisionData->getBody1()->getEntity() != mSphereBody1->getEntity();

			// Test contact points
            rp3d_test(collisionData->hasContactPointSimilarTo(swappedBodiesCollisionData ? localBody2Point : localBody1Point,
														 swappedBodiesCollisionData ? localBody1Point : localBody2Point,
														 penetrationDepth));

			// Reset the init transforms
			mSphereBody1->setTransform(initTransform1);
			mSphereBody2->setTransform(initTransform2);
		}

        void testSphereVsBoxCollision() {

			Transform initTransform1 = mSphereBody1->getTransform();
			Transform initTransform2 = mBoxBody1->getTransform();

			/********************************************************************************
			* Test Sphere vs Box Face collision                                             *
			*********************************************************************************/

			Transform transform1(Vector3(10, 20, 50), Quaternion::identity());
			Transform transform2(Vector3(14, 20, 50), Quaternion::identity());

			// Move spheres to collide with each other
			mSphereBody1->setTransform(transform1);
			mBoxBody1->setTransform(transform2);

			mOverlapCallback.reset();
            mWorld->testOverlap(mSphereBody1, mOverlapCallback);
            rp3d_test(mOverlapCallback.hasOverlapWithBody(mSphereBody1));

			mOverlapCallback.reset();
            mWorld->testOverlap(mBoxBody1, mOverlapCallback);
            rp3d_test(mOverlapCallback.hasOverlapWithBody(mBoxBody1));

			// ----- Test global collision test ----- // 

			mCollisionCallback.reset();
            mWorld->testCollision(mCollisionCallback);

            rp3d_test(mCollisionCallback.areCollidersColliding(mSphereCollider1, mBoxCollider1));

			// Get collision data
            const CollisionData* collisionData = mCollisionCallback.getCollisionData(mSphereCollider1, mBoxCollider1);
            rp3d_test(collisionData != nullptr);
            rp3d_test(collisionData->getNbContactPairs() == 1);
            rp3d_test(collisionData->getTotalNbContactPoints() == 1);

			// True if the bodies are swapped in the collision callback response
            bool swappedBodiesCollisionData = collisionData->getBody1()->getEntity() != mSphereBody1->getEntity();

			// Test contact points
			Vector3 localBody1Point(3, 0, 0);
			Vector3 localBody2Point(-3, 0, 0);
			decimal penetrationDepth = 2.0f;
            rp3d_test(collisionData->hasContactPointSimilarTo(swappedBodiesCollisionData ? localBody2Point : localBody1Point,
												         swappedBodiesCollisionData ? localBody1Point : localBody2Point,
													     penetrationDepth));

			// ----- Test collision against body 1 only ----- //

			mCollisionCallback.reset();
            mWorld->testCollision(mSphereBody1, mCollisionCallback);

            rp3d_test(mCollisionCallback.areCollidersColliding(mSphereCollider1, mBoxCollider1));

			// Get collision data
            collisionData = mCollisionCallback.getCollisionData(mSphereCollider1, mBoxCollider1);
            rp3d_test(collisionData != nullptr);
            rp3d_test(collisionData->getNbContactPairs() == 1);
            rp3d_test(collisionData->getTotalNbContactPoints() == 1);

			// True if the bodies are swapped in the collision callback response
            swappedBodiesCollisionData = collisionData->getBody1()->getEntity() != mSphereBody1->getEntity();

			// Test contact points
            rp3d_test(collisionData->hasContactPointSimilarTo(swappedBodiesCollisionData ? localBody2Point : localBody1Point,
														 swappedBodiesCollisionData ? localBody1Point : localBody2Point,
														 penetrationDepth));

			// ----- Test collision against body 2 only ----- //

			mCollisionCallback.reset();
            mWorld->testCollision(mBoxBody1, mCollisionCallback);

            rp3d_test(mCollisionCallback.areCollidersColliding(mSphereCollider1, mBoxCollider1));

			// Get collision data
            collisionData = mCollisionCallback.getCollisionData(mSphereCollider1, mBoxCollider1);
            rp3d_test(collisionData != nullptr);
            rp3d_test(collisionData->getNbContactPairs() == 1);
            rp3d_test(collisionData->getTotalNbContactPoints() == 1);

			// True if the bodies are swapped in the collision callback response
            swappedBodiesCollisionData = collisionData->getBody1()->getEntity() != mSphereBody1->getEntity();

			// Test contact points
            rp3d_test(collisionData->hasContactPointSimilarTo(swappedBodiesCollisionData ? localBody2Point : localBody1Point,
														 swappedBodiesCollisionData ? localBody1Point : localBody2Point,
														 penetrationDepth));

			// ----- Test collision against selected body 1 and 2 ----- //

			mCollisionCallback.reset();
            mWorld->testCollision(mSphereBody1, mBoxBody1, mCollisionCallback);

            rp3d_test(mCollisionCallback.areCollidersColliding(mSphereCollider1, mBoxCollider1));

			// Get collision data
            collisionData = mCollisionCallback.getCollisionData(mSphereCollider1, mBoxCollider1);
            rp3d_test(collisionData != nullptr);
            rp3d_test(collisionData->getNbContactPairs() == 1);
            rp3d_test(collisionData->getTotalNbContactPoints() == 1);

			// True if the bodies are swapped in the collision callback response
            swappedBodiesCollisionData = collisionData->getBody1()->getEntity() != mSphereBody1->getEntity();

			// Test contact points
            rp3d_test(collisionData->hasContactPointSimilarTo(swappedBodiesCollisionData ? localBody2Point : localBody1Point,
														 swappedBodiesCollisionData ? localBody1Point : localBody2Point,
														 penetrationDepth));

			/********************************************************************************
			* Test Sphere vs Box Edge collision                                             *
			*********************************************************************************/

			transform1 = Transform(Vector3(10, 20, 50), Quaternion::identity());
			transform2 = Transform(Vector3(14, 16, 50), Quaternion::identity());

			// Move spheres to collide with each other
			mSphereBody1->setTransform(transform1);
			mBoxBody1->setTransform(transform2);

			mOverlapCallback.reset();
            mWorld->testOverlap(mSphereBody1, mOverlapCallback);
            rp3d_test(mOverlapCallback.hasOverlapWithBody(mSphereBody1));

			mOverlapCallback.reset();
            mWorld->testOverlap(mBoxBody1, mOverlapCallback);
            rp3d_test(mOverlapCallback.hasOverlapWithBody(mBoxBody1));

			// ----- Test global collision test ----- // 

			mCollisionCallback.reset();
            mWorld->testCollision(mCollisionCallback);

            rp3d_test(mCollisionCallback.areCollidersColliding(mSphereCollider1, mBoxCollider1));

			// Get collision data
            collisionData = mCollisionCallback.getCollisionData(mSphereCollider1, mBoxCollider1);
            rp3d_test(collisionData != nullptr);
            rp3d_test(collisionData->getNbContactPairs() == 1);
            rp3d_test(collisionData->getTotalNbContactPoints() == 1);

			// True if the bodies are swapped in the collision callback response
            swappedBodiesCollisionData = collisionData->getBody1()->getEntity() != mSphereBody1->getEntity();

			// Test contact points
			localBody1Point = std::sqrt(4.5f) * Vector3(1, -1, 0);
			localBody2Point = Vector3(-3, 3, 0);
			penetrationDepth = decimal(3.0) - std::sqrt(2);
            rp3d_test(collisionData->hasContactPointSimilarTo(swappedBodiesCollisionData ? localBody2Point : localBody1Point,
												         swappedBodiesCollisionData ? localBody1Point : localBody2Point,
													     penetrationDepth));

			// ----- Test collision against body 1 only ----- //

			mCollisionCallback.reset();
            mWorld->testCollision(mSphereBody1, mCollisionCallback);

            rp3d_test(mCollisionCallback.areCollidersColliding(mSphereCollider1, mBoxCollider1));

			// Get collision data
            collisionData = mCollisionCallback.getCollisionData(mSphereCollider1, mBoxCollider1);
            rp3d_test(collisionData != nullptr);
            rp3d_test(collisionData->getNbContactPairs() == 1);
            rp3d_test(collisionData->getTotalNbContactPoints() == 1);

			// True if the bodies are swapped in the collision callback response
            swappedBodiesCollisionData = collisionData->getBody1()->getEntity() != mSphereBody1->getEntity();

			// Test contact points
            rp3d_test(collisionData->hasContactPointSimilarTo(swappedBodiesCollisionData ? localBody2Point : localBody1Point,
														 swappedBodiesCollisionData ? localBody1Point : localBody2Point,
														 penetrationDepth));

			// ----- Test collision against body 2 only ----- //

			mCollisionCallback.reset();
            mWorld->testCollision(mBoxBody1, mCollisionCallback);

            rp3d_test(mCollisionCallback.areCollidersColliding(mSphereCollider1, mBoxCollider1));

			// Get collision data
            collisionData = mCollisionCallback.getCollisionData(mSphereCollider1, mBoxCollider1);
            rp3d_test(collisionData != nullptr);
            rp3d_test(collisionData->getNbContactPairs() == 1);
            rp3d_test(collisionData->getTotalNbContactPoints() == 1);

			// True if the bodies are swapped in the collision callback response
            swappedBodiesCollisionData = collisionData->getBody1()->getEntity() != mSphereBody1->getEntity();

			// Test contact points
            rp3d_test(collisionData->hasContactPointSimilarTo(swappedBodiesCollisionData ? localBody2Point : localBody1Point,
														 swappedBodiesCollisionData ? localBody1Point : localBody2Point,
														 penetrationDepth));

			// ----- Test collision against selected body 1 and 2 ----- //

			mCollisionCallback.reset();
            mWorld->testCollision(mSphereBody1, mBoxBody1, mCollisionCallback);

            rp3d_test(mCollisionCallback.areCollidersColliding(mSphereCollider1, mBoxCollider1));

			// Get collision data
            collisionData = mCollisionCallback.getCollisionData(mSphereCollider1, mBoxCollider1);
            rp3d_test(collisionData != nullptr);
            rp3d_test(collisionData->getNbContactPairs() == 1);
            rp3d_test(collisionData->getTotalNbContactPoints() == 1);

			// True if the bodies are swapped in the collision callback response
            swappedBodiesCollisionData = collisionData->getBody1()->getEntity() != mSphereBody1->getEntity();

			// Test contact points
            rp3d_test(collisionData->hasContactPointSimilarTo(swappedBodiesCollisionData ? localBody2Point : localBody1Point,
														 swappedBodiesCollisionData ? localBody1Point : localBody2Point,
														 penetrationDepth));

			/********************************************************************************
			* Test Sphere vs Box Vertex collision                                             *
			*********************************************************************************/

			transform1 = Transform(Vector3(10, 20, 50), Quaternion::identity());
			transform2 = Transform(Vector3(14, 16, 46), Quaternion::identity());

			// Move spheres to collide with each other
			mSphereBody1->setTransform(transform1);
			mBoxBody1->setTransform(transform2);

			mOverlapCallback.reset();
            mWorld->testOverlap(mSphereBody1, mOverlapCallback);
            rp3d_test(mOverlapCallback.hasOverlapWithBody(mSphereBody1));

			mOverlapCallback.reset();
            mWorld->testOverlap(mBoxBody1, mOverlapCallback);
            rp3d_test(mOverlapCallback.hasOverlapWithBody(mBoxBody1));

			// ----- Test global collision test ----- // 

			mCollisionCallback.reset();
            mWorld->testCollision(mCollisionCallback);

            rp3d_test(mCollisionCallback.areCollidersColliding(mSphereCollider1, mBoxCollider1));

			// Get collision data
            collisionData = mCollisionCallback.getCollisionData(mSphereCollider1, mBoxCollider1);
            rp3d_test(collisionData != nullptr);
            rp3d_test(collisionData->getNbContactPairs() == 1);
            rp3d_test(collisionData->getTotalNbContactPoints() == 1);

			// True if the bodies are swapped in the collision callback response
            swappedBodiesCollisionData = collisionData->getBody1()->getEntity() != mSphereBody1->getEntity();

			// Test contact points
			localBody1Point = std::sqrt(9.0f / 3.0f) * Vector3(1, -1, -1);
			localBody2Point = Vector3(-3, 3, 3);
			penetrationDepth = decimal(3.0) - std::sqrt(3);
            rp3d_test(collisionData->hasContactPointSimilarTo(swappedBodiesCollisionData ? localBody2Point : localBody1Point,
												         swappedBodiesCollisionData ? localBody1Point : localBody2Point,
													     penetrationDepth));

			// ----- Test collision against body 1 only ----- //

			mCollisionCallback.reset();
            mWorld->testCollision(mSphereBody1, mCollisionCallback);

            rp3d_test(mCollisionCallback.areCollidersColliding(mSphereCollider1, mBoxCollider1));

			// Get collision data
            collisionData = mCollisionCallback.getCollisionData(mSphereCollider1, mBoxCollider1);
            rp3d_test(collisionData != nullptr);
            rp3d_test(collisionData->getNbContactPairs() == 1);
            rp3d_test(collisionData->getTotalNbContactPoints() == 1);

			// True if the bodies are swapped in the collision callback response
            swappedBodiesCollisionData = collisionData->getBody1()->getEntity() != mSphereBody1->getEntity();

			// Test contact points
            rp3d_test(collisionData->hasContactPointSimilarTo(swappedBodiesCollisionData ? localBody2Point : localBody1Point,
														 swappedBodiesCollisionData ? localBody1Point : localBody2Point,
														 penetrationDepth));

			// ----- Test collision against body 2 only ----- //

			mCollisionCallback.reset();
            mWorld->testCollision(mBoxBody1, mCollisionCallback);

            rp3d_test(mCollisionCallback.areCollidersColliding(mSphereCollider1, mBoxCollider1));

			// Get collision data
            collisionData = mCollisionCallback.getCollisionData(mSphereCollider1, mBoxCollider1);
            rp3d_test(collisionData != nullptr);
            rp3d_test(collisionData->getNbContactPairs() == 1);
            rp3d_test(collisionData->getTotalNbContactPoints() == 1);

			// True if the bodies are swapped in the collision callback response
            swappedBodiesCollisionData = collisionData->getBody1()->getEntity() != mSphereBody1->getEntity();

			// Test contact points
            rp3d_test(collisionData->hasContactPointSimilarTo(swappedBodiesCollisionData ? localBody2Point : localBody1Point,
														 swappedBodiesCollisionData ? localBody1Point : localBody2Point,
														 penetrationDepth));

			// ----- Test collision against selected body 1 and 2 ----- //

			mCollisionCallback.reset();
            mWorld->testCollision(mSphereBody1, mBoxBody1, mCollisionCallback);

            rp3d_test(mCollisionCallback.areCollidersColliding(mSphereCollider1, mBoxCollider1));

			// Get collision data
            collisionData = mCollisionCallback.getCollisionData(mSphereCollider1, mBoxCollider1);
            rp3d_test(collisionData != nullptr);
            rp3d_test(collisionData->getNbContactPairs() == 1);
            rp3d_test(collisionData->getTotalNbContactPoints() == 1);

			// True if the bodies are swapped in the collision callback response
            swappedBodiesCollisionData = collisionData->getBody1()->getEntity() != mSphereBody1->getEntity();

			// Test contact points
            rp3d_test(collisionData->hasContactPointSimilarTo(swappedBodiesCollisionData ? localBody2Point : localBody1Point,
														 swappedBodiesCollisionData ? localBody1Point : localBody2Point,
														 penetrationDepth));

			// Reset the init transforms
			mSphereBody1->setTransform(initTransform1);
			mBoxBody1->setTransform(initTransform2);
        }

		void testSphereVsCapsuleCollision() {

			Transform initTransform1 = mSphereBody1->getTransform();
			Transform initTransform2 = mCapsuleBody1->getTransform();

			/********************************************************************************
			* Test Sphere vs Capsule (sphere side) collision                                             *
			*********************************************************************************/

			Transform transform1(Vector3(10, 20, 50), Quaternion::identity());
			Transform transform2(Vector3(10, 14, 50), Quaternion::identity());

			// Move spheres to collide with each other
			mSphereBody1->setTransform(transform1);
			mCapsuleBody1->setTransform(transform2);

			mOverlapCallback.reset();
            mWorld->testOverlap(mSphereBody1, mOverlapCallback);
            rp3d_test(mOverlapCallback.hasOverlapWithBody(mSphereBody1));

			mOverlapCallback.reset();
            mWorld->testOverlap(mCapsuleBody1, mOverlapCallback);
            rp3d_test(mOverlapCallback.hasOverlapWithBody(mCapsuleBody1));

			// ----- Test global collision test ----- // 

			mCollisionCallback.reset();
            mWorld->testCollision(mCollisionCallback);

            rp3d_test(mCollisionCallback.areCollidersColliding(mSphereCollider1, mCapsuleCollider1));

			// Get collision data
            const CollisionData* collisionData = mCollisionCallback.getCollisionData(mSphereCollider1, mCapsuleCollider1);
            rp3d_test(collisionData != nullptr);
            rp3d_test(collisionData->getNbContactPairs() == 1);
            rp3d_test(collisionData->getTotalNbContactPoints() == 1);

			// True if the bodies are swapped in the collision callback response
            bool swappedBodiesCollisionData = collisionData->getBody1()->getEntity() != mSphereBody1->getEntity();

			// Test contact points
			Vector3 localBody1Point(0, -3, 0);
			Vector3 localBody2Point(0, 5, 0);
			decimal penetrationDepth = 2.0f;
            rp3d_test(collisionData->hasContactPointSimilarTo(swappedBodiesCollisionData ? localBody2Point : localBody1Point,
				swappedBodiesCollisionData ? localBody1Point : localBody2Point,
				penetrationDepth));

			// ----- Test collision against body 1 only ----- //

			mCollisionCallback.reset();
            mWorld->testCollision(mSphereBody1, mCollisionCallback);

            rp3d_test(mCollisionCallback.areCollidersColliding(mSphereCollider1, mCapsuleCollider1));

			// Get collision data
            collisionData = mCollisionCallback.getCollisionData(mSphereCollider1, mCapsuleCollider1);
            rp3d_test(collisionData != nullptr);
            rp3d_test(collisionData->getNbContactPairs() == 1);
            rp3d_test(collisionData->getTotalNbContactPoints() == 1);

			// True if the bodies are swapped in the collision callback response
            swappedBodiesCollisionData = collisionData->getBody1()->getEntity() != mSphereBody1->getEntity();

			// Test contact points
            rp3d_test(collisionData->hasContactPointSimilarTo(swappedBodiesCollisionData ? localBody2Point : localBody1Point,
				swappedBodiesCollisionData ? localBody1Point : localBody2Point,
				penetrationDepth));

			// ----- Test collision against body 2 only ----- //

			mCollisionCallback.reset();
            mWorld->testCollision(mCapsuleBody1, mCollisionCallback);

            rp3d_test(mCollisionCallback.areCollidersColliding(mSphereCollider1, mCapsuleCollider1));

			// Get collision data
            collisionData = mCollisionCallback.getCollisionData(mSphereCollider1, mCapsuleCollider1);
            rp3d_test(collisionData != nullptr);
            rp3d_test(collisionData->getNbContactPairs() == 1);
            rp3d_test(collisionData->getTotalNbContactPoints() == 1);

			// True if the bodies are swapped in the collision callback response
            swappedBodiesCollisionData = collisionData->getBody1()->getEntity() != mSphereBody1->getEntity();

			// Test contact points
            rp3d_test(collisionData->hasContactPointSimilarTo(swappedBodiesCollisionData ? localBody2Point : localBody1Point,
				swappedBodiesCollisionData ? localBody1Point : localBody2Point,
				penetrationDepth));

			// ----- Test collision against selected body 1 and 2 ----- //

			mCollisionCallback.reset();
            mWorld->testCollision(mSphereBody1, mCapsuleBody1, mCollisionCallback);

            rp3d_test(mCollisionCallback.areCollidersColliding(mSphereCollider1, mCapsuleCollider1));

			// Get collision data
            collisionData = mCollisionCallback.getCollisionData(mSphereCollider1, mCapsuleCollider1);
            rp3d_test(collisionData != nullptr);
            rp3d_test(collisionData->getNbContactPairs() == 1);
            rp3d_test(collisionData->getTotalNbContactPoints() == 1);

			// True if the bodies are swapped in the collision callback response
            swappedBodiesCollisionData = collisionData->getBody1()->getEntity() != mSphereBody1->getEntity();

			// Test contact points
            rp3d_test(collisionData->hasContactPointSimilarTo(swappedBodiesCollisionData ? localBody2Point : localBody1Point,
				swappedBodiesCollisionData ? localBody1Point : localBody2Point,
				penetrationDepth));

			/********************************************************************************
			* Test Sphere vs Box Capsule (cylinder side) collision                          *
			*********************************************************************************/

			transform1 = Transform(Vector3(10, 20, 50), Quaternion::identity());
			transform2 = Transform(Vector3(14, 19, 50), Quaternion::identity());

			// Move spheres to collide with each other
			mSphereBody1->setTransform(transform1);
			mCapsuleBody1->setTransform(transform2);

			mOverlapCallback.reset();
            mWorld->testOverlap(mSphereBody1, mOverlapCallback);
            rp3d_test(mOverlapCallback.hasOverlapWithBody(mSphereBody1));

			mOverlapCallback.reset();
            mWorld->testOverlap(mCapsuleBody1, mOverlapCallback);
            rp3d_test(mOverlapCallback.hasOverlapWithBody(mCapsuleBody1));

			// ----- Test global collision test ----- // 

			mCollisionCallback.reset();
            mWorld->testCollision(mCollisionCallback);

            rp3d_test(mCollisionCallback.areCollidersColliding(mSphereCollider1, mCapsuleCollider1));

			// Get collision data
            collisionData = mCollisionCallback.getCollisionData(mSphereCollider1, mCapsuleCollider1);
            rp3d_test(collisionData != nullptr);
            rp3d_test(collisionData->getNbContactPairs() == 1);
            rp3d_test(collisionData->getTotalNbContactPoints() == 1);

			// True if the bodies are swapped in the collision callback response
            swappedBodiesCollisionData = collisionData->getBody1()->getEntity() != mSphereBody1->getEntity();

			// Test contact points
			localBody1Point = Vector3(3, 0, 0);
			localBody2Point = Vector3(-2, 1, 0);
			penetrationDepth = decimal(1.0);
            rp3d_test(collisionData->hasContactPointSimilarTo(swappedBodiesCollisionData ? localBody2Point : localBody1Point,
				swappedBodiesCollisionData ? localBody1Point : localBody2Point,
				penetrationDepth));

			// ----- Test collision against body 1 only ----- //

			mCollisionCallback.reset();
            mWorld->testCollision(mSphereBody1, mCollisionCallback);

            rp3d_test(mCollisionCallback.areCollidersColliding(mSphereCollider1, mCapsuleCollider1));

			// Get collision data
            collisionData = mCollisionCallback.getCollisionData(mSphereCollider1, mCapsuleCollider1);
            rp3d_test(collisionData != nullptr);
            rp3d_test(collisionData->getNbContactPairs() == 1);
            rp3d_test(collisionData->getTotalNbContactPoints() == 1);

			// True if the bodies are swapped in the collision callback response
            swappedBodiesCollisionData = collisionData->getBody1()->getEntity() != mSphereBody1->getEntity();

			// Test contact points
            rp3d_test(collisionData->hasContactPointSimilarTo(swappedBodiesCollisionData ? localBody2Point : localBody1Point,
				swappedBodiesCollisionData ? localBody1Point : localBody2Point,
				penetrationDepth));

			// ----- Test collision against body 2 only ----- //

			mCollisionCallback.reset();
            mWorld->testCollision(mCapsuleBody1, mCollisionCallback);

            rp3d_test(mCollisionCallback.areCollidersColliding(mSphereCollider1, mCapsuleCollider1));

			// Get collision data
            collisionData = mCollisionCallback.getCollisionData(mSphereCollider1, mCapsuleCollider1);
            rp3d_test(collisionData != nullptr);
            rp3d_test(collisionData->getNbContactPairs() == 1);
            rp3d_test(collisionData->getTotalNbContactPoints() == 1);

			// True if the bodies are swapped in the collision callback response
            swappedBodiesCollisionData = collisionData->getBody1()->getEntity() != mSphereBody1->getEntity();

			// Test contact points
            rp3d_test(collisionData->hasContactPointSimilarTo(swappedBodiesCollisionData ? localBody2Point : localBody1Point,
				swappedBodiesCollisionData ? localBody1Point : localBody2Point,
				penetrationDepth));

			// ----- Test collision against selected body 1 and 2 ----- //

			mCollisionCallback.reset();
            mWorld->testCollision(mSphereBody1, mCapsuleBody1, mCollisionCallback);

            rp3d_test(mCollisionCallback.areCollidersColliding(mSphereCollider1, mCapsuleCollider1));

			// Get collision data
            collisionData = mCollisionCallback.getCollisionData(mSphereCollider1, mCapsuleCollider1);
            rp3d_test(collisionData != nullptr);
            rp3d_test(collisionData->getNbContactPairs() == 1);
            rp3d_test(collisionData->getTotalNbContactPoints() == 1);

			// True if the bodies are swapped in the collision callback response
            swappedBodiesCollisionData = collisionData->getBody1()->getEntity() != mSphereBody1->getEntity();

			// Test contact points
            rp3d_test(collisionData->hasContactPointSimilarTo(swappedBodiesCollisionData ? localBody2Point : localBody1Point,
				swappedBodiesCollisionData ? localBody1Point : localBody2Point,
				penetrationDepth));

			// Reset the init transforms
			mSphereBody1->setTransform(initTransform1);
			mCapsuleBody1->setTransform(initTransform2);
		}

        void testSphereVsConvexMeshCollision() {

			Transform initTransform1 = mSphereBody1->getTransform();
			Transform initTransform2 = mConvexMeshBody1->getTransform();

			/********************************************************************************
			* Test Sphere vs Convex Mesh (Cube Face) collision                              *
			*********************************************************************************/

			Transform transform1(Vector3(10, 20, 50), Quaternion::identity());
			Transform transform2(Vector3(14, 20, 50), Quaternion::identity());

			// Move spheres to collide with each other
			mSphereBody1->setTransform(transform1);
			mConvexMeshBody1->setTransform(transform2);

			mOverlapCallback.reset();
            mWorld->testOverlap(mSphereBody1, mOverlapCallback);
            rp3d_test(mOverlapCallback.hasOverlapWithBody(mSphereBody1));

			mOverlapCallback.reset();
            mWorld->testOverlap(mConvexMeshBody1, mOverlapCallback);
            rp3d_test(mOverlapCallback.hasOverlapWithBody(mConvexMeshBody1));

			// ----- Test global collision test ----- // 

			mCollisionCallback.reset();
            mWorld->testCollision(mCollisionCallback);

            rp3d_test(mCollisionCallback.areCollidersColliding(mSphereCollider1, mConvexMeshCollider1));

			// Get collision data
            const CollisionData* collisionData = mCollisionCallback.getCollisionData(mSphereCollider1, mConvexMeshCollider1);
            rp3d_test(collisionData != nullptr);
            rp3d_test(collisionData->getNbContactPairs() == 1);
            rp3d_test(collisionData->getTotalNbContactPoints() == 1);

			// True if the bodies are swapped in the collision callback response
            bool swappedBodiesCollisionData = collisionData->getBody1()->getEntity() != mSphereBody1->getEntity();

			// Test contact points
			Vector3 localBody1Point(3, 0, 0);
			Vector3 localBody2Point(-3, 0, 0);
			decimal penetrationDepth = 2.0f;
            rp3d_test(collisionData->hasContactPointSimilarTo(swappedBodiesCollisionData ? localBody2Point : localBody1Point,
												         swappedBodiesCollisionData ? localBody1Point : localBody2Point,
													     penetrationDepth));

			// ----- Test collision against body 1 only ----- //

			mCollisionCallback.reset();
            mWorld->testCollision(mSphereBody1, mCollisionCallback);

            rp3d_test(mCollisionCallback.areCollidersColliding(mSphereCollider1, mConvexMeshCollider1));

			// Get collision data
            collisionData = mCollisionCallback.getCollisionData(mSphereCollider1, mConvexMeshCollider1);
            rp3d_test(collisionData != nullptr);
            rp3d_test(collisionData->getNbContactPairs() == 1);
            rp3d_test(collisionData->getTotalNbContactPoints() == 1);

			// True if the bodies are swapped in the collision callback response
            swappedBodiesCollisionData = collisionData->getBody1()->getEntity() != mSphereBody1->getEntity();

			// Test contact points
            rp3d_test(collisionData->hasContactPointSimilarTo(swappedBodiesCollisionData ? localBody2Point : localBody1Point,
														 swappedBodiesCollisionData ? localBody1Point : localBody2Point,
														 penetrationDepth));

			// ----- Test collision against body 2 only ----- //

			mCollisionCallback.reset();
            mWorld->testCollision(mConvexMeshBody1, mCollisionCallback);

            rp3d_test(mCollisionCallback.areCollidersColliding(mSphereCollider1, mConvexMeshCollider1));

			// Get collision data
            collisionData = mCollisionCallback.getCollisionData(mSphereCollider1, mConvexMeshCollider1);
            rp3d_test(collisionData != nullptr);
            rp3d_test(collisionData->getNbContactPairs() == 1);
            rp3d_test(collisionData->getTotalNbContactPoints() == 1);

			// True if the bodies are swapped in the collision callback response
            swappedBodiesCollisionData = collisionData->getBody1()->getEntity() != mSphereBody1->getEntity();

			// Test contact points
            rp3d_test(collisionData->hasContactPointSimilarTo(swappedBodiesCollisionData ? localBody2Point : localBody1Point,
														 swappedBodiesCollisionData ? localBody1Point : localBody2Point,
														 penetrationDepth));

			// ----- Test collision against selected body 1 and 2 ----- //

			mCollisionCallback.reset();
            mWorld->testCollision(mSphereBody1, mConvexMeshBody1, mCollisionCallback);

            rp3d_test(mCollisionCallback.areCollidersColliding(mSphereCollider1, mConvexMeshCollider1));

			// Get collision data
            collisionData = mCollisionCallback.getCollisionData(mSphereCollider1, mConvexMeshCollider1);
            rp3d_test(collisionData != nullptr);
            rp3d_test(collisionData->getNbContactPairs() == 1);
            rp3d_test(collisionData->getTotalNbContactPoints() == 1);

			// True if the bodies are swapped in the collision callback response
            swappedBodiesCollisionData = collisionData->getBody1()->getEntity() != mSphereBody1->getEntity();

			// Test contact points
            rp3d_test(collisionData->hasContactPointSimilarTo(swappedBodiesCollisionData ? localBody2Point : localBody1Point,
														 swappedBodiesCollisionData ? localBody1Point : localBody2Point,
														 penetrationDepth));

			/********************************************************************************
			* Test Sphere vs Convex Mesh (Cube Edge) collision                              *
			*********************************************************************************/

			transform1 = Transform(Vector3(10, 20, 50), Quaternion::identity());
			transform2 = Transform(Vector3(14, 16, 50), Quaternion::identity());

			// Move spheres to collide with each other
			mSphereBody1->setTransform(transform1);
			mConvexMeshBody1->setTransform(transform2);

			mOverlapCallback.reset();
            mWorld->testOverlap(mSphereBody1, mOverlapCallback);
            rp3d_test(mOverlapCallback.hasOverlapWithBody(mSphereBody1));

			mOverlapCallback.reset();
            mWorld->testOverlap(mConvexMeshBody1, mOverlapCallback);
            rp3d_test(mOverlapCallback.hasOverlapWithBody(mConvexMeshBody1));

			// ----- Test global collision test ----- // 

			mCollisionCallback.reset();
            mWorld->testCollision(mCollisionCallback);

            rp3d_test(mCollisionCallback.areCollidersColliding(mSphereCollider1, mConvexMeshCollider1));

			// Get collision data
            collisionData = mCollisionCallback.getCollisionData(mSphereCollider1, mConvexMeshCollider1);
            rp3d_test(collisionData != nullptr);
            rp3d_test(collisionData->getNbContactPairs() == 1);
            rp3d_test(collisionData->getTotalNbContactPoints() == 1);

			// True if the bodies are swapped in the collision callback response
            swappedBodiesCollisionData = collisionData->getBody1()->getEntity() != mSphereBody1->getEntity();

			// Test contact points
			localBody1Point = std::sqrt(4.5f) * Vector3(1, -1, 0);
			localBody2Point = Vector3(-3, 3, 0);
			penetrationDepth = decimal(3.0) - std::sqrt(2);
            rp3d_test(collisionData->hasContactPointSimilarTo(swappedBodiesCollisionData ? localBody2Point : localBody1Point,
												         swappedBodiesCollisionData ? localBody1Point : localBody2Point,
													     penetrationDepth));

			// ----- Test collision against body 1 only ----- //

			mCollisionCallback.reset();
            mWorld->testCollision(mSphereBody1, mCollisionCallback);

            rp3d_test(mCollisionCallback.areCollidersColliding(mSphereCollider1, mConvexMeshCollider1));

			// Get collision data
            collisionData = mCollisionCallback.getCollisionData(mSphereCollider1, mConvexMeshCollider1);
            rp3d_test(collisionData != nullptr);
            rp3d_test(collisionData->getNbContactPairs() == 1);
            rp3d_test(collisionData->getTotalNbContactPoints() == 1);

			// True if the bodies are swapped in the collision callback response
            swappedBodiesCollisionData = collisionData->getBody1()->getEntity() != mSphereBody1->getEntity();

			// Test contact points
            rp3d_test(collisionData->hasContactPointSimilarTo(swappedBodiesCollisionData ? localBody2Point : localBody1Point,
														 swappedBodiesCollisionData ? localBody1Point : localBody2Point,
														 penetrationDepth));

			// ----- Test collision against body 2 only ----- //

			mCollisionCallback.reset();
            mWorld->testCollision(mConvexMeshBody1, mCollisionCallback);

            rp3d_test(mCollisionCallback.areCollidersColliding(mSphereCollider1, mConvexMeshCollider1));

			// Get collision data
            collisionData = mCollisionCallback.getCollisionData(mSphereCollider1, mConvexMeshCollider1);
            rp3d_test(collisionData != nullptr);
            rp3d_test(collisionData->getNbContactPairs() == 1);
            rp3d_test(collisionData->getTotalNbContactPoints() == 1);

			// True if the bodies are swapped in the collision callback response
            swappedBodiesCollisionData = collisionData->getBody1()->getEntity() != mSphereBody1->getEntity();

			// Test contact points
            rp3d_test(collisionData->hasContactPointSimilarTo(swappedBodiesCollisionData ? localBody2Point : localBody1Point,
														 swappedBodiesCollisionData ? localBody1Point : localBody2Point,
														 penetrationDepth));

			// ----- Test collision against selected body 1 and 2 ----- //

			mCollisionCallback.reset();
            mWorld->testCollision(mSphereBody1, mConvexMeshBody1, mCollisionCallback);

            rp3d_test(mCollisionCallback.areCollidersColliding(mSphereCollider1, mConvexMeshCollider1));

			// Get collision data
            collisionData = mCollisionCallback.getCollisionData(mSphereCollider1, mConvexMeshCollider1);
            rp3d_test(collisionData != nullptr);
            rp3d_test(collisionData->getNbContactPairs() == 1);
            rp3d_test(collisionData->getTotalNbContactPoints() == 1);

			// True if the bodies are swapped in the collision callback response
            swappedBodiesCollisionData = collisionData->getBody1()->getEntity() != mSphereBody1->getEntity();

			// Test contact points
            rp3d_test(collisionData->hasContactPointSimilarTo(swappedBodiesCollisionData ? localBody2Point : localBody1Point,
														 swappedBodiesCollisionData ? localBody1Point : localBody2Point,
														 penetrationDepth));

			/********************************************************************************
			* Test Sphere vs ConvexMesh (Cube Vertex) collision                             *
			*********************************************************************************/

			transform1 = Transform(Vector3(10, 20, 50), Quaternion::identity());
			transform2 = Transform(Vector3(14, 16, 46), Quaternion::identity());

			// Move spheres to collide with each other
			mSphereBody1->setTransform(transform1);
			mConvexMeshBody1->setTransform(transform2);

			mOverlapCallback.reset();
            mWorld->testOverlap(mSphereBody1, mOverlapCallback);
            rp3d_test(mOverlapCallback.hasOverlapWithBody(mSphereBody1));

			mOverlapCallback.reset();
            mWorld->testOverlap(mConvexMeshBody1, mOverlapCallback);
            rp3d_test(mOverlapCallback.hasOverlapWithBody(mConvexMeshBody1));

			// ----- Test global collision test ----- // 

			mCollisionCallback.reset();
            mWorld->testCollision(mCollisionCallback);

            rp3d_test(mCollisionCallback.areCollidersColliding(mSphereCollider1, mConvexMeshCollider1));

			// Get collision data
            collisionData = mCollisionCallback.getCollisionData(mSphereCollider1, mConvexMeshCollider1);
            rp3d_test(collisionData != nullptr);
            rp3d_test(collisionData->getNbContactPairs() == 1);
            rp3d_test(collisionData->getTotalNbContactPoints() == 1);

			// True if the bodies are swapped in the collision callback response
            swappedBodiesCollisionData = collisionData->getBody1()->getEntity() != mSphereBody1->getEntity();

			// Test contact points
			localBody1Point = std::sqrt(9.0f / 3.0f) * Vector3(1, -1, -1);
			localBody2Point = Vector3(-3, 3, 3);
			penetrationDepth = decimal(3.0) - std::sqrt(3);
            rp3d_test(collisionData->hasContactPointSimilarTo(swappedBodiesCollisionData ? localBody2Point : localBody1Point,
												         swappedBodiesCollisionData ? localBody1Point : localBody2Point,
													     penetrationDepth));

			// ----- Test collision against body 1 only ----- //

			mCollisionCallback.reset();
            mWorld->testCollision(mSphereBody1, mCollisionCallback);

            rp3d_test(mCollisionCallback.areCollidersColliding(mSphereCollider1, mConvexMeshCollider1));

			// Get collision data
            collisionData = mCollisionCallback.getCollisionData(mSphereCollider1, mConvexMeshCollider1);
            rp3d_test(collisionData != nullptr);
            rp3d_test(collisionData->getNbContactPairs() == 1);
            rp3d_test(collisionData->getTotalNbContactPoints() == 1);

			// True if the bodies are swapped in the collision callback response
            swappedBodiesCollisionData = collisionData->getBody1()->getEntity() != mSphereBody1->getEntity();

			// Test contact points
            rp3d_test(collisionData->hasContactPointSimilarTo(swappedBodiesCollisionData ? localBody2Point : localBody1Point,
														 swappedBodiesCollisionData ? localBody1Point : localBody2Point,
														 penetrationDepth));

			// ----- Test collision against body 2 only ----- //

			mCollisionCallback.reset();
            mWorld->testCollision(mConvexMeshBody1, mCollisionCallback);

            rp3d_test(mCollisionCallback.areCollidersColliding(mSphereCollider1, mConvexMeshCollider1));

			// Get collision data
            collisionData = mCollisionCallback.getCollisionData(mSphereCollider1, mConvexMeshCollider1);
            rp3d_test(collisionData != nullptr);
            rp3d_test(collisionData->getNbContactPairs() == 1);
            rp3d_test(collisionData->getTotalNbContactPoints() == 1);

			// True if the bodies are swapped in the collision callback response
            swappedBodiesCollisionData = collisionData->getBody1()->getEntity() != mSphereBody1->getEntity();

			// Test contact points
            rp3d_test(collisionData->hasContactPointSimilarTo(swappedBodiesCollisionData ? localBody2Point : localBody1Point,
														 swappedBodiesCollisionData ? localBody1Point : localBody2Point,
														 penetrationDepth));

			// ----- Test collision against selected body 1 and 2 ----- //

			mCollisionCallback.reset();
            mWorld->testCollision(mSphereBody1, mConvexMeshBody1, mCollisionCallback);

            rp3d_test(mCollisionCallback.areCollidersColliding(mSphereCollider1, mConvexMeshCollider1));

			// Get collision data
            collisionData = mCollisionCallback.getCollisionData(mSphereCollider1, mConvexMeshCollider1);
            rp3d_test(collisionData != nullptr);
            rp3d_test(collisionData->getNbContactPairs() == 1);
            rp3d_test(collisionData->getTotalNbContactPoints() == 1);

			// True if the bodies are swapped in the collision callback response
            swappedBodiesCollisionData = collisionData->getBody1()->getEntity() != mSphereBody1->getEntity();

			// Test contact points
            rp3d_test(collisionData->hasContactPointSimilarTo(swappedBodiesCollisionData ? localBody2Point : localBody1Point,
														 swappedBodiesCollisionData ? localBody1Point : localBody2Point,
														 penetrationDepth));

			// Reset the init transforms
			mSphereBody1->setTransform(initTransform1);
			mConvexMeshBody1->setTransform(initTransform2);
        }

        void testSphereVsConcaveMeshCollision() {

            Transform initTransform1 = mSphereBody1->getTransform();
            Transform initTransform2 = mConcaveMeshBody->getTransform();

            /********************************************************************************
            * Test Sphere vs Concave Mesh
            *********************************************************************************/

            Transform transform1(Vector3(10, 22.98f, 50), Quaternion::identity());
            Transform transform2(Vector3(10, 20, 50), Quaternion::identity());

            // Move spheres to collide with each other
            mSphereBody1->setTransform(transform1);
            mConcaveMeshBody->setTransform(transform2);

            mOverlapCallback.reset();
            mWorld->testOverlap(mSphereBody1, mOverlapCallback);
            rp3d_test(mOverlapCallback.hasOverlapWithBody(mSphereBody1));

            mOverlapCallback.reset();
            mWorld->testOverlap(mConcaveMeshBody, mOverlapCallback);
            rp3d_test(mOverlapCallback.hasOverlapWithBody(mConcaveMeshBody));

            // ----- Test global collision test ----- //

            mCollisionCallback.reset();
            mWorld->testCollision(mCollisionCallback);

            rp3d_test(mCollisionCallback.areCollidersColliding(mSphereCollider1, mConcaveMeshCollider));

            // Get collision data
            const CollisionData* collisionData = mCollisionCallback.getCollisionData(mSphereCollider1, mConcaveMeshCollider);
            rp3d_test(collisionData != nullptr);
            rp3d_test(collisionData->getNbContactPairs() == 1);
            rp3d_test(collisionData->getTotalNbContactPoints() == 1);

            // True if the bodies are swapped in the collision callback response
            bool swappedBodiesCollisionData = collisionData->getBody1()->getEntity() != mSphereBody1->getEntity();

            // Test contact points
            Vector3 localBody1Point(0, -3, 0);
            Vector3 localBody2Point(0, 0, 0);
            decimal penetrationDepth = 0.02f;
            rp3d_test(collisionData->hasContactPointSimilarTo(swappedBodiesCollisionData ? localBody2Point : localBody1Point,
                                                         swappedBodiesCollisionData ? localBody1Point : localBody2Point,
                                                         penetrationDepth));

            // ----- Test collision against body 1 only ----- //

            mCollisionCallback.reset();
            mWorld->testCollision(mSphereBody1, mCollisionCallback);

            rp3d_test(mCollisionCallback.areCollidersColliding(mSphereCollider1, mConcaveMeshCollider));

            // Get collision data
            collisionData = mCollisionCallback.getCollisionData(mSphereCollider1, mConcaveMeshCollider);
            rp3d_test(collisionData != nullptr);
            rp3d_test(collisionData->getNbContactPairs() == 1);
            rp3d_test(collisionData->getTotalNbContactPoints() == 1);

            // True if the bodies are swapped in the collision callback response
            swappedBodiesCollisionData = collisionData->getBody1()->getEntity() != mSphereBody1->getEntity();

            // Test contact points
            rp3d_test(collisionData->hasContactPointSimilarTo(swappedBodiesCollisionData ? localBody2Point : localBody1Point,
                                                         swappedBodiesCollisionData ? localBody1Point : localBody2Point,
                                                         penetrationDepth));

            // ----- Test collision against body 2 only ----- //

            mCollisionCallback.reset();
            mWorld->testCollision(mConcaveMeshBody, mCollisionCallback);

            rp3d_test(mCollisionCallback.areCollidersColliding(mSphereCollider1, mConcaveMeshCollider));

            // Get collision data
            collisionData = mCollisionCallback.getCollisionData(mSphereCollider1, mConcaveMeshCollider);
            rp3d_test(collisionData != nullptr);
            rp3d_test(collisionData->getNbContactPairs() == 1);
            rp3d_test(collisionData->getTotalNbContactPoints() == 1);

            // True if the bodies are swapped in the collision callback response
            swappedBodiesCollisionData = collisionData->getBody1()->getEntity() != mSphereBody1->getEntity();

            // Test contact points
            rp3d_test(collisionData->hasContactPointSimilarTo(swappedBodiesCollisionData ? localBody2Point : localBody1Point,
                                                         swappedBodiesCollisionData ? localBody1Point : localBody2Point,
                                                         penetrationDepth));

            // ----- Test collision against selected body 1 and 2 ----- //

            mCollisionCallback.reset();
            mWorld->testCollision(mSphereBody1, mConcaveMeshBody, mCollisionCallback);

            rp3d_test(mCollisionCallback.areCollidersColliding(mSphereCollider1, mConcaveMeshCollider));

            // Get collision data
            collisionData = mCollisionCallback.getCollisionData(mSphereCollider1, mConcaveMeshCollider);
            rp3d_test(collisionData != nullptr);
            rp3d_test(collisionData->getNbContactPairs() == 1);
            rp3d_test(collisionData->getTotalNbContactPoints() == 1);

            // True if the bodies are swapped in the collision callback response
            swappedBodiesCollisionData = collisionData->getBody1()->getEntity() != mSphereBody1->getEntity();

            // Test contact points
            rp3d_test(collisionData->hasContactPointSimilarTo(swappedBodiesCollisionData ? localBody2Point : localBody1Point,
                                                         swappedBodiesCollisionData ? localBody1Point : localBody2Point,
                                                         penetrationDepth));

            // Reset the init transforms
            mSphereBody1->setTransform(initTransform1);
            mConcaveMeshBody->setTransform(initTransform2);
        }

        void testBoxVsBoxCollision() {

            Transform initTransform1 = mBoxBody1->getTransform();
            Transform initTransform2 = mBoxBody2->getTransform();

            /********************************************************************************
            * Test Box vs Box Face collision                                             *
            *********************************************************************************/

            Transform transform1(Vector3(11, 20, 50), Quaternion::identity());
            Transform transform2(Vector3(4.5, 16, 40), Quaternion::identity());

            // Move spheres to collide with each other
            mBoxBody1->setTransform(transform1);
            mBoxBody2->setTransform(transform2);

            mOverlapCallback.reset();
            mWorld->testOverlap(mBoxBody1, mOverlapCallback);
            rp3d_test(mOverlapCallback.hasOverlapWithBody(mBoxBody1));

            mOverlapCallback.reset();
            mWorld->testOverlap(mBoxBody2, mOverlapCallback);
            rp3d_test(mOverlapCallback.hasOverlapWithBody(mBoxBody2));

            // ----- Test global collision test ----- //

            mCollisionCallback.reset();
            mWorld->testCollision(mCollisionCallback);

            rp3d_test(mCollisionCallback.areCollidersColliding(mBoxCollider1, mBoxCollider2));

            // Get collision data
            const CollisionData* collisionData = mCollisionCallback.getCollisionData(mBoxCollider1, mBoxCollider2);
            rp3d_test(collisionData != nullptr);
            rp3d_test(collisionData->getNbContactPairs() == 1);
            rp3d_test(collisionData->getTotalNbContactPoints() == 4);

            // True if the bodies are swapped in the collision callback response
            bool swappedBodiesCollisionData = collisionData->getBody1()->getEntity() != mBoxBody1->getEntity();

            // Test contact points
            Vector3 localBody1Point1(-3, -2, -2);
            Vector3 localBody2Point1(4, 2, 8);
            decimal penetrationDepth1 = 0.5f;
            rp3d_test(collisionData->hasContactPointSimilarTo(swappedBodiesCollisionData ? localBody2Point1 : localBody1Point1,
                                                         swappedBodiesCollisionData ? localBody1Point1 : localBody2Point1,
                                                         penetrationDepth1));
            Vector3 localBody1Point2(-3, -2, -3);
            Vector3 localBody2Point2(4, 2, 7);
            decimal penetrationDepth2 = 0.5f;
            rp3d_test(collisionData->hasContactPointSimilarTo(swappedBodiesCollisionData ? localBody2Point2 : localBody1Point2,
                                                         swappedBodiesCollisionData ? localBody1Point2 : localBody2Point2,
                                                         penetrationDepth2));
            Vector3 localBody1Point3(-3, -3, -2);
            Vector3 localBody2Point3(4, 1, 8);
            decimal penetrationDepth3 = 0.5f;
            rp3d_test(collisionData->hasContactPointSimilarTo(swappedBodiesCollisionData ? localBody2Point3 : localBody1Point3,
                                                         swappedBodiesCollisionData ? localBody1Point3 : localBody2Point3,
                                                         penetrationDepth3));
            Vector3 localBody1Point4(-3, -3, -3);
            Vector3 localBody2Point4(4, 1, 7);
            decimal penetrationDepth4 = 0.5f;
            rp3d_test(collisionData->hasContactPointSimilarTo(swappedBodiesCollisionData ? localBody2Point4 : localBody1Point4,
                                                         swappedBodiesCollisionData ? localBody1Point4 : localBody2Point4,
                                                         penetrationDepth4));

            // ----- Test collision against body 1 only ----- //

            mCollisionCallback.reset();
            mWorld->testCollision(mBoxBody1, mCollisionCallback);

            rp3d_test(mCollisionCallback.areCollidersColliding(mBoxCollider1, mBoxCollider2));

            // Get collision data
            collisionData = mCollisionCallback.getCollisionData(mBoxCollider1, mBoxCollider2);
            rp3d_test(collisionData != nullptr);
            rp3d_test(collisionData->getNbContactPairs() == 1);
            rp3d_test(collisionData->getTotalNbContactPoints() == 4);

            // True if the bodies are swapped in the collision callback response
            swappedBodiesCollisionData = collisionData->getBody1()->getEntity() != mBoxBody1->getEntity();

            // Test contact points
            rp3d_test(collisionData->hasContactPointSimilarTo(swappedBodiesCollisionData ? localBody2Point1 : localBody1Point1,
                                                         swappedBodiesCollisionData ? localBody1Point1 : localBody2Point1,
                                                         penetrationDepth1));
            rp3d_test(collisionData->hasContactPointSimilarTo(swappedBodiesCollisionData ? localBody2Point2 : localBody1Point2,
                                                         swappedBodiesCollisionData ? localBody1Point2 : localBody2Point2,
                                                         penetrationDepth2));
            rp3d_test(collisionData->hasContactPointSimilarTo(swappedBodiesCollisionData ? localBody2Point3 : localBody1Point3,
                                                         swappedBodiesCollisionData ? localBody1Point3 : localBody2Point3,
                                                         penetrationDepth3));
            rp3d_test(collisionData->hasContactPointSimilarTo(swappedBodiesCollisionData ? localBody2Point4 : localBody1Point4,
                                                         swappedBodiesCollisionData ? localBody1Point4 : localBody2Point4,
                                                         penetrationDepth4));
            // ----- Test collision against body 2 only ----- //

            mCollisionCallback.reset();
            mWorld->testCollision(mBoxBody2, mCollisionCallback);

            rp3d_test(mCollisionCallback.areCollidersColliding(mBoxCollider1, mBoxCollider2));

            // Get collision data
            collisionData = mCollisionCallback.getCollisionData(mBoxCollider1, mBoxCollider2);
            rp3d_test(collisionData != nullptr);
            rp3d_test(collisionData->getNbContactPairs() == 1);
            rp3d_test(collisionData->getTotalNbContactPoints() == 4);

            // True if the bodies are swapped in the collision callback response
            swappedBodiesCollisionData = collisionData->getBody1()->getEntity() != mBoxBody1->getEntity();

            // Test contact points
            rp3d_test(collisionData->hasContactPointSimilarTo(swappedBodiesCollisionData ? localBody2Point1 : localBody1Point1,
                                                         swappedBodiesCollisionData ? localBody1Point1 : localBody2Point1,
                                                         penetrationDepth1));
            rp3d_test(collisionData->hasContactPointSimilarTo(swappedBodiesCollisionData ? localBody2Point2 : localBody1Point2,
                                                         swappedBodiesCollisionData ? localBody1Point2 : localBody2Point2,
                                                         penetrationDepth2));
            rp3d_test(collisionData->hasContactPointSimilarTo(swappedBodiesCollisionData ? localBody2Point3 : localBody1Point3,
                                                         swappedBodiesCollisionData ? localBody1Point3 : localBody2Point3,
                                                         penetrationDepth3));
            rp3d_test(collisionData->hasContactPointSimilarTo(swappedBodiesCollisionData ? localBody2Point4 : localBody1Point4,
                                                         swappedBodiesCollisionData ? localBody1Point4 : localBody2Point4,
                                                         penetrationDepth4));

            // ----- Test collision against selected body 1 and 2 ----- //

            mCollisionCallback.reset();
            mWorld->testCollision(mBoxBody1, mBoxBody2, mCollisionCallback);

            rp3d_test(mCollisionCallback.areCollidersColliding(mBoxCollider1, mBoxCollider2));

            // Get collision data
            collisionData = mCollisionCallback.getCollisionData(mBoxCollider1, mBoxCollider2);
            rp3d_test(collisionData != nullptr);
            rp3d_test(collisionData->getNbContactPairs() == 1);
            rp3d_test(collisionData->getTotalNbContactPoints() == 4);

            // True if the bodies are swapped in the collision callback response
            swappedBodiesCollisionData = collisionData->getBody1()->getEntity() != mBoxBody1->getEntity();

            // Test contact points
            rp3d_test(collisionData->hasContactPointSimilarTo(swappedBodiesCollisionData ? localBody2Point1 : localBody1Point1,
                                                         swappedBodiesCollisionData ? localBody1Point1 : localBody2Point1,
                                                         penetrationDepth1));
            rp3d_test(collisionData->hasContactPointSimilarTo(swappedBodiesCollisionData ? localBody2Point2 : localBody1Point2,
                                                         swappedBodiesCollisionData ? localBody1Point2 : localBody2Point2,
                                                         penetrationDepth2));
            rp3d_test(collisionData->hasContactPointSimilarTo(swappedBodiesCollisionData ? localBody2Point3 : localBody1Point3,
                                                         swappedBodiesCollisionData ? localBody1Point3 : localBody2Point3,
                                                         penetrationDepth3));
            rp3d_test(collisionData->hasContactPointSimilarTo(swappedBodiesCollisionData ? localBody2Point4 : localBody1Point4,
                                                         swappedBodiesCollisionData ? localBody1Point4 : localBody2Point4,
                                                         penetrationDepth4));

            // reset the init transforms
            mBoxBody1->setTransform(initTransform1);
            mBoxBody2->setTransform(initTransform2);
        }

        void testBoxVsConvexMeshCollision() {

            Transform initTransform1 = mBoxBody1->getTransform();
            Transform initTransform2 = mConvexMeshBody2->getTransform();

            /********************************************************************************
            * Test Box vs Convex Mesh collision                                             *
            *********************************************************************************/

            Transform transform1(Vector3(11, 20, 50), Quaternion::identity());
            Transform transform2(Vector3(4.5, 16, 40), Quaternion::identity());

            // Move spheres to collide with each other
            mBoxBody1->setTransform(transform1);
            mConvexMeshBody2->setTransform(transform2);

            mOverlapCallback.reset();
            mWorld->testOverlap(mBoxBody1, mOverlapCallback);
            rp3d_test(mOverlapCallback.hasOverlapWithBody(mBoxBody1));

            mOverlapCallback.reset();
            mWorld->testOverlap(mConvexMeshBody2, mOverlapCallback);
            rp3d_test(mOverlapCallback.hasOverlapWithBody(mConvexMeshBody2));

            // ----- Test global collision test ----- //

            mCollisionCallback.reset();
            mWorld->testCollision(mCollisionCallback);

            rp3d_test(mCollisionCallback.areCollidersColliding(mBoxCollider1, mConvexMeshCollider2));

            // Get collision data
            const CollisionData* collisionData = mCollisionCallback.getCollisionData(mBoxCollider1, mConvexMeshCollider2);
            rp3d_test(collisionData != nullptr);
            rp3d_test(collisionData->getNbContactPairs() == 1);
            rp3d_test(collisionData->getTotalNbContactPoints() == 4);

            // True if the bodies are swapped in the collision callback response
            bool swappedBodiesCollisionData = collisionData->getBody1()->getEntity() != mBoxBody1->getEntity();

            // Test contact points
            Vector3 localBody1Point1(-3, -2, -2);
            Vector3 localBody2Point1(4, 2, 8);
            decimal penetrationDepth1 = 0.5f;
            rp3d_test(collisionData->hasContactPointSimilarTo(swappedBodiesCollisionData ? localBody2Point1 : localBody1Point1,
                                                         swappedBodiesCollisionData ? localBody1Point1 : localBody2Point1,
                                                         penetrationDepth1));
            Vector3 localBody1Point2(-3, -2, -3);
            Vector3 localBody2Point2(4, 2, 7);
            decimal penetrationDepth2 = 0.5f;
            rp3d_test(collisionData->hasContactPointSimilarTo(swappedBodiesCollisionData ? localBody2Point2 : localBody1Point2,
                                                         swappedBodiesCollisionData ? localBody1Point2 : localBody2Point2,
                                                         penetrationDepth2));
            Vector3 localBody1Point3(-3, -3, -2);
            Vector3 localBody2Point3(4, 1, 8);
            decimal penetrationDepth3 = 0.5f;
            rp3d_test(collisionData->hasContactPointSimilarTo(swappedBodiesCollisionData ? localBody2Point3 : localBody1Point3,
                                                         swappedBodiesCollisionData ? localBody1Point3 : localBody2Point3,
                                                         penetrationDepth3));
            Vector3 localBody1Point4(-3, -3, -3);
            Vector3 localBody2Point4(4, 1, 7);
            decimal penetrationDepth4 = 0.5f;
            rp3d_test(collisionData->hasContactPointSimilarTo(swappedBodiesCollisionData ? localBody2Point4 : localBody1Point4,
                                                         swappedBodiesCollisionData ? localBody1Point4 : localBody2Point4,
                                                         penetrationDepth4));

            // ----- Test collision against body 1 only ----- //

            mCollisionCallback.reset();
            mWorld->testCollision(mBoxBody1, mCollisionCallback);

            rp3d_test(mCollisionCallback.areCollidersColliding(mBoxCollider1, mConvexMeshCollider2));

            // Get collision data
            collisionData = mCollisionCallback.getCollisionData(mBoxCollider1, mConvexMeshCollider2);
            rp3d_test(collisionData != nullptr);
            rp3d_test(collisionData->getNbContactPairs() == 1);
            rp3d_test(collisionData->getTotalNbContactPoints() == 4);

            // True if the bodies are swapped in the collision callback response
            swappedBodiesCollisionData = collisionData->getBody1()->getEntity() != mBoxBody1->getEntity();

            // Test contact points
            rp3d_test(collisionData->hasContactPointSimilarTo(swappedBodiesCollisionData ? localBody2Point1 : localBody1Point1,
                                                         swappedBodiesCollisionData ? localBody1Point1 : localBody2Point1,
                                                         penetrationDepth1));
            rp3d_test(collisionData->hasContactPointSimilarTo(swappedBodiesCollisionData ? localBody2Point2 : localBody1Point2,
                                                         swappedBodiesCollisionData ? localBody1Point2 : localBody2Point2,
                                                         penetrationDepth2));
            rp3d_test(collisionData->hasContactPointSimilarTo(swappedBodiesCollisionData ? localBody2Point3 : localBody1Point3,
                                                         swappedBodiesCollisionData ? localBody1Point3 : localBody2Point3,
                                                         penetrationDepth3));
            rp3d_test(collisionData->hasContactPointSimilarTo(swappedBodiesCollisionData ? localBody2Point4 : localBody1Point4,
                                                         swappedBodiesCollisionData ? localBody1Point4 : localBody2Point4,
                                                         penetrationDepth4));
            // ----- Test collision against body 2 only ----- //

            mCollisionCallback.reset();
            mWorld->testCollision(mConvexMeshBody2, mCollisionCallback);

            rp3d_test(mCollisionCallback.areCollidersColliding(mBoxCollider1, mConvexMeshCollider2));

            // Get collision data
            collisionData = mCollisionCallback.getCollisionData(mBoxCollider1, mConvexMeshCollider2);
            rp3d_test(collisionData != nullptr);
            rp3d_test(collisionData->getNbContactPairs() == 1);
            rp3d_test(collisionData->getTotalNbContactPoints() == 4);

            // True if the bodies are swapped in the collision callback response
            swappedBodiesCollisionData = collisionData->getBody1()->getEntity() != mBoxBody1->getEntity();

            // Test contact points
            rp3d_test(collisionData->hasContactPointSimilarTo(swappedBodiesCollisionData ? localBody2Point1 : localBody1Point1,
                                                         swappedBodiesCollisionData ? localBody1Point1 : localBody2Point1,
                                                         penetrationDepth1));
            rp3d_test(collisionData->hasContactPointSimilarTo(swappedBodiesCollisionData ? localBody2Point2 : localBody1Point2,
                                                         swappedBodiesCollisionData ? localBody1Point2 : localBody2Point2,
                                                         penetrationDepth2));
            rp3d_test(collisionData->hasContactPointSimilarTo(swappedBodiesCollisionData ? localBody2Point3 : localBody1Point3,
                                                         swappedBodiesCollisionData ? localBody1Point3 : localBody2Point3,
                                                         penetrationDepth3));
            rp3d_test(collisionData->hasContactPointSimilarTo(swappedBodiesCollisionData ? localBody2Point4 : localBody1Point4,
                                                         swappedBodiesCollisionData ? localBody1Point4 : localBody2Point4,
                                                         penetrationDepth4));

            // ----- Test collision against selected body 1 and 2 ----- //

            mCollisionCallback.reset();
            mWorld->testCollision(mBoxBody1, mConvexMeshBody2, mCollisionCallback);

            rp3d_test(mCollisionCallback.areCollidersColliding(mBoxCollider1, mConvexMeshCollider2));

            // Get collision data
            collisionData = mCollisionCallback.getCollisionData(mBoxCollider1, mConvexMeshCollider2);
            rp3d_test(collisionData != nullptr);
            rp3d_test(collisionData->getNbContactPairs() == 1);
            rp3d_test(collisionData->getTotalNbContactPoints() == 4);

            // True if the bodies are swapped in the collision callback response
            swappedBodiesCollisionData = collisionData->getBody1()->getEntity() != mBoxBody1->getEntity();

            // Test contact points
            rp3d_test(collisionData->hasContactPointSimilarTo(swappedBodiesCollisionData ? localBody2Point1 : localBody1Point1,
                                                         swappedBodiesCollisionData ? localBody1Point1 : localBody2Point1,
                                                         penetrationDepth1));
            rp3d_test(collisionData->hasContactPointSimilarTo(swappedBodiesCollisionData ? localBody2Point2 : localBody1Point2,
                                                         swappedBodiesCollisionData ? localBody1Point2 : localBody2Point2,
                                                         penetrationDepth2));
            rp3d_test(collisionData->hasContactPointSimilarTo(swappedBodiesCollisionData ? localBody2Point3 : localBody1Point3,
                                                         swappedBodiesCollisionData ? localBody1Point3 : localBody2Point3,
                                                         penetrationDepth3));
            rp3d_test(collisionData->hasContactPointSimilarTo(swappedBodiesCollisionData ? localBody2Point4 : localBody1Point4,
                                                         swappedBodiesCollisionData ? localBody1Point4 : localBody2Point4,
                                                         penetrationDepth4));

            // reset the init transforms
            mBoxBody1->setTransform(initTransform1);
            mConvexMeshBody2->setTransform(initTransform2);
        }

        void testConvexMeshVsConvexMeshCollision() {

            Transform initTransform1 = mConvexMeshBody1->getTransform();
            Transform initTransform2 = mConvexMeshBody2->getTransform();

            /********************************************************************************
            * Test Convex Mesh vs Convex Mesh collision                                             *
            *********************************************************************************/

            Transform transform1(Vector3(11, 20, 50), Quaternion::identity());
            Transform transform2(Vector3(4.5, 16, 40), Quaternion::identity());

            // Move spheres to collide with each other
            mConvexMeshBody1->setTransform(transform1);
            mConvexMeshBody2->setTransform(transform2);

            mOverlapCallback.reset();
            mWorld->testOverlap(mConvexMeshBody1, mOverlapCallback);
            rp3d_test(mOverlapCallback.hasOverlapWithBody(mConvexMeshBody1));

            mOverlapCallback.reset();
            mWorld->testOverlap(mConvexMeshBody2, mOverlapCallback);
            rp3d_test(mOverlapCallback.hasOverlapWithBody(mConvexMeshBody2));

            // ----- Test global collision test ----- //

            mCollisionCallback.reset();
            mWorld->testCollision(mCollisionCallback);

            rp3d_test(mCollisionCallback.areCollidersColliding(mConvexMeshCollider1, mConvexMeshCollider2));

            // Get collision data
            const CollisionData* collisionData = mCollisionCallback.getCollisionData(mConvexMeshCollider1, mConvexMeshCollider2);
            rp3d_test(collisionData != nullptr);
            rp3d_test(collisionData->getNbContactPairs() == 1);
            rp3d_test(collisionData->getTotalNbContactPoints() == 4);

            // True if the bodies are swapped in the collision callback response
            bool swappedBodiesCollisionData = collisionData->getBody1()->getEntity() != mConvexMeshBody1->getEntity();

            // Test contact points
            Vector3 localBody1Point1(-3, -2, -2);
            Vector3 localBody2Point1(4, 2, 8);
            decimal penetrationDepth1 = 0.5f;
            rp3d_test(collisionData->hasContactPointSimilarTo(swappedBodiesCollisionData ? localBody2Point1 : localBody1Point1,
                                                         swappedBodiesCollisionData ? localBody1Point1 : localBody2Point1,
                                                         penetrationDepth1));
            Vector3 localBody1Point2(-3, -2, -3);
            Vector3 localBody2Point2(4, 2, 7);
            decimal penetrationDepth2 = 0.5f;
            rp3d_test(collisionData->hasContactPointSimilarTo(swappedBodiesCollisionData ? localBody2Point2 : localBody1Point2,
                                                         swappedBodiesCollisionData ? localBody1Point2 : localBody2Point2,
                                                         penetrationDepth2));
            Vector3 localBody1Point3(-3, -3, -2);
            Vector3 localBody2Point3(4, 1, 8);
            decimal penetrationDepth3 = 0.5f;
            rp3d_test(collisionData->hasContactPointSimilarTo(swappedBodiesCollisionData ? localBody2Point3 : localBody1Point3,
                                                         swappedBodiesCollisionData ? localBody1Point3 : localBody2Point3,
                                                         penetrationDepth3));
            Vector3 localBody1Point4(-3, -3, -3);
            Vector3 localBody2Point4(4, 1, 7);
            decimal penetrationDepth4 = 0.5f;
            rp3d_test(collisionData->hasContactPointSimilarTo(swappedBodiesCollisionData ? localBody2Point4 : localBody1Point4,
                                                         swappedBodiesCollisionData ? localBody1Point4 : localBody2Point4,
                                                         penetrationDepth4));

            // ----- Test collision against body 1 only ----- //

            mCollisionCallback.reset();
            mWorld->testCollision(mConvexMeshBody1, mCollisionCallback);

            rp3d_test(mCollisionCallback.areCollidersColliding(mConvexMeshCollider1, mConvexMeshCollider2));

            // Get collision data
            collisionData = mCollisionCallback.getCollisionData(mConvexMeshCollider1, mConvexMeshCollider2);
            rp3d_test(collisionData != nullptr);
            rp3d_test(collisionData->getNbContactPairs() == 1);
            rp3d_test(collisionData->getTotalNbContactPoints() == 4);

            // True if the bodies are swapped in the collision callback response
            swappedBodiesCollisionData = collisionData->getBody1()->getEntity() != mConvexMeshBody1->getEntity();

            // Test contact points
            rp3d_test(collisionData->hasContactPointSimilarTo(swappedBodiesCollisionData ? localBody2Point1 : localBody1Point1,
                                                         swappedBodiesCollisionData ? localBody1Point1 : localBody2Point1,
                                                         penetrationDepth1));
            rp3d_test(collisionData->hasContactPointSimilarTo(swappedBodiesCollisionData ? localBody2Point2 : localBody1Point2,
                                                         swappedBodiesCollisionData ? localBody1Point2 : localBody2Point2,
                                                         penetrationDepth2));
            rp3d_test(collisionData->hasContactPointSimilarTo(swappedBodiesCollisionData ? localBody2Point3 : localBody1Point3,
                                                         swappedBodiesCollisionData ? localBody1Point3 : localBody2Point3,
                                                         penetrationDepth3));
            rp3d_test(collisionData->hasContactPointSimilarTo(swappedBodiesCollisionData ? localBody2Point4 : localBody1Point4,
                                                         swappedBodiesCollisionData ? localBody1Point4 : localBody2Point4,
                                                         penetrationDepth4));
            // ----- Test collision against body 2 only ----- //

            mCollisionCallback.reset();
            mWorld->testCollision(mConvexMeshBody2, mCollisionCallback);

            rp3d_test(mCollisionCallback.areCollidersColliding(mConvexMeshCollider1, mConvexMeshCollider2));

            // Get collision data
            collisionData = mCollisionCallback.getCollisionData(mConvexMeshCollider1, mConvexMeshCollider2);
            rp3d_test(collisionData != nullptr);
            rp3d_test(collisionData->getNbContactPairs() == 1);
            rp3d_test(collisionData->getTotalNbContactPoints() == 4);

            // True if the bodies are swapped in the collision callback response
            swappedBodiesCollisionData = collisionData->getBody1()->getEntity() != mConvexMeshBody1->getEntity();

            // Test contact points
            rp3d_test(collisionData->hasContactPointSimilarTo(swappedBodiesCollisionData ? localBody2Point1 : localBody1Point1,
                                                         swappedBodiesCollisionData ? localBody1Point1 : localBody2Point1,
                                                         penetrationDepth1));
            rp3d_test(collisionData->hasContactPointSimilarTo(swappedBodiesCollisionData ? localBody2Point2 : localBody1Point2,
                                                         swappedBodiesCollisionData ? localBody1Point2 : localBody2Point2,
                                                         penetrationDepth2));
            rp3d_test(collisionData->hasContactPointSimilarTo(swappedBodiesCollisionData ? localBody2Point3 : localBody1Point3,
                                                         swappedBodiesCollisionData ? localBody1Point3 : localBody2Point3,
                                                         penetrationDepth3));
            rp3d_test(collisionData->hasContactPointSimilarTo(swappedBodiesCollisionData ? localBody2Point4 : localBody1Point4,
                                                         swappedBodiesCollisionData ? localBody1Point4 : localBody2Point4,
                                                         penetrationDepth4));

            // ----- Test collision against selected body 1 and 2 ----- //

            mCollisionCallback.reset();
            mWorld->testCollision(mConvexMeshBody1, mConvexMeshBody2, mCollisionCallback);

            rp3d_test(mCollisionCallback.areCollidersColliding(mConvexMeshCollider1, mConvexMeshCollider2));

            // Get collision data
            collisionData = mCollisionCallback.getCollisionData(mConvexMeshCollider1, mConvexMeshCollider2);
            rp3d_test(collisionData != nullptr);
            rp3d_test(collisionData->getNbContactPairs() == 1);
            rp3d_test(collisionData->getTotalNbContactPoints() == 4);

            // True if the bodies are swapped in the collision callback response
            swappedBodiesCollisionData = collisionData->getBody1()->getEntity() != mConvexMeshBody1->getEntity();

            // Test contact points
            rp3d_test(collisionData->hasContactPointSimilarTo(swappedBodiesCollisionData ? localBody2Point1 : localBody1Point1,
                                                         swappedBodiesCollisionData ? localBody1Point1 : localBody2Point1,
                                                         penetrationDepth1));
            rp3d_test(collisionData->hasContactPointSimilarTo(swappedBodiesCollisionData ? localBody2Point2 : localBody1Point2,
                                                         swappedBodiesCollisionData ? localBody1Point2 : localBody2Point2,
                                                         penetrationDepth2));
            rp3d_test(collisionData->hasContactPointSimilarTo(swappedBodiesCollisionData ? localBody2Point3 : localBody1Point3,
                                                         swappedBodiesCollisionData ? localBody1Point3 : localBody2Point3,
                                                         penetrationDepth3));
            rp3d_test(collisionData->hasContactPointSimilarTo(swappedBodiesCollisionData ? localBody2Point4 : localBody1Point4,
                                                         swappedBodiesCollisionData ? localBody1Point4 : localBody2Point4,
                                                         penetrationDepth4));

            // reset the init transforms
            mConvexMeshBody1->setTransform(initTransform1);
            mConvexMeshBody2->setTransform(initTransform2);
        }

        void testBoxVsCapsuleCollision() {

            Transform initTransform1 = mBoxBody1->getTransform();
            Transform initTransform2 = mCapsuleBody1->getTransform();

            /********************************************************************************
            * Test Box vs Capsule collision                                             *
            *********************************************************************************/

            Transform transform1(Vector3(10, 20, 50), Quaternion::identity());
            Transform transform2(Vector3(17, 21, 50), Quaternion::fromEulerAngles(0, 0, rp3d::PI * 0.5f));

            // Move spheres to collide with each other
            mBoxBody1->setTransform(transform1);
            mCapsuleBody1->setTransform(transform2);

            mOverlapCallback.reset();
            mWorld->testOverlap(mBoxBody1, mOverlapCallback);
            rp3d_test(mOverlapCallback.hasOverlapWithBody(mBoxBody1));

            mOverlapCallback.reset();
            mWorld->testOverlap(mCapsuleBody1, mOverlapCallback);
            rp3d_test(mOverlapCallback.hasOverlapWithBody(mCapsuleBody1));

            // ----- Test global collision test ----- //

            mCollisionCallback.reset();
            mWorld->testCollision(mCollisionCallback);

            rp3d_test(mCollisionCallback.areCollidersColliding(mBoxCollider1, mCapsuleCollider1));

            // Get collision data
            const CollisionData* collisionData = mCollisionCallback.getCollisionData(mBoxCollider1, mCapsuleCollider1);
            rp3d_test(collisionData != nullptr);
            rp3d_test(collisionData->getNbContactPairs() == 1);
            rp3d_test(collisionData->getTotalNbContactPoints() == 1);

            // True if the bodies are swapped in the collision callback response
            bool swappedBodiesCollisionData = collisionData->getBody1()->getEntity() != mBoxBody1->getEntity();

            // Test contact points
            Vector3 localBody1Point1(3, 1, 0);
            Vector3 localBody2Point1(0, 5, 0);
            decimal penetrationDepth1 = 1.0f;
            rp3d_test(collisionData->hasContactPointSimilarTo(swappedBodiesCollisionData ? localBody2Point1 : localBody1Point1,
                                                         swappedBodiesCollisionData ? localBody1Point1 : localBody2Point1,
                                                         penetrationDepth1));
            // ----- Test collision against body 1 only ----- //

            mCollisionCallback.reset();
            mWorld->testCollision(mBoxBody1, mCollisionCallback);

            rp3d_test(mCollisionCallback.areCollidersColliding(mBoxCollider1, mCapsuleCollider1));

            // Get collision data
            collisionData = mCollisionCallback.getCollisionData(mBoxCollider1, mCapsuleCollider1);
            rp3d_test(collisionData != nullptr);
            rp3d_test(collisionData->getNbContactPairs() == 1);
            rp3d_test(collisionData->getTotalNbContactPoints() == 1);

            // True if the bodies are swapped in the collision callback response
            swappedBodiesCollisionData = collisionData->getBody1()->getEntity() != mBoxBody1->getEntity();

            // Test contact points
            rp3d_test(collisionData->hasContactPointSimilarTo(swappedBodiesCollisionData ? localBody2Point1 : localBody1Point1,
                                                         swappedBodiesCollisionData ? localBody1Point1 : localBody2Point1,
                                                         penetrationDepth1));

            // ----- Test collision against body 2 only ----- //

            mCollisionCallback.reset();
            mWorld->testCollision(mCapsuleBody1, mCollisionCallback);

            rp3d_test(mCollisionCallback.areCollidersColliding(mBoxCollider1, mCapsuleCollider1));

            // Get collision data
            collisionData = mCollisionCallback.getCollisionData(mBoxCollider1, mCapsuleCollider1);
            rp3d_test(collisionData != nullptr);
            rp3d_test(collisionData->getNbContactPairs() == 1);
            rp3d_test(collisionData->getTotalNbContactPoints() == 1);

            // True if the bodies are swapped in the collision callback response
            swappedBodiesCollisionData = collisionData->getBody1()->getEntity() != mBoxBody1->getEntity();

            // Test contact points
            rp3d_test(collisionData->hasContactPointSimilarTo(swappedBodiesCollisionData ? localBody2Point1 : localBody1Point1,
                                                         swappedBodiesCollisionData ? localBody1Point1 : localBody2Point1,
                                                         penetrationDepth1));

            // ----- Test collision against selected body 1 and 2 ----- //

            mCollisionCallback.reset();
            mWorld->testCollision(mBoxBody1, mCapsuleBody1, mCollisionCallback);

            rp3d_test(mCollisionCallback.areCollidersColliding(mBoxCollider1, mCapsuleCollider1));

            // Get collision data
            collisionData = mCollisionCallback.getCollisionData(mBoxCollider1, mCapsuleCollider1);
            rp3d_test(collisionData != nullptr);
            rp3d_test(collisionData->getNbContactPairs() == 1);
            rp3d_test(collisionData->getTotalNbContactPoints() == 1);

            // True if the bodies are swapped in the collision callback response
            swappedBodiesCollisionData = collisionData->getBody1()->getEntity() != mBoxBody1->getEntity();

            // Test contact points
            rp3d_test(collisionData->hasContactPointSimilarTo(swappedBodiesCollisionData ? localBody2Point1 : localBody1Point1,
                                                         swappedBodiesCollisionData ? localBody1Point1 : localBody2Point1,
                                                         penetrationDepth1));

            // reset the init transforms
            mBoxBody1->setTransform(initTransform1);
            mCapsuleBody1->setTransform(initTransform2);
        }

        void testConvexMeshVsCapsuleCollision() {

            Transform initTransform1 = mConvexMeshBody1->getTransform();
            Transform initTransform2 = mCapsuleBody1->getTransform();

            /********************************************************************************
            * Test Convex Mesh vs Capsule collision                                             *
            *********************************************************************************/

            Transform transform1(Vector3(10, 20, 50), Quaternion::identity());
            Transform transform2(Vector3(17, 21, 50), Quaternion::fromEulerAngles(0, 0, rp3d::PI * 0.5f));

            // Move spheres to collide with each other
            mConvexMeshBody1->setTransform(transform1);
            mCapsuleBody1->setTransform(transform2);

            mOverlapCallback.reset();
            mWorld->testOverlap(mConvexMeshBody1, mOverlapCallback);
            rp3d_test(mOverlapCallback.hasOverlapWithBody(mConvexMeshBody1));

            mOverlapCallback.reset();
            mWorld->testOverlap(mCapsuleBody1, mOverlapCallback);
            rp3d_test(mOverlapCallback.hasOverlapWithBody(mCapsuleBody1));

            // ----- Test global collision test ----- //

            mCollisionCallback.reset();
            mWorld->testCollision(mCollisionCallback);

            rp3d_test(mCollisionCallback.areCollidersColliding(mConvexMeshCollider1, mCapsuleCollider1));

            // Get collision data
            const CollisionData* collisionData = mCollisionCallback.getCollisionData(mConvexMeshCollider1, mCapsuleCollider1);
            rp3d_test(collisionData != nullptr);
            rp3d_test(collisionData->getNbContactPairs() == 1);
            rp3d_test(collisionData->getTotalNbContactPoints() == 1);

            // True if the bodies are swapped in the collision callback response
            bool swappedBodiesCollisionData = collisionData->getBody1()->getEntity() != mConvexMeshBody1->getEntity();

            // Test contact points
            Vector3 localBody1Point1(3, 1, 0);
            Vector3 localBody2Point1(0, 5, 0);
            decimal penetrationDepth1 = 1.0f;
            rp3d_test(collisionData->hasContactPointSimilarTo(swappedBodiesCollisionData ? localBody2Point1 : localBody1Point1,
                                                         swappedBodiesCollisionData ? localBody1Point1 : localBody2Point1,
                                                         penetrationDepth1));
            // ----- Test collision against body 1 only ----- //

            mCollisionCallback.reset();
            mWorld->testCollision(mConvexMeshBody1, mCollisionCallback);

            rp3d_test(mCollisionCallback.areCollidersColliding(mConvexMeshCollider1, mCapsuleCollider1));

            // Get collision data
            collisionData = mCollisionCallback.getCollisionData(mConvexMeshCollider1, mCapsuleCollider1);
            rp3d_test(collisionData != nullptr);
            rp3d_test(collisionData->getNbContactPairs() == 1);
            rp3d_test(collisionData->getTotalNbContactPoints() == 1);

            // True if the bodies are swapped in the collision callback response
            swappedBodiesCollisionData = collisionData->getBody1()->getEntity() != mConvexMeshBody1->getEntity();

            // Test contact points
            rp3d_test(collisionData->hasContactPointSimilarTo(swappedBodiesCollisionData ? localBody2Point1 : localBody1Point1,
                                                         swappedBodiesCollisionData ? localBody1Point1 : localBody2Point1,
                                                         penetrationDepth1));

            // ----- Test collision against body 2 only ----- //

            mCollisionCallback.reset();
            mWorld->testCollision(mCapsuleBody1, mCollisionCallback);

            rp3d_test(mCollisionCallback.areCollidersColliding(mConvexMeshCollider1, mCapsuleCollider1));

            // Get collision data
            collisionData = mCollisionCallback.getCollisionData(mConvexMeshCollider1, mCapsuleCollider1);
            rp3d_test(collisionData != nullptr);
            rp3d_test(collisionData->getNbContactPairs() == 1);
            rp3d_test(collisionData->getTotalNbContactPoints() == 1);

            // True if the bodies are swapped in the collision callback response
            swappedBodiesCollisionData = collisionData->getBody1()->getEntity() != mConvexMeshBody1->getEntity();

            // Test contact points
            rp3d_test(collisionData->hasContactPointSimilarTo(swappedBodiesCollisionData ? localBody2Point1 : localBody1Point1,
                                                         swappedBodiesCollisionData ? localBody1Point1 : localBody2Point1,
                                                         penetrationDepth1));

            // ----- Test collision against selected body 1 and 2 ----- //

            mCollisionCallback.reset();
            mWorld->testCollision(mConvexMeshBody1, mCapsuleBody1, mCollisionCallback);

            rp3d_test(mCollisionCallback.areCollidersColliding(mConvexMeshCollider1, mCapsuleCollider1));

            // Get collision data
            collisionData = mCollisionCallback.getCollisionData(mConvexMeshCollider1, mCapsuleCollider1);
            rp3d_test(collisionData != nullptr);
            rp3d_test(collisionData->getNbContactPairs() == 1);
            rp3d_test(collisionData->getTotalNbContactPoints() == 1);

            // True if the bodies are swapped in the collision callback response
            swappedBodiesCollisionData = collisionData->getBody1()->getEntity() != mConvexMeshBody1->getEntity();

            // Test contact points
            rp3d_test(collisionData->hasContactPointSimilarTo(swappedBodiesCollisionData ? localBody2Point1 : localBody1Point1,
                                                         swappedBodiesCollisionData ? localBody1Point1 : localBody2Point1,
                                                         penetrationDepth1));

            // reset the init transforms
            mConvexMeshBody1->setTransform(initTransform1);
            mCapsuleBody1->setTransform(initTransform2);
        }

        void testBoxVsConcaveMeshCollision() {

            Transform initTransform1 = mBoxBody1->getTransform();
            Transform initTransform2 = mConcaveMeshBody->getTransform();

            /********************************************************************************
            * Test Box vs Concave Mesh
            *********************************************************************************/

            Transform transform1(Vector3(10, 22, 50), Quaternion::identity());
            Transform transform2(Vector3(10, 20, 50), Quaternion::identity());

            // Move spheres to collide with each other
            mBoxBody1->setTransform(transform1);
            mConcaveMeshBody->setTransform(transform2);

            mOverlapCallback.reset();
            mWorld->testOverlap(mBoxBody1, mOverlapCallback);
            rp3d_test(mOverlapCallback.hasOverlapWithBody(mBoxBody1));

            mOverlapCallback.reset();
            mWorld->testOverlap(mConcaveMeshBody, mOverlapCallback);
            rp3d_test(mOverlapCallback.hasOverlapWithBody(mConcaveMeshBody));

            // ----- Test global collision test ----- //

            mCollisionCallback.reset();
            mWorld->testCollision(mCollisionCallback);

            rp3d_test(mCollisionCallback.areCollidersColliding(mBoxCollider1, mConcaveMeshCollider));

            // Get collision data
            const CollisionData* collisionData = mCollisionCallback.getCollisionData(mBoxCollider1, mConcaveMeshCollider);
            rp3d_test(collisionData != nullptr);
            rp3d_test(collisionData->getNbContactPairs() == 1);
            rp3d_test(collisionData->getTotalNbContactPoints() == 4);

            for (size_t i=0; i<collisionData->contactPairs[0].contactPoints.size(); i++) {
                rp3d_test(approxEqual(collisionData->contactPairs[0].contactPoints[i].penetrationDepth, 1.0f));
            }

            // ----- Test collision against body 1 only ----- //

            mCollisionCallback.reset();
            mWorld->testCollision(mBoxBody1, mCollisionCallback);

            rp3d_test(mCollisionCallback.areCollidersColliding(mBoxCollider1, mConcaveMeshCollider));

            // Get collision data
            collisionData = mCollisionCallback.getCollisionData(mBoxCollider1, mConcaveMeshCollider);
            rp3d_test(collisionData != nullptr);
            rp3d_test(collisionData->getNbContactPairs() == 1);
            rp3d_test(collisionData->getTotalNbContactPoints() == 4);

            for (size_t i=0; i<collisionData->contactPairs[0].contactPoints.size(); i++) {
                rp3d_test(approxEqual(collisionData->contactPairs[0].contactPoints[i].penetrationDepth, 1.0f));
            }

            // ----- Test collision against body 2 only ----- //

            mCollisionCallback.reset();
            mWorld->testCollision(mConcaveMeshBody, mCollisionCallback);

            rp3d_test(mCollisionCallback.areCollidersColliding(mBoxCollider1, mConcaveMeshCollider));

            // Get collision data
            collisionData = mCollisionCallback.getCollisionData(mBoxCollider1, mConcaveMeshCollider);
            rp3d_test(collisionData != nullptr);
            rp3d_test(collisionData->getNbContactPairs() == 1);
            rp3d_test(collisionData->getTotalNbContactPoints() == 4);

            for (size_t i=0; i<collisionData->contactPairs[0].contactPoints.size(); i++) {
                rp3d_test(approxEqual(collisionData->contactPairs[0].contactPoints[i].penetrationDepth, 1.0f));
            }

            // ----- Test collision against selected body 1 and 2 ----- //

            mCollisionCallback.reset();
            mWorld->testCollision(mBoxBody1, mConcaveMeshBody, mCollisionCallback);

            rp3d_test(mCollisionCallback.areCollidersColliding(mBoxCollider1, mConcaveMeshCollider));

            // Get collision data
            collisionData = mCollisionCallback.getCollisionData(mBoxCollider1, mConcaveMeshCollider);
            rp3d_test(collisionData != nullptr);
            rp3d_test(collisionData->getNbContactPairs() == 1);
            rp3d_test(collisionData->getTotalNbContactPoints() == 4);

            // Test contact points
            for (size_t i=0; i<collisionData->contactPairs[0].contactPoints.size(); i++) {
                rp3d_test(approxEqual(collisionData->contactPairs[0].contactPoints[i].penetrationDepth, 1.0f));
            }

            // Reset the init transforms
            mBoxBody1->setTransform(initTransform1);
            mConcaveMeshBody->setTransform(initTransform2);
        }

        void testConvexMeshVsConcaveMeshCollision() {

            Transform initTransform1 = mConvexMeshBody1->getTransform();
            Transform initTransform2 = mConcaveMeshBody->getTransform();

            /********************************************************************************
            * Test Box vs Concave Mesh
            *********************************************************************************/

            Transform transform1(Vector3(10, 22, 50), Quaternion::identity());
            Transform transform2(Vector3(10, 20, 50), Quaternion::identity());

            // Move spheres to collide with each other
            mConvexMeshBody1->setTransform(transform1);
            mConcaveMeshBody->setTransform(transform2);

            mOverlapCallback.reset();
            mWorld->testOverlap(mConvexMeshBody1, mOverlapCallback);
            rp3d_test(mOverlapCallback.hasOverlapWithBody(mConvexMeshBody1));

            mOverlapCallback.reset();
            mWorld->testOverlap(mConcaveMeshBody, mOverlapCallback);
            rp3d_test(mOverlapCallback.hasOverlapWithBody(mConcaveMeshBody));

            // ----- Test global collision test ----- //

            mCollisionCallback.reset();
            mWorld->testCollision(mCollisionCallback);

            rp3d_test(mCollisionCallback.areCollidersColliding(mConvexMeshCollider1, mConcaveMeshCollider));

            // Get collision data
            const CollisionData* collisionData = mCollisionCallback.getCollisionData(mConvexMeshCollider1, mConcaveMeshCollider);
            rp3d_test(collisionData != nullptr);
            rp3d_test(collisionData->getNbContactPairs() == 1);
            rp3d_test(collisionData->getTotalNbContactPoints() == 4);

            for (size_t i=0; i<collisionData->contactPairs[0].contactPoints.size(); i++) {
                rp3d_test(approxEqual(collisionData->contactPairs[0].contactPoints[i].penetrationDepth, 1.0f));
            }

            // ----- Test collision against body 1 only ----- //

            mCollisionCallback.reset();
            mWorld->testCollision(mConvexMeshBody1, mCollisionCallback);

            rp3d_test(mCollisionCallback.areCollidersColliding(mConvexMeshCollider1, mConcaveMeshCollider));

            // Get collision data
            collisionData = mCollisionCallback.getCollisionData(mConvexMeshCollider1, mConcaveMeshCollider);
            rp3d_test(collisionData != nullptr);
            rp3d_test(collisionData->getNbContactPairs() == 1);
            rp3d_test(collisionData->getTotalNbContactPoints() == 4);

            for (size_t i=0; i<collisionData->contactPairs[0].contactPoints.size(); i++) {
                rp3d_test(approxEqual(collisionData->contactPairs[0].contactPoints[i].penetrationDepth, 1.0f));
            }

            // ----- Test collision against body 2 only ----- //

            mCollisionCallback.reset();
            mWorld->testCollision(mConcaveMeshBody, mCollisionCallback);

            rp3d_test(mCollisionCallback.areCollidersColliding(mConvexMeshCollider1, mConcaveMeshCollider));

            // Get collision data
            collisionData = mCollisionCallback.getCollisionData(mConvexMeshCollider1, mConcaveMeshCollider);
            rp3d_test(collisionData != nullptr);
            rp3d_test(collisionData->getNbContactPairs() == 1);
            rp3d_test(collisionData->getTotalNbContactPoints() == 4);

            for (size_t i=0; i<collisionData->contactPairs[0].contactPoints.size(); i++) {
                rp3d_test(approxEqual(collisionData->contactPairs[0].contactPoints[i].penetrationDepth, 1.0f));
            }

            // ----- Test collision against selected body 1 and 2 ----- //

            mCollisionCallback.reset();
            mWorld->testCollision(mConvexMeshBody1, mConcaveMeshBody, mCollisionCallback);

            rp3d_test(mCollisionCallback.areCollidersColliding(mConvexMeshCollider1, mConcaveMeshCollider));

            // Get collision data
            collisionData = mCollisionCallback.getCollisionData(mConvexMeshCollider1, mConcaveMeshCollider);
            rp3d_test(collisionData != nullptr);
            rp3d_test(collisionData->getNbContactPairs() == 1);
            rp3d_test(collisionData->getTotalNbContactPoints() == 4);

            // Test contact points
            for (size_t i=0; i<collisionData->contactPairs[0].contactPoints.size(); i++) {
                rp3d_test(approxEqual(collisionData->contactPairs[0].contactPoints[i].penetrationDepth, 1.0f));
            }

            // Reset the init transforms
            mConvexMeshBody1->setTransform(initTransform1);
            mConcaveMeshBody->setTransform(initTransform2);
        }

        void testCapsuleVsCapsuleCollision() {

            Transform initTransform1 = mCapsuleBody1->getTransform();
            Transform initTransform2 = mCapsuleBody2->getTransform();

            /********************************************************************************
            * Test Capsule (sphere cap) vs Capsule (sphere cap) collision                                             *
            *********************************************************************************/

            Transform transform1(Vector3(10, 20, 50), Quaternion::identity());
            Transform transform2(Vector3(16, 23, 50), Quaternion::fromEulerAngles(0, 0, rp3d::PI * 0.5f));

            // Move spheres to collide with each other
            mCapsuleBody1->setTransform(transform1);
            mCapsuleBody2->setTransform(transform2);

            mOverlapCallback.reset();
            mWorld->testOverlap(mCapsuleBody1, mOverlapCallback);
            rp3d_test(mOverlapCallback.hasOverlapWithBody(mCapsuleBody1));

            mOverlapCallback.reset();
            mWorld->testOverlap(mCapsuleBody2, mOverlapCallback);
            rp3d_test(mOverlapCallback.hasOverlapWithBody(mCapsuleBody2));

            // ----- Test global collision test ----- //

            mCollisionCallback.reset();
            mWorld->testCollision(mCollisionCallback);

            rp3d_test(mCollisionCallback.areCollidersColliding(mCapsuleCollider1, mCapsuleCollider2));

            // Get collision data
            const CollisionData* collisionData = mCollisionCallback.getCollisionData(mCapsuleCollider1, mCapsuleCollider2);
            rp3d_test(collisionData != nullptr);
            rp3d_test(collisionData->getNbContactPairs() == 1);
            rp3d_test(collisionData->getTotalNbContactPoints() == 1);

            // True if the bodies are swapped in the collision callback response
            bool swappedBodiesCollisionData = collisionData->getBody1()->getEntity() != mCapsuleBody1->getEntity();

            // Test contact points
            Vector3 localBody1Point(2, 3, 0);
            Vector3 localBody2Point(0, 5, 0);
            decimal penetrationDepth = 1.0f;
            rp3d_test(collisionData->hasContactPointSimilarTo(swappedBodiesCollisionData ? localBody2Point : localBody1Point,
                swappedBodiesCollisionData ? localBody1Point : localBody2Point,
                penetrationDepth));

            // ----- Test collision against body 1 only ----- //

            mCollisionCallback.reset();
            mWorld->testCollision(mCapsuleBody1, mCollisionCallback);

            rp3d_test(mCollisionCallback.areCollidersColliding(mCapsuleCollider1, mCapsuleCollider2));

            // Get collision data
            collisionData = mCollisionCallback.getCollisionData(mCapsuleCollider1, mCapsuleCollider2);
            rp3d_test(collisionData != nullptr);
            rp3d_test(collisionData->getNbContactPairs() == 1);
            rp3d_test(collisionData->getTotalNbContactPoints() == 1);

            // True if the bodies are swapped in the collision callback response
            swappedBodiesCollisionData = collisionData->getBody1()->getEntity() != mCapsuleBody1->getEntity();

            // Test contact points
            rp3d_test(collisionData->hasContactPointSimilarTo(swappedBodiesCollisionData ? localBody2Point : localBody1Point,
                swappedBodiesCollisionData ? localBody1Point : localBody2Point,
                penetrationDepth));

            // ----- Test collision against body 2 only ----- //

            mCollisionCallback.reset();
            mWorld->testCollision(mCapsuleBody2, mCollisionCallback);

            rp3d_test(mCollisionCallback.areCollidersColliding(mCapsuleCollider1, mCapsuleCollider2));

            // Get collision data
            collisionData = mCollisionCallback.getCollisionData(mCapsuleCollider1, mCapsuleCollider2);
            rp3d_test(collisionData != nullptr);
            rp3d_test(collisionData->getNbContactPairs() == 1);
            rp3d_test(collisionData->getTotalNbContactPoints() == 1);

            // True if the bodies are swapped in the collision callback response
            swappedBodiesCollisionData = collisionData->getBody1()->getEntity() != mCapsuleBody1->getEntity();

            // Test contact points
            rp3d_test(collisionData->hasContactPointSimilarTo(swappedBodiesCollisionData ? localBody2Point : localBody1Point,
                swappedBodiesCollisionData ? localBody1Point : localBody2Point,
                penetrationDepth));

            // ----- Test collision against selected body 1 and 2 ----- //

            mCollisionCallback.reset();
            mWorld->testCollision(mCapsuleBody1, mCapsuleBody2, mCollisionCallback);

            rp3d_test(mCollisionCallback.areCollidersColliding(mCapsuleCollider1, mCapsuleCollider2));

            // Get collision data
            collisionData = mCollisionCallback.getCollisionData(mCapsuleCollider1, mCapsuleCollider2);
            rp3d_test(collisionData != nullptr);
            rp3d_test(collisionData->getNbContactPairs() == 1);
            rp3d_test(collisionData->getTotalNbContactPoints() == 1);

            // True if the bodies are swapped in the collision callback response
            swappedBodiesCollisionData = collisionData->getBody1()->getEntity() != mCapsuleBody1->getEntity();

            // Test contact points
            rp3d_test(collisionData->hasContactPointSimilarTo(swappedBodiesCollisionData ? localBody2Point : localBody1Point,
                swappedBodiesCollisionData ? localBody1Point : localBody2Point,
                penetrationDepth));

            /********************************************************************************
            * Test Capsule (sphere cap) vs Capsule (cylinder side) collision                          *
            *********************************************************************************/

            transform1 = Transform(Vector3(10, 20, 50), Quaternion::identity());
            transform2 = Transform(Vector3(10, 27, 50), Quaternion::fromEulerAngles(0, 0, rp3d::PI * 0.5f));

            // Move spheres to collide with each other
            mCapsuleBody1->setTransform(transform1);
            mCapsuleBody2->setTransform(transform2);

            mOverlapCallback.reset();
            mWorld->testOverlap(mCapsuleBody1, mOverlapCallback);
            rp3d_test(mOverlapCallback.hasOverlapWithBody(mCapsuleBody1));

            mOverlapCallback.reset();
            mWorld->testOverlap(mCapsuleBody2, mOverlapCallback);
            rp3d_test(mOverlapCallback.hasOverlapWithBody(mCapsuleBody2));

            // ----- Test global collision test ----- //

            mCollisionCallback.reset();
            mWorld->testCollision(mCollisionCallback);

            rp3d_test(mCollisionCallback.areCollidersColliding(mCapsuleCollider1, mCapsuleCollider2));

            // Get collision data
            collisionData = mCollisionCallback.getCollisionData(mCapsuleCollider1, mCapsuleCollider2);
            rp3d_test(collisionData != nullptr);
            rp3d_test(collisionData->getNbContactPairs() == 1);
            rp3d_test(collisionData->getTotalNbContactPoints() == 1);

            // True if the bodies are swapped in the collision callback response
            swappedBodiesCollisionData = collisionData->getBody1()->getEntity() != mCapsuleBody1->getEntity();

            // Test contact points
            localBody1Point = Vector3(0, 5, 0);
            localBody2Point = Vector3(-3, 0, 0);
            penetrationDepth = decimal(1.0);
            rp3d_test(collisionData->hasContactPointSimilarTo(swappedBodiesCollisionData ? localBody2Point : localBody1Point,
                swappedBodiesCollisionData ? localBody1Point : localBody2Point,
                penetrationDepth));

            // ----- Test collision against body 1 only ----- //

            mCollisionCallback.reset();
            mWorld->testCollision(mCapsuleBody1, mCollisionCallback);

            rp3d_test(mCollisionCallback.areCollidersColliding(mCapsuleCollider1, mCapsuleCollider2));

            // Get collision data
            collisionData = mCollisionCallback.getCollisionData(mCapsuleCollider1, mCapsuleCollider2);
            rp3d_test(collisionData != nullptr);
            rp3d_test(collisionData->getNbContactPairs() == 1);
            rp3d_test(collisionData->getTotalNbContactPoints() == 1);

            // True if the bodies are swapped in the collision callback response
            swappedBodiesCollisionData = collisionData->getBody1()->getEntity() != mCapsuleBody1->getEntity();

            // Test contact points
            rp3d_test(collisionData->hasContactPointSimilarTo(swappedBodiesCollisionData ? localBody2Point : localBody1Point,
                swappedBodiesCollisionData ? localBody1Point : localBody2Point,
                penetrationDepth));

            // ----- Test collision against body 2 only ----- //

            mCollisionCallback.reset();
            mWorld->testCollision(mCapsuleBody2, mCollisionCallback);

            rp3d_test(mCollisionCallback.areCollidersColliding(mCapsuleCollider1, mCapsuleCollider2));

            // Get collision data
            collisionData = mCollisionCallback.getCollisionData(mCapsuleCollider1, mCapsuleCollider2);
            rp3d_test(collisionData != nullptr);
            rp3d_test(collisionData->getNbContactPairs() == 1);
            rp3d_test(collisionData->getTotalNbContactPoints() == 1);

            // True if the bodies are swapped in the collision callback response
            swappedBodiesCollisionData = collisionData->getBody1()->getEntity() != mCapsuleBody1->getEntity();

            // Test contact points
            rp3d_test(collisionData->hasContactPointSimilarTo(swappedBodiesCollisionData ? localBody2Point : localBody1Point,
                swappedBodiesCollisionData ? localBody1Point : localBody2Point,
                penetrationDepth));

            // ----- Test collision against selected body 1 and 2 ----- //

            mCollisionCallback.reset();
            mWorld->testCollision(mCapsuleBody1, mCapsuleBody2, mCollisionCallback);

            rp3d_test(mCollisionCallback.areCollidersColliding(mCapsuleCollider1, mCapsuleCollider2));

            // Get collision data
            collisionData = mCollisionCallback.getCollisionData(mCapsuleCollider1, mCapsuleCollider2);
            rp3d_test(collisionData != nullptr);
            rp3d_test(collisionData->getNbContactPairs() == 1);
            rp3d_test(collisionData->getTotalNbContactPoints() == 1);

            // True if the bodies are swapped in the collision callback response
            swappedBodiesCollisionData = collisionData->getBody1()->getEntity() != mCapsuleBody1->getEntity();

            // Test contact points
            rp3d_test(collisionData->hasContactPointSimilarTo(swappedBodiesCollisionData ? localBody2Point : localBody1Point,
                swappedBodiesCollisionData ? localBody1Point : localBody2Point,
                penetrationDepth));

            // Reset the init transforms
            mCapsuleBody1->setTransform(initTransform1);
            mCapsuleBody2->setTransform(initTransform2);
        }

        void testCapsuleVsConcaveMeshCollision() {

            Transform initTransform1 = mCapsuleBody1->getTransform();
            Transform initTransform2 = mConcaveMeshBody->getTransform();

            /********************************************************************************
            * Test Capsule vs Concave Mesh
            *********************************************************************************/

            Transform transform1(Vector3(10, 24.98f, 50), Quaternion::identity());
            Transform transform2(Vector3(10, 20, 50), Quaternion::identity());

            // Move spheres to collide with each other
            mCapsuleBody1->setTransform(transform1);
            mConcaveMeshBody->setTransform(transform2);

            mOverlapCallback.reset();
            mWorld->testOverlap(mCapsuleBody1, mOverlapCallback);
            rp3d_test(mOverlapCallback.hasOverlapWithBody(mCapsuleBody1));

            mOverlapCallback.reset();
            mWorld->testOverlap(mConcaveMeshBody, mOverlapCallback);
            rp3d_test(mOverlapCallback.hasOverlapWithBody(mConcaveMeshBody));

            // ----- Test global collision test ----- //

            mCollisionCallback.reset();
            mWorld->testCollision(mCollisionCallback);

            rp3d_test(mCollisionCallback.areCollidersColliding(mCapsuleCollider1, mConcaveMeshCollider));

            // Get collision data
            const CollisionData* collisionData = mCollisionCallback.getCollisionData(mCapsuleCollider1, mConcaveMeshCollider);
            rp3d_test(collisionData != nullptr);
            rp3d_test(collisionData->getNbContactPairs() == 1);
            rp3d_test(collisionData->getTotalNbContactPoints() == 1);

            // True if the bodies are swapped in the collision callback response
            bool swappedBodiesCollisionData = collisionData->getBody1()->getEntity() != mCapsuleBody1->getEntity();

            // Test contact points
            Vector3 localBody1Point(0, -5, 0);
            Vector3 localBody2Point(0, 0, 0);
            decimal penetrationDepth = 0.02f;
            rp3d_test(collisionData->hasContactPointSimilarTo(swappedBodiesCollisionData ? localBody2Point : localBody1Point,
                                                         swappedBodiesCollisionData ? localBody1Point : localBody2Point,
                                                         penetrationDepth));

            // ----- Test collision against body 1 only ----- //

            mCollisionCallback.reset();
            mWorld->testCollision(mCapsuleBody1, mCollisionCallback);

            rp3d_test(mCollisionCallback.areCollidersColliding(mCapsuleCollider1, mConcaveMeshCollider));

            // Get collision data
            collisionData = mCollisionCallback.getCollisionData(mCapsuleCollider1, mConcaveMeshCollider);
            rp3d_test(collisionData != nullptr);
            rp3d_test(collisionData->getNbContactPairs() == 1);
            rp3d_test(collisionData->getTotalNbContactPoints() == 1);

            // True if the bodies are swapped in the collision callback response
            swappedBodiesCollisionData = collisionData->getBody1()->getEntity() != mCapsuleBody1->getEntity();

            // Test contact points
            rp3d_test(collisionData->hasContactPointSimilarTo(swappedBodiesCollisionData ? localBody2Point : localBody1Point,
                                                         swappedBodiesCollisionData ? localBody1Point : localBody2Point,
                                                         penetrationDepth));

            // ----- Test collision against body 2 only ----- //

            mCollisionCallback.reset();
            mWorld->testCollision(mConcaveMeshBody, mCollisionCallback);

            rp3d_test(mCollisionCallback.areCollidersColliding(mCapsuleCollider1, mConcaveMeshCollider));

            // Get collision data
            collisionData = mCollisionCallback.getCollisionData(mCapsuleCollider1, mConcaveMeshCollider);
            rp3d_test(collisionData != nullptr);
            rp3d_test(collisionData->getNbContactPairs() == 1);
            rp3d_test(collisionData->getTotalNbContactPoints() == 1);

            // True if the bodies are swapped in the collision callback response
            swappedBodiesCollisionData = collisionData->getBody1()->getEntity() != mCapsuleBody1->getEntity();

            // Test contact points
            rp3d_test(collisionData->hasContactPointSimilarTo(swappedBodiesCollisionData ? localBody2Point : localBody1Point,
                                                         swappedBodiesCollisionData ? localBody1Point : localBody2Point,
                                                         penetrationDepth));

            // ----- Test collision against selected body 1 and 2 ----- //

            mCollisionCallback.reset();
            mWorld->testCollision(mCapsuleBody1, mConcaveMeshBody, mCollisionCallback);

            rp3d_test(mCollisionCallback.areCollidersColliding(mCapsuleCollider1, mConcaveMeshCollider));

            // Get collision data
            collisionData = mCollisionCallback.getCollisionData(mCapsuleCollider1, mConcaveMeshCollider);
            rp3d_test(collisionData != nullptr);
            rp3d_test(collisionData->getNbContactPairs() == 1);
            rp3d_test(collisionData->getTotalNbContactPoints() == 1);

            // True if the bodies are swapped in the collision callback response
            swappedBodiesCollisionData = collisionData->getBody1()->getEntity() != mCapsuleBody1->getEntity();

            // Test contact points
            rp3d_test(collisionData->hasContactPointSimilarTo(swappedBodiesCollisionData ? localBody2Point : localBody1Point,
                                                         swappedBodiesCollisionData ? localBody1Point : localBody2Point,
                                                         penetrationDepth));

            // Reset the init transforms
            mCapsuleBody1->setTransform(initTransform1);
            mConcaveMeshBody->setTransform(initTransform2);
        }
 };

}

#endif
