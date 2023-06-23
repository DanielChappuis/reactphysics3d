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

#ifndef TEST_RAYCAST_H
#define TEST_RAYCAST_H

// Libraries
#include "Test.h"
#include <reactphysics3d/engine/PhysicsCommon.h>
#include <reactphysics3d/engine/PhysicsWorld.h>
#include <reactphysics3d/body/Body.h>
#include <reactphysics3d/collision/shapes/BoxShape.h>
#include <reactphysics3d/collision/shapes/SphereShape.h>
#include <reactphysics3d/collision/shapes/CapsuleShape.h>
#include <reactphysics3d/collision/shapes/ConvexMeshShape.h>
#include <reactphysics3d/collision/shapes/TriangleShape.h>
#include <reactphysics3d/collision/shapes/ConcaveMeshShape.h>
#include <reactphysics3d/collision/shapes/HeightFieldShape.h>
#include <reactphysics3d/collision/TriangleMesh.h>
#include <reactphysics3d/collision/TriangleVertexArray.h>
#include <reactphysics3d/collision/RaycastInfo.h>
#include <reactphysics3d/collision/PolygonVertexArray.h>
#include <vector>

/// Reactphysics3D namespace
namespace reactphysics3d {

// Enumeration for categories
enum Category {
    CATEGORY1 = 0x0001,
    CATEGORY2 = 0x0002
};

/// Class WorldRaycastCallback
class WorldRaycastCallback : public RaycastCallback {

    public:

        RaycastInfo raycastInfo;
        Collider* shapeToTest;
        bool isHit;

        WorldRaycastCallback() {
            isHit = false;
            shapeToTest = nullptr;
        }

        virtual decimal notifyRaycastHit(const RaycastInfo& info) override {

            if (shapeToTest->getBody()->getEntity() == info.body->getEntity()) {
                raycastInfo.body = info.body;
                raycastInfo.hitFraction = info.hitFraction;
                raycastInfo.collider = info.collider;
                raycastInfo.worldNormal = info.worldNormal;
                raycastInfo.worldPoint = info.worldPoint;
                isHit = true;
            }

            // Return a fraction of 1.0 because we need to gather all hits
            return decimal(1.0);
        }

        void reset() {
            raycastInfo.body = nullptr;
            raycastInfo.hitFraction = decimal(0.0);
            raycastInfo.collider = nullptr;
            raycastInfo.worldNormal.setToZero();
            raycastInfo.worldPoint.setToZero();
            isHit = false;
        }
};

// Class TestPointInside
/**
 * Unit test for the RigidBody::testPointInside() method.
 */
class TestRaycast : public Test {

    private :

        // ---------- Atributes ---------- //

        PhysicsCommon mPhysicsCommon;

        // Raycast callback class
        WorldRaycastCallback mCallback;

        DefaultAllocator mAllocator;

        // Epsilon
        decimal epsilon;

        // Physics world
        PhysicsWorld* mWorld;

        // Bodies
        Body* mBoxBody;
        Body* mSphereBody;
        Body* mCapsuleBody;
        Body* mConvexMeshBody;
        Body* mCylinderBody;
        Body* mCompoundBody;
        Body* mConcaveMeshBody;
        Body* mHeightFieldBody;

        // Transform
        Transform mBodyTransform;
        Transform mShapeTransform;
        Transform mLocalShapeToWorld;
        Transform mLocalShape2ToWorld;

        // Collision shapes
        BoxShape* mBoxShape;
        SphereShape* mSphereShape;
        CapsuleShape* mCapsuleShape;
        ConvexMeshShape* mConvexMeshShape;
        ConcaveMeshShape* mConcaveMeshShape;
        HeightFieldShape* mHeightFieldShape;

        // Collider
        Collider* mBoxCollider;
        Collider* mSphereCollider;
        Collider* mCapsuleCollider;
        Collider* mConvexMeshCollider;
        Collider* mCompoundSphereCollider;
        Collider* mCompoundCapsuleCollider;
        Collider* mConcaveMeshCollider;
        Collider* mHeightFieldCollider;

        // Triangle meshes
        TriangleMesh* mConcaveTriangleMesh;

        std::vector<Vector3> mConcaveMeshVertices;
        std::vector<uint> mConcaveMeshIndices;
        float mHeightFieldData[100];
        ConvexMesh* mConvexMesh;
        HeightField* mHeightField;
        float mConvexMeshVertices[8 * 3];
        int mConvexMeshIndices[4 * 6];

    public :

        // ---------- Methods ---------- //

        /// Constructor
        TestRaycast(const std::string& name) : Test(name) {

            epsilon = decimal(0.0001);

            // Create the world
            mWorld = mPhysicsCommon.createPhysicsWorld();

            // Body transform
            Vector3 position(-3, 2, 7);
            Quaternion orientation = Quaternion::fromEulerAngles(PI_RP3D / 5, PI_RP3D / 6, PI_RP3D / 7);
            mBodyTransform = Transform(position, orientation);

            // Create the bodies
            mBoxBody = mWorld->createRigidBody(mBodyTransform);
            mSphereBody = mWorld->createRigidBody(mBodyTransform);
            mCapsuleBody = mWorld->createRigidBody(mBodyTransform);
            mConvexMeshBody = mWorld->createRigidBody(mBodyTransform);
            mCylinderBody = mWorld->createRigidBody(mBodyTransform);
            mCompoundBody = mWorld->createRigidBody(mBodyTransform);
            mConcaveMeshBody = mWorld->createRigidBody(mBodyTransform);
            mHeightFieldBody = mWorld->createRigidBody(mBodyTransform);

            // Collision shape transform
            Vector3 shapePosition(1, -4, -3);
            Quaternion shapeOrientation = Quaternion::fromEulerAngles(3 * PI_RP3D / 6 , -PI_RP3D / 8, PI_RP3D / 3);
            mShapeTransform = Transform(shapePosition, shapeOrientation);

            // Compute the the transform from a local shape point to world-space
            mLocalShapeToWorld = mBodyTransform * mShapeTransform;

            // Create collision shapes
            mBoxShape = mPhysicsCommon.createBoxShape(Vector3(2, 3, 4));
            mBoxCollider = mBoxBody->addCollider(mBoxShape, mShapeTransform);
            mBoxCollider->setIsSimulationCollider(false);

            mSphereShape = mPhysicsCommon.createSphereShape(3);
            mSphereCollider = mSphereBody->addCollider(mSphereShape, mShapeTransform);
            mSphereCollider->setIsSimulationCollider(false);

            mCapsuleShape = mPhysicsCommon.createCapsuleShape(2, 5);
            mCapsuleCollider = mCapsuleBody->addCollider(mCapsuleShape, mShapeTransform);
            mCapsuleCollider->setIsSimulationCollider(false);

            mConvexMeshVertices[0] = -2; mConvexMeshVertices[1] = -3; mConvexMeshVertices[2] = 4;
            mConvexMeshVertices[3] = 2; mConvexMeshVertices[4] = -3; mConvexMeshVertices[5] = 4;
            mConvexMeshVertices[6] = 2; mConvexMeshVertices[7] = -3; mConvexMeshVertices[8] = -4;
            mConvexMeshVertices[9] = -2; mConvexMeshVertices[10] = -3; mConvexMeshVertices[11] = -4;
            mConvexMeshVertices[12] = -2; mConvexMeshVertices[13] = 3; mConvexMeshVertices[14] = 4;
            mConvexMeshVertices[15] = 2; mConvexMeshVertices[16] = 3; mConvexMeshVertices[17] = 4;
            mConvexMeshVertices[18] = 2; mConvexMeshVertices[19] = 3; mConvexMeshVertices[20] = -4;
            mConvexMeshVertices[21] = -2; mConvexMeshVertices[22] = 3; mConvexMeshVertices[23] = -4;

            mConvexMeshIndices[0] = 0; mConvexMeshIndices[1] = 3; mConvexMeshIndices[2] = 2; mConvexMeshIndices[3] = 1;
            mConvexMeshIndices[4] = 4; mConvexMeshIndices[5] = 5; mConvexMeshIndices[6] = 6; mConvexMeshIndices[7] = 7;
            mConvexMeshIndices[8] = 0; mConvexMeshIndices[9] = 1; mConvexMeshIndices[10] = 5; mConvexMeshIndices[11] = 4;
            mConvexMeshIndices[12] = 1; mConvexMeshIndices[13] = 2; mConvexMeshIndices[14] = 6; mConvexMeshIndices[15] = 5;
            mConvexMeshIndices[16] = 2; mConvexMeshIndices[17] = 3; mConvexMeshIndices[18] = 7; mConvexMeshIndices[19] = 6;
            mConvexMeshIndices[20] = 0; mConvexMeshIndices[21] = 4; mConvexMeshIndices[22] = 7; mConvexMeshIndices[23] = 3;

            // Polygon faces descriptions for the convex mesh
            PolygonVertexArray::PolygonFace polygonFaces[6];
            PolygonVertexArray::PolygonFace* face = polygonFaces;
            for (int f = 0; f < 6; f++) {
                face->indexBase = f * 4;
                face->nbVertices = 4;
                face++;
            }

            // Create the polygon vertex array
            PolygonVertexArray polygonVertexArray(8, mConvexMeshVertices, 3 * sizeof(float),
                                         mConvexMeshIndices, sizeof(int), 6, polygonFaces,
                                         PolygonVertexArray::VertexDataType::VERTEX_FLOAT_TYPE,
                                         PolygonVertexArray::IndexDataType::INDEX_INTEGER_TYPE);

            std::vector<Message> errors;
            mConvexMesh = mPhysicsCommon.createConvexMesh(polygonVertexArray, errors);
            rp3d_test(mConvexMesh != nullptr);
            mConvexMeshShape = mPhysicsCommon.createConvexMeshShape(mConvexMesh);
            mConvexMeshCollider = mConvexMeshBody->addCollider(mConvexMeshShape, mShapeTransform);
            mConvexMeshCollider->setIsSimulationCollider(false);

            // Compound shape is a cylinder and a sphere
            Vector3 positionShape2(Vector3(4, 2, -3));
            Quaternion orientationShape2 = Quaternion::fromEulerAngles(-3 * PI_RP3D / 8, 1.5 * PI_RP3D/ 3, PI_RP3D / 13);
            Transform shapeTransform2(positionShape2, orientationShape2);
            mLocalShape2ToWorld = mBodyTransform * shapeTransform2;
            mCompoundCapsuleCollider = mCompoundBody->addCollider(mCapsuleShape, mShapeTransform);
            mCompoundSphereCollider = mCompoundBody->addCollider(mSphereShape, shapeTransform2);
            mCompoundCapsuleCollider->setIsSimulationCollider(false);
            mCompoundSphereCollider->setIsSimulationCollider(false);

            // Concave Mesh shape
            mConcaveMeshVertices.push_back(Vector3(-2, -3, -4));
            mConcaveMeshVertices.push_back(Vector3(2, -3, -4));
            mConcaveMeshVertices.push_back(Vector3(2, -3, 4));
            mConcaveMeshVertices.push_back(Vector3(-2, -3, 4));
            mConcaveMeshVertices.push_back(Vector3(-2, 3, -4));
            mConcaveMeshVertices.push_back(Vector3(2, 3, -4));
            mConcaveMeshVertices.push_back(Vector3(2, 3, 4));
            mConcaveMeshVertices.push_back(Vector3(-2, 3, 4));

            mConcaveMeshIndices.push_back(0); mConcaveMeshIndices.push_back(1); mConcaveMeshIndices.push_back(2);
            mConcaveMeshIndices.push_back(0); mConcaveMeshIndices.push_back(2); mConcaveMeshIndices.push_back(3);
            mConcaveMeshIndices.push_back(1); mConcaveMeshIndices.push_back(5); mConcaveMeshIndices.push_back(2);
            mConcaveMeshIndices.push_back(2); mConcaveMeshIndices.push_back(5); mConcaveMeshIndices.push_back(6);
            mConcaveMeshIndices.push_back(2); mConcaveMeshIndices.push_back(7); mConcaveMeshIndices.push_back(3);
            mConcaveMeshIndices.push_back(2); mConcaveMeshIndices.push_back(6); mConcaveMeshIndices.push_back(7);
            mConcaveMeshIndices.push_back(0); mConcaveMeshIndices.push_back(3); mConcaveMeshIndices.push_back(4);
            mConcaveMeshIndices.push_back(3); mConcaveMeshIndices.push_back(7); mConcaveMeshIndices.push_back(4);
            mConcaveMeshIndices.push_back(0); mConcaveMeshIndices.push_back(4); mConcaveMeshIndices.push_back(1);
            mConcaveMeshIndices.push_back(1); mConcaveMeshIndices.push_back(4); mConcaveMeshIndices.push_back(5);
            mConcaveMeshIndices.push_back(5); mConcaveMeshIndices.push_back(7); mConcaveMeshIndices.push_back(6);
            mConcaveMeshIndices.push_back(4); mConcaveMeshIndices.push_back(7); mConcaveMeshIndices.push_back(5);
            TriangleVertexArray::VertexDataType vertexType = sizeof(decimal) == 4 ? TriangleVertexArray::VertexDataType::VERTEX_FLOAT_TYPE :
                                                                                    TriangleVertexArray::VertexDataType::VERTEX_DOUBLE_TYPE;
            TriangleVertexArray concaveMeshVertexArray(8, &(mConcaveMeshVertices[0]), sizeof(Vector3),
                                                  12, &(mConcaveMeshIndices[0]), 3 * sizeof(uint),
                                                  vertexType, TriangleVertexArray::IndexDataType::INDEX_INTEGER_TYPE);

            // Add the triangle vertex array to the triangle mesh
            errors.clear();
            mConcaveTriangleMesh = mPhysicsCommon.createTriangleMesh(concaveMeshVertexArray, errors);
            mConcaveMeshShape = mPhysicsCommon.createConcaveMeshShape(mConcaveTriangleMesh);
            mConcaveMeshCollider = mConcaveMeshBody->addCollider(mConcaveMeshShape, mShapeTransform);
            mConcaveMeshCollider->setIsSimulationCollider(false);

            // Heightfield shape (plane height field at height=4)
            for (int i=0; i<100; i++) mHeightFieldData[i] = 4;
            std::vector<Message> messages;
            mHeightField = mPhysicsCommon.createHeightField(10, 10, mHeightFieldData, HeightField::HeightDataType::HEIGHT_FLOAT_TYPE, messages);
            rp3d_test(mHeightField != nullptr);

            mHeightFieldShape = mPhysicsCommon.createHeightFieldShape(mHeightField);
            mHeightFieldCollider = mHeightFieldBody->addCollider(mHeightFieldShape, mShapeTransform);
            mHeightFieldCollider->setIsSimulationCollider(false);

            // Assign colliders to the different categories
            mBoxCollider->setCollisionCategoryBits(CATEGORY1);
            mSphereCollider->setCollisionCategoryBits(CATEGORY1);
            mCapsuleCollider->setCollisionCategoryBits(CATEGORY1);
            mConvexMeshCollider->setCollisionCategoryBits(CATEGORY2);
            mCompoundSphereCollider->setCollisionCategoryBits(CATEGORY2);
            mCompoundCapsuleCollider->setCollisionCategoryBits(CATEGORY2);
            mConcaveMeshCollider->setCollisionCategoryBits(CATEGORY2);
            mHeightFieldCollider->setCollisionCategoryBits(CATEGORY2);
        }

        /// Destructor
        virtual ~TestRaycast() {

            mPhysicsCommon.destroyPhysicsWorld(mWorld);
            mPhysicsCommon.destroyBoxShape(mBoxShape);
            mPhysicsCommon.destroySphereShape(mSphereShape);
            mPhysicsCommon.destroyCapsuleShape(mCapsuleShape);
            mPhysicsCommon.destroyConvexMeshShape(mConvexMeshShape);
            mPhysicsCommon.destroyConcaveMeshShape(mConcaveMeshShape);
            mPhysicsCommon.destroyHeightFieldShape(mHeightFieldShape);
            mPhysicsCommon.destroyConvexMesh(mConvexMesh);
            mPhysicsCommon.destroyHeightField(mHeightField);
        }

        /// Run the tests
        void run() {
            testBox();
            testSphere();
            testCapsule();
            testConvexMesh();
            testCompound();
            testConcaveMesh();
            testHeightField();
        }

        /// Test the Collider::raycast(), RigidBody::raycast() and
        /// PhysicsWorld::raycast() methods.
        void testBox() {

            // ----- Test feedback data ----- //
            Vector3 point1 = mLocalShapeToWorld * Vector3(1 , 2, 10);
            Vector3 point2 = mLocalShapeToWorld * Vector3(1, 2, -20);
            Ray ray(point1, point2);
            Vector3 hitPoint = mLocalShapeToWorld * Vector3(1, 2, 4);

            mCallback.shapeToTest = mBoxCollider;

            // PhysicsWorld::raycast()
            mCallback.reset();
            mWorld->raycast(ray, &mCallback);
            rp3d_test(mCallback.isHit);
            rp3d_test(mCallback.raycastInfo.body == mBoxBody);
            rp3d_test(mCallback.raycastInfo.collider == mBoxCollider);
            rp3d_test(approxEqual(mCallback.raycastInfo.hitFraction, decimal(0.2), epsilon));
            rp3d_test(approxEqual(mCallback.raycastInfo.worldPoint.x, hitPoint.x, epsilon));
            rp3d_test(approxEqual(mCallback.raycastInfo.worldPoint.y, hitPoint.y, epsilon));
            rp3d_test(approxEqual(mCallback.raycastInfo.worldPoint.z, hitPoint.z, epsilon));

            // Correct category filter mask
            mCallback.reset();
            mWorld->raycast(ray, &mCallback, CATEGORY1);
            rp3d_test(mCallback.isHit);

            // Wrong category filter mask
            mCallback.reset();
            mWorld->raycast(ray, &mCallback, CATEGORY2);
            rp3d_test(!mCallback.isHit);

            // RigidBody::raycast()
            RaycastInfo raycastInfo2;
            rp3d_test(mBoxBody->raycast(ray, raycastInfo2));
            rp3d_test(raycastInfo2.body == mBoxBody);
            rp3d_test(raycastInfo2.collider == mBoxCollider);
            rp3d_test(approxEqual(raycastInfo2.hitFraction, decimal(0.2), epsilon));
            rp3d_test(approxEqual(raycastInfo2.worldPoint.x, hitPoint.x, epsilon));
            rp3d_test(approxEqual(raycastInfo2.worldPoint.y, hitPoint.y, epsilon));
            rp3d_test(approxEqual(raycastInfo2.worldPoint.z, hitPoint.z, epsilon));

            // Collider::raycast()
            RaycastInfo raycastInfo3;
            rp3d_test(mBoxCollider->raycast(ray, raycastInfo3));
            rp3d_test(raycastInfo3.body == mBoxBody);
            rp3d_test(raycastInfo3.collider == mBoxCollider);
            rp3d_test(approxEqual(raycastInfo3.hitFraction, decimal(0.2), epsilon));
            rp3d_test(approxEqual(raycastInfo3.worldPoint.x, hitPoint.x, epsilon));
            rp3d_test(approxEqual(raycastInfo3.worldPoint.y, hitPoint.y, epsilon));
            rp3d_test(approxEqual(raycastInfo3.worldPoint.z, hitPoint.z, epsilon));

            Ray ray1(mLocalShapeToWorld * Vector3(0, 0, 0), mLocalShapeToWorld * Vector3(5, 7, -1));
            Ray ray2(mLocalShapeToWorld * Vector3(5, 11, 7), mLocalShapeToWorld * Vector3(17, 29, 28));
            Ray ray3(mLocalShapeToWorld * Vector3(1, 2, 3), mLocalShapeToWorld * Vector3(-11, 2, 24));
            Ray ray4(mLocalShapeToWorld * Vector3(10, 10, 10), mLocalShapeToWorld * Vector3(22, 28, 31));
            Ray ray5(mLocalShapeToWorld * Vector3(3, 1, -5), mLocalShapeToWorld * Vector3(-30, 1, -5));
            Ray ray6(mLocalShapeToWorld * Vector3(4, 4, 1), mLocalShapeToWorld * Vector3(4, -20, 1));
            Ray ray7(mLocalShapeToWorld * Vector3(1, -4, 5), mLocalShapeToWorld * Vector3(1, -4, -20));
            Ray ray8(mLocalShapeToWorld * Vector3(-4, 4, 0), mLocalShapeToWorld * Vector3(20, 4, 0));
            Ray ray9(mLocalShapeToWorld * Vector3(0, -4, -7), mLocalShapeToWorld * Vector3(0, 50, -7));
            Ray ray10(mLocalShapeToWorld * Vector3(-3, 0, -6), mLocalShapeToWorld * Vector3(-3, 0, 20));
            Ray ray11(mLocalShapeToWorld * Vector3(3, 1, 2), mLocalShapeToWorld * Vector3(-20, 1, 2));
            Ray ray12(mLocalShapeToWorld * Vector3(1, 4, -1), mLocalShapeToWorld * Vector3(1, -20, -1));
            Ray ray13(mLocalShapeToWorld * Vector3(-1, 2, 5), mLocalShapeToWorld * Vector3(-1, 2, -20));
            Ray ray14(mLocalShapeToWorld * Vector3(-3, 2, -2), mLocalShapeToWorld * Vector3(20, 2, -2));
            Ray ray15(mLocalShapeToWorld * Vector3(0, -4, 1), mLocalShapeToWorld * Vector3(0, 20, 1));
            Ray ray16(mLocalShapeToWorld * Vector3(-1, 2, -5), mLocalShapeToWorld * Vector3(-1, 2, 20));

            // ----- Test raycast miss ----- //
            rp3d_test(!mBoxBody->raycast(ray1, raycastInfo3));
            rp3d_test(!mBoxCollider->raycast(ray1, raycastInfo3));
            mCallback.reset();
            mWorld->raycast(ray1, &mCallback);
            rp3d_test(!mCallback.isHit);
            mCallback.reset();
            mWorld->raycast(Ray(ray1.point1, ray1.point2, decimal(0.01)), &mCallback);
            rp3d_test(!mCallback.isHit);
            mCallback.reset();
            mWorld->raycast(Ray(ray1.point1, ray1.point2, decimal(100.0)), &mCallback);
            rp3d_test(!mCallback.isHit);

            rp3d_test(!mBoxBody->raycast(ray2, raycastInfo3));
            rp3d_test(!mBoxCollider->raycast(ray2, raycastInfo3));
            mCallback.reset();
            mWorld->raycast(ray2, &mCallback);
            rp3d_test(!mCallback.isHit);

            rp3d_test(!mBoxBody->raycast(ray3, raycastInfo3));
            rp3d_test(!mBoxCollider->raycast(ray3, raycastInfo3));
            mCallback.reset();
            mWorld->raycast(ray3, &mCallback);
            rp3d_test(!mCallback.isHit);

            rp3d_test(!mBoxBody->raycast(ray4, raycastInfo3));
            rp3d_test(!mBoxCollider->raycast(ray4, raycastInfo3));
            mCallback.reset();
            mWorld->raycast(ray4, &mCallback);
            rp3d_test(!mCallback.isHit);

            rp3d_test(!mBoxBody->raycast(ray5, raycastInfo3));
            rp3d_test(!mBoxCollider->raycast(ray5, raycastInfo3));
            mCallback.reset();
            mWorld->raycast(ray5, &mCallback);
            rp3d_test(!mCallback.isHit);

            rp3d_test(!mBoxBody->raycast(ray6, raycastInfo3));
            rp3d_test(!mBoxCollider->raycast(ray6, raycastInfo3));
            mCallback.reset();
            mWorld->raycast(ray6, &mCallback);
            rp3d_test(!mCallback.isHit);

            rp3d_test(!mBoxBody->raycast(ray7, raycastInfo3));
            rp3d_test(!mBoxCollider->raycast(ray7, raycastInfo3));
            mCallback.reset();
            mWorld->raycast(ray7, &mCallback);
            rp3d_test(!mCallback.isHit);

            rp3d_test(!mBoxBody->raycast(ray8, raycastInfo3));
            rp3d_test(!mBoxCollider->raycast(ray8, raycastInfo3));
            mCallback.reset();
            mWorld->raycast(ray8, &mCallback);
            rp3d_test(!mCallback.isHit);

            rp3d_test(!mBoxBody->raycast(ray9, raycastInfo3));
            rp3d_test(!mBoxCollider->raycast(ray9, raycastInfo3));
            mCallback.reset();
            mWorld->raycast(ray9, &mCallback);
            rp3d_test(!mCallback.isHit);

            rp3d_test(!mBoxBody->raycast(ray10, raycastInfo3));
            rp3d_test(!mBoxCollider->raycast(ray10, raycastInfo3));
            mCallback.reset();
            mWorld->raycast(ray10, &mCallback);
            rp3d_test(!mCallback.isHit);

            mCallback.reset();
            mWorld->raycast(Ray(ray11.point1, ray11.point2, decimal(0.01)), &mCallback);
            rp3d_test(!mCallback.isHit);
            mCallback.reset();
            mWorld->raycast(Ray(ray12.point1, ray12.point2, decimal(0.01)), &mCallback);
            rp3d_test(!mCallback.isHit);
            mCallback.reset();
            mWorld->raycast(Ray(ray13.point1, ray13.point2, decimal(0.01)), &mCallback);
            rp3d_test(!mCallback.isHit);
            mCallback.reset();
            mWorld->raycast(Ray(ray14.point1, ray14.point2, decimal(0.01)), &mCallback);
            rp3d_test(!mCallback.isHit);
            mCallback.reset();
            mWorld->raycast(Ray(ray15.point1, ray15.point2, decimal(0.01)), &mCallback);
            rp3d_test(!mCallback.isHit);
            mCallback.reset();
            mWorld->raycast(Ray(ray16.point1, ray16.point2, decimal(0.01)), &mCallback);
            rp3d_test(!mCallback.isHit);

            // ----- Test raycast hits ----- //
            rp3d_test(mBoxBody->raycast(ray11, raycastInfo3));
            rp3d_test(mBoxCollider->raycast(ray11, raycastInfo3));
            mCallback.reset();
            mWorld->raycast(ray11, &mCallback);
            rp3d_test(mCallback.isHit);
            mCallback.reset();
            mWorld->raycast(Ray(ray11.point1, ray11.point2, decimal(0.8)), &mCallback);
            rp3d_test(mCallback.isHit);

            rp3d_test(mBoxBody->raycast(ray12, raycastInfo3));
            rp3d_test(mBoxCollider->raycast(ray12, raycastInfo3));
            mCallback.reset();
            mWorld->raycast(ray12, &mCallback);
            rp3d_test(mCallback.isHit);
            mCallback.reset();
            mWorld->raycast(Ray(ray12.point1, ray12.point2, decimal(0.8)), &mCallback);
            rp3d_test(mCallback.isHit);

            rp3d_test(mBoxBody->raycast(ray13, raycastInfo3));
            rp3d_test(mBoxCollider->raycast(ray13, raycastInfo3));
            mCallback.reset();
            mWorld->raycast(ray13, &mCallback);
            rp3d_test(mCallback.isHit);
            mCallback.reset();
            mWorld->raycast(Ray(ray13.point1, ray13.point2, decimal(0.8)), &mCallback);
            rp3d_test(mCallback.isHit);

            rp3d_test(mBoxBody->raycast(ray14, raycastInfo3));
            rp3d_test(mBoxCollider->raycast(ray14, raycastInfo3));
            mCallback.reset();
            mWorld->raycast(ray14, &mCallback);
            rp3d_test(mCallback.isHit);
            mCallback.reset();
            mWorld->raycast(Ray(ray14.point1, ray14.point2, decimal(0.8)), &mCallback);
            rp3d_test(mCallback.isHit);

            rp3d_test(mBoxBody->raycast(ray15, raycastInfo3));
            rp3d_test(mBoxCollider->raycast(ray15, raycastInfo3));
            mCallback.reset();
            mWorld->raycast(ray15, &mCallback);
            rp3d_test(mCallback.isHit);
            mCallback.reset();
            mWorld->raycast(Ray(ray15.point1, ray15.point2, decimal(0.8)), &mCallback);
            rp3d_test(mCallback.isHit);

            rp3d_test(mBoxBody->raycast(ray16, raycastInfo3));
            rp3d_test(mBoxCollider->raycast(ray16, raycastInfo3));
            mCallback.reset();
            mWorld->raycast(ray16, &mCallback);
            rp3d_test(mCallback.isHit);
            mCallback.reset();
            mWorld->raycast(Ray(ray16.point1, ray16.point2, decimal(0.8)), &mCallback);
            rp3d_test(mCallback.isHit);
        }

        /// Test the Collider::raycast(), RigidBody::raycast() and
        /// PhysicsWorld::raycast() methods.
        void testSphere() {

            // ----- Test feedback data ----- //
            Vector3 point1 = mLocalShapeToWorld * Vector3(-5 , 0, 0);
            Vector3 point2 = mLocalShapeToWorld * Vector3(5, 0, 0);
            Ray ray(point1, point2);
            Vector3 hitPoint = mLocalShapeToWorld * Vector3(-3, 0, 0);

            mCallback.shapeToTest = mSphereCollider;

            // PhysicsWorld::raycast()
            mCallback.reset();
            mWorld->raycast(ray, &mCallback);
            rp3d_test(mCallback.isHit);
            rp3d_test(mCallback.raycastInfo.body == mSphereBody);
            rp3d_test(mCallback.raycastInfo.collider == mSphereCollider);
            rp3d_test(approxEqual(mCallback.raycastInfo.hitFraction, 0.2, epsilon));
            rp3d_test(approxEqual(mCallback.raycastInfo.worldPoint.x, hitPoint.x, epsilon));
            rp3d_test(approxEqual(mCallback.raycastInfo.worldPoint.y, hitPoint.y, epsilon));
            rp3d_test(approxEqual(mCallback.raycastInfo.worldPoint.z, hitPoint.z, epsilon));

            // Correct category filter mask
            mCallback.reset();
            mWorld->raycast(ray, &mCallback, CATEGORY1);
            rp3d_test(mCallback.isHit);

            // Wrong category filter mask
            mCallback.reset();
            mWorld->raycast(ray, &mCallback, CATEGORY2);
            rp3d_test(!mCallback.isHit);

            // RigidBody::raycast()
            RaycastInfo raycastInfo2;
            rp3d_test(mSphereBody->raycast(ray, raycastInfo2));
            rp3d_test(raycastInfo2.body == mSphereBody);
            rp3d_test(raycastInfo2.collider == mSphereCollider);
            rp3d_test(approxEqual(raycastInfo2.hitFraction, 0.2, epsilon));
            rp3d_test(approxEqual(raycastInfo2.worldPoint.x, hitPoint.x, epsilon));
            rp3d_test(approxEqual(raycastInfo2.worldPoint.y, hitPoint.y, epsilon));
            rp3d_test(approxEqual(raycastInfo2.worldPoint.z, hitPoint.z, epsilon));

            // Collider::raycast()
            RaycastInfo raycastInfo3;
            rp3d_test(mSphereCollider->raycast(ray, raycastInfo3));
            rp3d_test(raycastInfo3.body == mSphereBody);
            rp3d_test(raycastInfo3.collider == mSphereCollider);
            rp3d_test(approxEqual(raycastInfo3.hitFraction, 0.2, epsilon));
            rp3d_test(approxEqual(raycastInfo3.worldPoint.x, hitPoint.x, epsilon));
            rp3d_test(approxEqual(raycastInfo3.worldPoint.y, hitPoint.y, epsilon));
            rp3d_test(approxEqual(raycastInfo3.worldPoint.z, hitPoint.z, epsilon));

            Ray ray1(mLocalShapeToWorld * Vector3(0, 0, 0), mLocalShapeToWorld * Vector3(5, 7, -1));
            Ray ray2(mLocalShapeToWorld * Vector3(5, 11, 7), mLocalShapeToWorld * Vector3(4, 6, 7));
            Ray ray3(mLocalShapeToWorld * Vector3(1, 2, 2), mLocalShapeToWorld * Vector3(-4, 0, 7));
            Ray ray4(mLocalShapeToWorld * Vector3(10, 10, 10), mLocalShapeToWorld * Vector3(4, 6, 7));
            Ray ray5(mLocalShapeToWorld * Vector3(4, 1, -5), mLocalShapeToWorld * Vector3(-30, 1, -5));
            Ray ray6(mLocalShapeToWorld * Vector3(4, 4, 1), mLocalShapeToWorld * Vector3(4, -30, 1));
            Ray ray7(mLocalShapeToWorld * Vector3(1, -4, 5), mLocalShapeToWorld * Vector3(1, -4, -30));
            Ray ray8(mLocalShapeToWorld * Vector3(-4, 4, 0), mLocalShapeToWorld * Vector3(30, 4, 0));
            Ray ray9(mLocalShapeToWorld * Vector3(0, -4, -4), mLocalShapeToWorld * Vector3(0, 30, -4));
            Ray ray10(mLocalShapeToWorld * Vector3(-4, 0, -6), mLocalShapeToWorld * Vector3(-4, 0, 30));
            Ray ray11(mLocalShapeToWorld * Vector3(4, 1, 2), mLocalShapeToWorld * Vector3(-30, 1, 2));
            Ray ray12(mLocalShapeToWorld * Vector3(1, 4, -1), mLocalShapeToWorld * Vector3(1, -30, -1));
            Ray ray13(mLocalShapeToWorld * Vector3(-1, 2, 5), mLocalShapeToWorld * Vector3(-1, 2, -30));
            Ray ray14(mLocalShapeToWorld * Vector3(-5, 2, -2), mLocalShapeToWorld * Vector3(30, 2, -2));
            Ray ray15(mLocalShapeToWorld * Vector3(0, -4, 1), mLocalShapeToWorld * Vector3(0, 30, 1));
            Ray ray16(mLocalShapeToWorld * Vector3(-1, 2, -11), mLocalShapeToWorld * Vector3(-1, 2, 30));

            // ----- Test raycast miss ----- //
            rp3d_test(!mSphereBody->raycast(ray1, raycastInfo3));
            rp3d_test(!mSphereCollider->raycast(ray1, raycastInfo3));
            mCallback.reset();
            mWorld->raycast(ray1, &mCallback);
            rp3d_test(!mCallback.isHit);
            mCallback.reset();
            mWorld->raycast(Ray(ray1.point1, ray1.point2, decimal(0.01)), &mCallback);
            rp3d_test(!mCallback.isHit);
            mCallback.reset();
            mWorld->raycast(Ray(ray1.point1, ray1.point2, decimal(100.0)), &mCallback);
            rp3d_test(!mCallback.isHit);

            rp3d_test(!mSphereBody->raycast(ray2, raycastInfo3));
            rp3d_test(!mSphereCollider->raycast(ray2, raycastInfo3));
            mCallback.reset();
            mWorld->raycast(ray2, &mCallback);
            rp3d_test(!mCallback.isHit);

            rp3d_test(!mSphereBody->raycast(ray3, raycastInfo3));
            rp3d_test(!mSphereCollider->raycast(ray3, raycastInfo3));
            mCallback.reset();
            mWorld->raycast(ray3, &mCallback);
            rp3d_test(!mCallback.isHit);

            rp3d_test(!mSphereBody->raycast(ray4, raycastInfo3));
            rp3d_test(!mSphereCollider->raycast(ray4, raycastInfo3));
            mCallback.reset();
            mWorld->raycast(ray4, &mCallback);
            rp3d_test(!mCallback.isHit);

            rp3d_test(!mSphereBody->raycast(ray5, raycastInfo3));
            rp3d_test(!mSphereCollider->raycast(ray5, raycastInfo3));
            mCallback.reset();
            mWorld->raycast(ray5, &mCallback);
            rp3d_test(!mCallback.isHit);

            rp3d_test(!mSphereBody->raycast(ray6, raycastInfo3));
            rp3d_test(!mSphereCollider->raycast(ray6, raycastInfo3));
            mCallback.reset();
            mWorld->raycast(ray6, &mCallback);
            rp3d_test(!mCallback.isHit);

            rp3d_test(!mSphereBody->raycast(ray7, raycastInfo3));
            rp3d_test(!mSphereCollider->raycast(ray7, raycastInfo3));
            mCallback.reset();
            mWorld->raycast(ray7, &mCallback);
            rp3d_test(!mCallback.isHit);

            rp3d_test(!mSphereBody->raycast(ray8, raycastInfo3));
            rp3d_test(!mSphereCollider->raycast(ray8, raycastInfo3));
            mCallback.reset();
            mWorld->raycast(ray8, &mCallback);
            rp3d_test(!mCallback.isHit);

            rp3d_test(!mSphereBody->raycast(ray9, raycastInfo3));
            rp3d_test(!mSphereCollider->raycast(ray9, raycastInfo3));
            mCallback.reset();
            mWorld->raycast(ray9, &mCallback);
            rp3d_test(!mCallback.isHit);

            rp3d_test(!mSphereBody->raycast(ray10, raycastInfo3));
            rp3d_test(!mSphereCollider->raycast(ray10, raycastInfo3));
            mCallback.reset();
            mWorld->raycast(ray10, &mCallback);
            rp3d_test(!mCallback.isHit);

            mCallback.reset();
            mWorld->raycast(Ray(ray11.point1, ray11.point2, decimal(0.01)), &mCallback);
            rp3d_test(!mCallback.isHit);
            mCallback.reset();
            mWorld->raycast(Ray(ray12.point1, ray12.point2, decimal(0.01)), &mCallback);
            rp3d_test(!mCallback.isHit);
            mCallback.reset();
            mWorld->raycast(Ray(ray13.point1, ray13.point2, decimal(0.01)), &mCallback);
            rp3d_test(!mCallback.isHit);
            mCallback.reset();
            mWorld->raycast(Ray(ray14.point1, ray14.point2, decimal(0.01)), &mCallback);
            rp3d_test(!mCallback.isHit);
            mCallback.reset();
            mWorld->raycast(Ray(ray15.point1, ray15.point2, decimal(0.01)), &mCallback);
            rp3d_test(!mCallback.isHit);
            mCallback.reset();
            mWorld->raycast(Ray(ray16.point1, ray16.point2, decimal(0.01)), &mCallback);
            rp3d_test(!mCallback.isHit);

            // ----- Test raycast hits ----- //
            rp3d_test(mSphereBody->raycast(ray11, raycastInfo3));
            rp3d_test(mSphereCollider->raycast(ray11, raycastInfo3));
            mCallback.reset();
            mWorld->raycast(ray11, &mCallback);
            rp3d_test(mCallback.isHit);
            mCallback.reset();
            mWorld->raycast(Ray(ray11.point1, ray11.point2, decimal(0.8)), &mCallback);
            rp3d_test(mCallback.isHit);

            rp3d_test(mSphereBody->raycast(ray12, raycastInfo3));
            rp3d_test(mSphereCollider->raycast(ray12, raycastInfo3));
            mCallback.reset();
            mWorld->raycast(ray12, &mCallback);
            rp3d_test(mCallback.isHit);
            mCallback.reset();
            mWorld->raycast(Ray(ray12.point1, ray12.point2, decimal(0.8)), &mCallback);
            rp3d_test(mCallback.isHit);

            rp3d_test(mSphereBody->raycast(ray13, raycastInfo3));
            rp3d_test(mSphereCollider->raycast(ray13, raycastInfo3));
            mCallback.reset();
            mWorld->raycast(ray13, &mCallback);
            mCallback.reset();
            mWorld->raycast(Ray(ray13.point1, ray13.point2, decimal(0.8)), &mCallback);

            rp3d_test(mSphereBody->raycast(ray14, raycastInfo3));
            rp3d_test(mSphereCollider->raycast(ray14, raycastInfo3));
            mCallback.reset();
            mWorld->raycast(ray14, &mCallback);
            rp3d_test(mCallback.isHit);
            mCallback.reset();
            mWorld->raycast(Ray(ray14.point1, ray14.point2, decimal(0.8)), &mCallback);
            rp3d_test(mCallback.isHit);

            rp3d_test(mSphereBody->raycast(ray15, raycastInfo3));
            rp3d_test(mSphereCollider->raycast(ray15, raycastInfo3));
            mCallback.reset();
            mWorld->raycast(ray15, &mCallback);
            rp3d_test(mCallback.isHit);
            mCallback.reset();
            mWorld->raycast(Ray(ray15.point1, ray15.point2, decimal(0.8)), &mCallback);
            rp3d_test(mCallback.isHit);

            rp3d_test(mSphereBody->raycast(ray16, raycastInfo3));
            rp3d_test(mSphereCollider->raycast(ray16, raycastInfo3));
            mCallback.reset();
            mWorld->raycast(ray16, &mCallback);
            rp3d_test(mCallback.isHit);
            mCallback.reset();
            mWorld->raycast(Ray(ray16.point1, ray16.point2, decimal(0.8)), &mCallback);
            rp3d_test(mCallback.isHit);
        }

        /// Test the Collider::raycast(), RigidBody::raycast() and
        /// PhysicsWorld::raycast() methods.
        void testCapsule() {

            // ----- Test feedback data ----- //
            Vector3 point1A = mLocalShapeToWorld * Vector3(4 , 1, 0);
            Vector3 point1B = mLocalShapeToWorld * Vector3(-6, 1, 0);
            Ray ray(point1A, point1B);
            Vector3 hitPoint = mLocalShapeToWorld * Vector3(2, 1, 0);

            Vector3 point2A = mLocalShapeToWorld * Vector3(0 , 6.5, 0);
            Vector3 point2B = mLocalShapeToWorld * Vector3(0, -3.5, 0);
            Ray rayTop(point2A, point2B);
            Vector3 hitPointTop = mLocalShapeToWorld * Vector3(0, decimal(4.5), 0);

            Vector3 point3A = mLocalShapeToWorld * Vector3(0 , -6.5, 0);
            Vector3 point3B = mLocalShapeToWorld * Vector3(0, 3.5, 0);
            Ray rayBottom(point3A, point3B);
            Vector3 hitPointBottom = mLocalShapeToWorld * Vector3(0, decimal(-4.5), 0);

            mCallback.shapeToTest = mCapsuleCollider;

            // PhysicsWorld::raycast()
            mCallback.reset();
            mWorld->raycast(ray, &mCallback);
            rp3d_test(mCallback.isHit);
            rp3d_test(mCallback.raycastInfo.body == mCapsuleBody);
            rp3d_test(mCallback.raycastInfo.collider == mCapsuleCollider);
            rp3d_test(approxEqual(mCallback.raycastInfo.hitFraction, decimal(0.2), epsilon));
            rp3d_test(approxEqual(mCallback.raycastInfo.worldPoint.x, hitPoint.x, epsilon));
            rp3d_test(approxEqual(mCallback.raycastInfo.worldPoint.y, hitPoint.y, epsilon));
            rp3d_test(approxEqual(mCallback.raycastInfo.worldPoint.z, hitPoint.z, epsilon));

            // Correct category filter mask
            mCallback.reset();
            mWorld->raycast(ray, &mCallback, CATEGORY1);
            rp3d_test(mCallback.isHit);

            // Wrong category filter mask
            mCallback.reset();
            mWorld->raycast(ray, &mCallback, CATEGORY2);
            rp3d_test(!mCallback.isHit);

            // RigidBody::raycast()
            RaycastInfo raycastInfo2;
            rp3d_test(mCapsuleBody->raycast(ray, raycastInfo2));
            rp3d_test(raycastInfo2.body == mCapsuleBody);
            rp3d_test(raycastInfo2.collider == mCapsuleCollider);
            rp3d_test(approxEqual(raycastInfo2.hitFraction, decimal(0.2), epsilon));
            rp3d_test(approxEqual(raycastInfo2.worldPoint.x, hitPoint.x, epsilon));
            rp3d_test(approxEqual(raycastInfo2.worldPoint.y, hitPoint.y, epsilon));
            rp3d_test(approxEqual(raycastInfo2.worldPoint.z, hitPoint.z, epsilon));

            // Collider::raycast()
            RaycastInfo raycastInfo3;
            rp3d_test(mCapsuleCollider->raycast(ray, raycastInfo3));
            rp3d_test(raycastInfo3.body == mCapsuleBody);
            rp3d_test(raycastInfo3.collider == mCapsuleCollider);
            rp3d_test(approxEqual(raycastInfo3.hitFraction, decimal(0.2), epsilon));
            rp3d_test(approxEqual(raycastInfo3.worldPoint.x, hitPoint.x, epsilon));
            rp3d_test(approxEqual(raycastInfo3.worldPoint.y, hitPoint.y, epsilon));
            rp3d_test(approxEqual(raycastInfo3.worldPoint.z, hitPoint.z, epsilon));

            RaycastInfo raycastInfo4;
            rp3d_test(mCapsuleCollider->raycast(rayTop, raycastInfo4));
            rp3d_test(raycastInfo4.body == mCapsuleBody);
            rp3d_test(raycastInfo4.collider == mCapsuleCollider);
            rp3d_test(approxEqual(raycastInfo4.hitFraction, decimal(0.2), epsilon));
            rp3d_test(approxEqual(raycastInfo4.worldPoint.x, hitPointTop.x, epsilon));
            rp3d_test(approxEqual(raycastInfo4.worldPoint.y, hitPointTop.y, epsilon));
            rp3d_test(approxEqual(raycastInfo4.worldPoint.z, hitPointTop.z, epsilon));

            // Collider::raycast()
            RaycastInfo raycastInfo5;
            rp3d_test(mCapsuleCollider->raycast(rayBottom, raycastInfo5));
            rp3d_test(raycastInfo5.body == mCapsuleBody);
            rp3d_test(raycastInfo5.collider == mCapsuleCollider);
            rp3d_test(approxEqual(raycastInfo5.hitFraction, decimal(0.2), epsilon));
            rp3d_test(approxEqual(raycastInfo5.worldPoint.x, hitPointBottom.x, epsilon));
            rp3d_test(approxEqual(raycastInfo5.worldPoint.y, hitPointBottom.y, epsilon));
            rp3d_test(approxEqual(raycastInfo5.worldPoint.z, hitPointBottom.z, epsilon));

            Ray ray1(mLocalShapeToWorld * Vector3(0, 0, 0), mLocalShapeToWorld * Vector3(5, 7, -1));
            Ray ray2(mLocalShapeToWorld * Vector3(5, 11, 7), mLocalShapeToWorld * Vector3(9, 17, 14));
            Ray ray3(mLocalShapeToWorld * Vector3(1, 3, -1), mLocalShapeToWorld * Vector3(-3, 3, 6));
            Ray ray4(mLocalShapeToWorld * Vector3(10, 10, 10), mLocalShapeToWorld * Vector3(14, 16, 17));
            Ray ray5(mLocalShapeToWorld * Vector3(4, 1, -5), mLocalShapeToWorld * Vector3(1, 1, -5));
            Ray ray6(mLocalShapeToWorld * Vector3(4, 9, 1), mLocalShapeToWorld * Vector3(4, 7, 1));
            Ray ray7(mLocalShapeToWorld * Vector3(1, -9, 5), mLocalShapeToWorld * Vector3(1, -9, 3));
            Ray ray8(mLocalShapeToWorld * Vector3(-4, 9, 0), mLocalShapeToWorld * Vector3(-3, 9, 0));
            Ray ray9(mLocalShapeToWorld * Vector3(0, -9, -4), mLocalShapeToWorld * Vector3(0, -4, -4));
            Ray ray10(mLocalShapeToWorld * Vector3(-4, 0, -6), mLocalShapeToWorld * Vector3(-4, 0, 2));
            Ray ray11(mLocalShapeToWorld * Vector3(4, 1, 1.5), mLocalShapeToWorld * Vector3(-30, 1, 1.5));
            Ray ray12(mLocalShapeToWorld * Vector3(1, 9, -1), mLocalShapeToWorld * Vector3(1, -30, -1));
            Ray ray13(mLocalShapeToWorld * Vector3(-1, 2, 3), mLocalShapeToWorld * Vector3(-1, 2, -30));
            Ray ray14(mLocalShapeToWorld * Vector3(-3, 2, -1.7), mLocalShapeToWorld * Vector3(30, 2, -1.7));
            Ray ray15(mLocalShapeToWorld * Vector3(0, -9, 1), mLocalShapeToWorld * Vector3(0, 30, 1));
            Ray ray16(mLocalShapeToWorld * Vector3(-1, 2, -7), mLocalShapeToWorld * Vector3(-1, 2, 30));

            // ----- Test raycast miss ----- //
            rp3d_test(!mCapsuleBody->raycast(ray1, raycastInfo3));
            rp3d_test(!mCapsuleCollider->raycast(ray1, raycastInfo3));
            mCallback.reset();
            mWorld->raycast(ray1, &mCallback);
            rp3d_test(!mCallback.isHit);
            mCallback.reset();
            mWorld->raycast(Ray(ray1.point1, ray1.point2, decimal(0.01)), &mCallback);
            rp3d_test(!mCallback.isHit);
            mCallback.reset();
            mWorld->raycast(Ray(ray1.point1, ray1.point2, decimal(100.0)), &mCallback);
            rp3d_test(!mCallback.isHit);

            rp3d_test(!mCapsuleBody->raycast(ray2, raycastInfo3));
            rp3d_test(!mCapsuleCollider->raycast(ray2, raycastInfo3));
            mCallback.reset();
            mWorld->raycast(ray2, &mCallback);
            rp3d_test(!mCallback.isHit);

            rp3d_test(!mCapsuleBody->raycast(ray3, raycastInfo3));
            rp3d_test(!mCapsuleCollider->raycast(ray3, raycastInfo3));
            mCallback.reset();
            mWorld->raycast(ray3, &mCallback);
            rp3d_test(!mCallback.isHit);

            rp3d_test(!mCapsuleBody->raycast(ray4, raycastInfo3));
            rp3d_test(!mCapsuleCollider->raycast(ray4, raycastInfo3));
            mCallback.reset();
            mWorld->raycast(ray4, &mCallback);
            rp3d_test(!mCallback.isHit);

            rp3d_test(!mCapsuleBody->raycast(ray5, raycastInfo3));
            rp3d_test(!mCapsuleCollider->raycast(ray5, raycastInfo3));
            mCallback.reset();
            mWorld->raycast(ray5, &mCallback);
            rp3d_test(!mCallback.isHit);

            rp3d_test(!mCapsuleBody->raycast(ray6, raycastInfo3));
            rp3d_test(!mCapsuleCollider->raycast(ray6, raycastInfo3));
            mCallback.reset();
            mWorld->raycast(ray6, &mCallback);
            rp3d_test(!mCallback.isHit);

            rp3d_test(!mCapsuleBody->raycast(ray7, raycastInfo3));
            rp3d_test(!mCapsuleCollider->raycast(ray7, raycastInfo3));
            mWorld->raycast(ray7, &mCallback);
            rp3d_test(!mCallback.isHit);

            rp3d_test(!mCapsuleBody->raycast(ray8, raycastInfo3));
            rp3d_test(!mCapsuleCollider->raycast(ray8, raycastInfo3));
            mCallback.reset();
            mWorld->raycast(ray8, &mCallback);
            rp3d_test(!mCallback.isHit);

            rp3d_test(!mCapsuleBody->raycast(ray9, raycastInfo3));
            rp3d_test(!mCapsuleCollider->raycast(ray9, raycastInfo3));
            mCallback.reset();
            mWorld->raycast(ray9, &mCallback);
            rp3d_test(!mCallback.isHit);

            rp3d_test(!mCapsuleBody->raycast(ray10, raycastInfo3));
            rp3d_test(!mCapsuleCollider->raycast(ray10, raycastInfo3));
            mCallback.reset();
            mWorld->raycast(ray10, &mCallback);
            rp3d_test(!mCallback.isHit);

            mCallback.reset();
            mWorld->raycast(Ray(ray11.point1, ray11.point2, decimal(0.01)), &mCallback);
            rp3d_test(!mCallback.isHit);
            mCallback.reset();
            mWorld->raycast(Ray(ray12.point1, ray12.point2, decimal(0.01)), &mCallback);
            rp3d_test(!mCallback.isHit);
            mCallback.reset();
            mWorld->raycast(Ray(ray13.point1, ray13.point2, decimal(0.01)), &mCallback);
            rp3d_test(!mCallback.isHit);
            mCallback.reset();
            mWorld->raycast(Ray(ray14.point1, ray14.point2, decimal(0.01)), &mCallback);
            rp3d_test(!mCallback.isHit);
            mCallback.reset();
            mWorld->raycast(Ray(ray15.point1, ray15.point2, decimal(0.01)), &mCallback);
            rp3d_test(!mCallback.isHit);
            mCallback.reset();
            mWorld->raycast(Ray(ray16.point1, ray16.point2, decimal(0.01)), &mCallback);
            rp3d_test(!mCallback.isHit);

            // ----- Test raycast hits ----- //
            rp3d_test(mCapsuleBody->raycast(ray11, raycastInfo3));
            rp3d_test(mCapsuleCollider->raycast(ray11, raycastInfo3));
            mCallback.reset();
            mWorld->raycast(ray11, &mCallback);
            rp3d_test(mCallback.isHit);
            mCallback.reset();
            mWorld->raycast(Ray(ray11.point1, ray11.point2, decimal(0.8)), &mCallback);
            rp3d_test(mCallback.isHit);

            rp3d_test(mCapsuleBody->raycast(ray12, raycastInfo3));
            rp3d_test(mCapsuleCollider->raycast(ray12, raycastInfo3));
            mCallback.reset();
            mWorld->raycast(ray12, &mCallback);
            rp3d_test(mCallback.isHit);
            mCallback.reset();
            mWorld->raycast(Ray(ray12.point1, ray12.point2, decimal(0.8)), &mCallback);
            rp3d_test(mCallback.isHit);

            rp3d_test(mCapsuleBody->raycast(ray13, raycastInfo3));
            rp3d_test(mCapsuleCollider->raycast(ray13, raycastInfo3));
            mCallback.reset();
            mWorld->raycast(ray13, &mCallback);
            rp3d_test(mCallback.isHit);
            mCallback.reset();
            mWorld->raycast(Ray(ray13.point1, ray13.point2, decimal(0.8)), &mCallback);
            rp3d_test(mCallback.isHit);

            rp3d_test(mCapsuleBody->raycast(ray14, raycastInfo3));
            rp3d_test(mCapsuleCollider->raycast(ray14, raycastInfo3));
            mCallback.reset();
            mWorld->raycast(ray14, &mCallback);
            rp3d_test(mCallback.isHit);
            mCallback.reset();
            mWorld->raycast(Ray(ray14.point1, ray14.point2, decimal(0.8)), &mCallback);
            rp3d_test(mCallback.isHit);

            rp3d_test(mCapsuleBody->raycast(ray15, raycastInfo3));
            rp3d_test(mCapsuleCollider->raycast(ray15, raycastInfo3));
            mCallback.reset();
            mWorld->raycast(ray15, &mCallback);
            rp3d_test(mCallback.isHit);
            mCallback.reset();
            mWorld->raycast(Ray(ray15.point1, ray15.point2, decimal(0.8)), &mCallback);
            rp3d_test(mCallback.isHit);

            rp3d_test(mCapsuleBody->raycast(ray16, raycastInfo3));
            rp3d_test(mCapsuleCollider->raycast(ray16, raycastInfo3));
            mCallback.reset();
            mWorld->raycast(ray16, &mCallback);
            rp3d_test(mCallback.isHit);
            mCallback.reset();
            mWorld->raycast(Ray(ray16.point1, ray16.point2, decimal(0.8)), &mCallback);
            rp3d_test(mCallback.isHit);
        }

        /// Test the Collider::raycast(), RigidBody::raycast() and
        /// PhysicsWorld::raycast() methods.
        void testConvexMesh() {

            // ----- Test feedback data ----- //
            Vector3 point1 = mLocalShapeToWorld * Vector3(1 , 2, 6);
            Vector3 point2 = mLocalShapeToWorld * Vector3(1, 2, -4);
            Ray ray(point1, point2);
            Vector3 hitPoint = mLocalShapeToWorld * Vector3(1, 2, 4);

            mCallback.shapeToTest = mConvexMeshCollider;

            // PhysicsWorld::raycast()
            mCallback.reset();
            mWorld->raycast(ray, &mCallback);
            rp3d_test(mCallback.isHit);
            rp3d_test(mCallback.raycastInfo.body == mConvexMeshBody);
            rp3d_test(mCallback.raycastInfo.collider == mConvexMeshCollider);
            rp3d_test(approxEqual(mCallback.raycastInfo.hitFraction, decimal(0.2), epsilon));
            rp3d_test(approxEqual(mCallback.raycastInfo.worldPoint.x, hitPoint.x, epsilon));
            rp3d_test(approxEqual(mCallback.raycastInfo.worldPoint.y, hitPoint.y, epsilon));
            rp3d_test(approxEqual(mCallback.raycastInfo.worldPoint.z, hitPoint.z, epsilon));

            // Correct category filter mask
            mCallback.reset();
            mWorld->raycast(ray, &mCallback, CATEGORY2);
            rp3d_test(mCallback.isHit);

            // Wrong category filter mask
            mCallback.reset();
            mWorld->raycast(ray, &mCallback, CATEGORY1);
            rp3d_test(!mCallback.isHit);

            // RigidBody::raycast()
            RaycastInfo raycastInfo2;
            rp3d_test(mConvexMeshBody->raycast(ray, raycastInfo2));
            rp3d_test(raycastInfo2.body == mConvexMeshBody);
            rp3d_test(raycastInfo2.collider == mConvexMeshCollider);
            rp3d_test(approxEqual(raycastInfo2.hitFraction, decimal(0.2), epsilon));
            rp3d_test(approxEqual(raycastInfo2.worldPoint.x, hitPoint.x, epsilon));
            rp3d_test(approxEqual(raycastInfo2.worldPoint.y, hitPoint.y, epsilon));
            rp3d_test(approxEqual(raycastInfo2.worldPoint.z, hitPoint.z, epsilon));

            // Collider::raycast()
            RaycastInfo raycastInfo4;
            rp3d_test(mConvexMeshCollider->raycast(ray, raycastInfo4));
            rp3d_test(raycastInfo4.body == mConvexMeshBody);
            rp3d_test(raycastInfo4.collider == mConvexMeshCollider);
            rp3d_test(approxEqual(raycastInfo4.hitFraction, decimal(0.2), epsilon));
            rp3d_test(approxEqual(raycastInfo4.worldPoint.x, hitPoint.x, epsilon));
            rp3d_test(approxEqual(raycastInfo4.worldPoint.y, hitPoint.y, epsilon));
            rp3d_test(approxEqual(raycastInfo4.worldPoint.z, hitPoint.z, epsilon));

            Ray ray1(mLocalShapeToWorld * Vector3(0, 0, 0), mLocalShapeToWorld * Vector3(5, 7, -1));
            Ray ray2(mLocalShapeToWorld * Vector3(5, 11, 7), mLocalShapeToWorld * Vector3(17, 29, 28));
            Ray ray3(mLocalShapeToWorld * Vector3(1, 2, 3), mLocalShapeToWorld * Vector3(-11, 2, 24));
            Ray ray4(mLocalShapeToWorld * Vector3(10, 10, 10), mLocalShapeToWorld * Vector3(22, 28, 31));
            Ray ray5(mLocalShapeToWorld * Vector3(3, 1, -5), mLocalShapeToWorld * Vector3(-30, 1, -5));
            Ray ray6(mLocalShapeToWorld * Vector3(4, 4, 1), mLocalShapeToWorld * Vector3(4, -30, 1));
            Ray ray7(mLocalShapeToWorld * Vector3(1, -4, 5), mLocalShapeToWorld * Vector3(1, -4, -30));
            Ray ray8(mLocalShapeToWorld * Vector3(-4, 4, 0), mLocalShapeToWorld * Vector3(30, 4, 0));
            Ray ray9(mLocalShapeToWorld * Vector3(0, -4, -7), mLocalShapeToWorld * Vector3(0, 30, -7));
            Ray ray10(mLocalShapeToWorld * Vector3(-3, 0, -6), mLocalShapeToWorld * Vector3(-3, 0, 30));
            Ray ray11(mLocalShapeToWorld * Vector3(3, 1, 2), mLocalShapeToWorld * Vector3(-30, 0, -6));
            Ray ray12(mLocalShapeToWorld * Vector3(1, 4, -1), mLocalShapeToWorld * Vector3(1, -30, -1));
            Ray ray13(mLocalShapeToWorld * Vector3(-1, 2, 5), mLocalShapeToWorld * Vector3(-1, 2, -30));
            Ray ray14(mLocalShapeToWorld * Vector3(-3, 2, -2), mLocalShapeToWorld * Vector3(30, 2, -2));
            Ray ray15(mLocalShapeToWorld * Vector3(0, -4, 1), mLocalShapeToWorld * Vector3(0, 30, 1));
            Ray ray16(mLocalShapeToWorld * Vector3(-1, 2, -7), mLocalShapeToWorld * Vector3(-1, 2, 30));

            // ----- Test raycast miss ----- //
            RaycastInfo raycastInfo3;
            rp3d_test(!mConvexMeshBody->raycast(ray1, raycastInfo3));
            rp3d_test(!mConvexMeshCollider->raycast(ray1, raycastInfo3));
            mCallback.reset();
            mWorld->raycast(ray1, &mCallback);
            rp3d_test(!mCallback.isHit);
            mCallback.reset();
            mWorld->raycast(Ray(ray1.point1, ray1.point2, decimal(0.01)), &mCallback);
            rp3d_test(!mCallback.isHit);
            mCallback.reset();
            mWorld->raycast(Ray(ray1.point1, ray1.point2, decimal(100.0)), &mCallback);
            rp3d_test(!mCallback.isHit);

            rp3d_test(!mConvexMeshBody->raycast(ray2, raycastInfo3));
            rp3d_test(!mConvexMeshCollider->raycast(ray2, raycastInfo3));
            mCallback.reset();
            mWorld->raycast(ray2, &mCallback);
            rp3d_test(!mCallback.isHit);

            rp3d_test(!mConvexMeshBody->raycast(ray3, raycastInfo3));
            rp3d_test(!mConvexMeshCollider->raycast(ray3, raycastInfo3));
            mCallback.reset();
            mWorld->raycast(ray3, &mCallback);
            rp3d_test(!mCallback.isHit);

            rp3d_test(!mConvexMeshBody->raycast(ray4, raycastInfo3));
            rp3d_test(!mConvexMeshCollider->raycast(ray4, raycastInfo3));
            mCallback.reset();
            mWorld->raycast(ray4, &mCallback);
            rp3d_test(!mCallback.isHit);

            rp3d_test(!mConvexMeshBody->raycast(ray5, raycastInfo3));
            rp3d_test(!mConvexMeshCollider->raycast(ray5, raycastInfo3));
            mCallback.reset();
            mWorld->raycast(ray5, &mCallback);
            rp3d_test(!mCallback.isHit);

            rp3d_test(!mConvexMeshBody->raycast(ray6, raycastInfo3));
            rp3d_test(!mConvexMeshCollider->raycast(ray6, raycastInfo3));
            mCallback.reset();
            mWorld->raycast(ray6, &mCallback);
            rp3d_test(!mCallback.isHit);

            rp3d_test(!mConvexMeshBody->raycast(ray7, raycastInfo3));
            rp3d_test(!mConvexMeshCollider->raycast(ray7, raycastInfo3));
            mCallback.reset();
            mWorld->raycast(ray7, &mCallback);
            rp3d_test(!mCallback.isHit);

            rp3d_test(!mConvexMeshBody->raycast(ray8, raycastInfo3));
            rp3d_test(!mConvexMeshCollider->raycast(ray8, raycastInfo3));
            mCallback.reset();
            mWorld->raycast(ray8, &mCallback);
            rp3d_test(!mCallback.isHit);

            rp3d_test(!mConvexMeshBody->raycast(ray9, raycastInfo3));
            rp3d_test(!mConvexMeshCollider->raycast(ray9, raycastInfo3));
            mCallback.reset();
            mWorld->raycast(ray9, &mCallback);
            rp3d_test(!mCallback.isHit);

            rp3d_test(!mConvexMeshBody->raycast(ray10, raycastInfo3));
            rp3d_test(!mConvexMeshCollider->raycast(ray10, raycastInfo3));
            mCallback.reset();
            mWorld->raycast(ray10, &mCallback);
            rp3d_test(!mCallback.isHit);

            mCallback.reset();
            mWorld->raycast(Ray(ray11.point1, ray11.point2, decimal(0.01)), &mCallback);
            rp3d_test(!mCallback.isHit);
            mCallback.reset();
            mWorld->raycast(Ray(ray12.point1, ray12.point2, decimal(0.01)), &mCallback);
            rp3d_test(!mCallback.isHit);
            mCallback.reset();
            mWorld->raycast(Ray(ray13.point1, ray13.point2, decimal(0.01)), &mCallback);
            rp3d_test(!mCallback.isHit);
            mCallback.reset();
            mWorld->raycast(Ray(ray14.point1, ray14.point2, decimal(0.01)), &mCallback);
            rp3d_test(!mCallback.isHit);
            mCallback.reset();
            mWorld->raycast(Ray(ray15.point1, ray15.point2, decimal(0.01)), &mCallback);
            rp3d_test(!mCallback.isHit);
            mCallback.reset();
            mWorld->raycast(Ray(ray16.point1, ray16.point2, decimal(0.01)), &mCallback);
            rp3d_test(!mCallback.isHit);

            // ----- Test raycast hits ----- //
            rp3d_test(mConvexMeshBody->raycast(ray11, raycastInfo3));
            rp3d_test(mConvexMeshCollider->raycast(ray11, raycastInfo3));
           mCallback.reset();
            mWorld->raycast(ray11, &mCallback);
            rp3d_test(mCallback.isHit);
            mCallback.reset();
            mWorld->raycast(Ray(ray11.point1, ray11.point2, decimal(0.8)), &mCallback);
            rp3d_test(mCallback.isHit);

            rp3d_test(mConvexMeshBody->raycast(ray12, raycastInfo3));
            rp3d_test(mConvexMeshCollider->raycast(ray12, raycastInfo3));
            mCallback.reset();
            mWorld->raycast(ray12, &mCallback);
            rp3d_test(mCallback.isHit);
            mCallback.reset();
            mWorld->raycast(Ray(ray12.point1, ray12.point2, decimal(0.8)), &mCallback);
            rp3d_test(mCallback.isHit);

            rp3d_test(mConvexMeshBody->raycast(ray13, raycastInfo3));
            rp3d_test(mConvexMeshCollider->raycast(ray13, raycastInfo3));
            mCallback.reset();
            mWorld->raycast(ray13, &mCallback);
            rp3d_test(mCallback.isHit);
            mCallback.reset();
            mWorld->raycast(Ray(ray13.point1, ray13.point2, decimal(0.8)), &mCallback);
            rp3d_test(mCallback.isHit);

            rp3d_test(mConvexMeshBody->raycast(ray14, raycastInfo3));
            rp3d_test(mConvexMeshCollider->raycast(ray14, raycastInfo3));
            mCallback.reset();
            mWorld->raycast(ray14, &mCallback);
            rp3d_test(mCallback.isHit);
            mCallback.reset();
            mWorld->raycast(Ray(ray14.point1, ray14.point2, decimal(0.8)), &mCallback);
            rp3d_test(mCallback.isHit);

            rp3d_test(mConvexMeshBody->raycast(ray15, raycastInfo3));
            rp3d_test(mConvexMeshCollider->raycast(ray15, raycastInfo3));
            mCallback.reset();
            mWorld->raycast(ray15, &mCallback);
            rp3d_test(mCallback.isHit);
            mCallback.reset();
            mWorld->raycast(Ray(ray15.point1, ray15.point2, decimal(0.8)), &mCallback);
            rp3d_test(mCallback.isHit);

            rp3d_test(mConvexMeshBody->raycast(ray16, raycastInfo3));
            rp3d_test(mConvexMeshCollider->raycast(ray16, raycastInfo3));
            mCallback.reset();
            mWorld->raycast(ray16, &mCallback);
            rp3d_test(mCallback.isHit);
            mCallback.reset();
            mWorld->raycast(Ray(ray16.point1, ray16.point2, decimal(0.8)), &mCallback);
            rp3d_test(mCallback.isHit);
        }

        /// Test the RigidBody::raycast() and
        /// PhysicsWorld::raycast() methods.
        void testCompound() {

            // ----- Test feedback data ----- //

            // Raycast hit against the sphere shape
            Ray ray1(mLocalShape2ToWorld * Vector3(4, 1, 2), mLocalShape2ToWorld * Vector3(-30, 1, 2));
            Ray ray2(mLocalShape2ToWorld * Vector3(1, 4, -1), mLocalShape2ToWorld * Vector3(1, -30, -1));
            Ray ray3(mLocalShape2ToWorld * Vector3(-1, 2, 5), mLocalShape2ToWorld * Vector3(-1, 2, -30));
            Ray ray4(mLocalShape2ToWorld * Vector3(-5, 2, -2), mLocalShape2ToWorld * Vector3(30, 2, -2));
            Ray ray5(mLocalShape2ToWorld * Vector3(0, -4, 1), mLocalShape2ToWorld * Vector3(0, 30, 1));
            Ray ray6(mLocalShape2ToWorld * Vector3(-1, 2, -11), mLocalShape2ToWorld * Vector3(-1, 2, 30));

            mCallback.shapeToTest = mCompoundSphereCollider;

            // Correct category filter mask
            mCallback.reset();
            mWorld->raycast(ray1, &mCallback, CATEGORY2);
            rp3d_test(mCallback.isHit);

            // Wrong category filter mask
            mCallback.reset();
            mWorld->raycast(ray1, &mCallback, CATEGORY1);
            rp3d_test(!mCallback.isHit);

            RaycastInfo raycastInfo;
            rp3d_test(mCompoundBody->raycast(ray1, raycastInfo));
            mCallback.reset();
            mWorld->raycast(ray1, &mCallback);
            rp3d_test(mCallback.isHit);
            mCallback.reset();
            mWorld->raycast(Ray(ray1.point1, ray1.point2, decimal(0.8)), &mCallback);
            rp3d_test(mCallback.isHit);

            rp3d_test(mCompoundBody->raycast(ray2, raycastInfo));
            mCallback.reset();
            mWorld->raycast(ray2, &mCallback);
            rp3d_test(mCallback.isHit);
            mCallback.reset();
            mWorld->raycast(Ray(ray2.point1, ray2.point2, decimal(0.8)), &mCallback);
            rp3d_test(mCallback.isHit);

            rp3d_test(mCompoundBody->raycast(ray3, raycastInfo));
            mCallback.reset();
            mWorld->raycast(ray3, &mCallback);
            rp3d_test(mCallback.isHit);
            mCallback.reset();
            mWorld->raycast(Ray(ray3.point1, ray3.point2, decimal(0.8)), &mCallback);
            rp3d_test(mCallback.isHit);

            rp3d_test(mCompoundBody->raycast(ray4, raycastInfo));
            mCallback.reset();
            mWorld->raycast(ray4, &mCallback);
            rp3d_test(mCallback.isHit);
            mCallback.reset();
            mWorld->raycast(Ray(ray4.point1, ray4.point2, decimal(0.8)), &mCallback);
            rp3d_test(mCallback.isHit);

            rp3d_test(mCompoundBody->raycast(ray5, raycastInfo));
            mCallback.reset();
            mWorld->raycast(ray5, &mCallback);
            rp3d_test(mCallback.isHit);
            mCallback.reset();
            mWorld->raycast(Ray(ray5.point1, ray5.point2, decimal(0.8)), &mCallback);
            rp3d_test(mCallback.isHit);

            rp3d_test(mCompoundBody->raycast(ray6, raycastInfo));
            mCallback.reset();
            mWorld->raycast(ray6, &mCallback);
            rp3d_test(mCallback.isHit);
            mCallback.reset();
            mWorld->raycast(Ray(ray6.point1, ray6.point2, decimal(0.8)), &mCallback);
            rp3d_test(mCallback.isHit);

            // Raycast hit agains the capsule shape
            Ray ray11(mLocalShapeToWorld * Vector3(4, 1, 1.5), mLocalShapeToWorld * Vector3(-30, 1.5, 2));
            Ray ray12(mLocalShapeToWorld * Vector3(1.5, 9, -1), mLocalShapeToWorld * Vector3(1.5, -30, -1));
            Ray ray13(mLocalShapeToWorld * Vector3(-1, 2, 3), mLocalShapeToWorld * Vector3(-1, 2, -30));
            Ray ray14(mLocalShapeToWorld * Vector3(-3, 2, -1.5), mLocalShapeToWorld * Vector3(30, 1, -1.5));
            Ray ray15(mLocalShapeToWorld * Vector3(0, -9, 1), mLocalShapeToWorld * Vector3(0, 30, 1));
            Ray ray16(mLocalShapeToWorld * Vector3(-1, 2, -7), mLocalShapeToWorld * Vector3(-1, 2, 30));

            mCallback.shapeToTest = mCompoundCapsuleCollider;

            rp3d_test(mCompoundBody->raycast(ray11, raycastInfo));
            mCallback.reset();
            mWorld->raycast(ray11, &mCallback);
            rp3d_test(mCallback.isHit);
            mCallback.reset();
            mWorld->raycast(Ray(ray11.point1, ray11.point2, decimal(0.8)), &mCallback);
            rp3d_test(mCallback.isHit);

            rp3d_test(mCompoundBody->raycast(ray12, raycastInfo));
            mCallback.reset();
            mWorld->raycast(ray12, &mCallback);
            rp3d_test(mCallback.isHit);
            mCallback.reset();
            mWorld->raycast(Ray(ray12.point1, ray12.point2, decimal(0.8)), &mCallback);
            rp3d_test(mCallback.isHit);

            rp3d_test(mCompoundBody->raycast(ray13, raycastInfo));
            mCallback.reset();
            mWorld->raycast(ray13, &mCallback);
            rp3d_test(mCallback.isHit);
            mCallback.reset();
            mWorld->raycast(Ray(ray13.point1, ray13.point2, decimal(0.8)), &mCallback);
            rp3d_test(mCallback.isHit);

            rp3d_test(mCompoundBody->raycast(ray14, raycastInfo));
            mCallback.reset();
            mWorld->raycast(ray14, &mCallback);
            rp3d_test(mCallback.isHit);
            mCallback.reset();
            mWorld->raycast(Ray(ray14.point1, ray14.point2, decimal(0.8)), &mCallback);
            rp3d_test(mCallback.isHit);

            rp3d_test(mCompoundBody->raycast(ray15, raycastInfo));
            mCallback.reset();
            mWorld->raycast(ray15, &mCallback);
            rp3d_test(mCallback.isHit);
            mCallback.reset();
            mWorld->raycast(Ray(ray15.point1, ray15.point2, decimal(0.8)), &mCallback);
            rp3d_test(mCallback.isHit);

            rp3d_test(mCompoundBody->raycast(ray16, raycastInfo));
            mCallback.reset();
            mWorld->raycast(ray16, &mCallback);
            rp3d_test(mCallback.isHit);
            mCallback.reset();
            mWorld->raycast(Ray(ray16.point1, ray16.point2, decimal(0.8)), &mCallback);
            rp3d_test(mCallback.isHit);
        }


        void testConcaveMesh() {

            // ----- Test feedback data ----- //
            Vector3 point1 = mLocalShapeToWorld * Vector3(1 , 2, 6);
            Vector3 point2 = mLocalShapeToWorld * Vector3(1, 2, -4);
            Ray ray(point1, point2);
            Vector3 hitPoint = mLocalShapeToWorld * Vector3(1, 2, 4);

            mCallback.shapeToTest = mConcaveMeshCollider;

            // PhysicsWorld::raycast()
            mCallback.reset();
            mWorld->raycast(ray, &mCallback);
            rp3d_test(mCallback.isHit);
            rp3d_test(mCallback.raycastInfo.body == mConcaveMeshBody);
            rp3d_test(mCallback.raycastInfo.collider == mConcaveMeshCollider);
            rp3d_test(approxEqual(mCallback.raycastInfo.hitFraction, decimal(0.2), epsilon));
            rp3d_test(approxEqual(mCallback.raycastInfo.worldPoint.x, hitPoint.x, epsilon));
            rp3d_test(approxEqual(mCallback.raycastInfo.worldPoint.y, hitPoint.y, epsilon));
            rp3d_test(approxEqual(mCallback.raycastInfo.worldPoint.z, hitPoint.z, epsilon));

            // Correct category filter mask
            mCallback.reset();
            mWorld->raycast(ray, &mCallback, CATEGORY2);
            rp3d_test(mCallback.isHit);

            // Wrong category filter mask
            mCallback.reset();
            mWorld->raycast(ray, &mCallback, CATEGORY1);
            rp3d_test(!mCallback.isHit);

            // RigidBody::raycast()
            RaycastInfo raycastInfo2;
            rp3d_test(mConcaveMeshBody->raycast(ray, raycastInfo2));
            rp3d_test(raycastInfo2.body == mConcaveMeshBody);
            rp3d_test(raycastInfo2.collider == mConcaveMeshCollider);
            rp3d_test(approxEqual(raycastInfo2.hitFraction, decimal(0.2), epsilon));
            rp3d_test(approxEqual(raycastInfo2.worldPoint.x, hitPoint.x, epsilon));
            rp3d_test(approxEqual(raycastInfo2.worldPoint.y, hitPoint.y, epsilon));
            rp3d_test(approxEqual(raycastInfo2.worldPoint.z, hitPoint.z, epsilon));

            // Collider::raycast()
            RaycastInfo raycastInfo3;
            rp3d_test(mConcaveMeshBody->raycast(ray, raycastInfo3));
            rp3d_test(raycastInfo3.body == mConcaveMeshBody);
            rp3d_test(raycastInfo3.collider == mConcaveMeshCollider);
            rp3d_test(approxEqual(raycastInfo3.hitFraction, decimal(0.2), epsilon));
            rp3d_test(approxEqual(raycastInfo3.worldPoint.x, hitPoint.x, epsilon));
            rp3d_test(approxEqual(raycastInfo3.worldPoint.y, hitPoint.y, epsilon));
            rp3d_test(approxEqual(raycastInfo3.worldPoint.z, hitPoint.z, epsilon));

            // Collider::raycast()
            RaycastInfo raycastInfo4;
            rp3d_test(mConcaveMeshBody->raycast(ray, raycastInfo4));
            rp3d_test(raycastInfo4.body == mConcaveMeshBody);
            rp3d_test(raycastInfo4.collider == mConcaveMeshCollider);
            rp3d_test(approxEqual(raycastInfo4.hitFraction, decimal(0.2), epsilon));
            rp3d_test(approxEqual(raycastInfo4.worldPoint.x, hitPoint.x, epsilon));
            rp3d_test(approxEqual(raycastInfo4.worldPoint.y, hitPoint.y, epsilon));
            rp3d_test(approxEqual(raycastInfo4.worldPoint.z, hitPoint.z, epsilon));

            // Collider::raycast()
            RaycastInfo raycastInfo5;
            rp3d_test(mConcaveMeshBody->raycast(ray, raycastInfo5));
            rp3d_test(raycastInfo5.body == mConcaveMeshBody);
            rp3d_test(raycastInfo5.collider == mConcaveMeshCollider);
            rp3d_test(approxEqual(raycastInfo5.hitFraction, decimal(0.2), epsilon));
            rp3d_test(approxEqual(raycastInfo5.worldPoint.x, hitPoint.x, epsilon));
            rp3d_test(approxEqual(raycastInfo5.worldPoint.y, hitPoint.y, epsilon));
            rp3d_test(approxEqual(raycastInfo5.worldPoint.z, hitPoint.z, epsilon));

            Ray ray1(mLocalShapeToWorld * Vector3(0, 0, 0), mLocalShapeToWorld * Vector3(5, 7, -1));
            Ray ray2(mLocalShapeToWorld * Vector3(5, 11, 7), mLocalShapeToWorld * Vector3(17, 29, 28));
            Ray ray3(mLocalShapeToWorld * Vector3(1, 2, 3), mLocalShapeToWorld * Vector3(-11, 2, 24));
            Ray ray4(mLocalShapeToWorld * Vector3(10, 10, 10), mLocalShapeToWorld * Vector3(22, 28, 31));
            Ray ray5(mLocalShapeToWorld * Vector3(3, 1, -5), mLocalShapeToWorld * Vector3(-30, 1, -5));
            Ray ray6(mLocalShapeToWorld * Vector3(4, 4, 1), mLocalShapeToWorld * Vector3(4, -30, 1));
            Ray ray7(mLocalShapeToWorld * Vector3(1, -4, 5), mLocalShapeToWorld * Vector3(1, -4, -30));
            Ray ray8(mLocalShapeToWorld * Vector3(-4, 4, 0), mLocalShapeToWorld * Vector3(30, 4, 0));
            Ray ray9(mLocalShapeToWorld * Vector3(0, -4, -7), mLocalShapeToWorld * Vector3(0, 30, -7));
            Ray ray10(mLocalShapeToWorld * Vector3(-3, 0, -6), mLocalShapeToWorld * Vector3(-3, 0, 30));
            Ray ray11(mLocalShapeToWorld * Vector3(3, 1, 2), mLocalShapeToWorld * Vector3(-30, 0, -6));
            Ray ray12(mLocalShapeToWorld * Vector3(1, 4, -1), mLocalShapeToWorld * Vector3(1, -30, -1));
            Ray ray13(mLocalShapeToWorld * Vector3(-1, 2, 5), mLocalShapeToWorld * Vector3(-1, 2, -30));
            Ray ray14(mLocalShapeToWorld * Vector3(-3, 2, -2), mLocalShapeToWorld * Vector3(30, 2, -2));
            Ray ray15(mLocalShapeToWorld * Vector3(0, -4, 1), mLocalShapeToWorld * Vector3(0, 30, 1));
            Ray ray16(mLocalShapeToWorld * Vector3(-1, 2, -7), mLocalShapeToWorld * Vector3(-1, 2, 30));

            // ----- Test raycast miss ----- //
            rp3d_test(!mConcaveMeshBody->raycast(ray1, raycastInfo3));
            //rp3d_test(!mConvexMeshCollider->raycast(ray1, raycastInfo3));
            mCallback.reset();
            mWorld->raycast(ray1, &mCallback);
            rp3d_test(!mCallback.isHit);
            mCallback.reset();
            mWorld->raycast(Ray(ray1.point1, ray1.point2, decimal(0.01)), &mCallback);
            rp3d_test(!mCallback.isHit);
            mCallback.reset();
            mWorld->raycast(Ray(ray1.point1, ray1.point2, decimal(100.0)), &mCallback);
            rp3d_test(!mCallback.isHit);

            rp3d_test(!mConcaveMeshBody->raycast(ray2, raycastInfo3));
            rp3d_test(!mConcaveMeshCollider->raycast(ray2, raycastInfo3));
            mCallback.reset();
            mWorld->raycast(ray2, &mCallback);
            rp3d_test(!mCallback.isHit);

            rp3d_test(!mConcaveMeshBody->raycast(ray3, raycastInfo3));
            rp3d_test(!mConcaveMeshCollider->raycast(ray3, raycastInfo3));
            mCallback.reset();
            mWorld->raycast(ray3, &mCallback);
            rp3d_test(!mCallback.isHit);

            rp3d_test(!mConcaveMeshBody->raycast(ray4, raycastInfo3));
            rp3d_test(!mConcaveMeshCollider->raycast(ray4, raycastInfo3));
            mCallback.reset();
            mWorld->raycast(ray4, &mCallback);
            rp3d_test(!mCallback.isHit);

            rp3d_test(!mConcaveMeshBody->raycast(ray5, raycastInfo3));
            rp3d_test(!mConcaveMeshCollider->raycast(ray5, raycastInfo3));
            mCallback.reset();
            mWorld->raycast(ray5, &mCallback);
            rp3d_test(!mCallback.isHit);

            rp3d_test(!mConcaveMeshBody->raycast(ray6, raycastInfo3));
            rp3d_test(!mConcaveMeshCollider->raycast(ray6, raycastInfo3));
            mCallback.reset();
            mWorld->raycast(ray6, &mCallback);
            rp3d_test(!mCallback.isHit);

            rp3d_test(!mConcaveMeshBody->raycast(ray7, raycastInfo3));
            rp3d_test(!mConcaveMeshCollider->raycast(ray7, raycastInfo3));
            mCallback.reset();
            mWorld->raycast(ray7, &mCallback);
            rp3d_test(!mCallback.isHit);

            rp3d_test(!mConcaveMeshBody->raycast(ray8, raycastInfo3));
            rp3d_test(!mConcaveMeshCollider->raycast(ray8, raycastInfo3));
            mCallback.reset();
            mWorld->raycast(ray8, &mCallback);
            rp3d_test(!mCallback.isHit);

            rp3d_test(!mConcaveMeshBody->raycast(ray9, raycastInfo3));
            rp3d_test(!mConcaveMeshCollider->raycast(ray9, raycastInfo3));
            mCallback.reset();
            mWorld->raycast(ray9, &mCallback);
            rp3d_test(!mCallback.isHit);

            rp3d_test(!mConcaveMeshBody->raycast(ray10, raycastInfo3));
            rp3d_test(!mConcaveMeshCollider->raycast(ray10, raycastInfo3));
            mCallback.reset();
            mWorld->raycast(ray10, &mCallback);
            rp3d_test(!mCallback.isHit);

            mCallback.reset();
            mWorld->raycast(Ray(ray11.point1, ray11.point2, decimal(0.01)), &mCallback);
            rp3d_test(!mCallback.isHit);
            mCallback.reset();
            mWorld->raycast(Ray(ray12.point1, ray12.point2, decimal(0.01)), &mCallback);
            rp3d_test(!mCallback.isHit);
            mCallback.reset();
            mWorld->raycast(Ray(ray13.point1, ray13.point2, decimal(0.01)), &mCallback);
            rp3d_test(!mCallback.isHit);
            mCallback.reset();
            mWorld->raycast(Ray(ray14.point1, ray14.point2, decimal(0.01)), &mCallback);
            rp3d_test(!mCallback.isHit);
            mCallback.reset();
            mWorld->raycast(Ray(ray15.point1, ray15.point2, decimal(0.01)), &mCallback);
            rp3d_test(!mCallback.isHit);
            mCallback.reset();
            mWorld->raycast(Ray(ray16.point1, ray16.point2, decimal(0.01)), &mCallback);
            rp3d_test(!mCallback.isHit);

            // ----- Test raycast hits ----- //
            rp3d_test(mConcaveMeshBody->raycast(ray11, raycastInfo3));
            rp3d_test(mConcaveMeshCollider->raycast(ray11, raycastInfo3));
            mCallback.reset();
            mWorld->raycast(ray11, &mCallback);
            rp3d_test(mCallback.isHit);
            mCallback.reset();
            mWorld->raycast(Ray(ray11.point1, ray11.point2, decimal(0.8)), &mCallback);
            rp3d_test(mCallback.isHit);

            rp3d_test(mConcaveMeshBody->raycast(ray12, raycastInfo3));
            rp3d_test(mConcaveMeshCollider->raycast(ray12, raycastInfo3));
            mCallback.reset();
            mWorld->raycast(ray12, &mCallback);
            rp3d_test(mCallback.isHit);
            mCallback.reset();
            mWorld->raycast(Ray(ray12.point1, ray12.point2, decimal(0.8)), &mCallback);
            rp3d_test(mCallback.isHit);

            rp3d_test(mConcaveMeshBody->raycast(ray13, raycastInfo3));
            rp3d_test(mConcaveMeshCollider->raycast(ray13, raycastInfo3));
            mCallback.reset();
            mWorld->raycast(ray13, &mCallback);
            rp3d_test(mCallback.isHit);
            mCallback.reset();
            mWorld->raycast(Ray(ray13.point1, ray13.point2, decimal(0.8)), &mCallback);
            rp3d_test(mCallback.isHit);

            rp3d_test(mConcaveMeshBody->raycast(ray14, raycastInfo3));
            rp3d_test(mConcaveMeshCollider->raycast(ray14, raycastInfo3));
            mCallback.reset();
            mWorld->raycast(ray14, &mCallback);
            rp3d_test(mCallback.isHit);
            mCallback.reset();
            mWorld->raycast(Ray(ray14.point1, ray14.point2, decimal(0.8)), &mCallback);
            rp3d_test(mCallback.isHit);

            rp3d_test(mConcaveMeshBody->raycast(ray15, raycastInfo3));
            rp3d_test(mConcaveMeshCollider->raycast(ray15, raycastInfo3));
            mCallback.reset();
            mWorld->raycast(ray15, &mCallback);
            rp3d_test(mCallback.isHit);
            mCallback.reset();
            mWorld->raycast(Ray(ray15.point1, ray15.point2, decimal(0.8)), &mCallback);
            rp3d_test(mCallback.isHit);

            rp3d_test(mConcaveMeshBody->raycast(ray16, raycastInfo3));
            rp3d_test(mConcaveMeshCollider->raycast(ray16, raycastInfo3));
            mCallback.reset();
            mWorld->raycast(ray16, &mCallback);
            rp3d_test(mCallback.isHit);
            mCallback.reset();
            mWorld->raycast(Ray(ray16.point1, ray16.point2, decimal(0.8)), &mCallback);
            rp3d_test(mCallback.isHit);
        }

        void testHeightField() {

            // ----- Test feedback data ----- //
            Vector3 point1A = mLocalShapeToWorld * Vector3(0 , 10, 2);
            Vector3 point1B = mLocalShapeToWorld * Vector3(0, -10, 2);
            Ray ray(point1A, point1B);
            Vector3 hitPoint = mLocalShapeToWorld * Vector3(0, 0, 2);

            Vector3 point2A = mLocalShapeToWorld * Vector3(1 , 8, -4);
            Vector3 point2B = mLocalShapeToWorld * Vector3(1, -8, -4);
            Ray rayBottom(point2A, point2B);
            Vector3 hitPoint2 = mLocalShapeToWorld * Vector3(1, 0, -4);

            mCallback.shapeToTest = mHeightFieldCollider;

            // PhysicsWorld::raycast()
            mCallback.reset();
            mWorld->raycast(ray, &mCallback);
            rp3d_test(mCallback.isHit);
            rp3d_test(mCallback.raycastInfo.body == mHeightFieldBody);
            rp3d_test(mCallback.raycastInfo.collider == mHeightFieldCollider);
            rp3d_test(approxEqual(mCallback.raycastInfo.hitFraction, decimal(0.5), epsilon));
            rp3d_test(approxEqual(mCallback.raycastInfo.worldPoint.x, hitPoint.x, epsilon));
            rp3d_test(approxEqual(mCallback.raycastInfo.worldPoint.y, hitPoint.y, epsilon));
            rp3d_test(approxEqual(mCallback.raycastInfo.worldPoint.z, hitPoint.z, epsilon));

            // Correct category filter mask
            mCallback.reset();
            mWorld->raycast(ray, &mCallback, CATEGORY2);
            rp3d_test(mCallback.isHit);

            // Wrong category filter mask
            mCallback.reset();
            mWorld->raycast(ray, &mCallback, CATEGORY1);
            rp3d_test(!mCallback.isHit);

            // RigidBody::raycast()
            RaycastInfo raycastInfo2;
            rp3d_test(mHeightFieldBody->raycast(ray, raycastInfo2));
            rp3d_test(raycastInfo2.body == mHeightFieldBody);
            rp3d_test(raycastInfo2.collider == mHeightFieldCollider);
            rp3d_test(approxEqual(raycastInfo2.hitFraction, decimal(0.5), epsilon));
            rp3d_test(approxEqual(raycastInfo2.worldPoint.x, hitPoint.x, epsilon));
            rp3d_test(approxEqual(raycastInfo2.worldPoint.y, hitPoint.y, epsilon));
            rp3d_test(approxEqual(raycastInfo2.worldPoint.z, hitPoint.z, epsilon));

            // Collider::raycast()
            RaycastInfo raycastInfo3;
            rp3d_test(mHeightFieldCollider->raycast(ray, raycastInfo3));
            rp3d_test(raycastInfo3.body == mHeightFieldBody);
            rp3d_test(raycastInfo3.collider == mHeightFieldCollider);
            rp3d_test(approxEqual(raycastInfo3.hitFraction, decimal(0.5), epsilon));
            rp3d_test(approxEqual(raycastInfo3.worldPoint.x, hitPoint.x, epsilon));
            rp3d_test(approxEqual(raycastInfo3.worldPoint.y, hitPoint.y, epsilon));
            rp3d_test(approxEqual(raycastInfo3.worldPoint.z, hitPoint.z, epsilon));

            mCallback.reset();
            mWorld->raycast(rayBottom, &mCallback);
            rp3d_test(mCallback.isHit);
            rp3d_test(mCallback.raycastInfo.body == mHeightFieldBody);
            rp3d_test(mCallback.raycastInfo.collider == mHeightFieldCollider);
            rp3d_test(approxEqual(mCallback.raycastInfo.hitFraction, decimal(0.5), epsilon));
            rp3d_test(approxEqual(mCallback.raycastInfo.worldPoint.x, hitPoint2.x, epsilon));
            rp3d_test(approxEqual(mCallback.raycastInfo.worldPoint.y, hitPoint2.y, epsilon));
            rp3d_test(approxEqual(mCallback.raycastInfo.worldPoint.z, hitPoint2.z, epsilon));

            // RigidBody::raycast()
            RaycastInfo raycastInfo5;
            rp3d_test(mHeightFieldBody->raycast(rayBottom, raycastInfo5));
            rp3d_test(raycastInfo5.body == mHeightFieldBody);
            rp3d_test(raycastInfo5.collider == mHeightFieldCollider);
            rp3d_test(approxEqual(raycastInfo5.hitFraction, decimal(0.5), epsilon));
            rp3d_test(approxEqual(raycastInfo5.worldPoint.x, hitPoint2.x, epsilon));
            rp3d_test(approxEqual(raycastInfo5.worldPoint.y, hitPoint2.y, epsilon));
            rp3d_test(approxEqual(raycastInfo5.worldPoint.z, hitPoint2.z, epsilon));

            // Collider::raycast()
            RaycastInfo raycastInfo6;
            rp3d_test(mHeightFieldCollider->raycast(rayBottom, raycastInfo6));
            rp3d_test(raycastInfo6.body == mHeightFieldBody);
            rp3d_test(raycastInfo6.collider == mHeightFieldCollider);
            rp3d_test(approxEqual(raycastInfo6.hitFraction, decimal(0.5), epsilon));
            rp3d_test(approxEqual(raycastInfo6.worldPoint.x, hitPoint2.x, epsilon));
            rp3d_test(approxEqual(raycastInfo6.worldPoint.y, hitPoint2.y, epsilon));
            rp3d_test(approxEqual(raycastInfo6.worldPoint.z, hitPoint2.z, epsilon));

            Ray ray1(mLocalShapeToWorld * Vector3(0, 5, 0), mLocalShapeToWorld * Vector3(5, 7, 5));
            Ray ray2(mLocalShapeToWorld * Vector3(-4, -4, 7), mLocalShapeToWorld * Vector3(-4, 15, 7));
            Ray ray3(mLocalShapeToWorld * Vector3(23, 7, 2), mLocalShapeToWorld * Vector3(23, 1, 2));
            Ray ray4(mLocalShapeToWorld * Vector3(10, 3, 10), mLocalShapeToWorld * Vector3(22, 3, 31));
            Ray ray5(mLocalShapeToWorld * Vector3(4, 10, -1), mLocalShapeToWorld * Vector3(4, 3, -1));

            Ray ray11(mLocalShapeToWorld * Vector3(3, 15, 0.5), mLocalShapeToWorld * Vector3(3, -1, 0.5));
            Ray ray12(mLocalShapeToWorld * Vector3(0, 45, 0), mLocalShapeToWorld * Vector3(0, -5, 0));
            Ray ray13(mLocalShapeToWorld * Vector3(1, 23, 2), mLocalShapeToWorld * Vector3(1, -23, 2));
            Ray ray14(mLocalShapeToWorld * Vector3(3, 2, 3), mLocalShapeToWorld * Vector3(3, -0.5, 3));

            // ----- Test raycast miss ----- //
            rp3d_test(!mHeightFieldBody->raycast(ray1, raycastInfo3));
            rp3d_test(!mHeightFieldCollider->raycast(ray1, raycastInfo3));
            mCallback.reset();
            mWorld->raycast(ray1, &mCallback);
            rp3d_test(!mCallback.isHit);
            mCallback.reset();
            mWorld->raycast(Ray(ray1.point1, ray1.point2, decimal(0.01)), &mCallback);
            rp3d_test(!mCallback.isHit);
            mCallback.reset();
            mWorld->raycast(Ray(ray1.point1, ray1.point2, decimal(100.0)), &mCallback);
            rp3d_test(!mCallback.isHit);

            rp3d_test(!mHeightFieldBody->raycast(ray2, raycastInfo3));
            rp3d_test(!mHeightFieldCollider->raycast(ray2, raycastInfo3));
            mCallback.reset();
            mWorld->raycast(ray2, &mCallback);
            rp3d_test(!mCallback.isHit);

            rp3d_test(!mHeightFieldBody->raycast(ray3, raycastInfo3));
            rp3d_test(!mHeightFieldCollider->raycast(ray3, raycastInfo3));
            mCallback.reset();
            mWorld->raycast(ray3, &mCallback);
            rp3d_test(!mCallback.isHit);

            rp3d_test(!mHeightFieldBody->raycast(ray4, raycastInfo3));
            rp3d_test(!mHeightFieldCollider->raycast(ray4, raycastInfo3));
            mCallback.reset();
            mWorld->raycast(ray4, &mCallback);
            rp3d_test(!mCallback.isHit);

            rp3d_test(!mHeightFieldBody->raycast(ray5, raycastInfo3));
            rp3d_test(!mHeightFieldCollider->raycast(ray5, raycastInfo3));
            mCallback.reset();
            mWorld->raycast(ray5, &mCallback);
            rp3d_test(!mCallback.isHit);

            mCallback.reset();

            // ----- Test raycast hits ----- //
            rp3d_test(mHeightFieldBody->raycast(ray11, raycastInfo3));
            rp3d_test(mHeightFieldCollider->raycast(ray11, raycastInfo3));
            mCallback.reset();
            mWorld->raycast(ray11, &mCallback);
            rp3d_test(mCallback.isHit);
            mCallback.reset();
            mWorld->raycast(Ray(ray11.point1, ray11.point2, decimal(0.95)), &mCallback);
            rp3d_test(mCallback.isHit);

            rp3d_test(mHeightFieldBody->raycast(ray12, raycastInfo3));
            rp3d_test(mHeightFieldCollider->raycast(ray12, raycastInfo3));
            mCallback.reset();
            mWorld->raycast(ray12, &mCallback);
            rp3d_test(mCallback.isHit);
            mCallback.reset();
            mWorld->raycast(Ray(ray12.point1, ray12.point2, decimal(0.92)), &mCallback);
            rp3d_test(mCallback.isHit);

            rp3d_test(mHeightFieldBody->raycast(ray13, raycastInfo3));
            rp3d_test(mHeightFieldCollider->raycast(ray13, raycastInfo3));
            mCallback.reset();
            mWorld->raycast(ray13, &mCallback);
            rp3d_test(mCallback.isHit);
            mCallback.reset();
            mWorld->raycast(Ray(ray13.point1, ray13.point2, decimal(0.8)), &mCallback);
            rp3d_test(mCallback.isHit);

            rp3d_test(mHeightFieldBody->raycast(ray14, raycastInfo3));
            rp3d_test(mHeightFieldCollider->raycast(ray14, raycastInfo3));
            mCallback.reset();
            mWorld->raycast(ray14, &mCallback);
            rp3d_test(mCallback.isHit);
            mCallback.reset();
            mWorld->raycast(Ray(ray14.point1, ray14.point2, decimal(0.82)), &mCallback);
            rp3d_test(mCallback.isHit);
        }
};

}

#endif
