/********************************************************************************
* ReactPhysics3D physics library, http://code.google.com/p/reactphysics3d/      *
* Copyright (c) 2010-2013 Daniel Chappuis                                       *
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
#include "engine/CollisionWorld.h"
#include "body/CollisionBody.h"
#include "collision/shapes/BoxShape.h"
#include "collision/shapes/SphereShape.h"
#include "collision/shapes/CapsuleShape.h"
#include "collision/shapes/ConeShape.h"
#include "collision/shapes/ConvexMeshShape.h"
#include "collision/shapes/CylinderShape.h"

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
        ProxyShape* shapeToTest;
        bool isHit;

        WorldRaycastCallback() {
            isHit = false;
            shapeToTest = NULL;
        }

        virtual decimal notifyRaycastHit(const RaycastInfo& info) {

            if (shapeToTest->getBody()->getID() == info.body->getID()) {
                raycastInfo.body = info.body;
                raycastInfo.hitFraction = info.hitFraction;
                raycastInfo.proxyShape = info.proxyShape;
                raycastInfo.worldNormal = info.worldNormal;
                raycastInfo.worldPoint = info.worldPoint;
                isHit = true;
            }

            // Return a fraction of 1.0 because we need to gather all hits
            return decimal(1.0);
        }

        void reset() {
            raycastInfo.body = NULL;
            raycastInfo.hitFraction = decimal(0.0);
            raycastInfo.proxyShape = NULL;
            raycastInfo.worldNormal.setToZero();
            raycastInfo.worldPoint.setToZero();
            isHit = false;
        }
};

// Class TestPointInside
/**
 * Unit test for the CollisionBody::testPointInside() method.
 */
class TestRaycast : public Test {

    private :

        // ---------- Atributes ---------- //

        // Raycast callback class
        WorldRaycastCallback mCallback;

        // Epsilon
        decimal epsilon;

        // Physics world
        CollisionWorld* mWorld;

        // Bodies
        CollisionBody* mBoxBody;
        CollisionBody* mSphereBody;
        CollisionBody* mCapsuleBody;
        CollisionBody* mConeBody;
        CollisionBody* mConvexMeshBody;
        CollisionBody* mConvexMeshBodyEdgesInfo;
        CollisionBody* mCylinderBody;
        CollisionBody* mCompoundBody;

        // Transform
        Transform mBodyTransform;
        Transform mShapeTransform;
        Transform mLocalShapeToWorld;
        Transform mLocalShape2ToWorld;

        // Collision Shapes
        ProxyShape* mBoxShape;
        ProxyShape* mSphereShape;
        ProxyShape* mCapsuleShape;
        ProxyShape* mConeShape;
        ProxyShape* mConvexMeshShape;
        ProxyShape* mConvexMeshShapeEdgesInfo;
        ProxyShape* mCylinderShape;
        ProxyShape* mCompoundSphereShape;
        ProxyShape* mCompoundCylinderShape;

    public :

        // ---------- Methods ---------- //

        /// Constructor
        TestRaycast(const std::string& name) : Test(name) {

            epsilon = decimal(0.0001);

            // Create the world
            mWorld = new CollisionWorld();

            // Body transform
            Vector3 position(-3, 2, 7);
            Quaternion orientation(PI / 5, PI / 6, PI / 7);
            mBodyTransform = Transform(position, orientation);

            // Create the bodies
            mBoxBody = mWorld->createCollisionBody(mBodyTransform);
            mSphereBody = mWorld->createCollisionBody(mBodyTransform);
            mCapsuleBody = mWorld->createCollisionBody(mBodyTransform);
            mConeBody = mWorld->createCollisionBody(mBodyTransform);
            mConvexMeshBody = mWorld->createCollisionBody(mBodyTransform);
            mConvexMeshBodyEdgesInfo = mWorld->createCollisionBody(mBodyTransform);
            mCylinderBody = mWorld->createCollisionBody(mBodyTransform);
            mCompoundBody = mWorld->createCollisionBody(mBodyTransform);

            // Collision shape transform
            Vector3 shapePosition(1, -4, -3);
            Quaternion shapeOrientation(3 * PI / 6 , -PI / 8, PI / 3);
            mShapeTransform = Transform(shapePosition, shapeOrientation);

            // Compute the the transform from a local shape point to world-space
            mLocalShapeToWorld = mBodyTransform * mShapeTransform;

            // Create collision shapes
            BoxShape boxShape(Vector3(2, 3, 4), 0);
            mBoxShape = mBoxBody->addCollisionShape(boxShape, mShapeTransform);

            SphereShape sphereShape(3);
            mSphereShape = mSphereBody->addCollisionShape(sphereShape, mShapeTransform);

            CapsuleShape capsuleShape(2, 5);
            mCapsuleShape = mCapsuleBody->addCollisionShape(capsuleShape, mShapeTransform);

            ConeShape coneShape(2, 6, 0);
            mConeShape = mConeBody->addCollisionShape(coneShape, mShapeTransform);

            ConvexMeshShape convexMeshShape(0);             // Box of dimension (2, 3, 4)
            convexMeshShape.addVertex(Vector3(-2, -3, -4));
            convexMeshShape.addVertex(Vector3(2, -3, -4));
            convexMeshShape.addVertex(Vector3(2, -3, 4));
            convexMeshShape.addVertex(Vector3(-2, -3, 4));
            convexMeshShape.addVertex(Vector3(-2, 3, -4));
            convexMeshShape.addVertex(Vector3(2, 3, -4));
            convexMeshShape.addVertex(Vector3(2, 3, 4));
            convexMeshShape.addVertex(Vector3(-2, 3, 4));
            mConvexMeshShape = mConvexMeshBody->addCollisionShape(convexMeshShape, mShapeTransform);

            ConvexMeshShape convexMeshShapeEdgesInfo(0);
            convexMeshShapeEdgesInfo.addVertex(Vector3(-2, -3, -4));
            convexMeshShapeEdgesInfo.addVertex(Vector3(2, -3, -4));
            convexMeshShapeEdgesInfo.addVertex(Vector3(2, -3, 4));
            convexMeshShapeEdgesInfo.addVertex(Vector3(-2, -3, 4));
            convexMeshShapeEdgesInfo.addVertex(Vector3(-2, 3, -4));
            convexMeshShapeEdgesInfo.addVertex(Vector3(2, 3, -4));
            convexMeshShapeEdgesInfo.addVertex(Vector3(2, 3, 4));
            convexMeshShapeEdgesInfo.addVertex(Vector3(-2, 3, 4));
            convexMeshShapeEdgesInfo.addEdge(0, 1);
            convexMeshShapeEdgesInfo.addEdge(1, 2);
            convexMeshShapeEdgesInfo.addEdge(2, 3);
            convexMeshShapeEdgesInfo.addEdge(0, 3);
            convexMeshShapeEdgesInfo.addEdge(4, 5);
            convexMeshShapeEdgesInfo.addEdge(5, 6);
            convexMeshShapeEdgesInfo.addEdge(6, 7);
            convexMeshShapeEdgesInfo.addEdge(4, 7);
            convexMeshShapeEdgesInfo.addEdge(0, 4);
            convexMeshShapeEdgesInfo.addEdge(1, 5);
            convexMeshShapeEdgesInfo.addEdge(2, 6);
            convexMeshShapeEdgesInfo.addEdge(3, 7);
            convexMeshShapeEdgesInfo.setIsEdgesInformationUsed(true);
            mConvexMeshShapeEdgesInfo = mConvexMeshBodyEdgesInfo->addCollisionShape(
                                                                     convexMeshShapeEdgesInfo,
                                                                     mShapeTransform);

            CylinderShape cylinderShape(2, 5, 0);
            mCylinderShape = mCylinderBody->addCollisionShape(cylinderShape, mShapeTransform);

            // Compound shape is a cylinder and a sphere
            Vector3 positionShape2(Vector3(4, 2, -3));
            Quaternion orientationShape2(-3 *PI / 8, 1.5 * PI/ 3, PI / 13);
            Transform shapeTransform2(positionShape2, orientationShape2);
            mLocalShape2ToWorld = mBodyTransform * shapeTransform2;
            mCompoundCylinderShape = mCompoundBody->addCollisionShape(cylinderShape, mShapeTransform);
            mCompoundSphereShape = mCompoundBody->addCollisionShape(sphereShape, shapeTransform2);

            // Assign proxy shapes to the different categories
            mBoxShape->setCollisionCategoryBits(CATEGORY1);
            mSphereShape->setCollisionCategoryBits(CATEGORY1);
            mCapsuleShape->setCollisionCategoryBits(CATEGORY1);
            mConeShape->setCollisionCategoryBits(CATEGORY2);
            mConvexMeshShape->setCollisionCategoryBits(CATEGORY2);
            mConvexMeshShapeEdgesInfo->setCollisionCategoryBits(CATEGORY2);
            mCylinderShape->setCollisionCategoryBits(CATEGORY2);
            mCompoundSphereShape->setCollisionCategoryBits(CATEGORY2);
            mCompoundCylinderShape->setCollisionCategoryBits(CATEGORY2);
        }

        /// Run the tests
        void run() {
            testBox();
            testSphere();
            testCapsule();
            testCone();
            testConvexMesh();
            testCylinder();
            testCompound();
        }

        /// Test the ProxyBoxShape::raycast(), CollisionBody::raycast() and
        /// CollisionWorld::raycast() methods.
        void testBox() {

            // ----- Test feedback data ----- //
            Vector3 point1 = mLocalShapeToWorld * Vector3(1 , 2, 10);
            Vector3 point2 = mLocalShapeToWorld * Vector3(1, 2, -20);
            Ray ray(point1, point2);
            Vector3 hitPoint = mLocalShapeToWorld * Vector3(1, 2, 4);

            mCallback.shapeToTest = mBoxShape;

            // CollisionWorld::raycast()
            mCallback.reset();
            mWorld->raycast(ray, &mCallback);
            test(mCallback.isHit);
            test(mCallback.raycastInfo.body == mBoxBody);
            test(mCallback.raycastInfo.proxyShape == mBoxShape);
            test(approxEqual(mCallback.raycastInfo.hitFraction, decimal(0.2), epsilon));
            test(approxEqual(mCallback.raycastInfo.worldPoint.x, hitPoint.x, epsilon));
            test(approxEqual(mCallback.raycastInfo.worldPoint.y, hitPoint.y, epsilon));
            test(approxEqual(mCallback.raycastInfo.worldPoint.z, hitPoint.z, epsilon));

            // Correct category filter mask
            mCallback.reset();
            mWorld->raycast(ray, &mCallback, CATEGORY1);
            test(mCallback.isHit);

            // Wrong category filter mask
            mCallback.reset();
            mWorld->raycast(ray, &mCallback, CATEGORY2);
            test(!mCallback.isHit);

            // CollisionBody::raycast()
            RaycastInfo raycastInfo2;
            test(mBoxBody->raycast(ray, raycastInfo2));
            test(raycastInfo2.body == mBoxBody);
            test(raycastInfo2.proxyShape == mBoxShape);
            test(approxEqual(raycastInfo2.hitFraction, decimal(0.2), epsilon));
            test(approxEqual(raycastInfo2.worldPoint.x, hitPoint.x, epsilon));
            test(approxEqual(raycastInfo2.worldPoint.y, hitPoint.y, epsilon));
            test(approxEqual(raycastInfo2.worldPoint.z, hitPoint.z, epsilon));

            // ProxyCollisionShape::raycast()
            RaycastInfo raycastInfo3;
            test(mBoxShape->raycast(ray, raycastInfo3));
            test(raycastInfo3.body == mBoxBody);
            test(raycastInfo3.proxyShape == mBoxShape);
            test(approxEqual(raycastInfo3.hitFraction, decimal(0.2), epsilon));
            test(approxEqual(raycastInfo3.worldPoint.x, hitPoint.x, epsilon));
            test(approxEqual(raycastInfo3.worldPoint.y, hitPoint.y, epsilon));
            test(approxEqual(raycastInfo3.worldPoint.z, hitPoint.z, epsilon));

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
            test(!mBoxBody->raycast(ray1, raycastInfo3));
            test(!mBoxShape->raycast(ray1, raycastInfo3));
            mCallback.reset();
            mWorld->raycast(ray1, &mCallback);
            test(!mCallback.isHit);
            mCallback.reset();
            mWorld->raycast(Ray(ray1.point1, ray1.point2, decimal(0.01)), &mCallback);
            test(!mCallback.isHit);
            mCallback.reset();
            mWorld->raycast(Ray(ray1.point1, ray1.point2, decimal(100.0)), &mCallback);
            test(!mCallback.isHit);

            test(!mBoxBody->raycast(ray2, raycastInfo3));
            test(!mBoxShape->raycast(ray2, raycastInfo3));
            mCallback.reset();
            mWorld->raycast(ray2, &mCallback);
            test(!mCallback.isHit);

            test(!mBoxBody->raycast(ray3, raycastInfo3));
            test(!mBoxShape->raycast(ray3, raycastInfo3));
            mCallback.reset();
            mWorld->raycast(ray3, &mCallback);
            test(!mCallback.isHit);

            test(!mBoxBody->raycast(ray4, raycastInfo3));
            test(!mBoxShape->raycast(ray4, raycastInfo3));
            mCallback.reset();
            mWorld->raycast(ray4, &mCallback);
            test(!mCallback.isHit);

            test(!mBoxBody->raycast(ray5, raycastInfo3));
            test(!mBoxShape->raycast(ray5, raycastInfo3));
            mCallback.reset();
            mWorld->raycast(ray5, &mCallback);
            test(!mCallback.isHit);

            test(!mBoxBody->raycast(ray6, raycastInfo3));
            test(!mBoxShape->raycast(ray6, raycastInfo3));
            mCallback.reset();
            mWorld->raycast(ray6, &mCallback);
            test(!mCallback.isHit);

            test(!mBoxBody->raycast(ray7, raycastInfo3));
            test(!mBoxShape->raycast(ray7, raycastInfo3));
            mCallback.reset();
            mWorld->raycast(ray7, &mCallback);
            test(!mCallback.isHit);

            test(!mBoxBody->raycast(ray8, raycastInfo3));
            test(!mBoxShape->raycast(ray8, raycastInfo3));
            mCallback.reset();
            mWorld->raycast(ray8, &mCallback);
            test(!mCallback.isHit);

            test(!mBoxBody->raycast(ray9, raycastInfo3));
            test(!mBoxShape->raycast(ray9, raycastInfo3));
            mCallback.reset();
            mWorld->raycast(ray9, &mCallback);
            test(!mCallback.isHit);

            test(!mBoxBody->raycast(ray10, raycastInfo3));
            test(!mBoxShape->raycast(ray10, raycastInfo3));
            mCallback.reset();
            mWorld->raycast(ray10, &mCallback);
            test(!mCallback.isHit);

            mCallback.reset();
            mWorld->raycast(Ray(ray11.point1, ray11.point2, decimal(0.01)), &mCallback);
            test(!mCallback.isHit);
            mCallback.reset();
            mWorld->raycast(Ray(ray12.point1, ray12.point2, decimal(0.01)), &mCallback);
            test(!mCallback.isHit);
            mCallback.reset();
            mWorld->raycast(Ray(ray13.point1, ray13.point2, decimal(0.01)), &mCallback);
            test(!mCallback.isHit);
            mCallback.reset();
            mWorld->raycast(Ray(ray14.point1, ray14.point2, decimal(0.01)), &mCallback);
            test(!mCallback.isHit);
            mCallback.reset();
            mWorld->raycast(Ray(ray15.point1, ray15.point2, decimal(0.01)), &mCallback);
            test(!mCallback.isHit);
            mCallback.reset();
            mWorld->raycast(Ray(ray16.point1, ray16.point2, decimal(0.01)), &mCallback);
            test(!mCallback.isHit);

            // ----- Test raycast hits ----- //
            test(mBoxBody->raycast(ray11, raycastInfo3));
            test(mBoxShape->raycast(ray11, raycastInfo3));
            mCallback.reset();
            mWorld->raycast(ray11, &mCallback);
            test(mCallback.isHit);
            mCallback.reset();
            mWorld->raycast(Ray(ray11.point1, ray11.point2, decimal(0.8)), &mCallback);
            test(mCallback.isHit);

            test(mBoxBody->raycast(ray12, raycastInfo3));
            test(mBoxShape->raycast(ray12, raycastInfo3));
            mCallback.reset();
            mWorld->raycast(ray12, &mCallback);
            test(mCallback.isHit);
            mCallback.reset();
            mWorld->raycast(Ray(ray12.point1, ray12.point2, decimal(0.8)), &mCallback);
            test(mCallback.isHit);

            test(mBoxBody->raycast(ray13, raycastInfo3));
            test(mBoxShape->raycast(ray13, raycastInfo3));
            mCallback.reset();
            mWorld->raycast(ray13, &mCallback);
            test(mCallback.isHit);
            mCallback.reset();
            mWorld->raycast(Ray(ray13.point1, ray13.point2, decimal(0.8)), &mCallback);
            test(mCallback.isHit);

            test(mBoxBody->raycast(ray14, raycastInfo3));
            test(mBoxShape->raycast(ray14, raycastInfo3));
            mCallback.reset();
            mWorld->raycast(ray14, &mCallback);
            test(mCallback.isHit);
            mCallback.reset();
            mWorld->raycast(Ray(ray14.point1, ray14.point2, decimal(0.8)), &mCallback);
            test(mCallback.isHit);

            test(mBoxBody->raycast(ray15, raycastInfo3));
            test(mBoxShape->raycast(ray15, raycastInfo3));
            mCallback.reset();
            mWorld->raycast(ray15, &mCallback);
            test(mCallback.isHit);
            mCallback.reset();
            mWorld->raycast(Ray(ray15.point1, ray15.point2, decimal(0.8)), &mCallback);
            test(mCallback.isHit);

            test(mBoxBody->raycast(ray16, raycastInfo3));
            test(mBoxShape->raycast(ray16, raycastInfo3));
            mCallback.reset();
            mWorld->raycast(ray16, &mCallback);
            test(mCallback.isHit);
            mCallback.reset();
            mWorld->raycast(Ray(ray16.point1, ray16.point2, decimal(0.8)), &mCallback);
            test(mCallback.isHit);
        }

        /// Test the ProxySphereShape::raycast(), CollisionBody::raycast() and
        /// CollisionWorld::raycast() methods.
        void testSphere() {

            // ----- Test feedback data ----- //
            Vector3 point1 = mLocalShapeToWorld * Vector3(-5 , 0, 0);
            Vector3 point2 = mLocalShapeToWorld * Vector3(5, 0, 0);
            Ray ray(point1, point2);
            Vector3 hitPoint = mLocalShapeToWorld * Vector3(-3, 0, 0);

            mCallback.shapeToTest = mSphereShape;

            // CollisionWorld::raycast()
            mCallback.reset();
            mWorld->raycast(ray, &mCallback);
            test(mCallback.isHit);
            test(mCallback.raycastInfo.body == mSphereBody);
            test(mCallback.raycastInfo.proxyShape == mSphereShape);
            test(approxEqual(mCallback.raycastInfo.hitFraction, 0.2, epsilon));
            test(approxEqual(mCallback.raycastInfo.worldPoint.x, hitPoint.x, epsilon));
            test(approxEqual(mCallback.raycastInfo.worldPoint.y, hitPoint.y, epsilon));
            test(approxEqual(mCallback.raycastInfo.worldPoint.z, hitPoint.z, epsilon));

            // Correct category filter mask
            mCallback.reset();
            mWorld->raycast(ray, &mCallback, CATEGORY1);
            test(mCallback.isHit);

            // Wrong category filter mask
            mCallback.reset();
            mWorld->raycast(ray, &mCallback, CATEGORY2);
            test(!mCallback.isHit);

            // CollisionBody::raycast()
            RaycastInfo raycastInfo2;
            test(mSphereBody->raycast(ray, raycastInfo2));
            test(raycastInfo2.body == mSphereBody);
            test(raycastInfo2.proxyShape == mSphereShape);
            test(approxEqual(raycastInfo2.hitFraction, 0.2, epsilon));
            test(approxEqual(raycastInfo2.worldPoint.x, hitPoint.x, epsilon));
            test(approxEqual(raycastInfo2.worldPoint.y, hitPoint.y, epsilon));
            test(approxEqual(raycastInfo2.worldPoint.z, hitPoint.z, epsilon));

            // ProxyCollisionShape::raycast()
            RaycastInfo raycastInfo3;
            test(mSphereShape->raycast(ray, raycastInfo3));
            test(raycastInfo3.body == mSphereBody);
            test(raycastInfo3.proxyShape == mSphereShape);
            test(approxEqual(raycastInfo3.hitFraction, 0.2, epsilon));
            test(approxEqual(raycastInfo3.worldPoint.x, hitPoint.x, epsilon));
            test(approxEqual(raycastInfo3.worldPoint.y, hitPoint.y, epsilon));
            test(approxEqual(raycastInfo3.worldPoint.z, hitPoint.z, epsilon));

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
            test(!mSphereBody->raycast(ray1, raycastInfo3));
            test(!mSphereShape->raycast(ray1, raycastInfo3));
            mCallback.reset();
            mWorld->raycast(ray1, &mCallback);
            test(!mCallback.isHit);
            mCallback.reset();
            mWorld->raycast(Ray(ray1.point1, ray1.point2, decimal(0.01)), &mCallback);
            test(!mCallback.isHit);
            mCallback.reset();
            mWorld->raycast(Ray(ray1.point1, ray1.point2, decimal(100.0)), &mCallback);
            test(!mCallback.isHit);

            test(!mSphereBody->raycast(ray2, raycastInfo3));
            test(!mSphereShape->raycast(ray2, raycastInfo3));
            mCallback.reset();
            mWorld->raycast(ray2, &mCallback);
            test(!mCallback.isHit);

            test(!mSphereBody->raycast(ray3, raycastInfo3));
            test(!mSphereShape->raycast(ray3, raycastInfo3));
            mCallback.reset();
            mWorld->raycast(ray3, &mCallback);
            test(!mCallback.isHit);

            test(!mSphereBody->raycast(ray4, raycastInfo3));
            test(!mSphereShape->raycast(ray4, raycastInfo3));
            mCallback.reset();
            mWorld->raycast(ray4, &mCallback);
            test(!mCallback.isHit);

            test(!mSphereBody->raycast(ray5, raycastInfo3));
            test(!mSphereShape->raycast(ray5, raycastInfo3));
            mCallback.reset();
            mWorld->raycast(ray5, &mCallback);
            test(!mCallback.isHit);

            test(!mSphereBody->raycast(ray6, raycastInfo3));
            test(!mSphereShape->raycast(ray6, raycastInfo3));
            mCallback.reset();
            mWorld->raycast(ray6, &mCallback);
            test(!mCallback.isHit);

            test(!mSphereBody->raycast(ray7, raycastInfo3));
            test(!mSphereShape->raycast(ray7, raycastInfo3));
            mCallback.reset();
            mWorld->raycast(ray7, &mCallback);
            test(!mCallback.isHit);

            test(!mSphereBody->raycast(ray8, raycastInfo3));
            test(!mSphereShape->raycast(ray8, raycastInfo3));
            mCallback.reset();
            mWorld->raycast(ray8, &mCallback);
            test(!mCallback.isHit);

            test(!mSphereBody->raycast(ray9, raycastInfo3));
            test(!mSphereShape->raycast(ray9, raycastInfo3));
            mCallback.reset();
            mWorld->raycast(ray9, &mCallback);
            test(!mCallback.isHit);

            test(!mSphereBody->raycast(ray10, raycastInfo3));
            test(!mSphereShape->raycast(ray10, raycastInfo3));
            mCallback.reset();
            mWorld->raycast(ray10, &mCallback);
            test(!mCallback.isHit);

            mCallback.reset();
            mWorld->raycast(Ray(ray11.point1, ray11.point2, decimal(0.01)), &mCallback);
            test(!mCallback.isHit);
            mCallback.reset();
            mWorld->raycast(Ray(ray12.point1, ray12.point2, decimal(0.01)), &mCallback);
            test(!mCallback.isHit);
            mCallback.reset();
            mWorld->raycast(Ray(ray13.point1, ray13.point2, decimal(0.01)), &mCallback);
            test(!mCallback.isHit);
            mCallback.reset();
            mWorld->raycast(Ray(ray14.point1, ray14.point2, decimal(0.01)), &mCallback);
            test(!mCallback.isHit);
            mCallback.reset();
            mWorld->raycast(Ray(ray15.point1, ray15.point2, decimal(0.01)), &mCallback);
            test(!mCallback.isHit);
            mCallback.reset();
            mWorld->raycast(Ray(ray16.point1, ray16.point2, decimal(0.01)), &mCallback);
            test(!mCallback.isHit);

            // ----- Test raycast hits ----- //
            test(mSphereBody->raycast(ray11, raycastInfo3));
            test(mSphereShape->raycast(ray11, raycastInfo3));
            mCallback.reset();
            mWorld->raycast(ray11, &mCallback);
            test(mCallback.isHit);
            mCallback.reset();
            mWorld->raycast(Ray(ray11.point1, ray11.point2, decimal(0.8)), &mCallback);
            test(mCallback.isHit);

            test(mSphereBody->raycast(ray12, raycastInfo3));
            test(mSphereShape->raycast(ray12, raycastInfo3));
            mCallback.reset();
            mWorld->raycast(ray12, &mCallback);
            test(mCallback.isHit);
            mCallback.reset();
            mWorld->raycast(Ray(ray12.point1, ray12.point2, decimal(0.8)), &mCallback);
            test(mCallback.isHit);

            test(mSphereBody->raycast(ray13, raycastInfo3));
            test(mSphereShape->raycast(ray13, raycastInfo3));
            mCallback.reset();
            mWorld->raycast(ray13, &mCallback);
            mCallback.reset();
            mWorld->raycast(Ray(ray13.point1, ray13.point2, decimal(0.8)), &mCallback);

            test(mSphereBody->raycast(ray14, raycastInfo3));
            test(mSphereShape->raycast(ray14, raycastInfo3));
            mCallback.reset();
            mWorld->raycast(ray14, &mCallback);
            test(mCallback.isHit);
            mCallback.reset();
            mWorld->raycast(Ray(ray14.point1, ray14.point2, decimal(0.8)), &mCallback);
            test(mCallback.isHit);

            test(mSphereBody->raycast(ray15, raycastInfo3));
            test(mSphereShape->raycast(ray15, raycastInfo3));
            mCallback.reset();
            mWorld->raycast(ray15, &mCallback);
            test(mCallback.isHit);
            mCallback.reset();
            mWorld->raycast(Ray(ray15.point1, ray15.point2, decimal(0.8)), &mCallback);
            test(mCallback.isHit);

            test(mSphereBody->raycast(ray16, raycastInfo3));
            test(mSphereShape->raycast(ray16, raycastInfo3));
            mCallback.reset();
            mWorld->raycast(ray16, &mCallback);
            test(mCallback.isHit);
            mCallback.reset();
            mWorld->raycast(Ray(ray16.point1, ray16.point2, decimal(0.8)), &mCallback);
            test(mCallback.isHit);
        }

        /// Test the ProxyCapsuleShape::raycast(), CollisionBody::raycast() and
        /// CollisionWorld::raycast() methods.
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

            mCallback.shapeToTest = mCapsuleShape;

            // CollisionWorld::raycast()
            mCallback.reset();
            mWorld->raycast(ray, &mCallback);
            test(mCallback.isHit);
            test(mCallback.raycastInfo.body == mCapsuleBody);
            test(mCallback.raycastInfo.proxyShape == mCapsuleShape);
            test(approxEqual(mCallback.raycastInfo.hitFraction, decimal(0.2), epsilon));
            test(approxEqual(mCallback.raycastInfo.worldPoint.x, hitPoint.x, epsilon));
            test(approxEqual(mCallback.raycastInfo.worldPoint.y, hitPoint.y, epsilon));
            test(approxEqual(mCallback.raycastInfo.worldPoint.z, hitPoint.z, epsilon));

            // Correct category filter mask
            mCallback.reset();
            mWorld->raycast(ray, &mCallback, CATEGORY1);
            test(mCallback.isHit);

            // Wrong category filter mask
            mCallback.reset();
            mWorld->raycast(ray, &mCallback, CATEGORY2);
            test(!mCallback.isHit);

            // CollisionBody::raycast()
            RaycastInfo raycastInfo2;
            test(mCapsuleBody->raycast(ray, raycastInfo2));
            test(raycastInfo2.body == mCapsuleBody);
            test(raycastInfo2.proxyShape == mCapsuleShape);
            test(approxEqual(raycastInfo2.hitFraction, decimal(0.2), epsilon));
            test(approxEqual(raycastInfo2.worldPoint.x, hitPoint.x, epsilon));
            test(approxEqual(raycastInfo2.worldPoint.y, hitPoint.y, epsilon));
            test(approxEqual(raycastInfo2.worldPoint.z, hitPoint.z, epsilon));

            // ProxyCollisionShape::raycast()
            RaycastInfo raycastInfo3;
            test(mCapsuleShape->raycast(ray, raycastInfo3));
            test(raycastInfo3.body == mCapsuleBody);
            test(raycastInfo3.proxyShape == mCapsuleShape);
            test(approxEqual(raycastInfo3.hitFraction, decimal(0.2), epsilon));
            test(approxEqual(raycastInfo3.worldPoint.x, hitPoint.x, epsilon));
            test(approxEqual(raycastInfo3.worldPoint.y, hitPoint.y, epsilon));
            test(approxEqual(raycastInfo3.worldPoint.z, hitPoint.z, epsilon));

            RaycastInfo raycastInfo4;
            test(mCapsuleShape->raycast(rayTop, raycastInfo4));
            test(raycastInfo4.body == mCapsuleBody);
            test(raycastInfo4.proxyShape == mCapsuleShape);
            test(approxEqual(raycastInfo4.hitFraction, decimal(0.2), epsilon));
            test(approxEqual(raycastInfo4.worldPoint.x, hitPointTop.x, epsilon));
            test(approxEqual(raycastInfo4.worldPoint.y, hitPointTop.y, epsilon));
            test(approxEqual(raycastInfo4.worldPoint.z, hitPointTop.z, epsilon));

            // ProxyCollisionShape::raycast()
            RaycastInfo raycastInfo5;
            test(mCapsuleShape->raycast(rayBottom, raycastInfo5));
            test(raycastInfo5.body == mCapsuleBody);
            test(raycastInfo5.proxyShape == mCapsuleShape);
            test(approxEqual(raycastInfo5.hitFraction, decimal(0.2), epsilon));
            test(approxEqual(raycastInfo5.worldPoint.x, hitPointBottom.x, epsilon));
            test(approxEqual(raycastInfo5.worldPoint.y, hitPointBottom.y, epsilon));
            test(approxEqual(raycastInfo5.worldPoint.z, hitPointBottom.z, epsilon));

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
            test(!mCapsuleBody->raycast(ray1, raycastInfo3));
            test(!mCapsuleShape->raycast(ray1, raycastInfo3));
            mCallback.reset();
            mWorld->raycast(ray1, &mCallback);
            test(!mCallback.isHit);
            mCallback.reset();
            mWorld->raycast(Ray(ray1.point1, ray1.point2, decimal(0.01)), &mCallback);
            test(!mCallback.isHit);
            mCallback.reset();
            mWorld->raycast(Ray(ray1.point1, ray1.point2, decimal(100.0)), &mCallback);
            test(!mCallback.isHit);

            test(!mCapsuleBody->raycast(ray2, raycastInfo3));
            test(!mCapsuleShape->raycast(ray2, raycastInfo3));
            mCallback.reset();
            mWorld->raycast(ray2, &mCallback);
            test(!mCallback.isHit);

            test(!mCapsuleBody->raycast(ray3, raycastInfo3));
            test(!mCapsuleShape->raycast(ray3, raycastInfo3));
            mCallback.reset();
            mWorld->raycast(ray3, &mCallback);
            test(!mCallback.isHit);

            test(!mCapsuleBody->raycast(ray4, raycastInfo3));
            test(!mCapsuleShape->raycast(ray4, raycastInfo3));
            mCallback.reset();
            mWorld->raycast(ray4, &mCallback);
            test(!mCallback.isHit);

            test(!mCapsuleBody->raycast(ray5, raycastInfo3));
            test(!mCapsuleShape->raycast(ray5, raycastInfo3));
            mCallback.reset();
            mWorld->raycast(ray5, &mCallback);
            test(!mCallback.isHit);

            test(!mCapsuleBody->raycast(ray6, raycastInfo3));
            test(!mCapsuleShape->raycast(ray6, raycastInfo3));
            mCallback.reset();
            mWorld->raycast(ray6, &mCallback);
            test(!mCallback.isHit);

            test(!mCapsuleBody->raycast(ray7, raycastInfo3));
            test(!mCapsuleShape->raycast(ray7, raycastInfo3));
            mWorld->raycast(ray7, &mCallback);
            test(!mCallback.isHit);

            test(!mCapsuleBody->raycast(ray8, raycastInfo3));
            test(!mCapsuleShape->raycast(ray8, raycastInfo3));
            mCallback.reset();
            mWorld->raycast(ray8, &mCallback);
            test(!mCallback.isHit);

            test(!mCapsuleBody->raycast(ray9, raycastInfo3));
            test(!mCapsuleShape->raycast(ray9, raycastInfo3));
            mCallback.reset();
            mWorld->raycast(ray9, &mCallback);
            test(!mCallback.isHit);

            test(!mCapsuleBody->raycast(ray10, raycastInfo3));
            test(!mCapsuleShape->raycast(ray10, raycastInfo3));
            mCallback.reset();
            mWorld->raycast(ray10, &mCallback);
            test(!mCallback.isHit);

            mCallback.reset();
            mWorld->raycast(Ray(ray11.point1, ray11.point2, decimal(0.01)), &mCallback);
            test(!mCallback.isHit);
            mCallback.reset();
            mWorld->raycast(Ray(ray12.point1, ray12.point2, decimal(0.01)), &mCallback);
            test(!mCallback.isHit);
            mCallback.reset();
            mWorld->raycast(Ray(ray13.point1, ray13.point2, decimal(0.01)), &mCallback);
            test(!mCallback.isHit);
            mCallback.reset();
            mWorld->raycast(Ray(ray14.point1, ray14.point2, decimal(0.01)), &mCallback);
            test(!mCallback.isHit);
            mCallback.reset();
            mWorld->raycast(Ray(ray15.point1, ray15.point2, decimal(0.01)), &mCallback);
            test(!mCallback.isHit);
            mCallback.reset();
            mWorld->raycast(Ray(ray16.point1, ray16.point2, decimal(0.01)), &mCallback);
            test(!mCallback.isHit);

            // ----- Test raycast hits ----- //
            test(mCapsuleBody->raycast(ray11, raycastInfo3));
            test(mCapsuleShape->raycast(ray11, raycastInfo3));
            mCallback.reset();
            mWorld->raycast(ray11, &mCallback);
            test(mCallback.isHit);
            mCallback.reset();
            mWorld->raycast(Ray(ray11.point1, ray11.point2, decimal(0.8)), &mCallback);
            test(mCallback.isHit);

            test(mCapsuleBody->raycast(ray12, raycastInfo3));
            test(mCapsuleShape->raycast(ray12, raycastInfo3));
            mCallback.reset();
            mWorld->raycast(ray12, &mCallback);
            test(mCallback.isHit);
            mCallback.reset();
            mWorld->raycast(Ray(ray12.point1, ray12.point2, decimal(0.8)), &mCallback);
            test(mCallback.isHit);

            test(mCapsuleBody->raycast(ray13, raycastInfo3));
            test(mCapsuleShape->raycast(ray13, raycastInfo3));
            mCallback.reset();
            mWorld->raycast(ray13, &mCallback);
            test(mCallback.isHit);
            mCallback.reset();
            mWorld->raycast(Ray(ray13.point1, ray13.point2, decimal(0.8)), &mCallback);
            test(mCallback.isHit);

            test(mCapsuleBody->raycast(ray14, raycastInfo3));
            test(mCapsuleShape->raycast(ray14, raycastInfo3));
            mCallback.reset();
            mWorld->raycast(ray14, &mCallback);
            test(mCallback.isHit);
            mCallback.reset();
            mWorld->raycast(Ray(ray14.point1, ray14.point2, decimal(0.8)), &mCallback);
            test(mCallback.isHit);

            test(mCapsuleBody->raycast(ray15, raycastInfo3));
            test(mCapsuleShape->raycast(ray15, raycastInfo3));
            mCallback.reset();
            mWorld->raycast(ray15, &mCallback);
            test(mCallback.isHit);
            mCallback.reset();
            mWorld->raycast(Ray(ray15.point1, ray15.point2, decimal(0.8)), &mCallback);
            test(mCallback.isHit);

            test(mCapsuleBody->raycast(ray16, raycastInfo3));
            test(mCapsuleShape->raycast(ray16, raycastInfo3));
            mCallback.reset();
            mWorld->raycast(ray16, &mCallback);
            test(mCallback.isHit);
            mCallback.reset();
            mWorld->raycast(Ray(ray16.point1, ray16.point2, decimal(0.8)), &mCallback);
            test(mCallback.isHit);
        }

        /// Test the ProxyConeShape::raycast(), CollisionBody::raycast() and
        /// CollisionWorld::raycast() methods.
        void testCone() {

            // ----- Test feedback data ----- //
            Vector3 point1A = mLocalShapeToWorld * Vector3(0 , 0, 3);
            Vector3 point1B = mLocalShapeToWorld * Vector3(0, 0, -7);
            Ray ray(point1A, point1B);
            Vector3 hitPoint = mLocalShapeToWorld * Vector3(0, 0, 1);

            Vector3 point2A = mLocalShapeToWorld * Vector3(1 , -5, 0);
            Vector3 point2B = mLocalShapeToWorld * Vector3(1, 5, 0);
            Ray rayBottom(point2A, point2B);
            Vector3 hitPoint2 = mLocalShapeToWorld * Vector3(1, -3, 0);

            mCallback.shapeToTest = mConeShape;

            // CollisionWorld::raycast()
            mCallback.reset();
            mWorld->raycast(ray, &mCallback);
            test(mCallback.isHit);
            test(mCallback.raycastInfo.body == mConeBody);
            test(mCallback.raycastInfo.proxyShape == mConeShape);
            test(approxEqual(mCallback.raycastInfo.hitFraction, decimal(0.2), epsilon));
            test(approxEqual(mCallback.raycastInfo.worldPoint.x, hitPoint.x, epsilon));
            test(approxEqual(mCallback.raycastInfo.worldPoint.y, hitPoint.y, epsilon));
            test(approxEqual(mCallback.raycastInfo.worldPoint.z, hitPoint.z, epsilon));

            // Correct category filter mask
            mCallback.reset();
            mWorld->raycast(ray, &mCallback, CATEGORY2);
            test(mCallback.isHit);

            // Wrong category filter mask
            mCallback.reset();
            mWorld->raycast(ray, &mCallback, CATEGORY1);
            test(!mCallback.isHit);

            // CollisionBody::raycast()
            RaycastInfo raycastInfo2;
            test(mConeBody->raycast(ray, raycastInfo2));
            test(raycastInfo2.body == mConeBody);
            test(raycastInfo2.proxyShape == mConeShape);
            test(approxEqual(raycastInfo2.hitFraction, decimal(0.2), epsilon));
            test(approxEqual(raycastInfo2.worldPoint.x, hitPoint.x, epsilon));
            test(approxEqual(raycastInfo2.worldPoint.y, hitPoint.y, epsilon));
            test(approxEqual(raycastInfo2.worldPoint.z, hitPoint.z, epsilon));

            // ProxyCollisionShape::raycast()
            RaycastInfo raycastInfo3;
            test(mConeShape->raycast(ray, raycastInfo3));
            test(raycastInfo3.body == mConeBody);
            test(raycastInfo3.proxyShape == mConeShape);
            test(approxEqual(raycastInfo3.hitFraction, decimal(0.2), epsilon));
            test(approxEqual(raycastInfo3.worldPoint.x, hitPoint.x, epsilon));
            test(approxEqual(raycastInfo3.worldPoint.y, hitPoint.y, epsilon));
            test(approxEqual(raycastInfo3.worldPoint.z, hitPoint.z, epsilon));

            mCallback.reset();
            mWorld->raycast(rayBottom, &mCallback);
            test(mCallback.isHit);
            test(mCallback.raycastInfo.body == mConeBody);
            test(mCallback.raycastInfo.proxyShape == mConeShape);
            test(approxEqual(mCallback.raycastInfo.hitFraction, decimal(0.2), epsilon));
            test(approxEqual(mCallback.raycastInfo.worldPoint.x, hitPoint2.x, epsilon));
            test(approxEqual(mCallback.raycastInfo.worldPoint.y, hitPoint2.y, epsilon));
            test(approxEqual(mCallback.raycastInfo.worldPoint.z, hitPoint2.z, epsilon));

            // CollisionBody::raycast()
            RaycastInfo raycastInfo5;
            test(mConeBody->raycast(rayBottom, raycastInfo5));
            test(raycastInfo5.body == mConeBody);
            test(raycastInfo5.proxyShape == mConeShape);
            test(approxEqual(raycastInfo5.hitFraction, decimal(0.2), epsilon));
            test(approxEqual(raycastInfo5.worldPoint.x, hitPoint2.x, epsilon));
            test(approxEqual(raycastInfo5.worldPoint.y, hitPoint2.y, epsilon));
            test(approxEqual(raycastInfo5.worldPoint.z, hitPoint2.z, epsilon));

            // ProxyCollisionShape::raycast()
            RaycastInfo raycastInfo6;
            test(mConeShape->raycast(rayBottom, raycastInfo6));
            test(raycastInfo6.body == mConeBody);
            test(raycastInfo6.proxyShape == mConeShape);
            test(approxEqual(raycastInfo6.hitFraction, decimal(0.2), epsilon));
            test(approxEqual(raycastInfo6.worldPoint.x, hitPoint2.x, epsilon));
            test(approxEqual(raycastInfo6.worldPoint.y, hitPoint2.y, epsilon));
            test(approxEqual(raycastInfo6.worldPoint.z, hitPoint2.z, epsilon));

            Ray ray1(mLocalShapeToWorld * Vector3(0, 0, 0), mLocalShapeToWorld * Vector3(5, 7, -1));
            Ray ray2(mLocalShapeToWorld * Vector3(5, 11, 7), mLocalShapeToWorld * Vector3(17, 29, 28));
            Ray ray3(mLocalShapeToWorld * Vector3(-1, -2, 1), mLocalShapeToWorld * Vector3(-13, -2, 22));
            Ray ray4(mLocalShapeToWorld * Vector3(10, 10, 10), mLocalShapeToWorld * Vector3(22, 28, 31));
            Ray ray5(mLocalShapeToWorld * Vector3(4, 1, -1), mLocalShapeToWorld * Vector3(-26, 1, -1));
            Ray ray6(mLocalShapeToWorld * Vector3(3, 4, 1), mLocalShapeToWorld * Vector3(3, -16, 1));
            Ray ray7(mLocalShapeToWorld * Vector3(1, -4, 3), mLocalShapeToWorld * Vector3(1, -4, -17));
            Ray ray8(mLocalShapeToWorld * Vector3(-4, 4, 0), mLocalShapeToWorld * Vector3(26, 4, 0));
            Ray ray9(mLocalShapeToWorld * Vector3(0, -4, -7), mLocalShapeToWorld * Vector3(0, 46, -7));
            Ray ray10(mLocalShapeToWorld * Vector3(-3, -2, -6), mLocalShapeToWorld * Vector3(-3, -2, 74));
            Ray ray11(mLocalShapeToWorld * Vector3(3, -1, 0.5), mLocalShapeToWorld * Vector3(-27, -1, 0.5));
            Ray ray12(mLocalShapeToWorld * Vector3(1, 4, -1), mLocalShapeToWorld * Vector3(1, -26, -1));
            Ray ray13(mLocalShapeToWorld * Vector3(-1, -2, 3), mLocalShapeToWorld * Vector3(-1, -2, -27));
            Ray ray14(mLocalShapeToWorld * Vector3(-2, 0, 0.8), mLocalShapeToWorld * Vector3(30, 0, 0.8));
            Ray ray15(mLocalShapeToWorld * Vector3(0, -4, 1), mLocalShapeToWorld * Vector3(0, 30, 1));
            Ray ray16(mLocalShapeToWorld * Vector3(-0.9, 0, -4), mLocalShapeToWorld * Vector3(-0.9, 0, 30));

            // ----- Test raycast miss ----- //
            test(!mConeBody->raycast(ray1, raycastInfo3));
            test(!mConeShape->raycast(ray1, raycastInfo3));
            mCallback.reset();
            mWorld->raycast(ray1, &mCallback);
            test(!mCallback.isHit);
            mCallback.reset();
            mWorld->raycast(Ray(ray1.point1, ray1.point2, decimal(0.01)), &mCallback);
            test(!mCallback.isHit);
            mCallback.reset();
            mWorld->raycast(Ray(ray1.point1, ray1.point2, decimal(100.0)), &mCallback);
            test(!mCallback.isHit);

            test(!mConeBody->raycast(ray2, raycastInfo3));
            test(!mConeShape->raycast(ray2, raycastInfo3));
            mCallback.reset();
            mWorld->raycast(ray2, &mCallback);
            test(!mCallback.isHit);

            test(!mConeBody->raycast(ray3, raycastInfo3));
            test(!mConeShape->raycast(ray3, raycastInfo3));
            mCallback.reset();
            mWorld->raycast(ray3, &mCallback);
            test(!mCallback.isHit);

            test(!mConeBody->raycast(ray4, raycastInfo3));
            test(!mConeShape->raycast(ray4, raycastInfo3));
            mCallback.reset();
            mWorld->raycast(ray4, &mCallback);
            test(!mCallback.isHit);

            test(!mConeBody->raycast(ray5, raycastInfo3));
            test(!mConeShape->raycast(ray5, raycastInfo3));
            mCallback.reset();
            mWorld->raycast(ray5, &mCallback);
            test(!mCallback.isHit);

            test(!mConeBody->raycast(ray6, raycastInfo3));
            test(!mConeShape->raycast(ray6, raycastInfo3));
            mCallback.reset();
            mWorld->raycast(ray6, &mCallback);
            test(!mCallback.isHit);

            test(!mConeBody->raycast(ray7, raycastInfo3));
            test(!mConeShape->raycast(ray7, raycastInfo3));
            mCallback.reset();
            mWorld->raycast(ray7, &mCallback);
            test(!mCallback.isHit);

            test(!mConeBody->raycast(ray8, raycastInfo3));
            test(!mConeShape->raycast(ray8, raycastInfo3));
            mCallback.reset();
            mWorld->raycast(ray8, &mCallback);
            test(!mCallback.isHit);

            test(!mConeBody->raycast(ray9, raycastInfo3));
            test(!mConeShape->raycast(ray9, raycastInfo3));
            mCallback.reset();
            mWorld->raycast(ray9, &mCallback);
            test(!mCallback.isHit);

            test(!mConeBody->raycast(ray10, raycastInfo3));
            test(!mConeShape->raycast(ray10, raycastInfo3));
            mCallback.reset();
            mWorld->raycast(ray10, &mCallback);
            test(!mCallback.isHit);

            mCallback.reset();
            mWorld->raycast(Ray(ray11.point1, ray11.point2, decimal(0.01)), &mCallback);
            test(!mCallback.isHit);
            mCallback.reset();
            mWorld->raycast(Ray(ray12.point1, ray12.point2, decimal(0.01)), &mCallback);
            test(!mCallback.isHit);
            mCallback.reset();
            mWorld->raycast(Ray(ray13.point1, ray13.point2, decimal(0.01)), &mCallback);
            test(!mCallback.isHit);
            mCallback.reset();
            mWorld->raycast(Ray(ray14.point1, ray14.point2, decimal(0.01)), &mCallback);
            test(!mCallback.isHit);
            mCallback.reset();
            mWorld->raycast(Ray(ray15.point1, ray15.point2, decimal(0.01)), &mCallback);
            test(!mCallback.isHit);
            mCallback.reset();
            mWorld->raycast(Ray(ray16.point1, ray16.point2, decimal(0.01)), &mCallback);
            test(!mCallback.isHit);

            // ----- Test raycast hits ----- //
            test(mConeBody->raycast(ray11, raycastInfo3));
            test(mConeShape->raycast(ray11, raycastInfo3));
            mCallback.reset();
            mWorld->raycast(ray11, &mCallback);
            test(mCallback.isHit);
            mCallback.reset();
            mWorld->raycast(Ray(ray11.point1, ray11.point2, decimal(0.8)), &mCallback);
            test(mCallback.isHit);

            test(mConeBody->raycast(ray12, raycastInfo3));
            test(mConeShape->raycast(ray12, raycastInfo3));
            mCallback.reset();
            mWorld->raycast(ray12, &mCallback);
            test(mCallback.isHit);
            mCallback.reset();
            mWorld->raycast(Ray(ray12.point1, ray12.point2, decimal(0.8)), &mCallback);
            test(mCallback.isHit);

            test(mConeBody->raycast(ray13, raycastInfo3));
            test(mConeShape->raycast(ray13, raycastInfo3));
            mCallback.reset();
            mWorld->raycast(ray13, &mCallback);
            test(mCallback.isHit);
            mCallback.reset();
            mWorld->raycast(Ray(ray13.point1, ray13.point2, decimal(0.8)), &mCallback);
            test(mCallback.isHit);

            test(mConeBody->raycast(ray14, raycastInfo3));
            test(mConeShape->raycast(ray14, raycastInfo3));
            mCallback.reset();
            mWorld->raycast(ray14, &mCallback);
            test(mCallback.isHit);
            mCallback.reset();
            mWorld->raycast(Ray(ray14.point1, ray14.point2, decimal(0.8)), &mCallback);
            test(mCallback.isHit);

            test(mConeBody->raycast(ray15, raycastInfo3));
            test(mConeShape->raycast(ray15, raycastInfo3));
            mCallback.reset();
            mWorld->raycast(ray15, &mCallback);
            test(mCallback.isHit);
            mCallback.reset();
            mWorld->raycast(Ray(ray15.point1, ray15.point2, decimal(0.8)), &mCallback);
            test(mCallback.isHit);

            test(mConeBody->raycast(ray16, raycastInfo3));
            test(mConeShape->raycast(ray16, raycastInfo3));
            mCallback.reset();
            mWorld->raycast(ray16, &mCallback);
            test(mCallback.isHit);
            mCallback.reset();
            mWorld->raycast(Ray(ray16.point1, ray16.point2, decimal(0.8)), &mCallback);
            test(mCallback.isHit);
        }

        /// Test the ProxyConvexMeshShape::raycast(), CollisionBody::raycast() and
        /// CollisionWorld::raycast() methods.
        void testConvexMesh() {

            // ----- Test feedback data ----- //
            Vector3 point1 = mLocalShapeToWorld * Vector3(1 , 2, 6);
            Vector3 point2 = mLocalShapeToWorld * Vector3(1, 2, -4);
            Ray ray(point1, point2);
            Vector3 hitPoint = mLocalShapeToWorld * Vector3(1, 2, 4);

            mCallback.shapeToTest = mConvexMeshShape;

            // CollisionWorld::raycast()
            mCallback.reset();
            mWorld->raycast(ray, &mCallback);
            test(mCallback.isHit);
            test(mCallback.raycastInfo.body == mConvexMeshBody);
            test(mCallback.raycastInfo.proxyShape == mConvexMeshShape);
            test(approxEqual(mCallback.raycastInfo.hitFraction, decimal(0.2), epsilon));
            test(approxEqual(mCallback.raycastInfo.worldPoint.x, hitPoint.x, epsilon));
            test(approxEqual(mCallback.raycastInfo.worldPoint.y, hitPoint.y, epsilon));
            test(approxEqual(mCallback.raycastInfo.worldPoint.z, hitPoint.z, epsilon));

            // Correct category filter mask
            mCallback.reset();
            mWorld->raycast(ray, &mCallback, CATEGORY2);
            test(mCallback.isHit);

            // Wrong category filter mask
            mCallback.reset();
            mWorld->raycast(ray, &mCallback, CATEGORY1);
            test(!mCallback.isHit);

            // CollisionBody::raycast()
            RaycastInfo raycastInfo2;
            test(mConvexMeshBody->raycast(ray, raycastInfo2));
            test(raycastInfo2.body == mConvexMeshBody);
            test(raycastInfo2.proxyShape == mConvexMeshShape);
            test(approxEqual(raycastInfo2.hitFraction, decimal(0.2), epsilon));
            test(approxEqual(raycastInfo2.worldPoint.x, hitPoint.x, epsilon));
            test(approxEqual(raycastInfo2.worldPoint.y, hitPoint.y, epsilon));
            test(approxEqual(raycastInfo2.worldPoint.z, hitPoint.z, epsilon));

            // ProxyCollisionShape::raycast()
            RaycastInfo raycastInfo3;
            test(mConvexMeshBodyEdgesInfo->raycast(ray, raycastInfo3));
            test(raycastInfo3.body == mConvexMeshBodyEdgesInfo);
            test(raycastInfo3.proxyShape == mConvexMeshShapeEdgesInfo);
            test(approxEqual(raycastInfo3.hitFraction, decimal(0.2), epsilon));
            test(approxEqual(raycastInfo3.worldPoint.x, hitPoint.x, epsilon));
            test(approxEqual(raycastInfo3.worldPoint.y, hitPoint.y, epsilon));
            test(approxEqual(raycastInfo3.worldPoint.z, hitPoint.z, epsilon));

            // ProxyCollisionShape::raycast()
            RaycastInfo raycastInfo4;
            test(mConvexMeshShape->raycast(ray, raycastInfo4));
            test(raycastInfo4.body == mConvexMeshBody);
            test(raycastInfo4.proxyShape == mConvexMeshShape);
            test(approxEqual(raycastInfo4.hitFraction, decimal(0.2), epsilon));
            test(approxEqual(raycastInfo4.worldPoint.x, hitPoint.x, epsilon));
            test(approxEqual(raycastInfo4.worldPoint.y, hitPoint.y, epsilon));
            test(approxEqual(raycastInfo4.worldPoint.z, hitPoint.z, epsilon));

            // ProxyCollisionShape::raycast()
            RaycastInfo raycastInfo5;
            test(mConvexMeshShapeEdgesInfo->raycast(ray, raycastInfo5));
            test(raycastInfo5.body == mConvexMeshBodyEdgesInfo);
            test(raycastInfo5.proxyShape == mConvexMeshShapeEdgesInfo);
            test(approxEqual(raycastInfo5.hitFraction, decimal(0.2), epsilon));
            test(approxEqual(raycastInfo5.worldPoint.x, hitPoint.x, epsilon));
            test(approxEqual(raycastInfo5.worldPoint.y, hitPoint.y, epsilon));
            test(approxEqual(raycastInfo5.worldPoint.z, hitPoint.z, epsilon));

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
            test(!mConvexMeshBody->raycast(ray1, raycastInfo3));
            test(!mConvexMeshBodyEdgesInfo->raycast(ray1, raycastInfo3));
            test(!mConvexMeshShape->raycast(ray1, raycastInfo3));
            test(!mConvexMeshShapeEdgesInfo->raycast(ray1, raycastInfo3));
            mCallback.reset();
            mWorld->raycast(ray1, &mCallback);
            test(!mCallback.isHit);
            mCallback.reset();
            mWorld->raycast(Ray(ray1.point1, ray1.point2, decimal(0.01)), &mCallback);
            test(!mCallback.isHit);
            mCallback.reset();
            mWorld->raycast(Ray(ray1.point1, ray1.point2, decimal(100.0)), &mCallback);
            test(!mCallback.isHit);

            test(!mConvexMeshBody->raycast(ray2, raycastInfo3));
            test(!mConvexMeshBodyEdgesInfo->raycast(ray2, raycastInfo3));
            test(!mConvexMeshShape->raycast(ray2, raycastInfo3));
            test(!mConvexMeshShapeEdgesInfo->raycast(ray2, raycastInfo3));
            mCallback.reset();
            mWorld->raycast(ray2, &mCallback);
            test(!mCallback.isHit);

            test(!mConvexMeshBody->raycast(ray3, raycastInfo3));
            test(!mConvexMeshBodyEdgesInfo->raycast(ray3, raycastInfo3));
            test(!mConvexMeshShape->raycast(ray3, raycastInfo3));
            test(!mConvexMeshShapeEdgesInfo->raycast(ray3, raycastInfo3));
            mCallback.reset();
            mWorld->raycast(ray3, &mCallback);
            test(!mCallback.isHit);

            test(!mConvexMeshBody->raycast(ray4, raycastInfo3));
            test(!mConvexMeshBodyEdgesInfo->raycast(ray4, raycastInfo3));
            test(!mConvexMeshShape->raycast(ray4, raycastInfo3));
            test(!mConvexMeshShapeEdgesInfo->raycast(ray4, raycastInfo3));
            mCallback.reset();
            mWorld->raycast(ray4, &mCallback);
            test(!mCallback.isHit);

            test(!mConvexMeshBody->raycast(ray5, raycastInfo3));
            test(!mConvexMeshBodyEdgesInfo->raycast(ray5, raycastInfo3));
            test(!mConvexMeshShape->raycast(ray5, raycastInfo3));
            test(!mConvexMeshShapeEdgesInfo->raycast(ray5, raycastInfo3));
            mCallback.reset();
            mWorld->raycast(ray5, &mCallback);
            test(!mCallback.isHit);

            test(!mConvexMeshBody->raycast(ray6, raycastInfo3));
            test(!mConvexMeshBodyEdgesInfo->raycast(ray6, raycastInfo3));
            test(!mConvexMeshShape->raycast(ray6, raycastInfo3));
            test(!mConvexMeshShapeEdgesInfo->raycast(ray6, raycastInfo3));
            mCallback.reset();
            mWorld->raycast(ray6, &mCallback);
            test(!mCallback.isHit);

            test(!mConvexMeshBody->raycast(ray7, raycastInfo3));
            test(!mConvexMeshBodyEdgesInfo->raycast(ray7, raycastInfo3));
            test(!mConvexMeshShape->raycast(ray7, raycastInfo3));
            test(!mConvexMeshShapeEdgesInfo->raycast(ray7, raycastInfo3));
            mCallback.reset();
            mWorld->raycast(ray7, &mCallback);
            test(!mCallback.isHit);

            test(!mConvexMeshBody->raycast(ray8, raycastInfo3));
            test(!mConvexMeshBodyEdgesInfo->raycast(ray8, raycastInfo3));
            test(!mConvexMeshShape->raycast(ray8, raycastInfo3));
            test(!mConvexMeshShapeEdgesInfo->raycast(ray8, raycastInfo3));
            mCallback.reset();
            mWorld->raycast(ray8, &mCallback);
            test(!mCallback.isHit);

            test(!mConvexMeshBody->raycast(ray9, raycastInfo3));
            test(!mConvexMeshBodyEdgesInfo->raycast(ray9, raycastInfo3));
            test(!mConvexMeshShape->raycast(ray9, raycastInfo3));
            test(!mConvexMeshShapeEdgesInfo->raycast(ray9, raycastInfo3));
            mCallback.reset();
            mWorld->raycast(ray9, &mCallback);
            test(!mCallback.isHit);

            test(!mConvexMeshBody->raycast(ray10, raycastInfo3));
            test(!mConvexMeshBodyEdgesInfo->raycast(ray10, raycastInfo3));
            test(!mConvexMeshShape->raycast(ray10, raycastInfo3));
            test(!mConvexMeshShapeEdgesInfo->raycast(ray10, raycastInfo3));
            mCallback.reset();
            mWorld->raycast(ray10, &mCallback);
            test(!mCallback.isHit);

            mCallback.reset();
            mWorld->raycast(Ray(ray11.point1, ray11.point2, decimal(0.01)), &mCallback);
            test(!mCallback.isHit);
            mCallback.reset();
            mWorld->raycast(Ray(ray12.point1, ray12.point2, decimal(0.01)), &mCallback);
            test(!mCallback.isHit);
            mCallback.reset();
            mWorld->raycast(Ray(ray13.point1, ray13.point2, decimal(0.01)), &mCallback);
            test(!mCallback.isHit);
            mCallback.reset();
            mWorld->raycast(Ray(ray14.point1, ray14.point2, decimal(0.01)), &mCallback);
            test(!mCallback.isHit);
            mCallback.reset();
            mWorld->raycast(Ray(ray15.point1, ray15.point2, decimal(0.01)), &mCallback);
            test(!mCallback.isHit);
            mCallback.reset();
            mWorld->raycast(Ray(ray16.point1, ray16.point2, decimal(0.01)), &mCallback);
            test(!mCallback.isHit);

            // ----- Test raycast hits ----- //
            test(mConvexMeshBody->raycast(ray11, raycastInfo3));
            test(mConvexMeshBodyEdgesInfo->raycast(ray11, raycastInfo3));
            test(mConvexMeshShape->raycast(ray11, raycastInfo3));
            test(mConvexMeshShapeEdgesInfo->raycast(ray11, raycastInfo3));
            mCallback.reset();
            mWorld->raycast(ray11, &mCallback);
            test(mCallback.isHit);
            mCallback.reset();
            mWorld->raycast(Ray(ray11.point1, ray11.point2, decimal(0.8)), &mCallback);
            test(mCallback.isHit);

            test(mConvexMeshBody->raycast(ray12, raycastInfo3));
            test(mConvexMeshBodyEdgesInfo->raycast(ray12, raycastInfo3));
            test(mConvexMeshShape->raycast(ray12, raycastInfo3));
            test(mConvexMeshShapeEdgesInfo->raycast(ray12, raycastInfo3));
            mCallback.reset();
            mWorld->raycast(ray12, &mCallback);
            test(mCallback.isHit);
            mCallback.reset();
            mWorld->raycast(Ray(ray12.point1, ray12.point2, decimal(0.8)), &mCallback);
            test(mCallback.isHit);

            test(mConvexMeshBody->raycast(ray13, raycastInfo3));
            test(mConvexMeshBodyEdgesInfo->raycast(ray13, raycastInfo3));
            test(mConvexMeshShape->raycast(ray13, raycastInfo3));
            test(mConvexMeshShapeEdgesInfo->raycast(ray13, raycastInfo3));
            mCallback.reset();
            mWorld->raycast(ray13, &mCallback);
            test(mCallback.isHit);
            mCallback.reset();
            mWorld->raycast(Ray(ray13.point1, ray13.point2, decimal(0.8)), &mCallback);
            test(mCallback.isHit);

            test(mConvexMeshBody->raycast(ray14, raycastInfo3));
            test(mConvexMeshBodyEdgesInfo->raycast(ray14, raycastInfo3));
            test(mConvexMeshShape->raycast(ray14, raycastInfo3));
            test(mConvexMeshShapeEdgesInfo->raycast(ray14, raycastInfo3));
            mCallback.reset();
            mWorld->raycast(ray14, &mCallback);
            test(mCallback.isHit);
            mCallback.reset();
            mWorld->raycast(Ray(ray14.point1, ray14.point2, decimal(0.8)), &mCallback);
            test(mCallback.isHit);

            test(mConvexMeshBody->raycast(ray15, raycastInfo3));
            test(mConvexMeshBodyEdgesInfo->raycast(ray15, raycastInfo3));
            test(mConvexMeshShape->raycast(ray15, raycastInfo3));
            test(mConvexMeshShapeEdgesInfo->raycast(ray15, raycastInfo3));
            mCallback.reset();
            mWorld->raycast(ray15, &mCallback);
            test(mCallback.isHit);
            mCallback.reset();
            mWorld->raycast(Ray(ray15.point1, ray15.point2, decimal(0.8)), &mCallback);
            test(mCallback.isHit);

            test(mConvexMeshBody->raycast(ray16, raycastInfo3));
            test(mConvexMeshBodyEdgesInfo->raycast(ray16, raycastInfo3));
            test(mConvexMeshShape->raycast(ray16, raycastInfo3));
            test(mConvexMeshShapeEdgesInfo->raycast(ray16, raycastInfo3));
            mCallback.reset();
            mWorld->raycast(ray16, &mCallback);
            test(mCallback.isHit);
            mCallback.reset();
            mWorld->raycast(Ray(ray16.point1, ray16.point2, decimal(0.8)), &mCallback);
            test(mCallback.isHit);
        }

        /// Test the ProxyCylinderShape::raycast(), CollisionBody::raycast() and
        /// CollisionWorld::raycast() methods.
        void testCylinder() {

            // ----- Test feedback data ----- //
            Vector3 point1A = mLocalShapeToWorld * Vector3(4 , 1, 0);
            Vector3 point1B = mLocalShapeToWorld * Vector3(-6, 1, 0);
            Ray ray(point1A, point1B);
            Vector3 hitPoint = mLocalShapeToWorld * Vector3(2, 1, 0);

            Vector3 point2A = mLocalShapeToWorld * Vector3(0 , 4.5, 0);
            Vector3 point2B = mLocalShapeToWorld * Vector3(0, -5.5, 0);
            Ray rayTop(point2A, point2B);
            Vector3 hitPointTop = mLocalShapeToWorld * Vector3(0, decimal(2.5), 0);

            Vector3 point3A = mLocalShapeToWorld * Vector3(0 , -4.5, 0);
            Vector3 point3B = mLocalShapeToWorld * Vector3(0, 5.5, 0);
            Ray rayBottom(point3A, point3B);
            Vector3 hitPointBottom = mLocalShapeToWorld * Vector3(0, decimal(-2.5), 0);

            mCallback.shapeToTest = mCylinderShape;

            // CollisionWorld::raycast()
            mCallback.reset();
            mWorld->raycast(ray, &mCallback);
            test(mCallback.isHit);
            test(mCallback.raycastInfo.body == mCylinderBody);
            test(mCallback.raycastInfo.proxyShape == mCylinderShape);
            test(approxEqual(mCallback.raycastInfo.hitFraction, decimal(0.2), epsilon));
            test(approxEqual(mCallback.raycastInfo.worldPoint.x, hitPoint.x, epsilon));
            test(approxEqual(mCallback.raycastInfo.worldPoint.y, hitPoint.y, epsilon));
            test(approxEqual(mCallback.raycastInfo.worldPoint.z, hitPoint.z, epsilon));

            // Correct category filter mask
            mCallback.reset();
            mWorld->raycast(ray, &mCallback, CATEGORY2);
            test(mCallback.isHit);

            // Wrong category filter mask
            mCallback.reset();
            mWorld->raycast(ray, &mCallback, CATEGORY1);
            test(!mCallback.isHit);

            // CollisionBody::raycast()
            RaycastInfo raycastInfo2;
            test(mCylinderBody->raycast(ray, raycastInfo2));
            test(raycastInfo2.body == mCylinderBody);
            test(raycastInfo2.proxyShape == mCylinderShape);
            test(approxEqual(raycastInfo2.hitFraction, decimal(0.2), epsilon));
            test(approxEqual(raycastInfo2.worldPoint.x, hitPoint.x, epsilon));
            test(approxEqual(raycastInfo2.worldPoint.y, hitPoint.y, epsilon));
            test(approxEqual(raycastInfo2.worldPoint.z, hitPoint.z, epsilon));

            // ProxyCollisionShape::raycast()
            RaycastInfo raycastInfo3;
            test(mCylinderShape->raycast(ray, raycastInfo3));
            test(raycastInfo3.body == mCylinderBody);
            test(raycastInfo3.proxyShape == mCylinderShape);
            test(approxEqual(raycastInfo3.hitFraction, decimal(0.2), epsilon));
            test(approxEqual(raycastInfo3.worldPoint.x, hitPoint.x, epsilon));
            test(approxEqual(raycastInfo3.worldPoint.y, hitPoint.y, epsilon));
            test(approxEqual(raycastInfo3.worldPoint.z, hitPoint.z, epsilon));

            // ProxyCollisionShape::raycast()
            RaycastInfo raycastInfo5;
            test(mCylinderShape->raycast(rayTop, raycastInfo5));
            test(raycastInfo5.body == mCylinderBody);
            test(raycastInfo5.proxyShape == mCylinderShape);
            test(approxEqual(raycastInfo5.hitFraction, decimal(0.2), epsilon));
            test(approxEqual(raycastInfo5.worldPoint.x, hitPointTop.x, epsilon));
            test(approxEqual(raycastInfo5.worldPoint.y, hitPointTop.y, epsilon));
            test(approxEqual(raycastInfo5.worldPoint.z, hitPointTop.z, epsilon));

            // ProxyCollisionShape::raycast()
            RaycastInfo raycastInfo6;
            test(mCylinderShape->raycast(rayBottom, raycastInfo6));
            test(raycastInfo6.body == mCylinderBody);
            test(raycastInfo6.proxyShape == mCylinderShape);
            test(approxEqual(raycastInfo6.hitFraction, decimal(0.2), epsilon));
            test(approxEqual(raycastInfo6.worldPoint.x, hitPointBottom.x, epsilon));
            test(approxEqual(raycastInfo6.worldPoint.y, hitPointBottom.y, epsilon));
            test(approxEqual(raycastInfo6.worldPoint.z, hitPointBottom.z, epsilon));

            Ray ray1(mLocalShapeToWorld * Vector3(0, 0, 0), mLocalShapeToWorld * Vector3(5, 7, -1));
            Ray ray2(mLocalShapeToWorld * Vector3(5, 11, 7), mLocalShapeToWorld * Vector3(17, 20, 28));
            Ray ray3(mLocalShapeToWorld * Vector3(1, 3, -1), mLocalShapeToWorld * Vector3(-11,3, 20));
            Ray ray4(mLocalShapeToWorld * Vector3(10, 10, 10), mLocalShapeToWorld * Vector3(22, 28, 31));
            Ray ray5(mLocalShapeToWorld * Vector3(4, 1, -5), mLocalShapeToWorld * Vector3(-30, 1, -5));
            Ray ray6(mLocalShapeToWorld * Vector3(4, 9, 1), mLocalShapeToWorld * Vector3(4, -30, 1));
            Ray ray7(mLocalShapeToWorld * Vector3(1, -9, 5), mLocalShapeToWorld * Vector3(1, -9, -30));
            Ray ray8(mLocalShapeToWorld * Vector3(-4, 9, 0), mLocalShapeToWorld * Vector3(30, 9, 0));
            Ray ray9(mLocalShapeToWorld * Vector3(0, -9, -4), mLocalShapeToWorld * Vector3(0, 30, -4));
            Ray ray10(mLocalShapeToWorld * Vector3(-4, 0, -6), mLocalShapeToWorld * Vector3(-4, 0, 30));
            Ray ray11(mLocalShapeToWorld * Vector3(4, 1, 1.5), mLocalShapeToWorld * Vector3(-30, 1, 1.5));
            Ray ray12(mLocalShapeToWorld * Vector3(1, 9, -1), mLocalShapeToWorld * Vector3(1, -30, -1));
            Ray ray13(mLocalShapeToWorld * Vector3(-1, 2, 3), mLocalShapeToWorld * Vector3(-1, 2, -30));
            Ray ray14(mLocalShapeToWorld * Vector3(-3, 2, -1.7), mLocalShapeToWorld * Vector3(30, 2, -1.7));
            Ray ray15(mLocalShapeToWorld * Vector3(0, -9, 1), mLocalShapeToWorld * Vector3(0, 30, 1));
            Ray ray16(mLocalShapeToWorld * Vector3(-1, 2, -7), mLocalShapeToWorld * Vector3(-1, 2, 30));

            // ----- Test raycast miss ----- //
            test(!mCylinderBody->raycast(ray1, raycastInfo3));
            test(!mCylinderShape->raycast(ray1, raycastInfo3));
            mCallback.reset();
            mWorld->raycast(ray1, &mCallback);
            test(!mCallback.isHit);
            mCallback.reset();
            mWorld->raycast(Ray(ray1.point1, ray1.point2, decimal(0.01)), &mCallback);
            test(!mCallback.isHit);
            mCallback.reset();
            mWorld->raycast(Ray(ray1.point1, ray1.point2, decimal(100.0)), &mCallback);
            test(!mCallback.isHit);

            test(!mCylinderBody->raycast(ray2, raycastInfo3));
            test(!mCylinderShape->raycast(ray2, raycastInfo3));
            mCallback.reset();
            mWorld->raycast(ray2, &mCallback);
            test(!mCallback.isHit);

            test(!mCylinderBody->raycast(ray3, raycastInfo3));
            test(!mCylinderShape->raycast(ray3, raycastInfo3));
            mCallback.reset();
            mWorld->raycast(ray3, &mCallback);
            test(!mCallback.isHit);

            test(!mCylinderBody->raycast(ray4, raycastInfo3));
            test(!mCylinderShape->raycast(ray4, raycastInfo3));
            mCallback.reset();
            mWorld->raycast(ray4, &mCallback);
            test(!mCallback.isHit);

            test(!mCylinderBody->raycast(ray5, raycastInfo3));
            test(!mCylinderShape->raycast(ray5, raycastInfo3));
            mCallback.reset();
            mWorld->raycast(ray5, &mCallback);
            test(!mCallback.isHit);

            test(!mCylinderBody->raycast(ray6, raycastInfo3));
            test(!mCylinderShape->raycast(ray6, raycastInfo3));
            mCallback.reset();
            mWorld->raycast(ray6, &mCallback);
            test(!mCallback.isHit);

            test(!mCylinderBody->raycast(ray7, raycastInfo3));
            test(!mCylinderShape->raycast(ray7, raycastInfo3));
            mCallback.reset();
            mWorld->raycast(ray7, &mCallback);
            test(!mCallback.isHit);

            test(!mCylinderBody->raycast(ray8, raycastInfo3));
            test(!mCylinderShape->raycast(ray8, raycastInfo3));
            mCallback.reset();
            mWorld->raycast(ray8, &mCallback);
            test(!mCallback.isHit);

            test(!mCylinderBody->raycast(ray9, raycastInfo3));
            test(!mCylinderShape->raycast(ray9, raycastInfo3));
            mCallback.reset();
            mWorld->raycast(ray9, &mCallback);
            test(!mCallback.isHit);

            test(!mCylinderBody->raycast(ray10, raycastInfo3));
            test(!mCylinderShape->raycast(ray10, raycastInfo3));
            mCallback.reset();
            mWorld->raycast(ray10, &mCallback);
            test(!mCallback.isHit);

            mCallback.reset();
            mWorld->raycast(Ray(ray11.point1, ray11.point2, decimal(0.01)), &mCallback);
            test(!mCallback.isHit);
            mCallback.reset();
            mWorld->raycast(Ray(ray12.point1, ray12.point2, decimal(0.01)), &mCallback);
            test(!mCallback.isHit);
            mCallback.reset();
            mWorld->raycast(Ray(ray13.point1, ray13.point2, decimal(0.01)), &mCallback);
            test(!mCallback.isHit);
            mCallback.reset();
            mWorld->raycast(Ray(ray14.point1, ray14.point2, decimal(0.01)), &mCallback);
            test(!mCallback.isHit);
            mCallback.reset();
            mWorld->raycast(Ray(ray15.point1, ray15.point2, decimal(0.01)), &mCallback);
            test(!mCallback.isHit);
            mCallback.reset();
            mWorld->raycast(Ray(ray16.point1, ray16.point2, decimal(0.01)), &mCallback);
            test(!mCallback.isHit);

            // ----- Test raycast hits ----- //
            test(mCylinderBody->raycast(ray11, raycastInfo3));
            test(mCylinderShape->raycast(ray11, raycastInfo3));
            mCallback.reset();
            mWorld->raycast(ray11, &mCallback);
            test(mCallback.isHit);
            mCallback.reset();
            mWorld->raycast(Ray(ray11.point1, ray11.point2, decimal(0.8)), &mCallback);
            test(mCallback.isHit);

            test(mCylinderBody->raycast(ray12, raycastInfo3));
            test(mCylinderShape->raycast(ray12, raycastInfo3));
            mCallback.reset();
            mWorld->raycast(ray12, &mCallback);
            test(mCallback.isHit);
            mCallback.reset();
            mWorld->raycast(Ray(ray12.point1, ray12.point2, decimal(0.8)), &mCallback);
            test(mCallback.isHit);

            test(mCylinderBody->raycast(ray13, raycastInfo3));
            test(mCylinderShape->raycast(ray13, raycastInfo3));
            mCallback.reset();
            mWorld->raycast(ray13, &mCallback);
            test(mCallback.isHit);
            mCallback.reset();
            mWorld->raycast(Ray(ray13.point1, ray13.point2, decimal(0.8)), &mCallback);
            test(mCallback.isHit);

            test(mCylinderBody->raycast(ray14, raycastInfo3));
            test(mCylinderShape->raycast(ray14, raycastInfo3));
            mCallback.reset();
            mWorld->raycast(ray14, &mCallback);
            test(mCallback.isHit);
            mCallback.reset();
            mWorld->raycast(Ray(ray14.point1, ray14.point2, decimal(0.8)), &mCallback);
            test(mCallback.isHit);

            test(mCylinderBody->raycast(ray15, raycastInfo3));
            test(mCylinderShape->raycast(ray15, raycastInfo3));
            mCallback.reset();
            mWorld->raycast(ray15, &mCallback);
            test(mCallback.isHit);
            mCallback.reset();
            mWorld->raycast(Ray(ray15.point1, ray15.point2, decimal(0.8)), &mCallback);
            test(mCallback.isHit);

            test(mCylinderBody->raycast(ray16, raycastInfo3));
            test(mCylinderShape->raycast(ray16, raycastInfo3));
            mCallback.reset();
            mWorld->raycast(ray16, &mCallback);
            test(mCallback.isHit);
            mCallback.reset();
            mWorld->raycast(Ray(ray16.point1, ray16.point2, decimal(0.8)), &mCallback);
            test(mCallback.isHit);
        }

        /// Test the CollisionBody::raycast() and
        /// CollisionWorld::raycast() methods.
        void testCompound() {

            // ----- Test feedback data ----- //

            // Raycast hit against the sphere shape
            Ray ray1(mLocalShape2ToWorld * Vector3(4, 1, 2), mLocalShape2ToWorld * Vector3(-30, 1, 2));
            Ray ray2(mLocalShape2ToWorld * Vector3(1, 4, -1), mLocalShape2ToWorld * Vector3(1, -30, -1));
            Ray ray3(mLocalShape2ToWorld * Vector3(-1, 2, 5), mLocalShape2ToWorld * Vector3(-1, 2, -30));
            Ray ray4(mLocalShape2ToWorld * Vector3(-5, 2, -2), mLocalShape2ToWorld * Vector3(30, 2, -2));
            Ray ray5(mLocalShape2ToWorld * Vector3(0, -4, 1), mLocalShape2ToWorld * Vector3(0, 30, 1));
            Ray ray6(mLocalShape2ToWorld * Vector3(-1, 2, -11), mLocalShape2ToWorld * Vector3(-1, 2, 30));

            mCallback.shapeToTest = mCompoundSphereShape;

            // Correct category filter mask
            mCallback.reset();
            mWorld->raycast(ray1, &mCallback, CATEGORY2);
            test(mCallback.isHit);

            // Wrong category filter mask
            mCallback.reset();
            mWorld->raycast(ray1, &mCallback, CATEGORY1);
            test(!mCallback.isHit);

            RaycastInfo raycastInfo;
            test(mCompoundBody->raycast(ray1, raycastInfo));
            mCallback.reset();
            mWorld->raycast(ray1, &mCallback);
            test(mCallback.isHit);
            mCallback.reset();
            mWorld->raycast(Ray(ray1.point1, ray1.point2, decimal(0.8)), &mCallback);
            test(mCallback.isHit);

            test(mCompoundBody->raycast(ray2, raycastInfo));
            mCallback.reset();
            mWorld->raycast(ray2, &mCallback);
            test(mCallback.isHit);
            mCallback.reset();
            mWorld->raycast(Ray(ray2.point1, ray2.point2, decimal(0.8)), &mCallback);
            test(mCallback.isHit);

            test(mCompoundBody->raycast(ray3, raycastInfo));
            mCallback.reset();
            mWorld->raycast(ray3, &mCallback);
            test(mCallback.isHit);
            mCallback.reset();
            mWorld->raycast(Ray(ray3.point1, ray3.point2, decimal(0.8)), &mCallback);
            test(mCallback.isHit);

            test(mCompoundBody->raycast(ray4, raycastInfo));
            mCallback.reset();
            mWorld->raycast(ray4, &mCallback);
            test(mCallback.isHit);
            mCallback.reset();
            mWorld->raycast(Ray(ray4.point1, ray4.point2, decimal(0.8)), &mCallback);
            test(mCallback.isHit);

            test(mCompoundBody->raycast(ray5, raycastInfo));
            mCallback.reset();
            mWorld->raycast(ray5, &mCallback);
            test(mCallback.isHit);
            mCallback.reset();
            mWorld->raycast(Ray(ray5.point1, ray5.point2, decimal(0.8)), &mCallback);
            test(mCallback.isHit);

            test(mCompoundBody->raycast(ray6, raycastInfo));
            mCallback.reset();
            mWorld->raycast(ray6, &mCallback);
            test(mCallback.isHit);
            mCallback.reset();
            mWorld->raycast(Ray(ray6.point1, ray6.point2, decimal(0.8)), &mCallback);
            test(mCallback.isHit);

            // Raycast hit agains the cylinder shape
            Ray ray11(mLocalShapeToWorld * Vector3(4, 1, 1.5), mLocalShapeToWorld * Vector3(-30, 1.5, 2));
            Ray ray12(mLocalShapeToWorld * Vector3(1.5, 9, -1), mLocalShapeToWorld * Vector3(1.5, -30, -1));
            Ray ray13(mLocalShapeToWorld * Vector3(-1, 2, 3), mLocalShapeToWorld * Vector3(-1, 2, -30));
            Ray ray14(mLocalShapeToWorld * Vector3(-3, 2, -1.5), mLocalShapeToWorld * Vector3(30, 1, -1.5));
            Ray ray15(mLocalShapeToWorld * Vector3(0, -9, 1), mLocalShapeToWorld * Vector3(0, 30, 1));
            Ray ray16(mLocalShapeToWorld * Vector3(-1, 2, -7), mLocalShapeToWorld * Vector3(-1, 2, 30));

            mCallback.shapeToTest = mCompoundCylinderShape;

            test(mCompoundBody->raycast(ray11, raycastInfo));
            mCallback.reset();
            mWorld->raycast(ray11, &mCallback);
            test(mCallback.isHit);
            mCallback.reset();
            mWorld->raycast(Ray(ray11.point1, ray11.point2, decimal(0.8)), &mCallback);
            test(mCallback.isHit);

            test(mCompoundBody->raycast(ray12, raycastInfo));
            mCallback.reset();
            mWorld->raycast(ray12, &mCallback);
            test(mCallback.isHit);
            mCallback.reset();
            mWorld->raycast(Ray(ray12.point1, ray12.point2, decimal(0.8)), &mCallback);
            test(mCallback.isHit);

            test(mCompoundBody->raycast(ray13, raycastInfo));
            mCallback.reset();
            mWorld->raycast(ray13, &mCallback);
            test(mCallback.isHit);
            mCallback.reset();
            mWorld->raycast(Ray(ray13.point1, ray13.point2, decimal(0.8)), &mCallback);
            test(mCallback.isHit);

            test(mCompoundBody->raycast(ray14, raycastInfo));
            mCallback.reset();
            mWorld->raycast(ray14, &mCallback);
            test(mCallback.isHit);
            mCallback.reset();
            mWorld->raycast(Ray(ray14.point1, ray14.point2, decimal(0.8)), &mCallback);
            test(mCallback.isHit);

            test(mCompoundBody->raycast(ray15, raycastInfo));
            mCallback.reset();
            mWorld->raycast(ray15, &mCallback);
            test(mCallback.isHit);
            mCallback.reset();
            mWorld->raycast(Ray(ray15.point1, ray15.point2, decimal(0.8)), &mCallback);
            test(mCallback.isHit);

            test(mCompoundBody->raycast(ray16, raycastInfo));
            mCallback.reset();
            mWorld->raycast(ray16, &mCallback);
            test(mCallback.isHit);
            mCallback.reset();
            mWorld->raycast(Ray(ray16.point1, ray16.point2, decimal(0.8)), &mCallback);
            test(mCallback.isHit);
        }
};

}

#endif
