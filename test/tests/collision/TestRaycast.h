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
#include "../../Test.h"
#include "../../src/engine/CollisionWorld.h"
#include "../../src/body/CollisionBody.h"
#include "../../src/collision/shapes/BoxShape.h"
#include "../../src/collision/shapes/SphereShape.h"
#include "../../src/collision/shapes/CapsuleShape.h"
#include "../../src/collision/shapes/ConeShape.h"
#include "../../src/collision/shapes/ConvexMeshShape.h"
#include "../../src/collision/shapes/CylinderShape.h"

/// Reactphysics3D namespace
namespace reactphysics3d {

// Class TestPointInside
/**
 * Unit test for the CollisionBody::testPointInside() method.
 */
class TestRaycast : public Test {

    private :

        // ---------- Atributes ---------- //

        // Physics world
        DynamicsWorld* mWorld;

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
        ProxyBoxShape* mBoxShape;
        ProxySphereShape* mSpherShape;
        ProxyCapsuleShape* mCapsuleShape;
        ProxyConeShape* mConeShape;
        ProxyConvexMeshShape* mConvexMeshShape;
        ProxyConvexMeshShape* mConvexMeshShapeEdgesInfo;
        ProxyCylinderShape* mCylinderShape;

    public :

        // ---------- Methods ---------- //

        /// Constructor
        TestRaycast() {

            // Create the world
            mWorld = new rp3d::CollisionWorld();

            // Body transform
            Vector3 position(-3, 2, 7);
            Quaternion orientation(PI / 5, PI / 6, PI / 7);
            mBodyTransform = Transform(position, orientation);

            // Create the bodies
            mBoxBody = mWorld->createCollisionBody(bodyTransform);
            mSphereBody = mWorld->createCollisionBody(bodyTransform);
            mCapsuleBody = mWorld->createCollisionBody(bodyTransform);
            mConeBody = mWorld->createCollisionBody(bodyTransform);
            mConvexMeshBody = mWorld->createCollisionBody(bodyTransform);
            mConvexMeshBodyEdgesInfo = mWorld->createCollisionBody(bodyTransform);
            mCylinderBody = mWorld->createCollisionBody(bodyTransform);

            // Collision shape transform
            Vector3 shapePosition(1, -4, -3);
            Quaternion shapeOrientation(3 * PI / 6 , -PI / 8, PI / 3);
            mShapeTransform = Transform(shapePosition, shapeOrientation);

            // Compute the the transform from a local shape point to world-space
            mLocalShapeToWorld = mBodyTransform * mShapeTransform;

            // Create collision shapes
            BoxShape boxShape(Vector3(2, 3, 4), 0);
            mBoxShape = mBoxBody->addCollisionShape(boxShape, shapeTransform);

            SphereShape sphereShape(3);
            mSphereShape = mSphereBody->addCollisionShape(sphereShape, shapeTransform);

            CapsuleShape capsuleShape(2, 10);
            mCapsuleShape = mCapsuleBody->addCollisionShape(capsuleShape, shapeTransform);

            ConeShape coneShape(2, 6, 0);
            mConeShape = mConeBody->addCollisionShape(coneShape, shapeTransform);

            ConvexMeshShape convexMeshShape(0);             // Box of dimension (2, 3, 4)
            convexMeshShape.addVertex(Vector3(-2, -3, 4));
            convexMeshShape.addVertex(Vector3(2, -3, 4));
            convexMeshShape.addVertex(Vector3(-2, -3, 4));
            convexMeshShape.addVertex(Vector3(2, -3, -4));
            convexMeshShape.addVertex(Vector3(-2, 3, 4));
            convexMeshShape.addVertex(Vector3(2, 3, 4));
            convexMeshShape.addVertex(Vector3(-2, 3, -4));
            convexMeshShape.addVertex(Vector3(2, 3, -4));
            mConvexMeshShape = mConvexMeshBody->addCollisionShape(convexMeshShape, shapeTransform);

            ConvexMeshShape convexMeshShapeEdgesInfo(0);
            convexMeshShapeEdgesInfo.addVertex(Vector3(-2, -3, 4));
            convexMeshShapeEdgesInfo.addVertex(Vector3(2, -3, 4));
            convexMeshShapeEdgesInfo.addVertex(Vector3(-2, -3, 4));
            convexMeshShapeEdgesInfo.addVertex(Vector3(2, -3, -4));
            convexMeshShapeEdgesInfo.addVertex(Vector3(-2, 3, 4));
            convexMeshShapeEdgesInfo.addVertex(Vector3(2, 3, 4));
            convexMeshShapeEdgesInfo.addVertex(Vector3(-2, 3, -4));
            convexMeshShapeEdgesInfo.addVertex(Vector3(2, 3, -4));
            convexMeshShapeEdgesInfo.addEdge(0, 1);
            convexMeshShapeEdgesInfo.addEdge(1, 3);
            convexMeshShapeEdgesInfo.addEdge(2, 3);
            convexMeshShapeEdgesInfo.addEdge(0, 2);
            convexMeshShapeEdgesInfo.addEdge(4, 5);
            convexMeshShapeEdgesInfo.addEdge(5, 7);
            convexMeshShapeEdgesInfo.addEdge(6, 7);
            convexMeshShapeEdgesInfo.addEdge(4, 6);
            convexMeshShapeEdgesInfo.addEdge(0, 4);
            convexMeshShapeEdgesInfo.addEdge(1, 5);
            convexMeshShapeEdgesInfo.addEdge(2, 6);
            convexMeshShapeEdgesInfo.addEdge(3, 7);
            convexMeshShapeEdgesInfo.setIsEdgesInformationUsed(true);
            mConvexMeshShapeEdgesInfo = mConvexMeshBodyEdgesInfo->addCollisionShape(
                                                                     convexMeshShapeEdgesInfo);

            CylinderShape cylinderShape(3, 8, 0);
            mCylinderShape = mCylinderBody->addCollisionShape(cylinderShape, shapeTransform);

            // Compound shape is a cylinder and a sphere
            Vector3 positionShape2(Vector3(4, 2, -3));
            Quaternion orientationShape2(-3 *PI / 8, 1.5 * PI/ 3, PI / 13);
            Transform shapeTransform2(positionShape2, orientationShape2);
            mLocalShape2ToWorld = mBodyTransform * shapeTransform2;
            mCompoundBody->addCollisionShape(cylinderShape, shapeTransform);
            mCompoundBody->addCollisionShape(sphereShape, shapeTransform2);
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
            Vector3 origin = mLocalShapeToWorld * Vector3(1 , 2, 10);
            const Matrix3x3 mLocalToWorldMatrix = mLocalShapeToWorld.getOrientation().getMatrix();
            Vector3 direction = mLocalToWorldMatrix * Vector3(0, 0, -5);
            Ray ray(origin, direction);
            Vector3 hitPoint = mLocalShapeToWorld * Vector3(1, 2, 4);

            // CollisionWorld::raycast()
            RaycastInfo raycastInfo;
            test(mWorld->raycast(ray, raycastInfo));
            test(raycastInfo.body == mBoxBody);
            test(raycastInfo.proxyShape == mBoxShape);
            test(approxEqual(raycastInfo.distance, 6));
            test(approxEqual(raycastInfo.worldPoint.x, hitPoint.x));
            test(approxEqual(raycastInfo.worldPoint.y, hitPoint.y));
            test(approxEqual(raycastInfo.worldPoint.z, hitPoint.z));

            // CollisionBody::raycast()
            RaycastInfo raycastInfo2;
            test(mBoxBody->raycast(ray, raycastInfo2));
            test(raycastInfo2.body == mBoxBody);
            test(raycastInfo2.proxyShape == mBoxShape);
            test(approxEqual(raycastInfo2.distance, 6));
            test(approxEqual(raycastInfo2.worldPoint.x, hitPoint.x));
            test(approxEqual(raycastInfo2.worldPoint.y, hitPoint.y));
            test(approxEqual(raycastInfo2.worldPoint.z, hitPoint.z));

            // ProxyCollisionShape::raycast()
            RaycastInfo raycastInfo3;
            test(mBoxShape->raycast(ray, raycastInfo3));
            test(raycastInfo3.body == mBoxBody);
            test(raycastInfo3.proxyShape == mBoxShape);
            test(approxEqual(raycastInfo3.distance, 6));
            test(approxEqual(raycastInfo3.worldPoint.x, hitPoint.x));
            test(approxEqual(raycastInfo3.worldPoint.y, hitPoint.y));
            test(approxEqual(raycastInfo3.worldPoint.z, hitPoint.z));

            Ray ray1(mLocalShapeToWorld * Vector3(0, 0, 0), mLocalToWorldMatrix * Vector3(5, 7, -1));
            Ray ray2(mLocalShapeToWorld * Vector3(5, 11, 7), mLocalToWorldMatrix * Vector3(4, 6, 7));
            Ray ray3(mLocalShapeToWorld * Vector3(1, 2, 3), mLocalToWorldMatrix * Vector3(-4, 0, 7));
            Ray ray4(mLocalShapeToWorld * Vector3(10, 10, 10), mLocalToWorldMatrix * Vector3(4, 6, 7));
            Ray ray5(mLocalShapeToWorld * Vector3(3, 1, -5), mLocalToWorldMatrix * Vector3(-3, 0, 0));
            Ray ray6(mLocalShapeToWorld * Vector3(4, 4, 1), mLocalToWorldMatrix * Vector3(0, -2, 0));
            Ray ray7(mLocalShapeToWorld * Vector3(1, -4, 5), mLocalToWorldMatrix * Vector3(0, 0, -2));
            Ray ray8(mLocalShapeToWorld * Vector3(-4, 4, 0), mLocalToWorldMatrix * Vector3(1, 0, 0));
            Ray ray9(mLocalShapeToWorld * Vector3(0, -4, -7), mLocalToWorldMatrix * Vector3(0, 5, 0));
            Ray ray10(mLocalShapeToWorld * Vector3(-3, 0, -6), mLocalToWorldMatrix * Vector3(0, 0, 8));
            Ray ray11(mLocalShapeToWorld * Vector3(3, 1, 2), mLocalToWorldMatrix * Vector3(-4, 0, 0));
            Ray ray12(mLocalShapeToWorld * Vector3(1, 4, -1), mLocalToWorldMatrix * Vector3(0, -3, 0));
            Ray ray13(mLocalShapeToWorld * Vector3(-1, 2, 5), mLocalToWorldMatrix * Vector3(0, 0, -8));
            Ray ray14(mLocalShapeToWorld * Vector3(-3, 2, -2), mLocalToWorldMatrix * Vector3(4, 0, 0));
            Ray ray15(mLocalShapeToWorld * Vector3(0, -4, 1), mLocalToWorldMatrix * Vector3(0, 3, 0));
            Ray ray16(mLocalShapeToWorld * Vector3(-1, 2, -7), mLocalToWorldMatrix * Vector3(0, 0, 8));

            // ----- Test raycast miss ----- //
            test(!mBoxBody->raycast(ray1, raycastInfo3));
            test(!mBoxShape->raycast(ray1, raycastInfo3));
            test(!mWorld->raycast(ray1, raycastInfo3));
            test(!mWorld->raycast(ray1, raycastInfo3, 1));
            test(!mWorld->raycast(ray1, raycastInfo3, 100));
            test(!mWorld->raycast(ray1));

            test(!mBoxBody->raycast(ray2, raycastInfo3));
            test(!mBoxShape->raycast(ray2, raycastInfo3));
            test(!mWorld->raycast(ray2, raycastInfo3));
            test(!mWorld->raycast(ray2));

            test(!mBoxBody->raycast(ray3, raycastInfo3));
            test(!mBoxShape->raycast(ray3, raycastInfo3));
            test(!mWorld->raycast(ray3, raycastInfo3));
            test(!mWorld->raycast(ray3));

            test(!mBoxBody->raycast(ray4, raycastInfo3));
            test(!mBoxShape->raycast(ray4, raycastInfo3));
            test(!mWorld->raycast(ray4, raycastInfo3));
            test(!mWorld->raycast(ray4));

            test(!mBoxBody->raycast(ray5, raycastInfo3));
            test(!mBoxShape->raycast(ray5, raycastInfo3));
            test(!mWorld->raycast(ray5, raycastInfo3));
            test(!mWorld->raycast(ray5));

            test(!mBoxBody->raycast(ray6, raycastInfo3));
            test(!mBoxShape->raycast(ray6, raycastInfo3));
            test(!mWorld->raycast(ray6, raycastInfo3));
            test(!mWorld->raycast(ray6));

            test(!mBoxBody->raycast(ray7, raycastInfo3));
            test(!mBoxShape->raycast(ray7, raycastInfo3));
            test(!mWorld->raycast(ray7, raycastInfo3));
            test(!mWorld->raycast(ray7));

            test(!mBoxBody->raycast(ray8, raycastInfo3));
            test(!mBoxShape->raycast(ray8, raycastInfo3));
            test(!mWorld->raycast(ray8, raycastInfo3));
            test(!mWorld->raycast(ray8));

            test(!mBoxBody->raycast(ray9, raycastInfo3));
            test(!mBoxShape->raycast(ray9, raycastInfo3));
            test(!mWorld->raycast(ray9, raycastInfo3));
            test(!mWorld->raycast(ray9));

            test(!mBoxBody->raycast(ray10, raycastInfo3));
            test(!mBoxShape->raycast(ray10, raycastInfo3));
            test(!mWorld->raycast(ray10, raycastInfo3));
            test(!mWorld->raycast(ray10));

            test(!mWorld->raycast(ray11, raycastInfo3, 0.5));
            test(!mWorld->raycast(ray12, raycastInfo3, 0.5));
            test(!mWorld->raycast(ray13, raycastInfo3, 0.5));
            test(!mWorld->raycast(ray14, raycastInfo3, 0.5));
            test(!mWorld->raycast(ray15, raycastInfo3, 0.5));
            test(!mWorld->raycast(ray16, raycastInfo3, 2));

            // ----- Test raycast hits ----- //
            test(mBoxBody->raycast(ray11, raycastInfo3));
            test(mBoxShape->raycast(ray11, raycastInfo3));
            test(mWorld->raycast(ray11, raycastInfo3));
            test(mWorld->raycast(ray11, raycastInfo3, 2));
            test(mWorld->raycast(ray11));

            test(mBoxBody->raycast(ray12, raycastInfo3));
            test(mBoxShape->raycast(ray12, raycastInfo3));
            test(mWorld->raycast(ray12, raycastInfo3));
            test(mWorld->raycast(ray12, raycastInfo3, 2));
            test(mWorld->raycast(ray12));

            test(mBoxBody->raycast(ray13, raycastInfo3));
            test(mBoxShape->raycast(ray13, raycastInfo3));
            test(mWorld->raycast(ray13, raycastInfo3));
            test(mWorld->raycast(ray13, raycastInfo3, 2));
            test(mWorld->raycast(ray13));

            test(mBoxBody->raycast(ray14, raycastInfo3));
            test(mBoxShape->raycast(ray14, raycastInfo3));
            test(mWorld->raycast(ray14, raycastInfo3));
            test(mWorld->raycast(ray14, raycastInfo3, 2));
            test(mWorld->raycast(ray14));

            test(mBoxBody->raycast(ray15, raycastInfo3));
            test(mBoxShape->raycast(ray15, raycastInfo3));
            test(mWorld->raycast(ray15, raycastInfo3));
            test(mWorld->raycast(ray15, raycastInfo3, 2));
            test(mWorld->raycast(ray15));

            test(mBoxBody->raycast(ray16, raycastInfo3));
            test(mBoxShape->raycast(ray16, raycastInfo3));
            test(mWorld->raycast(ray16, raycastInfo3));
            test(mWorld->raycast(ray16, raycastInfo3, 4));
            test(mWorld->raycast(ray16));
        }

        /// Test the ProxySphereShape::testPointInside() and
        /// CollisionBody::testPointInside() methods
        void testSphere() {

            // Tests with CollisionBody
            test(mSphereBody->testPointInside(mLocalShapeToWorld * Vector3(0, 0, 0)));
            test(mSphereBody->testPointInside(mLocalShapeToWorld * Vector3(2.9, 0, 0)));
            test(mSphereBody->testPointInside(mLocalShapeToWorld * Vector3(-2.9, 0, 0)));
            test(mSphereBody->testPointInside(mLocalShapeToWorld * Vector3(0, 2.9, 0)));
            test(mSphereBody->testPointInside(mLocalShapeToWorld * Vector3(0, -2.9, 0)));
            test(mSphereBody->testPointInside(mLocalShapeToWorld * Vector3(0, 0, 2.9)));
            test(mSphereBody->testPointInside(mLocalShapeToWorld * Vector3(0, 0, 2.9)));
            test(mSphereBody->testPointInside(mLocalShapeToWorld * Vector3(-1, -2, -1.5)));
            test(mSphereBody->testPointInside(mLocalShapeToWorld * Vector3(-1, 2, -1.5)));
            test(mSphereBody->testPointInside(mLocalShapeToWorld * Vector3(1, -2, 1.5)));

            test(!mSphereBody->testPointInside(mLocalShapeToWorld * Vector3(3.1, 0, 0)));
            test(!mSphereBody->testPointInside(mLocalShapeToWorld * Vector3(-3.1, 0, 0)));
            test(!mSphereBody->testPointInside(mLocalShapeToWorld * Vector3(0, 3.1, 0)));
            test(!mSphereBody->testPointInside(mLocalShapeToWorld * Vector3(0, -3.1, 0)));
            test(!mSphereBody->testPointInside(mLocalShapeToWorld * Vector3(0, 0, 3.1)));
            test(!mSphereBody->testPointInside(mLocalShapeToWorld * Vector3(0, 0, -3.1)));
            test(!mSphereBody->testPointInside(mLocalShapeToWorld * Vector3(-2, -2, -2)));
            test(!mSphereBody->testPointInside(mLocalShapeToWorld * Vector3(-2, 2, -1.5)));
            test(!mSphereBody->testPointInside(mLocalShapeToWorld * Vector3(1.5, -2, 2.5)));

            // Tests with ProxySphereShape
            test(mSphereShape->testPointInside(mLocalShapeToWorld * Vector3(0, 0, 0)));
            test(mSphereShape->testPointInside(mLocalShapeToWorld * Vector3(2.9, 0, 0)));
            test(mSphereShape->testPointInside(mLocalShapeToWorld * Vector3(-2.9, 0, 0)));
            test(mSphereShape->testPointInside(mLocalShapeToWorld * Vector3(0, 2.9, 0)));
            test(mSphereShape->testPointInside(mLocalShapeToWorld * Vector3(0, -2.9, 0)));
            test(mSphereShape->testPointInside(mLocalShapeToWorld * Vector3(0, 0, 2.9)));
            test(mSphereShape->testPointInside(mLocalShapeToWorld * Vector3(0, 0, 2.9)));
            test(mSphereShape->testPointInside(mLocalShapeToWorld * Vector3(-1, -2, -1.5)));
            test(mSphereShape->testPointInside(mLocalShapeToWorld * Vector3(-1, 2, -1.5)));
            test(mSphereShape->testPointInside(mLocalShapeToWorld * Vector3(1, -2, 1.5)));

            test(!mSphereShape->testPointInside(mLocalShapeToWorld * Vector3(3.1, 0, 0)));
            test(!mSphereShape->testPointInside(mLocalShapeToWorld * Vector3(-3.1, 0, 0)));
            test(!mSphereShape->testPointInside(mLocalShapeToWorld * Vector3(0, 3.1, 0)));
            test(!mSphereShape->testPointInside(mLocalShapeToWorld * Vector3(0, -3.1, 0)));
            test(!mSphereShape->testPointInside(mLocalShapeToWorld * Vector3(0, 0, 3.1)));
            test(!mSphereShape->testPointInside(mLocalShapeToWorld * Vector3(0, 0, -3.1)));
            test(!mSphereShape->testPointInside(mLocalShapeToWorld * Vector3(-2, -2, -2)));
            test(!mSphereShape->testPointInside(mLocalShapeToWorld * Vector3(-2, 2, -1.5)));
            test(!mSphereShape->testPointInside(mLocalShapeToWorld * Vector3(1.5, -2, 2.5)));
        }

        /// Test the ProxyCapsuleShape::testPointInside() and
        /// CollisionBody::testPointInside() methods
        void testCapsule() {

            // Tests with CollisionBody
            test(mCapsuleBody->testPointInside(mLocalShapeToWorld * Vector3(0, 0, 0)));
            test(mCapsuleBody->testPointInside(mLocalShapeToWorld * Vector3(0, 5, 0)));
            test(mCapsuleBody->testPointInside(mLocalShapeToWorld * Vector3(0, -5, 0)));
            test(mCapsuleBody->testPointInside(mLocalShapeToWorld * Vector3(0, -6.9, 0)));
            test(mCapsuleBody->testPointInside(mLocalShapeToWorld * Vector3(0, 6.9, 0)));
            test(mCapsuleBody->testPointInside(mLocalShapeToWorld * Vector3(0, 0, 1.9)));
            test(mCapsuleBody->testPointInside(mLocalShapeToWorld * Vector3(0, 0, -1.9)));
            test(mCapsuleBody->testPointInside(mLocalShapeToWorld * Vector3(1.9, 0, 0)));
            test(mCapsuleBody->testPointInside(mLocalShapeToWorld * Vector3(-1.9, 0, 0)));
            test(mCapsuleBody->testPointInside(mLocalShapeToWorld * Vector3(0.9, 0, 0.9)));
            test(mCapsuleBody->testPointInside(mLocalShapeToWorld * Vector3(0.9, 0, -0.9)));
            test(mCapsuleBody->testPointInside(mLocalShapeToWorld * Vector3(0, 5, 1.9)));
            test(mCapsuleBody->testPointInside(mLocalShapeToWorld * Vector3(0, 5, -1.9)));
            test(mCapsuleBody->testPointInside(mLocalShapeToWorld * Vector3(1.9, 5, 0)));
            test(mCapsuleBody->testPointInside(mLocalShapeToWorld * Vector3(-1.9, 5, 0)));
            test(mCapsuleBody->testPointInside(mLocalShapeToWorld * Vector3(0.9, 5, 0.9)));
            test(mCapsuleBody->testPointInside(mLocalShapeToWorld * Vector3(0.9, 5, -0.9)));
            test(mCapsuleBody->testPointInside(mLocalShapeToWorld * Vector3(0, -5, 1.9)));
            test(mCapsuleBody->testPointInside(mLocalShapeToWorld * Vector3(0, -5, -1.9)));
            test(mCapsuleBody->testPointInside(mLocalShapeToWorld * Vector3(1.9, -5, 0)));
            test(mCapsuleBody->testPointInside(mLocalShapeToWorld * Vector3(-1.9, -5, 0)));
            test(mCapsuleBody->testPointInside(mLocalShapeToWorld * Vector3(0.9, -5, 0.9)));
            test(mCapsuleBody->testPointInside(mLocalShapeToWorld * Vector3(0.9, -5, -0.9)));
            test(mCapsuleBody->testPointInside(mLocalShapeToWorld * Vector3(-1.8, -4, -1)));
            test(mCapsuleBody->testPointInside(mLocalShapeToWorld * Vector3(-1, 2, 0.4)));
            test(mCapsuleBody->testPointInside(mLocalShapeToWorld * Vector3(1.3, 1, 1.6)));

            test(!mCapsuleBody->testPointInside(mLocalShapeToWorld * Vector3(0, -7.1, 0)));
            test(!mCapsuleBody->testPointInside(mLocalShapeToWorld * Vector3(0, 7.1, 0)));
            test(!mCapsuleBody->testPointInside(mLocalShapeToWorld * Vector3(0, 0, 2.1)));
            test(!mCapsuleBody->testPointInside(mLocalShapeToWorld * Vector3(0, 0, -2.1)));
            test(!mCapsuleBody->testPointInside(mLocalShapeToWorld * Vector3(2.1, 0, 0)));
            test(!mCapsuleBody->testPointInside(mLocalShapeToWorld * Vector3(-2.1, 0, 0)));
            test(!mCapsuleBody->testPointInside(mLocalShapeToWorld * Vector3(0, 5, 2.1)));
            test(!mCapsuleBody->testPointInside(mLocalShapeToWorld * Vector3(0, 5, -2.1)));
            test(!mCapsuleBody->testPointInside(mLocalShapeToWorld * Vector3(2.1, 5, 0)));
            test(!mCapsuleBody->testPointInside(mLocalShapeToWorld * Vector3(-2.1, 5, 0)));
            test(!mCapsuleBody->testPointInside(mLocalShapeToWorld * Vector3(1.5, 5, 1.6)));
            test(!mCapsuleBody->testPointInside(mLocalShapeToWorld * Vector3(1.5, 5, -1.7)));
            test(!mCapsuleBody->testPointInside(mLocalShapeToWorld * Vector3(0, -5, 2.1)));
            test(!mCapsuleBody->testPointInside(mLocalShapeToWorld * Vector3(0, -5, -2.1)));
            test(!mCapsuleBody->testPointInside(mLocalShapeToWorld * Vector3(2.1, -5, 0)));
            test(!mCapsuleBody->testPointInside(mLocalShapeToWorld * Vector3(-2.1, -5, 0)));
            test(!mCapsuleBody->testPointInside(mLocalShapeToWorld * Vector3(1.5, -5, 1.6)));
            test(!mCapsuleBody->testPointInside(mLocalShapeToWorld * Vector3(1.5, -5, -1.7)));

            // Tests with ProxyCapsuleShape
            test(mCapsuleShape->testPointInside(mLocalShapeToWorld * Vector3(0, 0, 0)));
            test(mCapsuleShape->testPointInside(mLocalShapeToWorld * Vector3(0, 5, 0)));
            test(mCapsuleShape->testPointInside(mLocalShapeToWorld * Vector3(0, -5, 0)));
            test(mCapsuleShape->testPointInside(mLocalShapeToWorld * Vector3(0, -6.9, 0)));
            test(mCapsuleShape->testPointInside(mLocalShapeToWorld * Vector3(0, 6.9, 0)));
            test(mCapsuleShape->testPointInside(mLocalShapeToWorld * Vector3(0, 0, 1.9)));
            test(mCapsuleShape->testPointInside(mLocalShapeToWorld * Vector3(0, 0, -1.9)));
            test(mCapsuleShape->testPointInside(mLocalShapeToWorld * Vector3(1.9, 0, 0)));
            test(mCapsuleShape->testPointInside(mLocalShapeToWorld * Vector3(-1.9, 0, 0)));
            test(mCapsuleShape->testPointInside(mLocalShapeToWorld * Vector3(0.9, 0, 0.9)));
            test(mCapsuleShape->testPointInside(mLocalShapeToWorld * Vector3(0.9, 0, -0.9)));
            test(mCapsuleShape->testPointInside(mLocalShapeToWorld * Vector3(0, 5, 1.9)));
            test(mCapsuleShape->testPointInside(mLocalShapeToWorld * Vector3(0, 5, -1.9)));
            test(mCapsuleShape->testPointInside(mLocalShapeToWorld * Vector3(1.9, 5, 0)));
            test(mCapsuleShape->testPointInside(mLocalShapeToWorld * Vector3(-1.9, 5, 0)));
            test(mCapsuleShape->testPointInside(mLocalShapeToWorld * Vector3(0.9, 5, 0.9)));
            test(mCapsuleShape->testPointInside(mLocalShapeToWorld * Vector3(0.9, 5, -0.9)));
            test(mCapsuleShape->testPointInside(mLocalShapeToWorld * Vector3(0, -5, 1.9)));
            test(mCapsuleShape->testPointInside(mLocalShapeToWorld * Vector3(0, -5, -1.9)));
            test(mCapsuleShape->testPointInside(mLocalShapeToWorld * Vector3(1.9, -5, 0)));
            test(mCapsuleShape->testPointInside(mLocalShapeToWorld * Vector3(-1.9, -5, 0)));
            test(mCapsuleShape->testPointInside(mLocalShapeToWorld * Vector3(0.9, -5, 0.9)));
            test(mCapsuleShape->testPointInside(mLocalShapeToWorld * Vector3(0.9, -5, -0.9)));
            test(mCapsuleShape->testPointInside(mLocalShapeToWorld * Vector3(-1.8, -4, -1)));
            test(mCapsuleShape->testPointInside(mLocalShapeToWorld * Vector3(-1, 2, 0.4)));
            test(mCapsuleShape->testPointInside(mLocalShapeToWorld * Vector3(1.3, 1, 1.6)));

            test(!mCapsuleShape->testPointInside(mLocalShapeToWorld * Vector3(0, -7.1, 0)));
            test(!mCapsuleShape->testPointInside(mLocalShapeToWorld * Vector3(0, 7.1, 0)));
            test(!mCapsuleShape->testPointInside(mLocalShapeToWorld * Vector3(0, 0, 2.1)));
            test(!mCapsuleShape->testPointInside(mLocalShapeToWorld * Vector3(0, 0, -2.1)));
            test(!mCapsuleShape->testPointInside(mLocalShapeToWorld * Vector3(2.1, 0, 0)));
            test(!mCapsuleShape->testPointInside(mLocalShapeToWorld * Vector3(-2.1, 0, 0)));
            test(!mCapsuleShape->testPointInside(mLocalShapeToWorld * Vector3(0, 5, 2.1)));
            test(!mCapsuleShape->testPointInside(mLocalShapeToWorld * Vector3(0, 5, -2.1)));
            test(!mCapsuleShape->testPointInside(mLocalShapeToWorld * Vector3(2.1, 5, 0)));
            test(!mCapsuleShape->testPointInside(mLocalShapeToWorld * Vector3(-2.1, 5, 0)));
            test(!mCapsuleShape->testPointInside(mLocalShapeToWorld * Vector3(1.5, 5, 1.6)));
            test(!mCapsuleShape->testPointInside(mLocalShapeToWorld * Vector3(1.5, 5, -1.7)));
            test(!mCapsuleShape->testPointInside(mLocalShapeToWorld * Vector3(0, -5, 2.1)));
            test(!mCapsuleShape->testPointInside(mLocalShapeToWorld * Vector3(0, -5, -2.1)));
            test(!mCapsuleShape->testPointInside(mLocalShapeToWorld * Vector3(2.1, -5, 0)));
            test(!mCapsuleShape->testPointInside(mLocalShapeToWorld * Vector3(-2.1, -5, 0)));
            test(!mCapsuleShape->testPointInside(mLocalShapeToWorld * Vector3(1.5, -5, 1.6)));
            test(!mCapsuleShape->testPointInside(mLocalShapeToWorld * Vector3(1.5, -5, -1.7)));
        }

        /// Test the ProxyConeShape::testPointInside() and
        /// CollisionBody::testPointInside() methods
        void testCone() {

            // Tests with CollisionBody
            test(mConeBody->testPointInside(mLocalShapeToWorld * Vector3(0, 0, 0)));
            test(mConeBody->testPointInside(mLocalShapeToWorld * Vector3(0.9, 0, 0)));
            test(mConeBody->testPointInside(mLocalShapeToWorld * Vector3(-0.9, 0, 0)));
            test(mConeBody->testPointInside(mLocalShapeToWorld * Vector3(0, 0, 0.9)));
            test(mConeBody->testPointInside(mLocalShapeToWorld * Vector3(0, 0, -0.9)));
            test(mConeBody->testPointInside(mLocalShapeToWorld * Vector3(0.6, 0, -0.7)));
            test(mConeBody->testPointInside(mLocalShapeToWorld * Vector3(0.6, 0, 0.7)));
            test(mConeBody->testPointInside(mLocalShapeToWorld * Vector3(-0.6, 0, -0.7)));
            test(mConeBody->testPointInside(mLocalShapeToWorld * Vector3(-0.6, 0, 0.7)));
            test(mConeBody->testPointInside(mLocalShapeToWorld * Vector3(0, 2.9, 0)));
            test(mConeBody->testPointInside(mLocalShapeToWorld * Vector3(0, -2.9, 0)));
            test(mConeBody->testPointInside(mLocalShapeToWorld * Vector3(1.96, -2.9, 0)));
            test(mConeBody->testPointInside(mLocalShapeToWorld * Vector3(-1.96, -2.9, 0)));
            test(mConeBody->testPointInside(mLocalShapeToWorld * Vector3(0, -2.9, 1.96)));
            test(mConeBody->testPointInside(mLocalShapeToWorld * Vector3(0, -2.9, -1.96)));
            test(mConeBody->testPointInside(mLocalShapeToWorld * Vector3(1.3, -2.9, -1.4)));
            test(mConeBody->testPointInside(mLocalShapeToWorld * Vector3(-1.3, -2.9, 1.4)));

            test(!mConeBody->testPointInside(mLocalShapeToWorld * Vector3(1.1, 0, 0)));
            test(!mConeBody->testPointInside(mLocalShapeToWorld * Vector3(-1.1, 0, 0)));
            test(!mConeBody->testPointInside(mLocalShapeToWorld * Vector3(0, 0, 1.1)));
            test(!mConeBody->testPointInside(mLocalShapeToWorld * Vector3(0, 0, -1.1)));
            test(!mConeBody->testPointInside(mLocalShapeToWorld * Vector3(0.8, 0, -0.8)));
            test(!mConeBody->testPointInside(mLocalShapeToWorld * Vector3(0.8, 0, 0.8)));
            test(!mConeBody->testPointInside(mLocalShapeToWorld * Vector3(-0.8, 0, -0.8)));
            test(!mConeBody->testPointInside(mLocalShapeToWorld * Vector3(-0.8, 0, 0.8)));
            test(!mConeBody->testPointInside(mLocalShapeToWorld * Vector3(0, 3.1, 0)));
            test(!mConeBody->testPointInside(mLocalShapeToWorld * Vector3(0, -3.1, 0)));
            test(!mConeBody->testPointInside(mLocalShapeToWorld * Vector3(1.97, -2.9, 0)));
            test(!mConeBody->testPointInside(mLocalShapeToWorld * Vector3(-1.97, -2.9, 0)));
            test(!mConeBody->testPointInside(mLocalShapeToWorld * Vector3(0, -2.9, 1.97)));
            test(!mConeBody->testPointInside(mLocalShapeToWorld * Vector3(0, -2.9, -1.97)));
            test(!mConeBody->testPointInside(mLocalShapeToWorld * Vector3(1.5, -2.9, -1.5)));
            test(!mConeBody->testPointInside(mLocalShapeToWorld * Vector3(-1.5, -2.9, 1.5)));

            // Tests with ProxyConeShape
            test(mConeShape->testPointInside(mLocalShapeToWorld * Vector3(0, 0, 0)));
            test(mConeShape->testPointInside(mLocalShapeToWorld * Vector3(0.9, 0, 0)));
            test(mConeShape->testPointInside(mLocalShapeToWorld * Vector3(-0.9, 0, 0)));
            test(mConeShape->testPointInside(mLocalShapeToWorld * Vector3(0, 0, 0.9)));
            test(mConeShape->testPointInside(mLocalShapeToWorld * Vector3(0, 0, -0.9)));
            test(mConeShape->testPointInside(mLocalShapeToWorld * Vector3(0.6, 0, -0.7)));
            test(mConeShape->testPointInside(mLocalShapeToWorld * Vector3(0.6, 0, 0.7)));
            test(mConeShape->testPointInside(mLocalShapeToWorld * Vector3(-0.6, 0, -0.7)));
            test(mConeShape->testPointInside(mLocalShapeToWorld * Vector3(-0.6, 0, 0.7)));
            test(mConeShape->testPointInside(mLocalShapeToWorld * Vector3(0, 2.9, 0)));
            test(mConeShape->testPointInside(mLocalShapeToWorld * Vector3(0, -2.9, 0)));
            test(mConeShape->testPointInside(mLocalShapeToWorld * Vector3(1.96, -2.9, 0)));
            test(mConeShape->testPointInside(mLocalShapeToWorld * Vector3(-1.96, -2.9, 0)));
            test(mConeShape->testPointInside(mLocalShapeToWorld * Vector3(0, -2.9, 1.96)));
            test(mConeShape->testPointInside(mLocalShapeToWorld * Vector3(0, -2.9, -1.96)));
            test(mConeShape->testPointInside(mLocalShapeToWorld * Vector3(1.3, -2.9, -1.4)));
            test(mConeShape->testPointInside(mLocalShapeToWorld * Vector3(-1.3, -2.9, 1.4)));

            test(!mConeShape->testPointInside(mLocalShapeToWorld * Vector3(1.1, 0, 0)));
            test(!mConeShape->testPointInside(mLocalShapeToWorld * Vector3(-1.1, 0, 0)));
            test(!mConeShape->testPointInside(mLocalShapeToWorld * Vector3(0, 0, 1.1)));
            test(!mConeShape->testPointInside(mLocalShapeToWorld * Vector3(0, 0, -1.1)));
            test(!mConeShape->testPointInside(mLocalShapeToWorld * Vector3(0.8, 0, -0.8)));
            test(!mConeShape->testPointInside(mLocalShapeToWorld * Vector3(0.8, 0, 0.8)));
            test(!mConeShape->testPointInside(mLocalShapeToWorld * Vector3(-0.8, 0, -0.8)));
            test(!mConeShape->testPointInside(mLocalShapeToWorld * Vector3(-0.8, 0, 0.8)));
            test(!mConeShape->testPointInside(mLocalShapeToWorld * Vector3(0, 3.1, 0)));
            test(!mConeShape->testPointInside(mLocalShapeToWorld * Vector3(0, -3.1, 0)));
            test(!mConeShape->testPointInside(mLocalShapeToWorld * Vector3(1.97, -2.9, 0)));
            test(!mConeShape->testPointInside(mLocalShapeToWorld * Vector3(-1.97, -2.9, 0)));
            test(!mConeShape->testPointInside(mLocalShapeToWorld * Vector3(0, -2.9, 1.97)));
            test(!mConeShape->testPointInside(mLocalShapeToWorld * Vector3(0, -2.9, -1.97)));
            test(!mConeShape->testPointInside(mLocalShapeToWorld * Vector3(1.5, -2.9, -1.5)));
            test(!mConeShape->testPointInside(mLocalShapeToWorld * Vector3(-1.5, -2.9, 1.5)));
        }

        /// Test the ProxyConvexMeshShape::testPointInside() and
        /// CollisionBody::testPointInside() methods
        void testConvexMesh() {

            // ----- Tests without using edges information ----- //

            // Tests with CollisionBody
            test(mConvexMeshBody->testPointInside(mLocalShapeToWorld * Vector3(0, 0, 0)));
            test(mConvexMeshBody->testPointInside(mLocalShapeToWorld * Vector3(-1.9, 0, 0)));
            test(mConvexMeshBody->testPointInside(mLocalShapeToWorld * Vector3(1.9, 0, 0)));
            test(mConvexMeshBody->testPointInside(mLocalShapeToWorld * Vector3(0, -2.9, 0)));
            test(mConvexMeshBody->testPointInside(mLocalShapeToWorld * Vector3(0, 2.9, 0)));
            test(mConvexMeshBody->testPointInside(mLocalShapeToWorld * Vector3(0, 0, -3.9)));
            test(mConvexMeshBody->testPointInside(mLocalShapeToWorld * Vector3(0, 0, 3.9)));
            test(mConvexMeshBody->testPointInside(mLocalShapeToWorld * Vector3(-1.9, -2.9, -3.9)));
            test(mConvexMeshBody->testPointInside(mLocalShapeToWorld * Vector3(1.9, 2.9, 3.9)));
            test(mConvexMeshBody->testPointInside(mLocalShapeToWorld * Vector3(-1, -2, -1.5)));
            test(mConvexMeshBody->testPointInside(mLocalShapeToWorld * Vector3(-1, 2, -2.5)));
            test(mConvexMeshBody->testPointInside(mLocalShapeToWorld * Vector3(1, -2, 3.5)));

            test(!mConvexMeshBody->testPointInside(mLocalShapeToWorld * Vector3(-2.1, 0, 0)));
            test(!mConvexMeshBody->testPointInside(mLocalShapeToWorld * Vector3(2.1, 0, 0)));
            test(!mConvexMeshBody->testPointInside(mLocalShapeToWorld * Vector3(0, -3.1, 0)));
            test(!mConvexMeshBody->testPointInside(mLocalShapeToWorld * Vector3(0, 3.1, 0)));
            test(!mConvexMeshBody->testPointInside(mLocalShapeToWorld * Vector3(0, 0, -4.1)));
            test(!mConvexMeshBody->testPointInside(mLocalShapeToWorld * Vector3(0, 0, 4.1)));
            test(!mConvexMeshBody->testPointInside(mLocalShapeToWorld * Vector3(-2.1, -3.1, -4.1)));
            test(!mConvexMeshBody->testPointInside(mLocalShapeToWorld * Vector3(2.1, 3.1, 4.1)));
            test(!mConvexMeshBody->testPointInside(mLocalShapeToWorld * Vector3(-10, -2, -1.5)));
            test(!mConvexMeshBody->testPointInside(mLocalShapeToWorld * Vector3(-1, 4, -2.5)));
            test(!mConvexMeshBody->testPointInside(mLocalShapeToWorld * Vector3(1, -2, 4.5)));

            // Tests with ProxyConvexMeshShape
            test(mConvexMeshShape->testPointInside(mLocalShapeToWorld * Vector3(0, 0, 0)));
            test(mConvexMeshShape->testPointInside(mLocalShapeToWorld * Vector3(-1.9, 0, 0)));
            test(mConvexMeshShape->testPointInside(mLocalShapeToWorld * Vector3(1.9, 0, 0)));
            test(mConvexMeshShape->testPointInside(mLocalShapeToWorld * Vector3(0, -2.9, 0)));
            test(mConvexMeshShape->testPointInside(mLocalShapeToWorld * Vector3(0, 2.9, 0)));
            test(mConvexMeshShape->testPointInside(mLocalShapeToWorld * Vector3(0, 0, -3.9)));
            test(mConvexMeshShape->testPointInside(mLocalShapeToWorld * Vector3(0, 0, 3.9)));
            test(mConvexMeshShape->testPointInside(mLocalShapeToWorld * Vector3(-1.9, -2.9, -3.9)));
            test(mConvexMeshShape->testPointInside(mLocalShapeToWorld * Vector3(1.9, 2.9, 3.9)));
            test(mConvexMeshShape->testPointInside(mLocalShapeToWorld * Vector3(-1, -2, -1.5)));
            test(mConvexMeshShape->testPointInside(mLocalShapeToWorld * Vector3(-1, 2, -2.5)));
            test(mConvexMeshShape->testPointInside(mLocalShapeToWorld * Vector3(1, -2, 3.5)));

            test(!mConvexMeshShape->testPointInside(mLocalShapeToWorld * Vector3(-2.1, 0, 0)));
            test(!mConvexMeshShape->testPointInside(mLocalShapeToWorld * Vector3(2.1, 0, 0)));
            test(!mConvexMeshShape->testPointInside(mLocalShapeToWorld * Vector3(0, -3.1, 0)));
            test(!mConvexMeshShape->testPointInside(mLocalShapeToWorld * Vector3(0, 3.1, 0)));
            test(!mConvexMeshShape->testPointInside(mLocalShapeToWorld * Vector3(0, 0, -4.1)));
            test(!mConvexMeshShape->testPointInside(mLocalShapeToWorld * Vector3(0, 0, 4.1)));
            test(!mConvexMeshShape->testPointInside(mLocalShapeToWorld * Vector3(-2.1, -3.1, -4.1)));
            test(!mConvexMeshShape->testPointInside(mLocalShapeToWorld * Vector3(2.1, 3.1, 4.1)));
            test(!mConvexMeshShape->testPointInside(mLocalShapeToWorld * Vector3(-10, -2, -1.5)));
            test(!mConvexMeshShape->testPointInside(mLocalShapeToWorld * Vector3(-1, 4, -2.5)));
            test(!mConvexMeshShape->testPointInside(mLocalShapeToWorld * Vector3(1, -2, 4.5)));

            // ----- Tests using edges information ----- //

            // Tests with CollisionBody
            test(mConvexMeshBodyEdgesInfo->testPointInside(mLocalShapeToWorld * Vector3(0, 0, 0)));
            test(mConvexMeshBodyEdgesInfo->testPointInside(mLocalShapeToWorld * Vector3(-1.9, 0, 0)));
            test(mConvexMeshBodyEdgesInfo->testPointInside(mLocalShapeToWorld * Vector3(1.9, 0, 0)));
            test(mConvexMeshBodyEdgesInfo->testPointInside(mLocalShapeToWorld * Vector3(0, -2.9, 0)));
            test(mConvexMeshBodyEdgesInfo->testPointInside(mLocalShapeToWorld * Vector3(0, 2.9, 0)));
            test(mConvexMeshBodyEdgesInfo->testPointInside(mLocalShapeToWorld * Vector3(0, 0, -3.9)));
            test(mConvexMeshBodyEdgesInfo->testPointInside(mLocalShapeToWorld * Vector3(0, 0, 3.9)));
            test(mConvexMeshBodyEdgesInfo->testPointInside(mLocalShapeToWorld * Vector3(-1.9, -2.9, -3.9)));
            test(mConvexMeshBodyEdgesInfo->testPointInside(mLocalShapeToWorld * Vector3(1.9, 2.9, 3.9)));
            test(mConvexMeshBodyEdgesInfo->testPointInside(mLocalShapeToWorld * Vector3(-1, -2, -1.5)));
            test(mConvexMeshBodyEdgesInfo->testPointInside(mLocalShapeToWorld * Vector3(-1, 2, -2.5)));
            test(mConvexMeshBodyEdgesInfo->testPointInside(mLocalShapeToWorld * Vector3(1, -2, 3.5)));

            test(!mConvexMeshBodyEdgesInfo->testPointInside(mLocalShapeToWorld * Vector3(-2.1, 0, 0)));
            test(!mConvexMeshBodyEdgesInfo->testPointInside(mLocalShapeToWorld * Vector3(2.1, 0, 0)));
            test(!mConvexMeshBodyEdgesInfo->testPointInside(mLocalShapeToWorld * Vector3(0, -3.1, 0)));
            test(!mConvexMeshBodyEdgesInfo->testPointInside(mLocalShapeToWorld * Vector3(0, 3.1, 0)));
            test(!mConvexMeshBodyEdgesInfo->testPointInside(mLocalShapeToWorld * Vector3(0, 0, -4.1)));
            test(!mConvexMeshBodyEdgesInfo->testPointInside(mLocalShapeToWorld * Vector3(0, 0, 4.1)));
            test(!mConvexMeshBodyEdgesInfo->testPointInside(mLocalShapeToWorld * Vector3(-2.1, -3.1, -4.1)));
            test(!mConvexMeshBodyEdgesInfo->testPointInside(mLocalShapeToWorld * Vector3(2.1, 3.1, 4.1)));
            test(!mConvexMeshBodyEdgesInfo->testPointInside(mLocalShapeToWorld * Vector3(-10, -2, -1.5)));
            test(!mConvexMeshBodyEdgesInfo->testPointInside(mLocalShapeToWorld * Vector3(-1, 4, -2.5)));
            test(!mConvexMeshBodyEdgesInfo->testPointInside(mLocalShapeToWorld * Vector3(1, -2, 4.5)));

            // Tests with ProxyConvexMeshShape
            test(mConvexMeshShapeEdgesInfo->testPointInside(mLocalShapeToWorld * Vector3(0, 0, 0)));
            test(mConvexMeshShapeEdgesInfo->testPointInside(mLocalShapeToWorld * Vector3(-1.9, 0, 0)));
            test(mConvexMeshShapeEdgesInfo->testPointInside(mLocalShapeToWorld * Vector3(1.9, 0, 0)));
            test(mConvexMeshShapeEdgesInfo->testPointInside(mLocalShapeToWorld * Vector3(0, -2.9, 0)));
            test(mConvexMeshShapeEdgesInfo->testPointInside(mLocalShapeToWorld * Vector3(0, 2.9, 0)));
            test(mConvexMeshShapeEdgesInfo->testPointInside(mLocalShapeToWorld * Vector3(0, 0, -3.9)));
            test(mConvexMeshShapeEdgesInfo->testPointInside(mLocalShapeToWorld * Vector3(0, 0, 3.9)));
            test(mConvexMeshShapeEdgesInfo->testPointInside(mLocalShapeToWorld * Vector3(-1.9, -2.9, -3.9)));
            test(mConvexMeshShapeEdgesInfo->testPointInside(mLocalShapeToWorld * Vector3(1.9, 2.9, 3.9)));
            test(mConvexMeshShapeEdgesInfo->testPointInside(mLocalShapeToWorld * Vector3(-1, -2, -1.5)));
            test(mConvexMeshShapeEdgesInfo->testPointInside(mLocalShapeToWorld * Vector3(-1, 2, -2.5)));
            test(mConvexMeshShapeEdgesInfo->testPointInside(mLocalShapeToWorld * Vector3(1, -2, 3.5)));

            test(!mConvexMeshShapeEdgesInfo->testPointInside(mLocalShapeToWorld * Vector3(-2.1, 0, 0)));
            test(!mConvexMeshShapeEdgesInfo->testPointInside(mLocalShapeToWorld * Vector3(2.1, 0, 0)));
            test(!mConvexMeshShapeEdgesInfo->testPointInside(mLocalShapeToWorld * Vector3(0, -3.1, 0)));
            test(!mConvexMeshShapeEdgesInfo->testPointInside(mLocalShapeToWorld * Vector3(0, 3.1, 0)));
            test(!mConvexMeshShapeEdgesInfo->testPointInside(mLocalShapeToWorld * Vector3(0, 0, -4.1)));
            test(!mConvexMeshShapeEdgesInfo->testPointInside(mLocalShapeToWorld * Vector3(0, 0, 4.1)));
            test(!mConvexMeshShapeEdgesInfo->testPointInside(mLocalShapeToWorld * Vector3(-2.1, -3.1, -4.1)));
            test(!mConvexMeshShapeEdgesInfo->testPointInside(mLocalShapeToWorld * Vector3(2.1, 3.1, 4.1)));
            test(!mConvexMeshShapeEdgesInfo->testPointInside(mLocalShapeToWorld * Vector3(-10, -2, -1.5)));
            test(!mConvexMeshShapeEdgesInfo->testPointInside(mLocalShapeToWorld * Vector3(-1, 4, -2.5)));
            test(!mConvexMeshShapeEdgesInfo->testPointInside(mLocalShapeToWorld * Vector3(1, -2, 4.5)));
        }

        /// Test the ProxyCylinderShape::testPointInside() and
        /// CollisionBody::testPointInside() methods
        void testCylinder() {

            // Tests with CollisionBody
            test(mCylinderBody->testPointInside(mLocalShapeToWorld * Vector3(0, 0, 0)));
            test(mCylinderBody->testPointInside(mLocalShapeToWorld * Vector3(0, 3.9, 0)));
            test(mCylinderBody->testPointInside(mLocalShapeToWorld * Vector3(0, -3.9, 0)));
            test(mCylinderBody->testPointInside(mLocalShapeToWorld * Vector3(2.9, 0, 0)));
            test(mCylinderBody->testPointInside(mLocalShapeToWorld * Vector3(-2.9, 0, 0)));
            test(mCylinderBody->testPointInside(mLocalShapeToWorld * Vector3(0, 0, 2.9)));
            test(mCylinderBody->testPointInside(mLocalShapeToWorld * Vector3(0, 0, -2.9)));
            test(mCylinderBody->testPointInside(mLocalShapeToWorld * Vector3(1.7, 0, 1.7)));
            test(mCylinderBody->testPointInside(mLocalShapeToWorld * Vector3(1.7, 0, -1.7)));
            test(mCylinderBody->testPointInside(mLocalShapeToWorld * Vector3(-1.7, 0, -1.7)));
            test(mCylinderBody->testPointInside(mLocalShapeToWorld * Vector3(-1.7, 0, 1.7)));
            test(mCylinderBody->testPointInside(mLocalShapeToWorld * Vector3(2.9, 3.9, 0)));
            test(mCylinderBody->testPointInside(mLocalShapeToWorld * Vector3(-2.9, 3.9, 0)));
            test(mCylinderBody->testPointInside(mLocalShapeToWorld * Vector3(0, 3.9, 2.9)));
            test(mCylinderBody->testPointInside(mLocalShapeToWorld * Vector3(0, 3.9, -2.9)));
            test(mCylinderBody->testPointInside(mLocalShapeToWorld * Vector3(1.7, 3.9, 1.7)));
            test(mCylinderBody->testPointInside(mLocalShapeToWorld * Vector3(1.7, 3.9, -1.7)));
            test(mCylinderBody->testPointInside(mLocalShapeToWorld * Vector3(-1.7, 3.9, -1.7)));
            test(mCylinderBody->testPointInside(mLocalShapeToWorld * Vector3(-1.7, 3.9, 1.7)));
            test(mCylinderBody->testPointInside(mLocalShapeToWorld * Vector3(2.9, -3.9, 0)));
            test(mCylinderBody->testPointInside(mLocalShapeToWorld * Vector3(-2.9, -3.9, 0)));
            test(mCylinderBody->testPointInside(mLocalShapeToWorld * Vector3(0, -3.9, 2.9)));
            test(mCylinderBody->testPointInside(mLocalShapeToWorld * Vector3(0, -3.9, -2.9)));
            test(mCylinderBody->testPointInside(mLocalShapeToWorld * Vector3(1.7, -3.9, 1.7)));
            test(mCylinderBody->testPointInside(mLocalShapeToWorld * Vector3(1.7, -3.9, -1.7)));
            test(mCylinderBody->testPointInside(mLocalShapeToWorld * Vector3(-1.7, -3.9, -1.7)));
            test(mCylinderBody->testPointInside(mLocalShapeToWorld * Vector3(-1.7, -3.9, 1.7)));

            test(!mCylinderBody->testPointInside(mLocalShapeToWorld * Vector3(0, 4.1, 0)));
            test(!!mCylinderBody->testPointInside(mLocalShapeToWorld * Vector3(0, -4.1, 0)));
            test(!mCylinderBody->testPointInside(mLocalShapeToWorld * Vector3(3.1, 0, 0)));
            test(!mCylinderBody->testPointInside(mLocalShapeToWorld * Vector3(-3.1, 0, 0)));
            test(!mCylinderBody->testPointInside(mLocalShapeToWorld * Vector3(0, 0, 3.1)));
            test(!mCylinderBody->testPointInside(mLocalShapeToWorld * Vector3(0, 0, -3.1)));
            test(!mCylinderBody->testPointInside(mLocalShapeToWorld * Vector3(2.2, 0, 2.2)));
            test(!mCylinderBody->testPointInside(mLocalShapeToWorld * Vector3(2.2, 0, -2.2)));
            test(!mCylinderBody->testPointInside(mLocalShapeToWorld * Vector3(-2.2, 0, -2.2)));
            test(!mCylinderBody->testPointInside(mLocalShapeToWorld * Vector3(-2.2, 0, 1.7)));
            test(!mCylinderBody->testPointInside(mLocalShapeToWorld * Vector3(3.1, 3.9, 0)));
            test(!mCylinderBody->testPointInside(mLocalShapeToWorld * Vector3(-3.1, 3.9, 0)));
            test(!mCylinderBody->testPointInside(mLocalShapeToWorld * Vector3(0, 3.9, 3.1)));
            test(!mCylinderBody->testPointInside(mLocalShapeToWorld * Vector3(0, 3.9, -3.1)));
            test(!mCylinderBody->testPointInside(mLocalShapeToWorld * Vector3(2.2, 3.9, 2.2)));
            test(!mCylinderBody->testPointInside(mLocalShapeToWorld * Vector3(2.2, 3.9, -2.2)));
            test(!mCylinderBody->testPointInside(mLocalShapeToWorld * Vector3(-2.2, 3.9, -2.2)));
            test(!mCylinderBody->testPointInside(mLocalShapeToWorld * Vector3(-2.2, 3.9, 2.2)));
            test(!mCylinderBody->testPointInside(mLocalShapeToWorld * Vector3(3.1, -3.9, 0)));
            test(!mCylinderBody->testPointInside(mLocalShapeToWorld * Vector3(-3.1, -3.9, 0)));
            test(!mCylinderBody->testPointInside(mLocalShapeToWorld * Vector3(0, -3.9, 3.1)));
            test(!mCylinderBody->testPointInside(mLocalShapeToWorld * Vector3(0, -3.9, -3.1)));
            test(!mCylinderBody->testPointInside(mLocalShapeToWorld * Vector3(2.2, -3.9, 2.2)));
            test(!mCylinderBody->testPointInside(mLocalShapeToWorld * Vector3(2.2, -3.9, -2.2)));
            test(!mCylinderBody->testPointInside(mLocalShapeToWorld * Vector3(-2.2, -3.9, -2.2)));
            test(!mCylinderBody->testPointInside(mLocalShapeToWorld * Vector3(-2.2, -3.9, 2.2)));

            // Tests with ProxyCylinderShape
            test(mCylinderShape->testPointInside(mLocalShapeToWorld * Vector3(0, 0, 0)));
            test(mCylinderShape->testPointInside(mLocalShapeToWorld * Vector3(0, 3.9, 0)));
            test(mCylinderShape->testPointInside(mLocalShapeToWorld * Vector3(0, -3.9, 0)));
            test(mCylinderShape->testPointInside(mLocalShapeToWorld * Vector3(2.9, 0, 0)));
            test(mCylinderShape->testPointInside(mLocalShapeToWorld * Vector3(-2.9, 0, 0)));
            test(mCylinderShape->testPointInside(mLocalShapeToWorld * Vector3(0, 0, 2.9)));
            test(mCylinderShape->testPointInside(mLocalShapeToWorld * Vector3(0, 0, -2.9)));
            test(mCylinderShape->testPointInside(mLocalShapeToWorld * Vector3(1.7, 0, 1.7)));
            test(mCylinderShape->testPointInside(mLocalShapeToWorld * Vector3(1.7, 0, -1.7)));
            test(mCylinderShape->testPointInside(mLocalShapeToWorld * Vector3(-1.7, 0, -1.7)));
            test(mCylinderShape->testPointInside(mLocalShapeToWorld * Vector3(-1.7, 0, 1.7)));
            test(mCylinderShape->testPointInside(mLocalShapeToWorld * Vector3(2.9, 3.9, 0)));
            test(mCylinderShape->testPointInside(mLocalShapeToWorld * Vector3(-2.9, 3.9, 0)));
            test(mCylinderShape->testPointInside(mLocalShapeToWorld * Vector3(0, 3.9, 2.9)));
            test(mCylinderShape->testPointInside(mLocalShapeToWorld * Vector3(0, 3.9, -2.9)));
            test(mCylinderShape->testPointInside(mLocalShapeToWorld * Vector3(1.7, 3.9, 1.7)));
            test(mCylinderShape->testPointInside(mLocalShapeToWorld * Vector3(1.7, 3.9, -1.7)));
            test(mCylinderShape->testPointInside(mLocalShapeToWorld * Vector3(-1.7, 3.9, -1.7)));
            test(mCylinderShape->testPointInside(mLocalShapeToWorld * Vector3(-1.7, 3.9, 1.7)));
            test(mCylinderShape->testPointInside(mLocalShapeToWorld * Vector3(2.9, -3.9, 0)));
            test(mCylinderShape->testPointInside(mLocalShapeToWorld * Vector3(-2.9, -3.9, 0)));
            test(mCylinderShape->testPointInside(mLocalShapeToWorld * Vector3(0, -3.9, 2.9)));
            test(mCylinderShape->testPointInside(mLocalShapeToWorld * Vector3(0, -3.9, -2.9)));
            test(mCylinderShape->testPointInside(mLocalShapeToWorld * Vector3(1.7, -3.9, 1.7)));
            test(mCylinderShape->testPointInside(mLocalShapeToWorld * Vector3(1.7, -3.9, -1.7)));
            test(mCylinderShape->testPointInside(mLocalShapeToWorld * Vector3(-1.7, -3.9, -1.7)));
            test(mCylinderShape->testPointInside(mLocalShapeToWorld * Vector3(-1.7, -3.9, 1.7)));

            test(!mCylinderShape->testPointInside(mLocalShapeToWorld * Vector3(0, 4.1, 0)));
            test(!!mCylinderShape->testPointInside(mLocalShapeToWorld * Vector3(0, -4.1, 0)));
            test(!mCylinderShape->testPointInside(mLocalShapeToWorld * Vector3(3.1, 0, 0)));
            test(!mCylinderShape->testPointInside(mLocalShapeToWorld * Vector3(-3.1, 0, 0)));
            test(!mCylinderShape->testPointInside(mLocalShapeToWorld * Vector3(0, 0, 3.1)));
            test(!mCylinderShape->testPointInside(mLocalShapeToWorld * Vector3(0, 0, -3.1)));
            test(!mCylinderShape->testPointInside(mLocalShapeToWorld * Vector3(2.2, 0, 2.2)));
            test(!mCylinderShape->testPointInside(mLocalShapeToWorld * Vector3(2.2, 0, -2.2)));
            test(!mCylinderShape->testPointInside(mLocalShapeToWorld * Vector3(-2.2, 0, -2.2)));
            test(!mCylinderShape->testPointInside(mLocalShapeToWorld * Vector3(-2.2, 0, 1.7)));
            test(!mCylinderShape->testPointInside(mLocalShapeToWorld * Vector3(3.1, 3.9, 0)));
            test(!mCylinderShape->testPointInside(mLocalShapeToWorld * Vector3(-3.1, 3.9, 0)));
            test(!mCylinderShape->testPointInside(mLocalShapeToWorld * Vector3(0, 3.9, 3.1)));
            test(!mCylinderShape->testPointInside(mLocalShapeToWorld * Vector3(0, 3.9, -3.1)));
            test(!mCylinderShape->testPointInside(mLocalShapeToWorld * Vector3(2.2, 3.9, 2.2)));
            test(!mCylinderShape->testPointInside(mLocalShapeToWorld * Vector3(2.2, 3.9, -2.2)));
            test(!mCylinderShape->testPointInside(mLocalShapeToWorld * Vector3(-2.2, 3.9, -2.2)));
            test(!mCylinderShape->testPointInside(mLocalShapeToWorld * Vector3(-2.2, 3.9, 2.2)));
            test(!mCylinderShape->testPointInside(mLocalShapeToWorld * Vector3(3.1, -3.9, 0)));
            test(!mCylinderShape->testPointInside(mLocalShapeToWorld * Vector3(-3.1, -3.9, 0)));
            test(!mCylinderShape->testPointInside(mLocalShapeToWorld * Vector3(0, -3.9, 3.1)));
            test(!mCylinderShape->testPointInside(mLocalShapeToWorld * Vector3(0, -3.9, -3.1)));
            test(!mCylinderShape->testPointInside(mLocalShapeToWorld * Vector3(2.2, -3.9, 2.2)));
            test(!mCylinderShape->testPointInside(mLocalShapeToWorld * Vector3(2.2, -3.9, -2.2)));
            test(!mCylinderShape->testPointInside(mLocalShapeToWorld * Vector3(-2.2, -3.9, -2.2)));
            test(!mCylinderShape->testPointInside(mLocalShapeToWorld * Vector3(-2.2, -3.9, 2.2)));
        }

        /// Test the CollisionBody::testPointInside() method
        void testCompound() {

            // Points on the cylinder
            test(mCompoundBody->testPointInside(mLocalShapeToWorld * Vector3(0, 0, 0)));
            test(mCompoundBody->testPointInside(mLocalShapeToWorld * Vector3(0, 3.9, 0)));
            test(mCompoundBody->testPointInside(mLocalShapeToWorld * Vector3(0, -3.9, 0)));
            test(mCompoundBody->testPointInside(mLocalShapeToWorld * Vector3(2.9, 0, 0)));
            test(mCompoundBody->testPointInside(mLocalShapeToWorld * Vector3(-2.9, 0, 0)));
            test(mCompoundBody->testPointInside(mLocalShapeToWorld * Vector3(0, 0, 2.9)));
            test(mCompoundBody->testPointInside(mLocalShapeToWorld * Vector3(0, 0, -2.9)));
            test(mCompoundBody->testPointInside(mLocalShapeToWorld * Vector3(1.7, 0, 1.7)));
            test(mCompoundBody->testPointInside(mLocalShapeToWorld * Vector3(1.7, 0, -1.7)));
            test(mCompoundBody->testPointInside(mLocalShapeToWorld * Vector3(-1.7, 0, -1.7)));
            test(mCompoundBody->testPointInside(mLocalShapeToWorld * Vector3(-1.7, 0, 1.7)));
            test(mCompoundBody->testPointInside(mLocalShapeToWorld * Vector3(2.9, 3.9, 0)));
            test(mCompoundBody->testPointInside(mLocalShapeToWorld * Vector3(-2.9, 3.9, 0)));
            test(mCompoundBody->testPointInside(mLocalShapeToWorld * Vector3(0, 3.9, 2.9)));
            test(mCompoundBody->testPointInside(mLocalShapeToWorld * Vector3(0, 3.9, -2.9)));
            test(mCompoundBody->testPointInside(mLocalShapeToWorld * Vector3(1.7, 3.9, 1.7)));
            test(mCompoundBody->testPointInside(mLocalShapeToWorld * Vector3(1.7, 3.9, -1.7)));
            test(mCompoundBody->testPointInside(mLocalShapeToWorld * Vector3(-1.7, 3.9, -1.7)));
            test(mCompoundBody->testPointInside(mLocalShapeToWorld * Vector3(-1.7, 3.9, 1.7)));
            test(mCompoundBody->testPointInside(mLocalShapeToWorld * Vector3(2.9, -3.9, 0)));
            test(mCompoundBody->testPointInside(mLocalShapeToWorld * Vector3(-2.9, -3.9, 0)));
            test(mCompoundBody->testPointInside(mLocalShapeToWorld * Vector3(0, -3.9, 2.9)));
            test(mCompoundBody->testPointInside(mLocalShapeToWorld * Vector3(0, -3.9, -2.9)));
            test(mCompoundBody->testPointInside(mLocalShapeToWorld * Vector3(1.7, -3.9, 1.7)));
            test(mCompoundBody->testPointInside(mLocalShapeToWorld * Vector3(1.7, -3.9, -1.7)));
            test(mCompoundBody->testPointInside(mLocalShapeToWorld * Vector3(-1.7, -3.9, -1.7)));
            test(mCompoundBody->testPointInside(mLocalShapeToWorld * Vector3(-1.7, -3.9, 1.7)));

            // Points on the sphere
            test(mCompoundBody->testPointInside(mLocalShape2ToWorld * Vector3(0, 0, 0)));
            test(mCompoundBody->testPointInside(mLocalShape2ToWorld * Vector3(2.9, 0, 0)));
            test(mCompoundBody->testPointInside(mLocalShape2ToWorld * Vector3(-2.9, 0, 0)));
            test(mCompoundBody->testPointInside(mLocalShape2ToWorld * Vector3(0, 2.9, 0)));
            test(mCompoundBody->testPointInside(mLocalShape2ToWorld * Vector3(0, -2.9, 0)));
            test(mCompoundBody->testPointInside(mLocalShape2ToWorld * Vector3(0, 0, 2.9)));
            test(mCompoundBody->testPointInside(mLocalShape2ToWorld * Vector3(0, 0, 2.9)));
            test(mCompoundBody->testPointInside(mLocalShape2ToWorld * Vector3(-1, -2, -1.5)));
            test(mCompoundBody->testPointInside(mLocalShape2ToWorld * Vector3(-1, 2, -1.5)));
            test(mCompoundBody->testPointInside(mLocalShape2ToWorld * Vector3(1, -2, 1.5)));
        }
 };

}

#endif
