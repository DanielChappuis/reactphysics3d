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

// Class TestPointInside
/**
 * Unit test for the CollisionBody::testPointInside() method.
 */
class TestRaycast : public Test {

    private :

        // ---------- Atributes ---------- //

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

    public :

        // ---------- Methods ---------- //

        /// Constructor
        TestRaycast() {

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
            convexMeshShape.addVertex(Vector3(-2, -3, 4));
            convexMeshShape.addVertex(Vector3(2, -3, 4));
            convexMeshShape.addVertex(Vector3(-2, -3, 4));
            convexMeshShape.addVertex(Vector3(2, -3, -4));
            convexMeshShape.addVertex(Vector3(-2, 3, 4));
            convexMeshShape.addVertex(Vector3(2, 3, 4));
            convexMeshShape.addVertex(Vector3(-2, 3, -4));
            convexMeshShape.addVertex(Vector3(2, 3, -4));
            mConvexMeshShape = mConvexMeshBody->addCollisionShape(convexMeshShape, mShapeTransform);

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

            CylinderShape cylinderShape(2, 5, 0);
            mCylinderShape = mCylinderBody->addCollisionShape(cylinderShape, mShapeTransform);

            // Compound shape is a cylinder and a sphere
            Vector3 positionShape2(Vector3(4, 2, -3));
            Quaternion orientationShape2(-3 *PI / 8, 1.5 * PI/ 3, PI / 13);
            Transform shapeTransform2(positionShape2, orientationShape2);
            mLocalShape2ToWorld = mBodyTransform * shapeTransform2;
            mCompoundBody->addCollisionShape(cylinderShape, mShapeTransform);
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

        /// Test the ProxySphereShape::raycast(), CollisionBody::raycast() and
        /// CollisionWorld::raycast() methods.
        void testSphere() {

            // ----- Test feedback data ----- //
            Vector3 origin = mLocalShapeToWorld * Vector3(-8 , 0, 0);
            const Matrix3x3 mLocalToWorldMatrix = mLocalShapeToWorld.getOrientation().getMatrix();
            Vector3 direction = mLocalToWorldMatrix * Vector3(5, 0, 0);
            Ray ray(origin, direction);
            Vector3 hitPoint = mLocalShapeToWorld * Vector3(-3, 0, 0);

            // CollisionWorld::raycast()
            RaycastInfo raycastInfo;
            test(mWorld->raycast(ray, raycastInfo));
            test(raycastInfo.body == mSphereBody);
            test(raycastInfo.proxyShape == mSphereShape);
            test(approxEqual(raycastInfo.distance, 6));
            test(approxEqual(raycastInfo.worldPoint.x, hitPoint.x));
            test(approxEqual(raycastInfo.worldPoint.y, hitPoint.y));
            test(approxEqual(raycastInfo.worldPoint.z, hitPoint.z));

            // CollisionBody::raycast()
            RaycastInfo raycastInfo2;
            test(mSphereBody->raycast(ray, raycastInfo2));
            test(raycastInfo2.body == mSphereBody);
            test(raycastInfo2.proxyShape == mSphereShape);
            test(approxEqual(raycastInfo2.distance, 6));
            test(approxEqual(raycastInfo2.worldPoint.x, hitPoint.x));
            test(approxEqual(raycastInfo2.worldPoint.y, hitPoint.y));
            test(approxEqual(raycastInfo2.worldPoint.z, hitPoint.z));

            // ProxyCollisionShape::raycast()
            RaycastInfo raycastInfo3;
            test(mSphereShape->raycast(ray, raycastInfo3));
            test(raycastInfo3.body == mSphereBody);
            test(raycastInfo3.proxyShape == mSphereShape);
            test(approxEqual(raycastInfo3.distance, 6));
            test(approxEqual(raycastInfo3.worldPoint.x, hitPoint.x));
            test(approxEqual(raycastInfo3.worldPoint.y, hitPoint.y));
            test(approxEqual(raycastInfo3.worldPoint.z, hitPoint.z));

            Ray ray1(mLocalShapeToWorld * Vector3(0, 0, 0), mLocalToWorldMatrix * Vector3(5, 7, -1));
            Ray ray2(mLocalShapeToWorld * Vector3(5, 11, 7), mLocalToWorldMatrix * Vector3(4, 6, 7));
            Ray ray3(mLocalShapeToWorld * Vector3(1, 2, 2), mLocalToWorldMatrix * Vector3(-4, 0, 7));
            Ray ray4(mLocalShapeToWorld * Vector3(10, 10, 10), mLocalToWorldMatrix * Vector3(4, 6, 7));
            Ray ray5(mLocalShapeToWorld * Vector3(4, 1, -5), mLocalToWorldMatrix * Vector3(-3, 0, 0));
            Ray ray6(mLocalShapeToWorld * Vector3(4, 4, 1), mLocalToWorldMatrix * Vector3(0, -2, 0));
            Ray ray7(mLocalShapeToWorld * Vector3(1, -4, 5), mLocalToWorldMatrix * Vector3(0, 0, -2));
            Ray ray8(mLocalShapeToWorld * Vector3(-4, 4, 0), mLocalToWorldMatrix * Vector3(1, 0, 0));
            Ray ray9(mLocalShapeToWorld * Vector3(0, -4, -4), mLocalToWorldMatrix * Vector3(0, 5, 0));
            Ray ray10(mLocalShapeToWorld * Vector3(-4, 0, -6), mLocalToWorldMatrix * Vector3(0, 0, 8));
            Ray ray11(mLocalShapeToWorld * Vector3(4, 1, 2), mLocalToWorldMatrix * Vector3(-4, 0, 0));
            Ray ray12(mLocalShapeToWorld * Vector3(1, 4, -1), mLocalToWorldMatrix * Vector3(0, -3, 0));
            Ray ray13(mLocalShapeToWorld * Vector3(-1, 2, 5), mLocalToWorldMatrix * Vector3(0, 0, -8));
            Ray ray14(mLocalShapeToWorld * Vector3(-5, 2, -2), mLocalToWorldMatrix * Vector3(4, 0, 0));
            Ray ray15(mLocalShapeToWorld * Vector3(0, -4, 1), mLocalToWorldMatrix * Vector3(0, 3, 0));
            Ray ray16(mLocalShapeToWorld * Vector3(-1, 2, -11), mLocalToWorldMatrix * Vector3(0, 0, 8));

            // ----- Test raycast miss ----- //
            test(!mSphereBody->raycast(ray1, raycastInfo3));
            test(!mSphereShape->raycast(ray1, raycastInfo3));
            test(!mWorld->raycast(ray1, raycastInfo3));
            test(!mWorld->raycast(ray1, raycastInfo3, 1));
            test(!mWorld->raycast(ray1, raycastInfo3, 100));
            test(!mWorld->raycast(ray1));

            test(!mSphereBody->raycast(ray2, raycastInfo3));
            test(!mSphereShape->raycast(ray2, raycastInfo3));
            test(!mWorld->raycast(ray2, raycastInfo3));
            test(!mWorld->raycast(ray2));

            test(!mSphereBody->raycast(ray3, raycastInfo3));
            test(!mSphereShape->raycast(ray3, raycastInfo3));
            test(!mWorld->raycast(ray3, raycastInfo3));
            test(!mWorld->raycast(ray3));

            test(!mSphereBody->raycast(ray4, raycastInfo3));
            test(!mSphereShape->raycast(ray4, raycastInfo3));
            test(!mWorld->raycast(ray4, raycastInfo3));
            test(!mWorld->raycast(ray4));

            test(!mSphereBody->raycast(ray5, raycastInfo3));
            test(!mSphereShape->raycast(ray5, raycastInfo3));
            test(!mWorld->raycast(ray5, raycastInfo3));
            test(!mWorld->raycast(ray5));

            test(!mSphereBody->raycast(ray6, raycastInfo3));
            test(!mSphereShape->raycast(ray6, raycastInfo3));
            test(!mWorld->raycast(ray6, raycastInfo3));
            test(!mWorld->raycast(ray6));

            test(!mSphereBody->raycast(ray7, raycastInfo3));
            test(!mSphereShape->raycast(ray7, raycastInfo3));
            test(!mWorld->raycast(ray7, raycastInfo3));
            test(!mWorld->raycast(ray7));

            test(!mSphereBody->raycast(ray8, raycastInfo3));
            test(!mSphereShape->raycast(ray8, raycastInfo3));
            test(!mWorld->raycast(ray8, raycastInfo3));
            test(!mWorld->raycast(ray8));

            test(!mSphereBody->raycast(ray9, raycastInfo3));
            test(!mSphereShape->raycast(ray9, raycastInfo3));
            test(!mWorld->raycast(ray9, raycastInfo3));
            test(!mWorld->raycast(ray9));

            test(!mSphereBody->raycast(ray10, raycastInfo3));
            test(!mSphereShape->raycast(ray10, raycastInfo3));
            test(!mWorld->raycast(ray10, raycastInfo3));
            test(!mWorld->raycast(ray10));

            test(!mWorld->raycast(ray11, raycastInfo3, 0.5));
            test(!mWorld->raycast(ray12, raycastInfo3, 0.5));
            test(!mWorld->raycast(ray13, raycastInfo3, 0.5));
            test(!mWorld->raycast(ray14, raycastInfo3, 0.5));
            test(!mWorld->raycast(ray15, raycastInfo3, 0.5));
            test(!mWorld->raycast(ray16, raycastInfo3, 2));

            // ----- Test raycast hits ----- //
            test(mSphereBody->raycast(ray11, raycastInfo3));
            test(mSphereShape->raycast(ray11, raycastInfo3));
            test(mWorld->raycast(ray11, raycastInfo3));
            test(mWorld->raycast(ray11, raycastInfo3, 2));
            test(mWorld->raycast(ray11));

            test(mSphereBody->raycast(ray12, raycastInfo3));
            test(mSphereShape->raycast(ray12, raycastInfo3));
            test(mWorld->raycast(ray12, raycastInfo3));
            test(mWorld->raycast(ray12, raycastInfo3, 2));
            test(mWorld->raycast(ray12));

            test(mSphereBody->raycast(ray13, raycastInfo3));
            test(mSphereShape->raycast(ray13, raycastInfo3));
            test(mWorld->raycast(ray13, raycastInfo3));
            test(mWorld->raycast(ray13, raycastInfo3, 2));
            test(mWorld->raycast(ray13));

            test(mSphereBody->raycast(ray14, raycastInfo3));
            test(mSphereShape->raycast(ray14, raycastInfo3));
            test(mWorld->raycast(ray14, raycastInfo3));
            test(mWorld->raycast(ray14, raycastInfo3, 2));
            test(mWorld->raycast(ray14));

            test(mSphereBody->raycast(ray15, raycastInfo3));
            test(mSphereShape->raycast(ray15, raycastInfo3));
            test(mWorld->raycast(ray15, raycastInfo3));
            test(mWorld->raycast(ray15, raycastInfo3, 2));
            test(mWorld->raycast(ray15));

            test(mSphereBody->raycast(ray16, raycastInfo3));
            test(mSphereShape->raycast(ray16, raycastInfo3));
            test(mWorld->raycast(ray16, raycastInfo3));
            test(mWorld->raycast(ray16, raycastInfo3, 4));
            test(mWorld->raycast(ray16));
        }

        /// Test the ProxyCapsuleShape::raycast(), CollisionBody::raycast() and
        /// CollisionWorld::raycast() methods.
        void testCapsule() {

            // ----- Test feedback data ----- //
            Vector3 origin = mLocalShapeToWorld * Vector3(0 , 10, 0);
            const Matrix3x3 mLocalToWorldMatrix = mLocalShapeToWorld.getOrientation().getMatrix();
            Vector3 direction = mLocalToWorldMatrix * Vector3(0, -3, 0);
            Ray ray(origin, direction);
            Vector3 hitPoint = mLocalShapeToWorld * Vector3(0, 7, 0);

            // CollisionWorld::raycast()
            RaycastInfo raycastInfo;
            test(mWorld->raycast(ray, raycastInfo));
            test(raycastInfo.body == mCapsuleBody);
            test(raycastInfo.proxyShape == mCapsuleShape);
            test(approxEqual(raycastInfo.distance, 6));
            test(approxEqual(raycastInfo.worldPoint.x, hitPoint.x));
            test(approxEqual(raycastInfo.worldPoint.y, hitPoint.y));
            test(approxEqual(raycastInfo.worldPoint.z, hitPoint.z));

            // CollisionBody::raycast()
            RaycastInfo raycastInfo2;
            test(mCapsuleBody->raycast(ray, raycastInfo2));
            test(raycastInfo2.body == mCapsuleBody);
            test(raycastInfo2.proxyShape == mCapsuleShape);
            test(approxEqual(raycastInfo2.distance, 6));
            test(approxEqual(raycastInfo2.worldPoint.x, hitPoint.x));
            test(approxEqual(raycastInfo2.worldPoint.y, hitPoint.y));
            test(approxEqual(raycastInfo2.worldPoint.z, hitPoint.z));

            // ProxyCollisionShape::raycast()
            RaycastInfo raycastInfo3;
            test(mCapsuleShape->raycast(ray, raycastInfo3));
            test(raycastInfo3.body == mCapsuleBody);
            test(raycastInfo3.proxyShape == mCapsuleShape);
            test(approxEqual(raycastInfo3.distance, 6));
            test(approxEqual(raycastInfo3.worldPoint.x, hitPoint.x));
            test(approxEqual(raycastInfo3.worldPoint.y, hitPoint.y));
            test(approxEqual(raycastInfo3.worldPoint.z, hitPoint.z));

            Ray ray1(mLocalShapeToWorld * Vector3(0, 0, 0), mLocalToWorldMatrix * Vector3(5, 7, -1));
            Ray ray2(mLocalShapeToWorld * Vector3(5, 11, 7), mLocalToWorldMatrix * Vector3(4, 6, 7));
            Ray ray3(mLocalShapeToWorld * Vector3(1, 3, -1), mLocalToWorldMatrix * Vector3(-4, 0, 7));
            Ray ray4(mLocalShapeToWorld * Vector3(10, 10, 10), mLocalToWorldMatrix * Vector3(4, 6, 7));
            Ray ray5(mLocalShapeToWorld * Vector3(4, 1, -5), mLocalToWorldMatrix * Vector3(-3, 0, 0));
            Ray ray6(mLocalShapeToWorld * Vector3(4, 9, 1), mLocalToWorldMatrix * Vector3(0, -2, 0));
            Ray ray7(mLocalShapeToWorld * Vector3(1, -9, 5), mLocalToWorldMatrix * Vector3(0, 0, -2));
            Ray ray8(mLocalShapeToWorld * Vector3(-4, 9, 0), mLocalToWorldMatrix * Vector3(1, 0, 0));
            Ray ray9(mLocalShapeToWorld * Vector3(0, -9, -4), mLocalToWorldMatrix * Vector3(0, 5, 0));
            Ray ray10(mLocalShapeToWorld * Vector3(-4, 0, -6), mLocalToWorldMatrix * Vector3(0, 0, 8));
            Ray ray11(mLocalShapeToWorld * Vector3(4, 1, 2), mLocalToWorldMatrix * Vector3(-4, 0, 0));
            Ray ray12(mLocalShapeToWorld * Vector3(1, 9, -1), mLocalToWorldMatrix * Vector3(0, -3, 0));
            Ray ray13(mLocalShapeToWorld * Vector3(-1, 2, 3), mLocalToWorldMatrix * Vector3(0, 0, -8));
            Ray ray14(mLocalShapeToWorld * Vector3(-3, 2, -2), mLocalToWorldMatrix * Vector3(4, 0, 0));
            Ray ray15(mLocalShapeToWorld * Vector3(0, -9, 1), mLocalToWorldMatrix * Vector3(0, 3, 0));
            Ray ray16(mLocalShapeToWorld * Vector3(-1, 2, -7), mLocalToWorldMatrix * Vector3(0, 0, 8));

            // ----- Test raycast miss ----- //
            test(!mCapsuleBody->raycast(ray1, raycastInfo3));
            test(!mCapsuleShape->raycast(ray1, raycastInfo3));
            test(!mWorld->raycast(ray1, raycastInfo3));
            test(!mWorld->raycast(ray1, raycastInfo3, 1));
            test(!mWorld->raycast(ray1, raycastInfo3, 100));
            test(!mWorld->raycast(ray1));

            test(!mCapsuleBody->raycast(ray2, raycastInfo3));
            test(!mCapsuleShape->raycast(ray2, raycastInfo3));
            test(!mWorld->raycast(ray2, raycastInfo3));
            test(!mWorld->raycast(ray2));

            test(!mCapsuleBody->raycast(ray3, raycastInfo3));
            test(!mCapsuleShape->raycast(ray3, raycastInfo3));
            test(!mWorld->raycast(ray3, raycastInfo3));
            test(!mWorld->raycast(ray3));

            test(!mCapsuleBody->raycast(ray4, raycastInfo3));
            test(!mCapsuleShape->raycast(ray4, raycastInfo3));
            test(!mWorld->raycast(ray4, raycastInfo3));
            test(!mWorld->raycast(ray4));

            test(!mCapsuleBody->raycast(ray5, raycastInfo3));
            test(!mCapsuleShape->raycast(ray5, raycastInfo3));
            test(!mWorld->raycast(ray5, raycastInfo3));
            test(!mWorld->raycast(ray5));

            test(!mCapsuleBody->raycast(ray6, raycastInfo3));
            test(!mCapsuleShape->raycast(ray6, raycastInfo3));
            test(!mWorld->raycast(ray6, raycastInfo3));
            test(!mWorld->raycast(ray6));

            test(!mCapsuleBody->raycast(ray7, raycastInfo3));
            test(!mCapsuleShape->raycast(ray7, raycastInfo3));
            test(!mWorld->raycast(ray7, raycastInfo3));
            test(!mWorld->raycast(ray7));

            test(!mCapsuleBody->raycast(ray8, raycastInfo3));
            test(!mCapsuleShape->raycast(ray8, raycastInfo3));
            test(!mWorld->raycast(ray8, raycastInfo3));
            test(!mWorld->raycast(ray8));

            test(!mCapsuleBody->raycast(ray9, raycastInfo3));
            test(!mCapsuleShape->raycast(ray9, raycastInfo3));
            test(!mWorld->raycast(ray9, raycastInfo3));
            test(!mWorld->raycast(ray9));

            test(!mCapsuleBody->raycast(ray10, raycastInfo3));
            test(!mCapsuleShape->raycast(ray10, raycastInfo3));
            test(!mWorld->raycast(ray10, raycastInfo3));
            test(!mWorld->raycast(ray10));

            test(!mWorld->raycast(ray11, raycastInfo3, 0.5));
            test(!mWorld->raycast(ray12, raycastInfo3, 0.5));
            test(!mWorld->raycast(ray13, raycastInfo3, 0.5));
            test(!mWorld->raycast(ray14, raycastInfo3, 0.5));
            test(!mWorld->raycast(ray15, raycastInfo3, 0.5));
            test(!mWorld->raycast(ray16, raycastInfo3, 2));

            // ----- Test raycast hits ----- //
            test(mCapsuleBody->raycast(ray11, raycastInfo3));
            test(mCapsuleShape->raycast(ray11, raycastInfo3));
            test(mWorld->raycast(ray11, raycastInfo3));
            test(mWorld->raycast(ray11, raycastInfo3, 2));
            test(mWorld->raycast(ray11));

            test(mCapsuleBody->raycast(ray12, raycastInfo3));
            test(mCapsuleShape->raycast(ray12, raycastInfo3));
            test(mWorld->raycast(ray12, raycastInfo3));
            test(mWorld->raycast(ray12, raycastInfo3, 2));
            test(mWorld->raycast(ray12));

            test(mCapsuleBody->raycast(ray13, raycastInfo3));
            test(mCapsuleShape->raycast(ray13, raycastInfo3));
            test(mWorld->raycast(ray13, raycastInfo3));
            test(mWorld->raycast(ray13, raycastInfo3, 2));
            test(mWorld->raycast(ray13));

            test(mCapsuleBody->raycast(ray14, raycastInfo3));
            test(mCapsuleShape->raycast(ray14, raycastInfo3));
            test(mWorld->raycast(ray14, raycastInfo3));
            test(mWorld->raycast(ray14, raycastInfo3, 2));
            test(mWorld->raycast(ray14));

            test(mCapsuleBody->raycast(ray15, raycastInfo3));
            test(mCapsuleShape->raycast(ray15, raycastInfo3));
            test(mWorld->raycast(ray15, raycastInfo3));
            test(mWorld->raycast(ray15, raycastInfo3, 2));
            test(mWorld->raycast(ray15));

            test(mCapsuleBody->raycast(ray16, raycastInfo3));
            test(mCapsuleShape->raycast(ray16, raycastInfo3));
            test(mWorld->raycast(ray16, raycastInfo3));
            test(mWorld->raycast(ray16, raycastInfo3, 4));
            test(mWorld->raycast(ray16));
        }

        /// Test the ProxyConeShape::raycast(), CollisionBody::raycast() and
        /// CollisionWorld::raycast() methods.
        void testCone() {

            // ----- Test feedback data ----- //
            Vector3 origin = mLocalShapeToWorld * Vector3(0 , 0, 3);
            const Matrix3x3 mLocalToWorldMatrix = mLocalShapeToWorld.getOrientation().getMatrix();
            Vector3 direction = mLocalToWorldMatrix * Vector3(0, 0, -8);
            Ray ray(origin, direction);
            Vector3 hitPoint = mLocalShapeToWorld * Vector3(0, 0, 1);

            // CollisionWorld::raycast()
            RaycastInfo raycastInfo;
            test(mWorld->raycast(ray, raycastInfo));
            test(raycastInfo.body == mConeBody);
            test(raycastInfo.proxyShape == mConeShape);
            test(approxEqual(raycastInfo.distance, 6));
            test(approxEqual(raycastInfo.worldPoint.x, hitPoint.x));
            test(approxEqual(raycastInfo.worldPoint.y, hitPoint.y));
            test(approxEqual(raycastInfo.worldPoint.z, hitPoint.z));

            // CollisionBody::raycast()
            RaycastInfo raycastInfo2;
            test(mConeBody->raycast(ray, raycastInfo2));
            test(raycastInfo2.body == mConeBody);
            test(raycastInfo2.proxyShape == mConeShape);
            test(approxEqual(raycastInfo2.distance, 6));
            test(approxEqual(raycastInfo2.worldPoint.x, hitPoint.x));
            test(approxEqual(raycastInfo2.worldPoint.y, hitPoint.y));
            test(approxEqual(raycastInfo2.worldPoint.z, hitPoint.z));

            // ProxyCollisionShape::raycast()
            RaycastInfo raycastInfo3;
            test(mConeShape->raycast(ray, raycastInfo3));
            test(raycastInfo3.body == mConeBody);
            test(raycastInfo3.proxyShape == mConeShape);
            test(approxEqual(raycastInfo3.distance, 6));
            test(approxEqual(raycastInfo3.worldPoint.x, hitPoint.x));
            test(approxEqual(raycastInfo3.worldPoint.y, hitPoint.y));
            test(approxEqual(raycastInfo3.worldPoint.z, hitPoint.z));

            Ray ray1(mLocalShapeToWorld * Vector3(0, 0, 0), mLocalToWorldMatrix * Vector3(5, 7, -1));
            Ray ray2(mLocalShapeToWorld * Vector3(5, 11, 7), mLocalToWorldMatrix * Vector3(4, 6, 7));
            Ray ray3(mLocalShapeToWorld * Vector3(-1, -2, 1), mLocalToWorldMatrix * Vector3(-4, 0, 7));
            Ray ray4(mLocalShapeToWorld * Vector3(10, 10, 10), mLocalToWorldMatrix * Vector3(4, 6, 7));
            Ray ray5(mLocalShapeToWorld * Vector3(4, 1, -1), mLocalToWorldMatrix * Vector3(-3, 0, 0));
            Ray ray6(mLocalShapeToWorld * Vector3(3, 4, 1), mLocalToWorldMatrix * Vector3(0, -2, 0));
            Ray ray7(mLocalShapeToWorld * Vector3(1, -4, 3), mLocalToWorldMatrix * Vector3(0, 0, -2));
            Ray ray8(mLocalShapeToWorld * Vector3(-4, 4, 0), mLocalToWorldMatrix * Vector3(1, 0, 0));
            Ray ray9(mLocalShapeToWorld * Vector3(0, -4, -7), mLocalToWorldMatrix * Vector3(0, 5, 0));
            Ray ray10(mLocalShapeToWorld * Vector3(-3, -2, -6), mLocalToWorldMatrix * Vector3(0, 0, 8));
            Ray ray11(mLocalShapeToWorld * Vector3(3, -1, 0.5), mLocalToWorldMatrix * Vector3(-4, 0, 0));
            Ray ray12(mLocalShapeToWorld * Vector3(1, 4, -1), mLocalToWorldMatrix * Vector3(0, -3, 0));
            Ray ray13(mLocalShapeToWorld * Vector3(-1, -2, 3), mLocalToWorldMatrix * Vector3(0, 0, -8));
            Ray ray14(mLocalShapeToWorld * Vector3(-2, 0, 0.8), mLocalToWorldMatrix * Vector3(4, 0, 0));
            Ray ray15(mLocalShapeToWorld * Vector3(0, -4, 1), mLocalToWorldMatrix * Vector3(0, 3, 0));
            Ray ray16(mLocalShapeToWorld * Vector3(-0.9, 0, -4), mLocalToWorldMatrix * Vector3(0, 0, 8));

            // ----- Test raycast miss ----- //
            test(!mConeBody->raycast(ray1, raycastInfo3));
            test(!mConeShape->raycast(ray1, raycastInfo3));
            test(!mWorld->raycast(ray1, raycastInfo3));
            test(!mWorld->raycast(ray1, raycastInfo3, 1));
            test(!mWorld->raycast(ray1, raycastInfo3, 100));
            test(!mWorld->raycast(ray1));

            test(!mConeBody->raycast(ray2, raycastInfo3));
            test(!mConeShape->raycast(ray2, raycastInfo3));
            test(!mWorld->raycast(ray2, raycastInfo3));
            test(!mWorld->raycast(ray2));

            test(!mConeBody->raycast(ray3, raycastInfo3));
            test(!mConeShape->raycast(ray3, raycastInfo3));
            test(!mWorld->raycast(ray3, raycastInfo3));
            test(!mWorld->raycast(ray3));

            test(!mConeBody->raycast(ray4, raycastInfo3));
            test(!mConeShape->raycast(ray4, raycastInfo3));
            test(!mWorld->raycast(ray4, raycastInfo3));
            test(!mWorld->raycast(ray4));

            test(!mConeBody->raycast(ray5, raycastInfo3));
            test(!mConeShape->raycast(ray5, raycastInfo3));
            test(!mWorld->raycast(ray5, raycastInfo3));
            test(!mWorld->raycast(ray5));

            test(!mConeBody->raycast(ray6, raycastInfo3));
            test(!mConeShape->raycast(ray6, raycastInfo3));
            test(!mWorld->raycast(ray6, raycastInfo3));
            test(!mWorld->raycast(ray6));

            test(!mConeBody->raycast(ray7, raycastInfo3));
            test(!mConeShape->raycast(ray7, raycastInfo3));
            test(!mWorld->raycast(ray7, raycastInfo3));
            test(!mWorld->raycast(ray7));

            test(!mConeBody->raycast(ray8, raycastInfo3));
            test(!mConeShape->raycast(ray8, raycastInfo3));
            test(!mWorld->raycast(ray8, raycastInfo3));
            test(!mWorld->raycast(ray8));

            test(!mConeBody->raycast(ray9, raycastInfo3));
            test(!mConeShape->raycast(ray9, raycastInfo3));
            test(!mWorld->raycast(ray9, raycastInfo3));
            test(!mWorld->raycast(ray9));

            test(!mConeBody->raycast(ray10, raycastInfo3));
            test(!mConeShape->raycast(ray10, raycastInfo3));
            test(!mWorld->raycast(ray10, raycastInfo3));
            test(!mWorld->raycast(ray10));

            test(!mWorld->raycast(ray11, raycastInfo3, 0.5));
            test(!mWorld->raycast(ray12, raycastInfo3, 0.5));
            test(!mWorld->raycast(ray13, raycastInfo3, 0.5));
            test(!mWorld->raycast(ray14, raycastInfo3, 0.5));
            test(!mWorld->raycast(ray15, raycastInfo3, 0.5));
            test(!mWorld->raycast(ray16, raycastInfo3, 2));

            // ----- Test raycast hits ----- //
            test(mConeBody->raycast(ray11, raycastInfo3));
            test(mConeShape->raycast(ray11, raycastInfo3));
            test(mWorld->raycast(ray11, raycastInfo3));
            test(mWorld->raycast(ray11, raycastInfo3, 2));
            test(mWorld->raycast(ray11));

            test(mConeBody->raycast(ray12, raycastInfo3));
            test(mConeShape->raycast(ray12, raycastInfo3));
            test(mWorld->raycast(ray12, raycastInfo3));
            test(mWorld->raycast(ray12, raycastInfo3, 2));
            test(mWorld->raycast(ray12));

            test(mConeBody->raycast(ray13, raycastInfo3));
            test(mConeShape->raycast(ray13, raycastInfo3));
            test(mWorld->raycast(ray13, raycastInfo3));
            test(mWorld->raycast(ray13, raycastInfo3, 2));
            test(mWorld->raycast(ray13));

            test(mConeBody->raycast(ray14, raycastInfo3));
            test(mConeShape->raycast(ray14, raycastInfo3));
            test(mWorld->raycast(ray14, raycastInfo3));
            test(mWorld->raycast(ray14, raycastInfo3, 2));
            test(mWorld->raycast(ray14));

            test(mConeBody->raycast(ray15, raycastInfo3));
            test(mConeShape->raycast(ray15, raycastInfo3));
            test(mWorld->raycast(ray15, raycastInfo3));
            test(mWorld->raycast(ray15, raycastInfo3, 2));
            test(mWorld->raycast(ray15));

            test(mConeBody->raycast(ray16, raycastInfo3));
            test(mConeShape->raycast(ray16, raycastInfo3));
            test(mWorld->raycast(ray16, raycastInfo3));
            test(mWorld->raycast(ray16, raycastInfo3, 4));
            test(mWorld->raycast(ray16));
        }

        /// Test the ProxyConvexMeshShape::raycast(), CollisionBody::raycast() and
        /// CollisionWorld::raycast() methods.
        void testConvexMesh() {

            // ----- Test feedback data ----- //
            Vector3 origin = mLocalShapeToWorld * Vector3(1 , 2, 10);
            const Matrix3x3 mLocalToWorldMatrix = mLocalShapeToWorld.getOrientation().getMatrix();
            Vector3 direction = mLocalToWorldMatrix * Vector3(0, 0, -5);
            Ray ray(origin, direction);
            Vector3 hitPoint = mLocalShapeToWorld * Vector3(1, 2, 4);

            // CollisionWorld::raycast()
            RaycastInfo raycastInfo;
            test(mWorld->raycast(ray, raycastInfo));
            test(raycastInfo.body == mConvexMeshBody);
            test(raycastInfo.proxyShape == mConvexMeshShape);
            test(approxEqual(raycastInfo.distance, 6));
            test(approxEqual(raycastInfo.worldPoint.x, hitPoint.x));
            test(approxEqual(raycastInfo.worldPoint.y, hitPoint.y));
            test(approxEqual(raycastInfo.worldPoint.z, hitPoint.z));

            // CollisionBody::raycast()
            RaycastInfo raycastInfo2;
            test(mConvexMeshBody->raycast(ray, raycastInfo2));
            test(raycastInfo2.body == mConvexMeshBody);
            test(raycastInfo2.proxyShape == mConvexMeshShape);
            test(approxEqual(raycastInfo2.distance, 6));
            test(approxEqual(raycastInfo2.worldPoint.x, hitPoint.x));
            test(approxEqual(raycastInfo2.worldPoint.y, hitPoint.y));
            test(approxEqual(raycastInfo2.worldPoint.z, hitPoint.z));

            // ProxyCollisionShape::raycast()
            RaycastInfo raycastInfo3;
            test(mConvexMeshBodyEdgesInfo->raycast(ray, raycastInfo3));
            test(raycastInfo3.body == mConvexMeshBodyEdgesInfo);
            test(raycastInfo3.proxyShape == mConvexMeshShapeEdgesInfo);
            test(approxEqual(raycastInfo3.distance, 6));
            test(approxEqual(raycastInfo3.worldPoint.x, hitPoint.x));
            test(approxEqual(raycastInfo3.worldPoint.y, hitPoint.y));
            test(approxEqual(raycastInfo3.worldPoint.z, hitPoint.z));

            // ProxyCollisionShape::raycast()
            RaycastInfo raycastInfo4;
            test(mConvexMeshShape->raycast(ray, raycastInfo4));
            test(raycastInfo4.body == mConvexMeshBody);
            test(raycastInfo4.proxyShape == mConvexMeshShape);
            test(approxEqual(raycastInfo4.distance, 6));
            test(approxEqual(raycastInfo4.worldPoint.x, hitPoint.x));
            test(approxEqual(raycastInfo4.worldPoint.y, hitPoint.y));
            test(approxEqual(raycastInfo4.worldPoint.z, hitPoint.z));

            // ProxyCollisionShape::raycast()
            RaycastInfo raycastInfo5;
            test(mConvexMeshShapeEdgesInfo->raycast(ray, raycastInfo5));
            test(raycastInfo5.body == mConvexMeshBodyEdgesInfo);
            test(raycastInfo5.proxyShape == mConvexMeshShapeEdgesInfo);
            test(approxEqual(raycastInfo5.distance, 6));
            test(approxEqual(raycastInfo5.worldPoint.x, hitPoint.x));
            test(approxEqual(raycastInfo5.worldPoint.y, hitPoint.y));
            test(approxEqual(raycastInfo5.worldPoint.z, hitPoint.z));

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
            test(!mConvexMeshBody->raycast(ray1, raycastInfo3));
            test(!mConvexMeshBodyEdgesInfo->raycast(ray1, raycastInfo3));
            test(!mConvexMeshShape->raycast(ray1, raycastInfo3));
            test(!mConvexMeshShapeEdgesInfo->raycast(ray1, raycastInfo3));
            test(!mWorld->raycast(ray1, raycastInfo3));
            test(!mWorld->raycast(ray1, raycastInfo3, 1));
            test(!mWorld->raycast(ray1, raycastInfo3, 100));
            test(!mWorld->raycast(ray1));

            test(!mConvexMeshBody->raycast(ray2, raycastInfo3));
            test(!mConvexMeshBodyEdgesInfo->raycast(ray2, raycastInfo3));
            test(!mConvexMeshShape->raycast(ray2, raycastInfo3));
            test(!mConvexMeshShapeEdgesInfo->raycast(ray2, raycastInfo3));
            test(!mWorld->raycast(ray2, raycastInfo3));
            test(!mWorld->raycast(ray2));

            test(!mConvexMeshBody->raycast(ray3, raycastInfo3));
            test(!mConvexMeshBodyEdgesInfo->raycast(ray3, raycastInfo3));
            test(!mConvexMeshShape->raycast(ray3, raycastInfo3));
            test(!mConvexMeshShapeEdgesInfo->raycast(ray3, raycastInfo3));
            test(!mWorld->raycast(ray3, raycastInfo3));
            test(!mWorld->raycast(ray3));

            test(!mConvexMeshBody->raycast(ray4, raycastInfo3));
            test(!mConvexMeshBodyEdgesInfo->raycast(ray4, raycastInfo3));
            test(!mConvexMeshShape->raycast(ray4, raycastInfo3));
            test(!mConvexMeshShapeEdgesInfo->raycast(ray4, raycastInfo3));
            test(!mWorld->raycast(ray4, raycastInfo3));
            test(!mWorld->raycast(ray4));

            test(!mConvexMeshBody->raycast(ray5, raycastInfo3));
            test(!mConvexMeshBodyEdgesInfo->raycast(ray5, raycastInfo3));
            test(!mConvexMeshShape->raycast(ray5, raycastInfo3));
            test(!mConvexMeshShapeEdgesInfo->raycast(ray5, raycastInfo3));
            test(!mWorld->raycast(ray5, raycastInfo3));
            test(!mWorld->raycast(ray5));

            test(!mConvexMeshBody->raycast(ray6, raycastInfo3));
            test(!mConvexMeshBodyEdgesInfo->raycast(ray6, raycastInfo3));
            test(!mConvexMeshShape->raycast(ray6, raycastInfo3));
            test(!mConvexMeshShapeEdgesInfo->raycast(ray6, raycastInfo3));
            test(!mWorld->raycast(ray6, raycastInfo3));
            test(!mWorld->raycast(ray6));

            test(!mConvexMeshBody->raycast(ray7, raycastInfo3));
            test(!mConvexMeshBodyEdgesInfo->raycast(ray7, raycastInfo3));
            test(!mConvexMeshShape->raycast(ray7, raycastInfo3));
            test(!mConvexMeshShapeEdgesInfo->raycast(ray7, raycastInfo3));
            test(!mWorld->raycast(ray7, raycastInfo3));
            test(!mWorld->raycast(ray7));

            test(!mConvexMeshBody->raycast(ray8, raycastInfo3));
            test(!mConvexMeshBodyEdgesInfo->raycast(ray8, raycastInfo3));
            test(!mConvexMeshShape->raycast(ray8, raycastInfo3));
            test(!mConvexMeshShapeEdgesInfo->raycast(ray8, raycastInfo3));
            test(!mWorld->raycast(ray8, raycastInfo3));
            test(!mWorld->raycast(ray8));

            test(!mConvexMeshBody->raycast(ray9, raycastInfo3));
            test(!mConvexMeshBodyEdgesInfo->raycast(ray9, raycastInfo3));
            test(!mConvexMeshShape->raycast(ray9, raycastInfo3));
            test(!mConvexMeshShapeEdgesInfo->raycast(ray9, raycastInfo3));
            test(!mWorld->raycast(ray9, raycastInfo3));
            test(!mWorld->raycast(ray9));

            test(!mConvexMeshBody->raycast(ray10, raycastInfo3));
            test(!mConvexMeshBodyEdgesInfo->raycast(ray10, raycastInfo3));
            test(!mConvexMeshShape->raycast(ray10, raycastInfo3));
            test(!mConvexMeshShapeEdgesInfo->raycast(ray10, raycastInfo3));
            test(!mWorld->raycast(ray10, raycastInfo3));
            test(!mWorld->raycast(ray10));

            test(!mWorld->raycast(ray11, raycastInfo3, 0.5));
            test(!mWorld->raycast(ray12, raycastInfo3, 0.5));
            test(!mWorld->raycast(ray13, raycastInfo3, 0.5));
            test(!mWorld->raycast(ray14, raycastInfo3, 0.5));
            test(!mWorld->raycast(ray15, raycastInfo3, 0.5));
            test(!mWorld->raycast(ray16, raycastInfo3, 2));

            // ----- Test raycast hits ----- //
            test(mConvexMeshBody->raycast(ray11, raycastInfo3));
            test(mConvexMeshBodyEdgesInfo->raycast(ray11, raycastInfo3));
            test(mConvexMeshShape->raycast(ray11, raycastInfo3));
            test(mConvexMeshShapeEdgesInfo->raycast(ray11, raycastInfo3));
            test(mWorld->raycast(ray11, raycastInfo3));
            test(mWorld->raycast(ray11, raycastInfo3, 2));
            test(mWorld->raycast(ray11));

            test(mConvexMeshBody->raycast(ray12, raycastInfo3));
            test(mConvexMeshBodyEdgesInfo->raycast(ray12, raycastInfo3));
            test(mConvexMeshShape->raycast(ray12, raycastInfo3));
            test(mConvexMeshShapeEdgesInfo->raycast(ray12, raycastInfo3));
            test(mWorld->raycast(ray12, raycastInfo3));
            test(mWorld->raycast(ray12, raycastInfo3, 2));
            test(mWorld->raycast(ray12));

            test(mConvexMeshBody->raycast(ray13, raycastInfo3));
            test(mConvexMeshBodyEdgesInfo->raycast(ray13, raycastInfo3));
            test(mConvexMeshShape->raycast(ray13, raycastInfo3));
            test(mConvexMeshShapeEdgesInfo->raycast(ray13, raycastInfo3));
            test(mWorld->raycast(ray13, raycastInfo3));
            test(mWorld->raycast(ray13, raycastInfo3, 2));
            test(mWorld->raycast(ray13));

            test(mConvexMeshBody->raycast(ray14, raycastInfo3));
            test(mConvexMeshBodyEdgesInfo->raycast(ray14, raycastInfo3));
            test(mConvexMeshShape->raycast(ray14, raycastInfo3));
            test(mConvexMeshShapeEdgesInfo->raycast(ray14, raycastInfo3));
            test(mWorld->raycast(ray14, raycastInfo3));
            test(mWorld->raycast(ray14, raycastInfo3, 2));
            test(mWorld->raycast(ray14));

            test(mConvexMeshBody->raycast(ray15, raycastInfo3));
            test(mConvexMeshBodyEdgesInfo->raycast(ray15, raycastInfo3));
            test(mConvexMeshShape->raycast(ray15, raycastInfo3));
            test(mConvexMeshShapeEdgesInfo->raycast(ray15, raycastInfo3));
            test(mWorld->raycast(ray15, raycastInfo3));
            test(mWorld->raycast(ray15, raycastInfo3, 2));
            test(mWorld->raycast(ray15));

            test(mConvexMeshBody->raycast(ray16, raycastInfo3));
            test(mConvexMeshBodyEdgesInfo->raycast(ray16, raycastInfo3));
            test(mConvexMeshShape->raycast(ray16, raycastInfo3));
            test(mConvexMeshShapeEdgesInfo->raycast(ray16, raycastInfo3));
            test(mWorld->raycast(ray16, raycastInfo3));
            test(mWorld->raycast(ray16, raycastInfo3, 4));
            test(mWorld->raycast(ray16));
        }

        /// Test the ProxyCylinderShape::raycast(), CollisionBody::raycast() and
        /// CollisionWorld::raycast() methods.
        void testCylinder() {

            // ----- Test feedback data ----- //
            Vector3 origin = mLocalShapeToWorld * Vector3(0 , 10, 0);
            const Matrix3x3 mLocalToWorldMatrix = mLocalShapeToWorld.getOrientation().getMatrix();
            Vector3 direction = mLocalToWorldMatrix * Vector3(0, -3, 0);
            Ray ray(origin, direction);
            Vector3 hitPoint = mLocalShapeToWorld * Vector3(0, 7, 0);

            // CollisionWorld::raycast()
            RaycastInfo raycastInfo;
            test(mWorld->raycast(ray, raycastInfo));
            test(raycastInfo.body == mCylinderBody);
            test(raycastInfo.proxyShape == mCylinderShape);
            test(approxEqual(raycastInfo.distance, 6));
            test(approxEqual(raycastInfo.worldPoint.x, hitPoint.x));
            test(approxEqual(raycastInfo.worldPoint.y, hitPoint.y));
            test(approxEqual(raycastInfo.worldPoint.z, hitPoint.z));

            // CollisionBody::raycast()
            RaycastInfo raycastInfo2;
            test(mCylinderBody->raycast(ray, raycastInfo2));
            test(raycastInfo2.body == mCylinderBody);
            test(raycastInfo2.proxyShape == mCylinderShape);
            test(approxEqual(raycastInfo2.distance, 6));
            test(approxEqual(raycastInfo2.worldPoint.x, hitPoint.x));
            test(approxEqual(raycastInfo2.worldPoint.y, hitPoint.y));
            test(approxEqual(raycastInfo2.worldPoint.z, hitPoint.z));

            // ProxyCollisionShape::raycast()
            RaycastInfo raycastInfo3;
            test(mCylinderShape->raycast(ray, raycastInfo3));
            test(raycastInfo3.body == mCylinderBody);
            test(raycastInfo3.proxyShape == mCylinderShape);
            test(approxEqual(raycastInfo3.distance, 6));
            test(approxEqual(raycastInfo3.worldPoint.x, hitPoint.x));
            test(approxEqual(raycastInfo3.worldPoint.y, hitPoint.y));
            test(approxEqual(raycastInfo3.worldPoint.z, hitPoint.z));

            Ray ray1(mLocalShapeToWorld * Vector3(0, 0, 0), mLocalToWorldMatrix * Vector3(5, 7, -1));
            Ray ray2(mLocalShapeToWorld * Vector3(5, 11, 7), mLocalToWorldMatrix * Vector3(4, 6, 7));
            Ray ray3(mLocalShapeToWorld * Vector3(1, 3, -1), mLocalToWorldMatrix * Vector3(-4, 0, 7));
            Ray ray4(mLocalShapeToWorld * Vector3(10, 10, 10), mLocalToWorldMatrix * Vector3(4, 6, 7));
            Ray ray5(mLocalShapeToWorld * Vector3(4, 1, -5), mLocalToWorldMatrix * Vector3(-3, 0, 0));
            Ray ray6(mLocalShapeToWorld * Vector3(4, 9, 1), mLocalToWorldMatrix * Vector3(0, -2, 0));
            Ray ray7(mLocalShapeToWorld * Vector3(1, -9, 5), mLocalToWorldMatrix * Vector3(0, 0, -2));
            Ray ray8(mLocalShapeToWorld * Vector3(-4, 9, 0), mLocalToWorldMatrix * Vector3(1, 0, 0));
            Ray ray9(mLocalShapeToWorld * Vector3(0, -9, -4), mLocalToWorldMatrix * Vector3(0, 5, 0));
            Ray ray10(mLocalShapeToWorld * Vector3(-4, 0, -6), mLocalToWorldMatrix * Vector3(0, 0, 8));
            Ray ray11(mLocalShapeToWorld * Vector3(4, 1, 2), mLocalToWorldMatrix * Vector3(-4, 0, 0));
            Ray ray12(mLocalShapeToWorld * Vector3(1, 9, -1), mLocalToWorldMatrix * Vector3(0, -3, 0));
            Ray ray13(mLocalShapeToWorld * Vector3(-1, 2, 3), mLocalToWorldMatrix * Vector3(0, 0, -8));
            Ray ray14(mLocalShapeToWorld * Vector3(-3, 2, -2), mLocalToWorldMatrix * Vector3(4, 0, 0));
            Ray ray15(mLocalShapeToWorld * Vector3(0, -9, 1), mLocalToWorldMatrix * Vector3(0, 3, 0));
            Ray ray16(mLocalShapeToWorld * Vector3(-1, 2, -7), mLocalToWorldMatrix * Vector3(0, 0, 8));

            // ----- Test raycast miss ----- //
            test(!mCylinderBody->raycast(ray1, raycastInfo3));
            test(!mCylinderShape->raycast(ray1, raycastInfo3));
            test(!mWorld->raycast(ray1, raycastInfo3));
            test(!mWorld->raycast(ray1, raycastInfo3, 1));
            test(!mWorld->raycast(ray1, raycastInfo3, 100));
            test(!mWorld->raycast(ray1));

            test(!mCylinderBody->raycast(ray2, raycastInfo3));
            test(!mCylinderShape->raycast(ray2, raycastInfo3));
            test(!mWorld->raycast(ray2, raycastInfo3));
            test(!mWorld->raycast(ray2));

            test(!mCylinderBody->raycast(ray3, raycastInfo3));
            test(!mCylinderShape->raycast(ray3, raycastInfo3));
            test(!mWorld->raycast(ray3, raycastInfo3));
            test(!mWorld->raycast(ray3));

            test(!mCylinderBody->raycast(ray4, raycastInfo3));
            test(!mCylinderShape->raycast(ray4, raycastInfo3));
            test(!mWorld->raycast(ray4, raycastInfo3));
            test(!mWorld->raycast(ray4));

            test(!mCylinderBody->raycast(ray5, raycastInfo3));
            test(!mCylinderShape->raycast(ray5, raycastInfo3));
            test(!mWorld->raycast(ray5, raycastInfo3));
            test(!mWorld->raycast(ray5));

            test(!mCylinderBody->raycast(ray6, raycastInfo3));
            test(!mCylinderShape->raycast(ray6, raycastInfo3));
            test(!mWorld->raycast(ray6, raycastInfo3));
            test(!mWorld->raycast(ray6));

            test(!mCylinderBody->raycast(ray7, raycastInfo3));
            test(!mCylinderShape->raycast(ray7, raycastInfo3));
            test(!mWorld->raycast(ray7, raycastInfo3));
            test(!mWorld->raycast(ray7));

            test(!mCylinderBody->raycast(ray8, raycastInfo3));
            test(!mCylinderShape->raycast(ray8, raycastInfo3));
            test(!mWorld->raycast(ray8, raycastInfo3));
            test(!mWorld->raycast(ray8));

            test(!mCylinderBody->raycast(ray9, raycastInfo3));
            test(!mCylinderShape->raycast(ray9, raycastInfo3));
            test(!mWorld->raycast(ray9, raycastInfo3));
            test(!mWorld->raycast(ray9));

            test(!mCylinderBody->raycast(ray10, raycastInfo3));
            test(!mCylinderShape->raycast(ray10, raycastInfo3));
            test(!mWorld->raycast(ray10, raycastInfo3));
            test(!mWorld->raycast(ray10));

            test(!mWorld->raycast(ray11, raycastInfo3, 0.5));
            test(!mWorld->raycast(ray12, raycastInfo3, 0.5));
            test(!mWorld->raycast(ray13, raycastInfo3, 0.5));
            test(!mWorld->raycast(ray14, raycastInfo3, 0.5));
            test(!mWorld->raycast(ray15, raycastInfo3, 0.5));
            test(!mWorld->raycast(ray16, raycastInfo3, 2));

            // ----- Test raycast hits ----- //
            test(mCylinderBody->raycast(ray11, raycastInfo3));
            test(mCylinderShape->raycast(ray11, raycastInfo3));
            test(mWorld->raycast(ray11, raycastInfo3));
            test(mWorld->raycast(ray11, raycastInfo3, 2));
            test(mWorld->raycast(ray11));

            test(mCylinderBody->raycast(ray12, raycastInfo3));
            test(mCylinderShape->raycast(ray12, raycastInfo3));
            test(mWorld->raycast(ray12, raycastInfo3));
            test(mWorld->raycast(ray12, raycastInfo3, 2));
            test(mWorld->raycast(ray12));

            test(mCylinderBody->raycast(ray13, raycastInfo3));
            test(mCylinderShape->raycast(ray13, raycastInfo3));
            test(mWorld->raycast(ray13, raycastInfo3));
            test(mWorld->raycast(ray13, raycastInfo3, 2));
            test(mWorld->raycast(ray13));

            test(mCylinderBody->raycast(ray14, raycastInfo3));
            test(mCylinderShape->raycast(ray14, raycastInfo3));
            test(mWorld->raycast(ray14, raycastInfo3));
            test(mWorld->raycast(ray14, raycastInfo3, 2));
            test(mWorld->raycast(ray14));

            test(mCylinderBody->raycast(ray15, raycastInfo3));
            test(mCylinderShape->raycast(ray15, raycastInfo3));
            test(mWorld->raycast(ray15, raycastInfo3));
            test(mWorld->raycast(ray15, raycastInfo3, 2));
            test(mWorld->raycast(ray15));

            test(mCylinderBody->raycast(ray16, raycastInfo3));
            test(mCylinderShape->raycast(ray16, raycastInfo3));
            test(mWorld->raycast(ray16, raycastInfo3));
            test(mWorld->raycast(ray16, raycastInfo3, 4));
            test(mWorld->raycast(ray16));
        }

        /// Test the CollisionBody::raycast() and
        /// CollisionWorld::raycast() methods.
        void testCompound() {

            // ----- Test feedback data ----- //
            const Matrix3x3 mLocalToWorldMatrix = mLocalShapeToWorld.getOrientation().getMatrix();

            // Raycast hit agains the sphere shape
            Ray ray1(mLocalShape2ToWorld * Vector3(4, 1, 2), mLocalToWorldMatrix * Vector3(-4, 0, 0));
            Ray ray2(mLocalShape2ToWorld * Vector3(1, 4, -1), mLocalToWorldMatrix * Vector3(0, -3, 0));
            Ray ray3(mLocalShape2ToWorld * Vector3(-1, 2, 5), mLocalToWorldMatrix * Vector3(0, 0, -8));
            Ray ray4(mLocalShape2ToWorld * Vector3(-5, 2, -2), mLocalToWorldMatrix * Vector3(4, 0, 0));
            Ray ray5(mLocalShape2ToWorld * Vector3(0, -4, 1), mLocalToWorldMatrix * Vector3(0, 3, 0));
            Ray ray6(mLocalShape2ToWorld * Vector3(-1, 2, -11), mLocalToWorldMatrix * Vector3(0, 0, 8));

            RaycastInfo raycastInfo;
            test(mCompoundBody->raycast(ray1, raycastInfo));
            test(mWorld->raycast(ray1, raycastInfo));
            test(mWorld->raycast(ray1, raycastInfo, 2));
            test(mWorld->raycast(ray1));

            test(mCompoundBody->raycast(ray2, raycastInfo));
            test(mWorld->raycast(ray2, raycastInfo));
            test(mWorld->raycast(ray2, raycastInfo, 2));
            test(mWorld->raycast(ray2));

            test(mCompoundBody->raycast(ray3, raycastInfo));
            test(mWorld->raycast(ray3, raycastInfo));
            test(mWorld->raycast(ray3, raycastInfo, 2));
            test(mWorld->raycast(ray3));

            test(mCompoundBody->raycast(ray4, raycastInfo));
            test(mWorld->raycast(ray4, raycastInfo));
            test(mWorld->raycast(ray4, raycastInfo, 2));
            test(mWorld->raycast(ray4));

            test(mCompoundBody->raycast(ray5, raycastInfo));
            test(mWorld->raycast(ray5, raycastInfo));
            test(mWorld->raycast(ray5, raycastInfo, 2));
            test(mWorld->raycast(ray5));

            test(mCompoundBody->raycast(ray6, raycastInfo));
            test(mWorld->raycast(ray6, raycastInfo));
            test(mWorld->raycast(ray6, raycastInfo, 4));
            test(mWorld->raycast(ray6));

            // Raycast hit agains the cylinder shape
            Ray ray11(mLocalShapeToWorld * Vector3(4, 1, 2), mLocalToWorldMatrix * Vector3(-4, 0, 0));
            Ray ray12(mLocalShapeToWorld * Vector3(1, 9, -1), mLocalToWorldMatrix * Vector3(0, -3, 0));
            Ray ray13(mLocalShapeToWorld * Vector3(-1, 2, 3), mLocalToWorldMatrix * Vector3(0, 0, -8));
            Ray ray14(mLocalShapeToWorld * Vector3(-3, 2, -2), mLocalToWorldMatrix * Vector3(4, 0, 0));
            Ray ray15(mLocalShapeToWorld * Vector3(0, -9, 1), mLocalToWorldMatrix * Vector3(0, 3, 0));
            Ray ray16(mLocalShapeToWorld * Vector3(-1, 2, -7), mLocalToWorldMatrix * Vector3(0, 0, 8));

            test(mCompoundBody->raycast(ray11, raycastInfo));
            test(mWorld->raycast(ray11, raycastInfo));
            test(mWorld->raycast(ray11, raycastInfo, 2));
            test(mWorld->raycast(ray11));

            test(mCompoundBody->raycast(ray12, raycastInfo));
            test(mWorld->raycast(ray12, raycastInfo));
            test(mWorld->raycast(ray12, raycastInfo, 2));
            test(mWorld->raycast(ray12));

            test(mCompoundBody->raycast(ray13, raycastInfo));
            test(mWorld->raycast(ray13, raycastInfo));
            test(mWorld->raycast(ray13, raycastInfo, 2));
            test(mWorld->raycast(ray13));

            test(mCompoundBody->raycast(ray14, raycastInfo));
            test(mWorld->raycast(ray14, raycastInfo));
            test(mWorld->raycast(ray14, raycastInfo, 2));
            test(mWorld->raycast(ray14));

            test(mCompoundBody->raycast(ray15, raycastInfo));
            test(mWorld->raycast(ray15, raycastInfo));
            test(mWorld->raycast(ray15, raycastInfo, 2));
            test(mWorld->raycast(ray15));

            test(mCompoundBody->raycast(ray16, raycastInfo));
            test(mWorld->raycast(ray16, raycastInfo));
            test(mWorld->raycast(ray16, raycastInfo, 4));
            test(mWorld->raycast(ray16));
        }
 };

}

#endif
