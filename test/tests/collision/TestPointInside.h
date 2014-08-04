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

#ifndef TEST_POINT_INSIDE_H
#define TEST_POINT_INSIDE_H

// Libraries
#include "Test.h"
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
class TestPointInside : public Test {

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
        TestPointInside() {

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

            CapsuleShape capsuleShape(2, 10);
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

            CylinderShape cylinderShape(3, 8, 0);
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

        /// Test the ProxyBoxShape::testPointInside() and
        /// CollisionBody::testPointInside() methods
        void testBox() {

            // Tests with CollisionBody
            test(mBoxBody->testPointInside(mLocalShapeToWorld * Vector3(0, 0, 0)));
            test(mBoxBody->testPointInside(mLocalShapeToWorld * Vector3(-1.9, 0, 0)));
            test(mBoxBody->testPointInside(mLocalShapeToWorld * Vector3(1.9, 0, 0)));
            test(mBoxBody->testPointInside(mLocalShapeToWorld * Vector3(0, -2.9, 0)));
            test(mBoxBody->testPointInside(mLocalShapeToWorld * Vector3(0, 2.9, 0)));
            test(mBoxBody->testPointInside(mLocalShapeToWorld * Vector3(0, 0, -3.9)));
            test(mBoxBody->testPointInside(mLocalShapeToWorld * Vector3(0, 0, 3.9)));
            test(mBoxBody->testPointInside(mLocalShapeToWorld * Vector3(-1.9, -2.9, -3.9)));
            test(mBoxBody->testPointInside(mLocalShapeToWorld * Vector3(1.9, 2.9, 3.9)));
            test(mBoxBody->testPointInside(mLocalShapeToWorld * Vector3(-1, -2, -1.5)));
            test(mBoxBody->testPointInside(mLocalShapeToWorld * Vector3(-1, 2, -2.5)));
            test(mBoxBody->testPointInside(mLocalShapeToWorld * Vector3(1, -2, 3.5)));

            test(!mBoxBody->testPointInside(mLocalShapeToWorld * Vector3(-2.1, 0, 0)));
            test(!mBoxBody->testPointInside(mLocalShapeToWorld * Vector3(2.1, 0, 0)));
            test(!mBoxBody->testPointInside(mLocalShapeToWorld * Vector3(0, -3.1, 0)));
            test(!mBoxBody->testPointInside(mLocalShapeToWorld * Vector3(0, 3.1, 0)));
            test(!mBoxBody->testPointInside(mLocalShapeToWorld * Vector3(0, 0, -4.1)));
            test(!mBoxBody->testPointInside(mLocalShapeToWorld * Vector3(0, 0, 4.1)));
            test(!mBoxBody->testPointInside(mLocalShapeToWorld * Vector3(-2.1, -3.1, -4.1)));
            test(!mBoxBody->testPointInside(mLocalShapeToWorld * Vector3(2.1, 3.1, 4.1)));
            test(!mBoxBody->testPointInside(mLocalShapeToWorld * Vector3(-10, -2, -1.5)));
            test(!mBoxBody->testPointInside(mLocalShapeToWorld * Vector3(-1, 4, -2.5)));
            test(!mBoxBody->testPointInside(mLocalShapeToWorld * Vector3(1, -2, 4.5)));

            // Tests with ProxyBoxShape
            test(mBoxShape->testPointInside(mLocalShapeToWorld * Vector3(0, 0, 0)));
            test(mBoxShape->testPointInside(mLocalShapeToWorld * Vector3(-1.9, 0, 0)));
            test(mBoxShape->testPointInside(mLocalShapeToWorld * Vector3(1.9, 0, 0)));
            test(mBoxShape->testPointInside(mLocalShapeToWorld * Vector3(0, -2.9, 0)));
            test(mBoxShape->testPointInside(mLocalShapeToWorld * Vector3(0, 2.9, 0)));
            test(mBoxShape->testPointInside(mLocalShapeToWorld * Vector3(0, 0, -3.9)));
            test(mBoxShape->testPointInside(mLocalShapeToWorld * Vector3(0, 0, 3.9)));
            test(mBoxShape->testPointInside(mLocalShapeToWorld * Vector3(-1.9, -2.9, -3.9)));
            test(mBoxShape->testPointInside(mLocalShapeToWorld * Vector3(1.9, 2.9, 3.9)));
            test(mBoxShape->testPointInside(mLocalShapeToWorld * Vector3(-1, -2, -1.5)));
            test(mBoxShape->testPointInside(mLocalShapeToWorld * Vector3(-1, 2, -2.5)));
            test(mBoxShape->testPointInside(mLocalShapeToWorld * Vector3(1, -2, 3.5)));

            test(!mBoxShape->testPointInside(mLocalShapeToWorld * Vector3(-2.1, 0, 0)));
            test(!mBoxShape->testPointInside(mLocalShapeToWorld * Vector3(2.1, 0, 0)));
            test(!mBoxShape->testPointInside(mLocalShapeToWorld * Vector3(0, -3.1, 0)));
            test(!mBoxShape->testPointInside(mLocalShapeToWorld * Vector3(0, 3.1, 0)));
            test(!mBoxShape->testPointInside(mLocalShapeToWorld * Vector3(0, 0, -4.1)));
            test(!mBoxShape->testPointInside(mLocalShapeToWorld * Vector3(0, 0, 4.1)));
            test(!mBoxShape->testPointInside(mLocalShapeToWorld * Vector3(-2.1, -3.1, -4.1)));
            test(!mBoxShape->testPointInside(mLocalShapeToWorld * Vector3(2.1, 3.1, 4.1)));
            test(!mBoxShape->testPointInside(mLocalShapeToWorld * Vector3(-10, -2, -1.5)));
            test(!mBoxShape->testPointInside(mLocalShapeToWorld * Vector3(-1, 4, -2.5)));
            test(!mBoxShape->testPointInside(mLocalShapeToWorld * Vector3(1, -2, 4.5)));
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
