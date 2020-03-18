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

#ifndef TEST_POINT_INSIDE_H
#define TEST_POINT_INSIDE_H

// Libraries
#include "Test.h"
#include <reactphysics3d/collision/shapes/BoxShape.h>
#include <reactphysics3d/collision/shapes/SphereShape.h>
#include <reactphysics3d/collision/shapes/CapsuleShape.h>
#include <reactphysics3d/collision/shapes/ConvexMeshShape.h>
#include <reactphysics3d/engine/PhysicsWorld.h>
#include <reactphysics3d/engine/PhysicsCommon.h>
#include <reactphysics3d/collision/PolygonVertexArray.h>

/// Reactphysics3D namespace
namespace reactphysics3d {

// Class TestPointInside
/**
 * Unit test for the CollisionBody::testPointInside() method.
 */
class TestPointInside : public Test {

    private :

        // ---------- Atributes ---------- //

        // Physics common
        PhysicsCommon mPhysicsCommon;

        // Physics world
        PhysicsWorld* mWorld;

        // Bodies
        CollisionBody* mBoxBody;
        CollisionBody* mSphereBody;
        CollisionBody* mCapsuleBody;
        CollisionBody* mConeBody;
        CollisionBody* mConvexMeshBody;
        CollisionBody* mConvexMeshBodyEdgesInfo;
        CollisionBody* mCylinderBody;
        CollisionBody* mCompoundBody;

        Vector3 mConvexMeshCubeVertices[8];
        int mConvexMeshCubeIndices[24];
        PolygonVertexArray* mConvexMeshPolygonVertexArray;
        PolyhedronMesh* mConvexMeshPolyhedronMesh;
        PolygonVertexArray::PolygonFace* mConvexMeshPolygonFaces;

        // Collision shapes
        BoxShape* mBoxShape;
        SphereShape* mSphereShape;
        CapsuleShape* mCapsuleShape;
        ConvexMeshShape* mConvexMeshShape;

        // Transform
        Transform mBodyTransform;
        Transform mShapeTransform;
        Transform mLocalShapeToWorld;
        Transform mLocalShape2ToWorld;

        // Colliders
        Collider* mBoxCollider;
        Collider* mSphereCollider;
        Collider* mCapsuleCollider;
        Collider* mConvexMeshCollider;

    public :

        // ---------- Methods ---------- //

        /// Constructor
        TestPointInside(const std::string& name) : Test(name) {

            // Create the world
            mWorld = mPhysicsCommon.createPhysicsWorld();

            // Body transform
            Vector3 position(-3, 2, 7);
            Quaternion orientation = Quaternion::fromEulerAngles(PI / 5, PI / 6, PI / 7);
            mBodyTransform = Transform(position, orientation);

            // Create the bodies
            mBoxBody = mWorld->createCollisionBody(mBodyTransform);
            mSphereBody = mWorld->createCollisionBody(mBodyTransform);
            mCapsuleBody = mWorld->createCollisionBody(mBodyTransform);
            mConeBody = mWorld->createCollisionBody(mBodyTransform);
            mConvexMeshBody = mWorld->createCollisionBody(mBodyTransform);
            mConvexMeshBodyEdgesInfo = mWorld->createCollisionBody(mBodyTransform);
            mCylinderBody = mWorld->createCollisionBody(mBodyTransform);
            mConvexMeshBody = mWorld->createCollisionBody(mBodyTransform);
            mCompoundBody = mWorld->createCollisionBody(mBodyTransform);

            // Collision shape transform
            Vector3 shapePosition(1, -4, -3);
            Quaternion shapeOrientation = Quaternion::fromEulerAngles(3 * PI / 6 , -PI / 8, PI / 3);
            mShapeTransform = Transform(shapePosition, shapeOrientation);

            // Compute the the transform from a local shape point to world-space
            mLocalShapeToWorld = mBodyTransform * mShapeTransform;

            // Create collision shapes
            mBoxShape = mPhysicsCommon.createBoxShape(Vector3(2, 3, 4));
            mBoxCollider = mBoxBody->addCollider(mBoxShape, mShapeTransform);

            mSphereShape = mPhysicsCommon.createSphereShape(3);
            mSphereCollider = mSphereBody->addCollider(mSphereShape, mShapeTransform);

            mCapsuleShape = mPhysicsCommon.createCapsuleShape(3, 10);
            mCapsuleCollider = mCapsuleBody->addCollider(mCapsuleShape, mShapeTransform);

            mConvexMeshCubeVertices[0] = Vector3(-2, -3, 4);
            mConvexMeshCubeVertices[1] = Vector3(2, -3, 4);
            mConvexMeshCubeVertices[2] = Vector3(2, -3, -4);
            mConvexMeshCubeVertices[3] = Vector3(-2, -3, -4);
            mConvexMeshCubeVertices[4] = Vector3(-2, 3, 4);
            mConvexMeshCubeVertices[5] = Vector3(2, 3, 4);
            mConvexMeshCubeVertices[6] = Vector3(2, 3, -4);
            mConvexMeshCubeVertices[7] = Vector3(-2, 3, -4);

            mConvexMeshCubeIndices[0] = 0; mConvexMeshCubeIndices[1] = 3; mConvexMeshCubeIndices[2] = 2; mConvexMeshCubeIndices[3] = 1;
            mConvexMeshCubeIndices[4] = 4; mConvexMeshCubeIndices[5] = 5; mConvexMeshCubeIndices[6] = 6; mConvexMeshCubeIndices[7] = 7;
            mConvexMeshCubeIndices[8] = 0; mConvexMeshCubeIndices[9] = 1; mConvexMeshCubeIndices[10] = 5; mConvexMeshCubeIndices[11] = 4;
            mConvexMeshCubeIndices[12] = 1; mConvexMeshCubeIndices[13] = 2; mConvexMeshCubeIndices[14] = 6; mConvexMeshCubeIndices[15] = 5;
            mConvexMeshCubeIndices[16] = 2; mConvexMeshCubeIndices[17] = 3; mConvexMeshCubeIndices[18] = 7; mConvexMeshCubeIndices[19] = 6;
            mConvexMeshCubeIndices[20] = 0; mConvexMeshCubeIndices[21] = 4; mConvexMeshCubeIndices[22] = 7; mConvexMeshCubeIndices[23] = 3;

            mConvexMeshPolygonFaces = new PolygonVertexArray::PolygonFace[6];
            PolygonVertexArray::PolygonFace* face = mConvexMeshPolygonFaces;
            for (int f = 0; f < 6; f++) {
                face->indexBase = f * 4;
                face->nbVertices = 4;
                face++;
            }
            mConvexMeshPolygonVertexArray = new PolygonVertexArray(8, &(mConvexMeshCubeVertices[0]), sizeof(Vector3),
                    &(mConvexMeshCubeIndices[0]), sizeof(int), 6, mConvexMeshPolygonFaces,
                    PolygonVertexArray::VertexDataType::VERTEX_FLOAT_TYPE,
                    PolygonVertexArray::IndexDataType::INDEX_INTEGER_TYPE);
            mConvexMeshPolyhedronMesh = mPhysicsCommon.createPolyhedronMesh(mConvexMeshPolygonVertexArray);
            mConvexMeshShape = mPhysicsCommon.createConvexMeshShape(mConvexMeshPolyhedronMesh);
            Transform convexMeshTransform(Vector3(10, 0, 0), Quaternion::identity());
            mConvexMeshCollider = mConvexMeshBody->addCollider(mConvexMeshShape, mShapeTransform);

            // Compound shape is a capsule and a sphere
            Vector3 positionShape2(Vector3(4, 2, -3));
            Quaternion orientationShape2 = Quaternion::fromEulerAngles(-3 * PI / 8, 1.5 * PI/ 3, PI / 13);
            Transform shapeTransform2(positionShape2, orientationShape2);
            mLocalShape2ToWorld = mBodyTransform * shapeTransform2;
            mCompoundBody->addCollider(mCapsuleShape, mShapeTransform);
            mCompoundBody->addCollider(mSphereShape, shapeTransform2);
        }

        /// Destructor
        virtual ~TestPointInside() {
            mPhysicsCommon.destroyPhysicsWorld(mWorld);
            mPhysicsCommon.destroyBoxShape(mBoxShape);
            mPhysicsCommon.destroySphereShape(mSphereShape);
            mPhysicsCommon.destroyCapsuleShape(mCapsuleShape);
            mPhysicsCommon.destroyConvexMeshShape(mConvexMeshShape);
            mPhysicsCommon.destroyPolyhedronMesh(mConvexMeshPolyhedronMesh);
            delete[] mConvexMeshPolygonFaces;
            delete mConvexMeshPolygonVertexArray;
        }

        /// Run the tests
        void run() {
            testBox();
            testSphere();
            testCapsule();
            testConvexMesh();
            testCompound();
        }

        /// Test the testPointInside() and
        /// CollisionBody::testPointInside() methods
        void testBox() {

            // Tests with CollisionBody
            rp3d_test(mBoxBody->testPointInside(mLocalShapeToWorld * Vector3(0, 0, 0)));
            rp3d_test(mBoxBody->testPointInside(mLocalShapeToWorld * Vector3(-1.9, 0, 0)));
            rp3d_test(mBoxBody->testPointInside(mLocalShapeToWorld * Vector3(1.9, 0, 0)));
            rp3d_test(mBoxBody->testPointInside(mLocalShapeToWorld * Vector3(0, -2.9, 0)));
            rp3d_test(mBoxBody->testPointInside(mLocalShapeToWorld * Vector3(0, 2.9, 0)));
            rp3d_test(mBoxBody->testPointInside(mLocalShapeToWorld * Vector3(0, 0, -3.9)));
            rp3d_test(mBoxBody->testPointInside(mLocalShapeToWorld * Vector3(0, 0, 3.9)));
            rp3d_test(mBoxBody->testPointInside(mLocalShapeToWorld * Vector3(-1.9, -2.9, -3.9)));
            rp3d_test(mBoxBody->testPointInside(mLocalShapeToWorld * Vector3(1.9, 2.9, 3.9)));
            rp3d_test(mBoxBody->testPointInside(mLocalShapeToWorld * Vector3(-1, -2, -1.5)));
            rp3d_test(mBoxBody->testPointInside(mLocalShapeToWorld * Vector3(-1, 2, -2.5)));
            rp3d_test(mBoxBody->testPointInside(mLocalShapeToWorld * Vector3(1, -2, 3.5)));

            rp3d_test(!mBoxBody->testPointInside(mLocalShapeToWorld * Vector3(-2.1, 0, 0)));
            rp3d_test(!mBoxBody->testPointInside(mLocalShapeToWorld * Vector3(2.1, 0, 0)));
            rp3d_test(!mBoxBody->testPointInside(mLocalShapeToWorld * Vector3(0, -3.1, 0)));
            rp3d_test(!mBoxBody->testPointInside(mLocalShapeToWorld * Vector3(0, 3.1, 0)));
            rp3d_test(!mBoxBody->testPointInside(mLocalShapeToWorld * Vector3(0, 0, -4.1)));
            rp3d_test(!mBoxBody->testPointInside(mLocalShapeToWorld * Vector3(0, 0, 4.1)));
            rp3d_test(!mBoxBody->testPointInside(mLocalShapeToWorld * Vector3(-2.1, -3.1, -4.1)));
            rp3d_test(!mBoxBody->testPointInside(mLocalShapeToWorld * Vector3(2.1, 3.1, 4.1)));
            rp3d_test(!mBoxBody->testPointInside(mLocalShapeToWorld * Vector3(-10, -2, -1.5)));
            rp3d_test(!mBoxBody->testPointInside(mLocalShapeToWorld * Vector3(-1, 4, -2.5)));
            rp3d_test(!mBoxBody->testPointInside(mLocalShapeToWorld * Vector3(1, -2, 4.5)));

            // Tests with Collider
            rp3d_test(mBoxCollider->testPointInside(mLocalShapeToWorld * Vector3(0, 0, 0)));
            rp3d_test(mBoxCollider->testPointInside(mLocalShapeToWorld * Vector3(-1.9, 0, 0)));
            rp3d_test(mBoxCollider->testPointInside(mLocalShapeToWorld * Vector3(1.9, 0, 0)));
            rp3d_test(mBoxCollider->testPointInside(mLocalShapeToWorld * Vector3(0, -2.9, 0)));
            rp3d_test(mBoxCollider->testPointInside(mLocalShapeToWorld * Vector3(0, 2.9, 0)));
            rp3d_test(mBoxCollider->testPointInside(mLocalShapeToWorld * Vector3(0, 0, -3.9)));
            rp3d_test(mBoxCollider->testPointInside(mLocalShapeToWorld * Vector3(0, 0, 3.9)));
            rp3d_test(mBoxCollider->testPointInside(mLocalShapeToWorld * Vector3(-1.9, -2.9, -3.9)));
            rp3d_test(mBoxCollider->testPointInside(mLocalShapeToWorld * Vector3(1.9, 2.9, 3.9)));
            rp3d_test(mBoxCollider->testPointInside(mLocalShapeToWorld * Vector3(-1, -2, -1.5)));
            rp3d_test(mBoxCollider->testPointInside(mLocalShapeToWorld * Vector3(-1, 2, -2.5)));
            rp3d_test(mBoxCollider->testPointInside(mLocalShapeToWorld * Vector3(1, -2, 3.5)));

            rp3d_test(!mBoxCollider->testPointInside(mLocalShapeToWorld * Vector3(-2.1, 0, 0)));
            rp3d_test(!mBoxCollider->testPointInside(mLocalShapeToWorld * Vector3(2.1, 0, 0)));
            rp3d_test(!mBoxCollider->testPointInside(mLocalShapeToWorld * Vector3(0, -3.1, 0)));
            rp3d_test(!mBoxCollider->testPointInside(mLocalShapeToWorld * Vector3(0, 3.1, 0)));
            rp3d_test(!mBoxCollider->testPointInside(mLocalShapeToWorld * Vector3(0, 0, -4.1)));
            rp3d_test(!mBoxCollider->testPointInside(mLocalShapeToWorld * Vector3(0, 0, 4.1)));
            rp3d_test(!mBoxCollider->testPointInside(mLocalShapeToWorld * Vector3(-2.1, -3.1, -4.1)));
            rp3d_test(!mBoxCollider->testPointInside(mLocalShapeToWorld * Vector3(2.1, 3.1, 4.1)));
            rp3d_test(!mBoxCollider->testPointInside(mLocalShapeToWorld * Vector3(-10, -2, -1.5)));
            rp3d_test(!mBoxCollider->testPointInside(mLocalShapeToWorld * Vector3(-1, 4, -2.5)));
            rp3d_test(!mBoxCollider->testPointInside(mLocalShapeToWorld * Vector3(1, -2, 4.5)));
        }

        /// Test the Collider::testPointInside() and
        /// CollisionBody::testPointInside() methods
        void testSphere() {

            // Tests with CollisionBody
            rp3d_test(mSphereBody->testPointInside(mLocalShapeToWorld * Vector3(0, 0, 0)));
            rp3d_test(mSphereBody->testPointInside(mLocalShapeToWorld * Vector3(2.9, 0, 0)));
            rp3d_test(mSphereBody->testPointInside(mLocalShapeToWorld * Vector3(-2.9, 0, 0)));
            rp3d_test(mSphereBody->testPointInside(mLocalShapeToWorld * Vector3(0, 2.9, 0)));
            rp3d_test(mSphereBody->testPointInside(mLocalShapeToWorld * Vector3(0, -2.9, 0)));
            rp3d_test(mSphereBody->testPointInside(mLocalShapeToWorld * Vector3(0, 0, 2.9)));
            rp3d_test(mSphereBody->testPointInside(mLocalShapeToWorld * Vector3(0, 0, 2.9)));
            rp3d_test(mSphereBody->testPointInside(mLocalShapeToWorld * Vector3(-1, -2, -1.5)));
            rp3d_test(mSphereBody->testPointInside(mLocalShapeToWorld * Vector3(-1, 2, -1.5)));
            rp3d_test(mSphereBody->testPointInside(mLocalShapeToWorld * Vector3(1, -2, 1.5)));

            rp3d_test(!mSphereBody->testPointInside(mLocalShapeToWorld * Vector3(3.1, 0, 0)));
            rp3d_test(!mSphereBody->testPointInside(mLocalShapeToWorld * Vector3(-3.1, 0, 0)));
            rp3d_test(!mSphereBody->testPointInside(mLocalShapeToWorld * Vector3(0, 3.1, 0)));
            rp3d_test(!mSphereBody->testPointInside(mLocalShapeToWorld * Vector3(0, -3.1, 0)));
            rp3d_test(!mSphereBody->testPointInside(mLocalShapeToWorld * Vector3(0, 0, 3.1)));
            rp3d_test(!mSphereBody->testPointInside(mLocalShapeToWorld * Vector3(0, 0, -3.1)));
            rp3d_test(!mSphereBody->testPointInside(mLocalShapeToWorld * Vector3(-2, -2, -2)));
            rp3d_test(!mSphereBody->testPointInside(mLocalShapeToWorld * Vector3(-2, 2, -1.5)));
            rp3d_test(!mSphereBody->testPointInside(mLocalShapeToWorld * Vector3(1.5, -2, 2.5)));

            // Tests with Collider
            rp3d_test(mSphereCollider->testPointInside(mLocalShapeToWorld * Vector3(0, 0, 0)));
            rp3d_test(mSphereCollider->testPointInside(mLocalShapeToWorld * Vector3(2.9, 0, 0)));
            rp3d_test(mSphereCollider->testPointInside(mLocalShapeToWorld * Vector3(-2.9, 0, 0)));
            rp3d_test(mSphereCollider->testPointInside(mLocalShapeToWorld * Vector3(0, 2.9, 0)));
            rp3d_test(mSphereCollider->testPointInside(mLocalShapeToWorld * Vector3(0, -2.9, 0)));
            rp3d_test(mSphereCollider->testPointInside(mLocalShapeToWorld * Vector3(0, 0, 2.9)));
            rp3d_test(mSphereCollider->testPointInside(mLocalShapeToWorld * Vector3(0, 0, 2.9)));
            rp3d_test(mSphereCollider->testPointInside(mLocalShapeToWorld * Vector3(-1, -2, -1.5)));
            rp3d_test(mSphereCollider->testPointInside(mLocalShapeToWorld * Vector3(-1, 2, -1.5)));
            rp3d_test(mSphereCollider->testPointInside(mLocalShapeToWorld * Vector3(1, -2, 1.5)));

            rp3d_test(!mSphereCollider->testPointInside(mLocalShapeToWorld * Vector3(3.1, 0, 0)));
            rp3d_test(!mSphereCollider->testPointInside(mLocalShapeToWorld * Vector3(-3.1, 0, 0)));
            rp3d_test(!mSphereCollider->testPointInside(mLocalShapeToWorld * Vector3(0, 3.1, 0)));
            rp3d_test(!mSphereCollider->testPointInside(mLocalShapeToWorld * Vector3(0, -3.1, 0)));
            rp3d_test(!mSphereCollider->testPointInside(mLocalShapeToWorld * Vector3(0, 0, 3.1)));
            rp3d_test(!mSphereCollider->testPointInside(mLocalShapeToWorld * Vector3(0, 0, -3.1)));
            rp3d_test(!mSphereCollider->testPointInside(mLocalShapeToWorld * Vector3(-2, -2, -2)));
            rp3d_test(!mSphereCollider->testPointInside(mLocalShapeToWorld * Vector3(-2, 2, -1.5)));
            rp3d_test(!mSphereCollider->testPointInside(mLocalShapeToWorld * Vector3(1.5, -2, 2.5)));
        }

        /// Test the Collider::testPointInside() and
        /// CollisionBody::testPointInside() methods
        void testCapsule() {

            // Tests with CollisionBody
            rp3d_test(mCapsuleBody->testPointInside(mLocalShapeToWorld * Vector3(0, 0, 0)));
            rp3d_test(mCapsuleBody->testPointInside(mLocalShapeToWorld * Vector3(0, 5, 0)));
            rp3d_test(mCapsuleBody->testPointInside(mLocalShapeToWorld * Vector3(0, -5, 0)));
            rp3d_test(mCapsuleBody->testPointInside(mLocalShapeToWorld * Vector3(0, -6.9, 0)));
            rp3d_test(mCapsuleBody->testPointInside(mLocalShapeToWorld * Vector3(0, 6.9, 0)));
            rp3d_test(mCapsuleBody->testPointInside(mLocalShapeToWorld * Vector3(0, 0, 1.9)));
            rp3d_test(mCapsuleBody->testPointInside(mLocalShapeToWorld * Vector3(0, 0, -1.9)));
            rp3d_test(mCapsuleBody->testPointInside(mLocalShapeToWorld * Vector3(1.9, 0, 0)));
            rp3d_test(mCapsuleBody->testPointInside(mLocalShapeToWorld * Vector3(-1.9, 0, 0)));
            rp3d_test(mCapsuleBody->testPointInside(mLocalShapeToWorld * Vector3(0.9, 0, 0.9)));
            rp3d_test(mCapsuleBody->testPointInside(mLocalShapeToWorld * Vector3(0.9, 0, -0.9)));
            rp3d_test(mCapsuleBody->testPointInside(mLocalShapeToWorld * Vector3(0, 5, 1.9)));
            rp3d_test(mCapsuleBody->testPointInside(mLocalShapeToWorld * Vector3(0, 5, -1.9)));
            rp3d_test(mCapsuleBody->testPointInside(mLocalShapeToWorld * Vector3(1.9, 5, 0)));
            rp3d_test(mCapsuleBody->testPointInside(mLocalShapeToWorld * Vector3(-1.9, 5, 0)));
            rp3d_test(mCapsuleBody->testPointInside(mLocalShapeToWorld * Vector3(0.9, 5, 0.9)));
            rp3d_test(mCapsuleBody->testPointInside(mLocalShapeToWorld * Vector3(0.9, 5, -0.9)));
            rp3d_test(mCapsuleBody->testPointInside(mLocalShapeToWorld * Vector3(0, -5, 1.9)));
            rp3d_test(mCapsuleBody->testPointInside(mLocalShapeToWorld * Vector3(0, -5, -1.9)));
            rp3d_test(mCapsuleBody->testPointInside(mLocalShapeToWorld * Vector3(1.9, -5, 0)));
            rp3d_test(mCapsuleBody->testPointInside(mLocalShapeToWorld * Vector3(-1.9, -5, 0)));
            rp3d_test(mCapsuleBody->testPointInside(mLocalShapeToWorld * Vector3(0.9, -5, 0.9)));
            rp3d_test(mCapsuleBody->testPointInside(mLocalShapeToWorld * Vector3(0.9, -5, -0.9)));
            rp3d_test(mCapsuleBody->testPointInside(mLocalShapeToWorld * Vector3(-1.7, -4, -0.9)));
            rp3d_test(mCapsuleBody->testPointInside(mLocalShapeToWorld * Vector3(-1, 2, 0.4)));
            rp3d_test(mCapsuleBody->testPointInside(mLocalShapeToWorld * Vector3(1.3, 1, 1.5)));

            rp3d_test(!mCapsuleBody->testPointInside(mLocalShapeToWorld * Vector3(0, -13.1, 0)));
            rp3d_test(!mCapsuleBody->testPointInside(mLocalShapeToWorld * Vector3(0, 13.1, 0)));
            rp3d_test(!mCapsuleBody->testPointInside(mLocalShapeToWorld * Vector3(0, 0, 3.1)));
            rp3d_test(!mCapsuleBody->testPointInside(mLocalShapeToWorld * Vector3(0, 0, -3.1)));
            rp3d_test(!mCapsuleBody->testPointInside(mLocalShapeToWorld * Vector3(3.1, 0, 0)));
            rp3d_test(!mCapsuleBody->testPointInside(mLocalShapeToWorld * Vector3(-3.1, 0, 0)));
            rp3d_test(!mCapsuleBody->testPointInside(mLocalShapeToWorld * Vector3(0, 5, 3.1)));
            rp3d_test(!mCapsuleBody->testPointInside(mLocalShapeToWorld * Vector3(0, 5, -3.1)));
            rp3d_test(!mCapsuleBody->testPointInside(mLocalShapeToWorld * Vector3(3.1, 5, 0)));
            rp3d_test(!mCapsuleBody->testPointInside(mLocalShapeToWorld * Vector3(-3.1, 5, 0)));
            rp3d_test(!mCapsuleBody->testPointInside(mLocalShapeToWorld * Vector3(2.5, 5, 2.6)));
            rp3d_test(!mCapsuleBody->testPointInside(mLocalShapeToWorld * Vector3(2.5, 5, -2.7)));
            rp3d_test(!mCapsuleBody->testPointInside(mLocalShapeToWorld * Vector3(0, -5, 3.1)));
            rp3d_test(!mCapsuleBody->testPointInside(mLocalShapeToWorld * Vector3(0, -5, -3.1)));
            rp3d_test(!mCapsuleBody->testPointInside(mLocalShapeToWorld * Vector3(3.1, -5, 0)));
            rp3d_test(!mCapsuleBody->testPointInside(mLocalShapeToWorld * Vector3(-3.1, -5, 0)));
            rp3d_test(!mCapsuleBody->testPointInside(mLocalShapeToWorld * Vector3(2.5, -5, 2.6)));
            rp3d_test(!mCapsuleBody->testPointInside(mLocalShapeToWorld * Vector3(2.5, -5, -2.7)));

            // Tests with Collider
            rp3d_test(mCapsuleCollider->testPointInside(mLocalShapeToWorld * Vector3(0, 0, 0)));
            rp3d_test(mCapsuleCollider->testPointInside(mLocalShapeToWorld * Vector3(0, 5, 0)));
            rp3d_test(mCapsuleCollider->testPointInside(mLocalShapeToWorld * Vector3(0, -5, 0)));
            rp3d_test(mCapsuleCollider->testPointInside(mLocalShapeToWorld * Vector3(0, -6.9, 0)));
            rp3d_test(mCapsuleCollider->testPointInside(mLocalShapeToWorld * Vector3(0, 6.9, 0)));
            rp3d_test(mCapsuleCollider->testPointInside(mLocalShapeToWorld * Vector3(0, 0, 1.9)));
            rp3d_test(mCapsuleCollider->testPointInside(mLocalShapeToWorld * Vector3(0, 0, -1.9)));
            rp3d_test(mCapsuleCollider->testPointInside(mLocalShapeToWorld * Vector3(1.9, 0, 0)));
            rp3d_test(mCapsuleCollider->testPointInside(mLocalShapeToWorld * Vector3(-1.9, 0, 0)));
            rp3d_test(mCapsuleCollider->testPointInside(mLocalShapeToWorld * Vector3(0.9, 0, 0.9)));
            rp3d_test(mCapsuleCollider->testPointInside(mLocalShapeToWorld * Vector3(0.9, 0, -0.9)));
            rp3d_test(mCapsuleCollider->testPointInside(mLocalShapeToWorld * Vector3(0, 5, 1.9)));
            rp3d_test(mCapsuleCollider->testPointInside(mLocalShapeToWorld * Vector3(0, 5, -1.9)));
            rp3d_test(mCapsuleCollider->testPointInside(mLocalShapeToWorld * Vector3(1.9, 5, 0)));
            rp3d_test(mCapsuleCollider->testPointInside(mLocalShapeToWorld * Vector3(-1.9, 5, 0)));
            rp3d_test(mCapsuleCollider->testPointInside(mLocalShapeToWorld * Vector3(0.9, 5, 0.9)));
            rp3d_test(mCapsuleCollider->testPointInside(mLocalShapeToWorld * Vector3(0.9, 5, -0.9)));
            rp3d_test(mCapsuleCollider->testPointInside(mLocalShapeToWorld * Vector3(0, -5, 1.9)));
            rp3d_test(mCapsuleCollider->testPointInside(mLocalShapeToWorld * Vector3(0, -5, -1.9)));
            rp3d_test(mCapsuleCollider->testPointInside(mLocalShapeToWorld * Vector3(1.9, -5, 0)));
            rp3d_test(mCapsuleCollider->testPointInside(mLocalShapeToWorld * Vector3(-1.9, -5, 0)));
            rp3d_test(mCapsuleCollider->testPointInside(mLocalShapeToWorld * Vector3(0.9, -5, 0.9)));
            rp3d_test(mCapsuleCollider->testPointInside(mLocalShapeToWorld * Vector3(0.9, -5, -0.9)));
            rp3d_test(mCapsuleCollider->testPointInside(mLocalShapeToWorld * Vector3(-1.7, -4, -0.9)));
            rp3d_test(mCapsuleCollider->testPointInside(mLocalShapeToWorld * Vector3(-1, 2, 0.4)));
            rp3d_test(mCapsuleCollider->testPointInside(mLocalShapeToWorld * Vector3(1.3, 1, 1.5)));

            rp3d_test(!mCapsuleCollider->testPointInside(mLocalShapeToWorld * Vector3(0, -13.1, 0)));
            rp3d_test(!mCapsuleCollider->testPointInside(mLocalShapeToWorld * Vector3(0, 13.1, 0)));
            rp3d_test(!mCapsuleCollider->testPointInside(mLocalShapeToWorld * Vector3(0, 0, 3.1)));
            rp3d_test(!mCapsuleCollider->testPointInside(mLocalShapeToWorld * Vector3(0, 0, -3.1)));
            rp3d_test(!mCapsuleCollider->testPointInside(mLocalShapeToWorld * Vector3(3.1, 0, 0)));
            rp3d_test(!mCapsuleCollider->testPointInside(mLocalShapeToWorld * Vector3(-3.1, 0, 0)));
            rp3d_test(!mCapsuleCollider->testPointInside(mLocalShapeToWorld * Vector3(0, 5, 3.1)));
            rp3d_test(!mCapsuleCollider->testPointInside(mLocalShapeToWorld * Vector3(0, 5, -3.1)));
            rp3d_test(!mCapsuleCollider->testPointInside(mLocalShapeToWorld * Vector3(3.1, 5, 0)));
            rp3d_test(!mCapsuleCollider->testPointInside(mLocalShapeToWorld * Vector3(-3.1, 5, 0)));
            rp3d_test(!mCapsuleCollider->testPointInside(mLocalShapeToWorld * Vector3(2.5, 5, 2.6)));
            rp3d_test(!mCapsuleCollider->testPointInside(mLocalShapeToWorld * Vector3(2.5, 5, -2.7)));
            rp3d_test(!mCapsuleCollider->testPointInside(mLocalShapeToWorld * Vector3(0, -5, 3.1)));
            rp3d_test(!mCapsuleCollider->testPointInside(mLocalShapeToWorld * Vector3(0, -5, -3.1)));
            rp3d_test(!mCapsuleCollider->testPointInside(mLocalShapeToWorld * Vector3(3.1, -5, 0)));
            rp3d_test(!mCapsuleCollider->testPointInside(mLocalShapeToWorld * Vector3(-3.1, -5, 0)));
            rp3d_test(!mCapsuleCollider->testPointInside(mLocalShapeToWorld * Vector3(2.5, -5, 2.6)));
            rp3d_test(!mCapsuleCollider->testPointInside(mLocalShapeToWorld * Vector3(2.5, -5, -2.7)));
        }

        /// Test the Collider::testPointInside() and
        /// CollisionBody::testPointInside() methods
        void testConvexMesh() {

            // Tests with CollisionBody
            rp3d_test(mConvexMeshBody->testPointInside(mLocalShapeToWorld * Vector3(0, 0, 0)));
            rp3d_test(mConvexMeshBody->testPointInside(mLocalShapeToWorld * Vector3(-1.9, 0, 0)));
            rp3d_test(mConvexMeshBody->testPointInside(mLocalShapeToWorld * Vector3(1.9, 0, 0)));
            rp3d_test(mConvexMeshBody->testPointInside(mLocalShapeToWorld * Vector3(0, -2.9, 0)));
            rp3d_test(mConvexMeshBody->testPointInside(mLocalShapeToWorld * Vector3(0, 2.9, 0)));
            rp3d_test(mConvexMeshBody->testPointInside(mLocalShapeToWorld * Vector3(0, 0, -3.9)));
            rp3d_test(mConvexMeshBody->testPointInside(mLocalShapeToWorld * Vector3(0, 0, 3.9)));
            rp3d_test(mConvexMeshBody->testPointInside(mLocalShapeToWorld * Vector3(-1.9, -2.9, -3.9)));
            rp3d_test(mConvexMeshBody->testPointInside(mLocalShapeToWorld * Vector3(1.9, 2.9, 3.9)));
            rp3d_test(mConvexMeshBody->testPointInside(mLocalShapeToWorld * Vector3(-1, -2, -1.5)));
            rp3d_test(mConvexMeshBody->testPointInside(mLocalShapeToWorld * Vector3(-1, 2, -2.5)));
            rp3d_test(mConvexMeshBody->testPointInside(mLocalShapeToWorld * Vector3(1, -2, 3.5)));

            rp3d_test(!mConvexMeshBody->testPointInside(mLocalShapeToWorld * Vector3(-2.1, 0, 0)));
            rp3d_test(!mConvexMeshBody->testPointInside(mLocalShapeToWorld * Vector3(2.1, 0, 0)));
            rp3d_test(!mConvexMeshBody->testPointInside(mLocalShapeToWorld * Vector3(0, -3.1, 0)));
            rp3d_test(!mConvexMeshBody->testPointInside(mLocalShapeToWorld * Vector3(0, 3.1, 0)));
            rp3d_test(!mConvexMeshBody->testPointInside(mLocalShapeToWorld * Vector3(0, 0, -4.1)));
            rp3d_test(!mConvexMeshBody->testPointInside(mLocalShapeToWorld * Vector3(0, 0, 4.1)));
            rp3d_test(!mConvexMeshBody->testPointInside(mLocalShapeToWorld * Vector3(-2.1, -3.1, -4.1)));
            rp3d_test(!mConvexMeshBody->testPointInside(mLocalShapeToWorld * Vector3(2.1, 3.1, 4.1)));
            rp3d_test(!mConvexMeshBody->testPointInside(mLocalShapeToWorld * Vector3(-10, -2, -1.5)));
            rp3d_test(!mConvexMeshBody->testPointInside(mLocalShapeToWorld * Vector3(-1, 4, -2.5)));
            rp3d_test(!mConvexMeshBody->testPointInside(mLocalShapeToWorld * Vector3(1, -2, 4.5)));

            // Tests with Collider
            rp3d_test(mConvexMeshCollider->testPointInside(mLocalShapeToWorld * Vector3(0, 0, 0)));
            rp3d_test(mConvexMeshCollider->testPointInside(mLocalShapeToWorld * Vector3(-1.9, 0, 0)));
            rp3d_test(mConvexMeshCollider->testPointInside(mLocalShapeToWorld * Vector3(1.9, 0, 0)));
            rp3d_test(mConvexMeshCollider->testPointInside(mLocalShapeToWorld * Vector3(0, -2.9, 0)));
            rp3d_test(mConvexMeshCollider->testPointInside(mLocalShapeToWorld * Vector3(0, 2.9, 0)));
            rp3d_test(mConvexMeshCollider->testPointInside(mLocalShapeToWorld * Vector3(0, 0, -3.9)));
            rp3d_test(mConvexMeshCollider->testPointInside(mLocalShapeToWorld * Vector3(0, 0, 3.9)));
            rp3d_test(mConvexMeshCollider->testPointInside(mLocalShapeToWorld * Vector3(-1.9, -2.9, -3.9)));
            rp3d_test(mConvexMeshCollider->testPointInside(mLocalShapeToWorld * Vector3(1.9, 2.9, 3.9)));
            rp3d_test(mConvexMeshCollider->testPointInside(mLocalShapeToWorld * Vector3(-1, -2, -1.5)));
            rp3d_test(mConvexMeshCollider->testPointInside(mLocalShapeToWorld * Vector3(-1, 2, -2.5)));
            rp3d_test(mConvexMeshCollider->testPointInside(mLocalShapeToWorld * Vector3(1, -2, 3.5)));

            rp3d_test(!mConvexMeshCollider->testPointInside(mLocalShapeToWorld * Vector3(-2.1, 0, 0)));
            rp3d_test(!mConvexMeshCollider->testPointInside(mLocalShapeToWorld * Vector3(2.1, 0, 0)));
            rp3d_test(!mConvexMeshCollider->testPointInside(mLocalShapeToWorld * Vector3(0, -3.1, 0)));
            rp3d_test(!mConvexMeshCollider->testPointInside(mLocalShapeToWorld * Vector3(0, 3.1, 0)));
            rp3d_test(!mConvexMeshCollider->testPointInside(mLocalShapeToWorld * Vector3(0, 0, -4.1)));
            rp3d_test(!mConvexMeshCollider->testPointInside(mLocalShapeToWorld * Vector3(0, 0, 4.1)));
            rp3d_test(!mConvexMeshCollider->testPointInside(mLocalShapeToWorld * Vector3(-2.1, -3.1, -4.1)));
            rp3d_test(!mConvexMeshCollider->testPointInside(mLocalShapeToWorld * Vector3(2.1, 3.1, 4.1)));
            rp3d_test(!mConvexMeshCollider->testPointInside(mLocalShapeToWorld * Vector3(-10, -2, -1.5)));
            rp3d_test(!mConvexMeshCollider->testPointInside(mLocalShapeToWorld * Vector3(-1, 4, -2.5)));
            rp3d_test(!mConvexMeshCollider->testPointInside(mLocalShapeToWorld * Vector3(1, -2, 4.5)));
        }

        /// Test the CollisionBody::testPointInside() method
        void testCompound() {

            // Points on the capsule
            rp3d_test(mCompoundBody->testPointInside(mLocalShapeToWorld * Vector3(0, 0, 0)));
            rp3d_test(mCompoundBody->testPointInside(mLocalShapeToWorld * Vector3(0, 3.9, 0)));
            rp3d_test(mCompoundBody->testPointInside(mLocalShapeToWorld * Vector3(0, -3.9, 0)));
            rp3d_test(mCompoundBody->testPointInside(mLocalShapeToWorld * Vector3(2.9, 0, 0)));
            rp3d_test(mCompoundBody->testPointInside(mLocalShapeToWorld * Vector3(-2.9, 0, 0)));
            rp3d_test(mCompoundBody->testPointInside(mLocalShapeToWorld * Vector3(0, 0, 2.9)));
            rp3d_test(mCompoundBody->testPointInside(mLocalShapeToWorld * Vector3(0, 0, -2.9)));
            rp3d_test(mCompoundBody->testPointInside(mLocalShapeToWorld * Vector3(1.7, 0, 1.7)));
            rp3d_test(mCompoundBody->testPointInside(mLocalShapeToWorld * Vector3(1.7, 0, -1.7)));
            rp3d_test(mCompoundBody->testPointInside(mLocalShapeToWorld * Vector3(-1.7, 0, -1.7)));
            rp3d_test(mCompoundBody->testPointInside(mLocalShapeToWorld * Vector3(-1.7, 0, 1.7)));
            rp3d_test(mCompoundBody->testPointInside(mLocalShapeToWorld * Vector3(2.9, 3.9, 0)));
            rp3d_test(mCompoundBody->testPointInside(mLocalShapeToWorld * Vector3(-2.9, 3.9, 0)));
            rp3d_test(mCompoundBody->testPointInside(mLocalShapeToWorld * Vector3(0, 3.9, 2.9)));
            rp3d_test(mCompoundBody->testPointInside(mLocalShapeToWorld * Vector3(0, 3.9, -2.9)));
            rp3d_test(mCompoundBody->testPointInside(mLocalShapeToWorld * Vector3(1.7, 3.9, 1.7)));
            rp3d_test(mCompoundBody->testPointInside(mLocalShapeToWorld * Vector3(1.7, 3.9, -1.7)));
            rp3d_test(mCompoundBody->testPointInside(mLocalShapeToWorld * Vector3(-1.7, 3.9, -1.7)));
            rp3d_test(mCompoundBody->testPointInside(mLocalShapeToWorld * Vector3(-1.7, 3.9, 1.7)));
            rp3d_test(mCompoundBody->testPointInside(mLocalShapeToWorld * Vector3(2.9, -3.9, 0)));
            rp3d_test(mCompoundBody->testPointInside(mLocalShapeToWorld * Vector3(-2.9, -3.9, 0)));
            rp3d_test(mCompoundBody->testPointInside(mLocalShapeToWorld * Vector3(0, -3.9, 2.9)));
            rp3d_test(mCompoundBody->testPointInside(mLocalShapeToWorld * Vector3(0, -3.9, -2.9)));
            rp3d_test(mCompoundBody->testPointInside(mLocalShapeToWorld * Vector3(1.7, -3.9, 1.7)));
            rp3d_test(mCompoundBody->testPointInside(mLocalShapeToWorld * Vector3(1.7, -3.9, -1.7)));
            rp3d_test(mCompoundBody->testPointInside(mLocalShapeToWorld * Vector3(-1.7, -3.9, -1.7)));
            rp3d_test(mCompoundBody->testPointInside(mLocalShapeToWorld * Vector3(-1.7, -3.9, 1.7)));

            // Points on the sphere
            rp3d_test(mCompoundBody->testPointInside(mLocalShape2ToWorld * Vector3(0, 0, 0)));
            rp3d_test(mCompoundBody->testPointInside(mLocalShape2ToWorld * Vector3(2.9, 0, 0)));
            rp3d_test(mCompoundBody->testPointInside(mLocalShape2ToWorld * Vector3(-2.9, 0, 0)));
            rp3d_test(mCompoundBody->testPointInside(mLocalShape2ToWorld * Vector3(0, 2.9, 0)));
            rp3d_test(mCompoundBody->testPointInside(mLocalShape2ToWorld * Vector3(0, -2.9, 0)));
            rp3d_test(mCompoundBody->testPointInside(mLocalShape2ToWorld * Vector3(0, 0, 2.9)));
            rp3d_test(mCompoundBody->testPointInside(mLocalShape2ToWorld * Vector3(0, 0, 2.9)));
            rp3d_test(mCompoundBody->testPointInside(mLocalShape2ToWorld * Vector3(-1, -2, -1.5)));
            rp3d_test(mCompoundBody->testPointInside(mLocalShape2ToWorld * Vector3(-1, 2, -1.5)));
            rp3d_test(mCompoundBody->testPointInside(mLocalShape2ToWorld * Vector3(1, -2, 1.5)));
        }
 };

}

#endif
