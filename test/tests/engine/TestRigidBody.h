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

#ifndef TEST_RIGIDBODY_H
#define TEST_RIGIDBODY_H

// Libraries
#include <reactphysics3d/reactphysics3d.h>

/// Reactphysics3D namespace
namespace reactphysics3d {

// Class TestRigidBody
/**
 * Unit test for the RigidBody class.
 */
class TestRigidBody : public Test {

    private :

        // ---------- Atributes ---------- //

        PhysicsCommon mPhysicsCommon;
        PhysicsWorld* mWorld;

        RigidBody* mRigidBody1;
        RigidBody* mRigidBody2Box;
        RigidBody* mRigidBody2Sphere;
        RigidBody* mRigidBody2Convex;
        RigidBody* mRigidBody3;

        Collider* mBoxCollider;
        Collider* mSphereCollider;
        Collider* mConvexMeshCollider;

        PolygonVertexArray* mConvexMeshPolygonVertexArray;
        PolyhedronMesh* mConvexMeshPolyhedronMesh;
        PolygonVertexArray::PolygonFace* mConvexMeshPolygonFaces;
        float mConvexMeshCubeVertices[8 * 3];
        int mConvexMeshCubeIndices[24];

    public :

        // ---------- Methods ---------- //

        /// Constructor
        TestRigidBody(const std::string& name) : Test(name) {

            mWorld = mPhysicsCommon.createPhysicsWorld();

            const Transform transform1(Vector3(1, 2, 3), Quaternion::identity());
            mRigidBody1 = mWorld->createRigidBody(transform1);

            //  Box

            const Transform transform2(Vector3(0, 0, 0), Quaternion::identity());
            mRigidBody2Box = mWorld->createRigidBody(transform2);
            BoxShape* boxShape = mPhysicsCommon.createBoxShape(Vector3(2, 2, 2));
            mBoxCollider = mRigidBody2Box->addCollider(boxShape, Transform::identity());

            //  Sphere

            mRigidBody2Sphere = mWorld->createRigidBody(transform2);
            SphereShape* sphereShape = mPhysicsCommon.createSphereShape(4);
            mSphereCollider = mRigidBody2Sphere->addCollider(sphereShape, Transform::identity());

            //  Convex Meshes (in the shape of a box)

            mConvexMeshCubeVertices[0] = -3; mConvexMeshCubeVertices[1] = -3; mConvexMeshCubeVertices[2] = 3;
            mConvexMeshCubeVertices[3] = 3; mConvexMeshCubeVertices[4] = -3; mConvexMeshCubeVertices[5] = 3;
            mConvexMeshCubeVertices[6] = 3; mConvexMeshCubeVertices[7] = -3; mConvexMeshCubeVertices[8] = -3;
            mConvexMeshCubeVertices[9] = -3; mConvexMeshCubeVertices[10] = -3; mConvexMeshCubeVertices[11] = -3;
            mConvexMeshCubeVertices[12] = -3; mConvexMeshCubeVertices[13] = 3; mConvexMeshCubeVertices[14] = 3;
            mConvexMeshCubeVertices[15] = 3; mConvexMeshCubeVertices[16] = 3; mConvexMeshCubeVertices[17] = 3;
            mConvexMeshCubeVertices[18] = 3; mConvexMeshCubeVertices[19] = 3; mConvexMeshCubeVertices[20] = -3;
            mConvexMeshCubeVertices[21] = -3; mConvexMeshCubeVertices[22] = 3; mConvexMeshCubeVertices[23] = -3;

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
            mConvexMeshPolygonVertexArray = new rp3d::PolygonVertexArray(8, &(mConvexMeshCubeVertices[0]), 3 * sizeof(float),
                    &(mConvexMeshCubeIndices[0]), sizeof(int), 6, mConvexMeshPolygonFaces,
                    rp3d::PolygonVertexArray::VertexDataType::VERTEX_FLOAT_TYPE,
                    rp3d::PolygonVertexArray::IndexDataType::INDEX_INTEGER_TYPE);
            mConvexMeshPolyhedronMesh = mPhysicsCommon.createPolyhedronMesh(mConvexMeshPolygonVertexArray);
            ConvexMeshShape* convexMeshShape = mPhysicsCommon.createConvexMeshShape(mConvexMeshPolyhedronMesh);
            Transform transform3(Vector3(10, 0, 0), Quaternion::identity());
            mRigidBody2Convex = mWorld->createRigidBody(transform3);
            mConvexMeshCollider = mRigidBody2Convex->addCollider(convexMeshShape, Transform::identity());

            //  Rigidbody 3

            const decimal angleRad = 20 * PI_RP3D / 180.0;
            const Transform transform4(Vector3(1, 2, 3), Quaternion::fromEulerAngles(angleRad, angleRad, angleRad));
            mRigidBody3 = mWorld->createRigidBody(transform4);
            BoxShape* boxShape3 = mPhysicsCommon.createBoxShape(Vector3(2, 2, 2));
            mRigidBody3->addCollider(boxShape3, Transform::identity());
        }

        /// Destructor
        virtual ~TestRigidBody() {

            mWorld->destroyRigidBody(mRigidBody1);
            mWorld->destroyRigidBody(mRigidBody2Box);
            mWorld->destroyRigidBody(mRigidBody2Sphere);

            delete[] mConvexMeshPolygonFaces;
            delete mConvexMeshPolygonVertexArray;
        }

        /// Run the tests
        void run() {
            testGettersSetters();
            testMassPropertiesMethods();
            testApplyForcesAndTorques();
        }

        void testGettersSetters() {

           mRigidBody1->setMass(34);
           rp3d_test(mRigidBody1->getMass() == 34);

           mRigidBody1->setLinearDamping(0.6);
           rp3d_test(approxEqual(mRigidBody1->getLinearDamping(), 0.6));

           mRigidBody1->setAngularDamping(0.6);
           rp3d_test(approxEqual(mRigidBody1->getAngularDamping(), 0.6));

           mRigidBody1->setLinearLockAxisFactor(Vector3(0.2, 0.3, 0.4));
           rp3d_test(approxEqual(mRigidBody1->getLinearLockAxisFactor(), Vector3(0.2, 0.3, 0.4)));

           mRigidBody1->setAngularLockAxisFactor(Vector3(0.2, 0.3, 0.4));
           rp3d_test(approxEqual(mRigidBody1->getAngularLockAxisFactor(), Vector3(0.2, 0.3, 0.4)));

           mRigidBody1->setLinearVelocity(Vector3(2, 3, 4));
           rp3d_test(approxEqual(mRigidBody1->getLinearVelocity(), Vector3(2, 3, 4)));

           mRigidBody1->setAngularVelocity(Vector3(2, 3, 4));
           rp3d_test(approxEqual(mRigidBody1->getAngularVelocity(), Vector3(2, 3, 4)));

           mRigidBody1->setTransform(Transform(Vector3(5, 4, 3), Quaternion::fromEulerAngles(1.7, 1.8, 1.9)));
           rp3d_test(approxEqual(mRigidBody1->getTransform().getPosition(), Vector3(5, 4, 3)));
           rp3d_test(approxEqual(mRigidBody1->getTransform().getOrientation().x, Quaternion::fromEulerAngles(1.7, 1.8, 1.9).x));
           rp3d_test(approxEqual(mRigidBody1->getTransform().getOrientation().y, Quaternion::fromEulerAngles(1.7, 1.8, 1.9).y));
           rp3d_test(approxEqual(mRigidBody1->getTransform().getOrientation().z, Quaternion::fromEulerAngles(1.7, 1.8, 1.9).z));
           rp3d_test(approxEqual(mRigidBody1->getTransform().getOrientation().w, Quaternion::fromEulerAngles(1.7, 1.8, 1.9).w));

           mRigidBody1->setLocalCenterOfMass(Vector3(10, 20, 30));
           rp3d_test(approxEqual(mRigidBody1->getLocalCenterOfMass(), Vector3(10, 20, 30)));

           mRigidBody1->setType(BodyType::KINEMATIC);
           rp3d_test(mRigidBody1->getType() == BodyType::KINEMATIC);

           mRigidBody1->setLocalInertiaTensor(Vector3(2, 4, 6));
           rp3d_test(approxEqual(mRigidBody1->getLocalInertiaTensor(), Vector3(2, 4, 6)));
        }

        void testMassPropertiesMethods() {

            // Box collider
            mBoxCollider->getMaterial().setMassDensity(3);
            mRigidBody2Box->updateMassFromColliders();
            rp3d_test(approxEqual(mRigidBody2Box->getMass(), 64 * 3));

            mRigidBody2Box->setLocalCenterOfMass(Vector3(1, 2, 3));
            mRigidBody2Box->setMass(1);
            mRigidBody2Box->updateMassPropertiesFromColliders();
            rp3d_test(approxEqual(mRigidBody2Box->getMass(), 64 * 3));
            rp3d_test(approxEqual(mRigidBody2Box->getLocalCenterOfMass(), Vector3::zero()));

            mRigidBody2Box->setLocalCenterOfMass(Vector3(1, 2, 3));
            mRigidBody2Box->updateLocalCenterOfMassFromColliders();
            rp3d_test(approxEqual(mRigidBody2Box->getLocalCenterOfMass(), Vector3::zero()));

            mRigidBody2Box->setLocalInertiaTensor(Vector3(1, 2, 3));
            mRigidBody2Box->updateLocalInertiaTensorFromColliders();
            decimal tensorBox = 1.0 / 6.0 * 64 * 3 * 4 * 4;
            rp3d_test(approxEqual(mRigidBody2Box->getLocalInertiaTensor(), Vector3(tensorBox, tensorBox, tensorBox)));

            // Sphere collider
            mSphereCollider->getMaterial().setMassDensity(3);
            mRigidBody2Sphere->updateMassFromColliders();
            const decimal sphereMass = 4.0 / 3.0 * PI_RP3D * 64 * 3;
            rp3d_test(approxEqual(mRigidBody2Sphere->getMass(), sphereMass));

            mRigidBody2Sphere->setLocalCenterOfMass(Vector3(1, 2, 3));
            mRigidBody2Sphere->setMass(1);
            mRigidBody2Sphere->updateMassPropertiesFromColliders();
            rp3d_test(approxEqual(mRigidBody2Sphere->getMass(), sphereMass));
            rp3d_test(approxEqual(mRigidBody2Sphere->getLocalCenterOfMass(), Vector3::zero()));

            mRigidBody2Sphere->setLocalCenterOfMass(Vector3(1, 2, 3));
            mRigidBody2Sphere->updateLocalCenterOfMassFromColliders();
            rp3d_test(approxEqual(mRigidBody2Sphere->getLocalCenterOfMass(), Vector3::zero()));

            mRigidBody2Sphere->setLocalInertiaTensor(Vector3(1, 2, 3));
            mRigidBody2Sphere->updateLocalInertiaTensorFromColliders();
            const decimal tensorSphere = 2.0 / 5.0 * sphereMass * 4 * 4;
            rp3d_test(approxEqual(mRigidBody2Sphere->getLocalInertiaTensor(), Vector3(tensorSphere, tensorSphere, tensorSphere)));

            // Convex mesh collider
            mConvexMeshCollider->getMaterial().setMassDensity(3);
            mRigidBody2Convex->updateMassFromColliders();
            rp3d_test(approxEqual(mRigidBody2Convex->getMass(), 648));

            mRigidBody2Convex->setLocalCenterOfMass(Vector3(1, 2, 3));
            mRigidBody2Convex->setMass(1);
            mConvexMeshCollider->getMaterial().setMassDensity(2);
            mRigidBody2Convex->updateMassPropertiesFromColliders();
            rp3d_test(approxEqual(mRigidBody2Convex->getMass(), 432));
            rp3d_test(approxEqual(mRigidBody2Convex->getLocalCenterOfMass(), Vector3::zero()));

            mRigidBody2Convex->setLocalCenterOfMass(Vector3(1, 2, 3));
            mRigidBody2Convex->updateLocalCenterOfMassFromColliders();
            rp3d_test(approxEqual(mRigidBody2Convex->getLocalCenterOfMass(), Vector3::zero()));

            mRigidBody2Convex->setLocalInertiaTensor(Vector3(1, 2, 3));
            mRigidBody2Convex->updateLocalInertiaTensorFromColliders();
            tensorBox = 1.0 / 6.0 * 432 * 6 * 6;
            rp3d_test(approxEqual(mRigidBody2Convex->getLocalInertiaTensor(), Vector3(tensorBox, tensorBox, tensorBox)));
        }

        void testApplyForcesAndTorques() {

            const Transform& worldTransform = mRigidBody3->getTransform();
            const Quaternion orientation = worldTransform.getOrientation();

            mRigidBody3->applyLocalForceAtCenterOfMass(Vector3(4, 5, 6));
            rp3d_test(approxEqual(mRigidBody3->getForce(), orientation * Vector3(4, 5, 6)));
            rp3d_test(approxEqual(mRigidBody3->getTorque(), orientation * Vector3::zero()));

            mRigidBody3->resetForce();
            mRigidBody3->resetTorque();

            mRigidBody3->applyWorldForceAtCenterOfMass(Vector3(4, 5, 6));
            rp3d_test(approxEqual(mRigidBody3->getForce(), Vector3(4, 5, 6)));
            rp3d_test(approxEqual(mRigidBody3->getTorque(), Vector3::zero()));
            mRigidBody3->resetForce();
            mRigidBody3->resetTorque();

            mRigidBody3->applyLocalForceAtLocalPosition(Vector3(0, 0, 3), Vector3(2, 0, 0));
            rp3d_test(approxEqual(mRigidBody3->getForce(), orientation * Vector3(0, 0, 3)));

            rp3d_test(approxEqual(mRigidBody3->getTorque(), orientation * Vector3(0, -3 * 2, 0), decimal(0.0001)));
            mRigidBody3->resetForce();
            mRigidBody3->resetTorque();

            rp3d_test(approxEqual(mRigidBody3->getForce(), Vector3::zero()));
            rp3d_test(approxEqual(mRigidBody3->getTorque(), Vector3::zero()));

            mRigidBody3->applyLocalForceAtWorldPosition(Vector3(0, 0, 3), worldTransform * Vector3(2, 0, 0));
            rp3d_test(approxEqual(mRigidBody3->getForce(), orientation * Vector3(0, 0, 3)));
            rp3d_test(approxEqual(mRigidBody3->getTorque(), orientation * Vector3(0, -3 * 2, 0), decimal(0.0001)));
            mRigidBody3->resetForce();
            mRigidBody3->resetTorque();

            mRigidBody3->applyWorldForceAtLocalPosition(orientation * Vector3(0, 0, 3), Vector3(2, 0, 0));
            rp3d_test(approxEqual(mRigidBody3->getForce(), orientation * Vector3(0, 0, 3)));
            rp3d_test(approxEqual(mRigidBody3->getTorque(), orientation * Vector3(0, -3 * 2, 0), decimal(0.0001)));
            mRigidBody3->resetForce();
            mRigidBody3->resetTorque();

            mRigidBody3->applyWorldForceAtWorldPosition(orientation * Vector3(0, 0, 3), worldTransform * Vector3(2, 0, 0));
            rp3d_test(approxEqual(mRigidBody3->getForce(), orientation * Vector3(0, 0, 3)));
            rp3d_test(approxEqual(mRigidBody3->getTorque(), orientation * Vector3(0, -3 * 2, 0), decimal(0.0001)));
            mRigidBody3->resetForce();
            mRigidBody3->resetTorque();

            mRigidBody3->applyWorldTorque(Vector3(0, 4, 0));
            rp3d_test(approxEqual(mRigidBody3->getForce(), Vector3::zero()));
            rp3d_test(approxEqual(mRigidBody3->getTorque(), Vector3(0, 4, 0), decimal(0.0001)));
            mRigidBody3->resetForce();
            mRigidBody3->resetTorque();

            mRigidBody3->applyLocalTorque(Vector3(0, 4, 0));
            rp3d_test(approxEqual(mRigidBody3->getForce(), Vector3::zero()));
            rp3d_test(approxEqual(mRigidBody3->getTorque(), orientation * Vector3(0, 4, 0), decimal(0.0001)));
            mRigidBody3->resetForce();
            mRigidBody3->resetTorque();
        }
 };

}

#endif
