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

/// Reactphysics3D namespace
namespace reactphysics3d {

// Enumeration for categories
enum CollisionCategory {
    CATEGORY_1 = 0x0001,
    CATEGORY_2 = 0x0002,
    CATEGORY_3 = 0x0004
};

// TODO : Add test for concave shape collision here

// Class
class WorldCollisionCallback : public CollisionCallback
{
    public:

        bool boxCollideWithSphere1;
        bool boxCollideWithCylinder;
        bool sphere1CollideWithCylinder;
        bool sphere1CollideWithSphere2;

        CollisionBody* boxBody;
        CollisionBody* sphere1Body;
        CollisionBody* sphere2Body;
        CollisionBody* cylinderBody;

        WorldCollisionCallback()
        {
            reset();
        }

        void reset()
        {
            boxCollideWithSphere1 = false;
            boxCollideWithCylinder = false;
            sphere1CollideWithCylinder = false;
            sphere1CollideWithSphere2 = false;
        }

        // This method will be called for contact
        virtual void notifyContact(const ContactPointInfo& contactPointInfo) {

            if (isContactBetweenBodies(boxBody, sphere1Body, contactPointInfo)) {
                boxCollideWithSphere1 = true;
            }
            else if (isContactBetweenBodies(boxBody, cylinderBody, contactPointInfo)) {
                boxCollideWithCylinder = true;
            }
            else if (isContactBetweenBodies(sphere1Body, cylinderBody, contactPointInfo)) {
                sphere1CollideWithCylinder = true;
            }
            else if (isContactBetweenBodies(sphere1Body, sphere2Body, contactPointInfo)) {
                sphere1CollideWithSphere2 = true;
            }
        }

        bool isContactBetweenBodies(const CollisionBody* body1, const CollisionBody* body2,
                                     const ContactPointInfo& contactPointInfo) {
            return (contactPointInfo.shape1->getBody()->getID() == body1->getID() &&
                    contactPointInfo.shape2->getBody()->getID() == body2->getID()) ||
                   (contactPointInfo.shape2->getBody()->getID() == body1->getID() &&
                    contactPointInfo.shape1->getBody()->getID() == body2->getID());
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
        CollisionBody* mBoxBody;
        CollisionBody* mSphere1Body;
        CollisionBody* mSphere2Body;
        CollisionBody* mCylinderBody;

        // Collision shapes
        BoxShape* mBoxShape;
        SphereShape* mSphereShape;
        CylinderShape* mCylinderShape;

        // Proxy shapes
        ProxyShape* mBoxProxyShape;
        ProxyShape* mSphere1ProxyShape;
        ProxyShape* mSphere2ProxyShape;
        ProxyShape* mCylinderProxyShape;

        // Collision callback class
        WorldCollisionCallback mCollisionCallback;

    public :

        // ---------- Methods ---------- //

        /// Constructor
        TestCollisionWorld(const std::string& name) : Test(name) {

            // Create the world
            mWorld = new CollisionWorld();

            // Create the bodies
            Transform boxTransform(Vector3(10, 0, 0), Quaternion::identity());
            mBoxBody = mWorld->createCollisionBody(boxTransform);
            mBoxShape = new BoxShape(Vector3(3, 3, 3));
            mBoxProxyShape = mBoxBody->addCollisionShape(mBoxShape, Transform::identity());

            mSphereShape = new SphereShape(3.0);
            Transform sphere1Transform(Vector3(10,5, 0), Quaternion::identity());
            mSphere1Body = mWorld->createCollisionBody(sphere1Transform);
            mSphere1ProxyShape = mSphere1Body->addCollisionShape(mSphereShape, Transform::identity());

            Transform sphere2Transform(Vector3(30, 10, 10), Quaternion::identity());
            mSphere2Body = mWorld->createCollisionBody(sphere2Transform);
            mSphere2ProxyShape = mSphere2Body->addCollisionShape(mSphereShape, Transform::identity());

            Transform cylinderTransform(Vector3(10, -5, 0), Quaternion::identity());
            mCylinderBody = mWorld->createCollisionBody(cylinderTransform);
            mCylinderShape = new CylinderShape(2, 5);
            mCylinderProxyShape = mCylinderBody->addCollisionShape(mCylinderShape, Transform::identity());

            // Assign collision categories to proxy shapes
            mBoxProxyShape->setCollisionCategoryBits(CATEGORY_1);
            mSphere1ProxyShape->setCollisionCategoryBits(CATEGORY_1);
            mSphere2ProxyShape->setCollisionCategoryBits(CATEGORY_2);
            mCylinderProxyShape->setCollisionCategoryBits(CATEGORY_3);

            mCollisionCallback.boxBody = mBoxBody;
            mCollisionCallback.sphere1Body = mSphere1Body;
            mCollisionCallback.sphere2Body = mSphere2Body;
            mCollisionCallback.cylinderBody = mCylinderBody;
        }

        /// Destructor
        ~TestCollisionWorld() {
            delete mBoxShape;
            delete mSphereShape;
            delete mCylinderShape;
        }

        /// Run the tests
        void run() {

            testCollisions();
        }

        void testCollisions() {

            mCollisionCallback.reset();
            mWorld->testCollision(&mCollisionCallback);
            test(mCollisionCallback.boxCollideWithSphere1);
            test(mCollisionCallback.boxCollideWithCylinder);
            test(!mCollisionCallback.sphere1CollideWithCylinder);
            test(!mCollisionCallback.sphere1CollideWithSphere2);

            test(mWorld->testAABBOverlap(mBoxBody, mSphere1Body));
            test(mWorld->testAABBOverlap(mBoxBody, mCylinderBody));
            test(!mWorld->testAABBOverlap(mSphere1Body, mCylinderBody));
            test(!mWorld->testAABBOverlap(mSphere1Body, mSphere2Body));

            test(mWorld->testAABBOverlap(mBoxProxyShape, mSphere1ProxyShape));
            test(mWorld->testAABBOverlap(mBoxProxyShape, mCylinderProxyShape));
            test(!mWorld->testAABBOverlap(mSphere1ProxyShape, mCylinderProxyShape));
            test(!mWorld->testAABBOverlap(mSphere1ProxyShape, mSphere2ProxyShape));

            mCollisionCallback.reset();
            mWorld->testCollision(mCylinderBody, &mCollisionCallback);
            test(!mCollisionCallback.boxCollideWithSphere1);
            test(mCollisionCallback.boxCollideWithCylinder);
            test(!mCollisionCallback.sphere1CollideWithCylinder);
            test(!mCollisionCallback.sphere1CollideWithSphere2);

            mCollisionCallback.reset();
            mWorld->testCollision(mBoxBody, mSphere1Body, &mCollisionCallback);
            test(mCollisionCallback.boxCollideWithSphere1);
            test(!mCollisionCallback.boxCollideWithCylinder);
            test(!mCollisionCallback.sphere1CollideWithCylinder);
            test(!mCollisionCallback.sphere1CollideWithSphere2);

            mCollisionCallback.reset();
            mWorld->testCollision(mBoxBody, mCylinderBody, &mCollisionCallback);
            test(!mCollisionCallback.boxCollideWithSphere1);
            test(mCollisionCallback.boxCollideWithCylinder);
            test(!mCollisionCallback.sphere1CollideWithCylinder);
            test(!mCollisionCallback.sphere1CollideWithSphere2);

            mCollisionCallback.reset();
            mWorld->testCollision(mCylinderProxyShape, &mCollisionCallback);
            test(!mCollisionCallback.boxCollideWithSphere1);
            test(mCollisionCallback.boxCollideWithCylinder);
            test(!mCollisionCallback.sphere1CollideWithCylinder);
            test(!mCollisionCallback.sphere1CollideWithSphere2);

            mCollisionCallback.reset();
            mWorld->testCollision(mBoxProxyShape, mCylinderProxyShape, &mCollisionCallback);
            test(!mCollisionCallback.boxCollideWithSphere1);
            test(mCollisionCallback.boxCollideWithCylinder);
            test(!mCollisionCallback.sphere1CollideWithCylinder);
            test(!mCollisionCallback.sphere1CollideWithSphere2);

            // Move sphere 1 to collide with sphere 2
            mSphere1Body->setTransform(Transform(Vector3(30, 15, 10), Quaternion::identity()));

            mCollisionCallback.reset();
            mWorld->testCollision(&mCollisionCallback);
            test(!mCollisionCallback.boxCollideWithSphere1);
            test(mCollisionCallback.boxCollideWithCylinder);
            test(!mCollisionCallback.sphere1CollideWithCylinder);
            test(mCollisionCallback.sphere1CollideWithSphere2);

            mCollisionCallback.reset();
            mWorld->testCollision(mBoxBody, mSphere1Body, &mCollisionCallback);
            test(!mCollisionCallback.boxCollideWithSphere1);
            test(!mCollisionCallback.boxCollideWithCylinder);
            test(!mCollisionCallback.sphere1CollideWithCylinder);
            test(!mCollisionCallback.sphere1CollideWithSphere2);

            mCollisionCallback.reset();
            mWorld->testCollision(mBoxBody, mCylinderBody, &mCollisionCallback);
            test(!mCollisionCallback.boxCollideWithSphere1);
            test(mCollisionCallback.boxCollideWithCylinder);
            test(!mCollisionCallback.sphere1CollideWithCylinder);
            test(!mCollisionCallback.sphere1CollideWithSphere2);

            // Move sphere 1 to collide with box
            mSphere1Body->setTransform(Transform(Vector3(10, 5, 0), Quaternion::identity()));

            // --------- Test collision with inactive bodies --------- //

            mCollisionCallback.reset();
            mBoxBody->setIsActive(false);
            mCylinderBody->setIsActive(false);
            mSphere1Body->setIsActive(false);
            mSphere2Body->setIsActive(false);
            mWorld->testCollision(&mCollisionCallback);
            test(!mCollisionCallback.boxCollideWithSphere1);
            test(!mCollisionCallback.boxCollideWithCylinder);
            test(!mCollisionCallback.sphere1CollideWithCylinder);
            test(!mCollisionCallback.sphere1CollideWithSphere2);

            test(!mWorld->testAABBOverlap(mBoxBody, mSphere1Body));
            test(!mWorld->testAABBOverlap(mBoxBody, mCylinderBody));
            test(!mWorld->testAABBOverlap(mSphere1Body, mCylinderBody));
            test(!mWorld->testAABBOverlap(mSphere1Body, mSphere2Body));

            test(!mWorld->testAABBOverlap(mBoxProxyShape, mSphere1ProxyShape));
            test(!mWorld->testAABBOverlap(mBoxProxyShape, mCylinderProxyShape));
            test(!mWorld->testAABBOverlap(mSphere1ProxyShape, mCylinderProxyShape));
            test(!mWorld->testAABBOverlap(mSphere1ProxyShape, mSphere2ProxyShape));

            mBoxBody->setIsActive(true);
            mCylinderBody->setIsActive(true);
            mSphere1Body->setIsActive(true);
            mSphere2Body->setIsActive(true);

            // --------- Test collision with collision filtering -------- //

            mBoxProxyShape->setCollideWithMaskBits(CATEGORY_1 | CATEGORY_3);
            mSphere1ProxyShape->setCollideWithMaskBits(CATEGORY_1 | CATEGORY_2);
            mSphere2ProxyShape->setCollideWithMaskBits(CATEGORY_1);
            mCylinderProxyShape->setCollideWithMaskBits(CATEGORY_1);

            mCollisionCallback.reset();
            mWorld->testCollision(&mCollisionCallback);
            test(mCollisionCallback.boxCollideWithSphere1);
            test(mCollisionCallback.boxCollideWithCylinder);
            test(!mCollisionCallback.sphere1CollideWithCylinder);
            test(!mCollisionCallback.sphere1CollideWithSphere2);

            // Move sphere 1 to collide with sphere 2
            mSphere1Body->setTransform(Transform(Vector3(30, 15, 10), Quaternion::identity()));

            mCollisionCallback.reset();
            mWorld->testCollision(&mCollisionCallback);
            test(!mCollisionCallback.boxCollideWithSphere1);
            test(mCollisionCallback.boxCollideWithCylinder);
            test(!mCollisionCallback.sphere1CollideWithCylinder);
            test(mCollisionCallback.sphere1CollideWithSphere2);

            mBoxProxyShape->setCollideWithMaskBits(CATEGORY_2);
            mSphere1ProxyShape->setCollideWithMaskBits(CATEGORY_2);
            mSphere2ProxyShape->setCollideWithMaskBits(CATEGORY_3);
            mCylinderProxyShape->setCollideWithMaskBits(CATEGORY_1);

            mCollisionCallback.reset();
            mWorld->testCollision(&mCollisionCallback);
            test(!mCollisionCallback.boxCollideWithSphere1);
            test(!mCollisionCallback.boxCollideWithCylinder);
            test(!mCollisionCallback.sphere1CollideWithCylinder);
            test(!mCollisionCallback.sphere1CollideWithSphere2);

            // Move sphere 1 to collide with box
            mSphere1Body->setTransform(Transform(Vector3(10, 5, 0), Quaternion::identity()));

            mBoxProxyShape->setCollideWithMaskBits(0xFFFF);
            mSphere1ProxyShape->setCollideWithMaskBits(0xFFFF);
            mSphere2ProxyShape->setCollideWithMaskBits(0xFFFF);
            mCylinderProxyShape->setCollideWithMaskBits(0xFFFF);
        }
 };

}

#endif
