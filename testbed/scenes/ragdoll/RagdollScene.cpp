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

// Libraries
#include "RagdollScene.h"
#include <cmath>

// Namespaces
using namespace openglframework;
using namespace ragdollscene;

// Constructor
RagdollScene::RagdollScene(const std::string& name, EngineSettings& settings, reactphysics3d::PhysicsCommon& physicsCommon)
      : SceneDemo(name, settings, physicsCommon, true, SCENE_RADIUS) {

    // Compute the radius and the center of the scene
    openglframework::Vector3 center(0, 5, 0);

    // Set the center of the scene
    setScenePosition(center, SCENE_RADIUS);

    mWorldSettings.worldName = name;
}

// Destructor
RagdollScene::~RagdollScene() {

    destroyPhysicsWorld();
}

// Create the physics world
void RagdollScene::createPhysicsWorld() {

    // Gravity vector in the physics world
    mWorldSettings.gravity = rp3d::Vector3(mEngineSettings.gravity.x, mEngineSettings.gravity.y, mEngineSettings.gravity.z);

    // Create the physics world for the physics simulation
    mPhysicsWorld = mPhysicsCommon.createPhysicsWorld(mWorldSettings);
    mPhysicsWorld->setEventListener(this);

    // Create the ragdoll
    createRagdolls();

    // ------------------------- FLOOR 1 ----------------------- //

    // Create the floor
    mFloor1 = new Box(true, FLOOR_1_SIZE, mPhysicsCommon, mPhysicsWorld, mMeshFolderPath);
    mFloor1->setTransform(rp3d::Transform(rp3d::Vector3(0, 5, -4), rp3d::Quaternion::identity()));
    mFloor1->setColor(mFloorColorDemo);
    mFloor1->setSleepingColor(mFloorColorDemo);
    mFloor1->getRigidBody()->setType(rp3d::BodyType::STATIC);
    mPhysicsObjects.push_back(mFloor1);

    // ------------------------- FLOOR 2 ----------------------- //

    // Create the floor
    mFloor2 = new Box(true, FLOOR_2_SIZE, mPhysicsCommon, mPhysicsWorld, mMeshFolderPath);
    mFloor2->setColor(mFloorColorDemo);
    mFloor2->setTransform(rp3d::Transform(rp3d::Vector3(0, -10, 0), rp3d::Quaternion::identity()));
    mFloor2->setSleepingColor(mFloorColorDemo);
    mFloor2->getRigidBody()->setType(rp3d::BodyType::STATIC);
    mPhysicsObjects.push_back(mFloor2);

    // ------------------------- Large Box ----------------------- //

    mLargeBox = new Box(true, Vector3(36, 15, 18), mPhysicsCommon, mPhysicsWorld, mMeshFolderPath);
    mLargeBox->setTransform(rp3d::Transform(rp3d::Vector3(0, 10, -14), rp3d::Quaternion::identity()));
    mLargeBox->setColor(mFloorColorDemo);
    mLargeBox->setSleepingColor(mFloorColorDemo);
    mLargeBox->getRigidBody()->setType(rp3d::BodyType::STATIC);
    mPhysicsObjects.push_back(mLargeBox);

    // ------------------------- Inclined Plane Box ----------------------- //

    mInclinedPlaneBox = new Box(true, Vector3(36, 1, 25), mPhysicsCommon, mPhysicsWorld, mMeshFolderPath);
    const rp3d::decimal planeAngle = 30 * rp3d::PI_RP3D / 180.0f;
    mInclinedPlaneBox->setTransform(rp3d::Transform(rp3d::Vector3(0, 10.82, 5.56), rp3d::Quaternion::fromEulerAngles(planeAngle, 0, 0)));
    mInclinedPlaneBox->setColor(mFloorColorDemo);
    mInclinedPlaneBox->setSleepingColor(mFloorColorDemo);
    mInclinedPlaneBox->getRigidBody()->setType(rp3d::BodyType::STATIC);
    mPhysicsObjects.push_back(mInclinedPlaneBox);
}

// Initialize the bodies positions
void RagdollScene::initBodiesPositions() {

    // For each ragdoll
    for (int i=0; i < NB_RAGDOLLS; i++) {

        mHeadBox[i]->setTransform(rp3d::Transform(mHeadPos[i], rp3d::Quaternion::identity()));
        mTorsoBox[i]->setTransform(rp3d::Transform(mTorsoPos[i], rp3d::Quaternion::identity()));
        mLeftUpperArmBox[i]->setTransform(rp3d::Transform(mLeftUpperArmPos[i], rp3d::Quaternion::identity()));
        mLeftLowerArmBox[i]->setTransform(rp3d::Transform(mLeftLowerArmPos[i], rp3d::Quaternion::identity()));
        mLeftUpperLegBox[i]->setTransform(rp3d::Transform(mLeftUpperLegPos[i], rp3d::Quaternion::identity()));
        mLeftLowerLegBox[i]->setTransform(rp3d::Transform(mLeftLowerLegPos[i], rp3d::Quaternion::identity()));
        mRightUpperArmBox[i]->setTransform(rp3d::Transform(mRightUpperArmPos[i], rp3d::Quaternion::identity()));
        mRightLowerArmBox[i]->setTransform(rp3d::Transform(mRightLowerArmPos[i], rp3d::Quaternion::identity()));
        mRightUpperLegBox[i]->setTransform(rp3d::Transform(mRightUpperLegPos[i], rp3d::Quaternion::identity()));
        mRightLowerLegBox[i]->setTransform(rp3d::Transform(mRightLowerLegPos[i], rp3d::Quaternion::identity()));
    }
}

// Destroy the physics world
void RagdollScene::destroyPhysicsWorld() {

    if (mPhysicsWorld != nullptr) {

        // For each ragdoll
        for (int i=0; i < NB_RAGDOLLS; i++) {

            delete mHeadBox[i];
            delete mTorsoBox[i];
            delete mLeftUpperArmBox[i];
            delete mLeftLowerArmBox[i];
            delete mLeftUpperLegBox[i];
            delete mLeftLowerLegBox[i];
            delete mRightUpperArmBox[i];
            delete mRightLowerArmBox[i];
            delete mRightUpperLegBox[i];
            delete mRightLowerLegBox[i];
        }

        delete mFloor1;
        delete mFloor2;

        mPhysicsObjects.clear();

        mPhysicsCommon.destroyPhysicsWorld(mPhysicsWorld);
        mPhysicsWorld = nullptr;
    }
}
// Reset the scene
void RagdollScene::reset() {

    SceneDemo::reset();

    destroyPhysicsWorld();
    createPhysicsWorld();
    initBodiesPositions();
}

// Create the boxes and joints for the ragdoll
void RagdollScene::createRagdolls() {

    const float linearDamping = 0.02f;
    const float angularDamping = 0.02f;

    // For each ragdoll
    for (int i=0; i < NB_RAGDOLLS_ROWS; i++) {

        for (int j=0; j < NB_RAGDOLLS_COLS; j++) {

            int ragdollIndex = i * NB_RAGDOLLS_COLS + j;

            const float ragdollDist = 15;
            const float ragdollHeight = 31;

            rp3d::Vector3 ragdollPosition((-(NB_RAGDOLLS_ROWS-1)/2.0 + i) * ragdollDist , ragdollHeight, -11 + (-(NB_RAGDOLLS_COLS-1)/2.0 + j) * ragdollDist);

            // --------------- Create the head box --------------- //
            mHeadPos[ragdollIndex] = ragdollPosition;
            mHeadBox[ragdollIndex] = new Box(true, Vector3(1.5, 1.5, 1.5) ,  mPhysicsCommon, mPhysicsWorld, mMeshFolderPath);
            mHeadBox[ragdollIndex]->setTransform(rp3d::Transform(mHeadPos[ragdollIndex], rp3d::Quaternion::identity()));
            mHeadBox[ragdollIndex]->setColor(mObjectColorDemo);
            mHeadBox[ragdollIndex]->setSleepingColor(mSleepingColorDemo);
            mHeadBox[ragdollIndex]->getCollider()->getMaterial().setMassDensity(7);
            mHeadBox[ragdollIndex]->getRigidBody()->updateMassPropertiesFromColliders();
            mHeadBox[ragdollIndex]->getRigidBody()->setLinearDamping(linearDamping);
            mHeadBox[ragdollIndex]->getRigidBody()->setAngularDamping(angularDamping);
            mPhysicsObjects.push_back(mHeadBox[ragdollIndex]);

            // --------------- Create the torso box --------------- //
            mTorsoPos[ragdollIndex] = mHeadPos[ragdollIndex] + rp3d::Vector3(0, -3.25, 0);
            mTorsoBox[ragdollIndex] = new Box(true, Vector3(3, 4, 1) ,  mPhysicsCommon, mPhysicsWorld, mMeshFolderPath);
            mTorsoBox[ragdollIndex]->setTransform(rp3d::Transform(mTorsoPos[ragdollIndex], rp3d::Quaternion::identity()));
            mTorsoBox[ragdollIndex]->setColor(mObjectColorDemo);
            mTorsoBox[ragdollIndex]->setSleepingColor(mSleepingColorDemo);
            mTorsoBox[ragdollIndex]->getCollider()->getMaterial().setMassDensity(9);
            mTorsoBox[ragdollIndex]->getRigidBody()->updateMassPropertiesFromColliders();
            mTorsoBox[ragdollIndex]->getRigidBody()->setLinearDamping(linearDamping);
            mTorsoBox[ragdollIndex]->getRigidBody()->setAngularDamping(angularDamping);
            //mTorsoBox->getRigidBody()->setType(rp3d::BodyType::STATIC);
            mPhysicsObjects.push_back(mTorsoBox[ragdollIndex]);

            // --------------- Create the left upper arm box --------------- //
            mLeftUpperArmPos[ragdollIndex] = mTorsoPos[ragdollIndex] + rp3d::Vector3(2.75, 1.5, 0);
            mLeftUpperArmBox[ragdollIndex] = new Box(true, Vector3(2, 1, 1) ,  mPhysicsCommon, mPhysicsWorld, mMeshFolderPath);
            mLeftUpperArmBox[ragdollIndex]->setTransform(rp3d::Transform(mLeftUpperArmPos[ragdollIndex], rp3d::Quaternion::identity()));
            mLeftUpperArmBox[ragdollIndex]->setColor(mObjectColorDemo);
            mLeftUpperArmBox[ragdollIndex]->setSleepingColor(mSleepingColorDemo);
            mLeftUpperArmBox[ragdollIndex]->getCollider()->getMaterial().setMassDensity(8);
            mLeftUpperArmBox[ragdollIndex]->getRigidBody()->updateMassPropertiesFromColliders();
            mLeftUpperArmBox[ragdollIndex]->getRigidBody()->setLinearDamping(linearDamping);
            mLeftUpperArmBox[ragdollIndex]->getRigidBody()->setAngularDamping(angularDamping);
            mPhysicsObjects.push_back(mLeftUpperArmBox[ragdollIndex]);

            // --------------- Create the left lower arm box --------------- //
            mLeftLowerArmPos[ragdollIndex] = mLeftUpperArmPos[ragdollIndex] + rp3d::Vector3(2.25, 0, 0);
            mLeftLowerArmBox[ragdollIndex] = new Box(true, Vector3(2, 1, 1) ,  mPhysicsCommon, mPhysicsWorld, mMeshFolderPath);
            mLeftLowerArmBox[ragdollIndex]->setTransform(rp3d::Transform(mLeftLowerArmPos[ragdollIndex], rp3d::Quaternion::identity()));
            mLeftLowerArmBox[ragdollIndex]->setColor(mObjectColorDemo);
            mLeftLowerArmBox[ragdollIndex]->setSleepingColor(mSleepingColorDemo);
            mLeftLowerArmBox[ragdollIndex]->getCollider()->getMaterial().setMassDensity(8);
            mLeftLowerArmBox[ragdollIndex]->getRigidBody()->updateMassPropertiesFromColliders();
            mLeftLowerArmBox[ragdollIndex]->getRigidBody()->setLinearDamping(linearDamping);
            mLeftLowerArmBox[ragdollIndex]->getRigidBody()->setAngularDamping(angularDamping);
            mPhysicsObjects.push_back(mLeftLowerArmBox[ragdollIndex]);

            // --------------- Create the left upper leg box --------------- //
            mLeftUpperLegPos[ragdollIndex] = mTorsoPos[ragdollIndex] + rp3d::Vector3(1, -3.75, 0);
            mLeftUpperLegBox[ragdollIndex] = new Box(true, Vector3(1, 3, 1) ,  mPhysicsCommon, mPhysicsWorld, mMeshFolderPath);
            mLeftUpperLegBox[ragdollIndex]->setTransform(rp3d::Transform(mLeftUpperLegPos[ragdollIndex], rp3d::Quaternion::identity()));
            mLeftUpperLegBox[ragdollIndex]->setColor(mObjectColorDemo);
            mLeftUpperLegBox[ragdollIndex]->setSleepingColor(mSleepingColorDemo);
            mLeftUpperLegBox[ragdollIndex]->getCollider()->getMaterial().setMassDensity(8);
            mLeftUpperLegBox[ragdollIndex]->getRigidBody()->updateMassPropertiesFromColliders();
            mLeftUpperLegBox[ragdollIndex]->getRigidBody()->setLinearDamping(linearDamping);
            mLeftUpperLegBox[ragdollIndex]->getRigidBody()->setAngularDamping(angularDamping);
            mPhysicsObjects.push_back(mLeftUpperLegBox[ragdollIndex]);

            // --------------- Create the left lower leg box --------------- //
            mLeftLowerLegPos[ragdollIndex] = mLeftUpperLegPos[ragdollIndex] + rp3d::Vector3(0, -3.25, 0);
            mLeftLowerLegBox[ragdollIndex] = new Box(true, Vector3(1, 3, 1) ,  mPhysicsCommon, mPhysicsWorld, mMeshFolderPath);
            mLeftLowerLegBox[ragdollIndex]->setTransform(rp3d::Transform(mLeftLowerLegPos[ragdollIndex], rp3d::Quaternion::identity()));
            mLeftLowerLegBox[ragdollIndex]->setColor(mObjectColorDemo);
            mLeftLowerLegBox[ragdollIndex]->setSleepingColor(mSleepingColorDemo);
            mLeftLowerLegBox[ragdollIndex]->getCollider()->getMaterial().setMassDensity(8);
            mLeftLowerLegBox[ragdollIndex]->getRigidBody()->updateMassPropertiesFromColliders();
            mLeftLowerLegBox[ragdollIndex]->getRigidBody()->setLinearDamping(linearDamping);
            mLeftLowerLegBox[ragdollIndex]->getRigidBody()->setAngularDamping(angularDamping);
            mPhysicsObjects.push_back(mLeftLowerLegBox[ragdollIndex]);

            // --------------- Create the right upper arm box --------------- //
            mRightUpperArmPos[ragdollIndex] = mTorsoPos[ragdollIndex] + rp3d::Vector3(-2.75, 1.5, 0);
            mRightUpperArmBox[ragdollIndex] = new Box(true, Vector3(2, 1, 1) ,  mPhysicsCommon, mPhysicsWorld, mMeshFolderPath);
            mRightUpperArmBox[ragdollIndex]->setTransform(rp3d::Transform(mRightUpperArmPos[ragdollIndex], rp3d::Quaternion::identity()));
            mRightUpperArmBox[ragdollIndex]->setColor(mObjectColorDemo);
            mRightUpperArmBox[ragdollIndex]->setSleepingColor(mSleepingColorDemo);
            mRightUpperArmBox[ragdollIndex]->getCollider()->getMaterial().setMassDensity(8);
            mRightUpperArmBox[ragdollIndex]->getRigidBody()->updateMassPropertiesFromColliders();
            mRightUpperArmBox[ragdollIndex]->getRigidBody()->setLinearDamping(linearDamping);
            mRightUpperArmBox[ragdollIndex]->getRigidBody()->setAngularDamping(angularDamping);
            mPhysicsObjects.push_back(mRightUpperArmBox[ragdollIndex]);

            // --------------- Create the right lower arm box --------------- //
            mRightLowerArmPos[ragdollIndex] = mRightUpperArmPos[ragdollIndex] + rp3d::Vector3(-2.25, 0, 0);
            mRightLowerArmBox[ragdollIndex] = new Box(true, Vector3(2, 1, 1) ,  mPhysicsCommon, mPhysicsWorld, mMeshFolderPath);
            mRightLowerArmBox[ragdollIndex]->setTransform(rp3d::Transform(mRightLowerArmPos[ragdollIndex], rp3d::Quaternion::identity()));
            mRightLowerArmBox[ragdollIndex]->setColor(mObjectColorDemo);
            mRightLowerArmBox[ragdollIndex]->setSleepingColor(mSleepingColorDemo);
            mRightLowerArmBox[ragdollIndex]->getCollider()->getMaterial().setMassDensity(8);
            mRightLowerArmBox[ragdollIndex]->getRigidBody()->updateMassPropertiesFromColliders();
            mRightLowerArmBox[ragdollIndex]->getRigidBody()->setLinearDamping(linearDamping);
            mRightLowerArmBox[ragdollIndex]->getRigidBody()->setAngularDamping(angularDamping);
            mPhysicsObjects.push_back(mRightLowerArmBox[ragdollIndex]);

            // --------------- Create the right upper leg box --------------- //
            mRightUpperLegPos[ragdollIndex] = mTorsoPos[ragdollIndex] + rp3d::Vector3(-1, -3.75, 0);
            mRightUpperLegBox[ragdollIndex] = new Box(true, Vector3(1, 3, 1) ,  mPhysicsCommon, mPhysicsWorld, mMeshFolderPath);
            mRightUpperLegBox[ragdollIndex]->setTransform(rp3d::Transform(mRightUpperLegPos[ragdollIndex], rp3d::Quaternion::identity()));
            mRightUpperLegBox[ragdollIndex]->setColor(mObjectColorDemo);
            mRightUpperLegBox[ragdollIndex]->setSleepingColor(mSleepingColorDemo);
            mRightUpperLegBox[ragdollIndex]->getCollider()->getMaterial().setMassDensity(8);
            mRightUpperLegBox[ragdollIndex]->getRigidBody()->updateMassPropertiesFromColliders();
            mRightUpperLegBox[ragdollIndex]->getRigidBody()->setLinearDamping(linearDamping);
            mRightUpperLegBox[ragdollIndex]->getRigidBody()->setAngularDamping(angularDamping);
            mPhysicsObjects.push_back(mRightUpperLegBox[ragdollIndex]);

            // --------------- Create the right lower leg box --------------- //
            mRightLowerLegPos[ragdollIndex] = mRightUpperLegPos[ragdollIndex] + rp3d::Vector3(0, -3.25, 0);
            mRightLowerLegBox[ragdollIndex] = new Box(true, Vector3(1, 3, 1) ,  mPhysicsCommon, mPhysicsWorld, mMeshFolderPath);
            mRightLowerLegBox[ragdollIndex]->setTransform(rp3d::Transform(mRightLowerLegPos[ragdollIndex], rp3d::Quaternion::identity()));
            mRightLowerLegBox[ragdollIndex]->setColor(mObjectColorDemo);
            mRightLowerLegBox[ragdollIndex]->setSleepingColor(mSleepingColorDemo);
            mRightLowerLegBox[ragdollIndex]->getCollider()->getMaterial().setMassDensity(8);
            mRightLowerLegBox[ragdollIndex]->getRigidBody()->updateMassPropertiesFromColliders();
            mRightLowerLegBox[ragdollIndex]->getRigidBody()->setLinearDamping(linearDamping);
            mRightLowerLegBox[ragdollIndex]->getRigidBody()->setAngularDamping(angularDamping);
            mPhysicsObjects.push_back(mRightLowerLegBox[ragdollIndex]);

            // --------------- Create the joint between head and torso --------------- //

            // Create the joint info object
            rp3d::RigidBody* body1 = mHeadBox[ragdollIndex]->getRigidBody();
            rp3d::RigidBody* body2 = mTorsoBox[ragdollIndex]->getRigidBody();
            rp3d::BallAndSocketJointInfo jointInfo1(body1, body2, mHeadPos[ragdollIndex] + rp3d::Vector3(0, -0.82, 0));
            jointInfo1.isCollisionEnabled = false;
            mHeadTorsoJoint[ragdollIndex] = dynamic_cast<rp3d::BallAndSocketJoint*>(mPhysicsWorld->createJoint(jointInfo1));
            mHeadTorsoJoint[ragdollIndex]->setConeLimitLocalAxisBody1(rp3d::Vector3(0, 1, 0));
            mHeadTorsoJoint[ragdollIndex]->setConeLimitLocalAxisBody2(rp3d::Vector3(0, 1, 0));
            mHeadTorsoJoint[ragdollIndex]->setConeLimitHalfAngle(40.0 * rp3d::PI_RP3D / 180.0);
            mHeadTorsoJoint[ragdollIndex]->enableConeLimit(true);

            // --------------- Create the joint between torso and left upper arm --------------- //

            // Create the joint info object
            body1 = mTorsoBox[ragdollIndex]->getRigidBody();
            body2 = mLeftUpperArmBox[ragdollIndex]->getRigidBody();
            rp3d::BallAndSocketJointInfo jointInfo2(body1, body2, mLeftUpperArmPos[ragdollIndex] + rp3d::Vector3(-0.5, 0, 0));
            jointInfo2.isCollisionEnabled = false;
            mTorsoLeftUpperArmJoint[ragdollIndex] = dynamic_cast<rp3d::BallAndSocketJoint*>(mPhysicsWorld->createJoint(jointInfo2));
            mTorsoLeftUpperArmJoint[ragdollIndex]->setConeLimitLocalAxisBody1(rp3d::Vector3(1, 0, 0));
            mTorsoLeftUpperArmJoint[ragdollIndex]->setConeLimitLocalAxisBody2(rp3d::Vector3(1, 0, 0));
            mTorsoLeftUpperArmJoint[ragdollIndex]->setConeLimitHalfAngle(90.0 * rp3d::PI_RP3D / 180.0);
            mTorsoLeftUpperArmJoint[ragdollIndex]->enableConeLimit(true);

            // --------------- Create the joint between left upper arm and left lower arm  --------------- //

            // Create the joint info object
            body1 = mLeftUpperArmBox[ragdollIndex]->getRigidBody();
            body2 = mLeftLowerArmBox[ragdollIndex]->getRigidBody();
            rp3d::Vector3 joint2WorldAnchor = (body1->getTransform().getPosition() + body2->getTransform().getPosition()) * 0.5f;
            rp3d::Vector3 joint2WorldAxis(0, 0, 1);
            rp3d::HingeJointInfo jointInfo3(body1, body2, joint2WorldAnchor, joint2WorldAxis);
            jointInfo3.isCollisionEnabled = false;
            mLeftUpperLeftLowerArmJoint[ragdollIndex] = dynamic_cast<rp3d::HingeJoint*>(mPhysicsWorld->createJoint(jointInfo3));

            // --------------- Create the joint between torso and left upper leg --------------- //

            // Create the joint info object
            body1 = mTorsoBox[ragdollIndex]->getRigidBody();
            body2 = mLeftUpperLegBox[ragdollIndex]->getRigidBody();
            rp3d::BallAndSocketJointInfo jointInfo4(body1, body2, mTorsoPos[ragdollIndex] + rp3d::Vector3(1, -1.87, 0));
            jointInfo4.isCollisionEnabled = false;
            mTorsoLeftUpperLegJoint[ragdollIndex] = dynamic_cast<rp3d::BallAndSocketJoint*>(mPhysicsWorld->createJoint(jointInfo4));
            mTorsoLeftUpperLegJoint[ragdollIndex]->setConeLimitLocalAxisBody1(rp3d::Vector3(0, 1, 0));
            mTorsoLeftUpperLegJoint[ragdollIndex]->setConeLimitLocalAxisBody2(rp3d::Vector3(0, 1, 0));
            mTorsoLeftUpperLegJoint[ragdollIndex]->setConeLimitHalfAngle(90.0 * rp3d::PI_RP3D / 180.0);
            mTorsoLeftUpperLegJoint[ragdollIndex]->enableConeLimit(true);

            // --------------- Create the joint between left upper leg and left lower leg  --------------- //

            // Create the joint info object
            body1 = mLeftUpperLegBox[ragdollIndex]->getRigidBody();
            body2 = mLeftLowerLegBox[ragdollIndex]->getRigidBody();
            rp3d::Vector3 joint5WorldAnchor = (body1->getTransform().getPosition() + body2->getTransform().getPosition()) * 0.5f;
            rp3d::Vector3 joint5WorldAxis(1, 0, 0);
            const rp3d::decimal joint5MinAngle = 0.0 * rp3d::PI_RP3D / 180.0;
            const rp3d::decimal joint5MaxAngle = 110.0 * rp3d::PI_RP3D / 180.0;
            rp3d::HingeJointInfo jointInfo5(body1, body2, joint5WorldAnchor, joint5WorldAxis, joint5MinAngle, joint5MaxAngle);
            jointInfo5.isCollisionEnabled = false;
            mLeftUpperLeftLowerLegJoint[ragdollIndex] = dynamic_cast<rp3d::HingeJoint*>(mPhysicsWorld->createJoint(jointInfo5));

            // --------------- Create the joint between torso and right upper arm --------------- //

            // Create the joint info object
            body1 = mTorsoBox[ragdollIndex]->getRigidBody();
            body2 = mRightUpperArmBox[ragdollIndex]->getRigidBody();
            rp3d::BallAndSocketJointInfo jointInfo6(body1, body2, mRightUpperArmPos[ragdollIndex] + rp3d::Vector3(0.5, 0, 0));
            jointInfo6.isCollisionEnabled = false;
            mTorsoRightUpperArmJoint[ragdollIndex] = dynamic_cast<rp3d::BallAndSocketJoint*>(mPhysicsWorld->createJoint(jointInfo6));
            mTorsoRightUpperArmJoint[ragdollIndex]->setConeLimitLocalAxisBody1(rp3d::Vector3(1, 0, 0));
            mTorsoRightUpperArmJoint[ragdollIndex]->setConeLimitLocalAxisBody2(rp3d::Vector3(1, 0, 0));
            mTorsoRightUpperArmJoint[ragdollIndex]->setConeLimitHalfAngle(90.0 * rp3d::PI_RP3D / 180.0);
            mTorsoRightUpperArmJoint[ragdollIndex]->enableConeLimit(true);

            // --------------- Create the joint between right upper arm and right lower arm  --------------- //

            // Create the joint info object
            body1 = mRightUpperArmBox[ragdollIndex]->getRigidBody();
            body2 = mRightLowerArmBox[ragdollIndex]->getRigidBody();
            rp3d::Vector3 joint7WorldAnchor = (body1->getTransform().getPosition() + body2->getTransform().getPosition()) * 0.5f;
            rp3d::Vector3 joint7WorldAxis(0, 0, 1);
            rp3d::HingeJointInfo jointInfo7(body1, body2, joint7WorldAnchor, joint7WorldAxis);
            jointInfo7.isCollisionEnabled = false;
            mRightUpperRightLowerArmJoint[ragdollIndex] = dynamic_cast<rp3d::HingeJoint*>(mPhysicsWorld->createJoint(jointInfo7));

            // --------------- Create the joint between torso and right upper leg --------------- //

            // Create the joint info object
            body1 = mTorsoBox[ragdollIndex]->getRigidBody();
            body2 = mRightUpperLegBox[ragdollIndex]->getRigidBody();
            rp3d::BallAndSocketJointInfo jointInfo8(body1, body2, mTorsoPos[ragdollIndex] + rp3d::Vector3(-1, -1.87, 0));
            jointInfo8.isCollisionEnabled = false;
            mTorsoRightUpperLegJoint[ragdollIndex] = dynamic_cast<rp3d::BallAndSocketJoint*>(mPhysicsWorld->createJoint(jointInfo8));
            mTorsoRightUpperLegJoint[ragdollIndex]->setConeLimitLocalAxisBody1(rp3d::Vector3(0, 1, 0));
            mTorsoRightUpperLegJoint[ragdollIndex]->setConeLimitLocalAxisBody2(rp3d::Vector3(0, 1, 0));
            mTorsoRightUpperLegJoint[ragdollIndex]->setConeLimitHalfAngle(90.0 * rp3d::PI_RP3D / 180.0);
            mTorsoRightUpperLegJoint[ragdollIndex]->enableConeLimit(true);

            // --------------- Create the joint between right upper leg and right lower leg  --------------- //

            // Create the joint info object
            body1 = mRightUpperLegBox[ragdollIndex]->getRigidBody();
            body2 = mRightLowerLegBox[ragdollIndex]->getRigidBody();
            rp3d::Vector3 joint9WorldAnchor = (body1->getTransform().getPosition() + body2->getTransform().getPosition()) * 0.5f;
            rp3d::Vector3 joint9WorldAxis(1, 0, 0);
            const rp3d::decimal joint9MinAngle = 0.0 * rp3d::PI_RP3D / 180.0;
            const rp3d::decimal joint9MaxAngle = 110.0 * rp3d::PI_RP3D / 180.0;
            rp3d::HingeJointInfo jointInfo9(body1, body2, joint9WorldAnchor, joint9WorldAxis, joint9MinAngle, joint9MaxAngle);
            jointInfo9.isCollisionEnabled = false;
            mRightUpperRightLowerLegJoint[ragdollIndex] = dynamic_cast<rp3d::HingeJoint*>(mPhysicsWorld->createJoint(jointInfo9));
        }
    }
}
