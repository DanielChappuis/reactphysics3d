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
    openglframework::Vector3 center(0, 10, 0);

    // Set the center of the scene
    setScenePosition(center, SCENE_RADIUS);
    setInitZoom(2.1);
    resetCameraToViewAll();

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
        mChestCapsule[i]->setTransform(rp3d::Transform(mChestPos[i], rp3d::Quaternion::fromEulerAngles(0, 0, rp3d::PI_RP3D / 2.0)));
        mWaistCapsule[i]->setTransform(rp3d::Transform(mWaistPos[i], rp3d::Quaternion::identity()));
        mHipCapsule[i]->setTransform(rp3d::Transform(mHipPos[i], rp3d::Quaternion::fromEulerAngles(0, 0, rp3d::PI_RP3D / 2.0)));
        mLeftUpperArmCapsule[i]->setTransform(rp3d::Transform(mLeftUpperArmPos[i], rp3d::Quaternion::fromEulerAngles(0, 0, rp3d::PI_RP3D / 2.0)));
        mLeftLowerArmCapsule[i]->setTransform(rp3d::Transform(mLeftLowerArmPos[i], rp3d::Quaternion::fromEulerAngles(0, 0, rp3d::PI_RP3D / 2.0)));
        mLeftUpperLegCapsule[i]->setTransform(rp3d::Transform(mLeftUpperLegPos[i], rp3d::Quaternion::identity()));
        mLeftLowerLegCapsule[i]->setTransform(rp3d::Transform(mLeftLowerLegPos[i], rp3d::Quaternion::identity()));
        mRightUpperArmCapsule[i]->setTransform(rp3d::Transform(mRightUpperArmPos[i], rp3d::Quaternion::fromEulerAngles(0, 0, rp3d::PI_RP3D / 2.0)));
        mRightLowerArmCapsule[i]->setTransform(rp3d::Transform(mRightLowerArmPos[i], rp3d::Quaternion::fromEulerAngles(0, 0, rp3d::PI_RP3D / 2.0)));
        mRightUpperLegCapsule[i]->setTransform(rp3d::Transform(mRightUpperLegPos[i], rp3d::Quaternion::identity()));
        mRightLowerLegCapsule[i]->setTransform(rp3d::Transform(mRightLowerLegPos[i], rp3d::Quaternion::identity()));
    }
}

// Destroy the physics world
void RagdollScene::destroyPhysicsWorld() {

    if (mPhysicsWorld != nullptr) {

        // For each ragdoll
        for (int i=0; i < NB_RAGDOLLS; i++) {

            delete mHeadBox[i];
            delete mChestCapsule[i];
            delete mWaistCapsule[i];
            delete mHipCapsule[i];
            delete mLeftUpperArmCapsule[i];
            delete mLeftLowerArmCapsule[i];
            delete mLeftUpperLegCapsule[i];
            delete mLeftLowerLegCapsule[i];
            delete mRightUpperArmCapsule[i];
            delete mRightLowerArmCapsule[i];
            delete mRightUpperLegCapsule[i];
            delete mRightLowerLegCapsule[i];
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
    const float frictionCoeff = 0.4f;

    // For each ragdoll
    for (int i=0; i < NB_RAGDOLLS_ROWS; i++) {

        for (int j=0; j < NB_RAGDOLLS_COLS; j++) {

            int ragdollIndex = i * NB_RAGDOLLS_COLS + j;

            const float ragdollDistX = 17;
            const float ragdollDistZ = 16;
            const float ragdollHeight = 31;

            rp3d::Vector3 ragdollPosition((-(NB_RAGDOLLS_ROWS-1)/2.0 + i) * ragdollDistX , ragdollHeight, -6 + (-(NB_RAGDOLLS_COLS-1)/2.0 + j) * ragdollDistZ);

            // --------------- Create the head box --------------- //
            mHeadPos[ragdollIndex] = ragdollPosition;
            mHeadBox[ragdollIndex] = new Sphere(true, 0.75f,  mPhysicsCommon, mPhysicsWorld, mMeshFolderPath);
            mHeadBox[ragdollIndex]->setTransform(rp3d::Transform(mHeadPos[ragdollIndex], rp3d::Quaternion::identity()));
            mHeadBox[ragdollIndex]->setColor(mObjectColorDemo);
            mHeadBox[ragdollIndex]->setSleepingColor(mSleepingColorDemo);
            mHeadBox[ragdollIndex]->getCollider()->getMaterial().setMassDensity(7);
            mHeadBox[ragdollIndex]->getCollider()->getMaterial().setFrictionCoefficient(frictionCoeff);
            mHeadBox[ragdollIndex]->getRigidBody()->updateMassPropertiesFromColliders();
            mHeadBox[ragdollIndex]->getRigidBody()->setLinearDamping(linearDamping);
            mHeadBox[ragdollIndex]->getRigidBody()->setAngularDamping(angularDamping);
            mPhysicsObjects.push_back(mHeadBox[ragdollIndex]);

            // --------------- Create the chest capsule --------------- //
            mChestPos[ragdollIndex] = mHeadPos[ragdollIndex] + rp3d::Vector3(0, -1.75, 0);
            mChestCapsule[ragdollIndex] = new Capsule(true, 1, 1.5,  mPhysicsCommon, mPhysicsWorld, mMeshFolderPath);
            mChestCapsule[ragdollIndex]->setTransform(rp3d::Transform(mChestPos[ragdollIndex], rp3d::Quaternion::fromEulerAngles(0, 0, rp3d::PI_RP3D / 2.0)));
            mChestCapsule[ragdollIndex]->setColor(mObjectColorDemo);
            mChestCapsule[ragdollIndex]->setSleepingColor(mSleepingColorDemo);
            mChestCapsule[ragdollIndex]->getCollider()->getMaterial().setMassDensity(9);
            mChestCapsule[ragdollIndex]->getCollider()->getMaterial().setFrictionCoefficient(frictionCoeff);
            mChestCapsule[ragdollIndex]->getRigidBody()->updateMassPropertiesFromColliders();
            mChestCapsule[ragdollIndex]->getRigidBody()->setLinearDamping(linearDamping);
            mChestCapsule[ragdollIndex]->getRigidBody()->setAngularDamping(angularDamping);
            mPhysicsObjects.push_back(mChestCapsule[ragdollIndex]);

            // --------------- Create the waist capsule --------------- //
            mWaistPos[ragdollIndex] = mChestPos[ragdollIndex] + rp3d::Vector3(0, -2, 0);
            mWaistCapsule[ragdollIndex] = new Capsule(true, 1, 1.5,  mPhysicsCommon, mPhysicsWorld, mMeshFolderPath);
            mWaistCapsule[ragdollIndex]->setTransform(rp3d::Transform(mWaistPos[ragdollIndex], rp3d::Quaternion::identity()));
            mWaistCapsule[ragdollIndex]->setColor(mObjectColorDemo);
            mWaistCapsule[ragdollIndex]->setSleepingColor(mSleepingColorDemo);
            mWaistCapsule[ragdollIndex]->getCollider()->getMaterial().setMassDensity(9);
            mWaistCapsule[ragdollIndex]->getCollider()->getMaterial().setFrictionCoefficient(frictionCoeff);
            mWaistCapsule[ragdollIndex]->getRigidBody()->updateMassPropertiesFromColliders();
            mWaistCapsule[ragdollIndex]->getRigidBody()->setLinearDamping(linearDamping);
            mWaistCapsule[ragdollIndex]->getRigidBody()->setAngularDamping(angularDamping);
            //mWaistCapsule[ragdollIndex]->getRigidBody()->setType(rp3d::BodyType::STATIC);
            mPhysicsObjects.push_back(mWaistCapsule[ragdollIndex]);

            // --------------- Create the hips capsule --------------- //
            mHipPos[ragdollIndex] = mWaistPos[ragdollIndex] + rp3d::Vector3(0, -2, 0);
            mHipCapsule[ragdollIndex] = new Capsule(true, 1, 1,  mPhysicsCommon, mPhysicsWorld, mMeshFolderPath);
            mHipCapsule[ragdollIndex]->setTransform(rp3d::Transform(mHipPos[ragdollIndex], rp3d::Quaternion::fromEulerAngles(0, 0, rp3d::PI_RP3D / 2.0)));
            mHipCapsule[ragdollIndex]->setColor(mObjectColorDemo);
            mHipCapsule[ragdollIndex]->setSleepingColor(mSleepingColorDemo);
            mHipCapsule[ragdollIndex]->getCollider()->getMaterial().setMassDensity(9);
            mHipCapsule[ragdollIndex]->getCollider()->getMaterial().setFrictionCoefficient(frictionCoeff);
            mHipCapsule[ragdollIndex]->getRigidBody()->updateMassPropertiesFromColliders();
            mHipCapsule[ragdollIndex]->getRigidBody()->setLinearDamping(linearDamping);
            mHipCapsule[ragdollIndex]->getRigidBody()->setAngularDamping(angularDamping);
            mPhysicsObjects.push_back(mHipCapsule[ragdollIndex]);

            // --------------- Create the left upper arm capsule --------------- //
            mLeftUpperArmPos[ragdollIndex] = mChestPos[ragdollIndex] + rp3d::Vector3(2.25, 0, 0);
            mLeftUpperArmCapsule[ragdollIndex] = new Capsule(true, 0.5, 2,  mPhysicsCommon, mPhysicsWorld, mMeshFolderPath);
            mLeftUpperArmCapsule[ragdollIndex]->setTransform(rp3d::Transform(mLeftUpperArmPos[ragdollIndex], rp3d::Quaternion::fromEulerAngles(0, 0, rp3d::PI_RP3D / 2.0)));
            mLeftUpperArmCapsule[ragdollIndex]->setColor(mObjectColorDemo);
            mLeftUpperArmCapsule[ragdollIndex]->setSleepingColor(mSleepingColorDemo);
            mLeftUpperArmCapsule[ragdollIndex]->getCollider()->getMaterial().setMassDensity(8);
            mLeftUpperArmCapsule[ragdollIndex]->getCollider()->getMaterial().setFrictionCoefficient(frictionCoeff);
            mLeftUpperArmCapsule[ragdollIndex]->getRigidBody()->updateMassPropertiesFromColliders();
            mLeftUpperArmCapsule[ragdollIndex]->getRigidBody()->setLinearDamping(linearDamping);
            mLeftUpperArmCapsule[ragdollIndex]->getRigidBody()->setAngularDamping(angularDamping);
            mPhysicsObjects.push_back(mLeftUpperArmCapsule[ragdollIndex]);

            // --------------- Create the left lower arm capsule --------------- //
            mLeftLowerArmPos[ragdollIndex] = mLeftUpperArmPos[ragdollIndex] + rp3d::Vector3(2.5, 0, 0);
            mLeftLowerArmCapsule[ragdollIndex] = new Capsule(true, 0.5, 2,  mPhysicsCommon, mPhysicsWorld, mMeshFolderPath);
            mLeftLowerArmCapsule[ragdollIndex]->setTransform(rp3d::Transform(mLeftLowerArmPos[ragdollIndex], rp3d::Quaternion::fromEulerAngles(0, 0, rp3d::PI_RP3D / 2.0)));
            mLeftLowerArmCapsule[ragdollIndex]->setColor(mObjectColorDemo);
            mLeftLowerArmCapsule[ragdollIndex]->setSleepingColor(mSleepingColorDemo);
            mLeftLowerArmCapsule[ragdollIndex]->getCollider()->getMaterial().setMassDensity(8);
            mLeftLowerArmCapsule[ragdollIndex]->getCollider()->getMaterial().setFrictionCoefficient(frictionCoeff);
            mLeftLowerArmCapsule[ragdollIndex]->getRigidBody()->updateMassPropertiesFromColliders();
            mLeftLowerArmCapsule[ragdollIndex]->getRigidBody()->setLinearDamping(linearDamping);
            mLeftLowerArmCapsule[ragdollIndex]->getRigidBody()->setAngularDamping(angularDamping);
            mPhysicsObjects.push_back(mLeftLowerArmCapsule[ragdollIndex]);

            // --------------- Create the left upper leg capsule --------------- //
            mLeftUpperLegPos[ragdollIndex] = mHipPos[ragdollIndex] + rp3d::Vector3(0.8, -1.5, 0);
            mLeftUpperLegCapsule[ragdollIndex] = new Capsule(true, 0.75, 2,  mPhysicsCommon, mPhysicsWorld, mMeshFolderPath);
            mLeftUpperLegCapsule[ragdollIndex]->setTransform(rp3d::Transform(mLeftUpperLegPos[ragdollIndex], rp3d::Quaternion::identity()));
            mLeftUpperLegCapsule[ragdollIndex]->setColor(mObjectColorDemo);
            mLeftUpperLegCapsule[ragdollIndex]->setSleepingColor(mSleepingColorDemo);
            mLeftUpperLegCapsule[ragdollIndex]->getCollider()->getMaterial().setMassDensity(8);
            mLeftUpperLegCapsule[ragdollIndex]->getCollider()->getMaterial().setFrictionCoefficient(frictionCoeff);
            mLeftUpperLegCapsule[ragdollIndex]->getRigidBody()->updateMassPropertiesFromColliders();
            mLeftUpperLegCapsule[ragdollIndex]->getRigidBody()->setLinearDamping(linearDamping);
            mLeftUpperLegCapsule[ragdollIndex]->getRigidBody()->setAngularDamping(angularDamping);
            mPhysicsObjects.push_back(mLeftUpperLegCapsule[ragdollIndex]);

            // --------------- Create the left lower leg capsule --------------- //
            mLeftLowerLegPos[ragdollIndex] = mLeftUpperLegPos[ragdollIndex] + rp3d::Vector3(0, -3, 0);
            mLeftLowerLegCapsule[ragdollIndex] = new Capsule(true, 0.5, 3,  mPhysicsCommon, mPhysicsWorld, mMeshFolderPath);
            mLeftLowerLegCapsule[ragdollIndex]->setTransform(rp3d::Transform(mLeftLowerLegPos[ragdollIndex], rp3d::Quaternion::identity()));
            mLeftLowerLegCapsule[ragdollIndex]->setColor(mObjectColorDemo);
            mLeftLowerLegCapsule[ragdollIndex]->setSleepingColor(mSleepingColorDemo);
            mLeftLowerLegCapsule[ragdollIndex]->getCollider()->getMaterial().setMassDensity(8);
            mLeftLowerLegCapsule[ragdollIndex]->getCollider()->getMaterial().setFrictionCoefficient(0.3);
            mLeftLowerLegCapsule[ragdollIndex]->getRigidBody()->updateMassPropertiesFromColliders();
            mLeftLowerLegCapsule[ragdollIndex]->getRigidBody()->setLinearDamping(linearDamping);
            mLeftLowerLegCapsule[ragdollIndex]->getRigidBody()->setAngularDamping(angularDamping);
            mPhysicsObjects.push_back(mLeftLowerLegCapsule[ragdollIndex]);

            // --------------- Create the right upper arm capsule --------------- //
            mRightUpperArmPos[ragdollIndex] = mChestPos[ragdollIndex] + rp3d::Vector3(-2.25, 0, 0);
            mRightUpperArmCapsule[ragdollIndex] = new Capsule(true, 0.5, 2,  mPhysicsCommon, mPhysicsWorld, mMeshFolderPath);
            mRightUpperArmCapsule[ragdollIndex]->setTransform(rp3d::Transform(mRightUpperArmPos[ragdollIndex], rp3d::Quaternion::fromEulerAngles(0, 0, rp3d::PI_RP3D / 2.0)));
            mRightUpperArmCapsule[ragdollIndex]->setColor(mObjectColorDemo);
            mRightUpperArmCapsule[ragdollIndex]->setSleepingColor(mSleepingColorDemo);
            mRightUpperArmCapsule[ragdollIndex]->getCollider()->getMaterial().setMassDensity(8);
            mRightUpperArmCapsule[ragdollIndex]->getCollider()->getMaterial().setFrictionCoefficient(frictionCoeff);
            mRightUpperArmCapsule[ragdollIndex]->getRigidBody()->updateMassPropertiesFromColliders();
            mRightUpperArmCapsule[ragdollIndex]->getRigidBody()->setLinearDamping(linearDamping);
            mRightUpperArmCapsule[ragdollIndex]->getRigidBody()->setAngularDamping(angularDamping);
            mPhysicsObjects.push_back(mRightUpperArmCapsule[ragdollIndex]);

            // --------------- Create the right lower arm capsule --------------- //
            mRightLowerArmPos[ragdollIndex] = mRightUpperArmPos[ragdollIndex] + rp3d::Vector3(-2.5, 0, 0);
            mRightLowerArmCapsule[ragdollIndex] = new Capsule(true, 0.5, 2,  mPhysicsCommon, mPhysicsWorld, mMeshFolderPath);
            mRightLowerArmCapsule[ragdollIndex]->setTransform(rp3d::Transform(mRightLowerArmPos[ragdollIndex], rp3d::Quaternion::fromEulerAngles(0, 0, rp3d::PI_RP3D / 2.0)));
            mRightLowerArmCapsule[ragdollIndex]->setColor(mObjectColorDemo);
            mRightLowerArmCapsule[ragdollIndex]->setSleepingColor(mSleepingColorDemo);
            mRightLowerArmCapsule[ragdollIndex]->getCollider()->getMaterial().setMassDensity(8);
            mRightLowerArmCapsule[ragdollIndex]->getCollider()->getMaterial().setFrictionCoefficient(frictionCoeff);
            mRightLowerArmCapsule[ragdollIndex]->getRigidBody()->updateMassPropertiesFromColliders();
            mRightLowerArmCapsule[ragdollIndex]->getRigidBody()->setLinearDamping(linearDamping);
            mRightLowerArmCapsule[ragdollIndex]->getRigidBody()->setAngularDamping(angularDamping);
            mPhysicsObjects.push_back(mRightLowerArmCapsule[ragdollIndex]);

            // --------------- Create the right upper leg capsule --------------- //
            mRightUpperLegPos[ragdollIndex] = mHipPos[ragdollIndex] + rp3d::Vector3(-0.8, -1.5, 0);
            mRightUpperLegCapsule[ragdollIndex] = new Capsule(true, 0.75, 2,  mPhysicsCommon, mPhysicsWorld, mMeshFolderPath);
            mRightUpperLegCapsule[ragdollIndex]->setTransform(rp3d::Transform(mRightUpperLegPos[ragdollIndex], rp3d::Quaternion::identity()));
            mRightUpperLegCapsule[ragdollIndex]->setColor(mObjectColorDemo);
            mRightUpperLegCapsule[ragdollIndex]->setSleepingColor(mSleepingColorDemo);
            mRightUpperLegCapsule[ragdollIndex]->getCollider()->getMaterial().setMassDensity(8);
            mRightUpperLegCapsule[ragdollIndex]->getCollider()->getMaterial().setFrictionCoefficient(frictionCoeff);
            mRightUpperLegCapsule[ragdollIndex]->getRigidBody()->updateMassPropertiesFromColliders();
            mRightUpperLegCapsule[ragdollIndex]->getRigidBody()->setLinearDamping(linearDamping);
            mRightUpperLegCapsule[ragdollIndex]->getRigidBody()->setAngularDamping(angularDamping);
            mPhysicsObjects.push_back(mRightUpperLegCapsule[ragdollIndex]);

            // --------------- Create the right lower leg capsule --------------- //
            mRightLowerLegPos[ragdollIndex] = mRightUpperLegPos[ragdollIndex] + rp3d::Vector3(0, -3, 0);
            mRightLowerLegCapsule[ragdollIndex] = new Capsule(true, 0.5, 3, mPhysicsCommon, mPhysicsWorld, mMeshFolderPath);
            mRightLowerLegCapsule[ragdollIndex]->setTransform(rp3d::Transform(mRightLowerLegPos[ragdollIndex], rp3d::Quaternion::identity()));
            mRightLowerLegCapsule[ragdollIndex]->setColor(mObjectColorDemo);
            mRightLowerLegCapsule[ragdollIndex]->setSleepingColor(mSleepingColorDemo);
            mRightLowerLegCapsule[ragdollIndex]->getCollider()->getMaterial().setMassDensity(8);
            mRightLowerLegCapsule[ragdollIndex]->getCollider()->getMaterial().setFrictionCoefficient(0.3);
            mRightLowerLegCapsule[ragdollIndex]->getRigidBody()->updateMassPropertiesFromColliders();
            mRightLowerLegCapsule[ragdollIndex]->getRigidBody()->setLinearDamping(linearDamping);
            mRightLowerLegCapsule[ragdollIndex]->getRigidBody()->setAngularDamping(angularDamping);
            mPhysicsObjects.push_back(mRightLowerLegCapsule[ragdollIndex]);

            // --------------- Create the joint between head and chest --------------- //

            // Create the joint info object
            rp3d::RigidBody* body1 = mHeadBox[ragdollIndex]->getRigidBody();
            rp3d::RigidBody* body2 = mChestCapsule[ragdollIndex]->getRigidBody();
            rp3d::BallAndSocketJointInfo jointInfo1(body1, body2, mHeadPos[ragdollIndex] + rp3d::Vector3(0, -0.75, 0));
            jointInfo1.isCollisionEnabled = false;
            mHeadChestJoint[ragdollIndex] = dynamic_cast<rp3d::BallAndSocketJoint*>(mPhysicsWorld->createJoint(jointInfo1));
            mHeadChestJoint[ragdollIndex]->setConeLimitHalfAngle(40.0 * rp3d::PI_RP3D / 180.0);
            mHeadChestJoint[ragdollIndex]->enableConeLimit(true);

            // --------------- Create the joint between chest and left upper arm --------------- //

            // Create the joint info object
            body1 = mChestCapsule[ragdollIndex]->getRigidBody();
            body2 = mLeftUpperArmCapsule[ragdollIndex]->getRigidBody();
            rp3d::BallAndSocketJointInfo jointInfo2(body1, body2, mLeftUpperArmPos[ragdollIndex] + rp3d::Vector3(-1, 0, 0));
            jointInfo2.isCollisionEnabled = false;
            mChestLeftUpperArmJoint[ragdollIndex] = dynamic_cast<rp3d::BallAndSocketJoint*>(mPhysicsWorld->createJoint(jointInfo2));
            mChestLeftUpperArmJoint[ragdollIndex]->setConeLimitHalfAngle(180.0 * rp3d::PI_RP3D / 180.0);
            mChestLeftUpperArmJoint[ragdollIndex]->enableConeLimit(true);

            // --------------- Create the joint between left upper arm and left lower arm  --------------- //

            // Create the joint info object
            body1 = mLeftUpperArmCapsule[ragdollIndex]->getRigidBody();
            body2 = mLeftLowerArmCapsule[ragdollIndex]->getRigidBody();
            rp3d::Vector3 joint2WorldAnchor = (body1->getTransform().getPosition() + body2->getTransform().getPosition()) * 0.5f;
            rp3d::Vector3 joint2WorldAxis(0, 0, 1);
            rp3d::HingeJointInfo jointInfo3(body1, body2, joint2WorldAnchor, joint2WorldAxis);
            jointInfo3.isCollisionEnabled = false;
            mLeftUpperLeftLowerArmJoint[ragdollIndex] = dynamic_cast<rp3d::HingeJoint*>(mPhysicsWorld->createJoint(jointInfo3));
            mLeftUpperLeftLowerArmJoint[ragdollIndex]->enableLimit(true);
            mLeftUpperLeftLowerArmJoint[ragdollIndex]->setMinAngleLimit(0.0 * rp3d::PI_RP3D / 180.0);
            mLeftUpperLeftLowerArmJoint[ragdollIndex]->setMaxAngleLimit(340.0 * rp3d::PI_RP3D / 180.0);

            // --------------- Create the joint between chest and waist  --------------- //

            // Create the joint info object
            body1 = mChestCapsule[ragdollIndex]->getRigidBody();
            body2 = mWaistCapsule[ragdollIndex]->getRigidBody();
            rp3d::Vector3 jointChestWaistWorldAnchor = (body1->getTransform().getPosition() + body2->getTransform().getPosition()) * 0.5f;
            rp3d::FixedJointInfo jointChestWaistInfo(body1, body2, jointChestWaistWorldAnchor);
            jointChestWaistInfo.isCollisionEnabled = false;
            mChestWaistJoint[ragdollIndex] = dynamic_cast<rp3d::FixedJoint*>(mPhysicsWorld->createJoint(jointChestWaistInfo));

            // --------------- Create the joint between waist and hips  --------------- //

            // Create the joint info object
            body1 = mWaistCapsule[ragdollIndex]->getRigidBody();
            body2 = mHipCapsule[ragdollIndex]->getRigidBody();
            rp3d::Vector3 jointWaistHipsWorldAnchor = (body1->getTransform().getPosition() + body2->getTransform().getPosition()) * 0.5f;
            rp3d::FixedJointInfo jointWaistHipsInfo(body1, body2, jointWaistHipsWorldAnchor);
            jointWaistHipsInfo.isCollisionEnabled = false;
            mWaistHipsJoint[ragdollIndex] = dynamic_cast<rp3d::FixedJoint*>(mPhysicsWorld->createJoint(jointWaistHipsInfo));

            // --------------- Create the joint between hip and left upper leg --------------- //

            // Create the joint info object
            body1 = mHipCapsule[ragdollIndex]->getRigidBody();
            body2 = mLeftUpperLegCapsule[ragdollIndex]->getRigidBody();
            rp3d::BallAndSocketJointInfo jointInfo4(body1, body2, mHipPos[ragdollIndex] + rp3d::Vector3(0.8, 0, 0));
            jointInfo4.isCollisionEnabled = false;
            mHipLeftUpperLegJoint[ragdollIndex] = dynamic_cast<rp3d::BallAndSocketJoint*>(mPhysicsWorld->createJoint(jointInfo4));
            mHipLeftUpperLegJoint[ragdollIndex]->setConeLimitHalfAngle(80.0 * rp3d::PI_RP3D / 180.0);
            mHipLeftUpperLegJoint[ragdollIndex]->enableConeLimit(true);

            // --------------- Create the joint between left upper leg and left lower leg  --------------- //

            // Create the joint info object
            body1 = mLeftUpperLegCapsule[ragdollIndex]->getRigidBody();
            body2 = mLeftLowerLegCapsule[ragdollIndex]->getRigidBody();
            rp3d::Vector3 joint5WorldAnchor = (body1->getTransform().getPosition() + body2->getTransform().getPosition()) * 0.5f;
            rp3d::Vector3 joint5WorldAxis(1, 0, 0);
            const rp3d::decimal joint5MinAngle = 0.0 * rp3d::PI_RP3D / 180.0;
            const rp3d::decimal joint5MaxAngle = 140.0 * rp3d::PI_RP3D / 180.0;
            rp3d::HingeJointInfo jointInfo5(body1, body2, joint5WorldAnchor, joint5WorldAxis, joint5MinAngle, joint5MaxAngle);
            jointInfo5.isCollisionEnabled = false;
            mLeftUpperLeftLowerLegJoint[ragdollIndex] = dynamic_cast<rp3d::HingeJoint*>(mPhysicsWorld->createJoint(jointInfo5));

            // --------------- Create the joint between chest and right upper arm --------------- //

            // Create the joint info object
            body1 = mChestCapsule[ragdollIndex]->getRigidBody();
            body2 = mRightUpperArmCapsule[ragdollIndex]->getRigidBody();
            rp3d::BallAndSocketJointInfo jointInfo6(body1, body2, mRightUpperArmPos[ragdollIndex] + rp3d::Vector3(1, 0, 0));
            jointInfo6.isCollisionEnabled = false;
            mChestRightUpperArmJoint[ragdollIndex] = dynamic_cast<rp3d::BallAndSocketJoint*>(mPhysicsWorld->createJoint(jointInfo6));
            mChestRightUpperArmJoint[ragdollIndex]->setConeLimitHalfAngle(180.0 * rp3d::PI_RP3D / 180.0);
            mChestRightUpperArmJoint[ragdollIndex]->enableConeLimit(true);

            // --------------- Create the joint between right upper arm and right lower arm  --------------- //

            // Create the joint info object
            body1 = mRightUpperArmCapsule[ragdollIndex]->getRigidBody();
            body2 = mRightLowerArmCapsule[ragdollIndex]->getRigidBody();
            rp3d::Vector3 joint7WorldAnchor = (body1->getTransform().getPosition() + body2->getTransform().getPosition()) * 0.5f;
            rp3d::Vector3 joint7WorldAxis(0, 0, 1);
            rp3d::HingeJointInfo jointInfo7(body1, body2, joint7WorldAnchor, joint7WorldAxis);
            jointInfo7.isCollisionEnabled = false;
            mRightUpperRightLowerArmJoint[ragdollIndex] = dynamic_cast<rp3d::HingeJoint*>(mPhysicsWorld->createJoint(jointInfo7));
            mRightUpperRightLowerArmJoint[ragdollIndex]->enableLimit(true);
            mRightUpperRightLowerArmJoint[ragdollIndex]->setMinAngleLimit(0.0 * rp3d::PI_RP3D / 180.0);
            mRightUpperRightLowerArmJoint[ragdollIndex]->setMaxAngleLimit(340.0 * rp3d::PI_RP3D / 180.0);

            // --------------- Create the joint between hips and right upper leg --------------- //

            // Create the joint info object
            body1 = mHipCapsule[ragdollIndex]->getRigidBody();
            body2 = mRightUpperLegCapsule[ragdollIndex]->getRigidBody();
            rp3d::BallAndSocketJointInfo jointInfo8(body1, body2, mHipPos[ragdollIndex] + rp3d::Vector3(-0.8, 0, 0));
            jointInfo8.isCollisionEnabled = false;
            mHipRightUpperLegJoint[ragdollIndex] = dynamic_cast<rp3d::BallAndSocketJoint*>(mPhysicsWorld->createJoint(jointInfo8));
            mHipRightUpperLegJoint[ragdollIndex]->setConeLimitHalfAngle(80.0 * rp3d::PI_RP3D / 180.0);
            mHipRightUpperLegJoint[ragdollIndex]->enableConeLimit(true);

            // --------------- Create the joint between right upper leg and right lower leg  --------------- //

            // Create the joint info object
            body1 = mRightUpperLegCapsule[ragdollIndex]->getRigidBody();
            body2 = mRightLowerLegCapsule[ragdollIndex]->getRigidBody();
            rp3d::Vector3 joint9WorldAnchor = (body1->getTransform().getPosition() + body2->getTransform().getPosition()) * 0.5f;
            rp3d::Vector3 joint9WorldAxis(1, 0, 0);
            const rp3d::decimal joint9MinAngle = 0.0 * rp3d::PI_RP3D / 180.0;
            const rp3d::decimal joint9MaxAngle = 140.0 * rp3d::PI_RP3D / 180.0;
            rp3d::HingeJointInfo jointInfo9(body1, body2, joint9WorldAnchor, joint9WorldAxis, joint9MinAngle, joint9MaxAngle);
            jointInfo9.isCollisionEnabled = false;
            mRightUpperRightLowerLegJoint[ragdollIndex] = dynamic_cast<rp3d::HingeJoint*>(mPhysicsWorld->createJoint(jointInfo9));
        }
    }
}
