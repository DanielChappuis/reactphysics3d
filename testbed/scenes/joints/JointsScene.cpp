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
#include "JointsScene.h"
#include <cmath>

// Namespaces
using namespace openglframework;
using namespace jointsscene;

// Constructor
JointsScene::JointsScene(const std::string& name, EngineSettings& settings, reactphysics3d::PhysicsCommon& physicsCommon)
      : SceneDemo(name, settings, physicsCommon, true) {

    // Compute the radius and the center of the scene
    openglframework::Vector3 center(0, 8, 0);

    // Set the center of the scene
    setScenePosition(center, SCENE_RADIUS);
    setInitZoom(1);
    resetCameraToViewAll();

    // Gravity vector in the physics world
    rp3d::Vector3 gravity(0, rp3d::decimal(-9.81), 0);

    mWorldSettings.worldName = name;
}

// Destructor
JointsScene::~JointsScene() {

    destroyPhysicsWorld();
}

// Update the physics world (take a simulation step)
void JointsScene::updatePhysics() {

    // Update the motor speed of the Slider Joint (to move up and down)
    double motorSpeed = 2.0 * std::cos(mEngineSettings.elapsedTime.count() * 1.5);
    mSliderJoint->setMotorSpeed(rp3d::decimal(motorSpeed));

    SceneDemo::updatePhysics();
}

// Create the physics world
void JointsScene::createPhysicsWorld() {

    // Gravity vector in the physics world
    mWorldSettings.gravity = rp3d::Vector3(mEngineSettings.gravity.x, mEngineSettings.gravity.y, mEngineSettings.gravity.z);

    // Create the physics world for the physics simulation
    mPhysicsWorld = mPhysicsCommon.createPhysicsWorld(mWorldSettings);
    mPhysicsWorld->setEventListener(this);

    // Create the Ball-and-Socket joint
    createBallAndSocketJoints();

    // Create the Slider joint
    createSliderJoint();

    // Create the Hinge joint
    createPropellerHingeJoint();

    // Create the Fixed joint
    createFixedJoints();

    // Create the floor
    createFloor();
}

// Initialize the bodies positions
void JointsScene::initBodiesPositions() {

    openglframework::Vector3 positionBox(0, 15, 5);
    openglframework::Vector3 boxDimension(1, 1, 1);

    for (int i=0; i<NB_BALLSOCKETJOINT_BOXES; i++) {

        // Initial position and orientation of the rigid body
        rp3d::Vector3 initPosition(positionBox.x, positionBox.y, positionBox.z);
        rp3d::Quaternion initOrientation = rp3d::Quaternion::identity();
        rp3d::Transform transform(initPosition, initOrientation);

        // Create a box and a corresponding rigid in the physics world
        mBallAndSocketJointChainBoxes[i]->setTransform(transform);

        positionBox.y -= boxDimension.y + 0.5f;
    }

    // --------------- Slider Joint --------------- //

    // Position of the box
    openglframework::Vector3 positionBox1(0, 2.1f, 0);
    rp3d::Vector3 initPosition(positionBox1.x, positionBox1.y, positionBox1.z);
    rp3d::Quaternion initOrientation = rp3d::Quaternion::identity();
    rp3d::Transform transformBottomBox(initPosition, initOrientation);

    // Create a box and a corresponding rigid in the physics world
    mSliderJointBottomBox->setTransform(transformBottomBox);

    // Position of the box
    openglframework::Vector3 positionBox2(0, 4.2f, 0);
    initPosition = rp3d::Vector3(positionBox2.x, positionBox2.y, positionBox2.z);
    initOrientation = rp3d::Quaternion::identity();
    rp3d::Transform transformTopBox(initPosition, initOrientation);

    // Create a box and a corresponding rigid in the physics world
    mSliderJointTopBox->setTransform(transformTopBox);

    // --------------- Propeller Hinge joint --------------- //

    // Position of the box
    positionBox1 = openglframework::Vector3(0, 7, 0);
    initPosition = rp3d::Vector3(positionBox1.x, positionBox1.y, positionBox1.z);
    initOrientation = rp3d::Quaternion::identity();
    rp3d::Transform transformHingeBox(initPosition, initOrientation);

    // Create a box and a corresponding rigid in the physics world
    mPropellerBox->setTransform(transformHingeBox);

    // --------------- Fixed joint --------------- //

    // Position of the box
    positionBox1 = openglframework::Vector3(5, 7, 0);
    initPosition = rp3d::Vector3(positionBox1.x, positionBox1.y, positionBox1.z);
    initOrientation = rp3d::Quaternion::identity();
    rp3d::Transform transformFixedBox1(initPosition, initOrientation);

    // Create a box and a corresponding rigid in the physics world
    mFixedJointBox1->setTransform(transformFixedBox1);

    // Position of the box
    positionBox2 = openglframework::Vector3(-5, 7, 0);
    initPosition = rp3d::Vector3(positionBox2.x, positionBox2.y, positionBox2.z);
    initOrientation = rp3d::Quaternion::identity();
    rp3d::Transform transformFixedBox2(initPosition, initOrientation);

    // Create a box and a corresponding rigid in the physics world
    mFixedJointBox2->setTransform(transformFixedBox2);
}

// Destroy the physics world
void JointsScene::destroyPhysicsWorld() {

    if (mPhysicsWorld != nullptr) {

        delete mSliderJointBottomBox;
        delete mSliderJointTopBox;
        delete mPropellerBox;
        delete mFixedJointBox1;
        delete mFixedJointBox2;
        for (int i=0; i<NB_BALLSOCKETJOINT_BOXES; i++) {
            delete mBallAndSocketJointChainBoxes[i];
        }

        // Destroy the floor
        delete mFloor;

        mPhysicsObjects.clear();

        mPhysicsCommon.destroyPhysicsWorld(mPhysicsWorld);
        mPhysicsWorld = nullptr;
    }
}
// Reset the scene
void JointsScene::reset() {

    SceneDemo::reset();

    destroyPhysicsWorld();
    createPhysicsWorld();
    initBodiesPositions();
}

// Create the boxes and joints for the Ball-and-Socket joint example
void JointsScene::createBallAndSocketJoints() {

    // --------------- Create the boxes --------------- //

    rp3d::Vector3 positionBox(0, 15, 5);
    openglframework::Vector3 boxDimension(1, 1, 1);

    for (int i=0; i<NB_BALLSOCKETJOINT_BOXES; i++) {

        // Create a box and a corresponding rigid in the physics world
        rp3d::BodyType type = i == 0 ? rp3d::BodyType::STATIC : rp3d::BodyType::DYNAMIC;
        mBallAndSocketJointChainBoxes[i] = new Box(type, true, boxDimension,  mPhysicsCommon, mPhysicsWorld, mMeshFolderPath);
        mBallAndSocketJointChainBoxes[i]->setTransform(rp3d::Transform(positionBox, rp3d::Quaternion::identity()));

        // Set the box color
        mBallAndSocketJointChainBoxes[i]->setColor(mObjectColorDemo);
        mBallAndSocketJointChainBoxes[i]->setSleepingColor(mSleepingColorDemo);

        // Add some angular velocity damping
        mBallAndSocketJointChainBoxes[i]->getRigidBody()->setAngularDamping(rp3d::decimal(0.2));

        // Change the material properties of the rigid body
        rp3d::Material& material = mBallAndSocketJointChainBoxes[i]->getCollider()->getMaterial();
        material.setBounciness(rp3d::decimal(0.4));

		mPhysicsObjects.push_back(mBallAndSocketJointChainBoxes[i]);

        positionBox.y -= boxDimension.y + 0.5f;
    }

    // --------------- Create the joints --------------- //

    for (int i=0; i<NB_BALLSOCKETJOINT_BOXES-1; i++) {

        // Create the joint info object
        rp3d::RigidBody* body1 = mBallAndSocketJointChainBoxes[i]->getRigidBody();
        rp3d::RigidBody* body2 = mBallAndSocketJointChainBoxes[i+1]->getRigidBody();
        rp3d::Vector3 body1Position = body1->getTransform().getPosition();
        rp3d::Vector3 body2Position = body2->getTransform().getPosition();
        const rp3d::Vector3 anchorPointWorldSpace = 0.5 * (body1Position + body2Position);
        rp3d::BallAndSocketJointInfo jointInfo(body1, body2, anchorPointWorldSpace);

        // Create the joint in the physics world
        mBallAndSocketJoints[i] = dynamic_cast<rp3d::BallAndSocketJoint*>(
                    mPhysicsWorld->createJoint(jointInfo));
    }
}

/// Create the boxes and joint for the Slider joint example
void JointsScene::createSliderJoint() {

    // --------------- Create the first box --------------- //

    // Position of the box
    rp3d::Vector3 positionBox1(0, 2.1f, 0);

    // Create a box and a corresponding rigid in the physics world
    openglframework::Vector3 box1Dimension(2, 4, 2);
    mSliderJointBottomBox = new Box(rp3d::BodyType::STATIC, true, box1Dimension, mPhysicsCommon, mPhysicsWorld, mMeshFolderPath);
    mSliderJointBottomBox->setTransform(rp3d::Transform(positionBox1, rp3d::Quaternion::identity()));

    // Set the box color
    mSliderJointBottomBox->setColor(mObjectColorDemo);
    mSliderJointBottomBox->setSleepingColor(mSleepingColorDemo);

    // Change the material properties of the rigid body
    rp3d::Material& material1 = mSliderJointBottomBox->getCollider()->getMaterial();
    material1.setBounciness(0.4f);
	mPhysicsObjects.push_back(mSliderJointBottomBox);

    // --------------- Create the second box --------------- //

    // Position of the box
    rp3d::Vector3 positionBox2(0, 4.2f, 0);

    // Create a box and a corresponding rigid in the physics world
    openglframework::Vector3 box2Dimension(1.5f, 4, 1.5f);
    mSliderJointTopBox = new Box(rp3d::BodyType::DYNAMIC, true, box2Dimension, mPhysicsCommon, mPhysicsWorld, mMeshFolderPath);
    mSliderJointTopBox->setTransform(rp3d::Transform(positionBox2, rp3d::Quaternion::identity()));

    // Set the box color
    mSliderJointTopBox->setColor(mObjectColorDemo);
    mSliderJointTopBox->setSleepingColor(mSleepingColorDemo);

    // Change the material properties of the rigid body
    rp3d::Material& material2 = mSliderJointTopBox->getCollider()->getMaterial();
    material2.setBounciness(0.4f);
	mPhysicsObjects.push_back(mSliderJointTopBox);

    // --------------- Create the joint --------------- //

    // Create the joint info object
    rp3d::RigidBody* body1 = mSliderJointBottomBox->getRigidBody();
    rp3d::RigidBody* body2 = mSliderJointTopBox->getRigidBody();
    const rp3d::Vector3& body1Position = body1->getTransform().getPosition();
    const rp3d::Vector3& body2Position = body2->getTransform().getPosition();
    const rp3d::Vector3 anchorPointWorldSpace = rp3d::decimal(0.5) * (body2Position + body1Position);
    const rp3d::Vector3 sliderAxisWorldSpace = (body2Position - body1Position);
    rp3d::SliderJointInfo jointInfo(body1, body2, anchorPointWorldSpace, sliderAxisWorldSpace,
                                    rp3d::decimal(-1.7), rp3d::decimal(1.7));
    jointInfo.isMotorEnabled = true;
    jointInfo.motorSpeed = 0.0;
    jointInfo.maxMotorForce = 10000.0;
    jointInfo.isCollisionEnabled = false;

    // Create the joint in the physics world
    mSliderJoint = dynamic_cast<rp3d::SliderJoint*>(mPhysicsWorld->createJoint(jointInfo));
}

/// Create the boxes and joint for the Hinge joint example
void JointsScene::createPropellerHingeJoint() {

    // --------------- Create the propeller box --------------- //

    // Position of the box
    rp3d::Vector3 positionBox1(0, 7, 0);

    // Create a box and a corresponding rigid in the physics world
    openglframework::Vector3 boxDimension(10, 1, 1);
    mPropellerBox = new Box(rp3d::BodyType::DYNAMIC, true, boxDimension, mPhysicsCommon, mPhysicsWorld, mMeshFolderPath);
    mPropellerBox->setTransform(rp3d::Transform(positionBox1, rp3d::Quaternion::identity()));

    // Set the box color
    mPropellerBox->setColor(mObjectColorDemo);
    mPropellerBox->setSleepingColor(mSleepingColorDemo);

    // Change the material properties of the rigid body
    rp3d::Material& material = mPropellerBox->getCollider()->getMaterial();
    material.setBounciness(rp3d::decimal(0.4));
	mPhysicsObjects.push_back(mPropellerBox);

    // --------------- Create the Hinge joint --------------- //

    // Create the joint info object
    rp3d::RigidBody* body1 = mPropellerBox->getRigidBody();
    rp3d::RigidBody* body2 = mSliderJointTopBox->getRigidBody();
    const rp3d::Vector3& body1Position = body1->getTransform().getPosition();
    const rp3d::Vector3& body2Position = body2->getTransform().getPosition();
    const rp3d::Vector3 anchorPointWorldSpace = 0.5 * (body2Position + body1Position);
    const rp3d::Vector3 hingeAxisWorldSpace(0, 1, 0);
    rp3d::HingeJointInfo jointInfo(body1, body2, anchorPointWorldSpace, hingeAxisWorldSpace);
    jointInfo.isMotorEnabled = true;
    jointInfo.motorSpeed = - rp3d::decimal(0.5) * PI;
    jointInfo.maxMotorTorque = rp3d::decimal(60.0);
    jointInfo.isCollisionEnabled = false;

    // Create the joint in the physics world
    mPropellerHingeJoint = dynamic_cast<rp3d::HingeJoint*>(mPhysicsWorld->createJoint(jointInfo));
}

/// Create the boxes and joints for the fixed joints
void JointsScene::createFixedJoints() {

    // --------------- Create the first box --------------- //

    // Position of the box
    rp3d::Vector3 positionBox1(5, 7, 0);

    // Create a box and a corresponding rigid in the physics world
    openglframework::Vector3 boxDimension(1.5, 1.5, 1.5);
    mFixedJointBox1 = new Box(rp3d::BodyType::DYNAMIC, true, boxDimension, mPhysicsCommon, mPhysicsWorld, mMeshFolderPath);
    mFixedJointBox1->setTransform(rp3d::Transform(positionBox1, rp3d::Quaternion::identity()));

    // Set the box color
    mFixedJointBox1->setColor(mObjectColorDemo);
    mFixedJointBox1->setSleepingColor(mSleepingColorDemo);

    // Change the material properties of the rigid body
    rp3d::Material& material1 = mFixedJointBox1->getCollider()->getMaterial();
    material1.setBounciness(rp3d::decimal(0.4));
	mPhysicsObjects.push_back(mFixedJointBox1);

    // --------------- Create the second box --------------- //

    // Position of the box
    rp3d::Vector3 positionBox2(-5, 7, 0);

    // Create a box and a corresponding rigid in the physics world
    mFixedJointBox2 = new Box(rp3d::BodyType::DYNAMIC, true, boxDimension, mPhysicsCommon, mPhysicsWorld, mMeshFolderPath);
    mFixedJointBox2->setTransform(rp3d::Transform(positionBox2, rp3d::Quaternion::identity()));

    // Set the box color
    mFixedJointBox2->setColor(mObjectColorDemo);
    mFixedJointBox2->setSleepingColor(mSleepingColorDemo);

    // Change the material properties of the rigid body
    rp3d::Material& material2 = mFixedJointBox2->getCollider()->getMaterial();
    material2.setBounciness(rp3d::decimal(0.4));
	mPhysicsObjects.push_back(mFixedJointBox2);

    // --------------- Create the first fixed joint --------------- //

    // Create the joint info object
    rp3d::RigidBody* body1 = mFixedJointBox1->getRigidBody();
    rp3d::RigidBody* propellerBody = mPropellerBox->getRigidBody();
    const rp3d::Vector3 anchorPointWorldSpace1(5, 7, 0);
    rp3d::FixedJointInfo jointInfo1(body1, propellerBody, anchorPointWorldSpace1);
    jointInfo1.isCollisionEnabled = false;

    // Create the joint in the physics world
    mFixedJoint1 = dynamic_cast<rp3d::FixedJoint*>(mPhysicsWorld->createJoint(jointInfo1));

    // --------------- Create the second fixed joint --------------- //

    // Create the joint info object
    rp3d::RigidBody* body2 = mFixedJointBox2->getRigidBody();
    const rp3d::Vector3 anchorPointWorldSpace2(-5, 7, 0);
    rp3d::FixedJointInfo jointInfo2(body2, propellerBody, anchorPointWorldSpace2);
    jointInfo2.isCollisionEnabled = false;

    // Create the joint in the physics world
    mFixedJoint2 = dynamic_cast<rp3d::FixedJoint*>(mPhysicsWorld->createJoint(jointInfo2));
}

// Create the floor
void JointsScene::createFloor() {

    // Create the floor
    rp3d::Vector3 floorPosition(0, 0, 0);
    mFloor = new Box(rp3d::BodyType::STATIC, true, FLOOR_SIZE, mPhysicsCommon, mPhysicsWorld, mMeshFolderPath);

    // Set the box color
    mFloor->setColor(mFloorColorDemo);
    mFloor->setSleepingColor(mFloorColorDemo);

    // Change the material properties of the rigid body
    rp3d::Material& material = mFloor->getCollider()->getMaterial();
    material.setBounciness(rp3d::decimal(0.3));
	mPhysicsObjects.push_back(mFloor);
}
