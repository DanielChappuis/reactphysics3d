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
#include "BallAndSocketJointScene.h"
#include <cmath>

// Namespaces
using namespace openglframework;
using namespace ballandsocketjointscene;

// Constructor
BallAndSocketJointScene::BallAndSocketJointScene(const std::string& name, EngineSettings& settings)
      : SceneDemo(name, settings, true, SCENE_RADIUS) {

    // Compute the radius and the center of the scene
    openglframework::Vector3 center(0, 5, 0);

    // Set the center of the scene
    setScenePosition(center, SCENE_RADIUS);

    // Gravity vector in the physics world
    rp3d::Vector3 gravity(0, rp3d::decimal(-9.81), 0);

    rp3d::PhysicsWorld::WorldSettings worldSettings;
    worldSettings.worldName = name;

    // Logger
    rp3d::DefaultLogger* defaultLogger = mPhysicsCommon.createDefaultLogger();
    uint logLevel = static_cast<uint>(rp3d::Logger::Level::Information) | static_cast<uint>(rp3d::Logger::Level::Warning) |
            static_cast<uint>(rp3d::Logger::Level::Error);
    defaultLogger->addFileDestination("rp3d_log_" + name + ".html", logLevel, rp3d::DefaultLogger::Format::HTML);
    mPhysicsCommon.setLogger(defaultLogger);

    // Create the physics world for the physics simulation
    rp3d::PhysicsWorld* physicsWorld = mPhysicsCommon.createPhysicsWorld(worldSettings);
    physicsWorld->setEventListener(this);
    mPhysicsWorld = physicsWorld;

    // Create the Ball-and-Socket joint
    createBallAndSocketJoint();

    // Get the physics engine parameters
    mEngineSettings.isGravityEnabled = mPhysicsWorld->isGravityEnabled();
    rp3d::Vector3 gravityVector = mPhysicsWorld->getGravity();
    mEngineSettings.gravity = openglframework::Vector3(gravityVector.x, gravityVector.y, gravityVector.z);
    mEngineSettings.isSleepingEnabled = mPhysicsWorld->isSleepingEnabled();
    mEngineSettings.sleepLinearVelocity = mPhysicsWorld->getSleepLinearVelocity();
    mEngineSettings.sleepAngularVelocity = mPhysicsWorld->getSleepAngularVelocity();
    mEngineSettings.nbPositionSolverIterations = mPhysicsWorld->getNbIterationsPositionSolver();
    mEngineSettings.nbVelocitySolverIterations = mPhysicsWorld->getNbIterationsVelocitySolver();
    mEngineSettings.timeBeforeSleep = mPhysicsWorld->getTimeBeforeSleep();
}

// Destructor
BallAndSocketJointScene::~BallAndSocketJointScene() {

    // Destroy the joints
    mPhysicsWorld->destroyJoint(mJoint);

    // Destroy all the rigid bodies of the scene
    mPhysicsWorld->destroyRigidBody(mBox1->getRigidBody());
    mPhysicsWorld->destroyRigidBody(mBox2->getRigidBody());

    delete mBox1;
    delete mBox2;

    // Destroy the physics world
    mPhysicsCommon.destroyPhysicsWorld(mPhysicsWorld);
}

// Reset the scene
void BallAndSocketJointScene::reset() {

    SceneDemo::reset();

    mBox1->setTransform(rp3d::Transform(rp3d::Vector3(0, 8, 0), rp3d::Quaternion::identity()));
    mBox2->setTransform(rp3d::Transform(rp3d::Vector3(0, 0, 0), rp3d::Quaternion::identity()));
}

// Create the boxes and joints for the Ball-and-Socket joint example
void BallAndSocketJointScene::createBallAndSocketJoint() {

    // --------------- Create the box 1 --------------- //
    mBox1 = new Box(true, Vector3(4, 4, 4) ,  mPhysicsCommon, mPhysicsWorld, mMeshFolderPath);
    mBox1->setTransform(rp3d::Transform(rp3d::Vector3(0, 8, 0), rp3d::Quaternion::identity()));

    // Set the box color
    mBox1->setColor(mObjectColorDemo);
    mBox1->setSleepingColor(mSleepingColorDemo);

    mBox1->getRigidBody()->setType(rp3d::BodyType::STATIC);

    mPhysicsObjects.push_back(mBox1);

    // --------------- Create the box 2 --------------- //

    mBox2 = new Box(true, Vector3(4, 8, 4),  mPhysicsCommon, mPhysicsWorld, mMeshFolderPath);
    mBox2->setTransform(rp3d::Transform(rp3d::Vector3(0, 0, 0), rp3d::Quaternion::identity()));

    // Set the box color
    mBox2->setColor(mObjectColorDemo);
    mBox2->setSleepingColor(mSleepingColorDemo);

    mPhysicsObjects.push_back(mBox2);

    // --------------- Create the joint --------------- //

    // Create the joint info object
    rp3d::RigidBody* body1 = mBox1->getRigidBody();
    rp3d::RigidBody* body2 = mBox2->getRigidBody();
    rp3d::Vector3 body1Position = body1->getTransform().getPosition();
    rp3d::Vector3 body2Position = body2->getTransform().getPosition();
    const rp3d::Vector3 anchorPointWorldSpace = 0.5 * (body1Position + body2Position);
    rp3d::BallAndSocketJointInfo jointInfo(body1, body2, anchorPointWorldSpace);
    jointInfo.isCollisionEnabled = false;

    // Create the joint in the physics world
    mJoint = dynamic_cast<rp3d::BallAndSocketJoint*>(mPhysicsWorld->createJoint(jointInfo));
    mJoint->setConeLimitLocalAxisBody1(rp3d::Vector3(0, 1, 0));
    mJoint->setConeLimitLocalAxisBody2(rp3d::Vector3(0, 1, 0));
    mJoint->setConeLimitHalfAngle(45.0 * rp3d::PI_RP3D / 180.0);
    mJoint->enableConeLimit(true);

}

// Update the scene
void BallAndSocketJointScene::update() {

    SceneDemo::update();
}
