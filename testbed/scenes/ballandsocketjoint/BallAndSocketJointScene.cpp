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
BallAndSocketJointScene::BallAndSocketJointScene(const std::string& name, EngineSettings& settings, reactphysics3d::PhysicsCommon& physicsCommon)
      : SceneDemo(name, settings, physicsCommon, true, SCENE_RADIUS) {

    // Compute the radius and the center of the scene
    openglframework::Vector3 center(0, 5, 0);

    // Set the center of the scene
    setScenePosition(center, SCENE_RADIUS);
    setInitZoom(1);
    resetCameraToViewAll();

    mWorldSettings.worldName = name;
}

// Destructor
BallAndSocketJointScene::~BallAndSocketJointScene() {

    destroyPhysicsWorld();
}

// Create the physics world
void BallAndSocketJointScene::createPhysicsWorld() {

    // Gravity vector in the physics world
    mWorldSettings.gravity = rp3d::Vector3(mEngineSettings.gravity.x, mEngineSettings.gravity.y, mEngineSettings.gravity.z);

    // Create the physics world for the physics simulation
    mPhysicsWorld = mPhysicsCommon.createPhysicsWorld(mWorldSettings);
    mPhysicsWorld->setEventListener(this);

    // Create the Ball-and-Socket joint
    createBallAndSocketJoint();
}

// Initialize the bodies positions
void BallAndSocketJointScene::initBodiesPositions() {

    mBox1->setTransform(rp3d::Transform(rp3d::Vector3(0, 8, 0), rp3d::Quaternion::identity()));
    mBox2->setTransform(rp3d::Transform(rp3d::Vector3(0, 0, 0), rp3d::Quaternion::identity()));
}

// Destroy the physics world
void BallAndSocketJointScene::destroyPhysicsWorld() {

    if (mPhysicsWorld != nullptr) {

        delete mBox1;
        delete mBox2;

        mPhysicsObjects.clear();

        mPhysicsCommon.destroyPhysicsWorld(mPhysicsWorld);
        mPhysicsWorld = nullptr;
    }
}

// Reset the scene
void BallAndSocketJointScene::reset() {

    SceneDemo::reset();

    destroyPhysicsWorld();
    createPhysicsWorld();
    initBodiesPositions();
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
    mJoint->setConeLimitHalfAngle(90.0 * rp3d::PI_RP3D / 180.0);
    mJoint->enableConeLimit(true);

}
