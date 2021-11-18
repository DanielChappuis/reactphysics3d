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
#include "FixedJointScene.h"
#include <cmath>

// Namespaces
using namespace openglframework;
using namespace fixedjointscene;

// Constructor
FixedJointScene::FixedJointScene(const std::string& name, EngineSettings& settings, reactphysics3d::PhysicsCommon& physicsCommon)
      : SceneDemo(name, settings, physicsCommon, true, SCENE_RADIUS) {

    // Compute the radius and the center of the scene
    openglframework::Vector3 center(0, 5, 0);

    // Set the center of the scene
    setScenePosition(center, SCENE_RADIUS);
    setInitZoom(1.1);
    resetCameraToViewAll();

    mWorldSettings.worldName = name;
}

// Destructor
FixedJointScene::~FixedJointScene() {

    destroyPhysicsWorld();
}

// Create the physics world
void FixedJointScene::createPhysicsWorld() {

    // Gravity vector in the physics world
    mWorldSettings.gravity = rp3d::Vector3(mEngineSettings.gravity.x, mEngineSettings.gravity.y, mEngineSettings.gravity.z);

    // Create the physics world for the physics simulation
    mPhysicsWorld = mPhysicsCommon.createPhysicsWorld(mWorldSettings);
    mPhysicsWorld->setEventListener(this);

    // Create the fixed joint
    createFixedJoint();
}

// Initialize the bodies positions
void FixedJointScene::initBodiesPositions() {

    mBox1->setTransform(rp3d::Transform(rp3d::Vector3(0, 4, 0), rp3d::Quaternion::identity()));
    mBox2->setTransform(rp3d::Transform(rp3d::Vector3(4, 8, 4), rp3d::Quaternion::identity()));
}

// Destroy the physics world
void FixedJointScene::destroyPhysicsWorld() {

    if (mPhysicsWorld != nullptr) {

        delete mBox1;
        delete mBox2;

        mPhysicsObjects.clear();

        mPhysicsCommon.destroyPhysicsWorld(mPhysicsWorld);
        mPhysicsWorld = nullptr;
    }
}
// Reset the scene
void FixedJointScene::reset() {

    SceneDemo::reset();

    destroyPhysicsWorld();
    createPhysicsWorld();
    initBodiesPositions();
}

/// Create the fixed joint
void FixedJointScene::createFixedJoint() {

    // --------------- Create the first box --------------- //

    // Position of the box
    rp3d::Vector3 positionBox1(0, 4, 0);

    // Create a box and a corresponding rigid in the physics world
    openglframework::Vector3 boxDimension(4, 4, 4);
    mBox1 = new Box(true, boxDimension, mPhysicsCommon, mPhysicsWorld, mMeshFolderPath);
    mBox1->setTransform(rp3d::Transform(positionBox1, rp3d::Quaternion::identity()));
    mBox1->getRigidBody()->setType(rp3d::BodyType::STATIC);

    // Set the box color
    mBox1->setColor(mObjectColorDemo);
    mBox1->setSleepingColor(mSleepingColorDemo);

    mPhysicsObjects.push_back(mBox1);

    // --------------- Create the second box --------------- //

    // Position of the box
    rp3d::Vector3 positionBox2(4, 8, 4);

    // Create a box and a corresponding rigid in the physics world
    mBox2 = new Box(true, boxDimension, mPhysicsCommon, mPhysicsWorld, mMeshFolderPath);
    mBox2->setTransform(rp3d::Transform(positionBox2, rp3d::Quaternion::identity()));

    // Set the box color
    mBox2->setColor(mObjectColorDemo);
    mBox2->setSleepingColor(mSleepingColorDemo);

    mPhysicsObjects.push_back(mBox2);

    // --------------- Create the fixed joint --------------- //

    // Create the joint info object
    rp3d::RigidBody* body1 = mBox1->getRigidBody();
    rp3d::RigidBody* body2 = mBox2->getRigidBody();
    const rp3d::Vector3 anchorPointWorldSpace1(5, 7, 0);
    rp3d::FixedJointInfo jointInfo1(body1, body2, anchorPointWorldSpace1);
    jointInfo1.isCollisionEnabled = false;

    // Create the joint in the physics world
    mFixedJoint = dynamic_cast<rp3d::FixedJoint*>(mPhysicsWorld->createJoint(jointInfo1));
}
