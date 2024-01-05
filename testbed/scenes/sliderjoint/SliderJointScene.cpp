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
#include "SliderJointScene.h"
#include <cmath>

// Namespaces
using namespace openglframework;
using namespace sliderjointscene;

// Constructor
SliderJointScene::SliderJointScene(const std::string& name, EngineSettings& settings, reactphysics3d::PhysicsCommon& physicsCommon)
      : SceneDemo(name, settings, physicsCommon, true) {

    // Compute the radius and the center of the scene
    openglframework::Vector3 center(0, 5, 0);

    // Set the center of the scene
    setScenePosition(center, SCENE_RADIUS);
    setInitZoom(1.1);
    resetCameraToViewAll();

    mWorldSettings.worldName = name;
}

// Destructor
SliderJointScene::~SliderJointScene() {

    destroyPhysicsWorld();
}

// Create the physics world
void SliderJointScene::createPhysicsWorld() {

    // Gravity vector in the physics world
    mWorldSettings.gravity = rp3d::Vector3(mEngineSettings.gravity.x, mEngineSettings.gravity.y, mEngineSettings.gravity.z);

    // Create the physics world for the physics simulation
    mPhysicsWorld = mPhysicsCommon.createPhysicsWorld(mWorldSettings);
    mPhysicsWorld->setEventListener(this);

    // Create the Slider joint
    createSliderJoint();
}

// Initialize the bodies positions
void SliderJointScene::initBodiesPositions() {

    rp3d::Transform transform1(rp3d::Vector3(0,4,0), rp3d::Quaternion::identity());
    mBox1->setTransform(transform1);

    rp3d::Transform transform2(rp3d::Vector3(0,0,0), rp3d::Quaternion::identity());
    mBox2->setTransform(transform2);
}

// Destroy the physics world
void SliderJointScene::destroyPhysicsWorld() {

    if (mPhysicsWorld != nullptr) {

        delete mBox1;
        delete mBox2;

        mPhysicsObjects.clear();

        mPhysicsCommon.destroyPhysicsWorld(mPhysicsWorld);
        mPhysicsWorld = nullptr;
    }
}

// Reset the scene
void SliderJointScene::reset() {

    SceneDemo::reset();

    destroyPhysicsWorld();
    createPhysicsWorld();
    initBodiesPositions();
}

/// Create the boxes and joint for the Slider joint example
void SliderJointScene::createSliderJoint() {

    // --------------- Create the first box --------------- //

    // Create a box and a corresponding rigid in the physics world
    openglframework::Vector3 box1Dimension(2, 4, 2);
    mBox1 = new Box(rp3d::BodyType::STATIC, true, box1Dimension, mPhysicsCommon, mPhysicsWorld, mMeshFolderPath);
    mBox1->setTransform(rp3d::Transform(rp3d::Vector3(0, 4, 0), rp3d::Quaternion::identity()));

    // Set the box color
    mBox1->setColor(mObjectColorDemo);
    mBox1->setSleepingColor(mSleepingColorDemo);

    // Change the material properties of the rigid body
    rp3d::Material& material1 = mBox1->getCollider()->getMaterial();
    material1.setBounciness(0.4f);
    mPhysicsObjects.push_back(mBox1);

    // --------------- Create the second box --------------- //

    // Create a box and a corresponding rigid in the physics world
    openglframework::Vector3 box2Dimension(2, 4, 2);
    mBox2 = new Box(rp3d::BodyType::DYNAMIC, true, box2Dimension, mPhysicsCommon, mPhysicsWorld, mMeshFolderPath);
    mBox2->setTransform(rp3d::Transform(rp3d::Vector3(0, 0, 0), rp3d::Quaternion::identity()));

    // Set the box color
    mBox2->setColor(mObjectColorDemo);
    mBox2->setSleepingColor(mSleepingColorDemo);

    // Change the material properties of the rigid body
    rp3d::Material& material2 = mBox2->getCollider()->getMaterial();
    material2.setBounciness(0.4f);
    mPhysicsObjects.push_back(mBox2);

    // --------------- Create the joint --------------- //

    // Create the joint info object
    rp3d::RigidBody* body1 = mBox1->getRigidBody();
    rp3d::RigidBody* body2 = mBox2->getRigidBody();
    const rp3d::Vector3& body1Position = body1->getTransform().getPosition();
    const rp3d::Vector3& body2Position = body2->getTransform().getPosition();
    const rp3d::Vector3 anchorPointWorldSpace = rp3d::decimal(0.5) * (body2Position + body1Position);
    const rp3d::Vector3 sliderAxisWorldSpace = (body2Position - body1Position);
    rp3d::SliderJointInfo jointInfo(body1, body2, anchorPointWorldSpace, sliderAxisWorldSpace,
                                    rp3d::decimal(-1.7), rp3d::decimal(1.7));
    jointInfo.isCollisionEnabled = false;

    // Create the joint in the physics world
    mJoint = dynamic_cast<rp3d::SliderJoint*>(mPhysicsWorld->createJoint(jointInfo));
}
