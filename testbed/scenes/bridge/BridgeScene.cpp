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
#include "BridgeScene.h"
#include <cmath>

// Namespaces
using namespace openglframework;
using namespace bridgescene;

// Constructor
BridgeScene::BridgeScene(const std::string& name, EngineSettings& settings, reactphysics3d::PhysicsCommon& physicsCommon)
      : SceneDemo(name, settings, physicsCommon, true) {

    std::string meshFolderPath("meshes/");

    // Compute the radius and the center of the scene
    openglframework::Vector3 center(0, 5, 0);

    // Set the center of the scene
    setScenePosition(center, SCENE_RADIUS);
    setInitZoom(1.1);
    resetCameraToViewAll();

    mWorldSettings.worldName = name;
}

// Destructor
BridgeScene::~BridgeScene() {

    destroyPhysicsWorld();
}

// Create the joints
void BridgeScene::createJoints() {

    for (int b=0; b < NB_BRIDGES; b++) {

        for (int i=0; i < NB_BOXES-1; i++) {

            const uint box1Index = b * NB_BOXES + i;
            const uint box2Index = b * NB_BOXES + i + 1;

            // Create the joint info object
            rp3d::RigidBody* body1 = mBoxes[box1Index]->getRigidBody();
            rp3d::RigidBody* body2 = mBoxes[box2Index]->getRigidBody();
            rp3d::Vector3 body1Position = body1->getTransform().getPosition();
            const rp3d::Vector3 anchorPointWorldSpace = body1Position + rp3d::Vector3(BOX_SIZE.x / 2.0f, 0, 0);
            rp3d::HingeJointInfo jointInfo(body1, body2, anchorPointWorldSpace, rp3d::Vector3(0, 0, 1));
            jointInfo.isCollisionEnabled = false;
            rp3d::HingeJoint* joint = dynamic_cast<rp3d::HingeJoint*>(mPhysicsWorld->createJoint(jointInfo));
            mHingeJoints.push_back(joint);
        }
    }
}

// Update the physics simulation
void BridgeScene::updatePhysics() {

    SceneDemo::updatePhysics();

    // Test if the joints of the last bridge should break or not
    for (int i=0; i <NB_BOXES-1; i++) {

        const uint jointIndex = mHingeJoints.size() - 1 - i;
        rp3d::HingeJoint* joint = mHingeJoints[jointIndex];

        if (joint != nullptr) {

            if (joint->getReactionForce(mEngineSettings.timeStep.count()).lengthSquare() > 60000000) {
                mPhysicsWorld->destroyJoint(joint);
                mHingeJoints[jointIndex] = nullptr;
            }
        }
    }
}

// Create the physics world
void BridgeScene::createPhysicsWorld() {

    // Gravity vector in the physics world
    mWorldSettings.gravity = rp3d::Vector3(mEngineSettings.gravity.x, mEngineSettings.gravity.y, mEngineSettings.gravity.z);

    // Create the physics world for the physics simulation
    mPhysicsWorld = mPhysicsCommon.createPhysicsWorld(mWorldSettings);
    mPhysicsWorld->setEventListener(this);

    for (int b=0; b < NB_BRIDGES; b++) {

        // Create all the boxes of the scene
        for (int i=0; i<NB_BOXES; i++) {

            const uint boxIndex = b * NB_BOXES + i;

            // Create a box and a corresponding rigid in the physics world
            rp3d::BodyType type = i == 0 || i == NB_BOXES-1 ? rp3d::BodyType::STATIC : rp3d::BodyType::DYNAMIC;
            mBoxes[boxIndex] = new Box(type, true, BOX_SIZE, mPhysicsCommon, mPhysicsWorld, mMeshFolderPath);

            // Set the box color
            mBoxes[boxIndex]->setColor(mFloorColorDemo);
            mBoxes[boxIndex]->setSleepingColor(mSleepingColorDemo);

            // Change the material properties of the rigid body
            rp3d::Material& material = mBoxes[boxIndex]->getCollider()->getMaterial();
            material.setBounciness(rp3d::decimal(0.0));

            // Add the box the list of boxes in the scene
            mPhysicsObjects.push_back(mBoxes[boxIndex]);
        }
    }

    // Create all the spheres of the scene
    for (int i=0; i<NB_BRIDGES; i++) {

        // Create a sphere and a corresponding rigid in the physics world
        mSpheres[i] = new Sphere(rp3d::BodyType::DYNAMIC, true, SPHERE_RADIUS, mPhysicsCommon, mPhysicsWorld, mMeshFolderPath);

        // Set the box color
        mSpheres[i]->setColor(mObjectColorDemo);
        mSpheres[i]->setSleepingColor(mSleepingColorDemo);

        // Change the material properties of the rigid body
        rp3d::Material& material = mSpheres[i]->getCollider()->getMaterial();
        material.setBounciness(rp3d::decimal(0.2));
        mSpheres[i]->getRigidBody()->setMass(SPHERE_MASS);

        // Add the sphere the list of sphere in the scene
        mPhysicsObjects.push_back(mSpheres[i]);
    }

    // Set the position of the boxes before the joints creation
    initBodiesPositions();

    // Create the Ball-and-Socket joints
    createJoints();
}

// Initialize the bodies positions
void BridgeScene::initBodiesPositions() {

    const rp3d::Quaternion initOrientation = rp3d::Quaternion::identity();

    // Bridges
    for (int b=0; b < NB_BRIDGES; b++) {

        for (int i=0; i < NB_BOXES; i++) {

            // Initial position and orientation of the rigid body
            rp3d::Vector3 initPosition((-NB_BOXES/2 + i) * (BOX_SIZE.x + 0.2),
                                        -7,
                                        (-NB_BRIDGES/2 + b) * (BOX_SIZE.z + 4));
            rp3d::Transform transform(initPosition, initOrientation);

            // Create a box and a corresponding rigid in the physics world
            mBoxes[b * NB_BOXES + i]->setTransform(transform);
        }
    }

    // Spheres
    for (int i=0; i < NB_BRIDGES; i++) {

        // Initial position and orientation of the rigid body
        rp3d::Vector3 initPosition(-12,
                                   10,
                                   (-NB_BRIDGES/2 + i) * (BOX_SIZE.z + 4));

        rp3d::Transform transform(initPosition, initOrientation);

        // Create a box and a corresponding rigid in the physics world
        mSpheres[i]->setTransform(transform);
    }

    // Destroy the joints
    for (uint i=0; i < mHingeJoints.size(); i++) {

        if (mHingeJoints[i] != nullptr) {
            mPhysicsWorld->destroyJoint(mHingeJoints[i]);
        }
    }
    mHingeJoints.clear();

    createJoints();
}

// Destroy the physics world
void BridgeScene::destroyPhysicsWorld() {

    if (mPhysicsWorld != nullptr) {

        // Destroy all the physics objects of the scene
        for (std::vector<PhysicsObject*>::iterator it = mPhysicsObjects.begin(); it != mPhysicsObjects.end(); ++it) {

            // Destroy the object
            delete (*it);
        }

        mHingeJoints.clear();
        mPhysicsObjects.clear();

        mPhysicsCommon.destroyPhysicsWorld(mPhysicsWorld);
        mPhysicsWorld = nullptr;
    }
}

// Reset the scene
void BridgeScene::reset() {

    SceneDemo::reset();

    destroyPhysicsWorld();
    createPhysicsWorld();
    initBodiesPositions();
}
