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
#define _USE_MATH_DEFINES
#include <cmath>
#include "BoxTowerScene.h"

// Namespaces
using namespace openglframework;
using namespace boxtowerscene;

// Constructor
BoxTowerScene::BoxTowerScene(const std::string& name, EngineSettings& settings, reactphysics3d::PhysicsCommon& physicsCommon)
       : SceneDemo(name, settings, physicsCommon, true, SCENE_RADIUS) {

    std::string meshFolderPath("meshes/");

    // Compute the radius and the center of the scene
    openglframework::Vector3 center(0, 10, 0);

    // Set the center of the scene
    setScenePosition(center, SCENE_RADIUS);
    setInitZoom(1.5);
    resetCameraToViewAll();

    // Gravity vector in the physics world
    rp3d::Vector3 gravity(0, -9.81f, 0);

    mWorldSettings.worldName = name;
}

// Destructor
BoxTowerScene::~BoxTowerScene() {

    destroyPhysicsWorld();
}

// Create the physics world
void BoxTowerScene::createPhysicsWorld() {

    // Gravity vector in the physics world
    mWorldSettings.gravity = rp3d::Vector3(mEngineSettings.gravity.x, mEngineSettings.gravity.y, mEngineSettings.gravity.z);

    // Create the physics world for the physics simulation
    mPhysicsWorld = mPhysicsCommon.createPhysicsWorld(mWorldSettings);
    mPhysicsWorld->setEventListener(this);

    // Create all the boxes of the scene
    for (int i=0; i<NB_BOXES; i++) {

        // Create a sphere and a corresponding rigid in the physics world
        Box* box = new Box(true, BOX_SIZE, mPhysicsCommon, mPhysicsWorld, mMeshFolderPath);

        // Set the box color
        box->setColor(mObjectColorDemo);
        box->setSleepingColor(mSleepingColorDemo);

        // Change the material properties of the rigid body
        rp3d::Material& material = box->getCollider()->getMaterial();
        material.setBounciness(rp3d::decimal(0.2));

        // Add the sphere the list of boxes in the scene
        mBoxes.push_back(box);
        mPhysicsObjects.push_back(box);
    }

    // ---------- Create the floor ---------

    mFloor = new Box(true, FLOOR_SIZE, mPhysicsCommon, mPhysicsWorld, mMeshFolderPath);
    mPhysicsObjects.push_back(mFloor);

    // Set the box color
    mFloor->setColor(mFloorColorDemo);
    mFloor->setSleepingColor(mFloorColorDemo);

    // The floor must be a static rigid body
    mFloor->getRigidBody()->setType(rp3d::BodyType::STATIC);

    // Change the material properties of the rigid body
    rp3d::Material& material = mFloor->getCollider()->getMaterial();
    material.setBounciness(rp3d::decimal(0.2));
}

// Initialize the bodies positions
void BoxTowerScene::initBodiesPositions() {

    float distFromCenter = 4.0f;

    bool rotated = false;
    int floorIndex = 0;

    // Create all the boxes of the scene
    for (uint i = 0; i<NB_BOXES; i++) {

        rp3d::Vector3 position = rp3d::Vector3(i % 2 == 0 ? -distFromCenter : distFromCenter, 4.0f + (floorIndex * (BOX_SIZE.y + 0.1f)), 0.0f);
        rp3d::Quaternion orientation = rp3d::Quaternion::identity();

        if (rotated) {
           orientation = rp3d::Quaternion::fromEulerAngles(0, M_PI / 2, 0);
           //orientation = rp3d::Quaternion::fromEulerAngles(0.01f, M_PI / 2, 0);
           position = rp3d::Vector3(0, 3.0f + (floorIndex * (BOX_SIZE.y + 0.2f)), i % 2 == 0 ? -distFromCenter : distFromCenter);
        }

        mBoxes[i]->setTransform(rp3d::Transform(position, orientation));

        if (i % 2 == 1) {
            rotated = !rotated;
            floorIndex++;
        }
    }

    // ---------- Create the triangular mesh ---------- //

    mFloor->setTransform(rp3d::Transform::identity());
}

// Destroy the physics world
void BoxTowerScene::destroyPhysicsWorld() {

    if (mPhysicsWorld != nullptr) {

        // Destroy all the physics objects of the scene
        for (std::vector<PhysicsObject*>::iterator it = mPhysicsObjects.begin(); it != mPhysicsObjects.end(); ++it) {

            // Destroy the object
            delete (*it);
        }
        mBoxes.clear();
        mSpheres.clear();
        mCapsules.clear();
        mConvexMeshes.clear();
        mDumbbells.clear();

        mPhysicsObjects.clear();

        // Destroy the physics world
        mPhysicsCommon.destroyPhysicsWorld(static_cast<rp3d::PhysicsWorld*>(mPhysicsWorld));
        mPhysicsWorld = nullptr;
    }
}

/// Reset the scene
void BoxTowerScene::reset() {

    SceneDemo::reset();

    destroyPhysicsWorld();
    createPhysicsWorld();
    initBodiesPositions();
}
