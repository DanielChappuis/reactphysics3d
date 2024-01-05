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
#include "CubesScene.h"

// Namespaces
using namespace openglframework;
using namespace cubesscene;

// Constructor
CubesScene::CubesScene(const std::string& name, EngineSettings& settings, reactphysics3d::PhysicsCommon& physicsCommon)
      : SceneDemo(name, settings, physicsCommon, true) {

    // Compute the radius and the center of the scene
    openglframework::Vector3 center(0, 10, 0);

    // Set the center of the scene
    setScenePosition(center, SCENE_RADIUS);
    setInitZoom(1.5);
    resetCameraToViewAll();

    mWorldSettings.worldName = name;
}

// Destructor
CubesScene::~CubesScene() {

    destroyPhysicsWorld();
}

// Create the physics world
void CubesScene::createPhysicsWorld() {

    // Gravity vector in the physics world
    mWorldSettings.gravity = rp3d::Vector3(mEngineSettings.gravity.x, mEngineSettings.gravity.y, mEngineSettings.gravity.z);

    // Create the physics world for the physics simulation
    mPhysicsWorld = mPhysicsCommon.createPhysicsWorld(mWorldSettings);
    mPhysicsWorld->setEventListener(this);

    // Create all the cubes of the scene
    for (int i=0; i<NB_CUBES; i++) {

        // Create a cube and a corresponding rigid in the physics world
        Box* cube = new Box(rp3d::BodyType::DYNAMIC, true, BOX_SIZE,  mPhysicsCommon, mPhysicsWorld, mMeshFolderPath);

        // Set the box color
        cube->setColor(mObjectColorDemo);
        cube->setSleepingColor(mSleepingColorDemo);

        // Change the material properties of the rigid body
        rp3d::Material& material = cube->getCollider()->getMaterial();
        material.setBounciness(rp3d::decimal(0.4));

        // Add the box the list of box in the scene
        mBoxes.push_back(cube);
        mPhysicsObjects.push_back(cube);
    }

    // ------------------------- FLOOR ----------------------- //

    // Create the floor
    mFloor = new Box(rp3d::BodyType::STATIC, true, FLOOR_SIZE, mPhysicsCommon, mPhysicsWorld, mMeshFolderPath);
    mFloor->setColor(mFloorColorDemo);
    mFloor->setSleepingColor(mFloorColorDemo);
    mPhysicsObjects.push_back(mFloor);
}

// Initialize the bodies positions
void CubesScene::initBodiesPositions() {

    const float radius = 2.0f;

    // Create all the cubes of the scene
    std::vector<Box*>::iterator it;
    int i = 0;
    for (it = mBoxes.begin(); it != mBoxes.end(); ++it) {

        // Position of the cubes
       float angle = i * 30.0f;
       rp3d::Vector3 position;

       if (i == 0) {

           position = rp3d::Vector3(0, 5, 0);
       }
       else {

           position = rp3d::Vector3(radius * std::cos(angle),
                                  10 + i * (BOX_SIZE.y + 0.3f),
                                  0);
       }

       (*it)->setTransform(rp3d::Transform(position, rp3d::Quaternion::identity()));

       i++;
    }

    mFloor->setTransform(rp3d::Transform(rp3d::Vector3::zero(), rp3d::Quaternion::identity()));
}

// Destroy the physics world
void CubesScene::destroyPhysicsWorld() {

    if (mPhysicsWorld != nullptr) {

        // Destroy all the cubes of the scene
        for (std::vector<Box*>::iterator it = mBoxes.begin(); it != mBoxes.end(); ++it) {

            // Destroy the cube
            delete (*it);
        }
        mBoxes.clear();

        // Destroy the floor
        delete mFloor;

        mPhysicsObjects.clear();

        mPhysicsCommon.destroyPhysicsWorld(mPhysicsWorld);
        mPhysicsWorld = nullptr;
    }
}

// Reset the scene
void CubesScene::reset() {

    SceneDemo::reset();

    destroyPhysicsWorld();
    createPhysicsWorld();
    initBodiesPositions();
}
