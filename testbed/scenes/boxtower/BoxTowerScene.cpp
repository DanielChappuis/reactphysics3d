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
#include "BoxTowerScene.h"

// Namespaces
using namespace openglframework;
using namespace boxtowerscene;

// Constructor
BoxTowerScene::BoxTowerScene(const std::string& name, EngineSettings& settings)
       : SceneDemo(name, settings, true, SCENE_RADIUS) {

    std::string meshFolderPath("meshes/");

    // Compute the radius and the center of the scene
    openglframework::Vector3 center(0, 5, 0);

    // Set the center of the scene
    setScenePosition(center, SCENE_RADIUS);

    // Gravity vector in the physics world
    rp3d::Vector3 gravity(0, -9.81f, 0);

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
BoxTowerScene::~BoxTowerScene() {

    // Destroy all the physics objects of the scene
    for (std::vector<PhysicsObject*>::iterator it = mPhysicsObjects.begin(); it != mPhysicsObjects.end(); ++it) {

        // Destroy the corresponding rigid body from the physics world
        mPhysicsWorld->destroyRigidBody((*it)->getRigidBody());

        // Destroy the object
        delete (*it);
    }

    // Destroy the physics world
    mPhysicsCommon.destroyPhysicsWorld(static_cast<rp3d::PhysicsWorld*>(mPhysicsWorld));
}

/// Reset the scene
void BoxTowerScene::reset() {

    SceneDemo::reset();

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
