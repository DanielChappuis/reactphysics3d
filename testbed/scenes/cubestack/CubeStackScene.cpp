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
#include "CubeStackScene.h"

// Namespaces
using namespace openglframework;
using namespace cubestackscene;

// Constructor
CubeStackScene::CubeStackScene(const std::string& name, EngineSettings& settings)
      : SceneDemo(name, settings, SCENE_RADIUS) {

    // Compute the radius and the center of the scene
    openglframework::Vector3 center(0, 5, 0);

    // Set the center of the scene
    setScenePosition(center, SCENE_RADIUS);

    // Gravity vector in the dynamics world
    rp3d::Vector3 gravity(0, rp3d::decimal(-9.81), 0);

    rp3d::WorldSettings worldSettings;
    worldSettings.worldName = name;

    // Create the dynamics world for the physics simulation
    mPhysicsWorld = new rp3d::DynamicsWorld(gravity, worldSettings);

    // Create all the cubes of the scene
    for (int i=1; i<=NB_FLOORS; i++) {

        for (int j=0; j<i; j++) {

            // Create a cube and a corresponding rigid in the dynamics world
            Box* cube = new Box(BOX_SIZE, BOX_MASS, getDynamicsWorld(), mMeshFolderPath);

            // Set the box color
            cube->setColor(mDemoColors[i % mNbDemoColors]);
            cube->setSleepingColor(mRedColorDemo);

            // Change the material properties of the rigid body
            rp3d::Material& material = cube->getRigidBody()->getMaterial();
            material.setBounciness(rp3d::decimal(0.4));

            // Add the box the list of box in the scene
            mBoxes.push_back(cube);
            mPhysicsObjects.push_back(cube);
        }
    }

    // ------------------------- FLOOR ----------------------- //

    // Create the floor
    mFloor = new Box(FLOOR_SIZE, FLOOR_MASS, getDynamicsWorld(), mMeshFolderPath);
    mFloor->setColor(mGreyColorDemo);
    mFloor->setSleepingColor(mGreyColorDemo);

    // The floor must be a static rigid body
    mFloor->getRigidBody()->setType(rp3d::BodyType::STATIC);
    mPhysicsObjects.push_back(mFloor);

    // Get the physics engine parameters
    mEngineSettings.isGravityEnabled = getDynamicsWorld()->isGravityEnabled();
    rp3d::Vector3 gravityVector = getDynamicsWorld()->getGravity();
    mEngineSettings.gravity = openglframework::Vector3(gravityVector.x, gravityVector.y, gravityVector.z);
    mEngineSettings.isSleepingEnabled = getDynamicsWorld()->isSleepingEnabled();
    mEngineSettings.sleepLinearVelocity = getDynamicsWorld()->getSleepLinearVelocity();
    mEngineSettings.sleepAngularVelocity = getDynamicsWorld()->getSleepAngularVelocity();
    mEngineSettings.nbPositionSolverIterations = getDynamicsWorld()->getNbIterationsPositionSolver();
    mEngineSettings.nbVelocitySolverIterations = getDynamicsWorld()->getNbIterationsVelocitySolver();
    mEngineSettings.timeBeforeSleep = getDynamicsWorld()->getTimeBeforeSleep();
}

// Destructor
CubeStackScene::~CubeStackScene() {

    // Destroy all the cubes of the scene
    for (std::vector<Box*>::iterator it = mBoxes.begin(); it != mBoxes.end(); ++it) {

        // Destroy the corresponding rigid body from the dynamics world
        getDynamicsWorld()->destroyRigidBody((*it)->getRigidBody());

        // Destroy the cube
        delete (*it);
    }

    // Destroy the rigid body of the floor
    getDynamicsWorld()->destroyRigidBody(mFloor->getRigidBody());

    // Destroy the floor
    delete mFloor;

    // Destroy the dynamics world
    delete getDynamicsWorld();
}

// Reset the scene
void CubeStackScene::reset() {

    int index = 0;
    for (int i=NB_FLOORS; i > 0; i--) {

        for (int j=0; j<i; j++) {

            // Create all the cubes of the scene
            Box* box = mBoxes[index];

            // Position of the cubes
            rp3d::Vector3 position((-i * 0.5f + j) * (0.1f + BOX_SIZE.x),
                                  BOX_SIZE.y + (NB_FLOORS - i) * (BOX_SIZE.y + 0.1f),
                                  0);

            box->setTransform(rp3d::Transform(position, rp3d::Quaternion::identity()));

            index++;
        }
    }

    mFloor->setTransform(rp3d::Transform(rp3d::Vector3::zero(), rp3d::Quaternion::identity()));
}
