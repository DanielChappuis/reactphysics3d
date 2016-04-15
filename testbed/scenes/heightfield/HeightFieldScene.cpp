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
#include "HeightFieldScene.h"

// Namespaces
using namespace openglframework;
using namespace heightfieldscene;

// Constructor
HeightFieldScene::HeightFieldScene(const std::string& name) : SceneDemo(name, SCENE_RADIUS) {

    // Compute the radius and the center of the scene
    openglframework::Vector3 center(0, 5, 0);

    // Set the center of the scene
    setScenePosition(center, SCENE_RADIUS);

    // Gravity vector in the dynamics world
    rp3d::Vector3 gravity(0, rp3d::decimal(-9.81), 0);

    // Create the dynamics world for the physics simulation
    mDynamicsWorld = new rp3d::DynamicsWorld(gravity);

    // Set the number of iterations of the constraint solver
    mDynamicsWorld->setNbIterationsVelocitySolver(15);

    // ---------- Create the boxes ----------- //

    // For each box
    for (int i=0; i<NB_BOXES; i++) {

        // Position
        openglframework::Vector3 position(15, 10 + 6 * i, 0);

        // Create a box and a corresponding rigid in the dynamics world
        mBoxes[i] = new Box(Vector3(3, 3, 3), position, 80.1, mDynamicsWorld);

        // Set the box color
        mBoxes[i]->setColor(mDemoColors[2]);
        mBoxes[i]->setSleepingColor(mRedColorDemo);

        // Change the material properties of the rigid body
        rp3d::Material& boxMaterial = mBoxes[i]->getRigidBody()->getMaterial();
        boxMaterial.setBounciness(rp3d::decimal(0.2));
    }

    // ---------- Create the height field ---------- //

    // Position
    openglframework::Vector3 position(0, 0, 0);
    rp3d::decimal mass = 1.0;

    // Create a convex mesh and a corresponding rigid in the dynamics world
    mHeightField = new HeightField(position, mass, mDynamicsWorld);

    // Set the mesh as beeing static
    mHeightField->getRigidBody()->setType(rp3d::STATIC);

    // Set the color
    mHeightField->setColor(mGreyColorDemo);
    mHeightField->setSleepingColor(mGreyColorDemo);

    // Change the material properties of the rigid body
    rp3d::Material& material = mHeightField->getRigidBody()->getMaterial();
    material.setBounciness(rp3d::decimal(0.2));
    material.setFrictionCoefficient(0.1);

    // Get the physics engine parameters
    mEngineSettings.isGravityEnabled = mDynamicsWorld->isGravityEnabled();
    rp3d::Vector3 gravityVector = mDynamicsWorld->getGravity();
    mEngineSettings.gravity = openglframework::Vector3(gravityVector.x, gravityVector.y, gravityVector.z);
    mEngineSettings.isSleepingEnabled = mDynamicsWorld->isSleepingEnabled();
    mEngineSettings.sleepLinearVelocity = mDynamicsWorld->getSleepLinearVelocity();
    mEngineSettings.sleepAngularVelocity = mDynamicsWorld->getSleepAngularVelocity();
    mEngineSettings.nbPositionSolverIterations = mDynamicsWorld->getNbIterationsPositionSolver();
    mEngineSettings.nbVelocitySolverIterations = mDynamicsWorld->getNbIterationsVelocitySolver();
    mEngineSettings.timeBeforeSleep = mDynamicsWorld->getTimeBeforeSleep();
}

// Destructor
HeightFieldScene::~HeightFieldScene() {

    // Destroy the corresponding rigid body from the dynamics world
    for (int i=0; i<NB_BOXES; i++) {
        mDynamicsWorld->destroyRigidBody(mBoxes[i]->getRigidBody());
    }
    mDynamicsWorld->destroyRigidBody(mHeightField->getRigidBody());

    for (int i=0; i<NB_BOXES; i++) {
       delete mBoxes[i];
    }

    // Destroy the convex mesh
    delete mHeightField;

    // Destroy the dynamics world
    delete mDynamicsWorld;
}

// Update the physics world (take a simulation step)
void HeightFieldScene::updatePhysics() {

    // Update the physics engine parameters
    mDynamicsWorld->setIsGratityEnabled(mEngineSettings.isGravityEnabled);
    rp3d::Vector3 gravity(mEngineSettings.gravity.x, mEngineSettings.gravity.y,
                                     mEngineSettings.gravity.z);
    mDynamicsWorld->setGravity(gravity);
    mDynamicsWorld->enableSleeping(mEngineSettings.isSleepingEnabled);
    mDynamicsWorld->setSleepLinearVelocity(mEngineSettings.sleepLinearVelocity);
    mDynamicsWorld->setSleepAngularVelocity(mEngineSettings.sleepAngularVelocity);
    mDynamicsWorld->setNbIterationsPositionSolver(mEngineSettings.nbPositionSolverIterations);
    mDynamicsWorld->setNbIterationsVelocitySolver(mEngineSettings.nbVelocitySolverIterations);
    mDynamicsWorld->setTimeBeforeSleep(mEngineSettings.timeBeforeSleep);

    // Take a simulation step
    mDynamicsWorld->update(mEngineSettings.timeStep);
}

// Update the scene
void HeightFieldScene::update() {

    SceneDemo::update();

    // Update the transform used for the rendering
    mHeightField->updateTransform(mInterpolationFactor);

    for (int i=0; i<NB_BOXES; i++) {
       mBoxes[i]->updateTransform(mInterpolationFactor);
    }
}

// Render the scene in a single pass
void HeightFieldScene::renderSinglePass(Shader& shader, const openglframework::Matrix4& worldToCameraMatrix) {

    // Bind the shader
    shader.bind();

    mHeightField->render(shader, worldToCameraMatrix);

    for (int i=0; i<NB_BOXES; i++) {
       mBoxes[i]->render(shader, worldToCameraMatrix);
    }

    // Unbind the shader
    shader.unbind();
}

// Reset the scene
void HeightFieldScene::reset() {

    // Reset the transform
    rp3d::Transform transform(rp3d::Vector3(0, 0, 0), rp3d::Quaternion::identity());
    mHeightField->resetTransform(transform);

    float heightFieldWidth = 10.0f;
    float stepDist = heightFieldWidth / (NB_BOXES + 1);
    for (int i=0; i<NB_BOXES; i++) {
        rp3d::Vector3 boxPosition(-heightFieldWidth * 0.5f + i * stepDist , 14 + 6.0f * i, -heightFieldWidth * 0.5f + i * stepDist);
        rp3d::Transform boxTransform(boxPosition, rp3d::Quaternion::identity());
        mBoxes[i]->resetTransform(boxTransform);
    }
}
