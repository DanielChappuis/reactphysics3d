
/********************************************************************************
* ReactPhysics3D physics library, http://www.reactphysics3d.com                 *
* Copyright (c) 2010-2015 Daniel Chappuis                                       *
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
#include "ConcaveMeshScene.h"

// Namespaces
using namespace openglframework;
using namespace trianglemeshscene;

// Constructor
ConcaveMeshScene::ConcaveMeshScene(const std::string& name)
      : SceneDemo(name, SCENE_RADIUS) {

    std::string meshFolderPath("meshes/");

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

    // ---------- Create the cube ----------- //

    // Position
    rp3d::decimal radius = 2.0;
    openglframework::Vector3 spherePos(15, 10, 0);

    // Create a sphere and a corresponding rigid in the dynamics world
    mBox = new Box(Vector3(3, 3, 3), spherePos, 80.1, mDynamicsWorld);

    // Set the sphere color
    mBox->setColor(mDemoColors[1]);
    mBox->setSleepingColor(mRedColorDemo);

    // Change the material properties of the rigid body
    rp3d::Material& sphereMat = mBox->getRigidBody()->getMaterial();
    sphereMat.setBounciness(rp3d::decimal(0.2));

    // ---------- Create the triangular mesh ---------- //

    // Position
    openglframework::Vector3 position(0, 0, 0);
    rp3d::decimal mass = 1.0;

    // Create a convex mesh and a corresponding rigid in the dynamics world
    mConcaveMesh = new ConcaveMesh(position, mass, mDynamicsWorld, meshFolderPath);

    // Set the mesh as beeing static
    mConcaveMesh->getRigidBody()->setType(rp3d::STATIC);

    // Set the box color
    mConcaveMesh->setColor(mDemoColors[0]);
    mConcaveMesh->setSleepingColor(mRedColorDemo);

    // Change the material properties of the rigid body
    rp3d::Material& material = mConcaveMesh->getRigidBody()->getMaterial();
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
ConcaveMeshScene::~ConcaveMeshScene() {

    mDynamicsWorld->destroyRigidBody(mBox->getRigidBody());
    // Destroy the corresponding rigid body from the dynamics world
    mDynamicsWorld->destroyRigidBody(mConcaveMesh->getRigidBody());

    delete mBox;

    // Destroy the convex mesh
    delete mConcaveMesh;

    // Destroy the dynamics world
    delete mDynamicsWorld;
}

// Update the physics world (take a simulation step)
void ConcaveMeshScene::updatePhysics() {

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
void ConcaveMeshScene::update() {

    SceneDemo::update();

    // Update the transform used for the rendering
    mConcaveMesh->updateTransform(mInterpolationFactor);
    mBox->updateTransform(mInterpolationFactor);
}

// Render the scene in a single pass
void ConcaveMeshScene::renderSinglePass(Shader& shader, const openglframework::Matrix4& worldToCameraMatrix) {

    // Bind the shader
    shader.bind();

    mConcaveMesh->render(shader, worldToCameraMatrix);
    mBox->render(shader, worldToCameraMatrix);

    // Unbind the shader
    shader.unbind();
}

// Reset the scene
void ConcaveMeshScene::reset() {

    // Reset the transform
    rp3d::Quaternion initOrientation(0, 0, 3.141/12);
    rp3d::Quaternion initOrientation2(0, 0, 3.141/13);
    //rp3d::Transform transform(rp3d::Vector3(0, 0, 0), initOrientation);
    rp3d::Transform transform(rp3d::Vector3(0, 0, 0), initOrientation2);
    mConcaveMesh->resetTransform(transform);

    rp3d::Vector3 spherePos(2, 15, 0);
    rp3d::Transform sphereTransform(spherePos, initOrientation2);
    mBox->resetTransform(sphereTransform);
}
