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
#include "BallAndSocketJointsNetScene.h"
#include <cmath>

// Namespaces
using namespace openglframework;
using namespace ballandsocketjointsnetscene;

// Constructor
BallAndSocketJointsNetScene::BallAndSocketJointsNetScene(const std::string& name, EngineSettings& settings, reactphysics3d::PhysicsCommon& physicsCommon)
      : SceneDemo(name, settings, physicsCommon, true) {

    std::string meshFolderPath("meshes/");

    // Compute the radius and the center of the scene
    openglframework::Vector3 center(0, 5, 0);

    // Set the center of the scene
    setScenePosition(center, SCENE_RADIUS);
    setInitZoom(1.3);
    resetCameraToViewAll();

    // Gravity vector in the physics world
    rp3d::Vector3 gravity(0, rp3d::decimal(-9.81), 0);

    rp3d::PhysicsWorld::WorldSettings worldSettings;
    worldSettings.worldName = name;

    // Create the physics world for the physics simulation
    rp3d::PhysicsWorld* physicsWorld = mPhysicsCommon.createPhysicsWorld(worldSettings);
    physicsWorld->setEventListener(this);
    mPhysicsWorld = physicsWorld;

    // Create all the spheres of the scene
    for (int i=0; i<NB_ROWS_NET_SPHERES; i++) {

        for (int j=0; j<NB_ROWS_NET_SPHERES; j++) {

            // Create a sphere and a corresponding rigid in the physics world
            Sphere* sphere = new Sphere(rp3d::BodyType::DYNAMIC, true, SPHERE_RADIUS, mPhysicsCommon, mPhysicsWorld, meshFolderPath);

            // Set the sphere color
            sphere->setColor(mObjectColorDemo);
            sphere->setSleepingColor(mSleepingColorDemo);

            // Change the material properties of the rigid body
            rp3d::Material& material = sphere->getCollider()->getMaterial();
            material.setBounciness(rp3d::decimal(0.0));

            // Add the sphere the list of sphere in the scene
            mNetSpheres[i][j] = sphere;
            mPhysicsObjects.push_back(sphere);
        }
    }

    // Set the position of the spheres before the joints creation
    reset();

    // Create the Ball-and-Socket joints
    createJoints();

    // Create the main sphere
    mMainSphere = new Sphere(rp3d::BodyType::STATIC, true, 7, mPhysicsCommon, mPhysicsWorld, meshFolderPath);
    mMainSphere->setColor(mObjectColorDemo);
    mMainSphere->setSleepingColor(mSleepingColorDemo);
    rp3d::Vector3 initPosition(0, 0, 0);
    rp3d::Quaternion initOrientation = rp3d::Quaternion::identity();
    rp3d::Transform transform(initPosition, initOrientation);
    mMainSphere->setTransform(transform);
    rp3d::Material& material = mMainSphere->getCollider()->getMaterial();
    material.setBounciness(rp3d::decimal(0.0));
    mPhysicsObjects.push_back(mMainSphere);

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
BallAndSocketJointsNetScene::~BallAndSocketJointsNetScene() {

    // Destroy the joints
    for (uint i=0; i < mBallAndSocketJoints.size(); i++) {

        mPhysicsWorld->destroyJoint(mBallAndSocketJoints[i]);
    }

    // Destroy all the rigid bodies of the scene
    for (int i=0; i<NB_ROWS_NET_SPHERES; i++) {

        for (int j=0; j<NB_ROWS_NET_SPHERES; j++) {

            mPhysicsWorld->destroyRigidBody(mNetSpheres[i][j]->getRigidBody());
        }
    }
    mPhysicsWorld->destroyRigidBody(mMainSphere->getRigidBody());

    // Destroy the physics world
    mPhysicsCommon.destroyPhysicsWorld(mPhysicsWorld);
}

// Create the joints
void BallAndSocketJointsNetScene::createJoints() {

    for (int i=0; i<NB_ROWS_NET_SPHERES; i++) {

        for (int j=0; j<NB_ROWS_NET_SPHERES; j++) {

            if (i > 0) {

                // Create the joint info object
                rp3d::RigidBody* body1 = mNetSpheres[i-1][j]->getRigidBody();
                rp3d::RigidBody* body2 = mNetSpheres[i][j]->getRigidBody();
                rp3d::Vector3 body2Position = body2->getTransform().getPosition();
                const rp3d::Vector3 anchorPointWorldSpace = body2Position;
                rp3d::BallAndSocketJointInfo jointInfo(body1, body2, anchorPointWorldSpace);
                jointInfo.isCollisionEnabled = false;
                rp3d::BallAndSocketJoint* joint = dynamic_cast<rp3d::BallAndSocketJoint*>( mPhysicsWorld->createJoint(jointInfo));
                mBallAndSocketJoints.push_back(joint);
            }

            if (j > 0) {

                // Create the joint info object
                rp3d::RigidBody* body1 = mNetSpheres[i][j-1]->getRigidBody();
                rp3d::RigidBody* body2 = mNetSpheres[i][j]->getRigidBody();
                rp3d::Vector3 body2Position = body2->getTransform().getPosition();
                const rp3d::Vector3 anchorPointWorldSpace = body2Position;
                rp3d::BallAndSocketJointInfo jointInfo(body1, body2, anchorPointWorldSpace);
                jointInfo.isCollisionEnabled = false;
                rp3d::BallAndSocketJoint* joint = dynamic_cast<rp3d::BallAndSocketJoint*>( mPhysicsWorld->createJoint(jointInfo));
                mBallAndSocketJoints.push_back(joint);
            }
        }
    }
}

// Reset the scene
void BallAndSocketJointsNetScene::reset() {

    SceneDemo::reset();

    const float space = 0.5f;
    const float startX = -(NB_ROWS_NET_SPHERES / 2.0f * (2.0 * SPHERE_RADIUS + space));
    const float startZ = -(NB_ROWS_NET_SPHERES / 2.0f * (2.0 * SPHERE_RADIUS + space));

    const rp3d::Quaternion initOrientation = rp3d::Quaternion::identity();

    for (int i=0; i<NB_ROWS_NET_SPHERES; i++) {

        for (int j=0; j<NB_ROWS_NET_SPHERES; j++) {

            // Initial position and orientation of the rigid body
            rp3d::Vector3 initPosition(startX + i * (2 * SPHERE_RADIUS + space), 12, startZ + j * (2 * SPHERE_RADIUS + space));
            rp3d::Transform transform(initPosition, initOrientation);

            // Create a box and a corresponding rigid in the physics world
            mNetSpheres[i][j]->setTransform(transform);
        }
    }
}
