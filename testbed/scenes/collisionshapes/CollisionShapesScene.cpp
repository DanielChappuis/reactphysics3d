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
#include "CollisionShapesScene.h"

// Namespaces
using namespace openglframework;
using namespace collisionshapesscene;

// Constructor
CollisionShapesScene::CollisionShapesScene(const std::string& name, EngineSettings& settings)
       : SceneDemo(name, settings, SCENE_RADIUS) {

    std::string meshFolderPath("meshes/");

    // Compute the radius and the center of the scene
    openglframework::Vector3 center(0, 5, 0);

    // Set the center of the scene
    setScenePosition(center, SCENE_RADIUS);

    // Gravity vector in the dynamics world
    rp3d::Vector3 gravity(0, -9.81f, 0);

    rp3d::WorldSettings worldSettings;
    worldSettings.worldName = name;

    // Create the dynamics world for the physics simulation
    mPhysicsWorld = new rp3d::DynamicsWorld(gravity, worldSettings);

    for (int i=0; i<NB_COMPOUND_SHAPES; i++) {

        // Create a convex mesh and a corresponding rigid in the dynamics world
        Dumbbell* dumbbell = new Dumbbell(getDynamicsWorld(), meshFolderPath);

        // Set the box color
        dumbbell->setColor(mDemoColors[i % mNbDemoColors]);
        dumbbell->setSleepingColor(mRedColorDemo);

        // Change the material properties of the rigid body
        rp3d::Material& material = dumbbell->getRigidBody()->getMaterial();
        material.setBounciness(rp3d::decimal(0.2));

        // Add the mesh the list of dumbbells in the scene
        mDumbbells.push_back(dumbbell);
		mPhysicsObjects.push_back(dumbbell);
    }

    // Create all the boxes of the scene
    for (int i=0; i<NB_BOXES; i++) {

        // Create a sphere and a corresponding rigid in the dynamics world
        Box* box = new Box(BOX_SIZE, BOX_MASS, getDynamicsWorld(), mMeshFolderPath);

        // Set the box color
        box->setColor(mDemoColors[i % mNbDemoColors]);
        box->setSleepingColor(mRedColorDemo);

        // Change the material properties of the rigid body
        rp3d::Material& material = box->getRigidBody()->getMaterial();
        material.setBounciness(rp3d::decimal(0.2));

        // Add the sphere the list of sphere in the scene
        mBoxes.push_back(box);
		mPhysicsObjects.push_back(box);
    }

    // Create all the spheres of the scene
    for (int i=0; i<NB_SPHERES; i++) {

        // Create a sphere and a corresponding rigid in the dynamics world
        Sphere* sphere = new Sphere(SPHERE_RADIUS, BOX_MASS, getDynamicsWorld(), meshFolderPath);

        // Add some rolling resistance
        sphere->getRigidBody()->getMaterial().setRollingResistance(rp3d::decimal(0.08));

        // Set the box color
        sphere->setColor(mDemoColors[i % mNbDemoColors]);
        sphere->setSleepingColor(mRedColorDemo);

        // Change the material properties of the rigid body
        rp3d::Material& material = sphere->getRigidBody()->getMaterial();
        material.setBounciness(rp3d::decimal(0.2));

        // Add the sphere the list of sphere in the scene
        mSpheres.push_back(sphere);
		mPhysicsObjects.push_back(sphere);
    }

    // Create all the capsules of the scene
    for (int i=0; i<NB_CAPSULES; i++) {

        // Create a cylinder and a corresponding rigid in the dynamics world
        Capsule* capsule = new Capsule(CAPSULE_RADIUS, CAPSULE_HEIGHT, CAPSULE_MASS,
                                       getDynamicsWorld(), meshFolderPath);

        capsule->getRigidBody()->getMaterial().setRollingResistance(rp3d::decimal(0.08f));

        // Set the box color
        capsule->setColor(mDemoColors[i % mNbDemoColors]);
        capsule->setSleepingColor(mRedColorDemo);

        // Change the material properties of the rigid body
        rp3d::Material& material = capsule->getRigidBody()->getMaterial();
        material.setBounciness(rp3d::decimal(0.2));

        // Add the cylinder the list of sphere in the scene
        mCapsules.push_back(capsule);
		mPhysicsObjects.push_back(capsule);
    }

    // Create all the convex meshes of the scene
    for (int i=0; i<NB_MESHES; i++) {

        // Create a convex mesh and a corresponding rigid in the dynamics world
        ConvexMesh* mesh = new ConvexMesh(MESH_MASS, getDynamicsWorld(), meshFolderPath + "convexmesh.obj");

        // Set the box color
        mesh->setColor(mDemoColors[i % mNbDemoColors]);
        mesh->setSleepingColor(mRedColorDemo);

        // Change the material properties of the rigid body
        rp3d::Material& material = mesh->getRigidBody()->getMaterial();
        material.setBounciness(rp3d::decimal(0.2));

        // Add the mesh the list of sphere in the scene
        mConvexMeshes.push_back(mesh);
		mPhysicsObjects.push_back(mesh);
    }

    // ---------- Create the floor ---------

    mFloor = new Box(FLOOR_SIZE, FLOOR_MASS, getDynamicsWorld(), mMeshFolderPath);
	mPhysicsObjects.push_back(mFloor);

    // Set the box color
    mFloor->setColor(mGreyColorDemo);
    mFloor->setSleepingColor(mGreyColorDemo);

    // The floor must be a static rigid body
    mFloor->getRigidBody()->setType(rp3d::BodyType::STATIC);

    // Change the material properties of the rigid body
    rp3d::Material& material = mFloor->getRigidBody()->getMaterial();
    material.setBounciness(rp3d::decimal(0.2));

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
CollisionShapesScene::~CollisionShapesScene() {

    // Destroy all the physics objects of the scene
    for (std::vector<PhysicsObject*>::iterator it = mPhysicsObjects.begin(); it != mPhysicsObjects.end(); ++it) {

        // Destroy the corresponding rigid body from the dynamics world
        getDynamicsWorld()->destroyRigidBody((*it)->getRigidBody());

        // Destroy the object
        delete (*it);
    }

    // Destroy the dynamics world
    delete mPhysicsWorld;
}

/// Reset the scene
void CollisionShapesScene::reset() {

    const float radius = 3.0f;

    for (uint i = 0; i<NB_COMPOUND_SHAPES; i++) {

        // Position
        float angle = i * 30.0f;
        rp3d::Vector3 position(radius * std::cos(angle),
            125 + i * (DUMBBELL_HEIGHT + 0.3f),
            radius * std::sin(angle));

        mDumbbells[i]->setTransform(rp3d::Transform(position, rp3d::Quaternion::identity()));
    }

    // Create all the boxes of the scene
    for (uint i = 0; i<NB_BOXES; i++) {

        // Position
        float angle = i * 30.0f;
        rp3d::Vector3 position(radius * std::cos(angle),
            85 + i * (BOX_SIZE.y + 0.8f),
            radius * std::sin(angle));

        mBoxes[i]->setTransform(rp3d::Transform(position, rp3d::Quaternion::identity()));
    }

    // Create all the spheres of the scene
    for (uint i = 0; i<NB_SPHERES; i++) {

        // Position
        float angle = i * 35.0f;
        rp3d::Vector3 position(radius * std::cos(angle),
            75 + i * (SPHERE_RADIUS + 0.8f),
            radius * std::sin(angle));

        mSpheres[i]->setTransform(rp3d::Transform(position, rp3d::Quaternion::identity()));
    }

    // Create all the capsules of the scene
    for (uint i = 0; i<NB_CAPSULES; i++) {

        // Position
        float angle = i * 45.0f;
        rp3d::Vector3 position(radius * std::cos(angle),
            40 + i * (CAPSULE_HEIGHT + 0.3f),
            radius * std::sin(angle));

        mCapsules[i]->setTransform(rp3d::Transform(position, rp3d::Quaternion::identity()));
    }

    // Create all the convex meshes of the scene
    for (uint i = 0; i<NB_MESHES; i++) {

        // Position
        float angle = i * 30.0f;
        rp3d::Vector3 position(radius * std::cos(angle),
            30 + i * (CAPSULE_HEIGHT + 0.3f),
            radius * std::sin(angle));

        mConvexMeshes[i]->setTransform(rp3d::Transform(position, rp3d::Quaternion::identity()));
    }

    // ---------- Create the triangular mesh ---------- //

    mFloor->setTransform(rp3d::Transform::identity());
}
