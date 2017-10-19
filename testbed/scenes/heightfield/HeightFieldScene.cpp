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

	std::string meshFolderPath("meshes/");

    // Compute the radius and the center of the scene
    openglframework::Vector3 center(0, 5, 0);

    // Set the center of the scene
    setScenePosition(center, SCENE_RADIUS);

    // Gravity vector in the dynamics world
    rp3d::Vector3 gravity(0, rp3d::decimal(-9.81), 0);

    // Create the dynamics world for the physics simulation
    mDynamicsWorld = new rp3d::DynamicsWorld(gravity);

	float radius = 3.0f;

	for (int i = 0; i<NB_COMPOUND_SHAPES; i++) {

		// Position
		float angle = i * 30.0f;
		openglframework::Vector3 position(radius * cos(angle),
			100 + i * (DUMBBELL_HEIGHT + 0.3f),
			radius * sin(angle));

		// Create a convex mesh and a corresponding rigid in the dynamics world
		Dumbbell* dumbbell = new Dumbbell(position, mDynamicsWorld, meshFolderPath);

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
	for (int i = 0; i<NB_BOXES; i++) {

		// Position
		float angle = i * 30.0f;
		openglframework::Vector3 position(radius * cos(angle),
			60 + i * (BOX_SIZE.y + 0.8f),
			radius * sin(angle));

		// Create a sphere and a corresponding rigid in the dynamics world
		Box* box = new Box(BOX_SIZE, position, BOX_MASS, mDynamicsWorld, mMeshFolderPath);

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
	for (int i = 0; i<NB_SPHERES; i++) {

		// Position
		float angle = i * 35.0f;
		openglframework::Vector3 position(radius * cos(angle),
			50 + i * (SPHERE_RADIUS + 0.8f),
			radius * sin(angle));

		// Create a sphere and a corresponding rigid in the dynamics world
		Sphere* sphere = new Sphere(SPHERE_RADIUS, position, BOX_MASS, mDynamicsWorld,
			meshFolderPath);

		// Add some rolling resistance
		sphere->getRigidBody()->getMaterial().setRollingResistance(0.08);

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
	for (int i = 0; i<NB_CAPSULES; i++) {

		// Position
		float angle = i * 45.0f;
		openglframework::Vector3 position(radius * cos(angle),
			15 + i * (CAPSULE_HEIGHT + 0.3f),
			radius * sin(angle));

		// Create a cylinder and a corresponding rigid in the dynamics world
		Capsule* capsule = new Capsule(CAPSULE_RADIUS, CAPSULE_HEIGHT, position,
			CAPSULE_MASS, mDynamicsWorld, meshFolderPath);

		capsule->getRigidBody()->getMaterial().setRollingResistance(0.08);

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
	for (int i = 0; i<NB_MESHES; i++) {

		// Position
		float angle = i * 30.0f;
		openglframework::Vector3 position(radius * cos(angle),
			5 + i * (CAPSULE_HEIGHT + 0.3f),
			radius * sin(angle));

		// Create a convex mesh and a corresponding rigid in the dynamics world
		ConvexMesh* mesh = new ConvexMesh(position, MESH_MASS, mDynamicsWorld, meshFolderPath + "convexmesh.obj");

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

    // ---------- Create the height field ---------- //

    // Position
    openglframework::Vector3 position(0, 0, 0);
    rp3d::decimal mass = 1.0;

    // Create a convex mesh and a corresponding rigid in the dynamics world
    mHeightField = new HeightField(position, mass, mDynamicsWorld);

    // Set the mesh as beeing static
    mHeightField->getRigidBody()->setType(rp3d::BodyType::STATIC);

	mPhysicsObjects.push_back(mHeightField);

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

	// Destroy all the physics objects of the scene
	for (std::vector<PhysicsObject*>::iterator it = mPhysicsObjects.begin(); it != mPhysicsObjects.end(); ++it) {

		// Destroy the corresponding rigid body from the dynamics world
		mDynamicsWorld->destroyRigidBody((*it)->getRigidBody());

		// Destroy the object
		delete (*it);
	}

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

// Reset the scene
void HeightFieldScene::reset() {

    // Reset the transform
    rp3d::Transform transform(rp3d::Vector3(0, 0, 0), rp3d::Quaternion::identity());
    mHeightField->setTransform(transform);

    float heightFieldWidth = 10.0f;
    float stepDist = heightFieldWidth / (NB_BOXES + 1);
    for (int i=0; i<NB_BOXES; i++) {
        rp3d::Vector3 boxPosition(-heightFieldWidth * 0.5f + i * stepDist , 14 + 6.0f * i, -heightFieldWidth * 0.5f + i * stepDist);
        rp3d::Transform boxTransform(boxPosition, rp3d::Quaternion::identity());
        mBoxes[i]->setTransform(boxTransform);
    }
}
