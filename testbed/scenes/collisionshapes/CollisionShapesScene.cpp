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
CollisionShapesScene::CollisionShapesScene(const std::string& name)
       : SceneDemo(name, SCENE_RADIUS) {

    std::string meshFolderPath("meshes/");

    // Compute the radius and the center of the scene
    openglframework::Vector3 center(0, 5, 0);

    // Set the center of the scene
    setScenePosition(center, SCENE_RADIUS);

    // Gravity vector in the dynamics world
    rp3d::Vector3 gravity(0, -9.81, 0);

    // Create the dynamics world for the physics simulation
    mDynamicsWorld = new rp3d::DynamicsWorld(gravity);

    // Set the number of iterations of the constraint solver
    mDynamicsWorld->setNbIterationsVelocitySolver(15);

    float radius = 3.0f;

    for (int i=0; i<NB_COMPOUND_SHAPES; i++) {

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
    }

    // Create all the boxes of the scene
    for (int i=0; i<NB_BOXES; i++) {

        // Position
        float angle = i * 30.0f;
        openglframework::Vector3 position(radius * cos(angle),
                                          60 + i * (BOX_SIZE.y + 0.8f),
                                          radius * sin(angle));

        // Create a sphere and a corresponding rigid in the dynamics world
        Box* box = new Box(BOX_SIZE, position , BOX_MASS, mDynamicsWorld);

        // Set the box color
        box->setColor(mDemoColors[i % mNbDemoColors]);
        box->setSleepingColor(mRedColorDemo);

        // Change the material properties of the rigid body
        rp3d::Material& material = box->getRigidBody()->getMaterial();
        material.setBounciness(rp3d::decimal(0.2));

        // Add the sphere the list of sphere in the scene
        mBoxes.push_back(box);
    }

    // Create all the spheres of the scene
    for (int i=0; i<NB_SPHERES; i++) {

        // Position
        float angle = i * 35.0f;
        openglframework::Vector3 position(radius * cos(angle),
                                          50 + i * (SPHERE_RADIUS + 0.8f),
                                          radius * sin(angle));

        // Create a sphere and a corresponding rigid in the dynamics world
        Sphere* sphere = new Sphere(SPHERE_RADIUS, position , BOX_MASS, mDynamicsWorld,
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
    }

    // Create all the cones of the scene
    for (int i=0; i<NB_CONES; i++) {

        // Position
        float angle = i * 50.0f;
        openglframework::Vector3 position(radius * cos(angle),
                                          35 + i * (CONE_HEIGHT + 0.3f),
                                          radius * sin(angle));

        // Create a cone and a corresponding rigid in the dynamics world
        Cone* cone = new Cone(CONE_RADIUS, CONE_HEIGHT, position, CONE_MASS, mDynamicsWorld,
                              meshFolderPath);

        // Add some rolling resistance
        cone->getRigidBody()->getMaterial().setRollingResistance(0.08);

        // Set the box color
        cone->setColor(mDemoColors[i % mNbDemoColors]);
        cone->setSleepingColor(mRedColorDemo);

        // Change the material properties of the rigid body
        rp3d::Material& material = cone->getRigidBody()->getMaterial();
        material.setBounciness(rp3d::decimal(0.2));

        // Add the cone the list of sphere in the scene
        mCones.push_back(cone);
    }

    // Create all the cylinders of the scene
    for (int i=0; i<NB_CYLINDERS; i++) {

        // Position
        float angle = i * 35.0f;
        openglframework::Vector3 position(radius * cos(angle),
                                          25 + i * (CYLINDER_HEIGHT + 0.3f),
                                          radius * sin(angle));

        // Create a cylinder and a corresponding rigid in the dynamics world
        Cylinder* cylinder = new Cylinder(CYLINDER_RADIUS, CYLINDER_HEIGHT, position ,
                                          CYLINDER_MASS, mDynamicsWorld, meshFolderPath);

        // Add some rolling resistance
        cylinder->getRigidBody()->getMaterial().setRollingResistance(0.08);

        // Set the box color
        cylinder->setColor(mDemoColors[i % mNbDemoColors]);
        cylinder->setSleepingColor(mRedColorDemo);

        // Change the material properties of the rigid body
        rp3d::Material& material = cylinder->getRigidBody()->getMaterial();
        material.setBounciness(rp3d::decimal(0.2));

        // Add the cylinder the list of sphere in the scene
        mCylinders.push_back(cylinder);
    }

    // Create all the capsules of the scene
    for (int i=0; i<NB_CAPSULES; i++) {

        // Position
        float angle = i * 45.0f;
        openglframework::Vector3 position(radius * cos(angle),
                                          15 + i * (CAPSULE_HEIGHT + 0.3f),
                                          radius * sin(angle));

        // Create a cylinder and a corresponding rigid in the dynamics world
        Capsule* capsule = new Capsule(CAPSULE_RADIUS, CAPSULE_HEIGHT, position ,
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
    }

    // Create all the convex meshes of the scene
    for (int i=0; i<NB_MESHES; i++) {

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
    }

    // ---------- Create the floor ---------

    openglframework::Vector3 floorPosition(0, 0, 0);
    mFloor = new Box(FLOOR_SIZE, floorPosition, FLOOR_MASS, mDynamicsWorld);

    // Set the box color
    mFloor->setColor(mGreyColorDemo);
    mFloor->setSleepingColor(mGreyColorDemo);

    // The floor must be a static rigid body
    mFloor->getRigidBody()->setType(rp3d::STATIC);

    // Change the material properties of the rigid body
    rp3d::Material& material = mFloor->getRigidBody()->getMaterial();
    material.setBounciness(rp3d::decimal(0.2));

    // ---------- Create the triangular mesh ---------- //

    /*
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
    */

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
CollisionShapesScene::~CollisionShapesScene() {

    // Destroy all the boxes of the scene
    for (std::vector<Box*>::iterator it = mBoxes.begin(); it != mBoxes.end(); ++it) {

        // Destroy the corresponding rigid body from the dynamics world
        mDynamicsWorld->destroyRigidBody((*it)->getRigidBody());

        // Destroy the box
        delete (*it);
    }

    // Destroy all the sphere of the scene
    for (std::vector<Sphere*>::iterator it = mSpheres.begin(); it != mSpheres.end(); ++it) {

        // Destroy the corresponding rigid body from the dynamics world
        mDynamicsWorld->destroyRigidBody((*it)->getRigidBody());

        // Destroy the sphere
        delete (*it);
    }

    // Destroy all the cones of the scene
    for (std::vector<Cone*>::iterator it = mCones.begin(); it != mCones.end(); ++it) {

        // Destroy the corresponding rigid body from the dynamics world
        mDynamicsWorld->destroyRigidBody((*it)->getRigidBody());

        // Destroy the sphere
        delete (*it);
    }

    // Destroy all the cylinders of the scene
    for (std::vector<Cylinder*>::iterator it = mCylinders.begin(); it != mCylinders.end(); ++it) {

        // Destroy the corresponding rigid body from the dynamics world
        mDynamicsWorld->destroyRigidBody((*it)->getRigidBody());

        // Destroy the sphere
        delete (*it);
    }

    // Destroy all the capsules of the scene
    for (std::vector<Capsule*>::iterator it = mCapsules.begin(); it != mCapsules.end(); ++it) {

        // Destroy the corresponding rigid body from the dynamics world
        mDynamicsWorld->destroyRigidBody((*it)->getRigidBody());

        // Destroy the sphere
        delete (*it);
    }

    // Destroy all the convex meshes of the scene
    for (std::vector<ConvexMesh*>::iterator it = mConvexMeshes.begin();
         it != mConvexMeshes.end(); ++it) {

        // Destroy the corresponding rigid body from the dynamics world
        mDynamicsWorld->destroyRigidBody((*it)->getRigidBody());

        // Destroy the convex mesh
        delete (*it);
    }

    // Destroy all the dumbbell of the scene
    for (std::vector<Dumbbell*>::iterator it = mDumbbells.begin();
         it != mDumbbells.end(); ++it) {

        // Destroy the corresponding rigid body from the dynamics world
        mDynamicsWorld->destroyRigidBody((*it)->getRigidBody());

        // Destroy the convex mesh
        delete (*it);
    }

    // Destroy the rigid body of the floor
    mDynamicsWorld->destroyRigidBody(mFloor->getRigidBody());
    //mDynamicsWorld->destroyRigidBody(mConcaveMesh->getRigidBody());

    // Destroy the floor
    delete mFloor;

    // Destroy the convex mesh
    //delete mConcaveMesh;

    // Destroy the dynamics world
    delete mDynamicsWorld;
}

// Update the physics world (take a simulation step)
void CollisionShapesScene::updatePhysics() {

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

// Take a step for the simulation
void CollisionShapesScene::update() {

    SceneDemo::update();

    // Update the position and orientation of the boxes
    for (std::vector<Box*>::iterator it = mBoxes.begin(); it != mBoxes.end(); ++it) {

        // Update the transform used for the rendering
        (*it)->updateTransform(mInterpolationFactor);
    }

    // Update the position and orientation of the sphere
    for (std::vector<Sphere*>::iterator it = mSpheres.begin(); it != mSpheres.end(); ++it) {

        // Update the transform used for the rendering
        (*it)->updateTransform(mInterpolationFactor);
    }

    // Update the position and orientation of the cones
    for (std::vector<Cone*>::iterator it = mCones.begin(); it != mCones.end(); ++it) {

        // Update the transform used for the rendering
        (*it)->updateTransform(mInterpolationFactor);
    }

    // Update the position and orientation of the cylinders
    for (std::vector<Cylinder*>::iterator it = mCylinders.begin(); it != mCylinders.end(); ++it) {

        // Update the transform used for the rendering
        (*it)->updateTransform(mInterpolationFactor);
    }

    // Update the position and orientation of the capsules
    for (std::vector<Capsule*>::iterator it = mCapsules.begin(); it != mCapsules.end(); ++it) {

        // Update the transform used for the rendering
        (*it)->updateTransform(mInterpolationFactor);
    }

    // Update the position and orientation of the convex meshes
    for (std::vector<ConvexMesh*>::iterator it = mConvexMeshes.begin();
         it != mConvexMeshes.end(); ++it) {

        // Update the transform used for the rendering
        (*it)->updateTransform(mInterpolationFactor);
    }

    // Update the position and orientation of the dumbbells
    for (std::vector<Dumbbell*>::iterator it = mDumbbells.begin();
         it != mDumbbells.end(); ++it) {

        // Update the transform used for the rendering
        (*it)->updateTransform(mInterpolationFactor);
    }

    //mConcaveMesh->updateTransform(mInterpolationFactor);

    mFloor->updateTransform(mInterpolationFactor);
}

// Render the scene
void CollisionShapesScene::renderSinglePass(openglframework::Shader& shader,
                                            const openglframework::Matrix4& worldToCameraMatrix) {

    // Bind the shader
    shader.bind();

    // Render all the boxes of the scene
    for (std::vector<Box*>::iterator it = mBoxes.begin(); it != mBoxes.end(); ++it) {
        (*it)->render(shader, worldToCameraMatrix);
    }

    // Render all the sphere of the scene
    for (std::vector<Sphere*>::iterator it = mSpheres.begin(); it != mSpheres.end(); ++it) {
        (*it)->render(shader, worldToCameraMatrix);
    }

    // Render all the cones of the scene
    for (std::vector<Cone*>::iterator it = mCones.begin(); it != mCones.end(); ++it) {
        (*it)->render(shader, worldToCameraMatrix);
    }

    // Render all the cylinders of the scene
    for (std::vector<Cylinder*>::iterator it = mCylinders.begin(); it != mCylinders.end(); ++it) {
        (*it)->render(shader, worldToCameraMatrix);
    }

    // Render all the capsules of the scene
    for (std::vector<Capsule*>::iterator it = mCapsules.begin(); it != mCapsules.end(); ++it) {
        (*it)->render(shader, worldToCameraMatrix);
    }

    // Render all the convex meshes of the scene
    for (std::vector<ConvexMesh*>::iterator it = mConvexMeshes.begin();
         it != mConvexMeshes.end(); ++it) {
        (*it)->render(shader, worldToCameraMatrix);
    }

    // Render all the dumbbells of the scene
    for (std::vector<Dumbbell*>::iterator it = mDumbbells.begin();
         it != mDumbbells.end(); ++it) {
        (*it)->render(shader, worldToCameraMatrix);
    }

    // Render the floor
    mFloor->render(shader, worldToCameraMatrix);

    //mConcaveMesh->render(shader, worldToCameraMatrix);

    // Unbind the shader
    shader.unbind();
}

/// Reset the scene
void CollisionShapesScene::reset() {

    float radius = 3.0f;

    for (int i=0; i<NB_COMPOUND_SHAPES; i++) {

        // Position
        float angle = i * 30.0f;
        openglframework::Vector3 position(radius * cos(angle),
                                          100 + i * (DUMBBELL_HEIGHT + 0.3f),
                                          radius * sin(angle));

        // Initial position and orientation of the rigid body
        rp3d::Vector3 initPosition(position.x, position.y, position.z);
        rp3d::Quaternion initOrientation = rp3d::Quaternion::identity();
        rp3d::Transform transform(initPosition, initOrientation);

        // Reset the transform
        mDumbbells[i]->resetTransform(transform);
    }

    // Create all the boxes of the scene
    for (int i=0; i<NB_BOXES; i++) {

        // Position
        float angle = i * 30.0f;
        openglframework::Vector3 position(radius * cos(angle),
                                          60 + i * (BOX_SIZE.y + 0.8f),
                                          radius * sin(angle));

        // Initial position and orientation of the rigid body
        rp3d::Vector3 initPosition(position.x, position.y, position.z);
        rp3d::Quaternion initOrientation = rp3d::Quaternion::identity();
        rp3d::Transform transform(initPosition, initOrientation);

        // Reset the transform
        mBoxes[i]->resetTransform(transform);
    }

    // Create all the spheres of the scene
    for (int i=0; i<NB_SPHERES; i++) {

        // Position
        float angle = i * 35.0f;
        openglframework::Vector3 position(radius * cos(angle),
                                          50 + i * (SPHERE_RADIUS + 0.8f),
                                          radius * sin(angle));

        // Initial position and orientation of the rigid body
        rp3d::Vector3 initPosition(position.x, position.y, position.z);
        rp3d::Quaternion initOrientation = rp3d::Quaternion::identity();
        rp3d::Transform transform(initPosition, initOrientation);

        // Reset the transform
        mSpheres[i]->resetTransform(transform);
    }

    // Create all the cones of the scene
    for (int i=0; i<NB_CONES; i++) {

        // Position
        float angle = i * 50.0f;
        openglframework::Vector3 position(radius * cos(angle),
                                          35 + i * (CONE_HEIGHT + 0.3f),
                                          radius * sin(angle));

        // Initial position and orientation of the rigid body
        rp3d::Vector3 initPosition(position.x, position.y, position.z);
        rp3d::Quaternion initOrientation = rp3d::Quaternion::identity();
        rp3d::Transform transform(initPosition, initOrientation);

        // Reset the transform
        mCones[i]->resetTransform(transform);
    }

    // Create all the cylinders of the scene
    for (int i=0; i<NB_CYLINDERS; i++) {

        // Position
        float angle = i * 35.0f;
        openglframework::Vector3 position(radius * cos(angle),
                                          25 + i * (CYLINDER_HEIGHT + 0.3f),
                                          radius * sin(angle));

        // Initial position and orientation of the rigid body
        rp3d::Vector3 initPosition(position.x, position.y, position.z);
        rp3d::Quaternion initOrientation = rp3d::Quaternion::identity();
        rp3d::Transform transform(initPosition, initOrientation);

        // Reset the transform
        mCylinders[i]->resetTransform(transform);
    }

    // Create all the capsules of the scene
    for (int i=0; i<NB_CAPSULES; i++) {

        // Position
        float angle = i * 45.0f;
        openglframework::Vector3 position(radius * cos(angle),
                                          15 + i * (CAPSULE_HEIGHT + 0.3f),
                                          radius * sin(angle));

        // Initial position and orientation of the rigid body
        rp3d::Vector3 initPosition(position.x, position.y, position.z);
        rp3d::Quaternion initOrientation = rp3d::Quaternion::identity();
        rp3d::Transform transform(initPosition, initOrientation);

        // Reset the transform
        mCapsules[i]->resetTransform(transform);
    }

    // Create all the convex meshes of the scene
    for (int i=0; i<NB_MESHES; i++) {

        // Position
        float angle = i * 30.0f;
        openglframework::Vector3 position(radius * cos(angle),
                                          5 + i * (CAPSULE_HEIGHT + 0.3f),
                                          radius * sin(angle));

        // Initial position and orientation of the rigid body
        rp3d::Vector3 initPosition(position.x, position.y, position.z);
        rp3d::Quaternion initOrientation = rp3d::Quaternion::identity();
        rp3d::Transform transform(initPosition, initOrientation);

        // Reset the transform
        mConvexMeshes[i]->resetTransform(transform);
    }
}
