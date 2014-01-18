/********************************************************************************
* ReactPhysics3D physics library, http://code.google.com/p/reactphysics3d/      *
* Copyright (c) 2010-2013 Daniel Chappuis                                       *
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
#include "Scene.h"

// Namespaces
using namespace openglframework;

// Constructor
Scene::Scene(Viewer* viewer, const std::string& shaderFolderPath, const std::string& meshFolderPath)
       : mViewer(viewer), mLight0(0),
         mPhongShader(shaderFolderPath + "phong.vert",
                      shaderFolderPath +"phong.frag"), mIsRunning(false) {

    // Move the light 0
    mLight0.translateWorld(Vector3(50, 50, 50));

    // Compute the radius and the center of the scene
    float radiusScene = 30.0f;
    openglframework::Vector3 center(0, 5, 0);

    // Set the center of the scene
    mViewer->setScenePosition(center, radiusScene);

    // Gravity vector in the dynamics world
    rp3d::Vector3 gravity(0, rp3d::decimal(-9.81), 0);

    // Time step for the physics simulation
    rp3d::decimal timeStep = 1.0f / 60.0f;

    // Create the dynamics world for the physics simulation
    mDynamicsWorld = new rp3d::DynamicsWorld(gravity, timeStep);

    // Set the number of iterations of the constraint solver
    mDynamicsWorld->setNbIterationsVelocitySolver(15);

    // Create the static data for the visual contact points
    VisualContactPoint::createStaticData(meshFolderPath);

    float radius = 3.0f;

    // Create all the boxes of the scene
    for (int i=0; i<NB_BOXES; i++) {

        // Position
        float angle = i * 30.0f;
        openglframework::Vector3 position(radius * cos(angle),
                                          60 + i * (BOX_SIZE.y + 0.8f),
                                          radius * sin(angle));

        // Create a sphere and a corresponding rigid in the dynamics world
        Box* box = new Box(BOX_SIZE, position , BOX_MASS, mDynamicsWorld);

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
        ConvexMesh* mesh = new ConvexMesh(position, MESH_MASS, mDynamicsWorld, meshFolderPath);

        // Change the material properties of the rigid body
        rp3d::Material& material = mesh->getRigidBody()->getMaterial();
        material.setBounciness(rp3d::decimal(0.2));

        // Add the mesh the list of sphere in the scene
        mConvexMeshes.push_back(mesh);
    }

    // Create the floor
    openglframework::Vector3 floorPosition(0, 0, 0);
    mFloor = new Box(FLOOR_SIZE, floorPosition, FLOOR_MASS, mDynamicsWorld);

    // The floor must be a static rigid body
    mFloor->getRigidBody()->setType(rp3d::STATIC);

    // Change the material properties of the rigid body
    rp3d::Material& material = mFloor->getRigidBody()->getMaterial();
    material.setBounciness(rp3d::decimal(0.2));

    // Start the simulation
    startSimulation();
}

// Destructor
Scene::~Scene() {

    // Stop the physics simulation
    stopSimulation();

    // Destroy the shader
    mPhongShader.destroy();

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

    // Destroy all the visual contact points
    for (std::vector<VisualContactPoint*>::iterator it = mContactPoints.begin();
         it != mContactPoints.end(); ++it) {
        delete (*it);
    }

    // Destroy the static data for the visual contact points
    VisualContactPoint::destroyStaticData();

    // Destroy the rigid body of the floor
    mDynamicsWorld->destroyRigidBody(mFloor->getRigidBody());

    // Destroy the floor
    delete mFloor;

    // Destroy the dynamics world
    delete mDynamicsWorld;
}

// Take a step for the simulation
void Scene::simulate() {

    // If the physics simulation is running
    if (mIsRunning) {

        // Take a simulation step
        mDynamicsWorld->update();

        // Update the position and orientation of the boxes
        for (std::vector<Box*>::iterator it = mBoxes.begin(); it != mBoxes.end(); ++it) {

            // Update the transform used for the rendering
            (*it)->updateTransform();
        }

        // Update the position and orientation of the sphere
        for (std::vector<Sphere*>::iterator it = mSpheres.begin(); it != mSpheres.end(); ++it) {

            // Update the transform used for the rendering
            (*it)->updateTransform();
        }

        // Update the position and orientation of the cones
        for (std::vector<Cone*>::iterator it = mCones.begin(); it != mCones.end(); ++it) {

            // Update the transform used for the rendering
            (*it)->updateTransform();
        }

        // Update the position and orientation of the cylinders
        for (std::vector<Cylinder*>::iterator it = mCylinders.begin(); it != mCylinders.end(); ++it) {

            // Update the transform used for the rendering
            (*it)->updateTransform();
        }

        // Update the position and orientation of the capsules
        for (std::vector<Capsule*>::iterator it = mCapsules.begin(); it != mCapsules.end(); ++it) {

            // Update the transform used for the rendering
            (*it)->updateTransform();
        }

        // Update the position and orientation of the convex meshes
        for (std::vector<ConvexMesh*>::iterator it = mConvexMeshes.begin();
             it != mConvexMeshes.end(); ++it) {

            // Update the transform used for the rendering
            (*it)->updateTransform();
        }

        // Destroy all the visual contact points
        for (std::vector<VisualContactPoint*>::iterator it = mContactPoints.begin();
             it != mContactPoints.end(); ++it) {
            delete (*it);
        }
        mContactPoints.clear();

        // Generate the new visual contact points
        const std::vector<rp3d::ContactManifold*>& manifolds = mDynamicsWorld->getContactManifolds();
        for (std::vector<rp3d::ContactManifold*>::const_iterator it = manifolds.begin();
             it != manifolds.end(); ++it) {
            for (unsigned int i=0; i<(*it)->getNbContactPoints(); i++) {
                rp3d::ContactPoint* point = (*it)->getContactPoint(i);

                const rp3d::Vector3 pos = point->getWorldPointOnBody1();
                openglframework::Vector3 position(pos.x, pos.y, pos.z);
                VisualContactPoint* visualPoint = new VisualContactPoint(position);
                mContactPoints.push_back(visualPoint);
            }
        }

        mFloor->updateTransform();
    }
}

// Render the scene
void Scene::render() {

    glEnable(GL_DEPTH_TEST);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glEnable(GL_CULL_FACE);

    // Get the world-space to camera-space matrix
    const Camera& camera = mViewer->getCamera();
    const openglframework::Matrix4 worldToCameraMatrix = camera.getTransformMatrix().getInverse();

    // Bind the shader
    mPhongShader.bind();

    // Set the variables of the shader
    mPhongShader.setMatrix4x4Uniform("projectionMatrix", camera.getProjectionMatrix());
    mPhongShader.setVector3Uniform("light0PosCameraSpace", worldToCameraMatrix * mLight0.getOrigin());
    mPhongShader.setVector3Uniform("lightAmbientColor", Vector3(0.3f, 0.3f, 0.3f));
    const Color& diffColLight0 = mLight0.getDiffuseColor();
    const Color& specColLight0 = mLight0.getSpecularColor();
    mPhongShader.setVector3Uniform("light0DiffuseColor", Vector3(diffColLight0.r, diffColLight0.g, diffColLight0.b));
    mPhongShader.setVector3Uniform("light0SpecularColor", Vector3(specColLight0.r, specColLight0.g, specColLight0.b));
    mPhongShader.setFloatUniform("shininess", 200.0f);

    // Render all the boxes of the scene
    for (std::vector<Box*>::iterator it = mBoxes.begin(); it != mBoxes.end(); ++it) {
        (*it)->render(mPhongShader, worldToCameraMatrix);
    }

    // Render all the sphere of the scene
    for (std::vector<Sphere*>::iterator it = mSpheres.begin(); it != mSpheres.end(); ++it) {
        (*it)->render(mPhongShader, worldToCameraMatrix);
    }

    // Render all the cones of the scene
    for (std::vector<Cone*>::iterator it = mCones.begin(); it != mCones.end(); ++it) {
        (*it)->render(mPhongShader, worldToCameraMatrix);
    }

    // Render all the cylinders of the scene
    for (std::vector<Cylinder*>::iterator it = mCylinders.begin(); it != mCylinders.end(); ++it) {
        (*it)->render(mPhongShader, worldToCameraMatrix);
    }

    // Render all the capsules of the scene
    for (std::vector<Capsule*>::iterator it = mCapsules.begin(); it != mCapsules.end(); ++it) {
        (*it)->render(mPhongShader, worldToCameraMatrix);
    }

    // Render all the convex meshes of the scene
    for (std::vector<ConvexMesh*>::iterator it = mConvexMeshes.begin();
         it != mConvexMeshes.end(); ++it) {
        (*it)->render(mPhongShader, worldToCameraMatrix);
    }

    // Render all the visual contact points
    for (std::vector<VisualContactPoint*>::iterator it = mContactPoints.begin();
         it != mContactPoints.end(); ++it) {
        (*it)->render(mPhongShader, worldToCameraMatrix);
    }

    // Render the floor
    mFloor->render(mPhongShader, worldToCameraMatrix);

    // Unbind the shader
    mPhongShader.unbind();
}
