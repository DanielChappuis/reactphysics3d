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
Scene::Scene(Viewer* viewer, const std::string& shaderFolderPath)
      : mViewer(viewer), mLight0(0),
        mPhongShader(shaderFolderPath + "phong.vert", shaderFolderPath + "phong.frag"),
        mIsRunning(false) {

    // Move the light 0
    mLight0.translateWorld(Vector3(7, 15, 15));

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

    float radius = 2.0f;

    // Create all the cubes of the scene
    for (int i=0; i<NB_SPHERES; i++) {

        // Position of the cubes
        float angle = i * 30.0f;
        openglframework::Vector3 position(radius * cos(angle),
                                          10 + i * (BOX_SIZE.y + 0.3f),
                                          0);

        // Create a cube and a corresponding rigid in the dynamics world
        Box* cube = new Box(BOX_SIZE, position , BOX_MASS, mDynamicsWorld);

        // Change the material properties of the rigid body
        rp3d::Material& material = cube->getRigidBody()->getMaterial();
        material.setBounciness(rp3d::decimal(0.4));

        // Add the box the list of box in the scene
        mBoxes.push_back(cube);
    }

    // Create the floor
    openglframework::Vector3 floorPosition(0, 0, 0);
    mFloor = new Box(FLOOR_SIZE, floorPosition, FLOOR_MASS, mDynamicsWorld);

    // The floor must be a static rigid body
    mFloor->getRigidBody()->setType(rp3d::STATIC);

    // Change the material properties of the floor rigid body
    rp3d::Material& material = mFloor->getRigidBody()->getMaterial();
    material.setBounciness(rp3d::decimal(0.3));

    // Start the simulation
    startSimulation();
}

// Destructor
Scene::~Scene() {

    // Stop the physics simulation
    stopSimulation();

    // Destroy the shader
    mPhongShader.destroy();

    // Destroy all the cubes of the scene
    for (std::vector<Box*>::iterator it = mBoxes.begin(); it != mBoxes.end(); ++it) {

        // Destroy the corresponding rigid body from the dynamics world
        mDynamicsWorld->destroyRigidBody((*it)->getRigidBody());

        // Destroy the cube
        delete (*it);
    }

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

        mFloor->updateTransform();

        // Set the color of the awake/sleeping bodies
        for (uint i=0; i<mBoxes.size(); i++) {
            if (mBoxes[i]->getRigidBody()->isSleeping()) {
                mBoxes[i]->setColor(Color(1, 0, 0, 1));
            }
            else {
                mBoxes[i]->setColor(Color(0, 1, 0, 1));
            }
        }
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
    mPhongShader.setVector3Uniform("light0PosCameraSpace",worldToCameraMatrix * mLight0.getOrigin());
    mPhongShader.setVector3Uniform("lightAmbientColor", Vector3(0.3f, 0.3f, 0.3f));
    const Color& diffCol = mLight0.getDiffuseColor();
    const Color& specCol = mLight0.getSpecularColor();
    mPhongShader.setVector3Uniform("light0DiffuseColor", Vector3(diffCol.r, diffCol.g, diffCol.b));
    mPhongShader.setVector3Uniform("light0SpecularColor", Vector3(specCol.r, specCol.g, specCol.b));
    mPhongShader.setFloatUniform("shininess", 60.0f);

    // Render all the cubes of the scene
    for (std::vector<Box*>::iterator it = mBoxes.begin(); it != mBoxes.end(); ++it) {
        (*it)->render(mPhongShader, worldToCameraMatrix);
    }

    // Render the floor
    mFloor->render(mPhongShader, worldToCameraMatrix);

    // Unbind the shader
    mPhongShader.unbind();
}
