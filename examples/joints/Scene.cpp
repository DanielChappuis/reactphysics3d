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
Scene::Scene(GlutViewer* viewer) : mViewer(viewer), mLight0(0),
                               mPhongShader("shaders/phong.vert",
                                            "shaders/phong.frag"), mIsRunning(false) {

    // Move the light 0
    mLight0.translateWorld(Vector3(7, 15, 15));

    // Compute the radius and the center of the scene
    float radiusScene = 10.0f;
    openglframework::Vector3 center(0, 5, 0);

    // Set the center of the scene
    mViewer->setScenePosition(center, radiusScene);

    // Gravity vector in the dynamics world
    rp3d::Vector3 gravity(0, -9.81, 0);

    // Time step for the physics simulation
    rp3d::decimal timeStep = 1.0f / 60.0f;

    // Create the dynamics world for the physics simulation
    mDynamicsWorld = new rp3d::DynamicsWorld(gravity, timeStep);

    // Set the number of iterations of the constraint solver
    mDynamicsWorld->setNbIterationsVelocitySolver(15);

    // Create the Ball-and-Socket joint
    createBallAndSocketJoints();

    // Create the Slider joint
    createSliderJoint();

    // Create the floor
    createFloor();

    // Start the simulation
    startSimulation();
}

// Destructor
Scene::~Scene() {

    // Stop the physics simulation
    stopSimulation();

    // Destroy the shader
    mPhongShader.destroy();

    // Destroy the joints
    mDynamicsWorld->destroyJoint(mBallAndSocketJoint);
    mDynamicsWorld->destroyJoint(mSliderJoint);

    // Destroy all the boxes of the scene
    mDynamicsWorld->destroyRigidBody(mBallAndSocketJointBox1->getRigidBody());
    mDynamicsWorld->destroyRigidBody(mBallAndSocketJointBox2->getRigidBody());
    mDynamicsWorld->destroyRigidBody(mSliderJointBox1->getRigidBody());
    mDynamicsWorld->destroyRigidBody(mSliderJointBox2->getRigidBody());
    delete mBallAndSocketJointBox1;
    delete mBallAndSocketJointBox2;
    delete mSliderJointBox1;
    delete mSliderJointBox2;

    // Destroy the floor
    mDynamicsWorld->destroyRigidBody(mFloor->getRigidBody());
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
        mBallAndSocketJointBox1->updateTransform();
        mBallAndSocketJointBox2->updateTransform();
        mSliderJointBox1->updateTransform();
        mSliderJointBox2->updateTransform();

        // Update the position and orientation of the floor
        mFloor->updateTransform();
    }
}

// Render the scene
void Scene::render() {

    glEnable(GL_DEPTH_TEST);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glEnable(GL_CULL_FACE);

    // Bind the shader
    mPhongShader.bind();

    // Set the variables of the shader
    const Camera& camera = mViewer->getCamera();
    Matrix4 matrixIdentity;
    matrixIdentity.setToIdentity();
    mPhongShader.setVector3Uniform("cameraWorldPosition", mViewer->getCamera().getOrigin());
    mPhongShader.setMatrix4x4Uniform("worldToCameraMatrix", camera.getTransformMatrix().getInverse());
    mPhongShader.setMatrix4x4Uniform("projectionMatrix", camera.getProjectionMatrix());
    mPhongShader.setVector3Uniform("lightWorldPosition", mLight0.getOrigin());
    mPhongShader.setVector3Uniform("lightAmbientColor", Vector3(0.3f, 0.3f, 0.3f));
    Color& diffCol = mLight0.getDiffuseColor();
    Color& specCol = mLight0.getSpecularColor();
    mPhongShader.setVector3Uniform("lightDiffuseColor", Vector3(diffCol.r, diffCol.g, diffCol.b));
    mPhongShader.setVector3Uniform("lightSpecularColor", Vector3(specCol.r, specCol.g, specCol.b));
    mPhongShader.setFloatUniform("shininess", 60.0f);

    // Render all the boxes
    mBallAndSocketJointBox1->render(mPhongShader);
    mBallAndSocketJointBox2->render(mPhongShader);
    mSliderJointBox1->render(mPhongShader);
    mSliderJointBox2->render(mPhongShader);

    // Render the floor
    mFloor->render(mPhongShader);

    // Unbind the shader
    mPhongShader.unbind();
}

// Create the boxes and joints for the Ball-and-Socket joint example
void Scene::createBallAndSocketJoints() {

    // --------------- Create the first box --------------- //

    // Position of the box
    openglframework::Vector3 positionBox1(0, 15, 0);

    // Create a box and a corresponding rigid in the dynamics world
    mBallAndSocketJointBox1 = new Box(BOX_SIZE, positionBox1 , BOX_MASS, mDynamicsWorld);

    // The fist box cannot move
    mBallAndSocketJointBox1->getRigidBody()->setIsMotionEnabled(false);

    // Set the bouncing factor of the box
    mBallAndSocketJointBox1->getRigidBody()->setRestitution(0.4);

    // --------------- Create the second box --------------- //

    // Position of the box
    openglframework::Vector3 positionBox2(5, 10, 0);

    // Create a box and a corresponding rigid in the dynamics world
    mBallAndSocketJointBox2 = new Box(BOX_SIZE, positionBox2 , BOX_MASS, mDynamicsWorld);

    // The second box is allowed to move
    mBallAndSocketJointBox2->getRigidBody()->setIsMotionEnabled(true);

    // Set the bouncing factor of the box
    mBallAndSocketJointBox2->getRigidBody()->setRestitution(0.4);

    // --------------- Create the joint --------------- //

    // Create the joint info object
    rp3d::RigidBody* body1 = mBallAndSocketJointBox1->getRigidBody();
    rp3d::RigidBody* body2 = mBallAndSocketJointBox2->getRigidBody();
    const rp3d::Vector3 anchorPointWorldSpace(0, 10, 0);
    rp3d::BallAndSocketJointInfo jointInfo(body1, body2, anchorPointWorldSpace);

    // Create the joint in the dynamics world
    mBallAndSocketJoint = dynamic_cast<rp3d::BallAndSocketJoint*>(
                                                  mDynamicsWorld->createJoint(jointInfo));
}

/// Create the boxes and joint for the Slider joint example
void Scene::createSliderJoint() {

    // --------------- Create the first box --------------- //

    // Position of the box
    openglframework::Vector3 positionBox1(-4, 6, 0);

    // Create a box and a corresponding rigid in the dynamics world
    mSliderJointBox1 = new Box(BOX_SIZE, positionBox1 , BOX_MASS, mDynamicsWorld);

    // The fist box cannot move
    mSliderJointBox1->getRigidBody()->setIsMotionEnabled(false);

    // Set the bouncing factor of the box
    mSliderJointBox1->getRigidBody()->setRestitution(0.4);

    // --------------- Create the second box --------------- //

    // Position of the box
    openglframework::Vector3 positionBox2(2, 4, 0);

    // Create a box and a corresponding rigid in the dynamics world
    mSliderJointBox2 = new Box(BOX_SIZE, positionBox2 , BOX_MASS, mDynamicsWorld);

    // The second box is allowed to move
    mSliderJointBox2->getRigidBody()->setIsMotionEnabled(true);

    // Set the bouncing factor of the box
    mSliderJointBox2->getRigidBody()->setRestitution(0.4);

    // --------------- Create the joint --------------- //

    // Create the joint info object
    rp3d::RigidBody* body1 = mSliderJointBox1->getRigidBody();
    rp3d::RigidBody* body2 = mSliderJointBox2->getRigidBody();
    const rp3d::Vector3& body1Position = body1->getTransform().getPosition();
    const rp3d::Vector3& body2Position = body2->getTransform().getPosition();
    const rp3d::Vector3 anchorPointWorldSpace = 0.5 * (body2Position + body1Position);
    const rp3d::Vector3 sliderAxisWorldSpace = body2Position - body1Position;
    rp3d::SliderJointInfo jointInfo(body1, body2, anchorPointWorldSpace, sliderAxisWorldSpace);

    // Create the joint in the dynamics world
    mSliderJoint = dynamic_cast<rp3d::SliderJoint*>(mDynamicsWorld->createJoint(jointInfo));
}

// Create the floor
void Scene::createFloor() {

    // Create the floor
    openglframework::Vector3 floorPosition(0, 0, 0);
    mFloor = new Box(FLOOR_SIZE, floorPosition, FLOOR_MASS, mDynamicsWorld);

    // The floor must be a non-moving rigid body
    mFloor->getRigidBody()->setIsMotionEnabled(false);

    // Set the bouncing factor of the floor
    mFloor->getRigidBody()->setRestitution(0.3);
}
