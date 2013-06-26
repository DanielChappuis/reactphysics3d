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
#include <cmath>

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

    // Create the Hinge joint
    createPropellerHingeJoint();

    // Create the Fixed joint
    createFixedJoints();

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
    mDynamicsWorld->destroyJoint(mSliderJoint);
    mDynamicsWorld->destroyJoint(mPropellerHingeJoint);
    mDynamicsWorld->destroyJoint(mFixedJoint1);
    mDynamicsWorld->destroyJoint(mFixedJoint2);
    for (int i=0; i<NB_BALLSOCKETJOINT_BOXES-1; i++) {
        mDynamicsWorld->destroyJoint(mBallAndSocketJoints[i]);
    }

    // Destroy all the rigid bodies of the scene
    mDynamicsWorld->destroyRigidBody(mSliderJointBottomBox->getRigidBody());
    mDynamicsWorld->destroyRigidBody(mSliderJointTopBox->getRigidBody());
    mDynamicsWorld->destroyRigidBody(mPropellerBox->getRigidBody());
    mDynamicsWorld->destroyRigidBody(mFixedJointBox1->getRigidBody());
    mDynamicsWorld->destroyRigidBody(mFixedJointBox2->getRigidBody());
    for (int i=0; i<NB_BALLSOCKETJOINT_BOXES; i++) {
        mDynamicsWorld->destroyRigidBody(mBallAndSocketJointChainBoxes[i]->getRigidBody());
    }

    delete mSliderJointBottomBox;
    delete mSliderJointTopBox;
    delete mPropellerBox;
    delete mFixedJointBox1;
    delete mFixedJointBox2;
    for (int i=0; i<NB_BALLSOCKETJOINT_BOXES; i++) {
        delete mBallAndSocketJointChainBoxes[i];
    }

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

        // Update the motor speed of the Slider Joint (to move up and down)
        long double motorSpeed = 3 * cos(mDynamicsWorld->getPhysicsTime() * 1.5);
        mSliderJoint->setMotorSpeed(motorSpeed);

        // Take a simulation step
        mDynamicsWorld->update();

        // Update the position and orientation of the boxes
        mSliderJointBottomBox->updateTransform();
        mSliderJointTopBox->updateTransform();
        mPropellerBox->updateTransform();
        mFixedJointBox1->updateTransform();
        mFixedJointBox2->updateTransform();
        for (int i=0; i<NB_BALLSOCKETJOINT_BOXES; i++) {
            mBallAndSocketJointChainBoxes[i]->updateTransform();
        }

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
    mSliderJointBottomBox->render(mPhongShader);
    mSliderJointTopBox->render(mPhongShader);
    mPropellerBox->render(mPhongShader);
    mFixedJointBox1->render(mPhongShader);
    mFixedJointBox2->render(mPhongShader);
    for (int i=0; i<NB_BALLSOCKETJOINT_BOXES; i++) {
        mBallAndSocketJointChainBoxes[i]->render(mPhongShader);
    }

    // Render the floor
    mFloor->render(mPhongShader);

    // Unbind the shader
    mPhongShader.unbind();
}

// Create the boxes and joints for the Ball-and-Socket joint example
void Scene::createBallAndSocketJoints() {

    // --------------- Create the boxes --------------- //

    openglframework::Vector3 positionBox(0, 15, 5);
    openglframework::Vector3 boxDimension(1, 1, 1);
    const float boxMass = 0.5f;

    for (int i=0; i<NB_BALLSOCKETJOINT_BOXES; i++) {

        // Create a box and a corresponding rigid in the dynamics world
        mBallAndSocketJointChainBoxes[i] = new Box(boxDimension, positionBox , boxMass,
                                                   mDynamicsWorld);

        // The fist box cannot move
        if (i == 0) mBallAndSocketJointChainBoxes[i]->getRigidBody()->setIsMotionEnabled(false);
        else mBallAndSocketJointChainBoxes[i]->getRigidBody()->setIsMotionEnabled(true);

        // Set the bouncing factor of the box
        mBallAndSocketJointChainBoxes[i]->getRigidBody()->setRestitution(0.4);

        positionBox.y -= boxDimension.y + 0.5;
    }

    // --------------- Create the joints --------------- //

    for (int i=0; i<NB_BALLSOCKETJOINT_BOXES-1; i++) {

        // Create the joint info object
        rp3d::RigidBody* body1 = mBallAndSocketJointChainBoxes[i]->getRigidBody();
        rp3d::RigidBody* body2 = mBallAndSocketJointChainBoxes[i+1]->getRigidBody();
        rp3d::Vector3 body1Position = body1->getTransform().getPosition();
        rp3d::Vector3 body2Position = body2->getTransform().getPosition();
        const rp3d::Vector3 anchorPointWorldSpace = 0.5 * (body1Position + body2Position);
        rp3d::BallAndSocketJointInfo jointInfo(body1, body2, anchorPointWorldSpace);

        // Create the joint in the dynamics world
        mBallAndSocketJoints[i] = dynamic_cast<rp3d::BallAndSocketJoint*>(
                    mDynamicsWorld->createJoint(jointInfo));
    }
}

/// Create the boxes and joint for the Slider joint example
void Scene::createSliderJoint() {

    // --------------- Create the first box --------------- //

    // Position of the box
    openglframework::Vector3 positionBox1(0, 2.1, 0);

    // Create a box and a corresponding rigid in the dynamics world
    openglframework::Vector3 box1Dimension(2, 4, 2);
    mSliderJointBottomBox = new Box(box1Dimension, positionBox1 , BOX_MASS, mDynamicsWorld);

    // The fist box cannot move
    mSliderJointBottomBox->getRigidBody()->setIsMotionEnabled(false);

    // Set the bouncing factor of the box
    mSliderJointBottomBox->getRigidBody()->setRestitution(0.4);

    // --------------- Create the second box --------------- //

    // Position of the box
    openglframework::Vector3 positionBox2(0, 4.2, 0);

    // Create a box and a corresponding rigid in the dynamics world
    openglframework::Vector3 box2Dimension(1.5, 4, 1.5);
    mSliderJointTopBox = new Box(box2Dimension, positionBox2 , BOX_MASS, mDynamicsWorld);

    // The second box is allowed to move
    mSliderJointTopBox->getRigidBody()->setIsMotionEnabled(true);

    // Set the bouncing factor of the box
    mSliderJointTopBox->getRigidBody()->setRestitution(0.4);

    // --------------- Create the joint --------------- //

    // Create the joint info object
    rp3d::RigidBody* body1 = mSliderJointBottomBox->getRigidBody();
    rp3d::RigidBody* body2 = mSliderJointTopBox->getRigidBody();
    const rp3d::Vector3& body1Position = body1->getTransform().getPosition();
    const rp3d::Vector3& body2Position = body2->getTransform().getPosition();
    const rp3d::Vector3 anchorPointWorldSpace = 0.5 * (body2Position + body1Position);
    const rp3d::Vector3 sliderAxisWorldSpace = (body2Position - body1Position);
    rp3d::SliderJointInfo jointInfo(body1, body2, anchorPointWorldSpace, sliderAxisWorldSpace,
                                    -1.7, 1.7);
    jointInfo.isMotorEnabled = true;
    jointInfo.motorSpeed = 0.0;
    jointInfo.maxMotorForce = 10000.0;
    jointInfo.isCollisionEnabled = false;

    // Create the joint in the dynamics world
    mSliderJoint = dynamic_cast<rp3d::SliderJoint*>(mDynamicsWorld->createJoint(jointInfo));
}

/// Create the boxes and joint for the Hinge joint example
void Scene::createPropellerHingeJoint() {

    // --------------- Create the propeller box --------------- //

    // Position of the box
    openglframework::Vector3 positionBox1(0, 7, 0);

    // Create a box and a corresponding rigid in the dynamics world
    openglframework::Vector3 boxDimension(10, 1, 1);
    mPropellerBox = new Box(boxDimension, positionBox1 , BOX_MASS, mDynamicsWorld);

    // The fist box cannot move
    mPropellerBox->getRigidBody()->setIsMotionEnabled(true);

    // Set the bouncing factor of the box
    mPropellerBox->getRigidBody()->setRestitution(0.4);

    // --------------- Create the Hinge joint --------------- //

    // Create the joint info object
    rp3d::RigidBody* body1 = mPropellerBox->getRigidBody();
    rp3d::RigidBody* body2 = mSliderJointTopBox->getRigidBody();
    const rp3d::Vector3& body1Position = body1->getTransform().getPosition();
    const rp3d::Vector3& body2Position = body2->getTransform().getPosition();
    const rp3d::Vector3 anchorPointWorldSpace = 0.5 * (body2Position + body1Position);
    const rp3d::Vector3 hingeAxisWorldSpace(0, 1, 0);
    rp3d::HingeJointInfo jointInfo(body1, body2, anchorPointWorldSpace, hingeAxisWorldSpace);
    jointInfo.isMotorEnabled = true;
    jointInfo.motorSpeed = -0.5 * PI;
    jointInfo.maxMotorTorque = 60.0;
    jointInfo.isCollisionEnabled = false;

    // Create the joint in the dynamics world
    mPropellerHingeJoint = dynamic_cast<rp3d::HingeJoint*>(mDynamicsWorld->createJoint(jointInfo));
}

/// Create the boxes and joints for the fixed joints
void Scene::createFixedJoints() {

    // --------------- Create the first box --------------- //

    // Position of the box
    openglframework::Vector3 positionBox1(5, 7, 0);

    // Create a box and a corresponding rigid in the dynamics world
    openglframework::Vector3 boxDimension(1.5, 1.5, 1.5);
    mFixedJointBox1 = new Box(boxDimension, positionBox1 , BOX_MASS, mDynamicsWorld);

    // The fist box cannot move
    mFixedJointBox1->getRigidBody()->setIsMotionEnabled(true);

    // Set the bouncing factor of the box
    mFixedJointBox1->getRigidBody()->setRestitution(0.4);

    // --------------- Create the second box --------------- //

    // Position of the box
    openglframework::Vector3 positionBox2(-5, 7, 0);

    // Create a box and a corresponding rigid in the dynamics world
    mFixedJointBox2 = new Box(boxDimension, positionBox2 , BOX_MASS, mDynamicsWorld);

    // The second box is allowed to move
    mFixedJointBox2->getRigidBody()->setIsMotionEnabled(true);

    // Set the bouncing factor of the box
    mFixedJointBox2->getRigidBody()->setRestitution(0.4);

    // --------------- Create the first fixed joint --------------- //

    // Create the joint info object
    rp3d::RigidBody* body1 = mFixedJointBox1->getRigidBody();
    rp3d::RigidBody* propellerBody = mPropellerBox->getRigidBody();
    const rp3d::Vector3 anchorPointWorldSpace1(5, 7, 0);
    rp3d::FixedJointInfo jointInfo1(body1, propellerBody, anchorPointWorldSpace1);

    // Create the joint in the dynamics world
    mFixedJoint1 = dynamic_cast<rp3d::FixedJoint*>(mDynamicsWorld->createJoint(jointInfo1));

    // --------------- Create the second fixed joint --------------- //

    // Create the joint info object
    rp3d::RigidBody* body2 = mFixedJointBox2->getRigidBody();
    const rp3d::Vector3 anchorPointWorldSpace2(-5, 7, 0);
    rp3d::FixedJointInfo jointInfo2(body2, propellerBody, anchorPointWorldSpace2);

    // Create the joint in the dynamics world
    mFixedJoint2 = dynamic_cast<rp3d::FixedJoint*>(mDynamicsWorld->createJoint(jointInfo2));
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
