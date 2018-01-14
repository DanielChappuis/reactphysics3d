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
#include "CollisionDetectionScene.h"

// Namespaces
using namespace openglframework;
using namespace collisiondetectionscene;

// Constructor
CollisionDetectionScene::CollisionDetectionScene(const std::string& name, EngineSettings& settings)
       : SceneDemo(name, settings, SCENE_RADIUS, false), mMeshFolderPath("meshes/"),
         mContactManager(mPhongShader, mMeshFolderPath),
         mAreNormalsDisplayed(false) {

    mSelectedShapeIndex = 0;
    mIsContactPointsDisplayed = true;
    mIsWireframeEnabled = true;

    // Compute the radius and the center of the scene
    openglframework::Vector3 center(0, 0, 0);

    // Set the center of the scene
    setScenePosition(center, SCENE_RADIUS);

    // Create the dynamics world for the physics simulation
    mPhysicsWorld = new rp3d::CollisionWorld();

#ifdef IS_PROFILING_ACTIVE

    mPhysicsWorld->setProfilerName(name + "_profiler");

#endif

    // ---------- Sphere 1 ---------- //

    // Create a sphere and a corresponding collision body in the dynamics world
    mSphere1 = new Sphere(4, mPhysicsWorld, mMeshFolderPath);
    mAllShapes.push_back(mSphere1);

    // Set the color
    mSphere1->setColor(mGreyColorDemo);
    mSphere1->setSleepingColor(mRedColorDemo);
    //mSphere1->setScaling(0.5f);
    mPhysicsObjects.push_back(mSphere1);

    // ---------- Sphere 2 ---------- //

    // Create a sphere and a corresponding collision body in the dynamics world
    mSphere2 = new Sphere(2, mPhysicsWorld, mMeshFolderPath);
    mAllShapes.push_back(mSphere2);

    // Set the color
    mSphere2->setColor(mGreyColorDemo);
    mSphere2->setSleepingColor(mRedColorDemo);
    mPhysicsObjects.push_back(mSphere2);


    // ---------- Capsule 1 ---------- //

    // Create a cylinder and a corresponding collision body in the dynamics world
    mCapsule1 = new Capsule(CAPSULE_RADIUS, CAPSULE_HEIGHT, mPhysicsWorld, mMeshFolderPath);
    mAllShapes.push_back(mCapsule1);

    // Set the color
    mCapsule1->setColor(mGreyColorDemo);
    mCapsule1->setSleepingColor(mRedColorDemo);
    mPhysicsObjects.push_back(mCapsule1);

    // ---------- Capsule 2 ---------- //

    // Create a cylinder and a corresponding collision body in the dynamics world
    mCapsule2 = new Capsule(CAPSULE_RADIUS, CAPSULE_HEIGHT, mPhysicsWorld, mMeshFolderPath);
    mAllShapes.push_back(mCapsule2);

    // Set the color
    mCapsule2->setColor(mGreyColorDemo);
    mCapsule2->setSleepingColor(mRedColorDemo);
    mPhysicsObjects.push_back(mCapsule2);

    // ---------- Concave Mesh ---------- //

    // Create a convex mesh and a corresponding collision body in the dynamics world
    mConcaveMesh = new ConcaveMesh(mPhysicsWorld, mMeshFolderPath + "city.obj");
    mAllShapes.push_back(mConcaveMesh);

    // Set the color
    mConcaveMesh->setColor(mGreyColorDemo);
    mConcaveMesh->setSleepingColor(mRedColorDemo);
    mPhysicsObjects.push_back(mConcaveMesh);

    // ---------- Box 1 ---------- //

    // Create a cylinder and a corresponding collision body in the dynamics world
    mBox1 = new Box(BOX_SIZE, mPhysicsWorld, mMeshFolderPath);
    mAllShapes.push_back(mBox1);

    // Set the color
    mBox1->setColor(mGreyColorDemo);
    mBox1->setSleepingColor(mRedColorDemo);
    mPhysicsObjects.push_back(mBox1);

    // ---------- Box 2 ---------- //

    // Create a cylinder and a corresponding collision body in the dynamics world
    mBox2 = new Box(openglframework::Vector3(3, 2, 5), mPhysicsWorld, mMeshFolderPath);
    mAllShapes.push_back(mBox2);

    // Set the color
    mBox2->setColor(mGreyColorDemo);
    mBox2->setSleepingColor(mRedColorDemo);
    mPhysicsObjects.push_back(mBox2);

    // ---------- Convex Mesh ---------- //

    // Create a convex mesh and a corresponding collision body in the dynamics world
    mConvexMesh = new ConvexMesh(mPhysicsWorld, mMeshFolderPath + "convexmesh.obj");
    mAllShapes.push_back(mConvexMesh);

    // Set the color
    mConvexMesh->setColor(mGreyColorDemo);
    mConvexMesh->setSleepingColor(mRedColorDemo);
    mPhysicsObjects.push_back(mConvexMesh);

    // ---------- Heightfield ---------- //

    // Create a convex mesh and a corresponding collision body in the dynamics world
    mHeightField = new HeightField(mPhysicsWorld);

    // Set the color
    mHeightField->setColor(mGreyColorDemo);
    mHeightField->setSleepingColor(mRedColorDemo);
	mPhysicsObjects.push_back(mHeightField);

    mAllShapes[mSelectedShapeIndex]->setColor(mBlueColorDemo);
}

// Reset the scene
void CollisionDetectionScene::reset() {

    mSphere1->setTransform(rp3d::Transform(rp3d::Vector3(15, 5, 0), rp3d::Quaternion::identity()));
    mSphere2->setTransform(rp3d::Transform(rp3d::Vector3(0, 6, 0), rp3d::Quaternion::identity()));
    mCapsule1->setTransform(rp3d::Transform(rp3d::Vector3(-8, 7, 0), rp3d::Quaternion::identity()));
    mCapsule2->setTransform(rp3d::Transform(rp3d::Vector3(11, -8, 0), rp3d::Quaternion::identity()));
    mBox1->setTransform(rp3d::Transform(rp3d::Vector3(-4, -7, 0), rp3d::Quaternion::identity()));
    mBox2->setTransform(rp3d::Transform(rp3d::Vector3(0, 9, 0), rp3d::Quaternion::identity()));
    mConvexMesh->setTransform(rp3d::Transform(rp3d::Vector3(-5, 0, 0), rp3d::Quaternion::identity()));
    mConcaveMesh->setTransform(rp3d::Transform(rp3d::Vector3(0, 15, 0), rp3d::Quaternion::identity()));
    mHeightField->setTransform(rp3d::Transform(rp3d::Vector3(0, -22, 0), rp3d::Quaternion::identity()));
}

// Destructor
CollisionDetectionScene::~CollisionDetectionScene() {

    // Destroy the box rigid body from the dynamics world
    //mPhysicsWorld->destroyCollisionBody(mBox->getCollisionBody());
    //delete mBox;

    // Destroy the spheres
    mPhysicsWorld->destroyCollisionBody(mSphere1->getCollisionBody());
    delete mSphere1;

    mPhysicsWorld->destroyCollisionBody(mSphere2->getCollisionBody());
    delete mSphere2;

    mPhysicsWorld->destroyCollisionBody(mCapsule1->getCollisionBody());
    delete mCapsule1;

    mPhysicsWorld->destroyCollisionBody(mCapsule2->getCollisionBody());
    delete mCapsule2;

    mPhysicsWorld->destroyCollisionBody(mBox1->getCollisionBody());
	delete mBox1;

    mPhysicsWorld->destroyCollisionBody(mBox2->getCollisionBody());
	delete mBox2;

    mPhysicsWorld->destroyCollisionBody(mConvexMesh->getCollisionBody());
	delete mConvexMesh;

    mPhysicsWorld->destroyCollisionBody(mConcaveMesh->getCollisionBody());
	delete mConcaveMesh;

    mPhysicsWorld->destroyCollisionBody(mHeightField->getCollisionBody());
	delete mHeightField;

    mContactManager.resetPoints();

    // Destroy the static data for the visual contact points
    VisualContactPoint::destroyStaticData();

    // Destroy the collision world
    delete mPhysicsWorld;
}

// Take a step for the simulation
void CollisionDetectionScene::update() {

    mContactManager.resetPoints();

    mPhysicsWorld->testCollision(&mContactManager);

    SceneDemo::update();
}

void CollisionDetectionScene::selectNextShape() {

    uint previousIndex = mSelectedShapeIndex;
    mSelectedShapeIndex++;
    if (mSelectedShapeIndex >= mAllShapes.size()) {
        mSelectedShapeIndex = 0;
    }

    mAllShapes[previousIndex]->setColor(mGreyColorDemo);
    mAllShapes[mSelectedShapeIndex]->setColor(mBlueColorDemo);
}

// Called when a keyboard event occurs
bool CollisionDetectionScene::keyboardEvent(int key, int scancode, int action, int mods) {

    // If the space key has been pressed
    if (key == GLFW_KEY_SPACE && action == GLFW_PRESS) {
        selectNextShape();
        return true;
    }

    float stepDist = 0.2f;
    float stepAngle = 15 * (3.14f / 180.0f);

    if (key == GLFW_KEY_RIGHT && action == GLFW_PRESS) {
        rp3d::Transform transform = mAllShapes[mSelectedShapeIndex]->getTransform();
        transform.setPosition(transform.getPosition() + rp3d::Vector3(stepDist, 0, 0));
        mAllShapes[mSelectedShapeIndex]->setTransform(transform);
    }
    else if (key == GLFW_KEY_LEFT && action == GLFW_PRESS) {
        rp3d::Transform transform = mAllShapes[mSelectedShapeIndex]->getTransform();
        transform.setPosition(transform.getPosition() + rp3d::Vector3(-stepDist, 0, 0));
        mAllShapes[mSelectedShapeIndex]->setTransform(transform);
    }
    else if (key == GLFW_KEY_UP && action == GLFW_PRESS) {
        rp3d::Transform transform = mAllShapes[mSelectedShapeIndex]->getTransform();
        transform.setPosition(transform.getPosition() + rp3d::Vector3(0, stepDist, 0));
        mAllShapes[mSelectedShapeIndex]->setTransform(transform);
    }
    else if (key == GLFW_KEY_DOWN && action == GLFW_PRESS) {
        rp3d::Transform transform = mAllShapes[mSelectedShapeIndex]->getTransform();
        transform.setPosition(transform.getPosition() + rp3d::Vector3(0, -stepDist, 0));
        mAllShapes[mSelectedShapeIndex]->setTransform(transform);
    }
    else if (key == GLFW_KEY_Z && action == GLFW_PRESS) {
        rp3d::Transform transform = mAllShapes[mSelectedShapeIndex]->getTransform();
        transform.setPosition(transform.getPosition() + rp3d::Vector3(0, 0, stepDist));
        mAllShapes[mSelectedShapeIndex]->setTransform(transform);
    }
    else if (key == GLFW_KEY_H && action == GLFW_PRESS) {
        rp3d::Transform transform = mAllShapes[mSelectedShapeIndex]->getTransform();
        transform.setPosition(transform.getPosition() + rp3d::Vector3(0, 0, -stepDist));
        mAllShapes[mSelectedShapeIndex]->setTransform(transform);
    }
    else if (key == GLFW_KEY_A && action == GLFW_PRESS) {
        rp3d::Transform transform = mAllShapes[mSelectedShapeIndex]->getTransform();
        transform.setOrientation(rp3d::Quaternion::fromEulerAngles(0, stepAngle, 0) * transform.getOrientation());
        mAllShapes[mSelectedShapeIndex]->setTransform(transform);
    }
    else if (key == GLFW_KEY_D && action == GLFW_PRESS) {
        rp3d::Transform transform = mAllShapes[mSelectedShapeIndex]->getTransform();
        transform.setOrientation(rp3d::Quaternion::fromEulerAngles(0, -stepAngle, 0) * transform.getOrientation());
        mAllShapes[mSelectedShapeIndex]->setTransform(transform);
    }
    else if (key == GLFW_KEY_W && action == GLFW_PRESS) {
        rp3d::Transform transform = mAllShapes[mSelectedShapeIndex]->getTransform();
        transform.setOrientation(rp3d::Quaternion::fromEulerAngles(stepAngle, 0, 0) * transform.getOrientation());
        mAllShapes[mSelectedShapeIndex]->setTransform(transform);
    }
    else if (key == GLFW_KEY_S && action == GLFW_PRESS) {
        rp3d::Transform transform = mAllShapes[mSelectedShapeIndex]->getTransform();
        transform.setOrientation(rp3d::Quaternion::fromEulerAngles(-stepAngle, 0, 0) * transform.getOrientation());
        mAllShapes[mSelectedShapeIndex]->setTransform(transform);
    }
    else if (key == GLFW_KEY_F && action == GLFW_PRESS) {
        rp3d::Transform transform = mAllShapes[mSelectedShapeIndex]->getTransform();
        transform.setOrientation(rp3d::Quaternion::fromEulerAngles(0, 0, stepAngle) * transform.getOrientation());
        mAllShapes[mSelectedShapeIndex]->setTransform(transform);
    }
    else if (key == GLFW_KEY_G && action == GLFW_PRESS) {
        rp3d::Transform transform = mAllShapes[mSelectedShapeIndex]->getTransform();
        transform.setOrientation(rp3d::Quaternion::fromEulerAngles(0, 0, -stepAngle) * transform.getOrientation());
        mAllShapes[mSelectedShapeIndex]->setTransform(transform);
    }

    return false;
}
