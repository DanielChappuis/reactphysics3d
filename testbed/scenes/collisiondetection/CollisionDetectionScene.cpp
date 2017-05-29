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
CollisionDetectionScene::CollisionDetectionScene(const std::string& name)
       : SceneDemo(name, SCENE_RADIUS, false), mMeshFolderPath("meshes/"),
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
    mCollisionWorld = new rp3d::CollisionWorld();

    // ---------- Sphere 1 ---------- //
    openglframework::Vector3 position1(12, 0, 0);

    // Create a sphere and a corresponding collision body in the dynamics world
    mSphere1 = new Sphere(4, position1, mCollisionWorld, mMeshFolderPath);
    mAllShapes.push_back(mSphere1);

    // Set the color
    mSphere1->setColor(mGreyColorDemo);
    mSphere1->setSleepingColor(mRedColorDemo);

    // ---------- Sphere 2 ---------- //
    openglframework::Vector3 position2(0, 0, 0);

    // Create a sphere and a corresponding collision body in the dynamics world
    mSphere2 = new Sphere(2, position2, mCollisionWorld, mMeshFolderPath);
    mAllShapes.push_back(mSphere2);

    // Set the color
    mSphere2->setColor(mGreyColorDemo);
    mSphere2->setSleepingColor(mRedColorDemo);

	// ---------- Capsule 1 ---------- //
	openglframework::Vector3 position3(8, 0, 0);

	// Create a cylinder and a corresponding collision body in the dynamics world
	mCapsule1 = new Capsule(CAPSULE_RADIUS, CAPSULE_HEIGHT, position3, mCollisionWorld, mMeshFolderPath);
	mAllShapes.push_back(mCapsule1);

	// Set the color
	mCapsule1->setColor(mGreyColorDemo);
	mCapsule1->setSleepingColor(mRedColorDemo);

    // ---------- Capsule 2 ---------- //
    openglframework::Vector3 position4(-8, 0, 0);

    // Create a cylinder and a corresponding collision body in the dynamics world
    mCapsule2 = new Capsule(CAPSULE_RADIUS, CAPSULE_HEIGHT, position4, mCollisionWorld, mMeshFolderPath);
    mAllShapes.push_back(mCapsule2);

    // Set the color
    mCapsule2->setColor(mGreyColorDemo);
    mCapsule2->setSleepingColor(mRedColorDemo);

	// ---------- Box 1 ---------- //
	openglframework::Vector3 position5(0, -12, 0);

	// Create a cylinder and a corresponding collision body in the dynamics world
	mBox1 = new Box(BOX_SIZE, position5, mCollisionWorld, mMeshFolderPath);
	mAllShapes.push_back(mBox1);

	// Set the color
	mBox1->setColor(mGreyColorDemo);
	mBox1->setSleepingColor(mRedColorDemo);

	// ---------- Box 2 ---------- //
	openglframework::Vector3 position6(0, 12, 0);

	// Create a cylinder and a corresponding collision body in the dynamics world
	mBox2 = new Box(openglframework::Vector3(3, 2, 5), position6, mCollisionWorld, mMeshFolderPath);
	mAllShapes.push_back(mBox2);

	// Set the color
	mBox2->setColor(mGreyColorDemo);
	mBox2->setSleepingColor(mRedColorDemo);

    // ---------- Cone ---------- //
    //openglframework::Vector3 position4(0, 0, 0);

    // Create a cone and a corresponding collision body in the dynamics world
    //mCone = new Cone(CONE_RADIUS, CONE_HEIGHT, position4, mCollisionWorld,
    //                 mMeshFolderPath);

    // Set the color
    //mCone->setColor(mGreyColorDemo);
    //mCone->setSleepingColor(mRedColorDemo);

    // ---------- Cylinder ---------- //
    //openglframework::Vector3 position5(0, 0, 0);

    // Create a cylinder and a corresponding collision body in the dynamics world
    //mCylinder = new Cylinder(CYLINDER_RADIUS, CYLINDER_HEIGHT, position5,
    //                         mCollisionWorld, mMeshFolderPath);

    // Set the color
    //mCylinder->setColor(mGreyColorDemo);
    //mCylinder->setSleepingColor(mRedColorDemo);


    // ---------- Convex Mesh ---------- //
    //openglframework::Vector3 position7(0, 0, 0);

    // Create a convex mesh and a corresponding collision body in the dynamics world
    //mConvexMesh = new ConvexMesh(position7, mCollisionWorld, mMeshFolderPath + "convexmesh.obj");

    // Set the color
    //mConvexMesh->setColor(mGreyColorDemo);
    //mConvexMesh->setSleepingColor(mRedColorDemo);

    // ---------- Concave Mesh ---------- //
    //openglframework::Vector3 position8(0, 0, 0);

    // Create a convex mesh and a corresponding collision body in the dynamics world
    //mConcaveMesh = new ConcaveMesh(position8, mCollisionWorld, mMeshFolderPath + "city.obj");

    // Set the color
    //mConcaveMesh->setColor(mGreyColorDemo);
    //mConcaveMesh->setSleepingColor(mRedColorDemo);

    // ---------- Heightfield ---------- //
    //openglframework::Vector3 position9(0, 0, 0);

    // Create a convex mesh and a corresponding collision body in the dynamics world
    //mHeightField = new HeightField(position9, mCollisionWorld);

    // Set the color
    //mHeightField->setColor(mGreyColorDemo);
    //mHeightField->setSleepingColor(mRedColorDemo);

    mAllShapes[mSelectedShapeIndex]->setColor(mBlueColorDemo);
}

// Reset the scene
void CollisionDetectionScene::reset() {

}

// Destructor
CollisionDetectionScene::~CollisionDetectionScene() {

    // Destroy the box rigid body from the dynamics world
    //mCollisionWorld->destroyCollisionBody(mBox->getCollisionBody());
    //delete mBox;

    // Destroy the spheres
    mCollisionWorld->destroyCollisionBody(mSphere1->getCollisionBody());
    delete mSphere1;

    mCollisionWorld->destroyCollisionBody(mSphere2->getCollisionBody());
    delete mSphere2;

    mCollisionWorld->destroyCollisionBody(mCapsule1->getCollisionBody());
    delete mCapsule1;

    mCollisionWorld->destroyCollisionBody(mCapsule2->getCollisionBody());
    delete mCapsule2;

	mCollisionWorld->destroyCollisionBody(mBox1->getCollisionBody());
	delete mBox1;

	mCollisionWorld->destroyCollisionBody(mBox2->getCollisionBody());
	delete mBox2;

    /*
    // Destroy the corresponding rigid body from the dynamics world
    mCollisionWorld->destroyCollisionBody(mCone->getCollisionBody());
    delete mCone;

    // Destroy the corresponding rigid body from the dynamics world
    mCollisionWorld->destroyCollisionBody(mCylinder->getCollisionBody());

    // Destroy the sphere
    delete mCylinder;

    // Destroy the corresponding rigid body from the dynamics world
    mCollisionWorld->destroyCollisionBody(mCapsule->getCollisionBody());

    // Destroy the sphere
    delete mCapsule;

    // Destroy the corresponding rigid body from the dynamics world
    mCollisionWorld->destroyCollisionBody(mConvexMesh->getCollisionBody());

    // Destroy the convex mesh
    delete mConvexMesh;

    // Destroy the corresponding rigid body from the dynamics world
    mCollisionWorld->destroyCollisionBody(mDumbbell->getCollisionBody());

    // Destroy the dumbbell
    delete mDumbbell;

    // Destroy the corresponding rigid body from the dynamics world
    mCollisionWorld->destroyCollisionBody(mConcaveMesh->getCollisionBody());

    // Destroy the convex mesh
    delete mConcaveMesh;

    // Destroy the corresponding rigid body from the dynamics world
    mCollisionWorld->destroyCollisionBody(mHeightField->getCollisionBody());

    // Destroy the convex mesh
    delete mHeightField;
    */

    mContactManager.resetPoints();

    // Destroy the static data for the visual contact points
    VisualContactPoint::destroyStaticData();

    // Destroy the collision world
    delete mCollisionWorld;
}

// Update the physics world (take a simulation step)
void CollisionDetectionScene::updatePhysics() {


}

// Take a step for the simulation
void CollisionDetectionScene::update() {

    mContactManager.resetPoints();

    mCollisionWorld->testCollision(&mContactManager);

    SceneDemo::update();
}

// Render the scene
void CollisionDetectionScene::renderSinglePass(openglframework::Shader& shader,
                                    const openglframework::Matrix4& worldToCameraMatrix) {

    // Render the shapes
    if (mSphere1->getCollisionBody()->isActive()) mSphere1->render(shader, worldToCameraMatrix, mIsWireframeEnabled);
    if (mSphere2->getCollisionBody()->isActive()) mSphere2->render(shader, worldToCameraMatrix, mIsWireframeEnabled);
	if (mCapsule1->getCollisionBody()->isActive()) mCapsule1->render(shader, worldToCameraMatrix, mIsWireframeEnabled);
    if (mCapsule2->getCollisionBody()->isActive()) mCapsule2->render(shader, worldToCameraMatrix, mIsWireframeEnabled);
	if (mBox1->getCollisionBody()->isActive()) mBox1->render(shader, worldToCameraMatrix, mIsWireframeEnabled);
	if (mBox2->getCollisionBody()->isActive()) mBox2->render(shader, worldToCameraMatrix, mIsWireframeEnabled);

    /*
    if (mBox->getCollisionBody()->isActive()) mBox->render(shader, worldToCameraMatrix);
    if (mCone->getCollisionBody()->isActive()) mCone->render(shader, worldToCameraMatrix);
    if (mCylinder->getCollisionBody()->isActive()) mCylinder->render(shader, worldToCameraMatrix);
    if (mCapsule->getCollisionBody()->isActive()) mCapsule->render(shader, worldToCameraMatrix);
    if (mConvexMesh->getCollisionBody()->isActive()) mConvexMesh->render(shader, worldToCameraMatrix);
    if (mDumbbell->getCollisionBody()->isActive()) mDumbbell->render(shader, worldToCameraMatrix);
    if (mConcaveMesh->getCollisionBody()->isActive()) mConcaveMesh->render(shader, worldToCameraMatrix);
    if (mHeightField->getCollisionBody()->isActive()) mHeightField->render(shader, worldToCameraMatrix);
    */

    shader.unbind();
}

void CollisionDetectionScene::selectNextShape() {

    int previousIndex = mSelectedShapeIndex;
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
        transform.setOrientation(rp3d::Quaternion(0, stepAngle, 0) * transform.getOrientation());
        mAllShapes[mSelectedShapeIndex]->setTransform(transform);
    }
    else if (key == GLFW_KEY_D && action == GLFW_PRESS) {
        rp3d::Transform transform = mAllShapes[mSelectedShapeIndex]->getTransform();
        transform.setOrientation(rp3d::Quaternion(0, -stepAngle, 0) * transform.getOrientation());
        mAllShapes[mSelectedShapeIndex]->setTransform(transform);
    }
    else if (key == GLFW_KEY_W && action == GLFW_PRESS) {
        rp3d::Transform transform = mAllShapes[mSelectedShapeIndex]->getTransform();
        transform.setOrientation(rp3d::Quaternion(stepAngle, 0, 0) * transform.getOrientation());
        mAllShapes[mSelectedShapeIndex]->setTransform(transform);
    }
    else if (key == GLFW_KEY_S && action == GLFW_PRESS) {
        rp3d::Transform transform = mAllShapes[mSelectedShapeIndex]->getTransform();
        transform.setOrientation(rp3d::Quaternion(-stepAngle, 0, 0) * transform.getOrientation());
        mAllShapes[mSelectedShapeIndex]->setTransform(transform);
    }
    else if (key == GLFW_KEY_F && action == GLFW_PRESS) {
        rp3d::Transform transform = mAllShapes[mSelectedShapeIndex]->getTransform();
        transform.setOrientation(rp3d::Quaternion(0, 0, stepAngle) * transform.getOrientation());
        mAllShapes[mSelectedShapeIndex]->setTransform(transform);
    }
    else if (key == GLFW_KEY_G && action == GLFW_PRESS) {
        rp3d::Transform transform = mAllShapes[mSelectedShapeIndex]->getTransform();
        transform.setOrientation(rp3d::Quaternion(0, 0, -stepAngle) * transform.getOrientation());
        mAllShapes[mSelectedShapeIndex]->setTransform(transform);
    }

    return false;
}
