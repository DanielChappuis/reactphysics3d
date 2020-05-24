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
#include <reactphysics3d/constraint/ContactPoint.h>
#include <reactphysics3d/collision/ContactManifold.h>

// Namespaces
using namespace openglframework;
using namespace collisiondetectionscene;

// Constructor
CollisionDetectionScene::CollisionDetectionScene(const std::string& name, EngineSettings& settings)
       : SceneDemo(name, settings, SCENE_RADIUS, false), mMeshFolderPath("meshes/"),
         mContactManager(mPhongShader, mMeshFolderPath, mSnapshotsContactPoints),
         mAreNormalsDisplayed(false) {

    mSelectedShapeIndex = 0;
    mAreContactPointsDisplayed = true;
    mAreContactNormalsDisplayed = false;
    mIsWireframeEnabled = true;

    // Compute the radius and the center of the scene
    openglframework::Vector3 center(0, 0, 0);

    // Set the center of the scene
    setScenePosition(center, SCENE_RADIUS);

    rp3d::PhysicsWorld::WorldSettings worldSettings;
    worldSettings.worldName = name;

    // Logger
    rp3d::DefaultLogger* defaultLogger = mPhysicsCommon.createDefaultLogger();
    uint logLevel = static_cast<uint>(rp3d::Logger::Level::Information) | static_cast<uint>(rp3d::Logger::Level::Warning) |
            static_cast<uint>(rp3d::Logger::Level::Error);
    defaultLogger->addFileDestination("rp3d_log_" + name + ".html", logLevel, rp3d::DefaultLogger::Format::HTML);
    mPhysicsCommon.setLogger(defaultLogger);

    // Create the physics world for the physics simulation
    mPhysicsWorld = mPhysicsCommon.createPhysicsWorld(worldSettings);

    // ---------- Sphere 1 ---------- //

    // Create a sphere and a corresponding collision body in the physics world
    mSphere1 = new Sphere(false, 4, mPhysicsCommon, mPhysicsWorld, mMeshFolderPath);
    mAllShapes.push_back(mSphere1);

    // Set the color
    mSphere1->setColor(mObjectColorDemo);
    mSphere1->setSleepingColor(mSleepingColorDemo);
    //mSphere1->setScaling(0.5f);
    mPhysicsObjects.push_back(mSphere1);

    // ---------- Sphere 2 ---------- //

    // Create a sphere and a corresponding collision body in the physics world
    mSphere2 = new Sphere(false, 2, mPhysicsCommon, mPhysicsWorld, mMeshFolderPath);
    mAllShapes.push_back(mSphere2);

    // Set the color
    mSphere2->setColor(mObjectColorDemo);
    mSphere2->setSleepingColor(mSleepingColorDemo);
    mPhysicsObjects.push_back(mSphere2);


    // ---------- Capsule 1 ---------- //

    // Create a cylinder and a corresponding collision body in the physics world
    mCapsule1 = new Capsule(false, CAPSULE_RADIUS, CAPSULE_HEIGHT, mPhysicsCommon, mPhysicsWorld, mMeshFolderPath);
    mAllShapes.push_back(mCapsule1);

    // Set the color
    mCapsule1->setColor(mObjectColorDemo);
    mCapsule1->setSleepingColor(mSleepingColorDemo);
    mPhysicsObjects.push_back(mCapsule1);

    // ---------- Capsule 2 ---------- //

    // Create a cylinder and a corresponding collision body in the physics world
    mCapsule2 = new Capsule(false, CAPSULE_RADIUS, CAPSULE_HEIGHT, mPhysicsCommon, mPhysicsWorld, mMeshFolderPath);
    mAllShapes.push_back(mCapsule2);

    // Set the color
    mCapsule2->setColor(mObjectColorDemo);
    mCapsule2->setSleepingColor(mSleepingColorDemo);
    mPhysicsObjects.push_back(mCapsule2);

    // ---------- Concave Mesh ---------- //

    // Create a convex mesh and a corresponding collision body in the physics world
    mConcaveMesh = new ConcaveMesh(false, mPhysicsCommon, mPhysicsWorld, mMeshFolderPath + "city.obj");
    mAllShapes.push_back(mConcaveMesh);

    // Set the color
    mConcaveMesh->setColor(mObjectColorDemo);
    mConcaveMesh->setSleepingColor(mSleepingColorDemo);
    mPhysicsObjects.push_back(mConcaveMesh);

    // ---------- Box 1 ---------- //

    // Create a cylinder and a corresponding collision body in the physics world
    mBox1 = new Box(false, BOX_SIZE, mPhysicsCommon, mPhysicsWorld, mMeshFolderPath);
    mAllShapes.push_back(mBox1);

    // Set the color
    mBox1->setColor(mObjectColorDemo);
    mBox1->setSleepingColor(mSleepingColorDemo);
    mPhysicsObjects.push_back(mBox1);

    // ---------- Box 2 ---------- //

    // Create a cylinder and a corresponding collision body in the physics world
    mBox2 = new Box(false, openglframework::Vector3(3, 2, 5), mPhysicsCommon, mPhysicsWorld, mMeshFolderPath);
    mAllShapes.push_back(mBox2);

    // Set the color
    mBox2->setColor(mObjectColorDemo);
    mBox2->setSleepingColor(mSleepingColorDemo);
    mPhysicsObjects.push_back(mBox2);

    // ---------- Convex Mesh ---------- //

    // Create a convex mesh and a corresponding collision body in the physics world
    mConvexMesh = new ConvexMesh(false, mPhysicsCommon, mPhysicsWorld, mMeshFolderPath + "convexmesh.obj");
    mAllShapes.push_back(mConvexMesh);

    // Set the color
    mConvexMesh->setColor(mObjectColorDemo);
    mConvexMesh->setSleepingColor(mSleepingColorDemo);
    mPhysicsObjects.push_back(mConvexMesh);

    // ---------- Heightfield ---------- //

    // Create a convex mesh and a corresponding collision body in the physics world
    mHeightField = new HeightField(false, mPhysicsCommon, mPhysicsWorld);

    // Set the color
    mHeightField->setColor(mObjectColorDemo);
    mHeightField->setSleepingColor(mSleepingColorDemo);
	mPhysicsObjects.push_back(mHeightField);

    mAllShapes[mSelectedShapeIndex]->setColor(mObjectColorDemo);
}

// Reset the scene
void CollisionDetectionScene::reset() {

    SceneDemo::reset();

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

    // Destroy the box rigid body from the physics world
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

    // Destroy the static data for the visual contact points
    VisualContactPoint::destroyStaticData();

    // Destroy the physics world
    mPhysicsCommon.destroyPhysicsWorld(mPhysicsWorld);
}

// Take a step for the simulation
void CollisionDetectionScene::update() {

    // Compute debug rendering primitives
    mPhysicsWorld->getDebugRenderer().reset();
    mPhysicsWorld->getDebugRenderer().computeDebugRenderingPrimitives(*mPhysicsWorld);
    mSnapshotsContactPoints.clear();

    mPhysicsWorld->testCollision(mContactManager);

    SceneDemo::update();
}

void CollisionDetectionScene::selectNextShape() {

    uint previousIndex = mSelectedShapeIndex;
    mSelectedShapeIndex++;
    if (mSelectedShapeIndex >= mAllShapes.size()) {
        mSelectedShapeIndex = 0;
    }

    mAllShapes[previousIndex]->setColor(mObjectColorDemo);
    mAllShapes[mSelectedShapeIndex]->setColor(mSelectedObjectColorDemo);
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

// This method is called when some contacts occur
void ContactManager::onContact(const CallbackData& callbackData) {

    // For each contact pair
    for (uint p=0; p < callbackData.getNbContactPairs(); p++) {

        ContactPair contactPair = callbackData.getContactPair(p);

        // For each contact point of the contact pair
        for (uint c=0; c < contactPair.getNbContactPoints(); c++) {

            ContactPoint contactPoint = contactPair.getContactPoint(c);

            // Contact normal
            rp3d::Vector3 normal = contactPoint.getWorldNormal();
            openglframework::Vector3 contactNormal(normal.x, normal.y, normal.z);

            rp3d::Vector3 point1 = contactPoint.getLocalPointOnCollider1();
            point1 = contactPair.getCollider1()->getLocalToWorldTransform() * point1;

            openglframework::Vector3 position1(point1.x, point1.y, point1.z);
            mContactPoints.push_back(SceneContactPoint(position1, contactNormal, openglframework::Color::red()));

            rp3d::Vector3 point2 = contactPoint.getLocalPointOnCollider2();
            point2 = contactPair.getCollider2()->getLocalToWorldTransform() * point2;
            openglframework::Vector3 position2(point2.x, point2.y, point2.z);
            mContactPoints.push_back(SceneContactPoint(position2, contactNormal, openglframework::Color::blue()));
        }
    }
}
