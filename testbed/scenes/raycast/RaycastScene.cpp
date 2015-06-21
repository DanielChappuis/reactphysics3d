/********************************************************************************
* ReactPhysics3D physics library, http://www.reactphysics3d.com                 *
* Copyright (c) 2010-2015 Daniel Chappuis                                       *
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
#include "RaycastScene.h"

// Namespaces
using namespace openglframework;
using namespace raycastscene;

// Constructor
RaycastScene::RaycastScene(const std::string& name)
       : Scene(name), mLight0(0), mPhongShader("shaders/phong.vert", "shaders/phong.frag"),
         mCurrentBodyIndex(-1), mAreNormalsDisplayed(false) {

    std::string meshFolderPath("meshes/");

    // Move the light 0
    mLight0.translateWorld(Vector3(50, 50, 50));

    // Compute the radius and the center of the scene
    float radiusScene = 30.0f;
    openglframework::Vector3 center(0, 0, 0);

    // Set the center of the scene
    setScenePosition(center, radiusScene);

    // Create the dynamics world for the physics simulation
    mCollisionWorld = new rp3d::CollisionWorld();

    // Create the static data for the visual contact points
    VisualContactPoint::createStaticData(meshFolderPath);

    // ---------- Dumbbell ---------- //
    openglframework::Vector3 position1(0, 0, 0);

    // Create a convex mesh and a corresponding collision body in the dynamics world
    mDumbbell = new Dumbbell(position1, mCollisionWorld, meshFolderPath, mPhongShader);

    // ---------- Box ---------- //
    openglframework::Vector3 position2(0, 0, 0);

    // Create a box and a corresponding collision body in the dynamics world
    mBox = new Box(BOX_SIZE, position2, mCollisionWorld, mPhongShader);
    mBox->getCollisionBody()->setIsActive(false);

    // ---------- Sphere ---------- //
    openglframework::Vector3 position3(0, 0, 0);

    // Create a sphere and a corresponding collision body in the dynamics world
    mSphere = new Sphere(SPHERE_RADIUS, position3, mCollisionWorld,
                         meshFolderPath, mPhongShader);

    // ---------- Cone ---------- //
    openglframework::Vector3 position4(0, 0, 0);

    // Create a cone and a corresponding collision body in the dynamics world
    mCone = new Cone(CONE_RADIUS, CONE_HEIGHT, position4, mCollisionWorld,
                     meshFolderPath, mPhongShader);

    // ---------- Cylinder ---------- //
    openglframework::Vector3 position5(0, 0, 0);

    // Create a cylinder and a corresponding collision body in the dynamics world
    mCylinder = new Cylinder(CYLINDER_RADIUS, CYLINDER_HEIGHT, position5,
                             mCollisionWorld, meshFolderPath, mPhongShader);

    // ---------- Capsule ---------- //
    openglframework::Vector3 position6(0, 0, 0);

    // Create a cylinder and a corresponding collision body in the dynamics world
    mCapsule = new Capsule(CAPSULE_RADIUS, CAPSULE_HEIGHT, position6 ,
                           mCollisionWorld, meshFolderPath, mPhongShader);

    // ---------- Convex Mesh ---------- //
    openglframework::Vector3 position7(0, 0, 0);

    // Create a convex mesh and a corresponding collision body in the dynamics world
    mConvexMesh = new ConvexMesh(position7, mCollisionWorld, meshFolderPath, mPhongShader);

    // Create the lines that will be used for raycasting
    createLines();

    changeBody();
}

// Create the raycast lines
void RaycastScene::createLines() {

      int nbRaysOneDimension = std::sqrt(float(NB_RAYS));

      for (int i=0; i<nbRaysOneDimension; i++) {
          for (int j=0; j<nbRaysOneDimension; j++) {

              float theta = i * 2.0f * M_PI / float(nbRaysOneDimension);
              float phi = j * M_PI / float(nbRaysOneDimension);

              // Generate a point on a sphere with spherical coordinates
              float x = RAY_LENGTH * std::sin(phi) * std::cos(theta);
              float y = RAY_LENGTH * std::sin(phi) * std::sin(theta);
              float z = RAY_LENGTH * std::cos(phi);

              // Create a line from the point on the sphere to the center of
              // the scene
              openglframework::Vector3 point1(x, y, z);
              openglframework::Vector3 point2(0.0f, 0.0f, 0.0f);
              Line* line = new Line(point1, point2);
              mLines.push_back(line);
          }
      }
}

// Change the body to raycast and to display
void RaycastScene::changeBody() {

    mCurrentBodyIndex++;
    if (mCurrentBodyIndex >= NB_BODIES) mCurrentBodyIndex = 0;

    mSphere->getCollisionBody()->setIsActive(false);
    mBox->getCollisionBody()->setIsActive(false);
    mCone->getCollisionBody()->setIsActive(false);
    mCylinder->getCollisionBody()->setIsActive(false);
    mCapsule->getCollisionBody()->setIsActive(false);
    mConvexMesh->getCollisionBody()->setIsActive(false);
    mDumbbell->getCollisionBody()->setIsActive(false);

    switch(mCurrentBodyIndex) {
        case 0: mSphere->getCollisionBody()->setIsActive(true);
                break;
        case 1: mBox->getCollisionBody()->setIsActive(true);
                break;
        case 2: mCone->getCollisionBody()->setIsActive(true);
                break;
        case 3: mCylinder->getCollisionBody()->setIsActive(true);
                break;
        case 4: mCapsule->getCollisionBody()->setIsActive(true);
                break;
        case 5: mConvexMesh->getCollisionBody()->setIsActive(true);
                break;
        case 6: mDumbbell->getCollisionBody()->setIsActive(true);
                break;

    }
}

// Reset the scene
void RaycastScene::reset() {

}

// Destructor
RaycastScene::~RaycastScene() {

    // Destroy the shader
    mPhongShader.destroy();

    // Destroy the box rigid body from the dynamics world
    mCollisionWorld->destroyCollisionBody(mBox->getCollisionBody());
    delete mBox;

    // Destroy the sphere
    mCollisionWorld->destroyCollisionBody(mSphere->getCollisionBody());
    delete mSphere;

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

    // Destroy the convex mesh
    delete mDumbbell;

    mRaycastManager.resetPoints();

    // Destroy the static data for the visual contact points
    VisualContactPoint::destroyStaticData();

    // Destroy the collision world
    delete mCollisionWorld;

    // Destroy the lines
    for (std::vector<Line*>::iterator it = mLines.begin(); it != mLines.end();
         ++it) {
        delete (*it);
    }
}

// Update the physics world (take a simulation step)
void RaycastScene::updatePhysics() {

}

// Take a step for the simulation
void RaycastScene::update() {

    mRaycastManager.resetPoints();

    // For each line of the scene
    for (std::vector<Line*>::iterator it = mLines.begin(); it != mLines.end();
         ++it) {

        Line* line = *it;

        // Create a ray corresponding to the line
        openglframework::Vector3 p1 = line->getPoint1();
        openglframework::Vector3 p2 = line->getPoint2();

        rp3d::Vector3 point1(p1.x, p1.y, p1.z);
        rp3d::Vector3 point2(p2.x, p2.y, p2.z);
        rp3d::Ray ray(point1, point2);

        // Perform a raycast query on the physics world by passing a raycast
        // callback class in argument.
        mCollisionWorld->raycast(ray, &mRaycastManager);
    }
}

// Render the scene
void RaycastScene::render() {

    glEnable(GL_DEPTH_TEST);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glEnable(GL_CULL_FACE);

    // Get the world-space to camera-space matrix
    const openglframework::Matrix4 worldToCameraMatrix = mCamera.getTransformMatrix().getInverse();

    // Bind the shader
    mPhongShader.bind();

    openglframework::Vector4 grey(0.7, 0.7, 0.7, 1);
    mPhongShader.setVector4Uniform("vertexColor", grey);

    // Set the variables of the shader
    mPhongShader.setMatrix4x4Uniform("projectionMatrix", mCamera.getProjectionMatrix());
    mPhongShader.setVector3Uniform("light0PosCameraSpace", worldToCameraMatrix * mLight0.getOrigin());
    mPhongShader.setVector3Uniform("lightAmbientColor", Vector3(0.3f, 0.3f, 0.3f));
    const Color& diffColLight0 = mLight0.getDiffuseColor();
    const Color& specColLight0 = mLight0.getSpecularColor();
    mPhongShader.setVector3Uniform("light0DiffuseColor", Vector3(diffColLight0.r, diffColLight0.g, diffColLight0.b));
    mPhongShader.setVector3Uniform("light0SpecularColor", Vector3(specColLight0.r, specColLight0.g, specColLight0.b));
    mPhongShader.setFloatUniform("shininess", 200.0f);

    if (mBox->getCollisionBody()->isActive()) mBox->render(mPhongShader, worldToCameraMatrix);
    if (mSphere->getCollisionBody()->isActive()) mSphere->render(mPhongShader, worldToCameraMatrix);
    if (mCone->getCollisionBody()->isActive()) mCone->render(mPhongShader, worldToCameraMatrix);
    if (mCylinder->getCollisionBody()->isActive()) mCylinder->render(mPhongShader, worldToCameraMatrix);
    if (mCapsule->getCollisionBody()->isActive()) mCapsule->render(mPhongShader, worldToCameraMatrix);
    if (mConvexMesh->getCollisionBody()->isActive()) mConvexMesh->render(mPhongShader, worldToCameraMatrix);
    if (mDumbbell->getCollisionBody()->isActive()) mDumbbell->render(mPhongShader, worldToCameraMatrix);

    mPhongShader.unbind();
    mPhongShader.bind();

    mPhongShader.setVector3Uniform("light0SpecularColor", Vector3(0, 0, 0));
    openglframework::Vector4 redColor(1, 0, 0, 1);
    mPhongShader.setVector4Uniform("vertexColor", redColor);

    // Render all the raycast hit points
    mRaycastManager.render(mPhongShader, worldToCameraMatrix, mAreNormalsDisplayed);

    mPhongShader.unbind();
    mPhongShader.bind();

    openglframework::Vector4 blueColor(0, 0.62, 0.92, 1);
    mPhongShader.setVector4Uniform("vertexColor", blueColor);

    // Render the lines
    for (std::vector<Line*>::iterator it = mLines.begin(); it != mLines.end();
         ++it) {
        (*it)->render(mPhongShader, worldToCameraMatrix);
    }

    // Unbind the shader
    mPhongShader.unbind();
}
