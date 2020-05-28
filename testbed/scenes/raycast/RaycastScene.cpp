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
#include "RaycastScene.h"

// Namespaces
using namespace openglframework;
using namespace raycastscene;

// Constructor
RaycastScene::RaycastScene(const std::string& name, EngineSettings& settings)
       : SceneDemo(name, settings, SCENE_RADIUS, false), mMeshFolderPath("meshes/"),
         mRaycastManager(mMeshFolderPath, mSnapshotsContactPoints), mCurrentBodyIndex(-1),
         mAreNormalsDisplayed(false), mVBOVertices(GL_ARRAY_BUFFER) {

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

    // ---------- Dumbbell ---------- //

    // Create a convex mesh and a corresponding collision body in the physics world
    mDumbbell = new Dumbbell(false, mPhysicsCommon, mPhysicsWorld, mMeshFolderPath);

    // Set the box color
    mDumbbell->setColor(mObjectColorDemo);
    mDumbbell->setSleepingColor(mSleepingColorDemo);
	mPhysicsObjects.push_back(mDumbbell);

    // ---------- Box ---------- //

    // Create a box and a corresponding collision body in the physics world
    mBox = new Box(false, BOX_SIZE, mPhysicsCommon, mPhysicsWorld, mMeshFolderPath);
    mBox->getCollisionBody()->setIsActive(false);

    // Set the box color
    mBox->setColor(mObjectColorDemo);
    mBox->setSleepingColor(mSleepingColorDemo);
	mPhysicsObjects.push_back(mBox);

    // ---------- Sphere ---------- //

    // Create a sphere and a corresponding collision body in the physics world
    mSphere = new Sphere(false, SPHERE_RADIUS, mPhysicsCommon, mPhysicsWorld, mMeshFolderPath);

    // Set the color
    mSphere->setColor(mObjectColorDemo);
    mSphere->setSleepingColor(mSleepingColorDemo);
	mPhysicsObjects.push_back(mSphere);

    // ---------- Capsule ---------- //
    openglframework::Vector3 position6(0, 0, 0);

    // Create a cylinder and a corresponding collision body in the physics world
    mCapsule = new Capsule(false, CAPSULE_RADIUS, CAPSULE_HEIGHT, mPhysicsCommon, mPhysicsWorld, mMeshFolderPath);

    // Set the color
    mCapsule->setColor(mObjectColorDemo);
    mCapsule->setSleepingColor(mSleepingColorDemo);
	mPhysicsObjects.push_back(mCapsule);

    // ---------- Convex Mesh ---------- //

    // Create a convex mesh and a corresponding collision body in the physics world
    mConvexMesh = new ConvexMesh(false,  mPhysicsCommon, mPhysicsWorld, mMeshFolderPath + "convexmesh.obj");

    // Set the color
    mConvexMesh->setColor(mObjectColorDemo);
    mConvexMesh->setSleepingColor(mSleepingColorDemo);
	mPhysicsObjects.push_back(mConvexMesh);

    // ---------- Concave Mesh ---------- //

    // Create a convex mesh and a corresponding collision body in the physics world
    mConcaveMesh = new ConcaveMesh(false, mPhysicsCommon, mPhysicsWorld, mMeshFolderPath + "city.obj");

    // Set the color
    mConcaveMesh->setColor(mObjectColorDemo);
    mConcaveMesh->setSleepingColor(mSleepingColorDemo);
	mPhysicsObjects.push_back(mConcaveMesh);

    // ---------- Heightfield ---------- //

    // Create a convex mesh and a corresponding collision body in the physics world
    mHeightField = new HeightField(false, mPhysicsCommon, mPhysicsWorld);

    // Set the color
    mHeightField->setColor(mObjectColorDemo);
    mHeightField->setSleepingColor(mObjectColorDemo);
	mPhysicsObjects.push_back(mHeightField);

    // Create the lines that will be used for raycasting
    createLines();

    // Create the VBO and VAO to render the lines
    createVBOAndVAO();

    changeBody();
}

// Create the raycast lines
void RaycastScene::createLines() {

      int nbRaysOneDimension = static_cast<int>(std::sqrt(float(NB_RAYS)));

      for (int i=0; i<nbRaysOneDimension; i++) {
          for (int j=0; j<nbRaysOneDimension; j++) {

              float theta = i * 2.0f * PI / float(nbRaysOneDimension);
              float phi = j * PI / float(nbRaysOneDimension);

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

              mLinePoints.push_back(point1);
              mLinePoints.push_back(point2);
          }
      }
}

// Change the body to raycast and to display
void RaycastScene::changeBody() {

    mCurrentBodyIndex++;
    if (mCurrentBodyIndex >= NB_BODIES) mCurrentBodyIndex = 0;

    mSphere->getCollisionBody()->setIsActive(false);
    mBox->getCollisionBody()->setIsActive(false);
    mCapsule->getCollisionBody()->setIsActive(false);
    mConvexMesh->getCollisionBody()->setIsActive(false);
    mDumbbell->getCollisionBody()->setIsActive(false);
    mConcaveMesh->getCollisionBody()->setIsActive(false);
    mHeightField->getCollisionBody()->setIsActive(false);

    switch(mCurrentBodyIndex) {
        case 0: mSphere->getCollisionBody()->setIsActive(true);
                break;
        case 1: mBox->getCollisionBody()->setIsActive(true);
                break;
        case 2: mCapsule->getCollisionBody()->setIsActive(true);
                break;
        case 3: mConvexMesh->getCollisionBody()->setIsActive(true);
                break;
        case 4: mDumbbell->getCollisionBody()->setIsActive(true);
                break;
        case 5: mConcaveMesh->getCollisionBody()->setIsActive(true);
                break;
        case 6: mHeightField->getCollisionBody()->setIsActive(true);
                break;

    }
}

// Reset the scene
void RaycastScene::reset() {

    SceneDemo::reset();

    std::vector<PhysicsObject*>::iterator it;
    for (it = mPhysicsObjects.begin(); it != mPhysicsObjects.end(); ++it) {
        (*it)->setTransform(rp3d::Transform(rp3d::Vector3::zero(), rp3d::Quaternion::identity()));
    }
}

// Destructor
RaycastScene::~RaycastScene() {

    // Destroy the box rigid body from the physics world
    mPhysicsWorld->destroyCollisionBody(mBox->getCollisionBody());
    delete mBox;

    // Destroy the sphere
    mPhysicsWorld->destroyCollisionBody(mSphere->getCollisionBody());
    delete mSphere;

    // Destroy the corresponding rigid body from the physics world
    mPhysicsWorld->destroyCollisionBody(mCapsule->getCollisionBody());

    // Destroy the sphere
    delete mCapsule;

    // Destroy the corresponding rigid body from the physics world
    mPhysicsWorld->destroyCollisionBody(mConvexMesh->getCollisionBody());

    // Destroy the convex mesh
    delete mConvexMesh;

    // Destroy the corresponding rigid body from the physics world
    mPhysicsWorld->destroyCollisionBody(mDumbbell->getCollisionBody());

    // Destroy the dumbbell
    delete mDumbbell;

    // Destroy the corresponding rigid body from the physics world
    mPhysicsWorld->destroyCollisionBody(mConcaveMesh->getCollisionBody());

    // Destroy the convex mesh
    delete mConcaveMesh;

    // Destroy the corresponding rigid body from the physics world
    mPhysicsWorld->destroyCollisionBody(mHeightField->getCollisionBody());

    // Destroy the convex mesh
    delete mHeightField;

    mRaycastManager.resetPoints();

    // Destroy the static data for the visual contact points
    VisualContactPoint::destroyStaticData();

    // Destroy the collision world
    mPhysicsCommon.destroyPhysicsWorld(mPhysicsWorld);

    // Destroy the lines
    for (std::vector<Line*>::iterator it = mLines.begin(); it != mLines.end();
         ++it) {
        delete (*it);
    }

    // Destroy the VBOs and VAO
    mVBOVertices.destroy();
    mVAO.destroy();
}

// Take a step for the simulation
void RaycastScene::update() {

    // Compute debug rendering primitives
    mPhysicsWorld->getDebugRenderer().reset();
    mPhysicsWorld->getDebugRenderer().computeDebugRenderingPrimitives(*mPhysicsWorld);

    mRaycastManager.resetPoints();

    // For each line of the scene
    for (std::vector<Line*>::iterator it = mLines.begin(); it != mLines.end(); ++it) {

        Line* line = *it;

        // Create a ray corresponding to the line
        openglframework::Vector3 p1 = line->getPoint1();
        openglframework::Vector3 p2 = line->getPoint2();

        rp3d::Vector3 point1(p1.x, p1.y, p1.z);
        rp3d::Vector3 point2(p2.x, p2.y, p2.z);
        rp3d::Ray ray(point1, point2);

        // Perform a raycast query on the physics world by passing a raycast
        // callback class in argument.
        mPhysicsWorld->raycast(ray, &mRaycastManager);
    }

    SceneDemo::update();
}

// Render the scene
void RaycastScene::renderSinglePass(openglframework::Shader& shader, const openglframework::Matrix4& worldToCameraMatrix) {

    if (mIsWireframeEnabled) {
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    }

    // Bind the VAO
    mVAO.bind();

    // Bind the shader
	mColorShader.bind();

    mVBOVertices.bind();

    // Set the model to camera matrix
    const Matrix4 localToCameraMatrix = Matrix4::identity();
	mColorShader.setMatrix4x4Uniform("localToWorldMatrix", localToCameraMatrix);
	mColorShader.setMatrix4x4Uniform("worldToCameraMatrix", worldToCameraMatrix);

    // Set the vertex color
    openglframework::Vector4 color(1, 0.55f, 0, 1);
    mColorShader.setIntUniform("isGlobalVertexColorEnabled", 1, false);
    mColorShader.setVector4Uniform("globalVertexColor", color, false);

    // Get the location of shader attribute variables
    GLint vertexPositionLoc = mColorShader.getAttribLocation("vertexPosition");

    glEnableVertexAttribArray(vertexPositionLoc);
    glVertexAttribPointer(vertexPositionLoc, 3, GL_FLOAT, GL_FALSE, 0, (char*)NULL);

    // Draw the lines
    glDrawArrays(GL_LINES, 0, mLinePoints.size() * 2);

    glDisableVertexAttribArray(vertexPositionLoc);

    mVBOVertices.unbind();

    // Unbind the VAO
    mVAO.unbind();

    mColorShader.unbind();

	// Bind the shader
	shader.bind();

	// Render all the physics objects of the scene
	for (std::vector<PhysicsObject*>::iterator it = mPhysicsObjects.begin(); it != mPhysicsObjects.end(); ++it) {
		if ((*it)->getCollisionBody()->isActive()) {
            (*it)->render(shader, worldToCameraMatrix);
		}
	}

	// Unbind the shader
	shader.unbind();

    if (mIsWireframeEnabled) {
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    }
}

// Create the Vertex Buffer Objects used to render with OpenGL.
/// We create two VBOs (one for vertices and one for indices)
void RaycastScene::createVBOAndVAO() {

    // Create the VBO for the vertices data
    mVBOVertices.create();
    mVBOVertices.bind();
    size_t sizeVertices = mLinePoints.size() * sizeof(openglframework::Vector3);
    mVBOVertices.copyDataIntoVBO(sizeVertices, &mLinePoints[0], GL_STATIC_DRAW);
    mVBOVertices.unbind();

    // Create the VAO for both VBOs
    mVAO.create();
    mVAO.bind();

    // Bind the VBO of vertices
    mVBOVertices.bind();

    // Unbind the VAO
    mVAO.unbind();
}

// Called when a keyboard event occurs
bool RaycastScene::keyboardEvent(int key, int scancode, int action, int mods) {

    // If the space key has been pressed
    if (key == GLFW_KEY_SPACE && action == GLFW_PRESS) {
        changeBody();
        return true;
    }

    return false;
}
