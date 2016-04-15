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
RaycastScene::RaycastScene(const std::string& name)
       : SceneDemo(name, SCENE_RADIUS, false), mMeshFolderPath("meshes/"),
         mRaycastManager(mPhongShader, mMeshFolderPath), mCurrentBodyIndex(-1),
         mAreNormalsDisplayed(false), mVBOVertices(GL_ARRAY_BUFFER) {

    mIsContactPointsDisplayed = true;

    // Compute the radius and the center of the scene
    openglframework::Vector3 center(0, 0, 0);

    // Set the center of the scene
    setScenePosition(center, SCENE_RADIUS);

    // Create the dynamics world for the physics simulation
    mCollisionWorld = new rp3d::CollisionWorld();

    // ---------- Dumbbell ---------- //
    openglframework::Vector3 position1(0, 0, 0);

    // Create a convex mesh and a corresponding collision body in the dynamics world
    mDumbbell = new Dumbbell(position1, mCollisionWorld, mMeshFolderPath);

    // Set the box color
    mDumbbell->setColor(mGreyColorDemo);
    mDumbbell->setSleepingColor(mRedColorDemo);

    // ---------- Box ---------- //
    openglframework::Vector3 position2(0, 0, 0);

    // Create a box and a corresponding collision body in the dynamics world
    mBox = new Box(BOX_SIZE, position2, mCollisionWorld);
    mBox->getCollisionBody()->setIsActive(false);

    // Set the box color
    mBox->setColor(mGreyColorDemo);
    mBox->setSleepingColor(mRedColorDemo);

    // ---------- Sphere ---------- //
    openglframework::Vector3 position3(0, 0, 0);

    // Create a sphere and a corresponding collision body in the dynamics world
    mSphere = new Sphere(SPHERE_RADIUS, position3, mCollisionWorld,
                         mMeshFolderPath);

    // Set the color
    mSphere->setColor(mGreyColorDemo);
    mSphere->setSleepingColor(mRedColorDemo);

    // ---------- Cone ---------- //
    openglframework::Vector3 position4(0, 0, 0);

    // Create a cone and a corresponding collision body in the dynamics world
    mCone = new Cone(CONE_RADIUS, CONE_HEIGHT, position4, mCollisionWorld,
                     mMeshFolderPath);

    // Set the color
    mCone->setColor(mGreyColorDemo);
    mCone->setSleepingColor(mRedColorDemo);

    // ---------- Cylinder ---------- //
    openglframework::Vector3 position5(0, 0, 0);

    // Create a cylinder and a corresponding collision body in the dynamics world
    mCylinder = new Cylinder(CYLINDER_RADIUS, CYLINDER_HEIGHT, position5,
                             mCollisionWorld, mMeshFolderPath);

    // Set the color
    mCylinder->setColor(mGreyColorDemo);
    mCylinder->setSleepingColor(mRedColorDemo);

    // ---------- Capsule ---------- //
    openglframework::Vector3 position6(0, 0, 0);

    // Create a cylinder and a corresponding collision body in the dynamics world
    mCapsule = new Capsule(CAPSULE_RADIUS, CAPSULE_HEIGHT, position6 ,
                           mCollisionWorld, mMeshFolderPath);

    // Set the color
    mCapsule->setColor(mGreyColorDemo);
    mCapsule->setSleepingColor(mRedColorDemo);

    // ---------- Convex Mesh ---------- //
    openglframework::Vector3 position7(0, 0, 0);

    // Create a convex mesh and a corresponding collision body in the dynamics world
    mConvexMesh = new ConvexMesh(position7, mCollisionWorld, mMeshFolderPath + "convexmesh.obj");

    // Set the color
    mConvexMesh->setColor(mGreyColorDemo);
    mConvexMesh->setSleepingColor(mRedColorDemo);

    // ---------- Concave Mesh ---------- //
    openglframework::Vector3 position8(0, 0, 0);

    // Create a convex mesh and a corresponding collision body in the dynamics world
    mConcaveMesh = new ConcaveMesh(position8, mCollisionWorld, mMeshFolderPath + "city.obj");

    // Set the color
    mConcaveMesh->setColor(mGreyColorDemo);
    mConcaveMesh->setSleepingColor(mRedColorDemo);

    // ---------- Heightfield ---------- //
    openglframework::Vector3 position9(0, 0, 0);

    // Create a convex mesh and a corresponding collision body in the dynamics world
    mHeightField = new HeightField(position9, mCollisionWorld);

    // Set the color
    mHeightField->setColor(mGreyColorDemo);
    mHeightField->setSleepingColor(mRedColorDemo);

    // Create the lines that will be used for raycasting
    createLines();

    // Create the VBO and VAO to render the lines
    createVBOAndVAO(mPhongShader);

    changeBody();
}

// Create the raycast lines
void RaycastScene::createLines() {

      int nbRaysOneDimension = std::sqrt(float(NB_RAYS));

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
    mCone->getCollisionBody()->setIsActive(false);
    mCylinder->getCollisionBody()->setIsActive(false);
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
        case 7: mConcaveMesh->getCollisionBody()->setIsActive(true);
                break;
        case 8: mHeightField->getCollisionBody()->setIsActive(true);
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

    // Destroy the VBOs and VAO
    mVBOVertices.destroy();
    mVAO.destroy();
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

    SceneDemo::update();
}

// Render the scene
void RaycastScene::renderSinglePass(openglframework::Shader& shader,
                                    const openglframework::Matrix4& worldToCameraMatrix) {

    // Bind the VAO
    mVAO.bind();

    // Bind the shader
    shader.bind();

    mVBOVertices.bind();

    // Set the model to camera matrix
    const Matrix4 localToCameraMatrix = Matrix4::identity();
    shader.setMatrix4x4Uniform("localToWorldMatrix", localToCameraMatrix);
    shader.setMatrix4x4Uniform("worldToCameraMatrix", worldToCameraMatrix);

    // Set the normal matrix (inverse transpose of the 3x3 upper-left sub matrix of the
    // model-view matrix)
    const openglframework::Matrix3 normalMatrix =
                       localToCameraMatrix.getUpperLeft3x3Matrix().getInverse().getTranspose();
    shader.setMatrix3x3Uniform("normalMatrix", normalMatrix, false);

    // Set the vertex color
    openglframework::Vector4 color(1, 0, 0, 1);
    shader.setVector4Uniform("vertexColor", color, false);

    // Get the location of shader attribute variables
    GLint vertexPositionLoc = shader.getAttribLocation("vertexPosition");

    glEnableVertexAttribArray(vertexPositionLoc);
    glVertexAttribPointer(vertexPositionLoc, 3, GL_FLOAT, GL_FALSE, 0, (char*)NULL);

    // Draw the lines
    glDrawArrays(GL_LINES, 0, NB_RAYS);

    glDisableVertexAttribArray(vertexPositionLoc);

    mVBOVertices.unbind();

    // Unbind the VAO
    mVAO.unbind();

    shader.unbind();

    // Render the shapes
    if (mBox->getCollisionBody()->isActive()) mBox->render(shader, worldToCameraMatrix);
    if (mSphere->getCollisionBody()->isActive()) mSphere->render(shader, worldToCameraMatrix);
    if (mCone->getCollisionBody()->isActive()) mCone->render(shader, worldToCameraMatrix);
    if (mCylinder->getCollisionBody()->isActive()) mCylinder->render(shader, worldToCameraMatrix);
    if (mCapsule->getCollisionBody()->isActive()) mCapsule->render(shader, worldToCameraMatrix);
    if (mConvexMesh->getCollisionBody()->isActive()) mConvexMesh->render(shader, worldToCameraMatrix);
    if (mDumbbell->getCollisionBody()->isActive()) mDumbbell->render(shader, worldToCameraMatrix);
    if (mConcaveMesh->getCollisionBody()->isActive()) mConcaveMesh->render(shader, worldToCameraMatrix);
    if (mHeightField->getCollisionBody()->isActive()) mHeightField->render(shader, worldToCameraMatrix);

    shader.unbind();
}

// Create the Vertex Buffer Objects used to render with OpenGL.
/// We create two VBOs (one for vertices and one for indices)
void RaycastScene::createVBOAndVAO(openglframework::Shader& shader) {

    // Bind the shader
    shader.bind();

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

    // Unbind the shader
    shader.unbind();
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
