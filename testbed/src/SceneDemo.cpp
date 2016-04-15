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
#include "SceneDemo.h"
#include <GLFW/glfw3.h>

using namespace openglframework;

int SceneDemo::shadowMapTextureLevel = 0;
openglframework::Color SceneDemo::mGreyColorDemo = Color(0.70f, 0.70f, 0.7f, 1.0);
openglframework::Color SceneDemo::mYellowColorDemo = Color(0.9, 0.88, 0.145, 1.0);
openglframework::Color SceneDemo::mBlueColorDemo = Color(0, 0.66, 0.95, 1.0);
openglframework::Color SceneDemo::mOrangeColorDemo = Color(0.9, 0.35, 0, 1.0);
openglframework::Color SceneDemo::mPinkColorDemo = Color(0.83, 0.48, 0.64, 1.0);
openglframework::Color SceneDemo::mRedColorDemo = Color(0.95, 0, 0, 1.0);
int SceneDemo::mNbDemoColors = 4;
openglframework::Color SceneDemo::mDemoColors[] = {SceneDemo::mYellowColorDemo, SceneDemo::mBlueColorDemo,
                                                   SceneDemo::mOrangeColorDemo, SceneDemo::mPinkColorDemo};

// Constructor
SceneDemo::SceneDemo(const std::string& name, float sceneRadius, bool isShadowMappingEnabled)
          : Scene(name, isShadowMappingEnabled), mIsShadowMappingInitialized(false),
                     mDepthShader("shaders/depth.vert", "shaders/depth.frag"),
                     mPhongShader("shaders/phong.vert", "shaders/phong.frag"),
                     mQuadShader("shaders/quad.vert", "shaders/quad.frag"),
                     mVBOQuad(GL_ARRAY_BUFFER), mMeshFolderPath("meshes/") {

    shadowMapTextureLevel++;

    // Move the light0
    mLight0.translateWorld(Vector3(-2, 35, 40));

    // Camera at light0 postion for the shadow map
    mShadowMapLightCamera.translateWorld(mLight0.getOrigin());
    mShadowMapLightCamera.rotateLocal(Vector3(1, 0, 0), -PI / 4.0f);
    mShadowMapLightCamera.rotateWorld(Vector3(0, 1, 0), PI / 8.0f);

    mShadowMapLightCamera.setDimensions(SHADOWMAP_WIDTH, SHADOWMAP_HEIGHT);
    mShadowMapLightCamera.setFieldOfView(80.0f);
    mShadowMapLightCamera.setSceneRadius(100);

    mShadowMapBiasMatrix.setAllValues(0.5, 0.0, 0.0, 0.5,
                                      0.0, 0.5, 0.0, 0.5,
                                      0.0, 0.0, 0.5, 0.5,
                                      0.0, 0.0, 0.0, 1.0);

    // Create the Shadow map FBO and texture
    if (mIsShadowMappingEnabled) {
        createShadowMapFBOAndTexture();
    }

    createQuadVBO();

    VisualContactPoint::createStaticData(mMeshFolderPath);
}

// Destructor
SceneDemo::~SceneDemo() {

    mShadowMapTexture.destroy();
    mFBOShadowMap.destroy();
    mVBOQuad.destroy();

    // Destroy the contact points
    removeAllContactPoints();

    VisualContactPoint::destroyStaticData();
}

// Update the scene
void SceneDemo::update() {

    // Update the contact points
    updateContactPoints();
}

// Render the scene (in multiple passes for shadow mapping)
void SceneDemo::render() {

    const Color& diffCol = mLight0.getDiffuseColor();

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);

    // ---------- Render the scene to generate the shadow map (first pass) ----------- //

    const Matrix4 shadowMapProjMatrix = mShadowMapLightCamera.getProjectionMatrix();
    const openglframework::Matrix4 worldToLightCameraMatrix = mShadowMapLightCamera.getTransformMatrix().getInverse();

    // If Shadow Mapping is enabled
    if (mIsShadowMappingEnabled) {

        // Culling switching, rendering only backface, this is done to avoid self-shadowing
        glCullFace(GL_BACK);

        mFBOShadowMap.bind();

        // Bind the shader
        mDepthShader.bind();

        // Set the variables of the shader
        mDepthShader.setMatrix4x4Uniform("projectionMatrix", shadowMapProjMatrix);

        // Set the viewport to render into the shadow map texture
        glViewport(0, 0, SHADOWMAP_WIDTH, SHADOWMAP_HEIGHT);

        // Clear previous frame values
        glClear(GL_DEPTH_BUFFER_BIT);

        // Disable color rendering, we only want to write to the Z-Buffer
        glColorMask(GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE);

        // Render the objects of the scene
        renderSinglePass(mDepthShader, worldToLightCameraMatrix);

        // Unbind the shader
        mDepthShader.unbind();

        mFBOShadowMap.unbind();

        glDisable(GL_POLYGON_OFFSET_FILL);
    }

    // ---------- Render the scene for final rendering (second pass) ----------- //

    glCullFace(GL_BACK);

    // Get the world-space to camera-space matrix
    const openglframework::Matrix4 worldToCameraMatrix = mCamera.getTransformMatrix().getInverse();

    mPhongShader.bind();

    if (mIsShadowMappingEnabled) mShadowMapTexture.bind();
    const GLuint textureUnit = 0;

    // Set the variables of the shader
    mPhongShader.setMatrix4x4Uniform("projectionMatrix", mCamera.getProjectionMatrix());
    mPhongShader.setMatrix4x4Uniform("shadowMapProjectionMatrix", mShadowMapBiasMatrix * shadowMapProjMatrix);
    mPhongShader.setMatrix4x4Uniform("worldToLight0CameraMatrix", worldToLightCameraMatrix);
    mPhongShader.setVector3Uniform("light0PosCameraSpace", worldToCameraMatrix * mLight0.getOrigin());
    mPhongShader.setVector3Uniform("lightAmbientColor", Vector3(0.4f, 0.4f, 0.4f));
    mPhongShader.setVector3Uniform("light0DiffuseColor", Vector3(diffCol.r, diffCol.g, diffCol.b));
    mPhongShader.setIntUniform("shadowMapSampler", textureUnit);
    mPhongShader.setIntUniform("isShadowEnabled", mIsShadowMappingEnabled);
    mPhongShader.setVector2Uniform("shadowMapDimension", Vector2(SHADOWMAP_WIDTH, SHADOWMAP_HEIGHT));

    // Set the viewport to render the scene
    glViewport(mViewportX, mViewportY, mViewportWidth, mViewportHeight);

    //Enabling color write (previously disabled for light POV z-buffer rendering)
    glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);

    // Clear previous frame values
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // Render the objects of the scene
    renderSinglePass(mPhongShader, worldToCameraMatrix);

    // Render the contact points
    if (mIsContactPointsDisplayed) {
        renderContactPoints(mPhongShader, worldToCameraMatrix);
    }

    if (mIsShadowMappingEnabled) mShadowMapTexture.unbind();
    mPhongShader.unbind();

   //drawTextureQuad();
}

// Create the Shadow map FBO and texture
void SceneDemo::createShadowMapFBOAndTexture() {

    // Create the texture for the depth values
    mShadowMapTexture.create(SHADOWMAP_WIDTH, SHADOWMAP_HEIGHT, GL_DEPTH_COMPONENT24, GL_DEPTH_COMPONENT,
                             GL_UNSIGNED_BYTE, GL_NEAREST, GL_NEAREST, GL_CLAMP_TO_EDGE, GL_CLAMP_TO_EDGE, NULL);

    // Create the FBO for the shadow map
    mFBOShadowMap.create(0, 0, false);
    mFBOShadowMap.bind();

    // Tell OpenGL that we won't bind a color texture with the currently binded FBO
    glDrawBuffer(GL_NONE);
    glReadBuffer(GL_NONE);

    mFBOShadowMap.attachTexture(GL_DEPTH_ATTACHMENT, mShadowMapTexture.getID());
    mFBOShadowMap.unbind();

    mIsShadowMappingInitialized = true;
}

// Used for debugging shadow maps
void SceneDemo::createQuadVBO() {

    mVAOQuad.create();
    mVAOQuad.bind();

    static const GLfloat quadVertexData[] = {
        -1.0f, -1.0f, 0.0f,
        1.0f, -1.0f, 0.0f,
        -1.0f,  1.0f, 0.0f,
        -1.0f,  1.0f, 0.0f,
        1.0f, -1.0f, 0.0f,
        1.0f,  1.0f, 0.0f,
    };

    mVBOQuad.create();
    mVBOQuad.bind();
    mVBOQuad.copyDataIntoVBO(sizeof(quadVertexData), quadVertexData, GL_STATIC_DRAW);
    mVBOQuad.unbind();

    mVAOQuad.unbind();
}

void SceneDemo::drawTextureQuad() {

    glViewport(mViewportX, mViewportY, mViewportWidth, mViewportHeight);
    glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);

    // Clear previous frame values
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    const GLuint textureUnit = 0;

    mVAOQuad.bind();
    mQuadShader.bind();
    mShadowMapTexture.bind();
    mQuadShader.setIntUniform("textureSampler", textureUnit);
    mVBOQuad.bind();

    GLint vertexPositionLoc = mQuadShader.getAttribLocation("vertexPosition");
    glEnableVertexAttribArray(vertexPositionLoc);

    glVertexAttribPointer(
            vertexPositionLoc,                  // attribute 0. No particular reason for 0, but must match the layout in the shader.
            3,                  // size
            GL_FLOAT,           // type
            GL_FALSE,           // normalized?
            0,                  // stride
            (void*)0            // array buffer offset
    );

    // Draw the triangles !
    glDrawArrays(GL_TRIANGLES, 0, 6); // 2*3 indices starting at 0 -> 2 triangles

    glDisableVertexAttribArray(vertexPositionLoc);

    mVBOQuad.unbind();
    mShadowMapTexture.unbind();
    mQuadShader.unbind();
    mVAOQuad.unbind();
}

// Gather and create contact points
void SceneDemo::updateContactPoints() {

    // Remove the previous contact points
    removeAllContactPoints();

    if (mIsContactPointsDisplayed) {

        // Get the current contact points of the scene
        std::vector<ContactPoint> contactPoints = getContactPoints();

        // For each contact point
        std::vector<ContactPoint>::const_iterator it;
        for (it = contactPoints.begin(); it != contactPoints.end(); ++it) {

            // Create a visual contact point for rendering
            VisualContactPoint* point = new VisualContactPoint(it->point, mMeshFolderPath);
            mContactPoints.push_back(point);
        }
    }
}

// Render the contact points
void SceneDemo::renderContactPoints(openglframework::Shader& shader,
                                    const openglframework::Matrix4& worldToCameraMatrix) {

    // Render all the raycast hit points
    for (std::vector<VisualContactPoint*>::iterator it = mContactPoints.begin();
         it != mContactPoints.end(); ++it) {
        (*it)->render(shader, worldToCameraMatrix);
    }
}

void SceneDemo::removeAllContactPoints() {

    // Destroy all the visual contact points
    for (std::vector<VisualContactPoint*>::iterator it = mContactPoints.begin();
         it != mContactPoints.end(); ++it) {
        delete (*it);
    }
    mContactPoints.clear();
}

// Return all the contact points of the scene
std::vector<ContactPoint> SceneDemo::computeContactPointsOfWorld(const rp3d::DynamicsWorld* world) const {

    std::vector<ContactPoint> contactPoints;

    // Get the list of contact manifolds from the world
    std::vector<const rp3d::ContactManifold*> manifolds = world->getContactsList();

    // For each contact manifold
    std::vector<const rp3d::ContactManifold*>::const_iterator it;
    for (it = manifolds.begin(); it != manifolds.end(); ++it) {

        const rp3d::ContactManifold* manifold = *it;

        // For each contact point of the manifold
        for (uint i=0; i<manifold->getNbContactPoints(); i++) {

            rp3d::ContactPoint* contactPoint = manifold->getContactPoint(i);
            rp3d::Vector3 point = contactPoint->getWorldPointOnBody1();
            ContactPoint contact(openglframework::Vector3(point.x, point.y, point.z));
            contactPoints.push_back(contact);
        }

    }

    return contactPoints;
}
