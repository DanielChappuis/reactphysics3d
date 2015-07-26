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
#include "SceneDemo.h"
#include <GLFW/glfw3.h>

using namespace openglframework;

// Constructor
SceneDemo::SceneDemo(const std::string& name, float sceneRadius) : Scene(name), mLight0(0),
                     mDepthShader("shaders/depth.vert", "shaders/depth.frag"),
                     mPhongShader("shaders/phong.vert", "shaders/phong.frag"),
                     mQuadShader("shaders/quad.vert", "shaders/quad.frag"),
                     mVBOQuad(GL_ARRAY_BUFFER) {

    // Move the light0
    mLight0.translateWorld(Vector3(0, 60, 00));

    // Camera at light0 postion for the shadow map
    mShadowMapLightCamera.translateWorld(mLight0.getOrigin());
    mShadowMapLightCamera.rotateLocal(Vector3(1, 0, 0), -PI / 2.0f);
    mShadowMapLightCamera.setDimensions(SHADOWMAP_WIDTH, SHADOWMAP_HEIGHT);
    mShadowMapLightCamera.setFieldOfView(70.0f);
    mShadowMapLightCamera.setSceneRadius(200);
    //mShadowMapLightCamera.setZoom(1.0);


    mShadowMapBiasMatrix.setAllValues(0.5, 0.0, 0.0, 0.5,
                                      0.0, 0.5, 0.0, 0.5,
                                      0.0, 0.0, 0.5, 0.5,
                                      0.0, 0.0, 0.0, 1.0);

    // Create the Shadow map FBO and texture
    createShadowMapFBOAndTexture();

    createQuadVBO();
}

// Destructor
SceneDemo::~SceneDemo() {

    mShadowMapTexture.destroy();
    mFBOShadowMap.destroy();
}

// Render the scene (in multiple passes for shadow mapping)
void SceneDemo::render() {

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);

    // ---------- Render the scene to generate the shadow map (first pass) ----------- //

    // Culling switching, rendering only backface, this is done to avoid self-shadowing
    glCullFace(GL_FRONT);

    Matrix4 shadowMapProjMatrix = mShadowMapLightCamera.getProjectionMatrix();

    // Get the world-space to camera-space matrix
    const openglframework::Matrix4 worldToLightCameraMatrix = mShadowMapLightCamera.getTransformMatrix().getInverse();

    mFBOShadowMap.bind();

    // Bind the shader
    mDepthShader.bind();

    // Set the variables of the shader
    mDepthShader.setMatrix4x4Uniform("projectionMatrix", shadowMapProjMatrix);

    // Set the viewport to render into the shadow map texture
    glViewport(0, 0, SHADOWMAP_WIDTH, SHADOWMAP_HEIGHT);

    // Clear previous frame values
    glClear( GL_DEPTH_BUFFER_BIT);

    // Disable color rendering, we only want to write to the Z-Buffer
    glColorMask(GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE);

    // Render the objects of the scene
    renderSinglePass(mDepthShader, worldToLightCameraMatrix);

    // Unbind the shader
    mDepthShader.unbind();

    mFBOShadowMap.unbind();


    // ---------- Render the scene for final rendering (second pass) ----------- //

    glCullFace(GL_BACK);

    // Get the world-space to camera-space matrix
    const openglframework::Matrix4 worldToCameraMatrix = mCamera.getTransformMatrix().getInverse();

    mPhongShader.bind();

    mShadowMapTexture.bind();

    // Set the variables of the shader
    mPhongShader.setMatrix4x4Uniform("projectionMatrix", mCamera.getProjectionMatrix());
    mPhongShader.setMatrix4x4Uniform("shadowMapProjectionMatrix", mShadowMapBiasMatrix * shadowMapProjMatrix);
    mPhongShader.setMatrix4x4Uniform("worldToLight0CameraMatrix", worldToLightCameraMatrix);
    mPhongShader.setVector3Uniform("light0PosCameraSpace", worldToCameraMatrix * mLight0.getOrigin());
    mPhongShader.setVector3Uniform("lightAmbientColor", Vector3(0.3f, 0.3f, 0.3f));
    const Color& diffCol = mLight0.getDiffuseColor();
    const Color& specCol = mLight0.getSpecularColor();
    mPhongShader.setVector3Uniform("light0DiffuseColor", Vector3(diffCol.r, diffCol.g, diffCol.b));
    mPhongShader.setVector3Uniform("light0SpecularColor", Vector3(specCol.r, specCol.g, specCol.b));
    mPhongShader.setFloatUniform("shininess", 60.0f);
    mPhongShader.setIntUniform("shadowMapSampler", mShadowMapTexture.getID());

    // Set the viewport to render the scene
    glViewport(mViewportX, mViewportY, mViewportWidth, mViewportHeight);

    //Enabling color write (previously disabled for light POV z-buffer rendering)
    glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);

    // Clear previous frame values
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // Render the objects of the scene
    renderSinglePass(mPhongShader, worldToCameraMatrix);

    mShadowMapTexture.unbind();
    mPhongShader.unbind();

   //drawTextureQuad();
}

// Create the Shadow map FBO and texture
void SceneDemo::createShadowMapFBOAndTexture() {

    // Create the texture for the depth values
    mShadowMapTexture.create(SHADOWMAP_WIDTH, SHADOWMAP_HEIGHT, GL_DEPTH_COMPONENT24, GL_DEPTH_COMPONENT,
                             GL_UNSIGNED_BYTE, GL_NEAREST, GL_NEAREST, GL_CLAMP, GL_CLAMP, NULL);
    mShadowMapTexture.setLayer(1);

    // Create the FBO for the shadow map
    mFBOShadowMap.create(0, 0, false);
    mFBOShadowMap.bind();

    // Tell OpenGL that we won't bind a color texture with the currently binded FBO
    glDrawBuffer(GL_NONE);
    glReadBuffer(GL_NONE);

    mFBOShadowMap.attachTexture(GL_DEPTH_ATTACHMENT_EXT, mShadowMapTexture.getID());
    mFBOShadowMap.unbind();
}

// TODO : Delete this
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

// TODO : Delete this
void SceneDemo::drawTextureQuad() {

    glViewport(mViewportX, mViewportY, mViewportWidth, mViewportHeight);
    glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);

    // Clear previous frame values
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    mVAOQuad.bind();
    mQuadShader.bind();
    mShadowMapTexture.bind();
    mQuadShader.setIntUniform("textureSampler", mShadowMapTexture.getID());
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
