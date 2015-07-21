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
                     mPhongShader("shaders/phong.vert", "shaders/phong.frag") {

    // Move the light0
    mLight0.translateWorld(Vector3(7, 15, 15));

    // Camera at light0 postion for the shadow map
    mShadowMapLightCamera.translateWorld(mLight0.getOrigin());
    mShadowMapLightCamera.setDimensions(SHADOWMAP_WIDTH, SHADOWMAP_HEIGHT);
    mShadowMapLightCamera.setFieldOfView(45.0f);
    mShadowMapLightCamera.setSceneRadius(sceneRadius);
    mShadowMapLightCamera.setZoom(1.0);

    // Bias matrix for the shadow map
    mShadowMapBiasMatrix.setAllValues(0.5f, 0.0f, 0.0f, 0.5f,
                                      0.0f, 0.5f, 0.5f, 0.5f,
                                      0.0f, 0.0f, 0.0f, 0.5f,
                                      0.0f, 0.0f, 0.0f, 1.0f);

    // Create the Shadow map FBO and texture
    createShadowMapFBOAndTexture();
}

// Destructor
SceneDemo::~SceneDemo() {

    mShadowMapTexture.destroy();
    mFBOShadowMap.destroy();
}

// Render the scene (in multiple passes for shadow mapping)
void SceneDemo::render() {

    glEnable(GL_DEPTH_TEST);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glEnable(GL_CULL_FACE);

    // Get the world-space to camera-space matrix
    const openglframework::Matrix4 worldToCameraMatrix = mCamera.getTransformMatrix().getInverse();

    // ---------- Render the scene to generate the shadow map (first pass) ----------- //

    // Bind the shader
    mDepthShader.bind();

    // Set the variables of the shader
    mDepthShader.setMatrix4x4Uniform("projectionMatrix", mCamera.getProjectionMatrix());

    // Render the objects of the scene
    renderSinglePass(mDepthShader);

    // Unbind the shader
    mDepthShader.unbind();

    // ---------- Render the scene for final rendering (second pass) ----------- //

    mPhongShader.bind();

    // Set the variables of the shader
    mPhongShader.setMatrix4x4Uniform("projectionMatrix", mCamera.getProjectionMatrix());
    mPhongShader.setVector3Uniform("light0PosCameraSpace",worldToCameraMatrix * mLight0.getOrigin());
    mPhongShader.setVector3Uniform("lightAmbientColor", Vector3(0.3f, 0.3f, 0.3f));
    const Color& diffCol = mLight0.getDiffuseColor();
    const Color& specCol = mLight0.getSpecularColor();
    mPhongShader.setVector3Uniform("light0DiffuseColor", Vector3(diffCol.r, diffCol.g, diffCol.b));
    mPhongShader.setVector3Uniform("light0SpecularColor", Vector3(specCol.r, specCol.g, specCol.b));
    mPhongShader.setFloatUniform("shininess", 60.0f);

    // Render the objects of the scene
    renderSinglePass(mPhongShader);

    mPhongShader.unbind();
}

// Create the Shadow map FBO and texture
void SceneDemo::createShadowMapFBOAndTexture() {

    // Create the texture for the depth values
    mShadowMapTexture.create(SHADOWMAP_WIDTH, SHADOWMAP_HEIGHT, GL_DEPTH_COMPONENT, GL_DEPTH_COMPONENT,
                             GL_UNSIGNED_BYTE, GL_NEAREST, GL_NEAREST, GL_CLAMP, GL_CLAMP, NULL);

    // Create the FBO for the shadow map
    mFBOShadowMap.create(0, 0, false);
    mFBOShadowMap.bind(GL_NONE);
    mFBOShadowMap.attachTexture(GL_DEPTH_ATTACHMENT_EXT, mShadowMapTexture.getID());
}

// Render the shadow map
void SceneDemo::renderShadowMap() {

}
