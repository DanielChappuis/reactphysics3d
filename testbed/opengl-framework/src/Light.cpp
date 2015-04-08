/********************************************************************************
* OpenGL-Framework                                                              *
* Copyright (c) 2013 Daniel Chappuis                                            *
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
#include "Light.h"

// Namespaces
using namespace openglframework;

// Constructor
Light::Light(GLuint id)
      : mLightID(id), mDiffuseColor(Color::white()),
        mSpecularColor(Color::white()), mIsActive(false) {

}

// Constructor
Light::Light(GLuint id, Color diffuseColor, Color specularColor)
      : mLightID(id), mDiffuseColor(diffuseColor),
        mSpecularColor(specularColor), mIsActive(false) {

}

// Destructor
Light::~Light() {

}

// Initialize the light
void Light::init() {

    // Enable the light
    enable();

    // Set the diffuse and specular color
    GLfloat diffuseColor[] = {mDiffuseColor.r, mDiffuseColor.g, mDiffuseColor.b, mDiffuseColor.a};
    GLfloat specularColor[] = {mSpecularColor.r,mSpecularColor.g,mSpecularColor.b,mSpecularColor.a};
    glLightfv(mLightID, GL_DIFFUSE, diffuseColor);
    glLightfv(mLightID, GL_SPECULAR, specularColor);
}

// Create a shadow map associated with this light
bool Light::createShadowMap(uint width, uint height) {

    // Destroy the current shadow map
    destroyShadowMap();

    // Create the Framebuffer object to render the shadow map
    bool isFBOCreated = mFBOShadowMap.create(width, height, false);
    if (!isFBOCreated) {
        std::cerr << "Error : Cannot create the Shadow Map !" << std::endl;
        destroyShadowMap();
        return false;
    }

    // Bind the Framebuffer object
    mFBOShadowMap.bind(GL_NONE);

    // Create the shadow map depth texture
    mShadowMap.create(width, height, GL_DEPTH_COMPONENT, GL_DEPTH_COMPONENT, GL_UNSIGNED_BYTE);

    // Attache the shadow map texture to the Framebuffer object
    mFBOShadowMap.attachTexture(GL_DEPTH_ATTACHMENT_EXT, mShadowMap.getID());

    // Unbind the Framebuffer object
    mFBOShadowMap.unbind();

    // TODO : Change the path of the shader here so that it does not depend on the build folder
    bool isShaderCreated = mDepthShader.create("../../opengl-framework/src/shaders/depth.vert",
                                               "../../opengl-framework/src/shaders/depth.vert");
    if (!isShaderCreated) {
        std::cerr << "Error : Cannot create the Shadow Map !" << std::endl;
        destroyShadowMap();
        return false;
    }

    return true;
}
