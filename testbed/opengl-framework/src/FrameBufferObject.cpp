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
#include "FrameBufferObject.h"
#include <iostream>

using namespace openglframework;
using namespace std;

// Constructor
FrameBufferObject::FrameBufferObject() : mFrameBufferID(0), mRenderBufferID (0) {

}

// Destructor
FrameBufferObject::~FrameBufferObject() {
    destroy();
}

// Create the frame buffer object
bool FrameBufferObject::create(uint width, uint height, bool needRenderBuffer) {

    // Destroy the current FBO
    destroy();

    // Check that the needed OpenGL extensions are available
    bool isExtensionOK = checkOpenGLExtensions();
    if (!isExtensionOK) {
        std::cerr << "Error : Impossible to use Framebuffer Object on this platform" << std::endl;
        assert(false);
        return false;
    }

    // Generate a new FBO
    glGenFramebuffers(1, &mFrameBufferID);
    assert(mFrameBufferID != 0);

    // If we also need to create a render buffer
    if (needRenderBuffer) {

        // Generate the render buffer
        glGenRenderbuffers(1, &mRenderBufferID);
        assert(mRenderBufferID != 0);

        glBindRenderbuffer(GL_RENDERBUFFER, mRenderBufferID);
        glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT32, width, height);
        glBindFramebuffer(GL_FRAMEBUFFER, mFrameBufferID);
        glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER,
                                  mRenderBufferID);
        glBindRenderbuffer(GL_RENDERBUFFER, 0);
        glBindFramebuffer(GL_FRAMEBUFFER, 0);
    }

    // Check the FBO status
    GLenum statusFBO = glCheckFramebufferStatus(GL_FRAMEBUFFER);
    if(statusFBO != GL_FRAMEBUFFER_COMPLETE) {
        std::cerr << "Error : An error occured while creating the Frame Buffer Object !" << endl;
        assert(false);
        return false;
    }

    return true;
}

// Destroy the FBO
void FrameBufferObject::destroy() {

    // Delete the frame buffer object
    if (mFrameBufferID) {
        glDeleteFramebuffers(1, &mFrameBufferID);
        mFrameBufferID = 0;
    }

    // Delete the render buffer
    if (mRenderBufferID) {
        glDeleteRenderbuffers(1, &mRenderBufferID);
    }
}

// Attach a texture to the frame buffer object
void FrameBufferObject::attachTexture(uint position, uint textureID) {
    assert(mFrameBufferID);

    // Bind the current FBO
    glBindFramebuffer(GL_FRAMEBUFFER, mFrameBufferID);

    // Bind the texture
    glFramebufferTexture2D(GL_FRAMEBUFFER, position, GL_TEXTURE_2D, textureID, 0);

    // Unbind the current FBO
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}
