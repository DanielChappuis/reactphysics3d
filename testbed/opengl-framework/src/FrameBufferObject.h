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

#ifndef FRAME_BUFFER_OBJECT_H
#define FRAME_BUFFER_OBJECT_H

// Libraries
#include "definitions.h"
#include <nanogui/opengl.h>
#include <cassert>
#include <iostream>

namespace openglframework {


// Class FrameBufferObject
class FrameBufferObject {

    private:

        // -------------------- Attributes -------------------- //

        // Frame buffer ID
        uint mFrameBufferID;

        // Render buffer ID
        uint mRenderBufferID;

    public:

        // -------------------- Methods -------------------- //

        // Constructor
        FrameBufferObject();

        // Destructor
        ~FrameBufferObject();

        // Create the frame buffer object
        bool create(uint width, uint height, bool needRenderBuffer = true);

        // Attach a texture to the frame buffer object
        void attachTexture(uint position, uint textureID);

        // Bind the FBO
        void bind() const;

        // Unbind the FBO
        void unbind() const;

        // Return true if the needed OpenGL extensions are available for FBO
        static bool checkOpenGLExtensions();

        // Destroy the FBO
        void destroy();
};

// Bind the FBO
inline void FrameBufferObject::bind() const {
    assert(mFrameBufferID != 0);
    glBindFramebuffer(GL_FRAMEBUFFER, mFrameBufferID);
}

// Unbind the FBO
inline void FrameBufferObject::unbind() const {
    assert(mFrameBufferID != 0);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

// Return true if the needed OpenGL extensions are available for FBO
inline bool FrameBufferObject::checkOpenGLExtensions() {

    // Check that OpenGL version is at least 3.0 or there the framebuffer object extension exists
    //return (GLEW_VERSION_3_0 || GLEW_ARB_framebuffer_object);
    return true;
}

}

#endif
