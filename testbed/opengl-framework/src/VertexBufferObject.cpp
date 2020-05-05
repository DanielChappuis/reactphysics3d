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
#include "VertexBufferObject.h"

using namespace openglframework;

// Constructor
VertexBufferObject::VertexBufferObject(GLenum targetData)
                   : mVertexBufferID(0), mTargetData(targetData) {

}

// Destructor
VertexBufferObject::~VertexBufferObject() {
    destroy();
}

// Create the vertex buffer object
bool VertexBufferObject::create() {

    // Destroy the current VBO
    destroy();

    // Check that the needed OpenGL extensions are available
    bool isExtensionOK = checkOpenGLExtensions();
    if (!isExtensionOK) {
        std::cerr << "Error : Impossible to use Vertex Buffer Object on this platform" << std::endl;
        assert(false);
        return false;
    }

    // Generate a new VBO
    glGenBuffers(1, &mVertexBufferID);
    assert(mVertexBufferID != 0);

    return true;
}

// Copy data into the VBO
void VertexBufferObject::copyDataIntoVBO(GLsizei size, const void* data, GLenum usage) {

    // Copy the data into the VBO
    glBufferData(mTargetData, size, data, usage);
}

// Map VBO data memory into client's memory
void* VertexBufferObject::mapBuffer(GLenum access) {
    return glMapBuffer(mTargetData, access);
}

// Unmap VBO data memory from client's memory
void VertexBufferObject::unmapBuffer() {
    glUnmapBuffer(mTargetData);
}

// Destroy the VBO
void VertexBufferObject::destroy() {

    // Delete the vertex buffer object
    if (mVertexBufferID != 0) {
        glDeleteFramebuffers(1, &mVertexBufferID);
        mVertexBufferID = 0;
    }
}
