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

#ifndef VERTEX_BUFFER_OBJECT_H
#define VERTEX_BUFFER_OBJECT_H

// Libraries
#include <nanogui/opengl.h>
#include <cassert>
#include <iostream>

namespace openglframework {


// Class VertexBufferObject
class VertexBufferObject {

    private :

        // -------------------- Attributes -------------------- //

        /// ID of the Vertex Buffer Object
        GLuint mVertexBufferID;

        /// Target data. This variable must be GL_ARRAY_BUFFER if the VBO contains vertex
        /// data (vertex coordinates, texture coordinates, normals, colors) or must be
        /// GL_ELEMENT_ARRAY_BUFFER if the VBO contains index data (index array).
        GLenum mTargetData;

    public :

        // -------------------- Methods -------------------- //

        /// Constructor
        VertexBufferObject(GLenum targetData);

        /// Destructor
        ~VertexBufferObject();

        /// Create the vertex buffer object
        bool create();

        /// Copy data into the VBO
        void copyDataIntoVBO(GLsizei size, const void* data, GLenum usage);

        /// Map VBO data memory into client's memory
        void* mapBuffer(GLenum access);

        /// Unmap VBO data memory from client's memory
        void unmapBuffer();

        /// Bind the VBO
        void bind() const;

        /// Unbind the VBO
        void unbind() const;

        /// Return true if the needed OpenGL extensions are available for VBO
        static bool checkOpenGLExtensions();

        /// Destroy the VBO
        void destroy();
};

// Bind the VBO
inline void VertexBufferObject::bind() const {
    assert(mVertexBufferID != 0);

    // Bind the VBO
    glBindBuffer(mTargetData, mVertexBufferID);
}

// Unbind the VBO
inline void VertexBufferObject::unbind() const {
    assert(mVertexBufferID != 0);

    // Unbind the VBO
    glBindBuffer(mTargetData, 0);
}

// Return true if the needed OpenGL extensions are available for VBO
inline bool VertexBufferObject::checkOpenGLExtensions() {

    // Check that OpenGL version is at least 1.5 or there the vertex buffer object extension exists
    //return (GLEW_VERSION_1_5 || GL_ARB_vertex_buffer_object);
    return true;
}

}

#endif
