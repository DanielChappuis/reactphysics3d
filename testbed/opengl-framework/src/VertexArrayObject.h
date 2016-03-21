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

#ifndef VERTEX_ARRAY_OBJECT_H
#define VERTEX_ARRAY_OBJECT_H

// Libraries
#include <nanogui/opengl.h>
#include <cassert>
#include <iostream>

namespace openglframework {


// Class VertexArrayObject
class VertexArrayObject {

    private :

        // -------------------- Attributes -------------------- //

        /// ID of the Vertex Array Object
        GLuint mVertexArrayID;

    public :

        // -------------------- Methods -------------------- //

        /// Constructor
        VertexArrayObject();

        /// Destructor
        ~VertexArrayObject();

        /// Create the vertex buffer object
        bool create();

        /// Bind the VAO
        void bind() const;

        /// Unbind the VAO
        void unbind() const;

        /// Return true if the needed OpenGL extensions are available for VAO
        static bool checkOpenGLExtensions();

        /// Destroy the VAO
        void destroy();
};

// Bind the VAO
inline void VertexArrayObject::bind() const {
    assert(mVertexArrayID != 0);

    // Bind the VAO
    glBindVertexArray(mVertexArrayID);
}

// Unbind the VAO
inline void VertexArrayObject::unbind() const {
    assert(mVertexArrayID != 0);

    // Unbind the VAO
    glBindVertexArray(0);
}

// Return true if the needed OpenGL extensions are available for VAO
inline bool VertexArrayObject::checkOpenGLExtensions() {

    // Check that OpenGL version is at least 3.0 or there the vertex array object extension exists
    //return (GLEW_VERSION_3_0 || GL_ARB_vertex_array_object);
    return true;
}

}

#endif

