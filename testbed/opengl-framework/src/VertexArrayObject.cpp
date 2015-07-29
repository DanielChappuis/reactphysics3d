/********************************************************************************
* OpenGL-Framework                                                              *
* Copyright (c) 2015 Daniel Chappuis                                            *
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
#include "VertexArrayObject.h"

using namespace openglframework;

// Constructor
VertexArrayObject::VertexArrayObject() : mVertexArrayID(0) {

}

// Destructor
VertexArrayObject::~VertexArrayObject() {
    destroy();
}

// Create the vertex buffer object
bool VertexArrayObject::create() {

    // Destroy the current VAO
    destroy();

    // Check that the needed OpenGL extensions are available
    bool isExtensionOK = checkOpenGLExtensions();
    if (!isExtensionOK) {
        std::cerr << "Error : Impossible to use Vertex Array Object on this platform" << std::endl;
        assert(false);
        return false;
    }

    // Generate a new VAO
    glGenVertexArrays(1, &mVertexArrayID);
    assert(mVertexArrayID != 0);

    return true;
}

// Destroy the VAO
void VertexArrayObject::destroy() {

    // Delete the vertex buffer object
    if (mVertexArrayID != 0) {
        glDeleteVertexArrays(1, &mVertexArrayID);
        mVertexArrayID = 0;
    }
}
