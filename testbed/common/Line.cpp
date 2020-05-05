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
#include "Line.h"

// Constructor
Line::Line(const openglframework::Vector3& worldPoint1,
           const openglframework::Vector3& worldPoint2)
     : mWorldPoint1(worldPoint1), mWorldPoint2(worldPoint2) {

}

// Destructor
Line::~Line() {


}

// Render the sphere at the correct position and with the correct orientation
void Line::render(openglframework::Shader& shader, const openglframework::Matrix4& worldToCameraMatrix) {

    // Bind the shader
    shader.bind();

    // Set the model to camera matrix
    shader.setMatrix4x4Uniform("localToWorldMatrix", openglframework::Matrix4::identity());
    shader.setMatrix4x4Uniform("worldToCameraMatrix", worldToCameraMatrix);

    // Set the vertex color
    openglframework::Vector4 color(1, 0, 0, 1);
    shader.setIntUniform("isGlobalVertexColorEnabled", 1, false);
    shader.setVector4Uniform("globalVertexColor", color, false);

    /*
    glBegin(GL_LINES);
        glVertex3f(mWorldPoint1.x, mWorldPoint1.y, mWorldPoint1.z);
        glVertex3f(mWorldPoint2.x, mWorldPoint2.y, mWorldPoint2.z);
    glEnd();
    */

    // Unbind the shader
    shader.unbind();
}
