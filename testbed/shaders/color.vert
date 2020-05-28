#version 330

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

// Uniform variables
uniform mat4 localToWorldMatrix;        // Local-space to world-space matrix
uniform mat4 worldToCameraMatrix;       // World-space to camera-space matrix
uniform mat4 projectionMatrix;          // Projection matrix

// In variables
in vec4 vertexPosition;
in uint vertexColor;

// Out variables
out vec4 vertexColorOut;

void main() {

    // Compute the vertex position
    vec4 positionCameraSpace = worldToCameraMatrix * localToWorldMatrix * vertexPosition;

    // Compute the clip-space vertex coordinates
    gl_Position = projectionMatrix * positionCameraSpace;

    // Transfer the vertex color to the fragment shader
    vertexColorOut = vec4((vertexColor & 0xFF0000u) >> 16, (vertexColor & 0x00FF00u) >> 8, vertexColor & 0x0000FFu, 0xFF);
}
