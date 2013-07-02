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

// Uniform variables
uniform mat4 localToCameraMatrix;       // Local-space to camera-space matrix
uniform mat4 projectionMatrix;          // Projection matrix
uniform mat3 normalMatrix;              // Normal matrix

// Varying variables
varying vec3 vertexPosCameraSpace;      // Camera-space position of the vertex
varying vec3 vertexNormalCameraSpace;   // Vertex normal in camera-space
varying vec2 texCoords;                 // Texture coordinates

void main() {

    // Compute the vertex position
    vec4 positionCameraSpace = localToCameraMatrix * gl_Vertex;
    vertexPosCameraSpace = positionCameraSpace.xyz;

    // Compute the world surface normal
    vertexNormalCameraSpace = normalMatrix * gl_Normal;

    // Get the texture coordinates
    texCoords = gl_MultiTexCoord0.xy;

    // Compute the clip-space vertex coordinates
    gl_Position = projectionMatrix * positionCameraSpace;
}
