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
uniform mat4 worldToLight0CameraMatrix; // World-space to light0 camera-space matrix (for shadow mapping)
uniform mat4 worldToLight1CameraMatrix; // World-space to light1 camera-space matrix (for shadow mapping)
uniform mat4 projectionMatrix;          // Projection matrix
uniform mat3 normalMatrix;              // Normal matrix
uniform mat4 shadowMapLight0ProjectionMatrix; // Shadow map projection matrix for light 0
uniform mat4 shadowMapLight1ProjectionMatrix; // Shadow map projection matrix for light 1

// In variables
in vec4 vertexPosition;
in vec3 vertexNormal;
in vec2 textureCoords;

// Out variables
out vec3 vertexPosCameraSpace;      // Camera-space position of the vertex
out vec3 vertexNormalCameraSpace;   // Vertex normal in camera-space
out vec2 texCoords;                 // Texture coordinates
out vec4 shadowMapCoords[2];        // Shadow map texture coords

void main() {

    // Compute the vertex position
    vec4 positionCameraSpace = worldToCameraMatrix * localToWorldMatrix * vertexPosition;
    vertexPosCameraSpace = positionCameraSpace.xyz;

    // Compute the world surface normal
    vertexNormalCameraSpace = normalMatrix * vertexNormal;

    // Get the texture coordinates
    texCoords = textureCoords;

    // Compute the texture coords of the vertex in the shadow map
    mat4 worldToLightCameraMatrix[2];
    worldToLightCameraMatrix[0] = worldToLight0CameraMatrix;
    worldToLightCameraMatrix[1] = worldToLight1CameraMatrix;
    mat4 shadowMapProjectionMatrix[2];
    shadowMapProjectionMatrix[0] = shadowMapLight0ProjectionMatrix;
    shadowMapProjectionMatrix[1] = shadowMapLight1ProjectionMatrix;
    for (int l=0; l < 2; l++) {
        shadowMapCoords[l] = shadowMapProjectionMatrix[l] * worldToLightCameraMatrix[l] * localToWorldMatrix * vertexPosition;
    }

    // Compute the clip-space vertex coordinates
    gl_Position = projectionMatrix * positionCameraSpace;
}
