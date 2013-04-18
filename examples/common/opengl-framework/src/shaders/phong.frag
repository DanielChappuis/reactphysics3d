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
uniform vec3 cameraWorldPosition;           // World position of the camera
uniform vec3 lightWorldPosition;            // World position of the light
uniform vec3 lightAmbientColor;             // Lights ambient color
uniform vec3 lightDiffuseColor;             // Light diffuse color
uniform vec3 lightSpecularColor;            // Light specular color
uniform float shininess;                    // Shininess
uniform sampler2D texture;                  // Texture
uniform bool isTexture;                     // True if we need to use the texture

// Varying variables
varying vec3 worldPosition;             // World position of the vertex
varying vec3 worldNormal;               // World surface normalWorld
varying vec2 texCoords;                 // Texture coordinates

void main() {

    // Compute the ambient term
    vec3 ambient = lightAmbientColor;

    // Get the texture color

    vec3 textureColor = vec3(1);
    if (isTexture) textureColor = texture2D(texture, texCoords).rgb;

    // Compute the diffuse term
    vec3 L = normalize(lightWorldPosition - worldPosition);
    vec3 N = normalize(worldNormal);
    vec3 diffuse = lightDiffuseColor * max(dot(N, L), 0.0) * textureColor;

    // Compute the specular term
    vec3 V = normalize(cameraWorldPosition - worldPosition);
    vec3 H = normalize(V + L);
    vec3 specular = lightSpecularColor * pow(max(dot(N, H), 0), shininess);

    // Compute the final color
    gl_FragColor = vec4(ambient + diffuse + specular, 1.0);
}
