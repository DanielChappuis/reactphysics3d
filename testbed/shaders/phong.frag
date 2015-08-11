#version 330

/********************************************************************************
* OpenGL-Framework                                                              *
* Copyright (c) 2015 Daniel Chappuis                                            *
*********************************************************************************
*                                                                               *
* This software is provided 'as-is', without any express or implied warranty.   *
* In no event will the authors be held liable for any damages arising from the  *
* use of this software.                                                         *
*                                                                                *
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
uniform vec3 lightAmbientColor;             // Lights ambient color
uniform vec3 light0PosCameraSpace;          // Camera-space position of the light
uniform vec3 light0DiffuseColor;            // Light 0 diffuse color
uniform sampler2D textureSampler;           // Texture
uniform sampler2D shadowMapSampler;         // Shadow map texture sampler
uniform bool isTexture;                     // True if we need to use the texture
uniform vec4 vertexColor;                   // Vertex color
uniform bool isShadowEnabled;               // True if shadow mapping is enabled
uniform vec2 shadowMapDimension;            // Shadow map dimension

// In variables
in vec3 vertexPosCameraSpace;          // Camera-space position of the vertex
in vec3 vertexNormalCameraSpace;       // Vertex normal in camera-space
in vec2 texCoords;                     // Texture coordinates
in vec4 shadowMapCoords;                // Shadow map texture coords

// Out variable
out vec4 color;                        // Output color

// Texture for PCF Shadow mapping
float textureLookupPCF(sampler2D map, vec2 texCoords, vec2 offset)
{
    vec2 shadowMapScale = vec2(1.0, 1.0) / shadowMapDimension;
    return texture(map, texCoords.xy + offset * shadowMapScale).r;
}

void main() {

    // Compute the ambient term
    vec3 ambient = lightAmbientColor;

    // Get the texture color
    vec3 textureColor = vertexColor.rgb;
    if (isTexture) textureColor = texture(textureSampler, texCoords).rgb;

    // Compute the surface normal vector
    vec3 N = normalize(vertexNormalCameraSpace);

    // Compute the diffuse term of light 0
    vec3 L0 = normalize(light0PosCameraSpace - vertexPosCameraSpace);
    float diffuseFactor = max(dot(N, L0), 0.0);
    vec3 diffuse = light0DiffuseColor * diffuseFactor * textureColor;

    // Compute shadow factor
    float shadow = 1.0;
    if (isShadowEnabled) {
        shadow = 0.0;
        float bias = 0.0003;
        float shadowBias = -0.000;
        vec4 shadowMapUV = shadowMapCoords;
        shadowMapUV.z -= shadowBias;
        vec4 shadowMapCoordsOverW = shadowMapUV / shadowMapUV.w;

        // PCF Shadow Mapping
        for (float i=-1; i<=1; i++) {
            for (float j=-1; j<=1; j++) {
                float distInShadowMap = textureLookupPCF(shadowMapSampler, shadowMapCoordsOverW.xy, vec2(i, j)) + bias;
                if (shadowMapCoords.w > 0) {
                    shadow += distInShadowMap < shadowMapCoordsOverW.z ? 0.5 : 1.0;
                }
            }
        }
        shadow /= 9.0;

        /*
        float distanceInShadowMap = texture(shadowMapSampler, shadowMapCoordsOverW.xy).r + bias;
        if (shadowMapCoords.w > 0) {
            shadow = distanceInShadowMap < shadowMapCoordsOverW.z ? 0.5 : 1.0;
        }
        */
    }

    // Compute the final color
    color = vec4(ambient + shadow * diffuse, 1.0);
}
