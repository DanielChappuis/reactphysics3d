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
uniform vec3 light0PosCameraSpace;			// Camera-space position of the light 0
uniform vec3 light1PosCameraSpace;			// Camera-space position of the light 1
uniform vec3 light2PosCameraSpace;			// Camera-space position of the light 2
uniform vec3 light0DiffuseColor;            // Light 0 diffuse color
uniform vec3 light1DiffuseColor;            // Light 1 diffuse color
uniform vec3 light2DiffuseColor;            // Light 2 diffuse color
uniform sampler2D textureSampler;           // Texture
uniform sampler2D shadowMapSampler0;      // Shadow map texture sampler
uniform sampler2D shadowMapSampler1;      // Shadow map texture sampler
uniform bool isTexture;                     // True if we need to use the texture
uniform vec4 globalVertexColor;                   // Vertex color
uniform bool isShadowEnabled;               // True if shadow mapping is enabled
uniform vec2 shadowMapDimension;            // Shadow map dimension

// In variables
in vec3 vertexPosCameraSpace;          // Camera-space position of the vertex
in vec3 vertexNormalCameraSpace;       // Vertex normal in camera-space
in vec2 texCoords;                     // Texture coordinates
in vec4 shadowMapCoords[2];            // Shadow map texture coords

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
        vec3 textureColor = globalVertexColor.rgb;
	if (isTexture) textureColor = texture(textureSampler, texCoords).rgb;

	// Compute the surface normal vector
	vec3 N = normalize(vertexNormalCameraSpace);

	color = vec4(ambient, 1);

        vec3 lightPosCameraSpace[3];
	lightPosCameraSpace[0] = light0PosCameraSpace;
	lightPosCameraSpace[1] = light1PosCameraSpace;
	lightPosCameraSpace[2] = light2PosCameraSpace;
        vec3 lightDiffuseColor[3];
	lightDiffuseColor[0] = light0DiffuseColor;
	lightDiffuseColor[1] = light1DiffuseColor;
	lightDiffuseColor[2] = light2DiffuseColor;

        bool isShadowEnabledForLight[3];
        isShadowEnabledForLight[0] = true;
        isShadowEnabledForLight[1] = true;
        isShadowEnabledForLight[2] = false;

	// For each light source
        for (int l=0; l < 3; l++) {

                // Compute the diffuse term of light 0
		vec3 L0 = normalize(lightPosCameraSpace[l] - vertexPosCameraSpace);
		float diffuseFactor = max(dot(N, L0), 0.0);
		vec3 diffuse = lightDiffuseColor[l] * diffuseFactor * textureColor;

		// Compute shadow factor
		float shadow = 1.0;
                if (isShadowEnabled && isShadowEnabledForLight[l]) {

			shadow = 0.0;
			float bias = 0.0003;
			float shadowBias = -0.000;
			vec4 shadowMapUV = shadowMapCoords[l];
			shadowMapUV.z -= shadowBias;
			vec4 shadowMapCoordsOverW = shadowMapUV / shadowMapUV.w;

			// PCF Shadow Mapping
			for (float i=-1; i<=1; i++) {
                            for (float j=-1; j<=1; j++) {
                                float distInShadowMap0 = textureLookupPCF(shadowMapSampler0, shadowMapCoordsOverW.xy, vec2(i, j)) + bias;
                                float distInShadowMap1 = textureLookupPCF(shadowMapSampler1, shadowMapCoordsOverW.xy, vec2(i, j)) + bias;
                                float distInShadowMap = l == 0 ? distInShadowMap0 : distInShadowMap1;
                                if (shadowMapCoords[l].w > 0) {
                                    shadow += distInShadowMap < shadowMapCoordsOverW.z ? 0.5 : 1.0;
                                }
                            }
			}
			shadow /= 9.0;
		}

		// Compute the final color
		color += vec4(shadow * diffuse, 0.0);
	}
}
