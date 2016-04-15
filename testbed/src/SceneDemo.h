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

#ifndef SCENEDEMO_H
#define SCENEDEMO_H

// Libraries
#include "Scene.h"
#include "VisualContactPoint.h"
#include "reactphysics3d.h"

// Constants
const int SHADOWMAP_WIDTH = 2048;
const int SHADOWMAP_HEIGHT = 2048;

// Class SceneDemo
// Abstract class that represents a 3D scene for the ReactPhysics3D examples.
// This scene has a single light source with shadow mapping.
class SceneDemo : public Scene {

    protected:

        // -------------------- Attributes -------------------- //

        /// Light 0
        openglframework::Light mLight0;

        /// True if the shadows FBO, textures have been created
        bool mIsShadowMappingInitialized;

        /// FBO for the shadow map
        openglframework::FrameBufferObject mFBOShadowMap;

        /// Shadow map texture
        openglframework::Texture2D mShadowMapTexture;

        static int shadowMapTextureLevel;

        /// All the visual contact points
        std::vector<VisualContactPoint*> mContactPoints;

        /// Shadow map bias matrix
        openglframework::Matrix4 mShadowMapBiasMatrix;

        /// Camera at light0 position for the shadow map
        openglframework::Camera mShadowMapLightCamera;

        /// Depth shader to render the shadow map
        openglframework::Shader mDepthShader;

        /// Phong shader
        openglframework::Shader mPhongShader;

        // TODO : Delete this
        openglframework::Shader mQuadShader;

        // TODO : Delete this
        openglframework::VertexArrayObject mVAOQuad;

        openglframework::VertexBufferObject mVBOQuad;

        static openglframework::Color mGreyColorDemo;
        static openglframework::Color mYellowColorDemo;
        static openglframework::Color mBlueColorDemo;
        static openglframework::Color mOrangeColorDemo;
        static openglframework::Color mPinkColorDemo;
        static openglframework::Color mRedColorDemo;
        static openglframework::Color mDemoColors[];
        static int mNbDemoColors;

        std::string mMeshFolderPath;

        // -------------------- Methods -------------------- //

        // Create the Shadow map FBO and texture
        void createShadowMapFBOAndTexture();

        // Used for debugging shadow maps
        void createQuadVBO();

        // TODO : Delete this
        void drawTextureQuad();

        // Update the contact points
        void updateContactPoints();

        // Render the contact points
        void renderContactPoints(openglframework::Shader& shader,
                                 const openglframework::Matrix4& worldToCameraMatrix);

        void removeAllContactPoints();

    public:

        // -------------------- Methods -------------------- //

        /// Constructor
        SceneDemo(const std::string& name, float sceneRadius, bool isShadowMappingEnabled = true);

        /// Destructor
        virtual ~SceneDemo();

        /// Update the scene
        virtual void update();

        /// Render the scene (possibly in multiple passes for shadow mapping)
        virtual void render();

        /// Render the scene in a single pass
        virtual void renderSinglePass(openglframework::Shader& shader,
                                      const openglframework::Matrix4& worldToCameraMatrix)=0;

        /// Enabled/Disable the shadow mapping
        void virtual setIsShadowMappingEnabled(bool isShadowMappingEnabled);

        /// Return all the contact points of the scene
        std::vector<ContactPoint> computeContactPointsOfWorld(const rp3d::DynamicsWorld* world) const;
};

// Enabled/Disable the shadow mapping
inline void SceneDemo::setIsShadowMappingEnabled(bool isShadowMappingEnabled) {

    Scene::setIsShadowMappingEnabled(isShadowMappingEnabled);

    if (mIsShadowMappingEnabled && !mIsShadowMappingInitialized) {
        createShadowMapFBOAndTexture();
    }
}

#endif


