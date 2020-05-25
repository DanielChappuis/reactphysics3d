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
#include <reactphysics3d/reactphysics3d.h>
#include "PhysicsObject.h"

// Constants
const int SHADOWMAP_WIDTH = 2048;
const int SHADOWMAP_HEIGHT = 2048;

// Class SceneDemo
// Abstract class that represents a 3D scene for the ReactPhysics3D examples.
// This scene has a single light source with shadow mapping.
class SceneDemo : public Scene {

    protected:

        // -------------------- Constants -------------------- //

		static constexpr int NB_SHADOW_MAPS = 3;

        // -------------------- Attributes -------------------- //

        /// Light 0
        openglframework::Light mLight0;

        /// Light 1
        openglframework::Light mLight1;

        /// Light 2
        openglframework::Light mLight2;

        /// True if the shadows FBO, textures have been created
        bool mIsShadowMappingInitialized;

        /// Array of FBO for the shadow maps
        openglframework::FrameBufferObject mFBOShadowMap[NB_SHADOW_MAPS];

        /// Shadow map texture
		openglframework::Texture2D mShadowMapTexture[NB_SHADOW_MAPS];

        static int shadowMapTextureLevel;

        /// All the visual contact points
        std::vector<VisualContactPoint*> mVisualContactPoints;

        /// Shadow map bias matrix
        openglframework::Matrix4 mShadowMapBiasMatrix;

        /// Cameras at lights position for the shadow maps
        openglframework::Camera mShadowMapLightCameras[NB_SHADOW_MAPS];

        /// Depth shader to render the shadow map
        openglframework::Shader mDepthShader;

        /// Phong shader
        openglframework::Shader mPhongShader;

		/// Constant color shader
		openglframework::Shader mColorShader;

        openglframework::Shader mQuadShader;

        openglframework::VertexArrayObject mVAOQuad;

        openglframework::VertexBufferObject mVBOQuad;

        /// Vertex Buffer Object for the debug info lines vertices data
        openglframework::VertexBufferObject mDebugVBOLinesVertices;

        /// Vertex Array Object for the lines vertex data
        openglframework::VertexArrayObject mDebugLinesVAO;

        /// Vertex Buffer Object for the debug info trinangles vertices data
        openglframework::VertexBufferObject mDebugVBOTrianglesVertices;

        /// Vertex Array Object for the triangles vertex data
        openglframework::VertexArrayObject mDebugTrianglesVAO;

        static openglframework::Color mObjectColorDemo;
        static openglframework::Color mFloorColorDemo;
        static openglframework::Color mSleepingColorDemo;
        static openglframework::Color mSelectedObjectColorDemo;

        std::string mMeshFolderPath;

        rp3d::PhysicsCommon mPhysicsCommon;

		std::vector<PhysicsObject*> mPhysicsObjects;

        rp3d::PhysicsWorld* mPhysicsWorld;

        /// True if we need to step the physics simulation each frame
        bool mIsPhysicsWorldSimulated;

        // -------------------- Methods -------------------- //

        /// Create the Shadow map FBO and texture
        void createShadowMapFBOAndTexture();

        /// Used for debugging shadow maps
        void createQuadVBO();

        /// Create a the VAO and VBOs to render the debug infos
        void createDebugVBO();

        void drawTextureQuad();

        /// Update the contact points
        void updateSnapshotContactPoints();

        /// Render the contact points
        void renderSnapshotsContactPoints(openglframework::Shader& shader,
                                 const openglframework::Matrix4& worldToCameraMatrix);

        /// Update VBO with vertices and indices of debug info
        void updateDebugVBO();

        /// Render Debug Infos
        void renderDebugInfos(openglframework::Shader& shader, const openglframework::Matrix4& worldToCameraMatrix);

        /// Remove all contact points
        void removeAllVisualContactPoints();

    public:

        // -------------------- Methods -------------------- //

        /// Constructor
        SceneDemo(const std::string& name, EngineSettings& settings, bool isPhysicsWorldSimulated, float sceneRadius, bool isShadowMappingEnabled = true);

        /// Destructor
        virtual ~SceneDemo() override;

        /// Update the scene
        virtual void update() override;

		/// Update the physics world (take a simulation step)
		/// Can be called several times per frame
		virtual void updatePhysics() override;

        /// Render the scene (possibly in multiple passes for shadow mapping)
        virtual void render() override;

        /// Update the engine settings
        virtual void updateEngineSettings() override;

        /// Render the scene in a single pass
        virtual void renderSinglePass(openglframework::Shader& shader, const openglframework::Matrix4& worldToCameraMatrix);

        /// Enabled/Disable the shadow mapping
        virtual void setIsShadowMappingEnabled(bool isShadowMappingEnabled) override;

        /// Enable/disable debug rendering
        virtual void setIsDebugRendererEnabled(bool isEnabled) override;
};

// Enabled/Disable the shadow mapping
inline void SceneDemo::setIsShadowMappingEnabled(bool isShadowMappingEnabled) {

    Scene::setIsShadowMappingEnabled(isShadowMappingEnabled);

    if (mIsShadowMappingEnabled && !mIsShadowMappingInitialized) {
        createShadowMapFBOAndTexture();
    }
}

// Enable/disable debug rendering
inline void SceneDemo::setIsDebugRendererEnabled(bool isEnabled) {
    mPhysicsWorld->setIsDebugRenderingEnabled(isEnabled);
}

#endif
