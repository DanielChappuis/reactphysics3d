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
#include "PhysicsObject.h"

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

		/// Constant color shader
		openglframework::Shader mColorShader;

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

		std::vector<PhysicsObject*> mPhysicsObjects;

        rp3d::CollisionWorld* mPhysicsWorld;

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


        /// Render the AABBs
        void renderAABBs(const openglframework::Matrix4& worldToCameraMatrix);

        /// Remove all contact points
        void removeAllContactPoints();

        /// Return a reference to the dynamics world
        rp3d::DynamicsWorld* getDynamicsWorld();

        /// Return a reference to the dynamics world
        const rp3d::DynamicsWorld* getDynamicsWorld() const;

    public:

        // -------------------- Methods -------------------- //

        /// Constructor
        SceneDemo(const std::string& name, EngineSettings& settings, float sceneRadius, bool isShadowMappingEnabled = true);

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

        /// Return all the contact points of the scene
        std::vector<ContactPoint> computeContactPointsOfWorld(reactphysics3d::DynamicsWorld *world);
};

// Enabled/Disable the shadow mapping
inline void SceneDemo::setIsShadowMappingEnabled(bool isShadowMappingEnabled) {

    Scene::setIsShadowMappingEnabled(isShadowMappingEnabled);

    if (mIsShadowMappingEnabled && !mIsShadowMappingInitialized) {
        createShadowMapFBOAndTexture();
    }
}

// Return a reference to the dynamics world
inline rp3d::DynamicsWorld* SceneDemo::getDynamicsWorld() {
    return dynamic_cast<rp3d::DynamicsWorld*>(mPhysicsWorld);
}

// Return a reference to the dynamics world
inline const rp3d::DynamicsWorld* SceneDemo::getDynamicsWorld() const {
    return dynamic_cast<rp3d::DynamicsWorld*>(mPhysicsWorld);
}

#endif


