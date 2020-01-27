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
#include "SceneDemo.h"
#include <GLFW/glfw3.h>
#include "AABB.h"
#include "constraint/ContactPoint.h"
#include "collision/ContactManifold.h"

using namespace openglframework;

int SceneDemo::shadowMapTextureLevel = 0;
openglframework::Color SceneDemo::mObjectColorDemo = Color(0.76f, 0.67f, 0.47f, 1.0f);
openglframework::Color SceneDemo::mFloorColorDemo = Color(0.47f, 0.48f, 0.49f, 1.0f);
openglframework::Color SceneDemo::mSleepingColorDemo = Color(1.0f, 0.25f, 0.25f, 1.0f);
openglframework::Color SceneDemo::mSelectedObjectColorDemo = Color(0.09f, 0.59f, 0.88f, 1.0f);

// Constructor
SceneDemo::SceneDemo(const std::string& name, EngineSettings& settings, bool isPhysicsWorldSimulated, float sceneRadius, bool isShadowMappingEnabled)
          : Scene(name, settings, isShadowMappingEnabled), mIsShadowMappingInitialized(false),
                     mDepthShader("shaders/depth.vert", "shaders/depth.frag"),
                     mPhongShader("shaders/phong.vert", "shaders/phong.frag"),
					 mColorShader("shaders/color.vert", "shaders/color.frag"),
                     mQuadShader("shaders/quad.vert", "shaders/quad.frag"),
                     mVBOQuad(GL_ARRAY_BUFFER), mMeshFolderPath("meshes/"),
                     mPhysicsWorld(nullptr), mIsPhysicsWorldSimulated(isPhysicsWorldSimulated) {

    shadowMapTextureLevel++;

    // Move the lights
	float lightsRadius = 30.0f;
	float lightsHeight = 20.0f;
    mLight0.translateWorld(Vector3(0 * lightsRadius, lightsHeight, 1 * lightsRadius));
    mLight1.translateWorld(Vector3(0.95f * lightsRadius, lightsHeight, -0.3f * lightsRadius));
    mLight2.translateWorld(Vector3(-0.58f * lightsRadius, lightsHeight, -0.81f * lightsRadius));

	// Set the lights colors
	mLight0.setDiffuseColor(Color(0.6f, 0.6f, 0.6f, 1.0f));
	mLight1.setDiffuseColor(Color(0.6f, 0.6f, 0.6f, 1.0f));
	mLight2.setDiffuseColor(Color(0.6f, 0.6f, 0.6f, 1.0f));

	mShadowMapLightCameras[0].translateWorld(mLight0.getOrigin());
	mShadowMapLightCameras[0].rotateLocal(Vector3(1, 0, 0), -PI / 4.0f);

	mShadowMapLightCameras[1].translateWorld(mLight1.getOrigin());
	mShadowMapLightCameras[1].rotateLocal(Vector3(0, 1, 0), -5.0f * PI/3.7f);
	mShadowMapLightCameras[1].rotateLocal(Vector3(1, 0, 0), -PI/4.0f);

	mShadowMapLightCameras[2].translateWorld(mLight2.getOrigin());
	mShadowMapLightCameras[2].rotateLocal(Vector3(0, 1, 0), 5 * PI/4.0f);
	mShadowMapLightCameras[2].rotateLocal(Vector3(1, 0 , 0), -PI/4.0f);

	for (int i = 0; i < NB_SHADOW_MAPS; i++) {
		mShadowMapLightCameras[i].setDimensions(SHADOWMAP_WIDTH, SHADOWMAP_HEIGHT);
		mShadowMapLightCameras[i].setFieldOfView(100.0f);
		mShadowMapLightCameras[i].setSceneRadius(100);
	}

    mShadowMapBiasMatrix.setAllValues(0.5, 0.0, 0.0, 0.5,
                                      0.0, 0.5, 0.0, 0.5,
                                      0.0, 0.0, 0.5, 0.5,
                                      0.0, 0.0, 0.0, 1.0);

    // Create the Shadow map FBO and texture
    if (mIsShadowMappingEnabled) {
        createShadowMapFBOAndTexture();
    }

    createQuadVBO();

    // Init rendering for the AABBs
    AABB::init();

    VisualContactPoint::createStaticData(mMeshFolderPath);
}

// Destructor
SceneDemo::~SceneDemo() {


    for (int i = 0; i < NB_SHADOW_MAPS; i++) {
        mShadowMapTexture[i].destroy();
        mFBOShadowMap[i].destroy();
    }

    mVBOQuad.destroy();

    mDepthShader.destroy();
    mPhongShader.destroy();
    mQuadShader.destroy();
    mColorShader.destroy();

    // Destroy the contact points
    removeAllVisualContactPoints();

    // Destroy rendering data for the AABB
    AABB::destroy();

    VisualContactPoint::destroyStaticData();
}

// Update the scene
void SceneDemo::update() {

    // Update the contact points
    updateContactPoints();

	// Update the position and orientation of the physics objects
	for (std::vector<PhysicsObject*>::iterator it = mPhysicsObjects.begin(); it != mPhysicsObjects.end(); ++it) {

		// Update the transform used for the rendering
		(*it)->updateTransform(mInterpolationFactor);
	}
}

// Update the physics world (take a simulation step)
// Can be called several times per frame
void SceneDemo::updatePhysics() {

    // Clear contacts points
    mContactPoints.clear();

    if (mIsPhysicsWorldSimulated) {

        // Take a simulation step
        mPhysicsWorld->update(mEngineSettings.timeStep);
    }
}

// Render the scene (in multiple passes for shadow mapping)
void SceneDemo::render() {

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);

	Matrix4 shadowMapProjMatrix[NB_SHADOW_MAPS];
	openglframework::Matrix4 worldToLightCameraMatrix[NB_SHADOW_MAPS];
	for (int i = 0; i < NB_SHADOW_MAPS; i++) {

		shadowMapProjMatrix[i] = mShadowMapLightCameras[i].getProjectionMatrix();
		worldToLightCameraMatrix[i] = mShadowMapLightCameras[i].getTransformMatrix().getInverse();
	}

    // ---------- Render the scene to generate the shadow map (first pass) ----------- //

    // If Shadow Mapping is enabled
    if (mIsShadowMappingEnabled) {

        // Culling switching, rendering only backface, this is done to avoid self-shadowing
        glCullFace(GL_BACK);

		// For each shadow map
		for (int i = 0; i < NB_SHADOW_MAPS; i++) {

			mFBOShadowMap[i].bind();

			// Bind the shader
			mDepthShader.bind();

			// Set the variables of the shader
			mDepthShader.setMatrix4x4Uniform("projectionMatrix", shadowMapProjMatrix[i]);

			// Set the viewport to render into the shadow map texture
			glViewport(0, 0, SHADOWMAP_WIDTH, SHADOWMAP_HEIGHT);

			// Clear previous frame values
			glClear(GL_DEPTH_BUFFER_BIT);

			// Disable color rendering, we only want to write to the Z-Buffer
			glColorMask(GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE);

			// Render the objects of the scene
			renderSinglePass(mDepthShader, worldToLightCameraMatrix[i]);

			// Unbind the shader
			mDepthShader.unbind();

			mFBOShadowMap[i].unbind();
		}

        glDisable(GL_POLYGON_OFFSET_FILL);
    }

    // ---------- Render the scene for final rendering (second pass) ----------- //

    glCullFace(GL_BACK);

    // Get the world-space to camera-space matrix
    const openglframework::Matrix4 worldToCameraMatrix = mCamera.getTransformMatrix().getInverse();

    mPhongShader.bind();

	// Is shadow mapping is enabled
	GLint textureUnits[NB_SHADOW_MAPS];
	if (mIsShadowMappingEnabled) {

		for (int i = 0; i < NB_SHADOW_MAPS; i++) {
			mShadowMapTexture[i].bind();
		}
		for (int i = 0; i < NB_SHADOW_MAPS; i++) {
			textureUnits[i] = mShadowMapTexture[i].getUnit();
		}
	}

    // Set the variables of the phong shader
    mPhongShader.setMatrix4x4Uniform("projectionMatrix", mCamera.getProjectionMatrix());
    mPhongShader.setMatrix4x4Uniform("shadowMapLight0ProjectionMatrix", mShadowMapBiasMatrix * shadowMapProjMatrix[0]);
    mPhongShader.setMatrix4x4Uniform("shadowMapLight1ProjectionMatrix", mShadowMapBiasMatrix * shadowMapProjMatrix[1]);
    mPhongShader.setMatrix4x4Uniform("shadowMapLight2ProjectionMatrix", mShadowMapBiasMatrix * shadowMapProjMatrix[2]);
    mPhongShader.setMatrix4x4Uniform("worldToLight0CameraMatrix", worldToLightCameraMatrix[0]);
    mPhongShader.setMatrix4x4Uniform("worldToLight1CameraMatrix", worldToLightCameraMatrix[1]);
    mPhongShader.setMatrix4x4Uniform("worldToLight2CameraMatrix", worldToLightCameraMatrix[2]);
    mPhongShader.setVector3Uniform("light0PosCameraSpace", worldToCameraMatrix * mLight0.getOrigin());
    mPhongShader.setVector3Uniform("light1PosCameraSpace", worldToCameraMatrix * mLight1.getOrigin());
    mPhongShader.setVector3Uniform("light2PosCameraSpace", worldToCameraMatrix * mLight2.getOrigin());
    mPhongShader.setVector3Uniform("lightAmbientColor", Vector3(0.3f, 0.3f, 0.3f));
    mPhongShader.setVector3Uniform("light0DiffuseColor", Vector3(mLight0.getDiffuseColor().r, mLight0.getDiffuseColor().g, mLight0.getDiffuseColor().b));
    mPhongShader.setVector3Uniform("light1DiffuseColor", Vector3(mLight1.getDiffuseColor().r, mLight1.getDiffuseColor().g, mLight1.getDiffuseColor().b));
    mPhongShader.setVector3Uniform("light2DiffuseColor", Vector3(mLight2.getDiffuseColor().r, mLight2.getDiffuseColor().g, mLight2.getDiffuseColor().b));
    mPhongShader.setIntUniform("shadowMapSampler0", textureUnits[0]);
    mPhongShader.setIntUniform("shadowMapSampler1", textureUnits[1]);
    mPhongShader.setIntUniform("shadowMapSampler2", textureUnits[2]);
    mPhongShader.setIntUniform("isShadowEnabled", mIsShadowMappingEnabled);
    mPhongShader.setVector2Uniform("shadowMapDimension", Vector2(SHADOWMAP_WIDTH, SHADOWMAP_HEIGHT));
	mPhongShader.unbind();

	// Set the variables of the color shader
	mColorShader.bind();
	mColorShader.setMatrix4x4Uniform("projectionMatrix", mCamera.getProjectionMatrix());
	mColorShader.unbind();

    // Set the viewport to render the scene
    glViewport(mViewportX, mViewportY, mViewportWidth, mViewportHeight);

    //Enabling color write (previously disabled for light POV z-buffer rendering)
    glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);

    // Clear previous frame values
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // Render the objects of the scene
    renderSinglePass(mPhongShader, worldToCameraMatrix);

    // Render the contact points
    if (mIsContactPointsDisplayed) {
        renderContactPoints(mPhongShader, worldToCameraMatrix);
    }

    // Render the AABBs
    if (mIsAABBsDisplayed) {
        renderAABBs(worldToCameraMatrix);
    }

	// Is shadow mapping is enabled
	if (mIsShadowMappingEnabled) {

		for (int i = 0; i < NB_SHADOW_MAPS; i++) {
			mShadowMapTexture[i].unbind();
		}
	}

    mPhongShader.unbind();

   //drawTextureQuad();
}

// Render the scene in a single pass
void SceneDemo::renderSinglePass(openglframework::Shader& shader, const openglframework::Matrix4& worldToCameraMatrix) {
	
    if (mIsWireframeEnabled) {
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    }

    // Bind the shader
	shader.bind();

	// Render all the physics objects of the scene
	for (std::vector<PhysicsObject*>::iterator it = mPhysicsObjects.begin(); it != mPhysicsObjects.end(); ++it) {
        (*it)->render(mIsWireframeEnabled ? mColorShader : shader, worldToCameraMatrix);
	}

	// Unbind the shader
	shader.unbind();

    if (mIsWireframeEnabled) {
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    }
}

// Create the Shadow map FBO and texture
void SceneDemo::createShadowMapFBOAndTexture() {

	// For each shadow map
	for (int i = 0; i < NB_SHADOW_MAPS; i++) {

		// Create the texture for the depth values
		mShadowMapTexture[i].create(SHADOWMAP_WIDTH, SHADOWMAP_HEIGHT, GL_DEPTH_COMPONENT24, GL_DEPTH_COMPONENT,
			GL_UNSIGNED_BYTE, GL_LINEAR, GL_LINEAR, GL_CLAMP_TO_BORDER, GL_CLAMP_TO_BORDER, NULL);

		mShadowMapTexture[i].setUnit(i);

		// Make sure that texture lookups outside the texture coords range will not
		// treated as beeing in shadow
		glBindTexture(GL_TEXTURE_2D, mShadowMapTexture[i].getID());
		GLfloat border[] = { 1.0f, 0.0f, 0.0f, 0.0f };
		glTexParameterfv(GL_TEXTURE_2D, GL_TEXTURE_BORDER_COLOR, border);
		glBindTexture(GL_TEXTURE_2D, 0);

		// Create the FBO for the shadow map
		mFBOShadowMap[i].create(0, 0, false);
		mFBOShadowMap[i].bind();

		// Tell OpenGL that we won't bind a color texture with the currently binded FBO
		glDrawBuffer(GL_NONE);
		glReadBuffer(GL_NONE);

		mFBOShadowMap[i].attachTexture(GL_DEPTH_ATTACHMENT, mShadowMapTexture[i].getID());
		mFBOShadowMap[i].unbind();
	}

    mIsShadowMappingInitialized = true;
}

// Used for debugging shadow maps
void SceneDemo::createQuadVBO() {

    mVAOQuad.create();
    mVAOQuad.bind();

    static const GLfloat quadVertexData[] = {
        -1.0f, -1.0f, 0.0f,
        1.0f, -1.0f, 0.0f,
        -1.0f,  1.0f, 0.0f,
        -1.0f,  1.0f, 0.0f,
        1.0f, -1.0f, 0.0f,
        1.0f,  1.0f, 0.0f,
    };

    mVBOQuad.create();
    mVBOQuad.bind();
    mVBOQuad.copyDataIntoVBO(sizeof(quadVertexData), quadVertexData, GL_STATIC_DRAW);
    mVBOQuad.unbind();

    mVAOQuad.unbind();
}

void SceneDemo::drawTextureQuad() {

    glViewport(mViewportX, mViewportY, mViewportWidth, mViewportHeight);
    glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);

    // Clear previous frame values
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);


	const int SHADOW_MAP_TEXTURE_TO_DRAW = 0;
    const GLuint textureUnit = SHADOW_MAP_TEXTURE_TO_DRAW;

    mVAOQuad.bind();
    mQuadShader.bind();
    mShadowMapTexture[SHADOW_MAP_TEXTURE_TO_DRAW].bind();
    mQuadShader.setIntUniform("textureSampler", textureUnit);
    mVBOQuad.bind();

    GLint vertexPositionLoc = mQuadShader.getAttribLocation("vertexPosition");
    glEnableVertexAttribArray(vertexPositionLoc);

    glVertexAttribPointer(
            vertexPositionLoc,                  // attribute 0. No particular reason for 0, but must match the layout in the shader.
            3,                  // size
            GL_FLOAT,           // type
            GL_FALSE,           // normalized?
            0,                  // stride
            (void*)0            // array buffer offset
    );

    // Draw the triangles !
    glDrawArrays(GL_TRIANGLES, 0, 6); // 2*3 indices starting at 0 -> 2 triangles

    glDisableVertexAttribArray(vertexPositionLoc);

    mVBOQuad.unbind();
    mShadowMapTexture[SHADOW_MAP_TEXTURE_TO_DRAW].unbind();
    mQuadShader.unbind();
    mVAOQuad.unbind();
}

// Gather and create contact points
void SceneDemo::updateContactPoints() {

    // Remove the previous contact points
    removeAllVisualContactPoints();

    if (mIsContactPointsDisplayed) {

        // For each contact point
        std::vector<SceneContactPoint>::const_iterator it;
        for (it = mContactPoints.begin(); it != mContactPoints.end(); ++it) {

            // Create a visual contact point for rendering
            VisualContactPoint* point = new VisualContactPoint(it->point, mMeshFolderPath, it->point + it->normal, it->color);
            mVisualContactPoints.push_back(point);
        }
    }
}

// Render the contact points
void SceneDemo::renderContactPoints(openglframework::Shader& shader, const openglframework::Matrix4& worldToCameraMatrix) {

    // Render all the contact points
    for (std::vector<VisualContactPoint*>::iterator it = mVisualContactPoints.begin();
         it != mVisualContactPoints.end(); ++it) {
        (*it)->render(mColorShader, worldToCameraMatrix);
    }
}

// Render the AABBs
void SceneDemo::renderAABBs(const openglframework::Matrix4& worldToCameraMatrix) {

    // For each physics object of the scene
    for (std::vector<PhysicsObject*>::iterator it = mPhysicsObjects.begin(); it != mPhysicsObjects.end(); ++it) {

       // For each collider of the object
       for (uint i=0; i < (*it)->getCollisionBody()->getNbColliders(); i++) {

           rp3d::Collider* collider = (*it)->getCollisionBody()->getCollider(i);

           // Get the broad-phase AABB corresponding to the collider
           rp3d::AABB aabb = mPhysicsWorld->getWorldAABB(collider);

           openglframework::Vector3 aabbCenter(aabb.getCenter().x, aabb.getCenter().y, aabb.getCenter().z);
           openglframework::Vector3 aabbMin(aabb.getMin().x, aabb.getMin().y, aabb.getMin().z);
           openglframework::Vector3 aabbMax(aabb.getMax().x, aabb.getMax().y, aabb.getMax().z);

           // Render the AABB
           AABB::render(aabbCenter, aabbMax - aabbMin, Color::green(), mColorShader, worldToCameraMatrix);
       }
    }
}

void SceneDemo::removeAllVisualContactPoints() {

    // Destroy all the visual contact points
    for (std::vector<VisualContactPoint*>::iterator it = mVisualContactPoints.begin();
         it != mVisualContactPoints.end(); ++it) {
        delete (*it);
    }
    mVisualContactPoints.clear();
}

// Update the engine settings
void SceneDemo::updateEngineSettings() {

    if (mIsPhysicsWorldSimulated) {

        // Update the physics engine parameters
        mPhysicsWorld->setIsGratityEnabled(mEngineSettings.isGravityEnabled);
        rp3d::Vector3 gravity(mEngineSettings.gravity.x, mEngineSettings.gravity.y,
                         mEngineSettings.gravity.z);
        mPhysicsWorld->setGravity(gravity);
        mPhysicsWorld->enableSleeping(mEngineSettings.isSleepingEnabled);
        mPhysicsWorld->setSleepLinearVelocity(mEngineSettings.sleepLinearVelocity);
        mPhysicsWorld->setSleepAngularVelocity(mEngineSettings.sleepAngularVelocity);
        mPhysicsWorld->setNbIterationsPositionSolver(mEngineSettings.nbPositionSolverIterations);
        mPhysicsWorld->setNbIterationsVelocitySolver(mEngineSettings.nbVelocitySolverIterations);
        mPhysicsWorld->setTimeBeforeSleep(mEngineSettings.timeBeforeSleep);
    }
}
