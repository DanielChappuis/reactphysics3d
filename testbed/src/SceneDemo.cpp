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
#include <reactphysics3d/constraint/ContactPoint.h>
#include <reactphysics3d/collision/ContactManifold.h>

using namespace openglframework;

int SceneDemo::shadowMapTextureLevel = 0;
//openglframework::Color SceneDemo::mObjectColorDemo = Color(0.76f, 0.67f, 0.47f, 1.0f);
openglframework::Color SceneDemo::mObjectColorDemo = Color(0.0f, 0.68f, 0.99f, 1.0f);
openglframework::Color SceneDemo::mFloorColorDemo = Color(0.7f, 0.7f, 0.7f, 1.0f);
openglframework::Color SceneDemo::mSleepingColorDemo = Color(1.0f, 0.0f, 0.0f, 1.0f);
openglframework::Color SceneDemo::mSelectedObjectColorDemo = Color(0.09f, 0.88f, 0.09f, 1.0f);

// Constructor
SceneDemo::SceneDemo(const std::string& name, EngineSettings& settings, reactphysics3d::PhysicsCommon& physicsCommon, bool isPhysicsWorldSimulated, bool isShadowMappingEnabled)
          : Scene(name, settings, isShadowMappingEnabled), mBackgroundColor(0.75, 0.75, 0.75, 1),
                     mIsShadowMappingInitialized(false),
                     mDepthShader("shaders/depth.vert", "shaders/depth.frag"),
                     mPhongShader("shaders/phong.vert", "shaders/phong.frag"),
					 mColorShader("shaders/color.vert", "shaders/color.frag"),
                     mQuadShader("shaders/quad.vert", "shaders/quad.frag"),
                     mVBOQuad(GL_ARRAY_BUFFER), mDebugVBOLinesVertices(GL_ARRAY_BUFFER), mDebugVBOTrianglesVertices(GL_ARRAY_BUFFER),
                     mMeshFolderPath("meshes/"), mPhysicsCommon(physicsCommon), mPhysicsWorld(nullptr), mIsPhysicsWorldSimulated(isPhysicsWorldSimulated),
                     mIsMovingBody(false), mMovingBody(nullptr), mCameraRotationAngle(0) {

    shadowMapTextureLevel++;

    // Move the lights
    float lightsRadius = 80.0f;
    float lightsHeight = 50.0f;
    mLight0.translateWorld(Vector3(0.4f * lightsRadius, 0.6 * lightsHeight, 0.4f * lightsRadius));
    mLight1.translateWorld(Vector3(-0.4 * lightsRadius, 0.6 * lightsHeight, 0.4 * lightsRadius));
    mLight2.translateWorld(Vector3(0, 0.6 * lightsHeight, -0.4f * lightsRadius));

	// Set the lights colors
    float lightIntensity = 0.5;
    Color lightColor(lightIntensity * 1.0, lightIntensity * 1.0f, lightIntensity * 1.0f, 1.0f);
    float lightIntensity2 = 0.1;
    Color lightColor2(lightIntensity2 * 1.0, lightIntensity2 * 1.0f, lightIntensity2 * 1.0f, 1.0f);
    mLight0.setDiffuseColor(lightColor);
    mLight1.setDiffuseColor(lightColor);
    mLight2.setDiffuseColor(lightColor2);

    mShadowMapLightCameras[0].translateWorld(mLight0.getOrigin());
    mShadowMapLightCameras[0].rotateLocal(Vector3(1, 0, 0), -PI / 4.0f);
    mShadowMapLightCameras[0].rotateLocal(Vector3(0, 1, 0), PI / 4.0f);

	mShadowMapLightCameras[1].translateWorld(mLight1.getOrigin());
	mShadowMapLightCameras[1].rotateLocal(Vector3(1, 0, 0), -PI/4.0f);
    mShadowMapLightCameras[1].rotateLocal(Vector3(0, 1, 0), -PI / 4.0f);

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

    createDebugVBO();

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

    mPhysicsWorld->getDebugRenderer().setIsDebugItemDisplayed(rp3d::DebugRenderer::DebugItem::CONTACT_POINT, mAreContactPointsDisplayed);
    mPhysicsWorld->getDebugRenderer().setIsDebugItemDisplayed(rp3d::DebugRenderer::DebugItem::CONTACT_NORMAL, mAreContactNormalsDisplayed);
    mPhysicsWorld->getDebugRenderer().setIsDebugItemDisplayed(rp3d::DebugRenderer::DebugItem::COLLIDER_BROADPHASE_AABB, mAreBroadPhaseAABBsDisplayed);
    mPhysicsWorld->getDebugRenderer().setIsDebugItemDisplayed(rp3d::DebugRenderer::DebugItem::COLLIDER_AABB, mAreCollidersAABBsDisplayed);
    mPhysicsWorld->getDebugRenderer().setIsDebugItemDisplayed(rp3d::DebugRenderer::DebugItem::COLLISION_SHAPE, mAreCollisionShapesDisplayed);

    // Update the contact points
    updateSnapshotContactPoints();

	// Update the position and orientation of the physics objects
	for (std::vector<PhysicsObject*>::iterator it = mPhysicsObjects.begin(); it != mPhysicsObjects.end(); ++it) {

		// Update the transform used for the rendering
		(*it)->updateTransform(mInterpolationFactor);
	}

    if (mIsCameraRotationAnimationEnabled) {
        rotateCameraAnimation();
    }
}

void SceneDemo::rotateCameraAnimation() {

   const float angle  = 0.12f * (PI / 180.0);
   mCamera.rotateAroundWorldPoint(Vector3(0, 1, 0), angle, mCenterScene);
}

// Update the physics world (take a simulation step)
// Can be called several times per frame
void SceneDemo::updatePhysics() {

    // Clear contacts points
    mSnapshotsContactPoints.clear();

    if (mIsPhysicsWorldSimulated) {

        // Take a simulation step
        mPhysicsWorld->update(mEngineSettings.timeStep.count());
    }
}

// Render the scene (in multiple passes for shadow mapping)
void SceneDemo::render() {

    // Update the VBO for the debug infos
    updateDebugVBO();

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);

    glClearColor(mBackgroundColor.r, mBackgroundColor.g, mBackgroundColor.b, mBackgroundColor.a);

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
    mPhongShader.setMatrix4x4Uniform("worldToLight0CameraMatrix", worldToLightCameraMatrix[0]);
    mPhongShader.setMatrix4x4Uniform("worldToLight1CameraMatrix", worldToLightCameraMatrix[1]);
    mPhongShader.setVector3Uniform("light0PosCameraSpace", worldToCameraMatrix * mLight0.getOrigin());
    mPhongShader.setVector3Uniform("light1PosCameraSpace", worldToCameraMatrix * mLight1.getOrigin());
    mPhongShader.setVector3Uniform("light2PosCameraSpace", worldToCameraMatrix * mLight2.getOrigin());
    mPhongShader.setVector3Uniform("lightAmbientColor", Vector3(0.1f, 0.1f, 0.1f));
    mPhongShader.setVector3Uniform("light0DiffuseColor", Vector3(mLight0.getDiffuseColor().r, mLight0.getDiffuseColor().g, mLight0.getDiffuseColor().b));
    mPhongShader.setVector3Uniform("light1DiffuseColor", Vector3(mLight1.getDiffuseColor().r, mLight1.getDiffuseColor().g, mLight1.getDiffuseColor().b));
    mPhongShader.setVector3Uniform("light2DiffuseColor", Vector3(mLight2.getDiffuseColor().r, mLight2.getDiffuseColor().g, mLight2.getDiffuseColor().b));
    mPhongShader.setIntUniform("shadowMapSampler0", textureUnits[0]);
    mPhongShader.setIntUniform("shadowMapSampler1", textureUnits[1]);
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

    // Render the snapshots contact points
    renderSnapshotsContactPoints(mPhongShader, worldToCameraMatrix);

    // Render the debug infos
    if (mPhysicsWorld->getIsDebugRenderingEnabled()) {
        renderDebugInfos(mColorShader, worldToCameraMatrix);
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

// Create a the VAO and VBOs to render the debug infos
void SceneDemo::createDebugVBO() {

    // ----- Lines ----- //

    // Create the VBO for the vertices data
    mDebugVBOLinesVertices.create();

    // Create the VAO for both VBOs
    mDebugLinesVAO.create();
    mDebugLinesVAO.bind();

    // Bind the VBO of vertices
    mDebugVBOLinesVertices.bind();

    // Unbind the VAO
    mDebugLinesVAO.unbind();

    mDebugVBOLinesVertices.unbind();

    // ----- Triangles ----- //

    // Create the VBO for the vertices data
    mDebugVBOTrianglesVertices.create();

    // Create the VAO for both VBOs
    mDebugTrianglesVAO.create();
    mDebugTrianglesVAO.bind();

    // Bind the VBO of vertices
    mDebugVBOTrianglesVertices.bind();

    // Unbind the VAO
    mDebugTrianglesVAO.unbind();

    mDebugVBOTrianglesVertices.unbind();
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

// Gather and create snapshots contact points
void SceneDemo::updateSnapshotContactPoints() {

    // Remove the previous contact points
    removeAllVisualContactPoints();

    // For each contact point
    std::vector<SceneContactPoint>::const_iterator it;
    for (it = mSnapshotsContactPoints.begin(); it != mSnapshotsContactPoints.end(); ++it) {

        // Create a visual contact point for rendering
        VisualContactPoint* point = new VisualContactPoint(it->point, it->point + it->normal, it->color);
        mVisualContactPoints.push_back(point);
    }
}

// Render the contact points
void SceneDemo::renderSnapshotsContactPoints(openglframework::Shader& /*shader*/, const openglframework::Matrix4& worldToCameraMatrix) {

    // Render all the contact points
    for (std::vector<VisualContactPoint*>::iterator it = mVisualContactPoints.begin();
         it != mVisualContactPoints.end(); ++it) {
        (*it)->render(mColorShader, worldToCameraMatrix);
    }
}

// Update VBO with vertices and indices of debug info
void SceneDemo::updateDebugVBO() {

    rp3d::DebugRenderer& debugRenderer = mPhysicsWorld->getDebugRenderer();

    if (mPhysicsWorld->getIsDebugRenderingEnabled()) {

        // ----- Lines ---- //

        const uint nbLines = debugRenderer.getNbLines();

        if (nbLines > 0) {

            // Vertices
            mDebugVBOLinesVertices.bind();
            GLsizei sizeVertices = static_cast<GLsizei>(nbLines * sizeof(rp3d::DebugRenderer::DebugLine));
            mDebugVBOLinesVertices.copyDataIntoVBO(sizeVertices, debugRenderer.getLinesArray(), GL_STREAM_DRAW);
            mDebugVBOLinesVertices.unbind();
        }

        // ----- Triangles ---- //

        const uint nbTriangles = debugRenderer.getNbTriangles();

        if (nbTriangles > 0) {

            // Vertices
            mDebugVBOTrianglesVertices.bind();
            GLsizei sizeVertices = static_cast<GLsizei>(nbTriangles * sizeof(rp3d::DebugRenderer::DebugTriangle));
            mDebugVBOTrianglesVertices.copyDataIntoVBO(sizeVertices, debugRenderer.getTrianglesArray(), GL_STREAM_DRAW);
            mDebugVBOTrianglesVertices.unbind();
        }
    }
}

// Render Debug Infos
void SceneDemo::renderDebugInfos(openglframework::Shader& shader, const openglframework::Matrix4& worldToCameraMatrix) {

    rp3d::DebugRenderer& debugRenderer = mPhysicsWorld->getDebugRenderer();

    // Render in wireframe mode
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

    // Bind the shader
    shader.bind();

    // Set the normal matrix (inverse transpose of the 3x3 upper-left sub matrix of the
    // model-view matrix)
    const openglframework::Matrix4 localToCameraMatrix = worldToCameraMatrix;
    const openglframework::Matrix3 normalMatrix = localToCameraMatrix.getUpperLeft3x3Matrix().getInverse().getTranspose();
    shader.setMatrix3x3Uniform("normalMatrix", normalMatrix, false);

    // Set the model to camera matrix
    shader.setMatrix4x4Uniform("localToWorldMatrix", openglframework::Matrix4::identity());
    shader.setMatrix4x4Uniform("worldToCameraMatrix", worldToCameraMatrix);

    shader.setIntUniform("isGlobalVertexColorEnabled", 0, false);

    // Get the location of shader attribute variables
    GLint vertexPositionLoc = shader.getAttribLocation("vertexPosition");
    GLint vertexColorLoc = shader.getAttribLocation("vertexColor");

    // Lines
    if (debugRenderer.getNbLines() > 0) {

        // Bind the VAO
        mDebugLinesVAO.bind();

        mDebugVBOLinesVertices.bind();

        glEnableVertexAttribArray(vertexPositionLoc);
        glVertexAttribPointer(vertexPositionLoc, 3, GL_FLOAT, GL_FALSE, sizeof(rp3d::Vector3) + sizeof(rp3d::uint32), (char*)nullptr);

        glEnableVertexAttribArray(vertexColorLoc);
        glVertexAttribIPointer(vertexColorLoc, 3, GL_UNSIGNED_INT, sizeof(rp3d::Vector3) + sizeof(rp3d::uint32), (void*)sizeof(rp3d::Vector3));

        // Draw the lines geometry
        glDrawArrays(GL_LINES, 0, debugRenderer.getNbLines() * 2);

        glDisableVertexAttribArray(vertexPositionLoc);
        glDisableVertexAttribArray(vertexColorLoc);

        mDebugVBOLinesVertices.unbind();

        // Unbind the VAO
        mDebugLinesVAO.unbind();
    }

    // Triangles
    if (debugRenderer.getNbTriangles() > 0) {

        // Bind the VAO
        mDebugTrianglesVAO.bind();

        mDebugVBOTrianglesVertices.bind();

        glEnableVertexAttribArray(vertexPositionLoc);
        glVertexAttribPointer(vertexPositionLoc, 3, GL_FLOAT, GL_FALSE, sizeof(rp3d::Vector3) + sizeof(rp3d::uint32), (char*)nullptr);

        glEnableVertexAttribArray(vertexColorLoc);
        glVertexAttribIPointer(vertexColorLoc, 3, GL_UNSIGNED_INT, sizeof(rp3d::Vector3) + sizeof(rp3d::uint32), (void*)sizeof(rp3d::Vector3));

        // Draw the triangles geometry
        glDrawArrays(GL_TRIANGLES, 0, debugRenderer.getNbTriangles() * 3);

        glDisableVertexAttribArray(vertexPositionLoc);
        glDisableVertexAttribArray(vertexColorLoc);

        mDebugVBOTrianglesVertices.unbind();

        // Unbind the VAO
        mDebugTrianglesVAO.unbind();
    }

    // Unbind the shader
    shader.unbind();

    // Disable wireframe mode
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
}

void SceneDemo::removeAllVisualContactPoints() {

    // Destroy all the visual contact points
    for (std::vector<VisualContactPoint*>::iterator it = mVisualContactPoints.begin();
         it != mVisualContactPoints.end(); ++it) {
        delete (*it);
    }
    mVisualContactPoints.clear();
}

// Called when the user is moving a body with the mouse
void SceneDemo::moveBodyWithMouse(double mousePosX, double mousePosY) {

    if (!mIsMovingBody) {

        // Find the body and the position of the mouse on that body (with raycasting)
        openglframework::Vector4 screenPoint1((mousePosX / mWindowWidth) * 2.0 - 1.0, ((mWindowHeight - mousePosY) / mWindowHeight) * 2.0 - 1.0, -1, 1);
        openglframework::Vector4 screenPoint2((mousePosX / mWindowWidth) * 2.0 - 1.0, ((mWindowHeight - mousePosY) / mWindowHeight) * 2.0 - 1.0, 1, 1);
        openglframework::Vector4 worldP1 = (mCamera.getTransformMatrix() * mCamera.getProjectionMatrix().getInverse()) * screenPoint1;
        openglframework::Vector4 worldP2 = (mCamera.getTransformMatrix() * mCamera.getProjectionMatrix().getInverse()) * screenPoint2;
        openglframework::Vector3 cameraPos = mCamera.getOrigin();
        rp3d::Vector3 worldPoint1(worldP1.x, worldP1.y, worldP1.z);
        rp3d::Vector3 worldPoint2(worldP2.x, worldP2.y, worldP2.z);
        rp3d::Ray ray(worldPoint1, worldPoint2);
        mPhysicsWorld->raycast(ray, this);
    }

    if (mMovingBody != nullptr) {
        openglframework::Vector4 previousScreenPos(mLastMouseX / mWindowWidth, (mWindowHeight - mLastMouseY) / mWindowHeight, 0, 0);
        openglframework::Vector4 currentScreenPos(mousePosX / mWindowWidth, (mWindowHeight - mousePosY) / mWindowHeight, 0, 0);
        openglframework::Vector4 forceScreen = currentScreenPos - previousScreenPos;
        openglframework::Vector4 f = mCamera.getTransformMatrix() * forceScreen * MOUSE_MOVE_BODY_FORCE;
        rp3d::Vector3 force(f.x, f.y, f.z);
        mMovingBody->applyWorldForceAtLocalPosition(force, mMovingBodyLocalPoint);
    }

    mLastMouseX = mousePosX;
    mLastMouseY = mousePosY;
    mIsMovingBody = true;
}

// Called when a mouse button event occurs
bool SceneDemo::mouseButtonEvent(int button, bool down, int mods, double mousePosX, double mousePosY) {

    // Left mouse click with CTRL key pressed on keyboard (moving a body)
    if (down && (mods & GLFW_MOD_CONTROL)) {

        moveBodyWithMouse(mousePosX, mousePosY);
        return true;
    }

    mIsMovingBody = false;
    mMovingBody = nullptr;

    return Scene::mouseButtonEvent(button, down, mods, mousePosX, mousePosY);
}

// Called when a mouse motion event occurs
bool SceneDemo::mouseMotionEvent(double xMouse, double yMouse, int leftButtonState, int rightButtonState, int middleButtonState, int altKeyState) {

    if (mIsMovingBody) {
        moveBodyWithMouse(xMouse, yMouse);
        return true;
    }

    return Scene::mouseMotionEvent(xMouse, yMouse, leftButtonState, rightButtonState, middleButtonState, altKeyState);
}

// Called when a raycast hit occurs (used to move a body with the mouse)
rp3d::decimal SceneDemo::notifyRaycastHit(const rp3d::RaycastInfo& raycastInfo) {

    rp3d::RigidBody* body = dynamic_cast<rp3d::RigidBody*>(raycastInfo.body);
    mMovingBody = body;
    const rp3d::Transform localToWorldTransform = raycastInfo.collider->getLocalToWorldTransform();
    mMovingBodyLocalPoint = localToWorldTransform.getInverse() * raycastInfo.worldPoint;

    return raycastInfo.hitFraction;
}

// Update the engine settings
void SceneDemo::updateEngineSettings() {

    // Update the physics engine parameters
    mPhysicsWorld->setIsGravityEnabled(mEngineSettings.isGravityEnabled);
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
