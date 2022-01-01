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
#define _USE_MATH_DEFINES
#include <cmath>
#include "Scene.h"
#include <GLFW/glfw3.h>

using namespace openglframework;

// Constructor
Scene::Scene(const std::string& name, EngineSettings& engineSettings, bool isShadowMappingEnabled)
      : mName(name), mEngineSettings(engineSettings), mLastMouseX(0), mLastMouseY(0), mInterpolationFactor(0.0f),
        mCurrentCameraVerticalAngle(0.0), mViewportX(0), mViewportY(0),
        mViewportWidth(0), mViewportHeight(0), mIsShadowMappingEnabled(isShadowMappingEnabled),
        mAreContactPointsDisplayed(true), mAreContactNormalsDisplayed(false), mAreBroadPhaseAABBsDisplayed(false),
        mAreCollidersAABBsDisplayed(false), mAreCollisionShapesDisplayed(false), mIsWireframeEnabled(false),
        mInitZoom(2.0f), mIsCameraRotationAnimationEnabled(false) {

}

// Destructor
Scene::~Scene() {

}

// Set the scene position (where the camera needs to look at)
void Scene::setScenePosition(const openglframework::Vector3& position, float sceneRadius) {

    // Set the position and radius of the scene
    mCenterScene = position;
    mCamera.setSceneRadius(sceneRadius);

    // Reset the camera position and zoom in order to view all the scene
    resetCameraToViewAll();
}

// Set the camera so that we can view the whole scene
void Scene::resetCameraToViewAll() {

    // Move the camera to the origin of the scene
    mCamera.translateWorld(-mCamera.getOrigin());

    // Move the camera
    mCamera.translateWorld(mCenterScene);

    // Set the zoom of the camera so that the scene center is
    // in negative view direction of the camera
    mCamera.setZoom(mInitZoom);
}

// Map the mouse x,y coordinates to a point on a sphere
bool Scene::mapMouseCoordinatesToSphere(double xMouse, double yMouse,
                                        Vector3& spherePoint) const {

    if ((xMouse >= 0) && (xMouse <= mWindowWidth) && (yMouse >= 0) && (yMouse <= mWindowHeight)) {
        float x = float(xMouse - 0.5f * mWindowWidth) / float(mWindowWidth);
        float y = float(0.5f * mWindowHeight - yMouse) / float(mWindowHeight);
        float sinx = std::sin(PI * x * 0.5f);
        float siny = std::sin(PI * y * 0.5f);
        float sinx2siny2 = sinx * sinx + siny * siny;

        // Compute the point on the sphere
        spherePoint.x = sinx;
        spherePoint.y = siny;
        spherePoint.z = (sinx2siny2 < 1.0f) ? std::sqrt(1.0f - sinx2siny2) : 0.0f;

        return true;
    }

    return false;
}

// Called when a mouse button event occurs
bool Scene::mouseButtonEvent(int /*button*/, bool down, int /*mods*/, double mousePosX, double mousePosY) {

    // If the mouse button is pressed
    if (down) {
        mLastMouseX = mousePosX;
        mLastMouseY = mousePosY;
    }

    return true;
}

// Called when a mouse motion event occurs
bool Scene::mouseMotionEvent(double xMouse, double yMouse, int leftButtonState,
                             int rightButtonState, int middleButtonState,
                             int altKeyState) {

    // Zoom
    if (leftButtonState == GLFW_PRESS && altKeyState == GLFW_PRESS) {

        float dy = static_cast<float>(yMouse - mLastMouseY);
        float h = static_cast<float>(mWindowHeight);

        // Zoom the camera
        zoom(-dy / h);
    }
    // Translation
    else if (middleButtonState == GLFW_PRESS || rightButtonState == GLFW_PRESS ||
             (leftButtonState == GLFW_PRESS && altKeyState == GLFW_PRESS)) {
        translate(xMouse, yMouse);
    }
    // Rotation
    else if (leftButtonState == GLFW_PRESS) {
        rotate(xMouse, yMouse);
    }

    // Remember the mouse position
    mLastMouseX = xMouse;
    mLastMouseY = yMouse;

    return true;
}

// Called when a scrolling event occurs
bool Scene::scrollingEvent(float /*xAxis*/, float yAxis, float scrollSensitivy) {
    zoom(yAxis * scrollSensitivy);

    return true;
}

// Zoom the camera
void Scene::zoom(float zoomDiff) {

    // Zoom the camera
    mCamera.setZoom(zoomDiff);
}

// Translate the camera
void Scene::translate(int xMouse, int yMouse) {
   float dx = static_cast<float>(xMouse - mLastMouseX);
   float dy = static_cast<float>(yMouse - mLastMouseY);

   // Translate the camera
   mCamera.translateCamera(-dx / float(mCamera.getWidth()),
                           -dy / float(mCamera.getHeight()), mCenterScene);
}

// Rotate the camera
void Scene::rotate(int xMouse, int yMouse) {

    const double deltaXMouse = mLastMouseX - xMouse;
    const double deltaYMouse = mLastMouseY - yMouse;

    double deltaHorizRotationAngle = deltaXMouse / mWindowWidth * MOUSE_CAMERA_ROTATION_SCALING_FACTOR * M_PI;
    double deltaVertRotationAngle = deltaYMouse / mWindowHeight * MOUSE_CAMERA_ROTATION_SCALING_FACTOR * M_PI;

    const double newVerticalAngle = mCurrentCameraVerticalAngle + deltaVertRotationAngle;

    // Limit Vertical angle
    constexpr double piOver2 = M_PI * 0.5f;
    if (newVerticalAngle > piOver2 || newVerticalAngle < -piOver2) {
        deltaVertRotationAngle = 0;
    }

    Vector3 localVertAxis = mCamera.getTransformMatrix().getUpperLeft3x3Matrix().getInverse() * Vector3(0, 1, 0);
    mCamera.rotateAroundLocalPoint(Vector3(1, 0, 0), deltaVertRotationAngle, mCenterScene);
    mCamera.rotateAroundLocalPoint(localVertAxis, deltaHorizRotationAngle, mCenterScene);

    mCurrentCameraVerticalAngle += deltaVertRotationAngle;
}
