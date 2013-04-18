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


// Libraries
#include "Camera.h"
#include "definitions.h"
#include <cmath>

// Namespaces
using namespace openglframework;

// Constructor
Camera::Camera() : Object3D() {

    // Set default values
    mFieldOfView = 45.0f;
    mSceneRadius = 1.0f;
    mNearPlane = 0.1f;
    mFarPlane = 10.0f;
    mWidth = 1;
    mHeight = 1;

    // Update the projection matrix
    updateProjectionMatrix();
}

// Destructor
Camera::~Camera() {

}

// Update the projection matrix
void Camera::updateProjectionMatrix() {

    // Compute the aspect ratio
    float aspect = float(mWidth) / float(mHeight);

    float top = mNearPlane * tan((mFieldOfView / 2.0f) * (float(PI) / 180.0f));
    float bottom = -top;
    float left = bottom * aspect;
    float right = top * aspect;

    float fx = 2.0f * mNearPlane / (right - left);
    float fy = 2.0f * mNearPlane / (top - bottom);
    float fz = -(mFarPlane + mNearPlane) / (mFarPlane - mNearPlane);
    float fw = -2.0f * mFarPlane * mNearPlane / (mFarPlane - mNearPlane);

    // Recompute the projection matrix
    mProjectionMatrix = Matrix4(fx, 0, 0, 0,
                                0, fy, 0, 0,
                                0, 0, fz, fw,
                                0, 0, -1, 0);
}

// Translate the camera go a given point using the dx, dy fraction
void Camera::translateCamera(float dx, float dy, const Vector3& worldPoint) {

    // Transform the world point into camera coordinates
    Vector3 pointCamera = mTransformMatrix.getInverse() * worldPoint;

    // Get the depth
    float z = -pointCamera.z;

    // Find the scaling of dx and dy from windows coordinates to near plane coordinates
    // and from there to camera coordinates at the object's depth
    float aspect = float(mWidth) / float(mHeight);
    float top = mNearPlane * tan(mFieldOfView * PI / 360.0f);
    float right = top * aspect;

    // Translate the camera
    translateLocal(Vector3(2.0f * dx * right / mNearPlane * z,
                           -2.0f * dy * top / mNearPlane * z,
                           0.0f));
}
