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
#include "GlutViewer.h"
#include <string>

// Namespaces
using namespace openglframework;
using namespace std;

// Constructor
GlutViewer::GlutViewer() {

    // Initialize the state of the mouse buttons
    for (int i=0; i<10; i++) {
        mIsButtonDown[i] = false;
    }
}

// Destructor
GlutViewer::~GlutViewer() {

}

// Initialize the viewer
bool GlutViewer::init(int argc, char** argv, const string& windowsTitle,
                      const Vector2& windowsSize, const Vector2& windowsPosition,
                      bool isMultisamplingActive) {

    // Initialize the GLUT library
    bool outputValue = initGLUT(argc, argv, windowsTitle, windowsSize,
                                windowsPosition, isMultisamplingActive);

    // Active the multi-sampling by default
    if (isMultisamplingActive) {
        activateMultiSampling(true);
    }

    return outputValue;
}

// Initialize the GLUT library
bool GlutViewer::initGLUT(int argc, char** argv, const string& windowsTitle,
                        const Vector2& windowsSize, const Vector2& windowsPosition,
                        bool isMultisamplingActive) {

    // Initialize GLUT
    glutInit(&argc, argv);
    uint modeWithoutMultiSampling = GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH;
    uint modeWithMultiSampling = GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH | GL_MULTISAMPLE;
    uint displayMode = isMultisamplingActive ? modeWithMultiSampling : modeWithoutMultiSampling;
    glutInitDisplayMode(displayMode);

    // Initialize the size of the GLUT windows
    glutInitWindowSize(static_cast<int>(windowsSize.x),
                       static_cast<int>(windowsSize.y));

    // Initialize the position of the GLUT windows
    glutInitWindowPosition(static_cast<int>(windowsPosition.x),
                           static_cast<int>(windowsPosition.y));

    // Create the GLUT windows
    glutCreateWindow(windowsTitle.c_str());

    // Initialize the GLEW library
    GLenum error = glewInit();
    if (error != GLEW_OK) {

        // Problem: glewInit failed, something is wrong
        cerr << "GLEW Error : " << glewGetErrorString(error) << std::endl;
        assert(false);
        return false;
    }

    return true;
}

// Set the camera so that we can view the whole scene
void GlutViewer::resetCameraToViewAll() {

    // Move the camera to the origin of the scene
    mCamera.translateWorld(-mCamera.getOrigin());

    // Move the camera to the center of the scene
    mCamera.translateWorld(mCenterScene);

    // Set the zoom of the camera so that the scene center is
    // in negative view direction of the camera
    mCamera.setZoom(1.0);
}

// Called when a GLUT mouse button event occurs
void GlutViewer::mouseButtonEvent(int button, int state, int x, int y) {

    // If the mouse button is pressed
    if (state == GLUT_DOWN) {
        mLastMouseX = x;
        mLastMouseY = y;
        mIsLastPointOnSphereValid = mapMouseCoordinatesToSphere(x, y, mLastPointOnSphere);
        mIsButtonDown[button] = true;
    }
    else {  // If the mouse button is released
        mIsLastPointOnSphereValid = false;
        mIsButtonDown[button] = false;

        // If it is a mouse wheel click event
        if (button == 3) {
            zoom(0, (int) (y - 0.05f * mCamera.getWidth()));
        }
        else if (button == 4) {
            zoom(0, (int) (y + 0.05f * mCamera.getHeight()));
        }
    }

    mModifiers = glutGetModifiers();

    // Notify GLUT to redisplay
    glutPostRedisplay();
}

// Called when a GLUT mouse motion event occurs
void GlutViewer::mouseMotionEvent(int xMouse, int yMouse) {

    // Zoom
    if ((mIsButtonDown[GLUT_LEFT_BUTTON] && mIsButtonDown[GLUT_MIDDLE_BUTTON]) ||
        (mIsButtonDown[GLUT_LEFT_BUTTON] &&  mModifiers == GLUT_ACTIVE_ALT)) {
        zoom(xMouse, yMouse);
    }
    // Translation
    else if (mIsButtonDown[GLUT_MIDDLE_BUTTON] || mIsButtonDown[GLUT_RIGHT_BUTTON] ||
             (mIsButtonDown[GLUT_LEFT_BUTTON] && (mModifiers == GLUT_ACTIVE_ALT))) {
        translate(xMouse, yMouse);
    }
    // Rotation
    else if (mIsButtonDown[GLUT_LEFT_BUTTON]) {
        rotate(xMouse, yMouse);
    }

    // Remember the mouse position
    mLastMouseX = xMouse;
    mLastMouseY = yMouse;
    mIsLastPointOnSphereValid = mapMouseCoordinatesToSphere(xMouse, yMouse, mLastPointOnSphere);

    // Notify GLUT to redisplay
    glutPostRedisplay();
}

// Map the mouse x,y coordinates to a point on a sphere
bool GlutViewer::mapMouseCoordinatesToSphere(int xMouse, int yMouse, Vector3& spherePoint) const {

    int width = mCamera.getWidth();
    int height = mCamera.getHeight();

    if ((xMouse >= 0) && (xMouse <= width) && (yMouse >= 0) && (yMouse <= height)) {
        float x = float(xMouse - 0.5f * width) / float(width);
        float y = float(0.5f * height - yMouse) / float(height);
        float sinx = sin(PI * x * 0.5f);
        float siny = sin(PI * y * 0.5f);
        float sinx2siny2 = sinx * sinx + siny * siny;

        // Compute the point on the sphere
        spherePoint.x = sinx;
        spherePoint.y = siny;
        spherePoint.z = (sinx2siny2 < 1.0) ? sqrt(1.0f - sinx2siny2) : 0.0f;

        return true;
    }

    return false;
}

// Zoom the camera
void GlutViewer::zoom(int xMouse, int yMouse) {
    float dy = static_cast<float>(yMouse - mLastMouseY);
    float h = static_cast<float>(mCamera.getHeight());

    // Zoom the camera
    mCamera.setZoom(-dy / h);
}

// Translate the camera
void GlutViewer::translate(int xMouse, int yMouse) {
   float dx = static_cast<float>(xMouse - mLastMouseX);
   float dy = static_cast<float>(yMouse - mLastMouseY);

   // Translate the camera
   mCamera.translateCamera(-dx / float(mCamera.getWidth()),
                           -dy / float(mCamera.getHeight()), mCenterScene);
}

// Rotate the camera
void GlutViewer::rotate(int xMouse, int yMouse) {

    if (mIsLastPointOnSphereValid) {

        Vector3 newPoint3D;
        bool isNewPointOK = mapMouseCoordinatesToSphere(xMouse, yMouse, newPoint3D);

        if (isNewPointOK) {
            Vector3 axis = mLastPointOnSphere.cross(newPoint3D);
            float cosAngle = mLastPointOnSphere.dot(newPoint3D);

            float epsilon = std::numeric_limits<float>::epsilon();
            if (fabs(cosAngle) < 1.0f && axis.length() > epsilon) {
                axis.normalize();
                float angle = 2.0f * acos(cosAngle);

                // Rotate the camera around the center of the scene
                mCamera.rotateAroundLocalPoint(axis, -angle, mCenterScene);
            }
        }
    }
}

// Check the OpenGL errors
void GlutViewer::checkOpenGLErrors() {
    GLenum glError;

    // Get the OpenGL errors
    glError = glGetError();

    // While there are errors
    while (glError != GL_NO_ERROR) {

        // Get the error string
        const GLubyte* stringError = gluErrorString(glError);

        // Display the error
        if (stringError)
            cerr << "OpenGL Error #" << glError << "(" << gluErrorString(glError) <<  endl;
        else
            cerr << "OpenGL Error #" << glError << " (no message available)" <<  endl;

        // Get the next error
        glError = glGetError();
    }
}
