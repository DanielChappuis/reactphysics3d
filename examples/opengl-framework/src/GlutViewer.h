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

#ifndef GLUT_VIEWER_H
#define GLUT_VIEWER_H

// Libraries
#include "Shader.h"
#include "Camera.h"
#include "maths/Vector2.h"
#include <string>
#include <GL/glew.h>
#include "GL/freeglut.h"

namespace openglframework {

// Class Renderer
class GlutViewer {

    private:

        // -------------------- Attributes -------------------- //

        // Camera
        Camera mCamera;

        // Center of the scene
        Vector3 mCenterScene;

        // Last mouse coordinates on the windows
        int mLastMouseX, mLastMouseY;

        // Last point computed on a sphere (for camera rotation)
        Vector3 mLastPointOnSphere;

        // True if the last point computed on a sphere (for camera rotation) is valid
        bool mIsLastPointOnSphereValid;

        // State of the mouse buttons
        bool mIsButtonDown[10];

        // GLUT keyboard modifiers
        int mModifiers;

        // -------------------- Methods -------------------- //

        // Initialize the GLUT library
        bool initGLUT(int argc, char** argv, const std::string& windowsTitle,
                      const Vector2& windowsSize, const Vector2& windowsPosition,
                      bool isMultisamplingActive);

        bool mapMouseCoordinatesToSphere(int xMouse, int yMouse, Vector3& spherePoint) const;

    public:

        // -------------------- Methods -------------------- //

        // Constructor
        GlutViewer();

        // Destructor
        ~GlutViewer();

        // Initialize the viewer
        bool init(int argc, char** argv, const std::string& windowsTitle,
                  const Vector2& windowsSize, const Vector2& windowsPosition,
                  bool isMultisamplingActive = false);

        // Called when the windows is reshaped
        void reshape(int width, int height);

        // Set the scene position (where the camera needs to look at)
        void setScenePosition(const Vector3& position, float sceneRadius);

        // Set the camera so that we can view the whole scene
        void resetCameraToViewAll();

        // Enable/Disable the multi-sampling for anti-aliasing
        void activateMultiSampling(bool isActive) const;

        // Zoom the camera
        void zoom(int xMouse, int yMouse);

        // Translate the camera
        void translate(int xMouse, int yMouse);

        // Rotate the camera
        void rotate(int xMouse, int yMouse);

        // Get the camera
        Camera& getCamera();

        void motion(int x, int y);

        // Called when a GLUT mouse button event occurs
        void mouseButtonEvent(int button, int state, int x, int y);

        // Called when a GLUT mouse motion event occurs
        void mouseMotionEvent(int xMouse, int yMouse);

        void keyboard(int key, int x, int y);
        void special(int key, int x, int y);

        // Check the OpenGL errors
        static void checkOpenGLErrors();
};

// Set the dimension of the camera viewport
inline void GlutViewer::reshape(int width, int height) {
    mCamera.setDimensions(width, height);
    glViewport(0, 0, width, height);
    glutPostRedisplay();
}

// Set the scene position (where the camera needs to look at)
inline void GlutViewer::setScenePosition(const Vector3& position, float sceneRadius) {

    // Set the position and radius of the scene
    mCenterScene = position;
    mCamera.setSceneRadius(sceneRadius);

    // Reset the camera position and zoom in order to view all the scene
    resetCameraToViewAll();
}

// Get the camera
inline Camera& GlutViewer::getCamera() {
   return mCamera;
}

// Enable/Disable the multi-sampling for anti-aliasing
inline void GlutViewer::activateMultiSampling(bool isActive) const {
    (isActive) ? glEnable(GL_MULTISAMPLE) : glDisable(GL_MULTISAMPLE);
}

}

#endif
