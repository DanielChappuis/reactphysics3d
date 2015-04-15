/********************************************************************************
* ReactPhysics3D physics library, http://www.reactphysics3d.com                 *
* Copyright (c) 2010-2015 Daniel Chappuis                                       *
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

#ifndef SCENE_H
#define SCENE_H

// Libraries
#include "openglframework.h"

// Class Scene
// Abstract class that represents a 3D scene.
class Scene {

    protected:

        // -------------------- Attributes -------------------- //

        /// Scene name
        std::string mName;

        /// Camera
        openglframework::Camera mCamera;

        /// Center of the scene
        openglframework::Vector3 mCenterScene;

        /// Last mouse coordinates on the windows
        double mLastMouseX, mLastMouseY;

        /// Window dimension
        int mWindowWidth, mWindowHeight;

        /// Last point computed on a sphere (for camera rotation)
        openglframework::Vector3 mLastPointOnSphere;

        /// True if the last point computed on a sphere (for camera rotation) is valid
        bool mIsLastPointOnSphereValid;

        // -------------------- Methods -------------------- //

        /// Set the scene position (where the camera needs to look at)
        void setScenePosition(const openglframework::Vector3& position, float sceneRadius);

        /// Set the camera so that we can view the whole scene
        void resetCameraToViewAll();

        /// Map mouse coordinates to coordinates on the sphere
        bool mapMouseCoordinatesToSphere(double xMouse, double yMouse,
                                         openglframework::Vector3& spherePoint) const;


        /// Zoom the camera
        void zoom(float zoomDiff);

        /// Translate the camera
        void translate(int xMouse, int yMouse);

        /// Rotate the camera
        void rotate(int xMouse, int yMouse);

    public:

        // -------------------- Methods -------------------- //

        /// Constructor
        Scene(const std::string& name);

        /// Destructor
        virtual ~Scene();

        /// Reshape the view
        virtual void reshape(int width, int height);

        /// Update the scene (take a simulation step)
        virtual void update()=0;

        /// Render the scene
        virtual void render()=0;

        /// Reset the scene
        virtual void reset()=0;

        /// Called when a keyboard event occurs
        virtual void keyboardEvent(int key, int scancode, int action, int mods);

        /// Called when a mouse button event occurs
        virtual void mouseButtonEvent(int button, int action, int mods,
                                      double mousePosX, double mousePosY);

        /// Called when a mouse motion event occurs
        virtual void mouseMotionEvent(double xMouse, double yMouse, int leftButtonState,
                                      int rightButtonState, int middleButtonState, int altKeyState);

        /// Called when a scrolling event occurs
        virtual void scrollingEvent(float xAxis, float yAxis, float scrollSensitivy);

        /// Set the window dimension
        void setWindowDimension(int width, int height);

        /// Return a reference to the camera
        const openglframework::Camera& getCamera() const;
};

// Called when a keyboard event occurs
inline void Scene::keyboardEvent(int key, int scancode, int action, int mods) {

}

/// Reshape the view
inline void Scene::reshape(int width, int height) {
    mCamera.setDimensions(width, height);
}

// Return a reference to the camera
inline const openglframework::Camera& Scene::getCamera() const  {
    return mCamera;
}

// Set the window dimension
inline void Scene::setWindowDimension(int width, int height) {
    mWindowWidth = width;
    mWindowHeight = height;
}


#endif
