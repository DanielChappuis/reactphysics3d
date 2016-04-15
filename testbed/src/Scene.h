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

#ifndef SCENE_H
#define SCENE_H

// Libraries
#include "openglframework.h"

// Structure ContactPoint
struct ContactPoint {

    public:
        openglframework::Vector3 point;

        /// Constructor
        ContactPoint(const openglframework::Vector3& pointWorld) : point(pointWorld) {

        }
};

/// Structure EngineSettings
/// This structure contains several physics engine parameters
struct EngineSettings {

    public:

       long double elapsedTime;             // Elapsed time (in seconds)
       float timeStep;                      // Current time step (in seconds)
       int nbVelocitySolverIterations;      // Nb of velocity solver iterations
       int nbPositionSolverIterations;      // Nb of position solver iterations
       bool isSleepingEnabled;              // True if sleeping technique is enabled
       float timeBeforeSleep;               // Time of inactivity before a body sleep
       float sleepLinearVelocity;           // Sleep linear velocity
       float sleepAngularVelocity;          // Sleep angular velocity
       bool isGravityEnabled;               // True if gravity is enabled
       openglframework::Vector3 gravity;    // Gravity vector

       /// Constructor
       EngineSettings() : elapsedTime(0.0f), timeStep(0.0f) {

       }
};

// Class Scene
// Abstract class that represents a 3D scene.
class Scene {

    protected:

        // -------------------- Attributes -------------------- //

        /// Scene name
        std::string mName;

        /// Physics engine settings
        EngineSettings mEngineSettings;

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

        /// Interpolation factor for the bodies in the current frame
        float mInterpolationFactor;

        /// Viewport x,y, width and height values
        int mViewportX, mViewportY, mViewportWidth, mViewportHeight;

        /// True if the shadow mapping is enabled
        bool mIsShadowMappingEnabled;

        /// True if contact points are displayed
        bool mIsContactPointsDisplayed;

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
        Scene(const std::string& name, bool isShadowMappingEnabled = false);

        /// Destructor
        virtual ~Scene();

        /// Reshape the view
        virtual void reshape(int width, int height);

        /// Update the physics world (take a simulation step)
        /// Can be called several times per frame
        virtual void updatePhysics()=0;

        /// Update the scene
        virtual void update()=0;

        /// Render the scene
        virtual void render()=0;

        /// Reset the scene
        virtual void reset()=0;

        /// Called when a keyboard event occurs
        virtual bool keyboardEvent(int key, int scancode, int action, int mods);

        /// Called when a mouse button event occurs
        virtual bool mouseButtonEvent(int button, bool down, int mods,
                                      double mousePosX, double mousePosY);

        /// Called when a mouse motion event occurs
        virtual bool mouseMotionEvent(double xMouse, double yMouse, int leftButtonState,
                                      int rightButtonState, int middleButtonState, int altKeyState);

        /// Called when a scrolling event occurs
        virtual bool scrollingEvent(float xAxis, float yAxis, float scrollSensitivy);

        /// Set the window dimension
        void setWindowDimension(int width, int height);

        /// Set the viewport to render the scene
        void setViewport(int x, int y, int width, int height);

        /// Return a reference to the camera
        const openglframework::Camera& getCamera() const;

        /// Get the engine settings
        EngineSettings getEngineSettings() const;

        /// Set the engine settings
        void setEngineSettings(const EngineSettings& settings);

        /// Set the interpolation factor
        void setInterpolationFactor(float interpolationFactor);

        /// Return the name of the scene
        std::string getName() const;

        /// Return true if the shadow mapping is enabled
        bool getIsShadowMappingEnabled() const;

        /// Enabled/Disable the shadow mapping
        void virtual setIsShadowMappingEnabled(bool isShadowMappingEnabled);

        /// Display/Hide the contact points
        void virtual setIsContactPointsDisplayed(bool display);

        /// Return all the contact points of the scene
        std::vector<ContactPoint> virtual getContactPoints() const;
};

// Called when a keyboard event occurs
inline bool Scene::keyboardEvent(int key, int scancode, int action, int mods) {
    return false;
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

// Set the viewport to render the scene
inline void Scene::setViewport(int x, int y, int width, int height) {
    mViewportX = x;
    mViewportY = y;
    mViewportWidth = width;
    mViewportHeight = height;
}

// Get the engine settings
inline EngineSettings Scene::getEngineSettings() const {
    return mEngineSettings;
}

// Set the engine settings
inline void Scene::setEngineSettings(const EngineSettings& settings) {
   mEngineSettings = settings;
}

// Set the interpolation factor
inline void Scene::setInterpolationFactor(float interpolationFactor) {
    mInterpolationFactor = interpolationFactor;
}

// Return the name of the scene
inline std::string Scene::getName() const {
    return mName;
}

// Return true if the shadow mapping is enabled
inline bool Scene::getIsShadowMappingEnabled() const {
    return mIsShadowMappingEnabled;
}

// Enabled/Disable the shadow mapping
inline void Scene::setIsShadowMappingEnabled(bool isShadowMappingEnabled) {
    mIsShadowMappingEnabled = isShadowMappingEnabled;
}

// Display/Hide the contact points
inline void Scene::setIsContactPointsDisplayed(bool display) {
    mIsContactPointsDisplayed = display;
}

// Return all the contact points of the scene
inline std::vector<ContactPoint> Scene::getContactPoints() const {

    // Return an empty list of contact points
    return std::vector<ContactPoint>();
}

#endif
