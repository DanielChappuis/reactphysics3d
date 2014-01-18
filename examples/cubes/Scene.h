/********************************************************************************
* ReactPhysics3D physics library, http://code.google.com/p/reactphysics3d/      *
* Copyright (c) 2010-2013 Daniel Chappuis                                       *
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
#include "reactphysics3d.h"
#include "Box.h"
#include "Viewer.h"

// Constants
const int NB_SPHERES = 20;                                 // Number of boxes in the scene
const openglframework::Vector3 BOX_SIZE(2, 2, 2);          // Box dimensions in meters
const openglframework::Vector3 FLOOR_SIZE(50, 0.5f, 50);   // Floor dimensions in meters
const float BOX_MASS = 1.0f;                               // Box mass in kilograms
const float FLOOR_MASS = 100.0f;                           // Floor mass in kilograms

// Class Scene
class Scene {

    private :

        // -------------------- Attributes -------------------- //

        /// Pointer to the viewer
        Viewer* mViewer;

        /// Light 0
        openglframework::Light mLight0;

        /// Phong shader
        openglframework::Shader mPhongShader;

        /// All the boxes of the scene
        std::vector<Box*> mBoxes;

        /// Box for the floor
        Box* mFloor;

        /// Dynamics world used for the physics simulation
        rp3d::DynamicsWorld* mDynamicsWorld;

        /// True if the physics simulation is running
        bool mIsRunning;

    public:

        // -------------------- Methods -------------------- //

        /// Constructor
        Scene(Viewer* viewer, const std::string& shaderFolderPath);

        /// Destructor
        ~Scene();

        /// Take a step for the simulation
        void simulate();

        /// Stop the simulation
        void stopSimulation();

        /// Start the simulation
        void startSimulation();

        /// Pause or continue simulation
        void pauseContinueSimulation();

        /// Render the scene
        void render();
};

// Stop the simulation
inline void Scene::stopSimulation() {
    mDynamicsWorld->stop();
    mIsRunning = false;
}

// Start the simulation
inline void Scene::startSimulation() {
    mDynamicsWorld->start();
    mIsRunning = true;
}

// Pause or continue simulation
inline void Scene::pauseContinueSimulation() {
    if (mIsRunning) {
        stopSimulation();
    }
    else {
        startSimulation();
    }
}

#endif
